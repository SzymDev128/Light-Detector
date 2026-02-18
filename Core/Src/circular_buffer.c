/**
 ******************************************************************************
 * @file           : circular_buffer.c
 * @brief          : Circular buffer implementation for USART and I2C
 ******************************************************************************
 */

#include "circular_buffer.h"
#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* External Variables */
extern UART_HandleTypeDef huart2;
extern uint8_t rx_byte;

/* USART Buffers */
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];

__IO int USART_TX_Empty = 0; // Nadawanie head
__IO int USART_TX_Busy = 0;  // Nadawanie tail
__IO int USART_RX_Empty = 0; // Odbieranie head
__IO int USART_RX_Busy = 0;  // Odbieranie tail

__IO uint8_t USART_RxBufOverflow = 0;

/* Light Buffer */
static measurement_entry_t LightBuffer[LIGHT_BUFFER_SIZE];
static volatile uint32_t LightBuffer_WritePos = 0;
static volatile uint32_t LightBuffer_Count = 0;
static volatile uint8_t LightBuffer_DataAvailable = 0;

/**
 * @brief Check if USART receive buffer has data
 * @retval 1 if data available, 0 if empty
 */
uint8_t USART_kbhit(void) {
	if (USART_RX_Empty == USART_RX_Busy) {
		return 0; // Buffer is empty
	} else {
		return 1; // Buffer has data
	}
}

/**
 * @brief Get one character from USART receive buffer
 * @retval Character from buffer or -1 if empty
 */
int16_t USART_getchar(void) {
	int16_t tmp;
	if (USART_RX_Empty != USART_RX_Busy) {
		tmp = USART_RxBuf[USART_RX_Busy];
		USART_RX_Busy++;
		if (USART_RX_Busy >= USART_RXBUF_LEN)
			USART_RX_Busy = 0;
		return tmp;
	} else
		return -1;
}

/**
 * @brief Send formatted string via USART
 * @param format: Format string (like printf)
 */
void USART_fsend(char *format, ...) {
	char tmp_rs[560];
	int i;
	__IO int idx;
	va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp_rs, format, arglist);
	va_end(arglist);
	idx = USART_TX_Empty;
	for (i = 0; i < strlen(tmp_rs); i++) {
		USART_TxBuf[idx] = tmp_rs[i];
		idx++;
		if (idx >= USART_TXBUF_LEN)
			idx = 0;
	}
	__disable_irq();
	if ((USART_TX_Empty == USART_TX_Busy)
			&& (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
		USART_TX_Empty = idx;
		uint8_t tmp = USART_TxBuf[USART_TX_Busy];
		USART_TX_Busy++;
		if (USART_TX_Busy >= USART_TXBUF_LEN)
			USART_TX_Busy = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	} else {
		USART_TX_Empty = idx;
	}
	__enable_irq();
}

uint8_t LightBuffer_Put(float lux) {
	__disable_irq();

	LightBuffer[LightBuffer_WritePos].lux = lux;
	LightBuffer_WritePos = (LightBuffer_WritePos + 1) % LIGHT_BUFFER_SIZE;
	if (LightBuffer_Count < LIGHT_BUFFER_SIZE) {
		LightBuffer_Count++;
	}
	LightBuffer_DataAvailable = 1;
	__enable_irq();
	return 1;
}

measurement_entry_t* LightBuffer_GetLatest(void) {
	if (!LightBuffer_DataAvailable) {
		return NULL;
	}
	uint32_t latest_index = (LightBuffer_WritePos == 0)
			? (LIGHT_BUFFER_SIZE - 1)
			: (LightBuffer_WritePos - 1);
	return &LightBuffer[latest_index];
}

uint32_t LightBuffer_GetCount(void) {
	return LightBuffer_Count;
}

measurement_entry_t* LightBuffer_GetByOffset(uint16_t offset) {
	if (!LightBuffer_DataAvailable) {
		return NULL;
	}
	uint32_t count = LightBuffer_Count;
	if (count == 0 || offset >= count) {
		return NULL;
	}
	uint32_t actual_index = (LightBuffer_WritePos - 1 - offset + LIGHT_BUFFER_SIZE)
			% LIGHT_BUFFER_SIZE;
	return &LightBuffer[actual_index];
}

measurement_entry_t* LightBuffer_GetByIndexOldest(uint16_t index) {
	uint32_t count = LightBuffer_Count;
	if (count == 0 || index >= count) {
		return NULL;
	}
	uint32_t offset_from_latest = (count - 1U) - index;
	return LightBuffer_GetByOffset((uint16_t)offset_from_latest);
}

void Measurement_FillTestData(void) {
	for (uint16_t i = 1; i <= 2500; i++) {
		float lux_value = 100.0f + (float)i;
		LightBuffer_Put(lux_value);
	}
}
