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

/* I2C Buffers */
uint8_t I2C_TxBuf[I2C_TXBUF_LEN];
uint8_t I2C_RxBuf[I2C_RXBUF_LEN];

__IO uint8_t I2C_Error = 0;

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
	char tmp_rs[300];
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

/**
 * @brief UART transmit complete callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (USART_TX_Empty != USART_TX_Busy) {
			uint8_t tmp = USART_TxBuf[USART_TX_Busy];
			USART_TX_Busy++;
			if (USART_TX_Busy >= USART_TXBUF_LEN)
				USART_TX_Busy = 0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}

/**
 * @brief UART receive complete callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		int next_head = (USART_RX_Empty + 1) % USART_RXBUF_LEN;
		if (next_head == USART_RX_Busy) {
			USART_RxBufOverflow = 1;
		} else {
			USART_RxBuf[USART_RX_Empty] = rx_byte;
			USART_RX_Empty = next_head;
		}
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
}
