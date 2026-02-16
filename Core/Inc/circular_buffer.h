/**
 ******************************************************************************
 * @file           : circular_buffer.h
 * @brief          : Circular buffer implementation for USART and I2C
 ******************************************************************************
 */

#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* USART Buffer Sizes */
#define USART_TXBUF_LEN 1512
#define USART_RXBUF_LEN 512

/* I2C Buffer Sizes */
#define I2C_TXBUF_LEN 512
#define I2C_RXBUF_LEN 512

/* USART Buffers */
extern uint8_t USART_TxBuf[USART_TXBUF_LEN];
extern uint8_t USART_RxBuf[USART_RXBUF_LEN];
extern __IO int USART_TX_Empty;
extern __IO int USART_TX_Busy;
extern __IO int USART_RX_Empty;
extern __IO int USART_RX_Busy;
extern __IO uint8_t USART_RxBufOverflow;

/* I2C Buffers */
extern uint8_t I2C_TxBuf[I2C_TXBUF_LEN];
extern uint8_t I2C_RxBuf[I2C_RXBUF_LEN];
extern __IO uint8_t I2C_Error;

/* USART Functions */
uint8_t USART_kbhit(void);
int16_t USART_getchar(void);
void USART_fsend(char *format, ...);

/* HAL Callbacks */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __CIRCULAR_BUFFER_H */
