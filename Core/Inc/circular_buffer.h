#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "bh1750.h"

/* Measurement Buffer Size */
#define MEASUREMENT_BUFFER_SIZE 1000

/* USART Buffer Sizes */
#define USART_TXBUF_LEN 1512
#define USART_RXBUF_LEN 512

/* Light Buffer Size */
#define LIGHT_BUFFER_SIZE MEASUREMENT_BUFFER_SIZE

/* USART Buffers */
extern uint8_t USART_TxBuf[USART_TXBUF_LEN];
extern uint8_t USART_RxBuf[USART_RXBUF_LEN];
extern __IO int USART_TX_Empty;
extern __IO int USART_TX_Busy;
extern __IO int USART_RX_Empty;
extern __IO int USART_RX_Busy;
extern __IO uint8_t USART_RxBufOverflow;

/* USART Functions */
uint8_t USART_kbhit(void);
int16_t USART_getchar(void);
void USART_fsend(char *format, ...);

/* Light Buffer Functions */
uint8_t LightBuffer_Put(float lux);
measurement_entry_t* LightBuffer_GetLatest(void);
uint32_t LightBuffer_GetCount(void);
measurement_entry_t* LightBuffer_GetByOffset(uint16_t offset);
measurement_entry_t* LightBuffer_GetByIndexOldest(uint16_t index);

/* Light Buffer Test Data */
void Measurement_FillTestData(void);

#ifdef __cplusplus
}
#endif

#endif /* __CIRCULAR_BUFFER_H */
