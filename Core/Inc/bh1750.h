/**
 ******************************************************************************
 * @file           : bh1750.h
 * @brief          : BH1750 light sensor driver and measurement management
 ******************************************************************************
 */

#ifndef __BH1750_H
#define __BH1750_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* BH1750 I2C Addresses */
#define BH1750_ADDR_LOW    0x23  // ADDR pin to GND
#define BH1750_ADDR_HIGH   0x5C  // ADDR pin to VCC

/* BH1750 Operation Modes */
#define BH1750_CONTINUOUS_HIGH_RES_MODE    0x10  // Continuous high resolution (1lx, 120ms)
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2  0x11  // Continuous high resolution 2 (0.5lx, 120ms)
#define BH1750_CONTINUOUS_LOW_RES_MODE     0x13  // Continuous low resolution (4lx, 16ms)
#define BH1750_ONETIME_HIGH_RES_MODE       0x20  // One-time high resolution (1lx, 120ms)
#define BH1750_ONETIME_HIGH_RES_MODE_2     0x21  // One-time high resolution 2 (0.5lx, 120ms)
#define BH1750_ONETIME_LOW_RES_MODE        0x23  // One-time low resolution (4lx, 16ms)

/* Measurement Buffer Size */
#define MEASUREMENT_BUFFER_SIZE 1000

/* Measurement Entry Structure */
typedef struct measurement_entry_t {
	float lux;           // Light intensity in lux
	uint32_t timestamp;  // Measurement timestamp (App_GetTick())
} measurement_entry_t;

/* BH1750 Functions */
void BH1750_Init_Process(void);
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode);
HAL_StatusTypeDef BH1750_ReadLight(float *lux);
uint8_t BH1750_IsTimingReady(void);
void BH1750_StartTiming(uint32_t wait_time_ms);

/* Measurement Functions */
void Measurement_AddEntry(float lux);
uint16_t Measurement_GetCount(void);
measurement_entry_t* Measurement_GetEntry(uint16_t index);
void Measurement_SetInterval(uint32_t interval_ms);
uint32_t Measurement_GetInterval(void);
void Measurement_EnableAutoRead(uint8_t enable);
void Measurement_AutoRead_Process(void);
void Measurement_FillTestData(void);

/* I2C IT Functions */
HAL_StatusTypeDef I2C_Transmit_IT(uint8_t address, uint8_t *data, uint16_t len);
HAL_StatusTypeDef I2C_Receive_IT(uint8_t address, uint8_t *data, uint16_t len);

/* Timer Functions */
uint32_t App_GetTick(void);

/* HAL Callbacks */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

/* External Access to BH1750 State */
extern uint8_t bh1750_current_mode;

#ifdef __cplusplus
}
#endif

#endif /* __BH1750_H */
