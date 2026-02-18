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

/* Measurement Entry Structure */
typedef struct measurement_entry_t {
	float lux;           // Light intensity in lux
} measurement_entry_t;

/* BH1750 Functions */
void BH1750_Init_Process(void);
uint8_t BH1750_GetCurrentMode(void);
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode);

/* Measurement Functions */
void Measurement_SetInterval(uint32_t interval_ms);
uint32_t Measurement_GetInterval(void);
void Measurement_EnableAutoRead(uint8_t enable);
void Measurement_AutoRead_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __BH1750_H */
