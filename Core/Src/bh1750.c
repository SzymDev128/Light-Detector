/**
 ******************************************************************************
 * @file           : bh1750.c
 * @brief          : BH1750 light sensor driver and measurement management
 ******************************************************************************
 */

#include "bh1750.h"
#include "circular_buffer.h"
#include "main.h"

/* External Variables */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;

/* BH1750 State Variables */
uint8_t bh1750_current_mode = BH1750_CONTINUOUS_HIGH_RES_MODE;
static uint8_t bh1750_initialized = 0;
static uint8_t bh1750_addr = BH1750_ADDR_LOW;

/* BH1750 Initialization State */
typedef enum {
	BH1750_INIT_IDLE = 0,
	BH1750_INIT_PWRON,
	BH1750_INIT_RESET,
	BH1750_INIT_MODE,
	BH1750_INIT_DONE
} bh1750_init_state_t;

static bh1750_init_state_t bh1750_init_state = BH1750_INIT_IDLE;

/* I2C Operation Structure */
typedef struct {
	uint8_t address;
	uint8_t *data;
	uint16_t len;
	uint8_t operation; // 0 = TX, 1 = RX
	uint8_t pending;
} i2c_operation_t;

static i2c_operation_t i2c_op = {0};

/* BH1750 Timing Structure */
typedef struct {
	uint32_t start_time;
	uint32_t wait_time; // in ms
	uint8_t active;
} bh1750_timing_t;

static bh1750_timing_t bh1750_timing = {0};

/* Automatic Measurement Structure */
typedef struct {
	uint32_t interval_ms;      // Measurement interval in ms
	uint32_t last_measurement; // Last measurement time
	uint8_t enabled;           // Auto-read enabled flag
} measurement_auto_t;

static measurement_auto_t measurement_auto = {
	.interval_ms = 1000,
	.last_measurement = 0,
	.enabled = 0
};

/* Application Timer (1ms based on TIM3) */
static __IO uint32_t app_tick = 0;

/* BH1750 Read Buffer */
static uint8_t bh1750_read_buffer[2] = {0};
static float bh1750_last_lux = 0.0f;
static uint8_t bh1750_read_ready = 0;

/* Measurement Buffer */
static measurement_entry_t measurement_buffer[MEASUREMENT_BUFFER_SIZE];
static uint16_t measurement_write_index = 0;
static uint16_t measurement_count = 0;

/**
 * @brief Get application tick count
 * @retval Current tick value in milliseconds
 */
uint32_t App_GetTick(void) {
	return app_tick;
}

/**
 * @brief I2C transmit with interrupt
 */
HAL_StatusTypeDef I2C_Transmit_IT(uint8_t address, uint8_t *data, uint16_t len) {
	if (i2c_op.pending) {
		return HAL_BUSY;
	}
	
	if (len > I2C_TXBUF_LEN) {
		return HAL_ERROR;
	}
	
	HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);
	if (state != HAL_I2C_STATE_READY) {
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		return HAL_ERROR;
	}
	
	for (uint16_t i = 0; i < len; i++) {
		I2C_TxBuf[i] = data[i];
	}
	
	i2c_op.address = address;
	i2c_op.data = I2C_TxBuf;
	i2c_op.len = len;
	i2c_op.operation = 0;
	i2c_op.pending = 1;
	
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&hi2c1, address << 1, I2C_TxBuf, len);
	
	if (status != HAL_OK) {
		i2c_op.pending = 0;
	}
	
	return status;
}

/**
 * @brief I2C receive with interrupt
 */
HAL_StatusTypeDef I2C_Receive_IT(uint8_t address, uint8_t *data, uint16_t len) {
	if (i2c_op.pending) {
		return HAL_BUSY;
	}
	
	if (len > I2C_RXBUF_LEN) {
		return HAL_ERROR;
	}
	
	i2c_op.address = address;
	i2c_op.data = data;
	i2c_op.len = len;
	i2c_op.operation = 1;
	i2c_op.pending = 1;
	
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, (address << 1) | 0x01, I2C_RxBuf, len);
	
	if (status != HAL_OK) {
		i2c_op.pending = 0;
	}
	
	return status;
}

/**
 * @brief Check if BH1750 timing has elapsed
 */
uint8_t BH1750_IsTimingReady(void) {
	if (!bh1750_timing.active) {
		return 1;
	}
	
	uint32_t current_time = App_GetTick();
	if ((current_time - bh1750_timing.start_time) >= bh1750_timing.wait_time) {
		bh1750_timing.active = 0;
		return 1;
	}
	
	return 0;
}

/**
 * @brief Start BH1750 timing tracker
 */
void BH1750_StartTiming(uint32_t wait_time_ms) {
	bh1750_timing.start_time = App_GetTick();
	bh1750_timing.wait_time = wait_time_ms;
	bh1750_timing.active = 1;
}

/**
 * @brief BH1750 non-blocking initialization process
 */
void BH1750_Init_Process(void) {
	static uint32_t last_retry_time = 0;
	
	if (bh1750_initialized) {
		return;
	}

	if (!measurement_auto.enabled) {
		return;
	}

	if (I2C_Error) {
		I2C_Error = 0;
		bh1750_initialized = 0;
		bh1750_init_state = BH1750_INIT_IDLE;
		return;
	}

	if (i2c_op.pending) {
		return;
	}
	
	if (bh1750_init_state == BH1750_INIT_IDLE) {
		uint32_t now = App_GetTick();
		if (now - last_retry_time < 1000) {
			return;
		}
		last_retry_time = now;
	}

	switch (bh1750_init_state) {
		case BH1750_INIT_IDLE: {
			uint8_t cmd = 0x01; // POWER ON
			HAL_StatusTypeDef status = I2C_Transmit_IT(bh1750_addr, &cmd, 1);
			if (status == HAL_OK) {
				bh1750_init_state = BH1750_INIT_PWRON;
			}
			break;
		}
		case BH1750_INIT_PWRON: {
			uint8_t cmd = 0x07; // RESET
			if (I2C_Transmit_IT(bh1750_addr, &cmd, 1) == HAL_OK) {
				bh1750_init_state = BH1750_INIT_RESET;
			}
			break;
		}
		case BH1750_INIT_RESET: {
			if (BH1750_SetMode(bh1750_current_mode) == HAL_OK) {
				bh1750_init_state = BH1750_INIT_MODE;
			}
			break;
		}
		case BH1750_INIT_MODE:
			if (!i2c_op.pending) {
				bh1750_initialized = 1;
				bh1750_init_state = BH1750_INIT_DONE;
			}
			break;
		default:
			break;
	}
}

/**
 * @brief Set BH1750 operation mode
 */
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode) {
	HAL_StatusTypeDef status;
	uint8_t addr = bh1750_addr;
	
	status = I2C_Transmit_IT(addr, &mode, 1);
	
	if (status == HAL_OK) {
		bh1750_current_mode = mode;
		
		if (mode == BH1750_CONTINUOUS_LOW_RES_MODE || mode == BH1750_ONETIME_LOW_RES_MODE) {
			BH1750_StartTiming(16);
		} else {
			BH1750_StartTiming(120);
		}
	}
	
	return status;
}

/**
 * @brief Add measurement entry to buffer
 */
void Measurement_AddEntry(float lux) {
	measurement_buffer[measurement_write_index].lux = lux;
	measurement_buffer[measurement_write_index].timestamp = App_GetTick();
	
	measurement_write_index++;
	if (measurement_write_index >= MEASUREMENT_BUFFER_SIZE) {
		measurement_write_index = 0;
	}
	
	if (measurement_count < MEASUREMENT_BUFFER_SIZE) {
		measurement_count++;
	}
}

/**
 * @brief Fill buffer with test data
 */
void Measurement_FillTestData(void) {
	for (uint16_t i = 1; i <= 2500; i++) {
		float lux_value = 100.0f + (float)i;
		Measurement_AddEntry(lux_value);
	}
}

/**
 * @brief Get measurement count
 */
uint16_t Measurement_GetCount(void) {
	return measurement_count;
}

/**
 * @brief Get measurement entry by index
 */
measurement_entry_t* Measurement_GetEntry(uint16_t index) {
	if (index >= measurement_count) {
		return NULL;
	}
	
	uint16_t real_index;
	if (measurement_count < MEASUREMENT_BUFFER_SIZE) {
		real_index = index;
	} else {
		real_index = (measurement_write_index + index) % MEASUREMENT_BUFFER_SIZE;
	}
	
	return &measurement_buffer[real_index];
}

/**
 * @brief Read light value from BH1750
 */
HAL_StatusTypeDef BH1750_ReadLight(float *lux) {
	uint8_t addr = bh1750_addr;
	
	if (i2c_op.pending) {
		return HAL_BUSY;
	}
	
	if (bh1750_read_ready) {
		*lux = bh1750_last_lux;
		bh1750_read_ready = 0;
		return HAL_OK;
	}
	
	HAL_StatusTypeDef status = I2C_Receive_IT(addr, bh1750_read_buffer, 2);
	
	return status;
}

/**
 * @brief Set measurement interval
 */
void Measurement_SetInterval(uint32_t interval_ms) {
	measurement_auto.interval_ms = interval_ms;
}

/**
 * @brief Get measurement interval
 */
uint32_t Measurement_GetInterval(void) {
	return measurement_auto.interval_ms;
}

/**
 * @brief Enable/disable automatic measurement
 */
void Measurement_EnableAutoRead(uint8_t enable) {
	measurement_auto.enabled = enable;
	if (enable) {
		measurement_auto.last_measurement = App_GetTick();
	}
}

/**
 * @brief Process automatic measurements (call in main loop)
 */
void Measurement_AutoRead_Process(void) {
	if (!measurement_auto.enabled) {
		return;
	}
	if (!bh1750_initialized) {
		return;
	}
	
	if (bh1750_read_ready) {
		Measurement_AddEntry(bh1750_last_lux);
		bh1750_read_ready = 0;
	}

	uint32_t current_time = App_GetTick();
	if ((current_time - measurement_auto.last_measurement) >= measurement_auto.interval_ms) {
		measurement_auto.last_measurement = current_time;
		
		if (!i2c_op.pending) {
			bh1750_read_ready = 0;
			HAL_StatusTypeDef status = BH1750_ReadLight(&bh1750_last_lux);
			
			if (status != HAL_OK) {
				Measurement_AddEntry(0.0f);
			}
		}
	}
}

/**
 * @brief TIM3 period elapsed callback (1ms tick)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		app_tick++;
	}
}

/**
 * @brief I2C master transmit complete callback
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		i2c_op.pending = 0;
	}
}

/**
 * @brief I2C master receive complete callback
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		if (i2c_op.operation == 1 && i2c_op.data != NULL) {
			for (uint16_t i = 0; i < i2c_op.len; i++) {
				i2c_op.data[i] = I2C_RxBuf[i];
			}
			
			if (i2c_op.len == 2 && i2c_op.data == bh1750_read_buffer) {
				uint16_t raw_value = (bh1750_read_buffer[0] << 8) | bh1750_read_buffer[1];
				float lux_divider = 1.2f;
				if (bh1750_current_mode == 0x11) {
					lux_divider = 2.4f;
				} else if (bh1750_current_mode == 0x13) {
					lux_divider = 0.5f;
				}
				bh1750_last_lux = raw_value / lux_divider;
				bh1750_read_ready = 1;
			}
		}
		i2c_op.pending = 0;
	}
}

/**
 * @brief I2C error callback
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		I2C_Error = 1;
		i2c_op.pending = 0;
	}
}
