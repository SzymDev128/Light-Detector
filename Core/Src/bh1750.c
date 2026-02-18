/**
 ******************************************************************************
 * @file           : bh1750.c
 * @brief          : Sterownik czujnika światła BH1750 i zarządzanie pomiarami
 ******************************************************************************
 */

#include "bh1750.h"
#include "circular_buffer.h"
#include "main.h"

/* Zmienne zewnętrzne */
extern I2C_HandleTypeDef hi2c1;

/* Flaga błędu I2C */
static __IO uint8_t I2C_Error = 0;

/* Zmienne stanu czujnika BH1750 */
static uint8_t bh1750_current_mode = BH1750_CONTINUOUS_HIGH_RES_MODE;
static uint8_t bh1750_addr = BH1750_ADDR_LOW;

/* Stan inicjalizacji BH1750 */
typedef enum {
	BH1750_STATE_IDLE = 0,
	BH1750_STATE_CONFIGURING,
	BH1750_STATE_POWERUP_WAIT,
	BH1750_STATE_MEASURING,    // Czekanie po wysłaniu komendy trybu One Time
	BH1750_STATE_READY,
	BH1750_STATE_BUSY,
	BH1750_STATE_ERROR
} bh1750_state_t;

static bh1750_state_t bh1750_state = BH1750_STATE_IDLE;
static uint32_t bh1750_state_tick = 0;
static uint8_t onetime_meas_phase = 0; // 0=brak, 1=tryb wysłany_czekanie, 2=gotowy_do_odczytu
static uint32_t onetime_phase_tick = 0; // Timestamp dla timeoutu phase

/* Struktura automatycznego pomiaru */
typedef struct {
	uint32_t interval_ms;      // Interwał pomiaru w ms
	uint32_t last_measurement; // Czas ostatniego pomiaru
	uint8_t enabled;           // Flaga włączenia automatycznego odczytu
} measurement_auto_t;

static measurement_auto_t measurement_auto = {
	.interval_ms = 1000,
	.last_measurement = 0,
	.enabled = 0
};

/* Bufor odczytu BH1750 */
static uint8_t bh1750_read_buffer[2] = {0};
static uint8_t config_step = 0;
static uint8_t bh1750_power_on_cmd = 0x01;

/* Bufor pomiarów jest w circular_buffer.c (LightBuffer) */

/**
 * @brief Sprawdza czy tryb to One Time
 * @param mode Komenda trybu BH1750
 * @retval 1 jeśli One Time, 0 jeśli Continuous
 */
static uint8_t is_onetime_mode(uint8_t mode) {
	return (mode == BH1750_ONETIME_HIGH_RES_MODE || mode == BH1750_ONETIME_HIGH_RES_MODE_2 || mode == BH1750_ONETIME_LOW_RES_MODE);
}

/**
 * @brief Automat stanów BH1750 (bez blokowania)
 */
void BH1750_Init_Process(void) {
	if (!measurement_auto.enabled) {
		return;
	}

	if (I2C_Error) {
		I2C_Error = 0;
		bh1750_state = BH1750_STATE_ERROR;
		bh1750_state_tick = HAL_GetTick();
		config_step = 0;
		onetime_meas_phase = 0;
		return;
	}

	switch (bh1750_state) {
		case BH1750_STATE_IDLE: {
			bh1750_state = BH1750_STATE_CONFIGURING;
			config_step = 0;
			break;
		}
		
		case BH1750_STATE_CONFIGURING: {
			switch (config_step) {
				case 0: {
					// Wyślij komendę Power On
						if (HAL_I2C_Master_Transmit_IT(&hi2c1, bh1750_addr << 1, &bh1750_power_on_cmd, 1) == HAL_OK) {
						config_step = 1;
					}
					break;
				}
				case 1: {
					// Tryb Continuous: wyślij komendę trybu w init
					// Tryb One Time: pomiń (będzie wysłane przed każdym pomiarem)
					if (is_onetime_mode(bh1750_current_mode)) {
						config_step = 2; // Przejdź do następnego kroku bez wysyłania komendy trybu
					} else {
						// Continuous mode: send mode now
						if (BH1750_SetMode(bh1750_current_mode) == HAL_OK) {
							config_step = 2;
						}
						else if ((HAL_GetTick() - bh1750_state_tick) > 500) {
							// Timeout
							I2C_Error = 1;
						}
					}
					break;
				}
				case 2: {
					bh1750_state = BH1750_STATE_POWERUP_WAIT;
					bh1750_state_tick = HAL_GetTick();
					config_step = 0;
					break;
				}
				default: {
					// Recover from unexpected config_step value
					config_step = 0;
					bh1750_state = BH1750_STATE_ERROR;
					bh1750_state_tick = HAL_GetTick();
					break;
				}
			}
			break;
		}
		
		case BH1750_STATE_POWERUP_WAIT: {
			// Tryb One Time: pomiń czekanie (będzie czekanie po wysłaniu komendy trybu w pomiarze)
			if (is_onetime_mode(bh1750_current_mode)) {
				bh1750_state = BH1750_STATE_READY;
			} else {
				// Tryb Continuous: czekaj 120ms (H-Res modes) lub 16ms (L-Res mode) po wysłaniu komendy trybu
				uint32_t wait_time = (bh1750_current_mode == BH1750_CONTINUOUS_LOW_RES_MODE) ? 16 : 120;
				if ((HAL_GetTick() - bh1750_state_tick) >= wait_time) {
					bh1750_state = BH1750_STATE_READY;
				}
			}
			break;
		}
	
		case BH1750_STATE_MEASURING: {
			// Czekanie po wysłaniu komendy trybu One Time (120ms dla H-Res/H-Res2, 16ms dla L-Res)
			uint32_t wait_time = (bh1750_current_mode == BH1750_ONETIME_LOW_RES_MODE) ? 16 : 120;
			if ((HAL_GetTick() - bh1750_state_tick) >= wait_time) {
				bh1750_state = BH1750_STATE_READY;
			}
			break;
		}
		
		case BH1750_STATE_READY: {
			// Czujnik gotowy, będzie wyzwolony przez Measurement_AutoRead_Process
			break;
		}
		
		case BH1750_STATE_BUSY: {
			// Timeout - jeśli callback nie przyjdzie w rozsądnym czasie
			if ((HAL_GetTick() - bh1750_state_tick) > 500) {
				bh1750_state = BH1750_STATE_ERROR;
				bh1750_state_tick = HAL_GetTick();
			}
			break;
		}
		
		case BH1750_STATE_ERROR: {
			// Spróbuj ponownie po 2 sekundach
			if ((HAL_GetTick() - bh1750_state_tick) >= 2000) {
				// Reset I2C peripheral jeśli jest w błędnym stanie
				if (hi2c1.State != HAL_I2C_STATE_READY) {
					HAL_I2C_DeInit(&hi2c1);
					HAL_I2C_Init(&hi2c1);
				}
				
				bh1750_state = BH1750_STATE_IDLE;
				config_step = 0;
			}
			break;
		}
		default:
			break;
	}
}

/**
 * @brief Pobranie aktualnego trybu pracy BH1750
 */
uint8_t BH1750_GetCurrentMode(void) {
	return bh1750_current_mode;
}

/**
 * @brief Ustawienie trybu pracy BH1750
 */
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode) {
	static uint8_t mode_cmd;
	HAL_StatusTypeDef status;
	
	mode_cmd = mode;
	status = HAL_I2C_Master_Transmit_IT(&hi2c1, bh1750_addr << 1, &mode_cmd, 1);
	
	if (status == HAL_OK) {
		bh1750_current_mode = mode;
	}
	
	return status;
}

/**
 * @brief Ustawienie interwału pomiaru
 */
void Measurement_SetInterval(uint32_t interval_ms) {
	measurement_auto.interval_ms = interval_ms;
}

/**
 * @brief Pobranie interwału pomiaru
 */
uint32_t Measurement_GetInterval(void) {
	return measurement_auto.interval_ms;
}

/**
 * @brief Włączenie/wyłączenie automatycznego odczytu
 */
void Measurement_EnableAutoRead(uint8_t enable) {
	measurement_auto.enabled = enable;
	if (enable) {
		measurement_auto.last_measurement = HAL_GetTick();
	}
}

/**
 * @brief Przetwarzanie automatycznych pomiarów (główna pętla)
 */
void Measurement_AutoRead_Process(void) {
	if (!measurement_auto.enabled) {
		return;
	}
	
	// Tryb One Time: wyzwolenie pomiaru wysłaniem komendy trybu
	if (is_onetime_mode(bh1750_current_mode)) {
		switch (onetime_meas_phase) {
			case 0: {
				if (bh1750_state == BH1750_STATE_READY) {
					static uint8_t phase0_first_entry = 1;
					if (phase0_first_entry) {
						onetime_phase_tick = HAL_GetTick();
						phase0_first_entry = 0;
					}
					if (HAL_I2C_Master_Transmit_IT(&hi2c1, bh1750_addr << 1, &bh1750_current_mode, 1) == HAL_OK) {
						bh1750_state = BH1750_STATE_MEASURING;
						onetime_meas_phase = 1;
						phase0_first_entry = 1;
					} else if ((HAL_GetTick() - onetime_phase_tick) > 500) {
						LightBuffer_Put(0.0f);
						measurement_auto.enabled = 0;
						onetime_meas_phase = 0;
						phase0_first_entry = 1;
					}
					return;
				}
				break;
			}
			case 1: {
				   // Flaga statyczna: ustawiona na 1 tylko przy pierwszym wejściu do fazy 1
				   static uint8_t phase1_first_entry = 1;
				   // Ustaw znacznik czasu timeoutu tylko raz na wejściu do fazy 1
				   if (phase1_first_entry) {
					   onetime_phase_tick = HAL_GetTick(); // Start odliczania timeoutu oczekiwania na zakończenie pomiaru
					   phase1_first_entry = 0; // Wyłącz, aby nie nadpisywać znacznika w kolejnych przebiegach
				   }
				if (bh1750_state == BH1750_STATE_READY) {
					onetime_meas_phase = 2;
					phase1_first_entry = 1;
				} else if ((HAL_GetTick() - onetime_phase_tick) > 500) {
					LightBuffer_Put(0.0f);
					measurement_auto.enabled = 0;
					onetime_meas_phase = 0;
					phase1_first_entry = 1;
				}
				return;
			}
			case 2: {
				static uint8_t phase2_first_entry = 1;
				if (phase2_first_entry) {
					onetime_phase_tick = HAL_GetTick();
					phase2_first_entry = 0;
				}
				if (bh1750_state == BH1750_STATE_READY) {
					bh1750_state = BH1750_STATE_BUSY;
					HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, (bh1750_addr << 1) | 0x01, bh1750_read_buffer, 2);
					if (status != HAL_OK) {
						bh1750_state = BH1750_STATE_READY;
						LightBuffer_Put(0.0f);
						measurement_auto.enabled = 0;
						onetime_meas_phase = 0;
					}
					phase2_first_entry = 1;
				} else if ((HAL_GetTick() - onetime_phase_tick) > 500) {
					LightBuffer_Put(0.0f);
					measurement_auto.enabled = 0;
					onetime_meas_phase = 0;
					bh1750_state = BH1750_STATE_READY;
					phase2_first_entry = 1;
				}
				return;
			}
			default:
				break;
		}
	}
	
	// Tryb Continuous: obsługa timeoutu i cyklicznego odczytu
	if (bh1750_state == BH1750_STATE_BUSY) {
		// Obsługa timeoutu na odczyt
		if ((HAL_GetTick() - bh1750_state_tick) > 500) {
			bh1750_state = BH1750_STATE_READY;
			LightBuffer_Put(0.0f);
		}
		return;
	}
	
	if (bh1750_state != BH1750_STATE_READY) {
		return;
	}

	uint32_t current_time = HAL_GetTick();
	if ((current_time - measurement_auto.last_measurement) >= measurement_auto.interval_ms) {
		measurement_auto.last_measurement = current_time;

		// Wysłanie komendy odczytu
		bh1750_state = BH1750_STATE_BUSY;
		bh1750_state_tick = HAL_GetTick();  // Timestamp dla timeout
		HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, (bh1750_addr << 1) | 0x01, bh1750_read_buffer, 2);

		if (status != HAL_OK) {
			bh1750_state = BH1750_STATE_READY;
			LightBuffer_Put(0.0f);
		}
	}
}

/**
 * @brief Callback ukończenia odczytu I2C
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c != &hi2c1) {
		return;
	}

	// Light measurement reading
	if (bh1750_state == BH1750_STATE_BUSY) {
		uint16_t raw_value = (bh1750_read_buffer[0] << 8) | bh1750_read_buffer[1];
		
		// Select divider based on measurement mode
		float lux_divider;
		switch (bh1750_current_mode) {
			case BH1750_CONTINUOUS_HIGH_RES_MODE_2:  // Continuous H-Res Mode2
			case BH1750_ONETIME_HIGH_RES_MODE_2:     // One Time H-Res Mode2
				lux_divider = 2.4f;  // 0.5 lx resolution
				break;
			
			case BH1750_CONTINUOUS_LOW_RES_MODE:      // Continuous L-Res Mode
			case BH1750_ONETIME_LOW_RES_MODE:         // One Time L-Res Mode
				lux_divider = 0.5f;  // 4 lx resolution
				break;
			
			case BH1750_CONTINUOUS_HIGH_RES_MODE:     // Continuous H-Res Mode
			case BH1750_ONETIME_HIGH_RES_MODE:        // One Time H-Res Mode
			default:
				lux_divider = 1.2f;  // 1 lx resolution
				break;
		}
		
		// Calculate lux value and add to buffer (both modes)
		float lux = raw_value / lux_divider;
		LightBuffer_Put(lux);
		
		// For One Time mode: disable automatic reading after single measurement
		if (is_onetime_mode(bh1750_current_mode)) {
			measurement_auto.enabled = 0;
			onetime_meas_phase = 0;
		}
		
		bh1750_state = BH1750_STATE_READY;
	}
}

/**
 * @brief Callback błędu I2C
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		I2C_Error = 1;
	}
}
