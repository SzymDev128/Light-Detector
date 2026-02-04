/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Struktura dla przechowywania pomiarów BH1750
typedef struct measurement_entry_t {
	float lux;           // Wartość natężenia światła w luksach
	uint32_t timestamp;  // Timestamp pomiaru (App_GetTick())
} measurement_entry_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t rx_byte;

// BH1750 definitions
#define BH1750_ADDR_LOW    0x23  // ADDR pin do GND
#define BH1750_ADDR_HIGH   0x5C  // ADDR pin do VCC

#define BH1750_CONTINUOUS_HIGH_RES_MODE    0x10  // Ciągły wysokiej rozdzielczości (1lx, 120ms)
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2   0x11  // Ciągły wysokiej rozdzielczości 2 (0.5lx, 120ms)
#define BH1750_CONTINUOUS_LOW_RES_MODE      0x13  // Ciągły niskiej rozdzielczości (4lx, 16ms)
#define BH1750_ONETIME_HIGH_RES_MODE        0x20  // Ręczny wysokiej rozdzielczości (1lx, 120ms)
#define BH1750_ONETIME_HIGH_RES_MODE_2      0x21  // Ręczny wysokiej rozdzielczości 2 (0.5lx, 120ms)
#define BH1750_ONETIME_LOW_RES_MODE         0x23  // Ręczny niskiej rozdzielczości (4lx, 16ms)

// Aktualny tryb BH1750 (domyślnie ciągły wysokiej rozdzielczości)
static uint8_t bh1750_current_mode = BH1750_CONTINUOUS_HIGH_RES_MODE;
static uint8_t bh1750_initialized = 0;
static uint8_t bh1750_addr = BH1750_ADDR_LOW;

typedef enum {
	BH1750_INIT_IDLE = 0,
	BH1750_INIT_PWRON,
	BH1750_INIT_RESET,
	BH1750_INIT_MODE,
	BH1750_INIT_DONE
} bh1750_init_state_t;

static bh1750_init_state_t bh1750_init_state = BH1750_INIT_IDLE;

/* FRAME PARSING */
typedef enum {
    ST_IDLE = 0,
    ST_COLLECT
} frame_state_t;

frame_state_t st = ST_IDLE;

char frame[300];
uint16_t pos = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t is_digits_only(const char *str, uint16_t len);
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data);
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode);
HAL_StatusTypeDef I2C_Transmit_IT(uint8_t address, uint8_t *data, uint16_t len);
HAL_StatusTypeDef I2C_Receive_IT(uint8_t address, uint8_t *data, uint16_t len);
uint8_t BH1750_IsTimingReady(void);
void BH1750_StartTiming(uint32_t wait_time_ms);
void BH1750_Init_Process(void);
void I2C_BusRecovery_Process(void);
void Measurement_AddEntry(float lux);
uint16_t Measurement_GetCount(void);
measurement_entry_t* Measurement_GetEntry(uint16_t index);
HAL_StatusTypeDef BH1750_ReadLight(float *lux);
void Measurement_AutoRead_Process(void);
void Measurement_SetInterval(uint32_t interval_ms);
void Measurement_EnableAutoRead(uint8_t enable);
uint8_t I2C_Scan_FirstAddress(uint8_t *found_addr);
void Measurement_FillTestData(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define USART_TXBUF_LEN 1512
#define USART_RXBUF_LEN 512
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];

__IO int USART_TX_Empty = 0; // Nadawanie head
__IO int USART_TX_Busy = 0;  // Nadawanie tail
__IO int USART_RX_Empty = 0; // Odbieranie head
__IO int USART_RX_Busy = 0;  // Odbieranie tail

// Sygnalizacja błędów
__IO uint8_t USART_RxBufOverflow = 0;

// I2C Circular Buffers
#define I2C_TXBUF_LEN 256
#define I2C_RXBUF_LEN 256
uint8_t I2C_TxBuf[I2C_TXBUF_LEN];
uint8_t I2C_RxBuf[I2C_RXBUF_LEN];

// Sygnalizacja błędów I2C
__IO uint8_t I2C_Error = 0;
__IO uint8_t I2C_BusReset_Pending = 0;

// Struktura dla operacji I2C
typedef struct {
	uint8_t address;
	uint8_t *data;
	uint16_t len;
	uint8_t operation; // 0 = TX, 1 = RX
	uint8_t pending;
} i2c_operation_t;

static i2c_operation_t i2c_op = {0};

// Struktura dla śledzenia czasu oczekiwania BH1750
typedef struct {
	uint32_t start_time;
	uint32_t wait_time; // w ms
	uint8_t active;
} bh1750_timing_t;

static bh1750_timing_t bh1750_timing = {0};

// Struktura dla automatycznego odczytu pomiarów
typedef struct {
	uint32_t interval_ms;      // Interwał pomiarowy w ms
	uint32_t last_measurement;  // Czas ostatniego pomiaru
	uint8_t enabled;            // Czy automatyczny odczyt jest włączony
} measurement_auto_t;

// Struktura dla nieblokującej operacji I2C Bus Recovery
typedef enum {
	BUS_RECOVERY_IDLE = 0,
	BUS_RECOVERY_DISABLE_I2C,
	BUS_RECOVERY_CONFIG_GPIO,
	BUS_RECOVERY_PULSE_LOW,
	BUS_RECOVERY_PULSE_HIGH,
	BUS_RECOVERY_STOP_SDA_LOW,
	BUS_RECOVERY_STOP_SCL_HIGH,
	BUS_RECOVERY_STOP_SDA_HIGH,
	BUS_RECOVERY_RESTORE_I2C
} bus_recovery_state_t;

typedef struct {
	bus_recovery_state_t state;
	uint8_t pulse_count;
	uint32_t last_time;
} bus_recovery_t;

static measurement_auto_t measurement_auto = {
	.interval_ms = 1000,        // Domyślnie 1 sekunda
	.last_measurement = 0,
	.enabled = 0                // Wyłączone domyślnie
};

static bus_recovery_t bus_recovery = {
	.state = BUS_RECOVERY_IDLE,
	.pulse_count = 0,
	.last_time = 0
};

// Timer aplikacyjny oparty o TIM2 (1ms)
static __IO uint32_t app_tick = 0;

static uint32_t App_GetTick(void) {
	return app_tick;
}

// Bufor do odczytu BH1750
static uint8_t bh1750_read_buffer[2] = {0};
static float bh1750_last_lux = 0.0f;
static uint8_t bh1750_read_ready = 0;

// Bufor cykliczny na 1000 wpisów pomiarów
#define MEASUREMENT_BUFFER_SIZE 1000
static measurement_entry_t measurement_buffer[MEASUREMENT_BUFFER_SIZE];
static uint16_t measurement_write_index = 0;  // Indeks do zapisu (head)
static uint16_t measurement_count = 0;       // Liczba zapisanych pomiarów (max 1000)

uint8_t USART_kbhit() {
	if (USART_RX_Empty == USART_RX_Busy) {
		return 0; //Buffer is empty
	} else {
		return 1; //Buffer has data
	}
} //USART_kbhit

int16_t USART_getchar() {
	int16_t tmp;
	if (USART_RX_Empty != USART_RX_Busy) {
		tmp = USART_RxBuf[USART_RX_Busy];
		USART_RX_Busy++;
		if (USART_RX_Busy >= USART_RXBUF_LEN)
			USART_RX_Busy = 0;
		return tmp;
	} else
		return -1;
} //USART_getchar

void USART_fsend(char *format, ...) {
	char tmp_rs[300];  // Zwiększone dla maksymalnej ramki (271) + margines
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
			&& (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE) == SET)) { //sprawdzic dodatkowo zajetosc bufora nadajnika
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
} //fsend

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
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

// Funkcje obsługi buforów I2C
HAL_StatusTypeDef I2C_Transmit_IT(uint8_t address, uint8_t *data, uint16_t len) {
	if (i2c_op.pending) {
		return HAL_BUSY; // Operacja w toku
	}
	
	if (len > I2C_TXBUF_LEN) {
		return HAL_ERROR; // Zbyt duże dane
	}
	
	// Sprawdź stan HAL I2C
	HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);
	if (state != HAL_I2C_STATE_READY) {
		// Próba resetu
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		return HAL_ERROR;
	}
	
	
	// Kopiowanie danych do bufora
	for (uint16_t i = 0; i < len; i++) {
		I2C_TxBuf[i] = data[i];
	}
	
	i2c_op.address = address;
	i2c_op.data = I2C_TxBuf;
	i2c_op.len = len;
	i2c_op.operation = 0; // TX
	i2c_op.pending = 1;
	
	// Rozpoczęcie transmisji przez przerwania
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&hi2c1, address << 1, I2C_TxBuf, len);
	
	if (status != HAL_OK) {
		i2c_op.pending = 0;
	}
	
	return status;
}

HAL_StatusTypeDef I2C_Receive_IT(uint8_t address, uint8_t *data, uint16_t len) {
	if (i2c_op.pending) {
		return HAL_BUSY; // Operacja w toku
	}
	
	if (len > I2C_RXBUF_LEN) {
		return HAL_ERROR; // Zbyt duże dane
	}
	
	i2c_op.address = address;
	i2c_op.data = data;
	i2c_op.len = len;
	i2c_op.operation = 1; // RX
	i2c_op.pending = 1;
	
	// Rozpoczęcie odbioru przez przerwania
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, (address << 1) | 0x01, I2C_RxBuf, len);
	
	if (status != HAL_OK) {
		i2c_op.pending = 0;
	}
	
	return status;
}

// Prosty skan I2C - zwraca pierwszy znaleziony adres (7-bit)
uint8_t I2C_Scan_FirstAddress(uint8_t *found_addr) {
	if (!found_addr) {
		return 0;
	}
	*found_addr = 0;

	HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);

	if (state != HAL_I2C_STATE_READY) {
		HAL_I2C_DeInit(&hi2c1);
		if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
			return 0;
		}
	}

	uint8_t test_addrs[] = {0x23, 0x5C}; // BH1750 adresy
	for (uint8_t i = 0; i < 2; i++) {
		uint8_t addr = test_addrs[i];
		HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100);
		
		if (result == HAL_OK) {
			*found_addr = addr;
			return 1;
		}
	}

	return 0;
}


// Konwersja dwóch znaków hex na bajt
uint8_t hex2byte(char hi, char lo) {
	uint8_t high = (hi >= '0' && hi <= '9') ? hi - '0' :
					(hi >= 'A' && hi <= 'F') ? hi - 'A' + 10 :
					(hi >= 'a' && hi <= 'f') ? hi - 'a' + 10 : 0;
	uint8_t low = (lo >= '0' && lo <= '9') ? lo - '0' :
					(lo >= 'A' && lo <= 'F') ? lo - 'A' + 10 :
					(lo >= 'a' && lo <= 'f') ? lo - 'a' + 10 : 0;
	return (high << 4) | low;
}

uint8_t crc8(uint8_t *data, uint16_t len) {
	uint8_t crc = 0x00;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
}

// Konwersja bajtu na dwa znaki hex
void byte2hex(uint8_t byte, char *hex) {
	static const char hex_chars[] = "0123456789ABCDEF";
	hex[0] = hex_chars[(byte >> 4) & 0x0F];
	hex[1] = hex_chars[byte & 0x0F];
}

// Funkcja sprawdzająca czy string zawiera tylko cyfry
uint8_t is_digits_only(const char *str, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		if (str[i] < '0' || str[i] > '9') {
			return 0;
		}
	}
	return 1;
}

static uint8_t is_addr_char_valid(char c) {
	return (c >= 0x21 && c <= 0x7E && c != '&' && c != '*');
}

// Funkcja do wysyłania ramki odpowiedzi
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data) {
	char frame[271];
	uint16_t pos = 0;
	uint8_t crc_buf[300];
	uint16_t crc_pos = 0;
	
	// Budowanie ramki: & SRC DST ID LEN DATA CRC *
	frame[pos++] = '&';
	
	// SRC (3 znaki) - adres nadawcy odpowiedzi (był odbiorcą w ramce wejściowej)
	memcpy(&frame[pos], src_addr, 3);
	pos += 3;
	memcpy(&crc_buf[crc_pos], src_addr, 3);
	crc_pos += 3;
	
	// DST (3 znaki) - adres odbiorcy odpowiedzi (był nadawcą w ramce wejściowej)
	memcpy(&frame[pos], dst_addr, 3);
	pos += 3;
	memcpy(&crc_buf[crc_pos], dst_addr, 3);
	crc_pos += 3;
	
	// ID (2 znaki)
	memcpy(&frame[pos], id, 2);
	pos += 2;
	memcpy(&crc_buf[crc_pos], id, 2);
	crc_pos += 2;
	
	// LEN (3 znaki) - długość danych
	uint16_t data_len = strlen(data);
	if (data_len > 256) {
		return; // Przekroczony maksymalny rozmiar danych
	}
	char len_str[4];
	// Formatowanie długości na 3 cyfry bez snprintf
	len_str[0] = '0' + (data_len / 100) % 10;
	len_str[1] = '0' + (data_len / 10) % 10;
	len_str[2] = '0' + data_len % 10;
	len_str[3] = 0;
	memcpy(&frame[pos], len_str, 3);
	pos += 3;
	memcpy(&crc_buf[crc_pos], len_str, 3);
	crc_pos += 3;
	
	// DATA
	memcpy(&frame[pos], data, data_len);
	pos += data_len;
	memcpy(&crc_buf[crc_pos], data, data_len);
	crc_pos += data_len;
	
	// Obliczanie CRC
	uint8_t crc = crc8(crc_buf, crc_pos);
	char crc_hex[3];
	byte2hex(crc, crc_hex);
	crc_hex[2] = 0;
	
	// CRC (2 znaki hex)
	memcpy(&frame[pos], crc_hex, 2);
	pos += 2;
	
	// Zakończenie ramki
	frame[pos++] = '*';
	frame[pos] = 0;
	
	// Wysyłanie ramki przez USART
	USART_fsend("%s", frame);
}

/**
 * @brief Sprawdzenie czy czas oczekiwania BH1750 minął
 * @retval 1 jeśli gotowe, 0 jeśli jeszcze czeka
 */
uint8_t BH1750_IsTimingReady(void) {
	if (!bh1750_timing.active) {
		return 1; // Nie ma aktywnego oczekiwania
	}
	
	uint32_t current_time = App_GetTick();
	if ((current_time - bh1750_timing.start_time) >= bh1750_timing.wait_time) {
		bh1750_timing.active = 0;
		return 1; // Czas minął
	}
	
	return 0; // Jeszcze czeka
}

/**
 * @brief Rozpoczęcie śledzenia czasu oczekiwania dla BH1750
 * @param wait_time_ms: Czas oczekiwania w milisekundach
 */
void BH1750_StartTiming(uint32_t wait_time_ms) {
	bh1750_timing.start_time = App_GetTick();
	bh1750_timing.wait_time = wait_time_ms;
	bh1750_timing.active = 1;
}

/**
 * @brief Inicjalizacja BH1750 bez blokowania (Power On -> Reset -> Set Mode)
 */
void BH1750_Init_Process(void) {
	static uint32_t last_retry_time = 0;
	
	if (bh1750_initialized) {
		return;
	}

	// Inicjuj tylko gdy automatyczny pomiar jest włączony
	if (!measurement_auto.enabled) {
		return;
	}

	// Jeśli był błąd I2C, zresetuj stan inicjalizacji
	if (I2C_Error) {
		I2C_Error = 0;
		bh1750_initialized = 0;
		bh1750_init_state = BH1750_INIT_IDLE;
		return;
	}

	// Nie rozpoczynaj kolejnego kroku, jeśli I2C jest zajęte
	if (i2c_op.pending) {
		return;
	}
	
	// Throttling - nie próbuj zbyt często (1 sekunda między próbami)
	if (bh1750_init_state == BH1750_INIT_IDLE) {
		uint32_t now = App_GetTick();
		if (now - last_retry_time < 1000) {
			return; // Czekaj
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
 * @brief Ustawienie trybu pracy czujnika BH1750 (przez przerwania, bez HAL_Delay)
 * @param mode: Tryb pracy (jedna z komend BH1750: 0x10, 0x11, 0x13, 0x20, 0x21, 0x23)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode) {
	HAL_StatusTypeDef status;
	uint8_t addr = bh1750_addr; // Adres I2C (7-bit, bez przesunięcia - funkcja I2C_Transmit_IT zrobi to)
	
	// Wysyłanie komendy trybu do czujnika przez przerwania
	status = I2C_Transmit_IT(addr, &mode, 1);
	
	if (status == HAL_OK) {
		bh1750_current_mode = mode;
		
		// Rozpoczęcie śledzenia czasu oczekiwania
		// Opóźnienie w zależności od trybu (120ms dla high res, 16ms dla low res)
		if (mode == BH1750_CONTINUOUS_LOW_RES_MODE || mode == BH1750_ONETIME_LOW_RES_MODE) {
			BH1750_StartTiming(16);
		} else {
			BH1750_StartTiming(120);
		}
	}
	
	return status;
}

/**
 * @brief Dodanie nowego pomiaru do bufora
 * @param lux: Wartość natężenia światła w luksach
 */
void Measurement_AddEntry(float lux) {
	// Zapisanie pomiaru do bufora
	measurement_buffer[measurement_write_index].lux = lux;
	measurement_buffer[measurement_write_index].timestamp = App_GetTick();
	
	// Aktualizacja indeksu (bufor cykliczny)
	measurement_write_index++;
	if (measurement_write_index >= MEASUREMENT_BUFFER_SIZE) {
		measurement_write_index = 0; // Zawinięcie bufora
	}
	
	// Aktualizacja licznika (max 1000)
	if (measurement_count < MEASUREMENT_BUFFER_SIZE) {
		measurement_count++;
	}
}

// Tymczasowe wypełnienie bufora pomiarów danymi testowymi (stałe wartości)
void Measurement_FillTestData(void) {
	static const float test_values[] = {
		10.0f, 100.0f, 250.0f, 500.0f, 750.0f,
		1000.0f, 1500.0f, 2000.0f, 3500.0f, 5000.0f
	};
	for (uint16_t i = 0; i < (uint16_t)(sizeof(test_values) / sizeof(test_values[0])); i++) {
		Measurement_AddEntry(test_values[i]);
	}
}

/**
 * @brief Pobranie liczby zapisanych pomiarów
 * @retval Liczba pomiarów (0-1000)
 */
uint16_t Measurement_GetCount(void) {
	return measurement_count;
}

/**
 * @brief Pobranie wpisu pomiaru z bufora
 * @param index: Indeks pomiaru (0 to najstarszy, count-1 to najnowszy)
 * @retval Wskaźnik do wpisu pomiaru lub NULL jeśli indeks nieprawidłowy
 */
measurement_entry_t* Measurement_GetEntry(uint16_t index) {
	if (index >= measurement_count) {
		return NULL; // Nieprawidłowy indeks
	}
	
	// Obliczenie rzeczywistego indeksu w buforze cyklicznym
	// Najstarszy pomiar jest na początku, jeśli bufor jest pełny
	uint16_t real_index;
	if (measurement_count < MEASUREMENT_BUFFER_SIZE) {
		// Bufor nie jest jeszcze pełny - indeksy są liniowe
		real_index = index;
	} else {
		// Bufor jest pełny - najstarszy jest po najnowszym
		real_index = (measurement_write_index + index) % MEASUREMENT_BUFFER_SIZE;
	}
	
	return &measurement_buffer[real_index];
}

/**
 * @brief Odczyt wartości natężenia światła z BH1750 (przez przerwania)
 * @param lux: Wskaźnik do zmiennej, gdzie zostanie zapisana wartość w luksach
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_ReadLight(float *lux) {
	uint8_t addr = bh1750_addr;
	
	// Sprawdzenie czy operacja I2C nie jest w toku
	if (i2c_op.pending) {
		return HAL_BUSY;
	}
	
	// Sprawdzenie czy ostatni odczyt jest gotowy
	if (bh1750_read_ready) {
		*lux = bh1750_last_lux;
		bh1750_read_ready = 0;
		return HAL_OK;
	}
	
	// Rozpoczęcie odczytu przez przerwania
	HAL_StatusTypeDef status = I2C_Receive_IT(addr, bh1750_read_buffer, 2);
	
	return status; // Wartość zostanie przetworzona w callbacku
}

/**
 * @brief Ustawienie interwału automatycznego odczytu pomiarów
 * @param interval_ms: Interwał w milisekundach
 */
void Measurement_SetInterval(uint32_t interval_ms) {
	measurement_auto.interval_ms = interval_ms;
}

/**
 * @brief Włączenie/wyłączenie automatycznego odczytu pomiarów
 * @param enable: 1 = włącz, 0 = wyłącz
 */
void Measurement_EnableAutoRead(uint8_t enable) {
	measurement_auto.enabled = enable;
	if (enable) {
		measurement_auto.last_measurement = App_GetTick();
	}
}

/**
 * @brief Przetwarzanie automatycznego odczytu pomiarów (wywoływane w pętli głównej)
 */
void Measurement_AutoRead_Process(void) {
	if (!measurement_auto.enabled) {
		return; // Automatyczny odczyt wyłączony
	}
	if (!bh1750_initialized) {
		return; // Czekaj na zakończenie inicjalizacji BH1750
	}
	
	// Sprawdzenie czy dane z odczytu są gotowe i zapisanie ich
	if (bh1750_read_ready) {
		// Zapis pomiaru do bufora
		Measurement_AddEntry(bh1750_last_lux);
		bh1750_read_ready = 0; // Wyzeruj flagę po zapisaniu
	}

	// Sprawdzenie czy minął interwał
	uint32_t current_time = App_GetTick();
	if ((current_time - measurement_auto.last_measurement) >= measurement_auto.interval_ms) {
		measurement_auto.last_measurement = current_time; // Zaktualizuj czas
		
		// Sprawdzenie czy I2C nie jest zajęty
		if (!i2c_op.pending) {
			// Rozpoczęcie nowego odczytu z czujnika
			bh1750_read_ready = 0; // Wyzeruj flagę przed rozpoczęciem odczytu
			HAL_StatusTypeDef status = BH1750_ReadLight(&bh1750_last_lux);
			
			// Jeśli I2C zawiódł, dodaj wartość 0 jako wskaźnik błędu
			if (status != HAL_OK) {
				Measurement_AddEntry(0.0f); // Błąd I2C
			}
		}
	}
}

void handle_command(char *cmd, const char *src_addr, const char *dst_addr, const char *id) {
	const char *device_addr = dst_addr;
	
	// Zakres dozwolonych wartości interwału
	const uint16_t MIN_INTERVAL = 1;
	const uint16_t MAX_INTERVAL = 9999;
	
	// Sprawdzenie minimalnej długości (kod komendy = 2 cyfry)
	if (strlen(cmd) < 2 || !is_digits_only(cmd, 2)) {
		USART_fsend("ERR: INVALID CMD\r\n");
		return;
	}
	
	// Wyodrębnienie kodu komendy (pierwsze 2 cyfry)
	uint8_t cmd_code = (cmd[0] - '0') * 10 + (cmd[1] - '0');
	const char *params = (strlen(cmd) > 2) ? &cmd[2] : "";
	
	// 10 - START
	if (cmd_code == 10) {
		Measurement_EnableAutoRead(1);
		send_response_frame(device_addr, src_addr, id, "00"); // OK
	}
	// 11 - STOP
	else if (cmd_code == 11) {
		Measurement_EnableAutoRead(0);
		send_response_frame(device_addr, src_addr, id, "00"); // OK
	}
	// 12 - DOWNLOAD (ostatni pomiar)
	else if (cmd_code == 12) {
		uint16_t count = Measurement_GetCount();
		if (count == 0) {
			send_response_frame(device_addr, src_addr, id, "03"); // ERR_NO_DATA
			return;
		}
		measurement_entry_t *entry = Measurement_GetEntry(count - 1);
		if (!entry) {
			send_response_frame(device_addr, src_addr, id, "03"); // ERR_NO_DATA
			return;
		}
		uint32_t lux_val = (uint32_t)(entry->lux + 0.5f);
		if (lux_val > 9999) {
			lux_val = 9999;
		}
		char response[5];
		response[0] = '0' + (lux_val / 1000) % 10;
		response[1] = '0' + (lux_val / 100) % 10;
		response[2] = '0' + (lux_val / 10) % 10;
		response[3] = '0' + lux_val % 10;
		response[4] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	}
	// 13 - VIEW (+ xxzz parametry)
	else if (cmd_code == 13) {
		if (strlen(params) < 4) {
			send_response_frame(device_addr, src_addr, id, "01"); // ERR_PARAM
			return;
		}
		uint8_t start_offset = (params[0] - '0') * 10 + (params[1] - '0');
		uint8_t count_req = (params[2] - '0') * 10 + (params[3] - '0');
		if (count_req == 0) {
			send_response_frame(device_addr, src_addr, id, "01"); // ERR_PARAM
			return;
		}
		uint16_t count = Measurement_GetCount();
		
		// Sprawdzenie czy offset nie przekracza liczby pomiarów
		if (start_offset >= count) {
			send_response_frame(device_addr, src_addr, id, "03"); // ERR_NO_DATA
			return;
		}
		
		// Maksymalna liczba pomiarów na ramkę: (256 - 2) / 4 = 63
		// Format: xxLLLLLLLLLLLL... (offset 2 znaki + N*4 znaki lux)
		#define MAX_MEASUREMENTS_PER_FRAME 63
		char data_out[256];
		uint8_t measurements_sent = 0;
		
		while (measurements_sent < count_req) {
			uint8_t current_offset = start_offset + measurements_sent;
			uint8_t batch_size = 0;
			uint16_t data_pos = 0;
			
			// Offset (2 znaki)
			data_out[data_pos++] = '0' + (current_offset / 10) % 10;
			data_out[data_pos++] = '0' + current_offset % 10;
			
			// Pakowanie pomiarów do ramki
			for (uint8_t i = 0; i < MAX_MEASUREMENTS_PER_FRAME && measurements_sent < count_req; i++) {
				int32_t idx = (int32_t)count - 1 - (int32_t)current_offset - (int32_t)i;
				if (idx < 0) {
					break;
				}
				measurement_entry_t *entry = Measurement_GetEntry((uint16_t)idx);
				if (!entry) {
					break;
				}
				uint32_t lux_val = (uint32_t)(entry->lux + 0.5f);
				if (lux_val > 9999) {
					lux_val = 9999;
				}
				// Dodanie 4-cyfrowej wartości lux
				data_out[data_pos++] = '0' + (lux_val / 1000) % 10;
				data_out[data_pos++] = '0' + (lux_val / 100) % 10;
				data_out[data_pos++] = '0' + (lux_val / 10) % 10;
				data_out[data_pos++] = '0' + lux_val % 10;
				batch_size++;
				measurements_sent++;
			}
			
			data_out[data_pos] = 0;
			send_response_frame(device_addr, src_addr, id, data_out);
			
			// Jeśli nie wysłano żadnych pomiarów, przerwij
			if (batch_size == 0) {
				break;
			}
		}
	}
	// 14 - SET_INTERVAL (+ xxxx parametr)
	else if (cmd_code == 14) {
		if (strlen(params) < 4) {
			send_response_frame(device_addr, src_addr, id, "01"); // ERR_PARAM
			return;
		}
		uint16_t interval_ms = atoi(params);
		if (interval_ms < MIN_INTERVAL || interval_ms > MAX_INTERVAL) {
			send_response_frame(device_addr, src_addr, id, "02"); // ERR_RANGE
			return;
		}
		Measurement_SetInterval(interval_ms);
		send_response_frame(device_addr, src_addr, id, "00"); // OK
	}
	// 15 - GET_INTERVAL
	else if (cmd_code == 15) {
		uint16_t interval_ms = measurement_auto.interval_ms;
		if (interval_ms > 9999) {
			interval_ms = 9999;
		}
		char response[6];
		response[0] = '0' + (interval_ms / 1000) % 10;
		response[1] = '0' + (interval_ms / 100) % 10;
		response[2] = '0' + (interval_ms / 10) % 10;
		response[3] = '0' + interval_ms % 10;
		response[4] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	}
	// 16 - SET_MODE (+ x parametr)
	else if (cmd_code == 16) {
		if (strlen(params) < 1 || params[0] < '1' || params[0] > '6') {
			send_response_frame(device_addr, src_addr, id, "01"); // ERR_PARAM
			return;
		}
		uint8_t mode_num = params[0] - '0';
		uint8_t mode_value;
		switch (mode_num) {
			case 1: mode_value = BH1750_CONTINUOUS_HIGH_RES_MODE; break;
			case 2: mode_value = BH1750_CONTINUOUS_HIGH_RES_MODE_2; break;
			case 3: mode_value = BH1750_CONTINUOUS_LOW_RES_MODE; break;
			case 4: mode_value = BH1750_ONETIME_HIGH_RES_MODE; break;
			case 5: mode_value = BH1750_ONETIME_HIGH_RES_MODE_2; break;
			case 6: mode_value = BH1750_ONETIME_LOW_RES_MODE; break;
			default:
				send_response_frame(device_addr, src_addr, id, "01"); // ERR_PARAM
				return;
		}
		HAL_StatusTypeDef status = BH1750_SetMode(mode_value);
		if (status == HAL_OK) {
			send_response_frame(device_addr, src_addr, id, "00"); // OK
		} else {
			send_response_frame(device_addr, src_addr, id, "04"); // ERR_I2C
		}
	}
	// 17 - GET_MODE
	else if (cmd_code == 17) {
		char mode_char = '1';
		switch (bh1750_current_mode) {
			case BH1750_CONTINUOUS_HIGH_RES_MODE: mode_char = '1'; break;
			case BH1750_CONTINUOUS_HIGH_RES_MODE_2: mode_char = '2'; break;
			case BH1750_CONTINUOUS_LOW_RES_MODE: mode_char = '3'; break;
			case BH1750_ONETIME_HIGH_RES_MODE: mode_char = '4'; break;
			case BH1750_ONETIME_HIGH_RES_MODE_2: mode_char = '5'; break;
			case BH1750_ONETIME_LOW_RES_MODE: mode_char = '6'; break;
			default: mode_char = '1'; break;
		}
		char response[2];
		response[0] = mode_char;
		response[1] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	}
	// 18 - I2C_SCAN (zwraca pierwszy znaleziony adres)
	else if (cmd_code == 18) {
		uint8_t found_addr = 0;
		char response[5];
		if (I2C_Scan_FirstAddress(&found_addr)) {
			// Odpowiedź: hex adresu (2 znaki)
			byte2hex(found_addr, &response[0]);
			response[2] = 0;
			send_response_frame(device_addr, src_addr, id, response);
		} else {
			// Brak urządzeń - wysyłamy "00"
			response[0] = '0';
			response[1] = '0';
			response[2] = 0;
			send_response_frame(device_addr, src_addr, id, response);
		}
	}
	// Nieznany kod komendy
	else {
		USART_fsend("ERR: UNKNOWN CMD\r\n");
	}
}

void validate_frame(char *f, uint16_t flen)
{
	char src[4], dst[4], id[3], len_str[4];
	
	/* ====== Sprawdzenie minimalnej długości ramki ====== */
	/* Minimalna ramka: & SRC(3) DST(3) ID(2) LEN(3) CRC(2) * = 13 znaków */
	if(flen < 13)
	{
		// Ramka zbyt krótka - błąd strukturalny
		USART_fsend("ERR: TOO SHORT\r\n");
		return;
	}

	/* ====== Parsowanie pól ====== */
	uint16_t pos = 1;  // po '&'

	memcpy(src, &f[pos], 3); src[3]=0; pos+=3;
	memcpy(dst, &f[pos], 3); dst[3]=0; pos+=3;
	memcpy(id,  &f[pos], 2); id[2]=0;  pos+=2;
	memcpy(len_str, &f[pos], 3); len_str[3]=0; pos+=3;

	/* ====== Walidacja znaków SRC/DST ====== */
	for (uint8_t i = 0; i < 3; i++) {
		if (!is_addr_char_valid(src[i]) || !is_addr_char_valid(dst[i])) {
			// Nieprawidłowe znaki w adresach - błąd strukturalny
			USART_fsend("ERR: INVALID ADDR\r\n");
			return;
		}
	}

	/* ====== Sprawdzenie formatu ID (tylko cyfry 0-9) ====== */
	if(!is_digits_only(id, 2))
	{
		// Nieprawidłowe znaki w ID - błąd strukturalny
		USART_fsend("ERR: INVALID ID\r\n");
		return;
	}

	/* ====== Sprawdzenie formatu długości (tylko cyfry 0-9) ====== */
	if(!is_digits_only(len_str, 3))
	{
		// Nieprawidłowe znaki w LEN - błąd strukturalny
		USART_fsend("ERR\r\n");
		return;
	}

	uint16_t data_len_declared = atoi(len_str);

	/* ====== Sprawdzenie maksymalnej długości ====== */
	if(data_len_declared > 256)
	{
		// Długość > 256 - błąd strukturalny
		USART_fsend("ERR_LENGTH\r\n");
		return;
	}

	/* ====== Sprawdzenie faktycznej długości danych ====== */
	uint16_t data_start = pos;
	
	// Sprawdzenie czy ramka jest wystarczająco długa dla deklarowanej długości danych
	// Minimalna długość: & + SRC(3) + DST(3) + ID(2) + LEN(3) + DATA + CRC(2) + * = 14 + DATA
	if(flen < (data_start + data_len_declared + 3)) // data_start + data_len + CRC(2) + '*'(1)
	{
		// Niezgodność długości - błąd strukturalny
		USART_fsend("ERR_LENGTH\r\n");
		return;
	}
	
	// CRC jest 3 znaki od końca (CRC ma 2 znaki + '*')
	uint16_t crc_pos = data_start + data_len_declared;
	uint16_t data_len_real = data_len_declared;

	/* ====== Odczyt danych ====== */
	char data[257];
	memcpy(data, &f[data_start], data_len_real);
	data[data_len_real] = 0;

	/* ====== Walidacja znaków DATA (tylko cyfry 0-9) ====== */
	if (!is_digits_only(data, data_len_real)) {
		// Nieprawidłowe znaki w DATA - błąd strukturalny
		USART_fsend("ERR\r\n");
		return;
	}

	/* ====== Sprawdzenie czy mamy CRC ====== */
	if(flen < (crc_pos + 2))
	{
		// Brak pola CRC - błąd strukturalny
		USART_fsend("ERR\r\n");
		return;
	}

	/* ====== Odczyt i weryfikacja CRC ====== */
	uint8_t rx_crc = hex2byte(f[crc_pos], f[crc_pos+1]);

	/* ====== Budowa bufora do obliczenia CRC ====== */
	/* Maksymalny rozmiar: src(3) + dst(3) + id(2) + len(3) + data(256) = 267 bajtów */
	uint8_t buf[270];
	uint16_t p = 0;

	memcpy(&buf[p], src, 3); p+=3;
	memcpy(&buf[p], dst, 3); p+=3;
	memcpy(&buf[p], id, 2);  p+=2;
	memcpy(&buf[p], len_str, 3); p+=3;
	memcpy(&buf[p], data, data_len_real); p+=data_len_real;

	/* ====== Obliczenie CRC ====== */
	uint8_t calc_crc = crc8(buf, p);

	/* ====== Sprawdzenie CRC ====== */
	if(calc_crc != rx_crc)
	{
		// Błędne CRC - błąd strukturalny
		USART_fsend("ERR_CRC\r\n");
		return;
	}
	
	// Przekazanie informacji o ramce do obsługi komendy
	handle_command(data, src, dst, id);
}

// Funkcja sprawdzająca i przetwarzająca ramki w buforze kołowym
void process_uart_buffer(void)
{
    while(USART_kbhit())
    {
        char c = USART_getchar();

        switch(st)
        {
        case ST_IDLE:
            if(c == '&')
            {
                pos = 0;
                frame[pos++] = c;
                st = ST_COLLECT;
            }
            break;

        case ST_COLLECT:

            /* nowy start resetuje ramkę */
            if(c == '&')
            {
                pos = 0;
                frame[pos++] = c;
                break;
            }

            /* zapisuj znak */
            if(pos < sizeof(frame)-1)
                frame[pos++] = c;

            /* koniec ramki */
            if(c == '*')
            {
                frame[pos] = 0;   // string end
                validate_frame(frame,pos);
                st = ST_IDLE;
            }
            break;
        }
    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	// Inicjalizacja BH1750 wykonywana bez blokowania w pętli głównej
	USART_fsend("\r\nSYSTEM START\r\n");
	// Tymczasowe dane testowe do sprawdzenia komend (stałe wartości)
	Measurement_FillTestData();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		I2C_BusRecovery_Process();
		process_uart_buffer();
		BH1750_Init_Process();
		Measurement_AutoRead_Process(); 

		if(USART_RxBufOverflow) {
			USART_fsend("\r\nERROR: USART RX buffer overflow!\r\n");
			USART_RxBufOverflow = 0;
		}
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		app_tick++;
	}
}

// Callbacki I2C
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		i2c_op.pending = 0;
		// Operacja zakończona - można wykonać kolejną
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		// Kopiowanie danych z bufora do miejsca docelowego
		if (i2c_op.operation == 1 && i2c_op.data != NULL) {
			for (uint16_t i = 0; i < i2c_op.len; i++) {
				i2c_op.data[i] = I2C_RxBuf[i];
			}
			
			// Jeśli to odczyt BH1750 (2 bajty)
			if (i2c_op.len == 2 && i2c_op.data == bh1750_read_buffer) {
				// Konwersja bajtów na 16-bitową wartość (big-endian)
				uint16_t raw_value = (bh1750_read_buffer[0] << 8) | bh1750_read_buffer[1];
				// Przeliczenie na luksy: wartość / 1.2 (dla trybu H_RES_MODE)
				bh1750_last_lux = raw_value / 1.2f;
				bh1750_read_ready = 1;
			}
		}
		i2c_op.pending = 0;
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		I2C_Error = 1;
		i2c_op.pending = 0;
		I2C_BusReset_Pending = 1;
	}
}

/**
 * @brief Nieblokujące odzyskiwanie zablokowanej magistrali I2C (Bus Recovery)
 * 
 * Funkcja implementuje procedurę odzyskiwania magistrali I2C zgodnie ze standardem I2C.
 * Wykorzystuje maszynę stanów do wykonania recovery bez blokowania głównej pętli programu.
 * 
 * PROBLEM:
 * Gdy urządzenie I2C (BH1750) zawiesi się i trzyma linię SDA na LOW, magistrala jest
 * zablokowana i STM32 nie może komunikować się z czujnikiem.
 * 
 * ROZWIĄZANIE:
 * 1. Wyłączenie peryferii I2C STM32
 * 2. Rekonfiguracja pinów jako GPIO (Open-Drain)
 * 3. Wygenerowanie 9 pulsów zegarowych (SCL) - wymusza slave do zakończenia operacji
 * 4. Wygenerowanie STOP condition (SDA: LOW→HIGH gdy SCL=HIGH)
 * 5. Przywrócenie konfiguracji I2C
 * 
 * TIMING:
 * - Każdy krok czeka 1ms (używa App_GetTick() z TIM2)
 * - Całość trwa ~23ms bez blokowania CPU
 * - Throttling: maksymalnie 1 recovery na 100ms
 * 
 * WYWOŁANIE:
 * Automatyczne uruchomienie gdy HAL_I2C_ErrorCallback() ustawi flagę I2C_BusReset_Pending
 * 
 * @note Funkcja wywoływana cyklicznie w pętli głównej
 */
void I2C_BusRecovery_Process(void) {
	static uint32_t last_reset = 0;
	uint32_t now = App_GetTick();

	// Sprawdź czy recovery jest wymagany
	if (!I2C_BusReset_Pending && bus_recovery.state == BUS_RECOVERY_IDLE) {
		return; // Brak błędu I2C - nic do roboty
	}

	// Throttling - rozpocznij recovery tylko raz na 100ms (zabezpieczenie przed zapętleniem)
	if (I2C_BusReset_Pending && bus_recovery.state == BUS_RECOVERY_IDLE) {
		if (now - last_reset < 100) {
			return; // Czekaj minimalnie 100ms między próbami recovery
		}
		last_reset = now;
		I2C_BusReset_Pending = 0;
		bus_recovery.state = BUS_RECOVERY_DISABLE_I2C;
		bus_recovery.pulse_count = 0;
		bus_recovery.last_time = now;
	}

	// Maszyna stanów dla nieblokującego recovery (każdy krok co 1ms)
	switch (bus_recovery.state) {
		case BUS_RECOVERY_IDLE:
			// Stan bezczynności - czekanie na błąd I2C
			break;

		case BUS_RECOVERY_DISABLE_I2C:
			// Krok 1: Wyłączenie peryferii I2C STM32
			__HAL_I2C_DISABLE(&hi2c1);
			bus_recovery.state = BUS_RECOVERY_CONFIG_GPIO;
			break;

		case BUS_RECOVERY_CONFIG_GPIO: {
			// Krok 2: Rekonfiguracja pinów I2C jako GPIO (Open-Drain z pull-up)
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // PB6=SCL, PB7=SDA
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;     // Open-Drain (jak I2C)
			GPIO_InitStruct.Pull = GPIO_PULLUP;             // Pull-up
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			bus_recovery.pulse_count = 0;
			bus_recovery.last_time = now;
			bus_recovery.state = BUS_RECOVERY_PULSE_LOW;
			break;
		}

		case BUS_RECOVERY_PULSE_LOW:
			// Krok 3a: Generowanie pulsów zegarowych - faza LOW (czekaj 1ms)
			if (now - bus_recovery.last_time >= 1) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // SCL LOW
				bus_recovery.last_time = now;
				bus_recovery.state = BUS_RECOVERY_PULSE_HIGH;
			}
			break;

		case BUS_RECOVERY_PULSE_HIGH:
			// Krok 3b: Generowanie pulsów zegarowych - faza HIGH (czekaj 1ms)
			// Powtarzamy 9 razy aby slave mógł zakończyć przerwane wysyłanie
			if (now - bus_recovery.last_time >= 1) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL HIGH
				bus_recovery.last_time = now;
				bus_recovery.pulse_count++;
				if (bus_recovery.pulse_count >= 9) {
					// 9 pulsów zakończone - przejdź do generowania STOP condition
					bus_recovery.state = BUS_RECOVERY_STOP_SDA_LOW;
				} else {
					// Kolejny puls
					bus_recovery.state = BUS_RECOVERY_PULSE_LOW;
				}
			}
			break;

		case BUS_RECOVERY_STOP_SDA_LOW:
			// Krok 4a: STOP condition - ustaw SDA na LOW (czekaj 1ms)
			if (now - bus_recovery.last_time >= 1) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // SDA LOW
				bus_recovery.last_time = now;
				bus_recovery.state = BUS_RECOVERY_STOP_SCL_HIGH;
			}
			break;

		case BUS_RECOVERY_STOP_SCL_HIGH:
			// Krok 4b: STOP condition - ustaw SCL na HIGH (czekaj 1ms)
			if (now - bus_recovery.last_time >= 1) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL HIGH
				bus_recovery.last_time = now;
				bus_recovery.state = BUS_RECOVERY_STOP_SDA_HIGH;
			}
			break;

		case BUS_RECOVERY_STOP_SDA_HIGH:
			// Krok 4c: STOP condition - ustaw SDA na HIGH (przejście LOW→HIGH = STOP)
			if (now - bus_recovery.last_time >= 1) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // SDA HIGH
				bus_recovery.last_time = now;
				bus_recovery.state = BUS_RECOVERY_RESTORE_I2C;
			}
			break;

		case BUS_RECOVERY_RESTORE_I2C:
			// Krok 5: Przywrócenie konfiguracji I2C i powrót do normalnej pracy
			if (now - bus_recovery.last_time >= 1) {
				HAL_I2C_DeInit(&hi2c1);  // Deinicjalizacja I2C
				HAL_I2C_Init(&hi2c1);    // Reinicjalizacja I2C (piny wrócą do funkcji AF)
				bus_recovery.state = BUS_RECOVERY_IDLE; // Gotowe - czekaj na kolejny błąd
			}
			break;

		default:
			// Zabezpieczenie - nieprawidłowy stan
			bus_recovery.state = BUS_RECOVERY_IDLE;
			break;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
