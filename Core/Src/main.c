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

/* FRAME PARSING */
typedef enum {
    ST_IDLE = 0,
    ST_COLLECT
} frame_state_t;

frame_state_t st = ST_IDLE;

char frame[600];
uint16_t pos = 0;


char src[4], dst[4], id[3], len_str[4];
char data[256+1];
uint8_t crc_rx = 0;
uint16_t len = 0;
uint16_t data_pos = 0;

uint8_t escape = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t is_digits_only(const char *str, uint16_t len);
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data);
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode);
HAL_StatusTypeDef I2C_Transmit_IT(uint8_t address, uint8_t *data, uint16_t len);
HAL_StatusTypeDef I2C_Receive_IT(uint8_t address, uint8_t *data, uint16_t len);
uint8_t BH1750_IsTimingReady(void);
void BH1750_StartTiming(uint32_t wait_time_ms);
void Measurement_AddEntry(float lux);
uint16_t Measurement_GetCount(void);
measurement_entry_t* Measurement_GetEntry(uint16_t index);
HAL_StatusTypeDef BH1750_ReadLight(float *lux);
void Measurement_AutoRead_Process(void);
void Measurement_SetInterval(uint32_t interval_ms);
void Measurement_EnableAutoRead(uint8_t enable);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define USART_TXBUF_LEN 1512 // dlaczego
#define USART_RXBUF_LEN 128
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

__IO int I2C_TX_Empty = 0; // Nadawanie head
__IO int I2C_TX_Busy = 0;  // Nadawanie tail
__IO int I2C_RX_Empty = 0; // Odbieranie head
__IO int I2C_RX_Busy = 0;  // Odbieranie tail

// Sygnalizacja błędów I2C
__IO uint8_t I2C_RxBufOverflow = 0;
__IO uint8_t I2C_Error = 0;

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

static measurement_auto_t measurement_auto = {
	.interval_ms = 1000,        // Domyślnie 1 sekunda
	.last_measurement = 0,
	.enabled = 0                // Wyłączone domyślnie
};

// Bufor do odczytu BH1750
static uint8_t bh1750_read_buffer[2] = {0};
static float bh1750_last_lux = 0.0f;
static uint8_t bh1750_read_ready = 0;

// Struktura dla przechowywania pomiarów BH1750
typedef struct {
	float lux;           // Wartość natężenia światła w luksach
	uint32_t timestamp;  // Timestamp pomiaru (HAL_GetTick())
} measurement_entry_t;

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

uint8_t USART_getline(char *buf) {
	static uint8_t bf[128];
	static uint8_t idx = 0;
	int i;
	uint8_t ret;
	while (USART_kbhit()) {
		bf[idx] = USART_getchar();
		// USART_fsend("[%02X]", bf[idx]);
		if (((bf[idx] == 10) || (bf[idx] == 13))) {
			bf[idx] = 0;
			for (i = 0; i <= idx; i++) {
				buf[i] = bf[i];
			}
			ret = idx;
			idx = 0;
			return ret;//odebrano linie
		} else {
			idx++;
			if (idx >= 128)
				idx = 0;
		}
	}
	return 0;
} //USART_getline

void USART_fsend(char *format, ...) {
	char tmp_rs[128];
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
		if (USART_TX_Emx`pty != USART_TX_Busy) {
			uint8_t tmp = USART_TxBuf[USART_TX_Busy];
			USART_TX_Busy++;
			if (USART_TX_Busy >= USART_TXBUF_LEN)
				USART_TX_Busy = 0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if(huart==&huart2){
		 USART_RX_Empty++;
		 if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0;
		 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1);
	 }
}

// Funkcja wywoływana w przerwaniu odbioru
//static uint8_t rx_byte;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	int next_head = (USART_RX_Empty + 1) % USART_RXBUF_LEN;
//	if (next_head == USART_RX_Busy)
//		USART_RxBufOverflow = 1;
//	else {
//		USART_RxBuf[USART_RX_Empty] = rx_byte;
//		USART_RX_Empty = next_head;
//	}
//	HAL_UART_Receive_IT(huart, &rx_byte, 1);
//}

void USART2_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
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

// Funkcja do wysyłania ramki odpowiedzi
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data) {
	char frame[600];
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
	
	uint32_t current_time = HAL_GetTick();
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
	bh1750_timing.start_time = HAL_GetTick();
	bh1750_timing.wait_time = wait_time_ms;
	bh1750_timing.active = 1;
}

/**
 * @brief Ustawienie trybu pracy czujnika BH1750 (przez przerwania, bez HAL_Delay)
 * @param mode: Tryb pracy (jedna z komend BH1750: 0x10, 0x11, 0x13, 0x20, 0x21, 0x23)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode) {
	HAL_StatusTypeDef status;
	uint8_t addr = BH1750_ADDR_LOW; // Adres I2C (7-bit, bez przesunięcia - funkcja I2C_Transmit_IT zrobi to)
	
	// Wysyłanie komendy trybu do czujnika przez przerwania
	status = I2C_Transmit_IT(addr, &mode, 1);
	
	if (status == HAL_OK) {
		bh1750_current_mode = mode;
		
		// Rozpoczęcie śledzenia czasu oczekiwania (bez HAL_Delay!)
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
	measurement_buffer[measurement_write_index].timestamp = HAL_GetTick();
	
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

// Bufor do odczytu BH1750
static uint8_t bh1750_read_buffer[2] = {0};
static float bh1750_last_lux = 0.0f;
static uint8_t bh1750_read_ready = 0;

/**
 * @brief Odczyt wartości natężenia światła z BH1750 (przez przerwania)
 * @param lux: Wskaźnik do zmiennej, gdzie zostanie zapisana wartość w luksach
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_ReadLight(float *lux) {
	uint8_t addr = BH1750_ADDR_LOW;
	
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
		measurement_auto.last_measurement = HAL_GetTick();
	}
}

/**
 * @brief Przetwarzanie automatycznego odczytu pomiarów (wywoływane w pętli głównej)
 */
void Measurement_AutoRead_Process(void) {
	if (!measurement_auto.enabled) {
		return; // Automatyczny odczyt wyłączony
	}
	
	// Sprawdzenie czy minął interwał
	uint32_t current_time = HAL_GetTick();
	if ((current_time - measurement_auto.last_measurement) >= measurement_auto.interval_ms) {
		// Sprawdzenie czy I2C nie jest zajęty i czy czujnik jest gotowy
		if (!i2c_op.pending && BH1750_IsTimingReady()) {
			// Sprawdzenie czy ostatni odczyt jest gotowy
			if (bh1750_read_ready) {
				// Zapis pomiaru do bufora
				Measurement_AddEntry(bh1750_last_lux);
				measurement_auto.last_measurement = current_time;
				bh1750_read_ready = 0;
			} else {
				// Rozpoczęcie nowego odczytu
				BH1750_ReadLight(&bh1750_last_lux); // Rozpocznie asynchroniczny odczyt
			}
		}
	}
}

void handle_command(char *cmd, const char *src_addr, const char *dst_addr, const char *id) {
	// Domyślny adres urządzenia (można zmienić na właściwy)
	const char *device_addr = "PC_";
	
	// Zakres dozwolonych wartości interwału (można dostosować)
	const uint16_t MIN_INTERVAL = 1;
	const uint16_t MAX_INTERVAL = 3600;
	
	if (strcmp(cmd, "START") == 0) {
		Measurement_EnableAutoRead(1); // Włączenie automatycznego odczytu
		send_response_frame(device_addr, src_addr, id, "OK_START");
	} else if (strcmp(cmd, "STOP") == 0) {
		Measurement_EnableAutoRead(0); // Wyłączenie automatycznego odczytu
		send_response_frame(device_addr, src_addr, id, "OK_STOP");
	} else if (strcmp(cmd, "ALL") == 0) {
		send_response_frame(device_addr, src_addr, id, "OK_ALL");
	} else if (strncmp(cmd, "VIEW_ONE", 8) == 0) {
		char response[270];
		// Konkatenacja "OK_" z cmd bez snprintf
		memcpy(response, "OK_", 3);
		uint16_t cmd_len = strlen(cmd);
		memcpy(&response[3], cmd, cmd_len);
		response[3 + cmd_len] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	} else if (strncmp(cmd, "SET_INTERVAL", 12) == 0) {
		// Komenda SET_INTERVAL - ustawienie interwału pomiarowego
		// Format: SET_INTERVALxxxx gdzie xxxx to interwał w sekundach (np. SET_INTERVAL005 = 5 sekund)
		
		if (strlen(cmd) < 16) { // SET_INTERVAL + 4 cyfry = 16 znaków
			send_response_frame(device_addr, src_addr, id, "ERR_INVALID_PARAM");
			return;
		}
		
		// Wyodrębnienie parametru (4 cyfry po "SET_INTERVAL")
		const char *param_str = &cmd[12];
		
		// Sprawdzenie czy parametr zawiera tylko cyfry
		if (!is_digits_only(param_str, 4)) {
			send_response_frame(device_addr, src_addr, id, "ERR_INVALID_PARAM");
			return;
		}
		
		// Konwersja parametru na liczbę (sekundy)
		uint16_t interval_sec = atoi(param_str);
		
		// Sprawdzenie zakresu wartości (1-3600 sekund)
		if (interval_sec < MIN_INTERVAL || interval_sec > MAX_INTERVAL) {
			send_response_frame(device_addr, src_addr, id, "ERR_INVALID_PARAM");
			return;
		}
		
		// Ustawienie interwału (konwersja sekund na milisekundy)
		Measurement_SetInterval(interval_sec * 1000);
		
		char response[50];
		memcpy(response, "OK_SET_INTERVAL", 15);
		memcpy(&response[15], param_str, 4);
		response[19] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	} else if (strcmp(cmd, "GET_INTERVAL") == 0) {
		// Zwrócenie aktualnego interwału
		uint16_t interval_sec = measurement_auto.interval_ms / 1000;
		char response[50];
		memcpy(response, "OK_INTERVAL", 11);
		// Formatowanie interwału na 4 cyfry
		response[11] = '0' + (interval_sec / 1000) % 10;
		response[12] = '0' + (interval_sec / 100) % 10;
		response[13] = '0' + (interval_sec / 10) % 10;
		response[14] = '0' + interval_sec % 10;
		response[15] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	} else if (strncmp(cmd, "SET_MODE", 8) == 0) {	
		if (strlen(cmd) < 9) {
			// Zbyt krótka komenda - brak parametru
			send_response_frame(device_addr, src_addr, id, "ERR_INVALID_PARAM");
			return;
		}
		
		// Wyodrębnienie parametru (cyfra po "SET_MODE")
		char mode_char = cmd[8];
		
		// Sprawdzenie czy to cyfra
		if (mode_char < '1' || mode_char > '6') {
			send_response_frame(device_addr, src_addr, id, "ERR_INVALID_PARAM");
			return;
		}
		
		// Konwersja cyfry na numer trybu (1-6)
		uint8_t mode_num = mode_char - '0';
		
		// Mapowanie numeru trybu na wartość hex BH1750
		uint8_t mode_value;
		switch (mode_num) {
			case 1:
				mode_value = BH1750_CONTINUOUS_HIGH_RES_MODE;    // 0x10
				break;
			case 2:
				mode_value = BH1750_CONTINUOUS_HIGH_RES_MODE_2; // 0x11
				break;
			case 3:
				mode_value = BH1750_CONTINUOUS_LOW_RES_MODE;    // 0x13
				break;
			case 4:
				mode_value = BH1750_ONETIME_HIGH_RES_MODE;     // 0x20
				break;
			case 5:
				mode_value = BH1750_ONETIME_HIGH_RES_MODE_2;    // 0x21
				break;
			case 6:
				mode_value = BH1750_ONETIME_LOW_RES_MODE;        // 0x23
				break;
			default:
				send_response_frame(device_addr, src_addr, id, "ERR_INVALID_PARAM");
				return;
		}
		
		// Ustawienie trybu
		HAL_StatusTypeDef status = BH1750_SetMode(mode_value);
		
		if (status == HAL_OK) {
			char response[50];
			// Konkatenacja "OK_SET_MODE" z cyfrą bez snprintf
			memcpy(response, "OK_SET_MODE", 11);
			response[11] = '0' + mode_num;
			response[12] = 0;
			send_response_frame(device_addr, src_addr, id, response);
		} else {
			send_response_frame(device_addr, src_addr, id, "ERR_I2C");
		}
	} else {
		send_response_frame(device_addr, src_addr, id, "ERR_UNKNOWN_CMD");
	}
}

void validate_frame(char *f, uint16_t flen)
{
	const char *device_addr = "PC_";
	char src[4], dst[4], id[3], len_str[4];
	
	/* ====== Sprawdzenie minimalnej długości ramki ====== */
	/* Minimalna ramka: & SRC(3) DST(3) ID(2) LEN(3) CRC(2) * = 13 znaków */
	if(flen < 13)
	{
		// Nie możemy wysłać odpowiedzi, bo nie mamy pełnych danych ramki
		USART_fsend("ERR: TOO SHORT\r\n");
		return;
	}

	/* ====== Sprawdzenie granic ramki ====== */
	if(f[0] != '&' || f[flen-1] != '*')
	{
		// Nie możemy wysłać odpowiedzi, bo nie mamy pełnych danych ramki
		USART_fsend("ERR: BAD BOUNDARY\r\n");
		return;
	}

	/* ====== Parsowanie pól ====== */
	uint16_t pos = 1;  // po '&'

	// Sprawdzenie czy mamy wystarczająco danych do parsowania podstawowych pól
	if(flen < 12) // & + SRC(3) + DST(3) + ID(2) + LEN(3) = 12
	{
		USART_fsend("ERR: TOO SHORT\r\n");
		return;
	}

	memcpy(src, &f[pos], 3); src[3]=0; pos+=3;
	memcpy(dst, &f[pos], 3); dst[3]=0; pos+=3;
	memcpy(id,  &f[pos], 2); id[2]=0;  pos+=2;
	memcpy(len_str, &f[pos], 3); len_str[3]=0; pos+=3;

	/* ====== Sprawdzenie formatu ID (tylko cyfry 0-9) ====== */
	if(!is_digits_only(id, 2))
	{
		send_response_frame(device_addr, src, id, "ERR");
		return;
	}

	/* ====== Sprawdzenie formatu długości (tylko cyfry 0-9) ====== */
	if(!is_digits_only(len_str, 3))
	{
		send_response_frame(device_addr, src, id, "ERR");
		return;
	}

	uint16_t data_len_declared = atoi(len_str);

	/* ====== Sprawdzenie maksymalnej długości ====== */
	if(data_len_declared > 256)
	{
		send_response_frame(device_addr, src, id, "ERR_LENGTH");
		return;
	}

	/* ====== Sprawdzenie faktycznej długości danych ====== */
	uint16_t data_start = pos;
	uint16_t crc_pos = flen - 3;   // 2 znaki CRC + '*'
	
	// Sprawdzenie czy ramka jest wystarczająco długa dla deklarowanej długości danych
	if(flen < (data_start + data_len_declared + 2 + 1)) // data_start + data_len + CRC(2) + '*'
	{
		send_response_frame(device_addr, src, id, "ERR_LENGTH");
		return;
	}
	
	uint16_t data_len_real = crc_pos - data_start;

	/* ====== Sprawdzenie zgodności długości ====== */
	if(data_len_real != data_len_declared)
	{
		send_response_frame(device_addr, src, id, "ERR_LENGTH");
		return;
	}

	/* ====== Odczyt danych ====== */
	char data[257];
	memcpy(data, &f[data_start], data_len_real);
	data[data_len_real] = 0;

	/* ====== Sprawdzenie czy mamy CRC ====== */
	if(flen < (crc_pos + 2))
	{
		send_response_frame(device_addr, src, id, "ERR");
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
		send_response_frame(device_addr, src, id, "ERR_CRC");
		return;
	}

	/* ====== SUKCES - Przetwarzanie komendy ====== */
	USART_fsend("\nFRAME OK\r\n");
	USART_fsend("\nSRC=%s DST=%s ID=%s LEN=%d DATA=%s\r\n",
				src, dst, id, data_len_real, data);

	// Przekazanie informacji o ramce do obsługi komendy
	handle_command(data, src, dst, id);
}

void reset_frame(void)
{
    pos = 0;
    st = ST_IDLE;
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

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		process_uart_buffer();
		Measurement_AutoRead_Process(); // Automatyczny odczyt pomiarów co interwał

//    	while(USART_kbhit()) {
//			int16_t ch = USART_getchar();
//			if(ch != -1) {
//				process_uart_char((char)ch, USART_RX_Busy, USART_RxBuf);
//			}
//		}
//
//		if(USART_RxBufOverflow) {
//			USART_fsend("\r\nERROR: USART RX buffer overflow!\r\n");
//			USART_RxBufOverflow = 0;
//		}
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
		// Reset błędu I2C
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
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
