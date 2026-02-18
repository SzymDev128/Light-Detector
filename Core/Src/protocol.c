/**
 ******************************************************************************
 * @file           : protocol.c
 * @brief          : Communication protocol frame parser and command handler
 ******************************************************************************
 */

#include "protocol.h"
#include "circular_buffer.h"
#include "crc8.h"
#include "bh1750.h"
#include <string.h>
#include <stdlib.h>

/* Frame Parser State */
typedef enum {
    ST_IDLE = 0,
    ST_COLLECT
} frame_state_t;

static frame_state_t st = ST_IDLE;
static char frame[300];
static uint16_t pos = 0;

/**
 * @brief Check if string contains only digits
 */
uint8_t is_digits_only(const char *str, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		if (str[i] < '0' || str[i] > '9') {
			return 0;
		}
	}
	return 1;
}

/**
 * @brief Check if character is valid for address field
 */
uint8_t is_addr_char_valid(char c) {
	return (c >= 0x21 && c <= 0x7E && c != '&' && c != '*');
}

/**
 * @brief Send response frame
 */
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data) {
	char frame[271];
	uint16_t pos = 0;
	uint8_t crc_buf[300];
	uint16_t crc_pos = 0;

	uint16_t data_len = strlen(data);
	if (data_len > 256) {
		return;
	}

	uint16_t total_len = 1 + 3 + 3 + 2 + 3 + data_len + 2 + 1 + 1;
	if (total_len > sizeof(frame)) {
		// Przekroczenie bufora ramki
		return;
	}

	frame[pos++] = '&';

	memcpy(&frame[pos], src_addr, 3);
	pos += 3;
	memcpy(&crc_buf[crc_pos], src_addr, 3);
	crc_pos += 3;

	memcpy(&frame[pos], dst_addr, 3);
	pos += 3;
	memcpy(&crc_buf[crc_pos], dst_addr, 3);
	crc_pos += 3;

	memcpy(&frame[pos], id, 2);
	pos += 2;
	memcpy(&crc_buf[crc_pos], id, 2);
	crc_pos += 2;

	char len_str[4];
	len_str[0] = '0' + (data_len / 100) % 10;
	len_str[1] = '0' + (data_len / 10) % 10;
	len_str[2] = '0' + data_len % 10;
	len_str[3] = 0;
	memcpy(&frame[pos], len_str, 3);
	pos += 3;
	memcpy(&crc_buf[crc_pos], len_str, 3);
	crc_pos += 3;

	memcpy(&frame[pos], data, data_len);
	pos += data_len;
	memcpy(&crc_buf[crc_pos], data, data_len);
	crc_pos += data_len;

	uint8_t crc = crc8(crc_buf, crc_pos);
	char crc_hex[3];
	byte2hex(crc, crc_hex);
	crc_hex[2] = 0;

	memcpy(&frame[pos], crc_hex, 2);
	pos += 2;

	frame[pos++] = '*';
	frame[pos] = 0;

	USART_fsend("%s", frame);
}

/**
 * @brief Handle received command
 */
void handle_command(char *cmd, const char *src_addr, const char *dst_addr, const char *id) {
	const char *device_addr = dst_addr;

	const uint16_t MIN_INTERVAL = 1;
	const uint16_t MAX_INTERVAL = 9999;

	if (strlen(cmd) < 2 || !is_digits_only(cmd, 2)) {
		USART_fsend("ERR: INVALID CMD\r\n");
		return;
	}

	uint8_t cmd_code = (cmd[0] - '0') * 10 + (cmd[1] - '0');
	const char *params = (strlen(cmd) > 2) ? &cmd[2] : "";
	
	// 10 - START
	if (cmd_code == 10) {
		Measurement_EnableAutoRead(1);
		send_response_frame(device_addr, src_addr, id, "00");
	}
	// 11 - STOP
	else if (cmd_code == 11) {
		Measurement_EnableAutoRead(0);
		send_response_frame(device_addr, src_addr, id, "00");
	}
	// 12 - DOWNLOAD (last measurement)
	else if (cmd_code == 12) {
		uint16_t count = (uint16_t)LightBuffer_GetCount();
		if (count == 0) {
			send_response_frame(device_addr, src_addr, id, "03");
			return;
		}
		measurement_entry_t *entry = LightBuffer_GetLatest();

		uint32_t lux_val = (uint32_t)(entry->lux + 0.5f);
		if (lux_val > 65535) {
			lux_val = 65535;
		}
		char response[6];
		response[0] = '0' + (lux_val / 10000) % 10;
		response[1] = '0' + (lux_val / 1000) % 10;
		response[2] = '0' + (lux_val / 100) % 10;
		response[3] = '0' + (lux_val / 10) % 10;
		response[4] = '0' + lux_val % 10;
		response[5] = 0;
		send_response_frame(device_addr, src_addr, id, response);
	}
	// 13 - VIEW (+ xxxzzz parameters)
	else if (cmd_code == 13) {
		if (strlen(params) < 6) {
			send_response_frame(device_addr, src_addr, id, "01");
			return;
		}
		uint16_t start_offset = (params[0] - '0') * 100 + (params[1] - '0') * 10 + (params[2] - '0');
		uint16_t count_req = (params[3] - '0') * 100 + (params[4] - '0') * 10 + (params[5] - '0');
		uint16_t count = (uint16_t)LightBuffer_GetCount();
		if (start_offset >= count) {
			send_response_frame(device_addr, src_addr, id, "03");
			return;
		}
		// pobranie wszystkich danych z bufora dla '000000'
		if (start_offset == 0 && count_req == 0) {
			count_req = count;
		} else if (count_req == 0) {
			count_req = count - start_offset;
		}
		
		#define MAX_MEASUREMENTS_PER_FRAME 50
		char data_out[256];
		uint16_t measurements_sent = 0;
		
		while (measurements_sent < count_req) {
			uint16_t current_offset = start_offset + measurements_sent;
			uint16_t batch_size = 0;
			uint16_t data_pos = 0;
			
			data_out[data_pos++] = '0' + (current_offset / 100) % 10;
			data_out[data_pos++] = '0' + (current_offset / 10) % 10;
			data_out[data_pos++] = '0' + current_offset % 10;
			
			for (uint16_t i = 0; i < MAX_MEASUREMENTS_PER_FRAME && measurements_sent < count_req; i++) {
				int32_t idx = (int32_t)count - 1 - (int32_t)current_offset - (int32_t)i;
				if (idx < 0) {
					break;
				}
				measurement_entry_t *entry = LightBuffer_GetByIndexOldest((uint16_t)idx);
				if (!entry) {
					break;
				}
				uint32_t lux_val = (uint32_t)(entry->lux + 0.5f);
				if (lux_val > 65535) {
					lux_val = 65535;
				}
				data_out[data_pos++] = '0' + (lux_val / 10000) % 10;
				data_out[data_pos++] = '0' + (lux_val / 1000) % 10;
				data_out[data_pos++] = '0' + (lux_val / 100) % 10;
				data_out[data_pos++] = '0' + (lux_val / 10) % 10;
				data_out[data_pos++] = '0' + lux_val % 10;
				batch_size++;
				measurements_sent++;
			}
			
			data_out[data_pos] = 0;
			send_response_frame(device_addr, src_addr, id, data_out);
			
			if (batch_size == 0) {
				break;
			}
		}
	}
	// 14 - SET_INTERVAL (+ xxxx parameter)
	else if (cmd_code == 14) {
		if (strlen(params) < 4) {
			send_response_frame(device_addr, src_addr, id, "01");
			return;
		}
		uint16_t interval_ms = atoi(params);
		if (interval_ms < MIN_INTERVAL || interval_ms > MAX_INTERVAL) {
			send_response_frame(device_addr, src_addr, id, "02");
			return;
		}
		Measurement_SetInterval(interval_ms);
		send_response_frame(device_addr, src_addr, id, "00");
	}
	// 15 - GET_INTERVAL
	else if (cmd_code == 15) {
		uint32_t interval_ms = Measurement_GetInterval();
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
	// 16 - SET_MODE (+ x parameter)
	else if (cmd_code == 16) {
		if (strlen(params) < 1 || params[0] < '1' || params[0] > '6') {
			send_response_frame(device_addr, src_addr, id, "01");
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
		}
		HAL_StatusTypeDef status = BH1750_SetMode(mode_value);
		if (status == HAL_OK) {
			send_response_frame(device_addr, src_addr, id, "00");
		} else {
			send_response_frame(device_addr, src_addr, id, "04");
		}
	}
	// 17 - GET_MODE
	else if (cmd_code == 17) {
		char mode_char = '1';
		uint8_t current_mode = BH1750_GetCurrentMode();
		switch (current_mode) {
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
	else {
		USART_fsend("ERR: UNKNOWN CMD\r\n");
	}
}

/**
 * @brief Validate received frame
 */
void validate_frame(char *f, uint16_t flen) {
	char src[4], dst[4], id[3], len_str[4];

	if (flen < 15) {
		USART_fsend("ERR: TOO SHORT\r\n");
		return;
	}

	uint16_t pos = 1;
	memcpy(src, &f[pos], 3); src[3] = 0; pos += 3;
	memcpy(dst, &f[pos], 3); dst[3] = 0; pos += 3;
	memcpy(id,  &f[pos], 2); id[2] = 0;  pos += 2;
	memcpy(len_str, &f[pos], 3); len_str[3] = 0; pos += 3;

	// Odrzucaj ramki nie skierowane do 'STM'
	if (strcmp(dst, "STM") != 0) {
		return;
	}

	for (uint8_t i = 0; i < 3; i++) {
		if (!is_addr_char_valid(src[i]) || !is_addr_char_valid(dst[i])) {
			USART_fsend("ERR: INVALID ADDR\r\n");
			return;
		}
	}

	if (!is_digits_only(id, 2)) {
		USART_fsend("ERR: INVALID ID\r\n");
		return;
	}

	if (!is_digits_only(len_str, 3)) {
		USART_fsend("ERR\r\n");
		return;
	}

	uint16_t data_len_declared = atoi(len_str);

	if (data_len_declared > 256) {
		USART_fsend("ERR_LENGTH\r\n");
		return;
	}
	
	// Sprawdzenie czy rzeczywista długość pola DATA odpowiada zadeklarowanej (LEN)
	uint16_t actual_data_len = flen - pos - 3;
	
	if (actual_data_len != data_len_declared) {
		USART_fsend("ERR: DATA LENGTH MISMATCH\r\n");
		return;
	}

	uint16_t crc_pos = pos + data_len_declared;

	char data[257];
	memcpy(data, &f[pos], data_len_declared);
	data[data_len_declared] = 0;

	if (!is_digits_only(data, data_len_declared)) {
		USART_fsend("ERR\r\n");
		return;
	}

	if (!is_hex_char(f[crc_pos]) || !is_hex_char(f[crc_pos + 1])) {
		USART_fsend("ERR\r\n");
		return;
	}

	uint8_t rx_crc = hex2byte(f[crc_pos], f[crc_pos + 1]);

	uint8_t buf[270];
	uint16_t p = 0;

	memcpy(&buf[p], src, 3); p += 3;
	memcpy(&buf[p], dst, 3); p += 3;
	memcpy(&buf[p], id, 2);  p += 2;
	memcpy(&buf[p], len_str, 3); p += 3;
	memcpy(&buf[p], data, data_len_declared); p += data_len_declared;

	uint8_t calc_crc = crc8(buf, p);

	if (calc_crc != rx_crc) {
		USART_fsend("ERR_CRC\r\n");
		return;
	}
	
	handle_command(data, src, dst, id);
}

/**
 * @brief Process UART buffer and extract frames
 */
void process_uart_buffer(void) {
	while (USART_kbhit()) {
		char c = USART_getchar();

		switch (st) {
			case ST_IDLE: {
				if (c == '&') {
					pos = 0;
					frame[pos++] = c;
					st = ST_COLLECT;
				}
				break;
			}

			case ST_COLLECT: {
				if (c == '&') {
					pos = 0;
					frame[pos++] = c;
					break;
				}

				if (pos < sizeof(frame) - 1) {
					frame[pos++] = c;
				} else {
					// Buffer overflow: odrzucamy ramkę i wracamy do ST_IDLE
					pos = 0;
					st = ST_IDLE;
					break;
				}

				if (c == '*') {
					frame[pos] = 0;
					validate_frame(frame, pos);
					st = ST_IDLE;
				}
				break;
			}
		}
	}
}
