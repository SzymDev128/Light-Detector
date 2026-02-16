/**
 ******************************************************************************
 * @file           : crc8.c
 * @brief          : CRC-8 calculation and hex conversion utilities
 ******************************************************************************
 */

#include "crc8.h"

/**
 * @brief Check if character is valid hex (0-9, A-F, a-f)
 * @param c: Character to check
 * @retval 1 if valid hex, 0 otherwise
 */
uint8_t is_hex_char(char c) {
	return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

/**
 * @brief Convert two hex characters to byte
 * @param hi: High nibble character
 * @param lo: Low nibble character
 * @retval Converted byte value
 */
uint8_t hex2byte(char hi, char lo) {
	uint8_t high = (hi >= '0' && hi <= '9') ? hi - '0' :
					(hi >= 'A' && hi <= 'F') ? hi - 'A' + 10 :
					(hi >= 'a' && hi <= 'f') ? hi - 'a' + 10 : 0;
	uint8_t low = (lo >= '0' && lo <= '9') ? lo - '0' :
					(lo >= 'A' && lo <= 'F') ? lo - 'A' + 10 :
					(lo >= 'a' && lo <= 'f') ? lo - 'a' + 10 : 0;
	return (high << 4) | low;
}

/**
 * @brief Calculate CRC-8 (polynomial 0x07)
 * @param data: Pointer to data buffer
 * @param len: Length of data
 * @retval CRC-8 value
 */
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

/**
 * @brief Convert byte to two hex characters
 * @param byte: Byte to convert
 * @param hex: Output buffer (must be at least 2 bytes)
 */
void byte2hex(uint8_t byte, char *hex) {
	static const char hex_chars[] = "0123456789ABCDEF";
	hex[0] = hex_chars[(byte >> 4) & 0x0F];
	hex[1] = hex_chars[byte & 0x0F];
}
