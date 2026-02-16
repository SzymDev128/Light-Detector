/**
 ******************************************************************************
 * @file           : crc8.h
 * @brief          : CRC-8 calculation and hex conversion utilities
 ******************************************************************************
 */

#ifndef __CRC8_H
#define __CRC8_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* CRC Functions */
uint8_t crc8(uint8_t *data, uint16_t len);

/* Hex Conversion Functions */
uint8_t hex2byte(char hi, char lo);
void byte2hex(uint8_t byte, char *hex);
uint8_t is_hex_char(char c);

#ifdef __cplusplus
}
#endif

#endif /* __CRC8_H */
