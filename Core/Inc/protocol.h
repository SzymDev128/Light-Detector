/**
 ******************************************************************************
 * @file           : protocol.h
 * @brief          : Communication protocol frame parser and command handler
 ******************************************************************************
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Frame Parser Functions */
void process_uart_buffer(void);
void validate_frame(char *f, uint16_t flen);
void handle_command(char *cmd, const char *src_addr, const char *dst_addr, const char *id);

/* Helper Functions */
uint8_t is_digits_only(const char *str, uint16_t len);
uint8_t is_addr_char_valid(char c);
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
