#ifndef UART_PLATFORM_H
#define UART_PLATFORM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART with given TX, RX pins and baud rate.
 */
bool uart_platform_init(int tx_pin, int rx_pin, int baud_rate);

/**
 * @brief Write data to UART.
 */
bool uart_platform_write(const uint8_t *data, size_t len);

/**
 * @brief Read data from UART with timeout.
 */
int uart_platform_read(uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Flush UART input buffer.
 */
void uart_platform_flush(void);

#ifdef __cplusplus
}
#endif

#endif
