#ifndef VESC_UART_WRITE_H
#define VESC_UART_WRITE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send a request to switch to a specific profile on the VESC.
 *
 * @param profile_id Profile number to switch to (usually 0–4).
 * @return true if the packet was sent successfully, false otherwise.
 */
bool vesc_set_profile(uint8_t profile_id);

#ifdef __cplusplus
}
#endif

#endif // VESC_UART_WRITE_H
