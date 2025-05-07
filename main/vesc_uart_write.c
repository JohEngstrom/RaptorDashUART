#include "vesc_uart_write.h"
#include "uart_platform.h"
#include "crc.h"

#define COMM_SELECT_PROFILE 0x4F

bool vesc_set_profile(uint8_t profile_id) {
    uint8_t payload[] = { COMM_SELECT_PROFILE, profile_id };
    uint16_t crc = crc16(payload, sizeof(payload));

    uint8_t packet[] = {
        2,                  // Start byte
        sizeof(payload),   // Payload length
        payload[0],        // Command ID
        payload[1],        // Profile ID
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF),
        3                  // End byte
    };

    uart_platform_flush();
    return uart_platform_write(packet, sizeof(packet));
}
