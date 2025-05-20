#include "vesc_uart.h"
#include "uart_platform.h"
#include "vesc_parser.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#define TAG "VESC_UART"

static vesc_data_t last_data;

bool vesc_uart_poll(vesc_data_t *data) {
    uint8_t cmd_payload[] = {
        50,              // COMM_GET_VALUES_SELECTIVE
        0xFF, 0xFF, 0xFF, 0xFF  // mask: request all fields
    };

    uint16_t crc = 0;
    for (size_t i = 0; i < sizeof(cmd_payload); i++) {
        crc ^= ((uint16_t)cmd_payload[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }

    uint8_t packet[] = {
        2, sizeof(cmd_payload),
        cmd_payload[0], cmd_payload[1], cmd_payload[2], cmd_payload[3], cmd_payload[4],
        (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF), 3
    };

    uart_platform_flush();

    if (!uart_platform_write(packet, sizeof(packet))) {
        ESP_LOGW(TAG, "Failed to write UART packet");
        return false;
    }

    uint8_t buf[128];
    int len = uart_platform_read(buf, sizeof(buf), 100);
    if (len <= 0 || buf[0] != 2) {
        ESP_LOGW(TAG, "Timeout or invalid start byte");
        return false;
    }

    int payload_len = buf[1];
    if ((payload_len + 5) > len) {
        ESP_LOGW(TAG, "Incomplete packet");
        return false;
    }

    uint16_t received_crc = (buf[2 + payload_len] << 8) | buf[2 + payload_len + 1];
    uint16_t computed_crc = 0;
    for (int i = 0; i < payload_len; i++) {
        computed_crc ^= ((uint16_t)buf[2 + i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            computed_crc = (computed_crc & 0x8000) ? (computed_crc << 1) ^ 0x1021 : (computed_crc << 1);
        }
    }

    if (received_crc != computed_crc || buf[2 + payload_len + 2] != 3) {
        ESP_LOGW(TAG, "CRC or end byte error");
        return false;
    }

    const uint8_t *payload = &buf[2];

    ESP_LOG_BUFFER_HEXDUMP(TAG, payload, payload_len, ESP_LOG_INFO);

    if (!vesc_parse_payload(payload, payload_len, data)) {
        ESP_LOGW(TAG, "Failed to parse VESC payload");
        return false;
    }

    last_data = *data;
    return true;
}

void vesc_uart_init(int tx_pin, int rx_pin) {
    if (!uart_platform_init(tx_pin, rx_pin, 115200)) {
        ESP_LOGE(TAG, "UART init failed");
    }
}

void vesc_data_set(const vesc_data_t* src) {
    last_data = *src;
}

bool vesc_data_get(vesc_data_t* dst) {
    if (dst) {
        *dst = last_data;
        return true;
    }
    return false;
}

bool vesc_read_packet(uint8_t *payload_out, size_t *payload_len) {
    // Not used in this implementation
    return false;
}
