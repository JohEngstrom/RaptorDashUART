#include "uart_platform.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT UART_NUM_1
#define TAG "UART_PLATFORM"

bool uart_platform_init(int tx_pin, int rx_pin, int baud_rate) {
    uart_config_t config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    if (uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return false;
    }

    if (uart_param_config(UART_PORT, &config) != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return false;
    }

    if (uart_set_pin(UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed");
        return false;
    }

    return true;
}

bool uart_platform_write(const uint8_t *data, size_t len) {
    int written = uart_write_bytes(UART_PORT, (const char *)data, len);
    return (written == len);
}

int uart_platform_read(uint8_t *buf, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_PORT, buf, max_len, pdMS_TO_TICKS(timeout_ms));
}

void uart_platform_flush(void) {
    uart_flush(UART_PORT);
}
