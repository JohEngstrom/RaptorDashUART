// vesc_uart.c
#include "vesc_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

#define UART_PORT UART_NUM_1
#define BUF_SIZE 128
#define TAG "VESC_UART"

static vesc_data_t last_data;
static float smoothed_watts = 0;
static float peak_watts = 0;
static float wheel_circum_mm = 890.0f;
static int motor_pole_pairs = 15;
static float battery_min_v = 48.0f;
static float battery_max_v = 67.2f;
static float battery_wh = 960.0f;
static int32_t last_tachometer = -1;
static float session_km = 0;

static uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

void vesc_uart_init(int tx_pin, int rx_pin) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static float read_float16(const uint8_t *data, float scale, int *index) {
    int16_t raw = ((int16_t)data[*index] << 8) | data[*index + 1];
    *index += 2;
    return raw / scale;
}

static float read_float32(const uint8_t *data, float scale, int *index) {
    int32_t raw = ((int32_t)data[*index] << 24) |
                  ((int32_t)data[*index + 1] << 16) |
                  ((int32_t)data[*index + 2] << 8) |
                  data[*index + 3];
    *index += 4;
    return raw / scale;
}

static int32_t read_int32(const uint8_t *data, int *index) {
    int32_t val = ((int32_t)data[*index] << 24) |
                  ((int32_t)data[*index + 1] << 16) |
                  ((int32_t)data[*index + 2] << 8) |
                  data[*index + 3];
    *index += 4;
    return val;
}

bool vesc_uart_poll(vesc_data_t *data) {
    uint8_t cmd_payload[] = {
        50,              // COMM_GET_VALUES_SELECTIVE
        0xFF, 0xFF, 0xFF, 0xFF  // mask: request all fields
    };

    uint16_t crc = crc16(cmd_payload, sizeof(cmd_payload));

    uint8_t packet[] = {
        2, sizeof(cmd_payload),
        cmd_payload[0], cmd_payload[1], cmd_payload[2], cmd_payload[3], cmd_payload[4],
        (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF), 3
    };

    uart_flush(UART_PORT);

    ESP_LOGI(TAG, "Sending UART packet:");
    for (int k = 0; k < sizeof(packet); k++) {
        printf("%02X ", packet[k]);
    }
    printf("\n");

    uart_write_bytes(UART_PORT, (const char *)packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t buf[128];
    int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(100));
    if (len <= 0 || buf[0] != 2) {
        ESP_LOGW(TAG, "Timeout or invalid start byte");
        return false;
    }

    int payload_len = buf[1];
    if (payload_len + 5 > len) {
        ESP_LOGW(TAG, "Incomplete packet");
        return false;
    }

    uint16_t received_crc = (buf[2 + payload_len] << 8) | buf[2 + payload_len + 1];
    uint16_t computed_crc = crc16(&buf[2], payload_len);

    if (received_crc != computed_crc || buf[2 + payload_len + 2] != 3) {
        ESP_LOGW(TAG, "CRC or end byte error");
        return false;
    }

    const uint8_t *payload = &buf[2];

    ESP_LOGI(TAG, "Received payload (len = %d):", payload_len);
    for (int j = 0; j < payload_len; j++) {
        printf("%02X ", payload[j]);
    }
    printf("\n");

    if (payload_len < 40) {
        ESP_LOGW(TAG, "Payload too short");
        return false;
    }

    int i = 5; // Skip packet ID and 4-byte mask
    data->mosfet_temp         = read_float16(payload, 10.0f, &i);
    data->motor_temp          = read_float16(payload, 10.0f, &i);
    data->current             = read_float32(payload, 100.0f, &i);
    data->input_current       = read_float32(payload, 100.0f, &i);
    read_float32(payload, 100.0f, &i); // skip id
    read_float32(payload, 100.0f, &i); // skip iq
    data->duty_cycle_percent  = read_float16(payload, 1000.0f, &i);
    data->rpm                 = read_int32(payload, &i);
    data->voltage             = read_float16(payload, 10.0f, &i);
    data->amp_hours           = read_float32(payload, 10000.0f, &i);
    data->amp_hours_charged   = read_float32(payload, 10000.0f, &i);
    data->watt_hours          = read_float32(payload, 10000.0f, &i);
    data->watt_hours_charged  = read_float32(payload, 10000.0f, &i);
    int32_t tach              = read_int32(payload, &i);
    int32_t tach_abs          = read_int32(payload, &i);

    data->charge_cycles = ((uint16_t)payload[i] << 8) | payload[i + 1];
    i += 2;
    data->charged_ah_total = (((uint16_t)payload[i] << 8) | payload[i + 1]) / 100.0f;
    i += 2;

    float watts_now = data->voltage * data->current;
    smoothed_watts += (watts_now - smoothed_watts) / 5.0f;
    if (watts_now > peak_watts) peak_watts = watts_now;

    data->watts = watts_now;
    data->avg_watts = smoothed_watts;
    data->peak_watts = peak_watts;

    data->battery_percentage = 100.0f * (data->voltage - battery_min_v) / (battery_max_v - battery_min_v);
    data->battery_percentage = fmaxf(0, fminf(100, data->battery_percentage));

    float mech_rpm = data->rpm / (float)motor_pole_pairs;
    data->speed_kmh = (mech_rpm * wheel_circum_mm * 60.0f) / 1e6f;

    if (last_tachometer >= 0) {
      int32_t delta = tach - last_tachometer;
      float revs = delta / (float)motor_pole_pairs;
      session_km += (revs * wheel_circum_mm) / 1e6f;
    }
    last_tachometer = tach;

    data->distance_session_km = session_km;
    data->distance_total_km = (tach_abs / (float)motor_pole_pairs) * wheel_circum_mm / 1e6f;

    if (smoothed_watts > 1.0f) {
        float hours_left = battery_wh * (data->battery_percentage / 100.0f) / smoothed_watts;
        data->estimated_range_km = hours_left * data->speed_kmh;
    } else {
        data->estimated_range_km = 0;
    }

    last_data = *data;
    return true;
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
    return false;
}
