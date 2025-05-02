
#include "crc16.h"
#include <math.h>
#include "vesc_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/semphr.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 128
#define POLL_INTERVAL 1000

static int16_t read_int16(const uint8_t *data, int *index);
static uint16_t read_uint16(const uint8_t *data, int *index);
static int32_t read_int32(const uint8_t *data, int *index);

static const char *TAG = "VESC_UART";
static float smoothed_watts = 0;
static float peak_watts = 0;
static float wheel_circum_mm = 890.0f;
static int motor_pole_pairs = 15;
static float battery_min_v = 48.0f;
static float battery_max_v = 67.2f;
static float battery_wh = 960.0f;
static int32_t last_tachometer = -1;
static float session_km = 0;

static vesc_data_t latest_data;
static SemaphoreHandle_t data_mutex = NULL;

void vesc_data_set(const vesc_data_t* src) {
    if (data_mutex == NULL) return;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        latest_data = *src;
        xSemaphoreGive(data_mutex);
    }
}

bool vesc_data_get(vesc_data_t* dst) {
    if (data_mutex == NULL) return false;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        *dst = latest_data;
        xSemaphoreGive(data_mutex);
        return true;
    }
    return false;
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

    if (data_mutex == NULL) {
      data_mutex = xSemaphoreCreateMutex();
    }
}

bool vesc_read_packet(uint8_t *payload_out, size_t *payload_len) {
    uint8_t buffer[128];

    // Wait for start byte
    while (true) {
        int read = uart_read_bytes(UART_PORT, buffer, 1, pdMS_TO_TICKS(100));
        if (read == 1 && buffer[0] == 2) break;
    }

    // Read length
    int read = uart_read_bytes(UART_PORT, buffer + 1, 1, pdMS_TO_TICKS(100));
    if (read != 1) return false;
    uint8_t len = buffer[1];

    // Read payload + CRC (2 bytes) + end byte
    int needed = len + 2 + 1;
    read = uart_read_bytes(UART_PORT, buffer + 2, needed, pdMS_TO_TICKS(200));
    if (read != needed) return false;

    // Verify end byte
    if (buffer[2 + len + 2] != 3) return false;

    // Check CRC
    uint16_t received_crc = ((uint16_t)buffer[2 + len] << 8) | buffer[2 + len + 1];
    uint16_t computed_crc = crc16(buffer + 2, len);

    if (received_crc != computed_crc) return false;

    // Copy payload out
    memcpy(payload_out, buffer + 2, len);
    *payload_len = len;
    return true;
}

void vesc_send_command(uint8_t command) {
    uint8_t payload[1] = {command};
    uint16_t crc = crc16(payload, 1);

    uint8_t packet[6];
    packet[0] = 2;              // Start byte (short packet)
    packet[1] = 1;              // Payload length
    packet[2] = command;        // Payload: the command
    packet[3] = (crc >> 8) & 0xFF;  // CRC High byte
    packet[4] = crc & 0xFF;         // CRC Low byte
    packet[5] = 3;              // End byte

    uart_write_bytes(UART_PORT, (const char *)packet, sizeof(packet));
}

bool vesc_uart_poll(vesc_data_t* data) {
    vesc_send_command(4); // COMM_GET_VALUES

    uint8_t payload[64];
    size_t payload_len = 0;
    if (!vesc_read_packet(payload, &payload_len)) return false;

    int i = 0;

    int32_t erpm = read_int32(payload, &i);
    int16_t motor_current = read_int16(payload, &i);
    int16_t input_current = read_int16(payload, &i);
    int16_t duty_cycle = read_int16(payload, &i);
    uint16_t voltage_raw = read_uint16(payload, &i);
    int16_t temp_fet = read_int16(payload, &i);
    int16_t temp_motor = read_int16(payload, &i);
    i += 2 * 4; // skip current_in, pid_pos, id, iq

    int32_t tachometer = read_int32(payload, &i);
    i += 4; // skip tachometer_abs

    int32_t wh_consumed = read_int32(payload, &i);
    int32_t wh_charged = read_int32(payload, &i);
    int32_t ah_consumed = read_int32(payload, &i);
    int32_t ah_charged = read_int32(payload, &i);

    uint16_t charge_cycles = read_uint16(payload, &i);
    uint16_t charged_ah_total_raw = read_uint16(payload, &i);

    // Assign core values
    data->current = motor_current / 10.0f;
    data->input_current = input_current / 10.0f;
    data->duty_cycle_percent = duty_cycle / 10.0f;
    data->voltage = voltage_raw / 10.0f;
    data->mosfet_temp = temp_fet / 10.0f;
    data->motor_temp = temp_motor / 10.0f;

    data->watt_hours = wh_consumed / 1000.0f;
    data->watt_hours_charged = wh_charged / 1000.0f;
    data->amp_hours = ah_consumed / 1000.0f;
    data->amp_hours_charged = ah_charged / 1000.0f;
    data->charge_cycles = charge_cycles;
    data->charged_ah_total = charged_ah_total_raw / 100.0f;

    // Derived/calculated metrics
    float watts_now = data->voltage * data->current;
    smoothed_watts += (watts_now - smoothed_watts) / 5.0f;
    if (watts_now > peak_watts) peak_watts = watts_now;
    data->watts = watts_now;
    data->avg_watts = smoothed_watts;
    data->peak_watts = peak_watts;

    data->battery_percentage = 100.0f * (data->voltage - battery_min_v) / (battery_max_v - battery_min_v);
    data->battery_percentage = fmaxf(0, fminf(100, data->battery_percentage));

    data->rpm = erpm / motor_pole_pairs;
    data->speed_kmh = data->rpm * wheel_circum_mm * 60.0f / 1e6f;

    if (last_tachometer >= 0) {
        int32_t delta = tachometer - last_tachometer;
        float revs = delta / (float)motor_pole_pairs;
        session_km += (revs * wheel_circum_mm) / 1e6f;
    }
    last_tachometer = tachometer;

    data->distance_session_km = session_km;
    float total_revs = tachometer / (float)motor_pole_pairs;
    data->distance_total_km = (total_revs * wheel_circum_mm) / 1e6f;

    if (smoothed_watts > 1.0f) {
        float hours_left = battery_wh * (data->battery_percentage / 100.0f) / smoothed_watts;
        data->estimated_range_km = hours_left * data->speed_kmh;
    } else {
        data->estimated_range_km = 0;
    }

    ESP_LOGI(TAG, "V=%.1fV I=%.1fA Iin=%.1fA RPM=%d Speed=%.1f km/h Duty=%.1f%% SoC=%.1f%% Ah=%.2f Wh=%.2f Tmotor=%.1f°C",
        data->voltage, data->current, data->input_current,
        (int)data->rpm, data->speed_kmh, data->duty_cycle_percent,
        data->battery_percentage, data->amp_hours, data->watt_hours, data->motor_temp);

    return true;
}

static int16_t read_int16(const uint8_t *data, int *index) {
    int16_t value = ((int16_t)data[*index] << 8) | data[*index + 1];
    *index += 2;
    return value;
}

static uint16_t read_uint16(const uint8_t *data, int *index) {
    uint16_t value = ((uint16_t)data[*index] << 8) | data[*index + 1];
    *index += 2;
    return value;
}

static int32_t read_int32(const uint8_t *data, int *index) {
    int32_t value = ((int32_t)data[*index] << 24) |
                    ((int32_t)data[*index + 1] << 16) |
                    ((int32_t)data[*index + 2] << 8) |
                    data[*index + 3];
    *index += 4;
    return value;
}
