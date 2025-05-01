
#include <math.h>
#include "vesc_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 128
#define POLL_INTERVAL 1000

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

bool vesc_uart_poll(vesc_data_t* data) {
    uint8_t request[] = {2, 4, 4 ^ 2, 3};
    uart_write_bytes(UART_PORT, (const char*)request, sizeof(request));

    uint8_t d[BUF_SIZE];
    int len = 0;
    uint32_t start = xTaskGetTickCount();
    while (len < 55 && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(POLL_INTERVAL)) {
        int read = uart_read_bytes(UART_PORT, d + len, BUF_SIZE - len, pdMS_TO_TICKS(100));
        if (read > 0) len += read;
    }

    if (len < 55) return false;

    int32_t erpm = (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3];
    data->current = ((int16_t)((d[4] << 8) | d[5])) / 10.0f;
    data->voltage = ((uint16_t)((d[6] << 8) | d[7])) / 10.0f;
    data->mosfet_temp = ((int16_t)((d[8] << 8) | d[9])) / 10.0f;
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

    int32_t tachometer = (d[52] << 24) | (d[53] << 16) | (d[54] << 8) | d[55];
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

    return true;
}
