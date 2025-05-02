#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vesc_uart.h"

#define VESC_RX 6
#define VESC_TX 14
#define TAG "APP"

// Optional: lower polling interval if needed (200–1000ms is typical)
#define POLL_INTERVAL_MS 1000

void vesc_polling_task(void *arg) {
    vesc_data_t data;
    int failure_count = 0;

    ESP_LOGI(TAG, "Starting VESC polling task...");

    while (1) {
        if (vesc_uart_poll(&data)) {
            failure_count = 0;

            // Save the latest data for other tasks (like LVGL)
            vesc_data_set(&data);

            ESP_LOGI(TAG, "Voltage: %.2f V | Current: %.2f A | Input: %.2f A | Speed: %.2f km/h | RPM: %.0f",
                     data.voltage, data.current, data.input_current, data.speed_kmh, data.rpm);
            ESP_LOGI(TAG, "Duty: %.1f %% | SoC: %.1f %% | MOSFET: %.1f °C | Motor: %.1f °C",
                     data.duty_cycle_percent, data.battery_percentage, data.mosfet_temp, data.motor_temp);
            ESP_LOGI(TAG, "Wh: %.2f | Ah: %.2f | Peak: %.1f W | Est. Range: %.2f km",
                     data.watt_hours, data.amp_hours, data.peak_watts, data.estimated_range_km);
            ESP_LOGI(TAG, "Distance: Session = %.2f km | Total = %.2f km\n",
                     data.distance_session_km, data.distance_total_km);
        } else {
            failure_count++;
            ESP_LOGW(TAG, "Failed to poll VESC (%d)", failure_count);
            if (failure_count >= 5) {
                ESP_LOGE(TAG, "Too many failed polls. Check UART wiring or VESC power.");
                // Optional: take action (LED blink, system restart, alert, etc.)
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500)); // optional startup delay
    ESP_LOGI(TAG, "Initializing VESC UART on TX=%d, RX=%d", VESC_TX, VESC_RX);

    vesc_uart_init(VESC_TX, VESC_RX);

    xTaskCreatePinnedToCore(
        vesc_polling_task,
        "vesc_poll_task",
        4096,
        NULL,
        5,
        NULL,
        APP_CPU_NUM  // or tskNO_AFFINITY if not specific
    );
}
