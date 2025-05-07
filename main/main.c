#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vesc_uart.h"
#include "vesc_uart_write.h"
#include "esp_log.h"

#define VESC_TX 43
#define VESC_RX 44

#define TAG "VESC_TEST"

void app_main(void) {
    ESP_LOGI(TAG, "Initializing VESC UART on TX=%d, RX=%d", VESC_TX, VESC_RX);
    vesc_uart_init(VESC_TX, VESC_RX);
    vesc_data_t data;

    while (true) {
        if (vesc_uart_poll(&data)) {
            printf("\n==== VESC Telemetry ====\n");
            printf("Voltage:           %.2f V\n", data.voltage);
            printf("Current:           %.2f A\n", data.current);
            printf("Watts:             %.1f W\n", data.watts);
            printf("Avg Power:         %.1f W\n", data.avg_watts);
            printf("Peak Power:        %.1f W\n", data.peak_watts);
            printf("Battery %%:         %.1f %%\n", data.battery_percentage);
            printf("MOSFET Temp:       %.1f °C\n", data.mosfet_temp);
            printf("Motor Temp:        %.1f °C\n", data.motor_temp);
            printf("RPM:               %.0f\n", data.rpm);
            printf("Speed:             %.2f km/h\n", data.speed_kmh);
            printf("Session Distance:  %.2f km\n", data.distance_session_km);
            printf("Total Distance:    %.2f km\n", data.distance_total_km);
            printf("Est. Range:        %.2f km\n", data.estimated_range_km);
            printf("Input Current:     %.2f A\n", data.input_current);
            printf("Duty Cycle:        %.1f %%\n", data.duty_cycle_percent);
            printf("Ah Consumed:       %.2f Ah\n", data.amp_hours);
            printf("Ah Charged:        %.2f Ah\n", data.amp_hours_charged);
            printf("Wh Consumed:       %.2f Wh\n", data.watt_hours);
            printf("Wh Charged:        %.2f Wh\n", data.watt_hours_charged);
            printf("Charge Cycles:     %d\n", data.charge_cycles);
            printf("Total Charged Ah:  %.2f Ah\n", data.charged_ah_total);
        } else {
            ESP_LOGW(TAG, "Failed to poll VESC");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
