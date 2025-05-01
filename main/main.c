
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vesc_uart.h"

#define VESC_RX 6
#define VESC_TX 14

void app_main(void) {
    vesc_uart_init(VESC_TX, VESC_RX);
    vesc_data_t data;

    while (1) {
        if (vesc_uart_poll(&data)) {
            printf("\n==== VESC Telemetry ====");
            printf("\nVoltage:           %.2f V", data.voltage);
            printf("\nCurrent:           %.2f A", data.current);
            printf("\nWatts:             %.1f W", data.watts);
            printf("\nAvg Power:         %.1f W", data.avg_watts);
            printf("\nPeak Power:        %.1f W", data.peak_watts);
            printf("\nBattery %%:         %.1f %%", data.battery_percentage);
            printf("\nMOSFET Temp:       %.1f °C", data.mosfet_temp);
            printf("\nRPM:               %.0f", data.rpm);
            printf("\nSpeed:             %.2f km/h", data.speed_kmh);
            printf("\nDistance (session):%.2f km", data.distance_session_km);
            printf("\nDistance (total):  %.2f km", data.distance_total_km);
            printf("\nEstimated Range:   %.2f km\n", data.estimated_range_km);
        } else {
            printf("Failed to poll VESC.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
