
#ifndef VESC_UART_H
#define VESC_UART_H

#include <stdbool.h>

typedef struct {
    float voltage;
    float current;
    float watts;
    float avg_watts;
    float peak_watts;
    float battery_percentage;
    float mosfet_temp;
    float rpm;
    float speed_kmh;
    float distance_session_km;
    float distance_total_km;
    float estimated_range_km;
} vesc_data_t;

void vesc_uart_init(int tx_pin, int rx_pin);
bool vesc_uart_poll(vesc_data_t* data);

#endif
