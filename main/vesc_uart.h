
#ifndef VESC_UART_H
#define VESC_UART_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

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
    float input_current;
    float duty_cycle_percent;
    float motor_temp;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    float charged_ah_total;
    uint16_t charge_cycles;
} vesc_data_t;

void vesc_uart_init(int tx_pin, int rx_pin);
void vesc_data_set(const vesc_data_t* src);
bool vesc_data_get(vesc_data_t* dst);
bool vesc_read_packet(uint8_t *payload_out, size_t *payload_len);
bool vesc_uart_poll(vesc_data_t* data);

#endif
