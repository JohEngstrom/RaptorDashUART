#include "vesc_parser.h"
#include <math.h>

static float smoothed_watts = 0;
static float peak_watts = 0;
static float wheel_circum_mm = 890.0f;
static int motor_pole_pairs = 15;
static float battery_min_v = 48.0f;
static float battery_max_v = 67.2f;
static float battery_wh = 960.0f;
static int32_t last_tachometer = -1;
static float session_km = 0;

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

bool vesc_parse_payload(const uint8_t *payload, int len, vesc_data_t *data) {
    if (len < 40) return false;

    int i = 5; // skip packet id and 4-byte mask
    data->mosfet_temp       = read_float16(payload, 10.0f, &i);
    data->motor_temp        = read_float16(payload, 10.0f, &i);
    data->current           = read_float32(payload, 100.0f, &i);
    data->input_current     = read_float32(payload, 100.0f, &i);
    read_float32(payload, 100.0f, &i); // skip id
    read_float32(payload, 100.0f, &i); // skip iq
    data->duty_cycle_percent = read_float16(payload, 1000.0f, &i);
    data->rpm               = read_int32(payload, &i);
    data->voltage           = read_float16(payload, 10.0f, &i);
    data->amp_hours         = read_float32(payload, 10000.0f, &i);
    data->amp_hours_charged = read_float32(payload, 10000.0f, &i);
    data->watt_hours        = read_float32(payload, 10000.0f, &i);
    data->watt_hours_charged = read_float32(payload, 10000.0f, &i);
    int32_t tach            = read_int32(payload, &i);
    int32_t tach_abs        = read_int32(payload, &i);

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

    return true;
}
