#ifndef VESC_PARSER_H
#define VESC_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "vesc_uart.h"

bool vesc_parse_payload(const uint8_t *payload, int len, vesc_data_t *out);

#endif
