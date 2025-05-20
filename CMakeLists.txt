idf_component_register(
    SRCS "crc16.c"
         "uart_platform_esp32.c"
         "vesc_parser.c"
         "vesc_uart.c"
         "vesc_uart_write.c"
    INCLUDE_DIRS "."
)
