/*
 * cc1310_uart.c
 *
 *  Created on: Mar 28, 2016
 *      Author: Nick
 */


#include "CC1310_UART.h"


UART_Handle uart;
UART_Params uart_params;

void CC1310_UART_init(void) {
    UART_init();
    /* Create a UART with data processing off. */
    UART_Params_init(&uart_params);
    uart_params.writeDataMode = UART_DATA_BINARY;
    uart_params.readDataMode = UART_DATA_BINARY;
    uart_params.readReturnMode = UART_RETURN_FULL;
    uart_params.readEcho = UART_ECHO_OFF;
    uart_params.baudRate = 115200;
    uart = UART_open(Board_UART, &uart_params);

    if (uart == NULL) {
        System_abort("Error opening the UART!");
    }
}

void UART_read_func(void *buf, int len) {
    UART_read(uart, (char *)buf, len);
}

void UART_printf(char *to_print, unsigned int len) {
    UART_write(uart, (void *)to_print, len);
}
