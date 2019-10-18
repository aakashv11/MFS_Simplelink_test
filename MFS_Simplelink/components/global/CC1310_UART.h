/*
 * cc1310_uart.h
 *
 *  Created on: Mar 28, 2016
 *      Author: Nick
 */
#include <xdc/runtime/System.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include "DRONE_Board.h"

#ifndef CC1310_UART_H_
#define CC1310_UART_H_

void CC1310_UART_init(void);

void UART_read_func(void *buf, int len);
void UART_printf(char *to_print, unsigned int len);

#endif /* CC1310_UART_H_ */
