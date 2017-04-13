/*
 * UART.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_UART_H_
#define HABIP_UART_H_
// #include <msp430.h>
// #include <driverlib.h>
//#include <common.h>
  // UART
void config_UART_4_GPIO(void);
void config_UART_4_9600_ACLK_32768Hz(void);
void config_UART_4_9600_SMCLK_8MHz(void);
void UART_read_msg(void);
void UART_write_msg(char* message);
void chris_init(void);

#endif /* HABIP_UART_H_ */
