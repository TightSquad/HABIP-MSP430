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

#define LISTENING_FOR_RESPONSE 0x00
#define CAPTURING_RESPONSE 0x01

  // UART
void config_UART_B3_GPIO(void);
void config_UART_B3_9600_ACLK_32768Hz(void);
void config_UART_B3_9600_SMCLK_8MHz(void);
void UART_read_msg(void);
void UART_write_msg(char* message);
//void UART_read_response(char* message,volatile int* RXSWFG);
//void UART_B0_read_response(volatile int* RXSWFG0);
//void UART_B1_read_response(volatile int* RXSWFG1);
//void UART_B2_read_response(volatile int* RXSWFG2);
void UART_B3_read_response(volatile int* RXSWFG3);
void chris_init(void);
void array_copy(volatile char array_from[],volatile char array_to[]);

#endif /* HABIP_UART_H_ */
