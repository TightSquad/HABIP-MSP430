/*
 * UART.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_UART_H_
#define HABIP_UART_H_

#define LISTENING_FOR_RESPONSE 0x00
#define CAPTURING_RESPONSE 0x01
#define MSG_BUFFER_FULL 0x02

// buffer status
#define AVAILABLE 0x00
#define VALID 0x01

// Interrupt setup
#define BUFFER 0x00
#define NO_BUFFER 0x01

  // UART
void config_UART_GPIO(int brd_num);
void config_UART_9600_ACLK_32768Hz(int brd_num);
void config_UART_9600_SMCLK_8MHz(int brd_num);
void UART_write_msg(int brd_num, char* message);
void UART_read_response(int brd_num, volatile int* RXSWFG);

#endif /* HABIP_UART_H_ */
