/*
 * SPI.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef SPI_H_
#define SPI_H_
// #include <msp430.h>
//#include <common.h>


#define LISTENING_FOR_COMMAND 0x00
#define CAPTURING_COMMAND 0x01
#define OBTAINING_DATA 0x02
#define SENDING_DATA 0x03

	// SPI
void config_SPI_B0_Master_GPIO(void);
void config_SPI_B0_Master(void);
void config_SPI_B1_Slave_GPIO(void);
void config_SPI_B1_Slave(void);
void config_SPI_A0_Slave_GPIO(void);
void config_SPI_A0_Slave(void);
void SPI_read_msg(void);
void SPI_command_host_to_slave(char* message,volatile int* read_done,volatile char* TXDATA);

#endif /* SPI_H_ */
