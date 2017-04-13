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
	// SPI
void config_SPI_B0_Master_GPIO(void);
void config_SPI_B0_Master(void);
void config_SPI_B1_Slave_GPIO(void);
void config_SPI_B1_Slave(void);
void config_SPI_A0_Slave_GPIO(void);
void config_SPI_A0_Slave(void);
void SPI_read_msg(void);
void SPI_write_msg(char* message);

#endif /* SPI_H_ */
