/*
 * daqcs_imu.h
 *
 *  Created on: Apr 11, 2017
 *      Author: stevy
 */

#ifndef DAQCS_IMU_H_
#define DAQCS_IMU_H_

#include <msp430.h>
#include <driverlib.h>

void write_IMU_SPI(unsigned int register_address, unsigned char date_write);
int read_IMU_SPI(unsigned char register_address);
void setup_IMU_SPI(void);


//Sensor's Read Register Adddresses
const unsigned char POWERSUPPLY = 0x02;            	// Power Supply Voltage
const unsigned char XGYRO = 0x04;                  	// X Gyro Measurement
const unsigned char YGYRO = 0x06;                  	// Y Gyro Measurement
const unsigned char ZGYRO = 0x08;                  	// Z Gyro Measurement
const unsigned char XACCEL = 0x0A;                 	// X Acceleration Measurement
const unsigned char YACCEL = 0x0C;                 	// Y Acceleration Measurement
const unsigned char ZACCEL = 0x0E;                 	// Z Acceleration Measurement
const unsigned char XTEMP = 0x10;                  	// X Temperature Measurement
const unsigned char YTEMP = 0x12;                  	// Y Temperature Measurement
const unsigned char ZTEMP = 0x14;                  	// Z Temperature Measurement
const unsigned char XGYROOFF = 0x1A;			   	// X Gyro Offset
const unsigned char YGYROOFF = 0x1C;			   	// Y Gyro Offset
const unsigned char ZGYROOFF = 0x1E;			   	// Z Gyro Offset
const unsigned char XACCELOFF = 0x20;			   	// X Accel Offset
const unsigned char YACCELOFF = 0x22;			   	// Y Accel Offset
const unsigned char ZACCELOFF = 0x24;			   	// Z Accel Offset

const unsigned char SMPL_PRD = 0x36;			   	// Z Accel Offset
const unsigned char SMPL_PRD_1 = 0x07;			   	// Z Accel Offset


//Send/Receive Headers
const unsigned char READcmd = 0x00;            		// ADIS16350's read command
const unsigned char WRITEcmd = 0x80;                // ADIS16350's write command


#endif /* DAQCS_IMU_H_ */
