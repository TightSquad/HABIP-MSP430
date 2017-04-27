/******************************************************************************
 *  Filename:       pres_ms5607.c
 *  Revised:
 *  Revision:
 *
 *  Description:    Driver for the I2C pressure sensor TE Connectivity part
*  				number MS560702BA03-50. Digi-key part number 223-1198-1-ND
 *
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "msp430.h"
#include "driverlib.h"
#include "i2c_driver.h"
#include "pres_ms5607.h"

static uint8_t buf[SENSOR_DATA_SIZE];
static uint16_t val;

// Reset pressure sensor
bool presMs5607Reset(void){
	bool success;

	// Write to the sensor reset register
	success = writeI2C(PRES_MS5607_I2C_ADDRESS, PRES_MS5607_REG_RESET, 0x00, 1);

	if (!success){
		// sensorSetErrorData
	}

	return (success);
}

// Read calibration data (factory calibrated) from PROM
bool presMs5607ReadProm(uint8_t ui8Reg){
	bool success;

	// Read register
	success = readI2C(PRES_MS5607_I2C_ADDRESS, ui8Reg, &buf[0], REGISTER_LENGTH);

	if (!success){
		// sensorSetErrorData
	}

	// Swap bytes
	val = (buf[0]<<8) | buf[1];

	return (success);
}


// Read C1 from sensor
bool presMs5607ReadPromC1(uint16_t *SENS_T1){
	bool success;

	// Read register
	success = presMs5607ReadProm(PRES_MS5607_REG_PROM_C1);
	*SENS_T1 = val;

	return (success);
}

// Read C2 from sensor
bool presMs5607ReadPromC2(uint16_t *OFF_T1){
	bool success;

	// Read register
	success = presMs5607ReadProm(PRES_MS5607_REG_PROM_C2);
	*OFF_T1 = val;

	return (success);
}

// Read C3 from sensor
bool presMs5607ReadPromC3(uint16_t *TCS){
	bool success;

	// Read register
	success = presMs5607ReadProm(PRES_MS5607_REG_PROM_C3);
	*TCS = val;

	return (success);
}

// Read C4 from sensor
bool presMs5607ReadPromC4(uint16_t *TCO){
	bool success;

	// Read register
	success = presMs5607ReadProm(PRES_MS5607_REG_PROM_C4);
	*TCO = val;

	return (success);
}

// Read C5 from sensor
bool presMs5607ReadPromC5(uint16_t *T_REF){
	bool success;

	// Read register
	success = presMs5607ReadProm(PRES_MS5607_REG_PROM_C5);
	*T_REF = val;

	return (success);
}

// Read C6 from sensor
bool presMs5607ReadPromC6(uint16_t *TEMPSENS){
	bool success;

	// Read register
	success = presMs5607ReadProm(PRES_MS5607_REG_PROM_C6);
	*TEMPSENS = val;

	return (success);
}
