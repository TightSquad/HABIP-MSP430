
//***************************************************************************************
// Read Temperature data from PCT2075 I2C sensor
// SDA on P7.0
// SCL on P7.1
//***************************************************************************************

#include <driverlib.h>
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "driverlib.h"
#include "i2c_driver.h"
#include "pres_ms5607.h"

void configure_clocks(void);
int32_t movingAvg(int prevAvg, int16_t newValue);

uint32_t clockValue;			// Variable to store current clock values
uint16_t clockStatus;			// Variable to store status of Oscillator fault flags
uint32_t MCLKValue = 0;

uint16_t SENS_T1;
uint16_t OFF_T1;
uint16_t TCS;
uint16_t TCO;
uint16_t T_REF;
uint16_t TEMPSENS;

int main(void) {

    // Stop Watchdog Timer
	WDT_A_hold(WDT_A_BASE);

	// Setup clock configuration
	configure_clocks();

	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;

	P1DIR |= 0x01;                          // Set P1.0 to output direction

	// Initialize i2c
	initI2C();

	// Read calibration data from PROM
	presMs5607ReadPromC1(&SENS_T1);
	presMs5607ReadPromC2(&OFF_T1);
	presMs5607ReadPromC3(&TCS);
	presMs5607ReadPromC4(&TCO);
	presMs5607ReadPromC5(&T_REF);
	presMs5607ReadPromC6(&TEMPSENS);

	// Modify calibration data to be easier to calculate temp and pressure later

	// Then, to get temp and/or pressure, read the digital pressure and/or temp data & calculate the temp and temp compensated pressure

	// Read/convert tmp007 and opt3001 data
	while(1){
	//	sensorTmp007Read(&rawTemp, &rawObjTemp);
	//	shiftTemp = rawTemp >> 5;
	//	temp = (float)(shiftTemp) / 8;
	//	temp_F = (temp * 1.8) + 32;
	}

}

void configure_clocks(void)
{
    // Sets DCO to 8 MHz
	CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);

    // Sets SMCLK, MCLK source as DCO
	CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Sets ACLK source as VLO (~9.4 kHz)
//	CS_initClockSignal(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Sets ACLK source as LFMODCLK (~38kHz)
	CS_initClockSignal(CS_ACLK, CS_LFMODOSC_SELECT, CS_CLOCK_DIVIDER_1);

    // Clear and enable global oscillator fault flag
	SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
	//SFR_enableInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);

    __bis_SR_register(GIE);		// Enable global interrupt

	clockValue = CS_getACLK();
    clockValue = CS_getSMCLK();
    MCLKValue = CS_getMCLK();
}
