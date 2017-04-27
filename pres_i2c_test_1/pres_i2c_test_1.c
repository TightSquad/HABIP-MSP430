
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
void calculateTempPres(signed long *temp_actual, signed long *pressure_actual);

uint32_t clockValue;			// Variable to store current clock values
uint16_t clockStatus;			// Variable to store status of Oscillator fault flags
uint32_t MCLKValue = 0;

// Calibration values
uint16_t SENS_T1;
signed long long SENS_T1_modified;
uint16_t OFF_T1;
signed long long OFF_T1_modified;
uint16_t TCS;
uint16_t TCO;
uint16_t T_REF;
signed long T_REF_modified;
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
	SENS_T1_modified = (signed long long)SENS_T1 << 16;
	presMs5607ReadPromC2(&OFF_T1);
	OFF_T1_modified = (signed long long)OFF_T1 << 17;
	presMs5607ReadPromC3(&TCS);
	presMs5607ReadPromC4(&TCO);
	presMs5607ReadPromC5(&T_REF);
	T_REF_modified = (signed long)T_REF << 8;
	presMs5607ReadPromC6(&TEMPSENS);

	// Sensor data
	signed long temp_actual; // actual temperature in degC
	signed long pressure_actual; // actual pressure in mBar

	// Read and convert pres_ms5607 data
	while(1){
		calculateTempPres(&temp_actual, &pressure_actual);
		__delay_cycles(2000); // Wait some
	}

	return 0;
}

void calculateTempPres(signed long *temp_actual, signed long *pressure_actual){ // TODO: check for issues in not typecasting in places*****************************************************
	// ADC values
	signed long adc_pressure;
	signed long adc_temperature;

	// Sensor calculated values
	signed long temp_diff; // difference between the actual and reference temperature
	signed long temperature; // calculated temperature
	signed long long pres_offset; // offset at actual temperature
	signed long long pres_sens; // sensitivity at actual temperature
	signed long pressure_pa; // calculated pressure in Pascals

	// For compensation
	signed long temperature2;
	signed long long pres_offset2;
	signed long long pres_sens2;

	// Read pressure and temperature ADC values
	presMs5607ReadData(&adc_pressure, &adc_temperature);

	// Calculate temperature
	temp_diff = adc_temperature - T_REF_modified; // dT = D2 - C5*2^8
	temperature = 2000 + ((temp_diff*TEMPSENS) >> 23); // TEMP = 2000 + dT*C6/2^23
	*temp_actual = temperature / 100.0; // temperature in degC

	// Second order temperature compensation
	temperature2 = 0.0; // T2 = 0
	pres_offset2 = 0.0; // OFF2 = 0
	pres_sens2 = 0.0; // SENS2 = 0
	if (temperature < 2000){ // Check for low temperature (< 20 degC)
		temperature2 = (temp_diff*temp_diff) >> 31; // T2 = dT^2/2^31
		pres_offset2 = (61 * (temperature-2000)*(temperature-2000)) >> 4; // OFF2 = 61 * (TEMP-2000)^2 / 2^4
		pres_sens2 = 2 * (temperature-2000) * (temperature-2000); // SENS2 = 2 * (TEMP-2000)^2

		// Check for very low temperature (< -15 degC)
		if (temperature < -1500){
			pres_offset2 = pres_offset2 + (15 * (temperature+1500) * (temperature+1500)); // OFF2 = OFF2 + 15 * (TEMP+1500)^2
			pres_sens2 = pres_sens2 + (8 * (temperature+1500) * (temperature+1500)); // SENS2 = SENS2 + 8 * (TEMP+1500)^2
		}
	}

	// Calculate temperature compensated pressure
	temperature = temperature - temperature2; // Adjusting TEMP with second order compensation (if no compensation, TEMP2 = 0)
	pres_offset = OFF_T1_modified + ((TCO*temp_diff) >> 6); // Offset at actual temperature. OFF = C2*2^17 + (C4*dT)/2^6
	pres_offset = pres_offset - pres_offset2; // Adjusting OFF with second order compensation (if no compensation, OFF2 = 0)
	pres_sens = SENS_T1_modified + ((TCS*temp_diff) >> 7); // Sensitivity at actual temperature. SENS = C1*2^16 + (C3*dT)/2^7
	pres_sens = pres_sens - pres_sens2; // Adjusting SENS with second order compensation (if no compensation, SENS2 = 0)
	pressure_pa = (((adc_pressure*pres_sens) >> 21) - pres_offset) >> 15; // temperature compensated pressure. P = (D1 * SENS/2^21 - OFF)/2^15
	*pressure_actual = pressure_pa / 100.0; // Convert pressure to mBar

	return;
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
