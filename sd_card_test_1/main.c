/*******************************************************************************
 *
 * main.c
 *
 * Out of Box Demo for the MSP-EXP430FR5994
 * Main loop, initialization, and interrupt service routines
 *
 * Feb 2016
 * E. Chen
 *
 ******************************************************************************/

//#include "main.h"
#include "LiveTempMode.h"
#include "FRAMLogMode.h"
#include "SDCardLogMode.h"

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "driverlib.h"
#include "i2c_driver.h"
#include "tmp007.h"

extern int mode;
extern int pingHost;

uint8_t RXData = 0;       		// UART Receive byte
int mode = 0;                   // mode selection variable
int pingHost = 0;               // ping request from PC GUI
int noSDCard = 0;
Calendar calendar;              // Calendar used for RTC

Calendar currTime;

//***** Global Data *****
uint32_t clockValue;			// Variable to store current clock values
uint16_t clockStatus;			// Variable to store status of Oscillator fault flags
uint32_t MCLKValue = 0;

// TMP007
uint16_t rawTemp;
uint16_t rawObjTemp;
float    tObjTemp;
float    tObjAmb;

int shiftTemp;
float temp, temp_F;

#define CAL_ADC_12T30_L  *(int8_t *)(0x1A1E) // Temperature Sensor Calibration-30 C 2.0V ref
#define CAL_ADC_12T30_H  *(int8_t *)(0x1A1F)
#define CAL_ADC_12T85_L  *(int8_t *)(0x1A20) // Temperature Sensor Calibration-85 C 2.0V ref
#define CAL_ADC_12T85_H  *(int8_t *)(0x1A21)

#if defined(__IAR_SYSTEMS_ICC__)
#pragma location = 0x9000
__no_init uint16_t dataArray[12289];
#endif


void Init_GPIO(void);
void Init_Clock(void);
void Init_RTC(void);
void enterLPM35(void);

void configure_clocks(void);


//-----------------------------------------------------------------------------
int _system_pre_init(void)
{
    // Stop Watchdog timer
    WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WDT

    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);

    /*==================================*/
    /* Choose if segment initialization */
    /* should be done or not. */
    /* Return: 0 to omit initialization */
    /* 1 to run initialization */
    /*==================================*/
    return 1;
}

/*
 * main.c
 */
int main(void) {


	Init_GPIO();
	GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

	// Toggle LED1 and LED2 to indicate OutOfBox Demo start
	int i;
//    	for (i=10;i>0;i--)
//    	{
//            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN1);
//            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
//            __delay_cycles(200000);
//    	}
//

    // Board initializations
    Init_Clock();

    storeTimeStampSDCard();

	for (i=0;i<3;i++)
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		__delay_cycles(300000);
		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
		__delay_cycles(2000000);
	}

	// Setup clock configuration
	configure_clocks();

	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;

	P1DIR |= 0x01;                          // Set P1.0 to output direction

	// Initialize i2c
	//initI2C();

//	sensorTmp007Read(&rawTemp, &rawObjTemp);
//	shiftTemp = rawTemp >> 5;
//	temp = (double)(shiftTemp) / 8;
//	double temp_F = (temp * 1.8) + 32;
//	char charray[200];
//	sprintf(charray, "%0.7f", temp_F);

	int log_num = 0;

	while (log_num < 20) {
//		sensorTmp007Read(&rawTemp, &rawObjTemp);
//		shiftTemp = rawTemp >> 5;
//		temp = (double)(shiftTemp) / 8;
//		double temp_F = (temp * 1.8) + 32;
//		char char_array[200];
//		sprintf(charray, "%0.3f", temp_F);
//		char_array = "hello_world";
		char char_array[11] = "hello_world";
		writeData(char_array);
		log_num++;
		__delay_cycles(1000000);
	}


//	currTime = RTC_C_getCalendarTime(RTC_C_BASE);
//
//	char day;
//	day = currTime.DayOfWeek;


	for (i=0;i<3;i++)
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		__delay_cycles(300000);
		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
		__delay_cycles(2000000);
	}
    //}
}

/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low for low power
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);

	// Configure P2.0 - UCA0TXD and P2.1 - UCA0RXD
//	GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
//	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
//    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);

    // Set PJ.4 and PJ.5 as Primary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * Clock System Initialization
 */
void Init_Clock()
{
    // Set DCO frequency to 8 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768, 0);
    //Set ACLK=LFXT
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}

/*
 * Real Time Clock Initialization
 */


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

/*
 * Timer0_A3 Interrupt Vector (TAIV) handler
 * Used to trigger ADC conversion every 0.125 seconds
 *
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits); // Exit active CPU
}


