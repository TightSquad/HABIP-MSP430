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
//#include "LiveTempMode.h"
//#include "FRAMLogMode.h"
#include "SDCardLogMode.h"
#include "driverlib.h"
#include "daqcs_imu.h"
#include "daqcs_motor.h"
#include "daqcs_adc.h"

extern int mode;
extern int pingHost;

//uint8_t RXData = 0;                               // UART Receive byte
int mode = 0;                                     // mode selection variable
int pingHost = 0;                                 // ping request from PC GUI
int noSDCard = 0;
Calendar calendar;                                // Calendar used for RTC

Calendar currTime;

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
void blinkP1OUT(int blink_number);


signed int data_array [500];


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

    // Board initializations
    Init_Clock();

//    //initialze the SPI
    //setup_IMU_SPI();

    // create the folder on SD card
    storeTimeStampSDCard();

    blinkP1OUT(3);

	int log_num = 0;
	char test_data[] = "how are you";
	char time_sec[8];
	int curr_sec;
	//uint32_t currMS;
	int z_gyro_data;
	char z_gyro_char[8];
	char adc_char[8];
	int current_adc_val;

	int i = 0;

	//start the RTC
	Init_RTC();

	__delay_cycles(2000);

	// setup the pins and timers for the motor control
	setupMotor();

	// setup ADC to read from Header J8, SPEED_IN
	setupADC();

	//Init_Clock();

	//setup_IMU_SPI();
	//write_IMU_SPI(SMPL_PRD, SMPL_PRD_1);

	SDFindOpenNewFile();

	while (log_num < 100) {
		//initialze the SPI
		setup_IMU_SPI();

		// grab IMU data
		z_gyro_data = read_IMU_SPI(ZGYRO);

		//store in an array to plot later
		data_array[i] = z_gyro_data;
		i++;
		//convert the zgyro int to a char string
		itoa(z_gyro_data, z_gyro_char, 10);


		// if CW rotation is detected, make a PWM that is linearly proportional to the IMU data
		if(z_gyro_data >= 30){
			motorSpeed(z_gyro_data);
			//spin the motor in CW direction
			motorCW();
			motorON();
		}

		// if CCW motion is detected, make a PWM that is linearly proportional to the IMU data
		else if(z_gyro_data < -30){
			// invert the magnitude, so a positive value can be sent to PWM
			z_gyro_data = z_gyro_data * -1;
			motorSpeed(z_gyro_data);

			// spin motor CCW
			motorCCW();
			motorON();
		}

		else{
			motorOFF();
		}

		//capture current ADC value
		current_adc_val = readADC();
		itoa(current_adc_val, adc_char, 10);
//		adc_array[i] = current_adc_val;

		Init_Clock();

		//get current time in seconds from RTC_C
		currTime = RTC_C_getCalendarTime(RTC_C_BASE);
		curr_sec = currTime.Seconds;
		itoa(curr_sec, time_sec, 10);

		GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);
		// Write the time and gyro data to a single line
		//writeDataSameLine(time_sec, z_gyro_char, adc_char);
		writeDataSameLine_2(time_sec, z_gyro_char, adc_char);

		//writeData(time_sec);
		GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN5);

		log_num++;

		//blink an LED
		blinkP1OUT(1);
	}

	SDCloseSPI();
	GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

}

/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    // Set all GPIO pins to output low for low power
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);
//    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN7);
//
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
//    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7|GPIO_PIN8|GPIO_PIN9|GPIO_PIN10|GPIO_PIN11|GPIO_PIN12|GPIO_PIN13|GPIO_PIN14|GPIO_PIN15);

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
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_4);
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
void Init_RTC()
{
    //Setup Current Time for Calendar
    calendar.Seconds    = 0x00;
    calendar.Minutes    = 0x00;
    calendar.Hours      = 0x02;
    calendar.DayOfWeek  = 0x06;
    calendar.DayOfMonth = 0x11;
    calendar.Month      = 0x04;
    calendar.Year       = 0x2017;

    // Initialize RTC with the specified Calendar above
    RTC_C_initCalendar(RTC_C_BASE,
                       &calendar,
                       RTC_C_FORMAT_BCD);

    RTC_C_setCalendarEvent(RTC_C_BASE,
    		               RTC_C_CALENDAREVENT_MINUTECHANGE
    		               );

    RTC_C_clearInterrupt(RTC_C_BASE,
                         RTC_C_TIME_EVENT_INTERRUPT
                         );

    RTC_C_enableInterrupt(RTC_C_BASE,
                          RTC_C_TIME_EVENT_INTERRUPT
                          );

    //Start RTC Clock
    RTC_C_startClock(RTC_C_BASE);
}


/*
 * Enter Low Power Mode 3.5
 */
void enterLPM35()
{
	// Configure button S2 (P5.5) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P5, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN5);

    // Request the disabling of the core voltage regulator when device enters
    // LPM3 (or LPM4) so that we can effectively enter LPM3.5 (or LPM4.5).
    PMM_turnOffRegulator();

    //Enter LPM3 mode with interrupts enabled
    __bis_SR_register(LPM4_bits + GIE);
    __no_operation();
}


void blinkP1OUT(int blink_number){
	int i;

	for (i=0;i<blink_number;i++)
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		__delay_cycles(5000);
		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
		__delay_cycles(5000);
	}
}


