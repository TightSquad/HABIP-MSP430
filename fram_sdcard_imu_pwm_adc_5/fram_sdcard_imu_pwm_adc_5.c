/*******************************************************************************
 *
 * main.c
 *
 ******************************************************************************/

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

int NumLoggedDataRows = 25000;
//int NumLoggedDataRows = 1000;

unsigned char count = 0;
unsigned long data;


#pragma PERSISTENT(fram_data)
unsigned short fram_data[0x186A2] = {0};
#pragma NOINIT(numLogFiles)
unsigned short numLogFiles;

void Init_GPIO(void);
void Init_Clock(void);
void Init_RTC(void);
void enterLPM35(void);
void blinkP1OUT(int blink_number);


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

    // Board initializations
    Init_Clock();

    // create the folder on SD card
    storeTimeStampSDCard(&numLogFiles);

	int log_num = 0;
	int num_file = 0;
	int blink_count = 0;

	char hour_min_char[8];
	char sec_ms_char[8];

	uint16_t curr_sec_ms;
	uint16_t rtc_ms;
	uint16_t curr_sec;

	uint16_t curr_hour_min;
	uint16_t curr_min;
	uint16_t curr_hour;

	signed short z_gyro_data;
	char z_gyro_char[8];
	char adc_char[8];
	int current_adc_val;


	long row = 0;

	//motor control variables
	int motor_duty_cycle = 2000;
	int second_counter = 0;
	int period_counter = 0;
	int high_speed = 1;

	//start the RTC
	Init_RTC();

	__delay_cycles(2000);

	// setup the pins and timers for the motor control
	setupMotor();

	// setup ADC to read from Header J8, SPEED_IN
	setupADC();

	Init_Clock();

	//start motor PWM to 0
	motorSpeed(motor_duty_cycle);
	motorON();

	while (num_file < 3){

		motorON();

		while (log_num < NumLoggedDataRows) {
			//initialze the SPI
			setup_IMU_SPI();

			// grab IMU data
			z_gyro_data = (signed short)EMA((double)read_IMU_SPI(ZGYRO), (double)z_gyro_data);

			//capture current ADC value
			current_adc_val = readADC();

			//speed up clock to 8MHz
			Init_Clock();

			//get current time in seconds from RTC_C
			currTime = RTC_C_getCalendarTime(RTC_C_BASE);
			curr_hour = (uint16_t)currTime.Hours;
			curr_min = (uint16_t)currTime.Minutes;
			curr_sec = (uint16_t)currTime.Seconds;

			//grab RTC counter
			rtc_ms =  RTCPS;

			// convert rtcps to ms
			rtc_ms = (uint16_t)(((long)rtc_ms * 500l)/32768l);

			// concatenate seconds/ms and hour/min
			curr_sec_ms = (curr_sec << 10) | rtc_ms;
			curr_hour_min = (curr_hour << 8) | curr_min;

			// write the data to FRAM
			__data20_write_short((unsigned long int)fram_data + row, curr_hour_min);
			__data20_write_short((unsigned long int)fram_data + row + 2, curr_sec_ms);
			__data20_write_short((unsigned long int)fram_data + row + 4, z_gyro_data);
			__data20_write_short((unsigned long int)fram_data + row + 6, current_adc_val);


			//motor control stuff
			if (second_counter < (int)curr_sec){
				period_counter++;
				if (period_counter == 10){
					if (high_speed == 1){
						motor_duty_cycle = 8000;
						motorSpeed(motor_duty_cycle);
						period_counter = 0;
						high_speed = 0;
					}
					else {
						motor_duty_cycle = 18000;
						motorSpeed(motor_duty_cycle);
						period_counter = 0;
						high_speed = 1;
					}
				}
				second_counter = (int)curr_sec;
			}

			//after a minute need to reset the second_counter back to 0
			if ((int)curr_sec == 0){
				second_counter = 0;
			}

			//blink every 100 samples captured
			if (blink_count == 100){
				GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
				GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN1);
				blink_count = 0;
			}

			log_num++;
			row = row + 8;
			blink_count++;

			__delay_cycles(32000);
		}

		//slow down and stop the motor
		motorRampDown();
		motorOFF();

		//Open txt file
		SDFindOpenNewFile();

		//reset counter variables
		row = 0;
		blink_count = 0;
		log_num = 0;

		//start data logging
		for(log_num = 0; log_num < NumLoggedDataRows; log_num++){
			//read from FRAM2 and convert int to char string
			itoa(__data20_read_short((unsigned long int)fram_data + row), hour_min_char, 16);
			itoa(__data20_read_short((unsigned long int)fram_data+ row + 2), sec_ms_char, 16);
			itoa(__data20_read_short((unsigned long int)fram_data+ row + 4), z_gyro_char, 10);
			itoa(__data20_read_short((unsigned long int)fram_data+ row + 6), adc_char, 10);

			row = row + 8;

			//write all data to a single line in the currently open txt file
			writeDataSameLine_4(hour_min_char, sec_ms_char, z_gyro_char, adc_char);

			//keep track when to give status blink
			blink_count++;
			if (blink_count == 100){
				GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
				blink_count = 0;
			}
		}

		//close the currently open txt file
		SDCloseSPI();

		row = 0;
		log_num = 0;
		num_file++;
		 // create new file to save data
		SDCardNewFile(&numLogFiles);
	}

	GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
	GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
	while(1);
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
void Init_RTC()
{
    //Setup Current Time for Calendar
    calendar.Seconds    = 0x00;
    calendar.Minutes    = 0x00;
    calendar.Hours      = 0x00;
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

//    RTC_C_enableInterrupt(RTC_C_BASE, RTC_C_TIME_EVENT_INTERRUPT);


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



