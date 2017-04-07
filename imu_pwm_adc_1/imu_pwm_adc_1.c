//******************************************************************************
//   DACQS Host Board - Motor MSP430 - Main IMU Read and PWM control example
//
//	 Using the IMu data, drive a PWM and GPIO based on the directon and magnitude of the IMU data
//******************************************************************************
#include <msp430.h>
#include <driverlib.h>

int read_IMU_SPI(unsigned char register_address);
void setup_IMU_SPI(void);

void setupADC(void);
int readADC(void);

void setupMotor(void);
void motorON(void);
void motorOFF(void);
void motorCCW(void);
void motorCW(void);
void motorSpeed(int duty_cycle);
void currentControl(int current_duty_cycle);

volatile unsigned char RXData = 0;
volatile unsigned char RXData2 = 0;
volatile unsigned char TXData;

//adc stuff
uint16_t adc_val;
int adc_software_flag = 0;
int adc_array[500];

signed int data_array [500];

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

//Send/Receive Headers
const unsigned char READcmd = 0x00;            		// ADIS16350's read command
const unsigned char WRITEcmd = 0x80;                // ADIS16350's write command


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer


    // setup the pins and timers for the motor control
    setupMotor();

    //initialze the SPI
    setup_IMU_SPI();

    // setup ADC to read from Header J8, SPEED_IN
    setupADC();

    int i = 0;
    int z_gyro_data;
    int current_adc_val;

    while(1)
    {
    	// grab IMU data
    	z_gyro_data = read_IMU_SPI(ZGYRO);

    	//store in an array to plot later
		data_array[i] = z_gyro_data;
		i++;

    	//capture current ADC value
    	current_adc_val = readADC();
    	adc_array[i] = current_adc_val;


    	// if CW rotation is detected, make a PWM that is linearly proportional to the IMU data
    	if(z_gyro_data >= 30){
    		motorSpeed(z_gyro_data);
    		//spin the motor in CW direction
    		motorCW();
    		motorON();
    	}

    	// if CCW motion is detected, make a PWM that is linearly proportional to the IMU data
    	if(z_gyro_data < -30){
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

    	__delay_cycles(2000);
    }
}


void setup_IMU_SPI(void){

    // Configure GPIO
    P2SEL1 |= (BIT3 + BIT4 + BIT5 + BIT6); 	 // UCA1STE, UCA1CLK, UCA1SIMO, UCA1SOMI
    P2SEL0 &= ~(BIT3 + BIT4 + BIT5 + BIT6);  // UCA1STE, UCA1CLK, UCA1SIMO, UCA1SOMI
    PJSEL0 |= BIT4 | BIT5;					 // enable the external

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // XT1 Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
    CSCTL1 &= ~DCORSEL;
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__16 | DIVS__16 | DIVM__16;   // Set all dividers
    CSCTL4 &= ~LFXTOFF;
    do
    {
        CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers

    // Configure USCI_B1 for SPI operation
    UCA1CTLW0 = UCSWRST;                    // **Put state machine in reset**
                                            // 4-pin, 8-bit SPI master
    UCA1CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB | UCMODE_2 | UCSTEM;
                                            // Clock polarity high, MSB
    //UCB1CTLW0 |= UCSSEL__ACLK;              // ACLK
    UCA1CTLW0 |= UCSSEL_3;              	// SMCLK
    UCA1BRW = 0x4;                         // /4
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt

}


int read_IMU_SPI(unsigned char register_address){
	signed int dataOut = 0;
	// ADIS16350's read filler (Dont care bits after register addr)
	const unsigned char READFILLER = 0x5A;

	//write the address you want to read
	TXData = register_address;
	UCA1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

    //send filler data to make total data frame 16 bits
    TXData = READFILLER;
    UCA1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

    //second time around, this time we will get the data. Need to clock data again so send the same stuff again.
    TXData = register_address;
	UCA1IE |= UCTXIE;
	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

	// acquire first data frame from RX
	dataOut = RXData;

	// shift the result to the MSB of 16bit dataOut
	dataOut = dataOut << 8;

	//send next data frame to receive the final output
	TXData = READFILLER;
	UCA1IE |= UCTXIE;
	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

	// concatenate the current data frame (RXData) with the previous frame
	dataOut = dataOut | RXData;

	//First 2 bits are flags/alarms
	dataOut = dataOut & 0x3fff;

	// to determine the sign of the result, determine if the 14th bit is a one or not. this will determine if it is in Two Complement form
	if(dataOut & 0x2000){
		// if in Two's comp, make bit 16 and 15 ones. This will allow int to read as a negative number
		dataOut = dataOut | 0xC000;
	}
	
	return dataOut;
}

void setupADC(void){
	   // Configure P3.0 for ADC, A12
	   P3SEL1 |= BIT0;
	   P3SEL0 |= BIT0;

	   //clear and use 16clk cycles, turn ADC on
	   ADC12CTL0 = 0;
	   ADC12CTL0 |= ADC12SHT0_2 + ADC12ON;

	   //clear and turn on sampling timer
	   ADC12CTL1 = 0;
	   ADC12CTL1 = ADC12SHP;                   // Use sampling timer

	   //set 12bit resolution
	   ADC12CTL2 = 0;
	   ADC12CTL2 |= ADC12RES_2;

	   //clear and setup A12 as input source for MEM12
	   ADC12MCTL12 = 0;
	   ADC12MCTL12 |= ADC12_B_INPUT_A12;

	   //start acquistions on MEM12
	   ADC12CTL3 = 0;
	   ADC12CTL3 |= ADC12CSTARTADD_12;

	   // enable interrupts for A12
	   ADC12IER0 = 0;
	   ADC12IER0 |= ADC12IE12;
}

int readADC(void){
	// Enable ADC and start acquisition
   ADC12CTL0 |= ADC12ENC + ADC12SC;

   //enable general interrupts
   __bis_SR_register(LPM0_bits | GIE);

   //wait for the adc software flag to be set
   while(adc_software_flag == 0);

   //disable the ADC
   ADC12CTL0 &= ~ADC12ENC;

   //clear the adc software flag
   adc_software_flag = 0;

   return adc_val;
}

void setupMotor(void){
	//-------------------------------------------
	// GPIO setup for motor direction and enable
    // Set P4.0 and P4.1 to output direction
    P4DIR |= BIT0 + BIT1;
	//-------------------------------------------


	//-------------------------------------------
    // Setup GPIO to generate PWM for motor speed and current
    // Set P1.2 and P1.3 to output
    P1DIR |= BIT2 + BIT3;

    // Set P1.2 and P1.3 to TA1.1 and TA1.2 mode respectively, See Table 6-20 and TABLE 6-21 in MSP430FR4994 Data Sheet
    P1SEL1 &= ~(BIT2 + BIT3);
    P1SEL0 |= BIT2 + BIT3;


	//-------------------------------------------
    // Timer1 Set-Up

    // Set PWM Period in uSeconds
	TA1CCR0 |= 20000;

	//Set output mode to reset/set
	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_7;

	// Set the default duty cylce of the PWM in uSeconds
	TA1CCR1 |= 0;
	TA1CCR2 |= 0;

	//Start Timer1 and PWM
	TA1CTL |= TASSEL_2 + MC_1;
	//-------------------------------------------

}

void motorON(void){
	//set the P4.0 bit high to enable motor
	P4OUT |= BIT0;
}

void motorOFF(void){
	//set the P4.0 low high to disable motor
	P4OUT &= ~BIT0;
}

void motorCCW(void){
	//set the P4.1 bit low to direct motor in the counter clock-wise direction
	P4OUT &= ~BIT1;
}

void motorCW(void){
	// set the P4.1 bit to high to direct the motor in the clockwise direction
	P4OUT |= BIT1;
}

void motorSpeed(int duty_cycle){
	TA1CCR1 = duty_cycle;
}

void currentControl(int current_duty_cycle){
	TA1CCR2 = current_duty_cycle;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA1IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
        	// Receive characters
        	RXData= UCA1RXBUF;
        	UCA1IFG &= ~UCRXIFG;
            __bic_SR_register_on_exit(LPM0_bits);
            break;

        case USCI_SPI_UCTXIFG:
        	// Transmit characters
            UCA1TXBUF = TXData;
            UCA1IE &= ~UCTXIE;
            break;
        default: break;
    }
}

//ADC Interrupt Service Routine
#pragma vector=ADC12_B_VECTOR
__interrupt void ADC12_ISR(void)
{
	//set the software flag
	adc_software_flag = 1;
	//grab the ADC value
	adc_val = ADC12MEM12;			//store value of ADC in array
	__bic_SR_register_on_exit(LPM0_bits);
	return;

}
