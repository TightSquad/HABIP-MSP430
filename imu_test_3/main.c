#include <msp430.h>

/*
 * MSP430 SPI Interface to ADIS16350 (Analog Devices IMU)
 * Developed by Matt Zachary
 * Rochester Institute of Technology
 * CubeSat
 * 2/23/2016
 */

//Function Prototypes
int readRegister (unsigned char);
void writeRegister (unsigned char, unsigned int);
void setupSPI();
//void setupUART();
void USCI0RX_ISR(void);
int hexIntToChar(int);
void delay(void);

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
const unsigned char READFILLER = 0x5A;             	// ADIS16350's read filler (Dont care bits after register addr)

unsigned int speed = 0x0FFF;
int baseline_captured = 0;
int baseline;

int i = 0;
volatile unsigned int data_array [1000];
/*
 * P1.4: CS
 * P1.6: MISO
 * P1.7: MOSI
 * P1.5: CLK
 */

int main(void)
{

	// Stop watchdog timer
	WDTCTL = WDTPW | WDTHOLD;

	//Increase SMCLK to 16MHZ
	//BCSCTL1 = CALBC1_16MHZ;       // Set range
	//DCOCTL = CALDCO_16MHZ;        // Set DCO step and modulation

	//Perform SPI Setup
	setupSPI();

	while(1){
		UCB1IE |= UCTXIE;
		data_array[i] = readRegister(ZGYRO);
		i++;
		//SpinMotor(dataReceived);
		delay();
	}
//	//ISR is servicing Command Requests
//	while(1);
}

/*
 * Read data from the device at register [registerNumber]
 * Will return all 16 bits from that register
 * 2 most significant bits are removed
 * They are flags/alarms
 */

int hexIntToChar(int i) {
	if (i < 0xA) {
		return (i & 0xf) + 0x30;
	}
	else {
		return (i & 0xf) + 0x37;
	}

}

void delay(void){
	unsigned int j;
	j = speed;
	j--;
	while(j > 0){	// software delay
		j--;
		j--;
	}
}

int readRegister(unsigned char registerNumber)
{
	//Received character
	unsigned char firstByte = READcmd | registerNumber;
	unsigned char secondByte = READFILLER;
	int resultLength = 2;

	//Received Data
	unsigned char receivedChar = 0;
	int dataOut = 0;

	//Select Device Manually (not built in)
	P5OUT &= (~BIT3);

	/*
	 * Send first byte in a frame
	 */

	//USCI_A0 TX Buffer Ready?
	//while (!(IFG2 & UCB0TXIFG));
	while(!(UCB1IFG & UCTXIFG))

	//Send 1st byte (header/reg addr)
	UCB1TXBUF = firstByte;

	//USCI_A0 TX Buffer Ready?
	while(!(UCB1IFG & UCTXIFG))

	//Send 2nd byte (filler/dont care)
	UCB1TXBUF = secondByte;

	//USCI_A0 TX Buffer Ready?
	while(!(UCB1IFG & UCTXIFG))

	//Continously read in bytes
	//Dont care what they are
	while ((resultLength > 0))
	{
		//Byte Received?
		while(!(UCB1IFG & UCRXIFG))

		//Store Received data
		receivedChar = UCB1RXBUF;

		//Decrement Result Length
		resultLength--;
	}

	//Reset Result Length
	resultLength = 2;

	/*
	 * Send random byte in separate frame to capture data from first frame
	 */

	//USCI_A0 TX Buffer Ready?
	while(!(UCB1IFG & UCTXIFG))

	//Send 1st byte (random byte 1)
	UCB1TXBUF = firstByte;

	//USCI_A0 TX Buffer Ready?
	while(!(UCB1IFG & UCTXIFG))

	//Send 2nd byte (random byte 2)
	UCB1TXBUF = secondByte;

	//USCI_A0 TX Buffer Ready?
	while(!(UCB1IFG & UCTXIFG))

	//Continously read in bytes
	//ACTUAL data
	while ((resultLength > 0))
	{
		//Byte Received?
		while(!(UCB1IFG & UCRXIFG))

		//Store Received data
		receivedChar = UCB1RXBUF;

		//Append received byte to data
		dataOut = dataOut << 8;
		dataOut = dataOut | receivedChar;

		//First 2 bits are flags/alarms
		dataOut = dataOut & 0x3fff;

		//Decrement Result Length
		resultLength--;
	}

	//Unselect Device
	P5OUT |= (BIT3);

	//Done, return
	return(dataOut);
}


/*
 * Messy SPI Setup Stuff
 */
void setupSPI()
{
	//Setup CS Output Pins
	//CS (Current Slave?) not the same as STE for MSP430
	//Used like a GPIO
	P5OUT |= BIT3;
	P5DIR |= BIT3;

	// Configure GPIO
	    P5SEL1 &= ~(BIT0 | BIT1 | BIT2); // USCI_B1 SCLK, MOSI,
	    P5SEL0 |= (BIT0 | BIT1 | BIT2);  // STE, and MISO pin
	    PJSEL0 |= BIT4 | BIT5;

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
	    UCB1CTLW0 = UCSWRST;                    // **Put state machine in reset**
	                                            // 4-pin, 8-bit SPI master
	    UCB1CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB | UCMODE_2 | UCSTEM;
	                                            // Clock polarity high, MSB
	    //UCB1CTLW0 |= UCSSEL__ACLK;              // ACLK
	    UCB1CTLW0 |= UCSSEL_3;              // SMCLK
	    UCB1BRW = 0x4;                         // /2
	    UCB1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
	    UCB1IE |= UCRXIE;                       // Enable USCI_B1 RX interrupt

	//Done
	return;
}




