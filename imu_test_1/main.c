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
void setupUART();
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

unsigned int speed = 0xFFFF;
int baseline_captured = 0;
int baseline;

int i = 0;
volatile unsigned int data_array [100];
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

	// UART Setup
	//setupUART();

	while(1){
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
	P1OUT &= (~BIT4);

	/*
	 * Send first byte in a frame
	 */

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 1st byte (header/reg addr)
	UCB0TXBUF = firstByte;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 2nd byte (filler/dont care)
	UCB0TXBUF = secondByte;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Continously read in bytes
	//Dont care what they are
	while ((resultLength > 0))
	{
		//Byte Received?
		while (!(IFG2 & UCB0RXIFG));

		//Store Received data
		receivedChar = UCB0RXBUF;

		//Decrement Result Length
		resultLength--;
	}

	//Reset Result Length
	resultLength = 2;

	/*
	 * Send random byte in separate frame to capture data from first frame
	 */

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 1st byte (random byte 1)
	UCB0TXBUF = firstByte;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 2nd byte (random byte 2)
	UCB0TXBUF = secondByte;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Continously read in bytes
	//ACTUAL data
	while ((resultLength > 0))
	{
		//Byte Received?
		while (!(IFG2 & UCB0RXIFG));

		//Store Received data
		receivedChar = UCB0RXBUF;

		//Append received byte to data
		dataOut = dataOut << 8;
		dataOut = dataOut | receivedChar;

		//First 2 bits are flags/alarms
		dataOut = dataOut & 0x3fff;

		//Decrement Result Length
		resultLength--;
	}

	//Unselect Device
	P1OUT |= (BIT4);

	//Done, return
	return(dataOut);
}

/*
 * Write data to the device at register [registerNumber] and register [registerNumber+1]
 * registerNumber+1: MSB
 * registerNumber: LSB
 * 8 bits each
 */
void writeRegister (unsigned char registerNumber, unsigned int dataToSend)
{
	//Assemble packets
	unsigned char firstByte = WRITEcmd | registerNumber;
	unsigned char secondByte0 = dataToSend & 0x00FF;
	unsigned char secondByte1 = (dataToSend & 0xFF00) >> 8;

	//Loop counter: Number of bytes to send
	int resultLength = 2;

	//Received Data
	unsigned char receivedChar;

	/*
	 * Send first byte in a frame
	 */
	//Select Device Manually (not built in)
	P1OUT &= (~BIT4);

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 1st byte (header/reg address)
	UCB0TXBUF = firstByte;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 2nd byte (data byte 1, LSBs)
	UCB0TXBUF = secondByte0;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Continously read in bytes
	//Dont care what they are
	while (resultLength > 0)
	{
		//Byte Received?
		while (!(IFG2 & UCB0RXIFG));

		//Store Received data
		receivedChar = UCB0RXBUF;

		//Decrement Result Length
		resultLength--;
	}

	//Reset Result Length
	resultLength = 2;

	/*
	 * Send second byte in separate frame
	 */

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 1st byte (header/reg addr)
	//Increment to MSB
	UCB0TXBUF = firstByte + 1;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Send 2nd byte (data byte 2, MSB)
	UCB0TXBUF = secondByte1;

	//USCI_A0 TX Buffer Ready?
	while (!(IFG2 & UCB0TXIFG));

	//Continously read in bytes
	//Dont care what they are
	while ((resultLength > 0))
	{
		//Byte Received?
		while (!(IFG2 & UCB0RXIFG));

		//Store Received data
		receivedChar = UCB0RXBUF;

		//Decrement Result Length
		resultLength--;
	}

	//Unselect Device
	P1OUT |= (BIT4);

	//Done
	return;
}

/*
 * Messy SPI Setup Stuff
 */
void setupSPI()
{
	//Setup CS Output Pins
	//CS (Current Slave?) not the same as STE for MSP430
	//Used like a GPIO
	P1OUT |= BIT4;
	P1DIR |= BIT4;

	//Setup SPI I/O
	P1SEL |= BIT5 + BIT6 + BIT7;
	P1SEL2 |= BIT5 + BIT6 + BIT7;

	//Pause USCI State Machine
	UCB0CTL1 = UCSWRST;

	//CLK Polarity = 1, MSB first, SPI Master, 3 Pin, 8 bit
	//(CLK Active High)s
	UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC;

	//SMCLK = 16MHz
	UCB0CTL1 |= UCSSEL_2;

	//Divide SMClock (16MHz) by 64 = 250 kHz
	//Trial and error with 6" cables, max frequency
	UCB0BR0 = 64;
	UCB0BR1 = 0;

	//No Modulation
	//UCB0MCTL = 0;

	//Initialize USCI State Machine
	UCB0CTL1 &= ~UCSWRST;

	//Done
	return;
}


/*
 * Messy UART Setup Stuff
 */
void setupUART()
{
	//P1.1 = UART RX
	//P1.2 = UART TX
	P1SEL |= BIT1 + BIT2;
	P1SEL2 |= BIT1 + BIT2;
	UCA0CTL0 = 0;
	UCA0STAT = 0;
	UCA0CTL1 |= UCSSEL_2;   // SMCLK
	UCA0BR0 = 139;          // 16MHz => 115200 baud
	UCA0BR1 = 0;

	UCA0MCTL = UCBRS0;      // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;   // Initialize USCI state machine
	IE2 |= UCA0RXIE;        // Enable USCI_A0 RX interrupt
  	__bis_SR_register(GIE);

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
}

/* Get the register number to reply data with */
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	int reg_to_read = UCA0RXBUF;
	int dataReceived = readRegister(reg_to_read);

	//SpinMotor(dataReceived);

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
	//TX
	UCA0TXBUF = hexIntToChar((dataReceived>>12) & 0xf);

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
	//TX
	UCA0TXBUF = hexIntToChar((dataReceived>>8) & 0xf);

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
	//TX
	UCA0TXBUF = hexIntToChar((dataReceived>>4) & 0xf);

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
	//TX
	UCA0TXBUF = hexIntToChar((dataReceived) & 0xf);

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
	//Carraige Return
	UCA0TXBUF = 0xD;

	//Wait for TX buffer to be empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
	//New Line
	UCA0TXBUF = 0xA;

	//Wait for TX Buffer to empty
	while (UCA0TXIFG != (UCA0TXIFG & IFG2));
}

