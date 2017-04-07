//******************************************************************************
//   MSP430FR5x9x Demo - eUSCI_B1, SPI 3-Wire Master Incremented Data
//
//   Description: SPI master talks to SPI slave using 3-wire mode. Incrementing
//   data is sent by the master starting at 0x01. Received data is expected to
//   be same as the previous transmission TXData = RXData-1.
//   USCI RX ISR is used to handle communication with the CPU, normally in LPM0.
//   ACLK = 32.768kHz, MCLK = SMCLK = DCO ~1MHz.  BRCLK = ACLK/2
//
//
//                   MSP430FR5994
//                 -----------------
//            /|\ |              XIN|-
//             |  |                 |  32KHz Crystal
//             ---|RST          XOUT|-
//                |                 |
//                |             P5.0|-> Data Out (UCB1SIMO)
//                |                 |
//                |             P5.1|<- Data In (UCB1SOMI)
//                |                 |
//                |             P5.2|-> Serial Clock Out (UCB1CLK)

//******************************************************************************
#include <msp430.h>

int read_IMU_SPI(unsigned char register_address);
void setup_IMU_SPI(void);

volatile unsigned char RXData = 0;
volatile unsigned char RXData2 = 0;
volatile unsigned char TXData;

int i = 0;
signed int data_array [1000];

//Sensor's Read Register Adddresses
const unsigned char POWERSUPPLY = 0x02;            	// Power Supply Voltage
const unsigned char XGYRO = 0x04;                  	// X Gyro Measurement
const unsigned char YGYRO = 0x06;                  	// Y Gyro Measurement
//const unsigned char ZGYRO = 0x0008;                  	// Z Gyro Measurement
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

    setup_IMU_SPI();

    int z_gyro_data;

    while(1)
    {
    	z_gyro_data = read_IMU_SPI(ZGYRO);
    	data_array[i] = z_gyro_data;
    	i++;
    	__delay_cycles(2000);
    }
}


void setup_IMU_SPI(void){

    // Configure GPIO
    P5SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3); // USCI_B1 SCLK, MOSI,
    P5SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3);  // STE, and MISO pin
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
    UCB1CTLW0 |= UCSSEL_3;              // ACLK
    UCB1BRW = 0x4;                         // /2
    UCB1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCB1IE |= UCRXIE;                       // Enable USCI_B1 RX interrupt

}


int read_IMU_SPI(unsigned char register_address){
	signed int dataOut = 0;
	// ADIS16350's read filler (Dont care bits after register addr)
	const unsigned char READFILLER = 0x5A;

	//write the address you want to read
	TXData = register_address;
	UCB1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

    //send filler data to make total data frame 16 bits
    TXData = READFILLER;
    UCB1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

    //second time around, this time we will get the data. Need to clock data again so send the same stuff again.
    TXData = register_address;
	UCB1IE |= UCTXIE;
	__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

	// acquire first data frame from RX
	dataOut = RXData;

	// shift the result to the MSB of 16bit dataOut
	dataOut = dataOut << 8;

	//send next data frame to receive the final output
	TXData = READFILLER;
	UCB1IE |= UCTXIE;
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


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_B1_VECTOR))) USCI_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB1IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
        	RXData= UCB1RXBUF;
        	UCB1IFG &= ~UCRXIFG;
            __bic_SR_register_on_exit(LPM0_bits);
            break;

        case USCI_SPI_UCTXIFG:
            UCB1TXBUF = TXData;             // Transmit characters
            UCB1IE &= ~UCTXIE;
            break;
        default: break;
    }
}
