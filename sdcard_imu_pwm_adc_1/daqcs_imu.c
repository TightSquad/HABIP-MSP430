/*
 * daqcs_imu.c
 *
 *  Created on: Apr 11, 2017
 *      Author: stevy
 */


#include "driverlib.h"

volatile unsigned char RXData = 0;
volatile unsigned char RXData2 = 0;
volatile unsigned char TXData;
volatile unsigned int RX_flag = 0;


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
    CSCTL3 = DIVA__16 | DIVA__16 | DIVA__16;   // Set all dividers
    CSCTL4 &= ~LFXTOFF;
    do
    {
        CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers

    // Configure USCI_A1 for SPI operation
    UCA1CTLW0 = UCSWRST;                    // **Put state machine in reset**
                                            // 4-pin, 8-bit SPI master
    UCA1CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB | UCMODE_2 | UCSTEM;
                                            // Clock polarity high, MSB
    //UCB1CTLW0 |= UCSSEL__ACLK;              // ACLK
    UCA1CTLW0 |= UCSSEL_2;              	// SMCLK
    UCA1BRW = 0x04;                         // /4
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt

}


void write_IMU_SPI(unsigned int register_address, unsigned char date_write){
	signed int dataOut = 0;
	// ADIS16350's read filler (Dont care bits after register addr)
	const unsigned char READFILLER = 0x5A;

	register_address = register_address | 0x80;

	//write the address you want to read
	TXData = register_address;
	UCA1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

    //send filler data to make total data frame 16 bits
    TXData = date_write;
    UCA1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

	return;
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
