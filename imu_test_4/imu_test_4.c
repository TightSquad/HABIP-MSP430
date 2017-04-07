/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
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
//
//   William Goh
//   Texas Instruments Inc.
//   October 2015
//   Built with IAR Embedded Workbench V6.30 & Code Composer Studio V6.1
//******************************************************************************
#include <msp430.h>
#include  <stdint.h>

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
const unsigned char READFILLER = 0x5A;             	// ADIS16350's read filler (Dont care bits after register addr)

signed int dataOut = 0;
int byte_num = 0;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer


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
    //TXData = 0x1;                           // Holds TX data

    while(1)
    {

    	//TXData = 0xBE;
    	TXData = ZGYRO;
    	UCB1IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

        dataOut = RXData;
        dataOut = dataOut << 8;
//        data_array[i] = dataOut;
//        i++;

//        TXData = 0x82;
        TXData = READFILLER;
        UCB1IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt


        dataOut = dataOut | RXData;
		//First 2 bits are flags/alarms
		dataOut = dataOut & 0x3fff;
		if(dataOut & 0x2000){
			dataOut = dataOut | 0xC000;
		}
        //dataOut = RXData;
		data_array[i] = dataOut;
		i++;

		dataOut = 0;


        TXData = ZGYRO;
		UCB1IE |= UCTXIE;
		__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt

		dataOut = RXData;
		dataOut = dataOut << 8;

		//dataOut = RXData;
//		data_array[i] = dataOut;
//		i++;

		//dataOut = RXData;
		//dataOut = dataOut << 8;

		TXData = READFILLER;
		UCB1IE |= UCTXIE;
		__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt


		dataOut = dataOut | RXData;
		//First 2 bits are flags/alarms
		dataOut = dataOut & 0x3fff;
		if(dataOut & 0x2000){
			dataOut = dataOut | 0xC000;
		}
		data_array[i] = dataOut;
		i++;
//
		dataOut = 0;

//		dataOut = RXData;
//		data_array[i] = dataOut;
//		i++;

//		dataOut = dataOut | RXData;
//		//First 2 bits are flags/alarms
//		dataOut = dataOut & 0x3fff;
//		data_array[i] = dataOut;
//		i++;
//		dataOut = 0;

//        //Append received byte to data
//		dataOut = dataOut << 8;
//		dataOut = dataOut | RXData;
//        if(byte_num == 1){
//        	data_array[i] = dataOut;
//        	i++;
//        	byte_num = 0;
//        }
        __no_operation();                   // Remain in LPM0
        __delay_cycles(2000);               // Delay before next transmission
        //TXData++;                           // Increment transmit data
    }
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
        	//RXData2= UCB1RXBUF;
        	byte_num++;
        	UCB1IFG &= ~UCRXIFG;
            __bic_SR_register_on_exit(LPM0_bits);
            break;

        case USCI_SPI_UCTXIFG:
            UCB1TXBUF = TXData;             // Transmit characters
            //while(!(UCB1IFG & UCTXIFG))
            //UCB1IE &= ~UCTXIE;
//            UCB1TXBUF = READFILLER;
            UCB1IE &= ~UCTXIE;
////            while(!(UCB1IFG & UCTXIFG))
            break;
        default: break;
    }
}
