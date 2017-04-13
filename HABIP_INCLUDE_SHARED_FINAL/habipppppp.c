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
//  MSP430FR5x9x Demo - eUSCI_A3 UART echo at 9600 baud using BRCLK = 32768Hz
//
//  Description: This demo echoes back characters received via a PC serial port.
//  ACLK is used as a clock source and the device is put in LPM3
//  Note that level shifter hardware is needed to shift between RS232 and MSP
//  voltage levels.
//
//  The example code shows proper initialization of registers
//  and interrupts to receive and transmit data.
//  To test code in LPM3, disconnect the debugger and use an external supply
//  Otherwise use LPM0
//
//  ACLK = 32768Hz, MCLK =  SMCLK = default DCO
//
//                MSP430FR5994
//             -----------------
//            |                 |--LFXTIN (32768Hz reqd.)
//            |                 |--LFXTOUT
//            |                 |
//       RST -|     P6.0/UCA3TXD|----> PC (echo)
//            |                 |
//            |                 |
//            |     P6.1/UCA3RXD|<---- PC
//            |                 |
//
//   William Goh
//   Texas Instruments Inc.
//   October 2015
//   Built with IAR Embedded Workbench V6.30 & Code Composer Studio V6.1
//******************************************************************************
#include <msp430.h>
#include "driverlib.h" //added for blink LED
#include "habip.h"
/*
// UD Functions

	// Clocks
void config_XT1_GPIO(void);
void config_XT1_ACLK_32768Hz(void);
void config_XT1_ACLK_32768Hz_DCO_1MHz(void);
void config_DCO_8MHz(void);
void config_DCO_1MHz(void);

	// UART
void config_UART_4_GPIO(void);
void config_UART_4_9600_ACLK_32768Hz(void);
void config_UART_4_9600_SMCLK_8MHz(void);
void UART_read_msg(void);
void UART_write_msg(char* message);
void chris_init(void);

	// SPI
void config_SPI_B0_Master_GPIO(void);
void config_SPI_B0_Master(void);
void config_SPI_B1_Slave_GPIO(void);
void config_SPI_B1_Slave(void);
void config_SPI_A0_Slave_GPIO(void);
void config_SPI_A0_Slave(void);
void SPI_read_msg(void);
void SPI_write_msg(char* message);

	// General
void activate_GPIO_config(void);
void config_DS4_LED(void);
void Toggle_ON_OFF_DS4_LED(void);
void delay_LED(void);

// UART UD Constants
//const int EOT=0x7D; // End of Transmission ASCII Character as per Protocol Format
const int EOT=4; // End of Transmission ASCII Character
#define MSG_LEN 64 // Default for now
*/
// UART UD Variables
extern volatile char uart_read_buffer[MSG_LEN];
extern volatile char uart_read_message[MSG_LEN];
extern volatile char uart_index;
extern volatile int uart_readDoneFG ;

// SPI UD Variables
//volatile char RXData = '\0';
//volatile char TXData = '\0';
extern volatile char spi_read_buffer[MSG_LEN];
extern volatile char spi_read_message[MSG_LEN];
extern volatile char spi_tx_msg[MSG_LEN];
extern volatile char spi_index;
extern volatile char msg_index;
extern volatile char spi_read_index;
extern volatile int spi_read_enable;
extern volatile int spi_readDoneFG;
extern volatile char TXDATA;
extern volatile char RXDATA;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_DS4_LED();
    // SPI GPIO
    config_SPI_B0_Master_GPIO();
//    P1SEL0 &= ~(BIT3);
//    P1SEL1 &= ~(BIT3);
//    P1DIR |= (BIT3);
    config_XT1_GPIO();						// XT1 Crystal

// Configure Clock
    config_XT1_ACLK_32768Hz_DCO_1MHz();

// Configure SPI
    config_SPI_B0_Master();

//// Begin Main Code
////    P1OUT |= (BIT3);
//    SPI_write_msg("Hi from Host");
//    TXDATA = 0;
//    UCB0IE |= UCTXIE;
//    SPI_read_msg();
////    P1OUT &= ~(BIT3);
//// End Main Code
//    TXDATA = 0x30;                           // Holds TX data
    TXDATA = spi_tx_msg[msg_index];
    volatile int dummy_values = 0;
    while(spi_readDoneFG == 0)
    {
    	UCB0IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt
        __no_operation();                   // Remain in LPM0
        __delay_cycles(2000);               // Delay before next transmission
        if(dummy_values == 0){
        	if(TXDATA == 0x7D){
        		dummy_values == 1;
        	}
        	else{
        		TXDATA = spi_tx_msg[++msg_index];                           // Increment transmit data
        	}
        }
        else if (dummy_values == 1){
        	TXDATA = 0x58;
        }
    }
	while(1) ; // catchall for debug
}
//*********************************************************************************************************//
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A3_VECTOR))) USCI_A3_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA3IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            uart_read_buffer[uart_index] = UCA3RXBUF;
//            UCA3TXBUF = uart_read_buffer[uart_index];
            if((uart_read_buffer[uart_index] == END_CHAR)||(uart_index == MSG_LEN-1)) {
            	unsigned int i;
            	for(i = 0; i < uart_index; i++) {
            		uart_read_message[i]=uart_read_buffer[i];
            	}
            	uart_read_message[uart_index]='\0';
            	uart_index = 0;
            	uart_readDoneFG = 1;
            } else {
            	uart_index++;
            }
            __no_operation();
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
//            RXData = UCA0RXBUF;
            UCA0IFG &= ~UCRXIFG;
//            __bic_SR_register_on_exit(LPM0_bits); // Wake up to setup next TX
//            break;
            spi_read_buffer[spi_index] = UCA0RXBUF;
//            UCA0TXBUF = spi_read_buffer[spi_index];
			if((spi_read_buffer[spi_index] == END_CHAR)||(spi_index == MSG_LEN-1)) {
				unsigned int i;
				for(i = 0; i < spi_index; i++) {
					spi_read_message[i]=spi_read_buffer[i];
				}
				spi_read_message[spi_index]='\0';
				spi_index = 0;
				spi_readDoneFG = 1;
			} else {
				spi_index++;
			}
			__no_operation();
			break;
        case USCI_SPI_UCTXIFG:
//            UCA0TXBUF = TXData;                   // Transmit characters
//            UCA0IE &= ~UCTXIE;
            break;
        default: break;
    }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
//    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
//    {
//        case USCI_NONE: break;
//        case USCI_SPI_UCRXIFG:
////            RXData = UCB0RXBUF;
////            UCB0IFG &= ~UCRXIFG;
////            __bic_SR_register_on_exit(LPM0_bits); // Wake up to setup next TX
////            break;
//            spi_read_buffer[spi_index] = UCB0RXBUF;
////            UCB0TXBUF = spi_read_buffer[spi_index];
//			if((spi_read_buffer[spi_index] == EOT)||(spi_index == MSG_LEN-1)) {
//				unsigned int i;
//				for(i = 0; i < spi_index; i++) {
//					spi_read_message[i]=spi_read_buffer[i];
//				}
//				spi_read_message[spi_index]='\0';
//				spi_index = 0;
//				spi_readDoneFG = 1;
//			} else {
//				spi_index++;
//			}
//			__no_operation();
//			break;
//        case USCI_SPI_UCTXIFG:
//            UCB0TXBUF = TXDATA;                   // Transmit characters
//            TXDATA = TXDATA + 1;
////            UCB0IE &= ~UCTXIE;
//            break;
//        default: break;
//    }
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
        	spi_read_buffer[spi_index] = UCB0RXBUF;
        	if(spi_read_enable == 0){
        		if(spi_read_buffer[spi_index] == 0x7B){
					spi_read_enable = 1;
				}
        	}
        	if(spi_read_enable == 1){
        		spi_read_message[spi_read_index] = spi_read_buffer[spi_index];
        		if(spi_read_message[spi_read_index] == 0x7D){
        			// Insert clear stuff later
        			spi_readDoneFG = 1;
        			UCB0IE &= ~UCRXIE;
        		}
        		spi_read_index++;
        	}

        	spi_index++;
            UCB0IFG &= ~UCRXIFG;

            // Wake up to setup next TX
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        case USCI_SPI_UCTXIFG:
            UCB0TXBUF = TXDATA;             // Transmit characters
            UCB0IE &= ~UCTXIE;
            break;
        default: break;
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
//            RXData = UCB1RXBUF;
            UCB1IFG &= ~UCRXIFG;
//            __bic_SR_register_on_exit(LPM0_bits); // Wake up to setup next TX
//            break;
            spi_read_buffer[spi_index] = UCB1RXBUF;
//            UCB1TXBUF = spi_read_buffer[spi_index];
			if((spi_read_buffer[spi_index] == END_CHAR)||(spi_index == MSG_LEN-1)) {
				unsigned int i;
				for(i = 0; i < spi_index; i++) {
					spi_read_message[i]=spi_read_buffer[i];
				}
				spi_read_message[spi_index]='\0';
				spi_index = 0;
				spi_readDoneFG = 1;
			} else {
				spi_index++;
			}
			__no_operation();
			break;
        case USCI_SPI_UCTXIFG:
//            UCB1TXBUF = TXData;                   // Transmit characters
//            UCB1IE &= ~UCTXIE;
            break;
        default: break;
    }
}
//*********************************************************************************************************//
