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
#include "driverlib.h"
#include "habip.h"

//// UD Functions
//void activate_GPIO_config(void);
//
//void config_XT1_GPIO(void);
//void config_XT1_ACLK_32768Hz(void);
//void config_DCO_8MHz(void);
//void config_DCO_1MHz(void);
//
//void config_UART_4_GPIO(void);
//void config_UART_4_9600_ACLK_32768Hz(void);
//void config_UART_4_9600_SMCLK_8MHz(void);
//void UART_read_msg(void);
//void UART_write_msg(char* message);
//void chris_init(void);
//
//void config_DS4_LED(void);
//void Toggle_ON_OFF_DS4_LED(void);
//void delay_LED(void);


//// UART UD Constants
//#define END_CHAR 0x7D // End of Transmission ASCII Character as per Protocol Format
//#define MSG_LEN 64 // Default for now
//
//#define LISTENING_FOR_RESPONSE 0x00
//#define CAPTURING_RESPONSE 0x01

// UART UD Variables
extern volatile char uart_read_buffer[MSG_LEN];
extern volatile char uart_read_message[MSG_LEN];
extern volatile int uart_index;
extern volatile int uart_readDoneFG;
extern volatile int uart_fsm_state;
extern volatile int uart_read_index;
//*********************************************************************************************************//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_DS4_LED();
    config_UART_4_GPIO();
//    config_XT1_GPIO();						// XT1 Crystal

// Configure Clock
//    config_XT1_ACLK_32768Hz();
    config_DCO_8MHz();

// Configure UART
//    config_UART_4_9600_ACLK_32768Hz();
    config_UART_4_9600_SMCLK_8MHz();

    __bis_SR_register(GIE);

// Begin Main Code
    UART_read_msg();
    UART_write_msg("Hi from MSP");
// End Main Code

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
        	while (!(UCA3IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
        	uart_read_buffer[uart_index] = UCA3RXBUF;
// LISTENING_FOR_RESPONSE
        	if(uart_fsm_state == LISTENING_FOR_RESPONSE){
				if(uart_read_buffer[uart_index] == 0x7B){
					uart_fsm_state = CAPTURING_RESPONSE;
					uart_read_index = 0; // May cause overwriting in future
				}
				else{
					uart_index++;
				}
        	}
// CAPTURING_RESPONSE
        	if(uart_fsm_state == CAPTURING_RESPONSE){
        		uart_read_message[uart_read_index] = uart_read_buffer[uart_index];
				if(uart_read_message[uart_read_index] == 0x7D){
					uart_fsm_state = LISTENING_FOR_RESPONSE;
					uart_readDoneFG = 1; // reset where?
				}
				else{
					uart_read_index++;
					uart_index++;
				}
			}
        	if(uart_index == MSG_LEN){
        		uart_index = 0;
        	}
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
