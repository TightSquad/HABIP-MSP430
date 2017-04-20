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
#include "string.h"

// UART UD Variables
extern volatile char uart_read_buffer[MSG_LEN];
extern volatile char uart_read_message[MSG_LEN];
extern volatile char uart_index;
//extern volatile int uart_readDoneFG;

// SPI UD Variables
extern volatile char TXDATA;
extern char spi_read_buffer[BUFF_LEN];
extern char spi_read_message[MSG_LEN];
extern char spi_send_message[MSG_LEN];
extern char spi_send_buffer[BUFF_LEN];
extern volatile int msg_return;	//when to respond to SPI master
extern volatile int spi_fsm_state;
extern volatile int spi_index;
extern volatile int spi_req_data;
extern volatile int spi_data_available;
extern volatile int spi_read_index;
extern volatile int spi_write_index;
extern volatile int spi_readDoneFG;

char* sensor_responses[DAQCS_SENSOR_CNT] = {"{B4:TB0:1110}",
							"{B4:P0:1111}",
							"{B4:PB:1112}",
							"{B4:V:1113}",
							"{B4:C:1114}",
							"{B4:XGY:1115}",
							"{B4:XAC:1116}",
							"{B4:YGY:1117}",
							"{B4:YAC:1118}",
							"{B4:ZGY:1119}",
							"{B4:ZAC:1120}",
							"{B4:MS:1121}",
							"{B4:MC:1122}",
							"{B4:MV:1123}",
							"{B4:MD:1124}",
							"{B4:ME:1125}"

};
int count = 0;
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_DS4_LED();
    // SPI GPIO
    config_SPI_A0_Slave_GPIO();
    config_XT1_GPIO();						// XT1 Crystal

// Configure Clock
    config_XT1_ACLK_32768Hz_DCO_1MHz();

// Configure SPI
    config_SPI_A0_Slave();

    UCA0TXBUF = 0x58;

while(1){
    __bis_SR_register(LPM0_bits | GIE);

    __no_operation();
    __delay_cycles(200);
    if(spi_req_data == 1){
    	strcpy(spi_send_message,sensor_responses[count++]);
    	spi_data_available = 1;
    	if(count == DAQCS_SENSOR_CNT){
    		count = 0;
    	}
    }
//    __bis_SR_register(LPM0_bits | GIE);
// Begin Main Code
//    SPI_read_msg();
//    SPI_write_msg("Hi from Slave");
// End Main Code
}
	while(1) ; // catchall for debug
}
//*********************************************************************************************************//
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
        	while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
        	spi_read_buffer[spi_index] = UCA0RXBUF;
        	TXDATA = 0x58;
// LISTENING_FOR_COMMAND
        	if(spi_fsm_state == LISTENING_FOR_COMMAND){
				if(spi_read_buffer[spi_index] == 0x7B){
					spi_fsm_state = CAPTURING_COMMAND;
					spi_read_index = 0;
				}
				else{
					spi_index++;
				}
        	}
// CAPTURING_COMMAND
        	if(spi_fsm_state == CAPTURING_COMMAND){
        		spi_read_message[spi_read_index] = spi_read_buffer[spi_index];
				if(spi_read_message[spi_read_index] == 0x7D){
					spi_fsm_state = OBTAINING_DATA;
					spi_req_data = 1;
					__bic_SR_register_on_exit(LPM0_bits | GIE);
				}
				else{
					spi_read_index++;
					spi_index++;
				}
			}
// OBTAINING_DATA
        	if(spi_fsm_state == OBTAINING_DATA){
				if(spi_data_available == 1){
					spi_fsm_state = SENDING_DATA;
					spi_data_available = 0;
					spi_req_data = 0;
					spi_write_index = 0;
				}
				else{
					spi_index++;
				}
			}
// SENDING_DATA
        	if(spi_fsm_state == SENDING_DATA){
				TXDATA = spi_send_message[spi_write_index];
        		if(spi_send_message[spi_write_index] == 0x7D){
        			spi_fsm_state = LISTENING_FOR_COMMAND;
        			spi_index++;
        		}
        		else{
        			spi_write_index++;
        			spi_index++;
        		}
        	}
        	UCA0TXBUF = TXDATA;
        	spi_send_buffer[spi_index-1] = TXDATA;
        	if(spi_index == BUFF_LEN){
        		spi_index = 0;
        	}
			break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}
//*********************************************************************************************************//
