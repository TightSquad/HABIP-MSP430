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
#include "stdlib.h"
#include <string.h>

// UART UD Variables
//extern char uart_read_buffer[4][MSG_LEN];
extern char uart_read_message[4][MSG_LEN];
extern int uart_fsm_state[4];
extern volatile int RXSWFG0;
extern volatile int RXSWFG1;
extern volatile int RXSWFG2;
extern volatile int RXSWFG3;

char* commands[3] = {	"{00:B0:V}",
						"{00:B0:V}",
						"{00:B0:C}"
					};//temp

char* command_PI[4] = {	"00:", // Grab Sensor Data
				"01", // Grab all sensor data
				"05:", // Board Reset
				"06:" // Time Sync
			};
char* command_DAQCS_host[4] = {	"00:", // Grab Sensor Data
					"01", // Grab all sensor data
					"06:", // Time Sync
					"FF" // Trigger Cutdown
			};
char* command_DAQCS_motor[5] = {	"00:", // Grab Sensor Data
						"01", // Grab all sensor data
						"03:", // Reaction Wheel On/Off
						"04", // Reaction Wheel Control (degrees)
						"06:", // Time Sync
			};
char* board_list[5] = {	"B0", // Pi Hat 0 - UCA0 - "UART_1" - J1
			"B1", // Pi Hat 1 - UCA1 - "UART_2" - J2
			"B2", // Pi Hat 2 - UCA2 - "UART_3" - J3
			"B3", // Pi Hat 3 - UCA3 - "UART_4" - J4
			"B4" // Self / Motor MSP - Host-UCB0 - Motor-UCA0 SPI
		};

//temp?
extern char response_buffer_b0[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b1[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b2[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b3[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b4[DAQCS_SENSOR_CNT][MSG_LEN];

extern char response_status_b0[PI_HAT_SENSOR_CNT];
extern char response_status_b1[PI_HAT_SENSOR_CNT];
extern char response_status_b2[PI_HAT_SENSOR_CNT];
extern char response_status_b3[PI_HAT_SENSOR_CNT];
extern char response_status[4][PI_HAT_SENSOR_CNT];
extern char response_status_b4[DAQCS_SENSOR_CNT];

//*********************************************************************************************************//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_UART_GPIO(0);
    config_UART_GPIO(1);
    config_UART_GPIO(2);
    config_UART_GPIO(3);

// Configure Clock
    config_DCO_8MHz();

// Configure UART
    config_UART_9600_SMCLK_8MHz(0);
    config_UART_9600_SMCLK_8MHz(1);
    config_UART_9600_SMCLK_8MHz(2);
    config_UART_9600_SMCLK_8MHz(3);

    __bis_SR_register(GIE);

// Begin Main Code
//    char msg[MSG_LEN];
//    strcpy(msg,"{00:B0:TD0}")
while(1){
	__delay_cycles(8000000);
//	delay_LED();
	if(RXSWFG0){
		UART_read_response(0,&RXSWFG0);
		UART_write_msg(0,uart_read_message[0]);
		parse_response(uart_read_message[0]);
//		parse_cmd_from_comms(uart_read_message[0]);
	}
	else if(RXSWFG1){
		UART_read_response(1,&RXSWFG1);
		UART_write_msg(1,uart_read_message[1]);
		parse_response(uart_read_message[1]);
//		parse_cmd_from_comms(uart_read_message[1]);
	}
	else if(RXSWFG2){
		UART_read_response(2,&RXSWFG2);
		UART_write_msg(2,uart_read_message[2]);
		parse_response(uart_read_message[2]);
//		parse_cmd_from_comms(uart_read_message[2]);
	}
	else if(RXSWFG3){
		UART_read_response(3,&RXSWFG3);
		UART_write_msg(3,uart_read_message[3]);
		parse_response(uart_read_message[3]);
//		parse_cmd_from_comms(uart_read_message[3]);
	}
	else {
		__no_operation();
	}
	__no_operation();
}
// End Main Code

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
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	RXSWFG0 = 1;
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
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
    switch(__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	RXSWFG1 = 1;
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A2_VECTOR))) USCI_A2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        	RXSWFG2 = 1;
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
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
        	RXSWFG3 = 1;
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
//*********************************************************************************************************//
