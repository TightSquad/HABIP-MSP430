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

// SPI UD Variables
// Slave (slv)
extern volatile int spi_slv_fsm_state;
extern char spi_slv_read_buffer[BUFF_LEN];
extern char spi_slv_read_message[MSG_LEN];
extern char spi_slv_send_message[MSG_LEN];
extern char spi_slv_send_buffer[BUFF_LEN];
extern volatile int spi_slv_index;
extern volatile int spi_slv_req_data;
extern volatile int spi_slv_data_available;
extern volatile int spi_slv_write_index;
extern volatile int spi_slv_read_index;
extern volatile char spi_slv_tx_data;



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

    __bis_SR_register(GIE);

      // Temp Fill with dummy values.
      store_response_val(4,"P0","1111"); // pressure on board
      store_response_val(4,"PB","1112"); // pressure on balloon
      store_response_val(4,"V","1113"); // board voltage
      store_response_val(4,"C","1114"); // board current
      store_response_val(4,"XGY","1115");
      store_response_val(4,"XAC","1116");
      store_response_val(4,"YGY","1117");
      store_response_val(4,"YAC","1118");
      store_response_val(4,"ZGY","1119");
      store_response_val(4,"ZAC","1120");
      store_response_val(4,"MS","1121"); //motor speed
      store_response_val(4,"MC","1122"); //motor current
      store_response_val(4,"MD","1124"); //motor direction
      store_response_val(4,"ME","1125"); //motor enable/status

// Begin Main Code
	while(1){
		__no_operation();
		__bis_SR_register(LPM0_bits);
  		__no_operation();
  		toggle_heartbeat();
              if(spi_slv_fsm_state == PARSING_COMMAND){
                parse_cmd_from_host(spi_slv_read_message);
              }
              store_response_val(4,"ZGY","6969");
	}
// End Main Code

	// while(1) ; // catchall for debug
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
        	spi_slv_read_buffer[spi_slv_index] = UCA0RXBUF;
             switch(spi_slv_fsm_state)
             {
                case LISTENING_FOR_COMMAND:
                    if(spi_slv_read_buffer[spi_slv_index] == '{'){
                      spi_slv_fsm_state = CAPTURING_COMMAND;
                      spi_slv_read_message[0] = '{';
                      spi_slv_read_index = 1;
                      TX_A0_SPI('C');
                    }
                    else{
//                        TX_A0_SPI('L');
                    	UCA0TXBUF = 'L';
                    }
                    break;
                case CAPTURING_COMMAND:
                    spi_slv_read_message[spi_slv_read_index++] = spi_slv_read_buffer[spi_slv_index];
                    if(spi_slv_read_buffer[spi_slv_index] == '}'){
                      spi_slv_fsm_state = PARSING_COMMAND;
                      spi_slv_read_message[spi_slv_read_index] = '\0';
                      spi_slv_write_index = 0;
                      TX_A0_SPI('D');
                      __bic_SR_register_on_exit(LPM0_bits);
                    }
                    else {
                      TX_A0_SPI('C');
                    }
                    break;
                case PARSING_COMMAND:
                    TX_A0_SPI('P');
                    break;
                case OBTAINING_DATA:
                    TX_A0_SPI('O');
                    break;
                case RESPONDING_WITH_DATA:
                    spi_slv_tx_data = spi_slv_send_message[spi_slv_write_index++];
                    if(spi_slv_tx_data == '}'){
                      spi_slv_fsm_state = LISTENING_FOR_COMMAND;
                    }
                    TX_A0_SPI(spi_slv_tx_data);
                    break;
//                case RESPONDING_WITH_ALL_DATA:
//                    spi_slv_tx_data = respond_all_data_msg[spi_slv_write_index++];
//                    if(spi_slv_tx_data == '$'){
//                      spi_slv_fsm_state = LISTENING_FOR_COMMAND;
//                    }
//                    else {
//                        TX_A0_SPI(spi_slv_tx_data);
//                    }
//                    break;
                default:

                    break;
             }
        	if(spi_slv_index == BUFF_LEN-1){
        		spi_slv_index = 0;
        	}
              else {
                spi_slv_index++;
              }
			break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}
//*********************************************************************************************************//
