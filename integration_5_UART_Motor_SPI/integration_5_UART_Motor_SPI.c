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

//******************************************************************************
#include <msp430.h>
#include "driverlib.h"
#include "habip.h"
#include "stdlib.h"
#include <string.h>

// UART UD Variables
extern char uart_read_buffer[4][MSG_LEN];
extern char uart_read_message[4][MSG_LEN];
extern int uart_fsm_state[4];
extern volatile int RXSWFG0;
extern volatile int RXSWFG1;
extern volatile int RXSWFG2;
extern volatile int RXSWFG3;
extern char uart_read_message_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN];
extern char uart_read_message_buffer_status[4][PI_HAT_SENSOR_CNT];
extern char uart_interrupt_style[4];
extern volatile int msg_buffer_index[4];

// SPI UD Variables
// Master (mst)
extern volatile int spi_mst_fsm_state;
extern char spi_mst_read_buffer[BUFF_LEN];
extern char spi_mst_read_message[MSG_LEN];
extern char spi_mst_send_message[MSG_LEN];
extern char spi_mst_send_buffer[BUFF_LEN];
extern volatile int spi_mst_index;
extern volatile int spi_mst_write_index;
extern volatile int spi_mst_read_index;
extern volatile char spi_mst_tx_data;
extern volatile int spi_mst_read_enable; // obsolete?
extern volatile int spi_mst_currently_writing; // obsolete?
extern volatile int spi_mst_readDoneFG;

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
extern volatile int spi_slv_readDoneFG;

extern volatile int uart_index[4];
extern volatile int uart_read_index[4];
extern volatile int uart_status_index[4];
extern volatile int uart_status[4];

volatile int count = 0;

//*********************************************************************************************************//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_UART_GPIO(0);
    config_UART_GPIO(1);
    config_UART_GPIO(2);
    config_UART_GPIO(3);
    config_SPI_B0_Master_GPIO();
    config_XT1_GPIO();


// temp
    P1DIR |= (BIT2);
    P1OUT &= ~(BIT2);
    P7DIR |= (BIT0);
    P7OUT &= ~(BIT0);

// Configure Clock
    config_ACLK_XT1_32KHz_DCO_8MHz_SMCLK_250KHz();

// Configure UART
    config_UART_9600_SMCLK_250KHz(0);
    config_UART_9600_SMCLK_250KHz(1);
    config_UART_9600_SMCLK_250KHz(2);
    config_UART_9600_SMCLK_250KHz(3);
    config_SPI_B0_Master_SMCLK();

    __no_operation();
    __bis_SR_register(GIE);

// Begin Main Code
while(1){
	UART_parse(0);
	UART_parse(1);
	UART_parse(2);
	UART_parse(3);
	SPI_command_host_to_slave("{00:B4:ZGY}",&spi_readDoneFG,&TXDATA);
	parse_response(spi_read_message);
	__no_operation();
}
// End Main Code

//	while(1) ; // catchall for debug
}
//*********************************************************************************************************//
//*********************************		TALK TO PI HATS UART	  *****************************************//
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
        	P1OUT |= (BIT2);
        	if(uart_interrupt_style[0] == NO_BUFFER){
        		RXSWFG0 = 1;
        	}
        	else if (uart_interrupt_style[0] == BUFFER){
        		uart_read_buffer[0][uart_index[0]] = UCA0RXBUF;
        		if(uart_status[0] >= PI_HAT_SENSOR_CNT){
        			uart_fsm_state[0] = MSG_BUFFER_FULL;
        		}
        	// MSG_BUFFER_FULL
				if(uart_fsm_state[0] == MSG_BUFFER_FULL){
					if(uart_status[0] < PI_HAT_SENSOR_CNT){
						uart_fsm_state[0] = LISTENING_FOR_RESPONSE;
					}
					else {
						while(!(UCA0IFG&UCTXIFG));
							UCA0TXBUF = 'F'; // TODO: Chris OK with?
							uart_index[0]++;
					}
				}
			// LISTENING_FOR_RESPONSE
				if(uart_fsm_state[0] == LISTENING_FOR_RESPONSE){
					if(uart_read_buffer[0][uart_index[0]] == 0x7B){
						// find open buffer slot and if none found, go to buffer full state
						for(uart_status_index[0] = 0; uart_status_index[0] <PI_HAT_SENSOR_CNT; uart_status_index[0]++){
							if(uart_read_message_buffer_status[0][uart_status_index[0]]==AVAILABLE){
								uart_fsm_state[0] = CAPTURING_RESPONSE;
								uart_read_index[0] = 0; // May cause overwriting in future
								msg_buffer_index[0] = uart_status_index[0];
								break;
							}
							else if(uart_status_index[0] == PI_HAT_SENSOR_CNT-1){
								uart_fsm_state[0] = MSG_BUFFER_FULL;
							}
						}
					}
					else{
						while(!(UCA0IFG&UCTXIFG));
						UCA0TXBUF = 'L';
						uart_index[0]++;
					}
				}
			// CAPTURING_RESPONSE
				if(uart_fsm_state[0] == CAPTURING_RESPONSE){
					uart_read_message_buffer[0][msg_buffer_index[0]][uart_read_index[0]] = uart_read_buffer[0][uart_index[0]];
					if(uart_read_message_buffer[0][msg_buffer_index[0]][uart_read_index[0]] == 0x7D){// TODO: If end of msg len stop reading in...
						uart_fsm_state[0] = LISTENING_FOR_RESPONSE;
						uart_read_message_buffer[0][msg_buffer_index[0]][uart_read_index[0]+1] = '\0';
						uart_read_message_buffer_status[0][uart_status_index[0]]=VALID;
						msg_buffer_index[0]++;
						uart_status[0]++;
						while(!(UCA0IFG&UCTXIFG));
						UCA0TXBUF = 'D';
					}
					else{
						uart_read_index[0]++;
						uart_index[0]++;
						while(!(UCA0IFG&UCTXIFG));
						UCA0TXBUF = 'C';
					}
				}
				if(uart_index[0] == MSG_LEN){
					uart_index[0] = 0;
				}
        	}
        	P1OUT &= ~(BIT2);
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
        	if(uart_interrupt_style[1] == NO_BUFFER){
        		RXSWFG1 = 1;
        	}
        	else if (uart_interrupt_style[1] == BUFFER){
        		uart_read_buffer[1][uart_index[1]] = UCA1RXBUF;
        		if(uart_status[1] >= PI_HAT_SENSOR_CNT){
        			uart_fsm_state[1] = MSG_BUFFER_FULL;
        		}
        	// MSG_BUFFER_FULL
				if(uart_fsm_state[1] == MSG_BUFFER_FULL){
					if(uart_status[1] < PI_HAT_SENSOR_CNT){
						uart_fsm_state[1] = LISTENING_FOR_RESPONSE;
					}
					else {
						while(!(UCA1IFG&UCTXIFG));
							UCA1TXBUF = 'F'; // TODO: Chris OK with?
							uart_index[1]++;
					}
				}
			// LISTENING_FOR_RESPONSE
				if(uart_fsm_state[1] == LISTENING_FOR_RESPONSE){
					if(uart_read_buffer[1][uart_index[1]] == 0x7B){
						// find open buffer slot and if none found, go to buffer full state
						for(uart_status_index[1] = 0; uart_status_index[1] <PI_HAT_SENSOR_CNT; uart_status_index[1]++){
							if(uart_read_message_buffer_status[1][uart_status_index[1]]==AVAILABLE){
								uart_fsm_state[1] = CAPTURING_RESPONSE;
								uart_read_index[1] = 0; // May cause overwriting in future
								msg_buffer_index[1] = uart_status_index[1];
								break;
							}
							else if(uart_status_index[1] == PI_HAT_SENSOR_CNT-1){
								uart_fsm_state[1] = MSG_BUFFER_FULL;
							}
						}
					}
					else{
						while(!(UCA1IFG&UCTXIFG));
						UCA1TXBUF = 'L';
						uart_index[1]++;
					}
				}
			// CAPTURING_RESPONSE
				if(uart_fsm_state[1] == CAPTURING_RESPONSE){
					uart_read_message_buffer[1][msg_buffer_index[1]][uart_read_index[1]] = uart_read_buffer[1][uart_index[1]];
					if(uart_read_message_buffer[1][msg_buffer_index[1]][uart_read_index[1]] == 0x7D){// TODO: If end of msg len stop reading in...
						uart_fsm_state[1] = LISTENING_FOR_RESPONSE;
						uart_read_message_buffer[1][msg_buffer_index[1]][uart_read_index[1]+1] = '\0';
						uart_read_message_buffer_status[1][uart_status_index[1]]=VALID;
						msg_buffer_index[1]++;
						uart_status[1]++;
						while(!(UCA1IFG&UCTXIFG));
						UCA1TXBUF = 'D';
					}
					else{
						uart_read_index[1]++;
						uart_index[1]++;
						while(!(UCA1IFG&UCTXIFG));
						UCA1TXBUF = 'C';
					}
				}
				if(uart_index[1] == MSG_LEN){
					uart_index[1] = 0;
				}
        	}
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
        	if(uart_interrupt_style[2] == NO_BUFFER){
        		RXSWFG2 = 1;
        	}
        	else if (uart_interrupt_style[2] == BUFFER){
        		uart_read_buffer[2][uart_index[2]] = UCA2RXBUF;
        		if(uart_status[2] >= PI_HAT_SENSOR_CNT){
        			uart_fsm_state[2] = MSG_BUFFER_FULL;
        		}
        	// MSG_BUFFER_FULL
				if(uart_fsm_state[2] == MSG_BUFFER_FULL){
					if(uart_status[2] < PI_HAT_SENSOR_CNT){
						uart_fsm_state[2] = LISTENING_FOR_RESPONSE;
					}
					else {
						while(!(UCA2IFG&UCTXIFG));
							UCA2TXBUF = 'F'; // TODO: Chris OK with?
							uart_index[2]++;
					}
				}
			// LISTENING_FOR_RESPONSE
				if(uart_fsm_state[2] == LISTENING_FOR_RESPONSE){
					if(uart_read_buffer[2][uart_index[2]] == 0x7B){
						// find open buffer slot and if none found, go to buffer full state
						for(uart_status_index[2] = 0; uart_status_index[2] <PI_HAT_SENSOR_CNT; uart_status_index[2]++){
							if(uart_read_message_buffer_status[2][uart_status_index[2]]==AVAILABLE){
								uart_fsm_state[2] = CAPTURING_RESPONSE;
								uart_read_index[2] = 0; // May cause overwriting in future
								msg_buffer_index[2] = uart_status_index[2];
								break;
							}
							else if(uart_status_index[2] == PI_HAT_SENSOR_CNT-1){
								uart_fsm_state[2] = MSG_BUFFER_FULL;
							}
						}
					}
					else{
						while(!(UCA2IFG&UCTXIFG));
						UCA2TXBUF = 'L';
						uart_index[2]++;
					}
				}
			// CAPTURING_RESPONSE
				if(uart_fsm_state[2] == CAPTURING_RESPONSE){
					uart_read_message_buffer[2][msg_buffer_index[2]][uart_read_index[2]] = uart_read_buffer[2][uart_index[2]];
					if(uart_read_message_buffer[2][msg_buffer_index[2]][uart_read_index[2]] == 0x7D){// TODO: If end of msg len stop reading in...
						uart_fsm_state[2] = LISTENING_FOR_RESPONSE;
						uart_read_message_buffer[2][msg_buffer_index[2]][uart_read_index[2]+1] = '\0';
						uart_read_message_buffer_status[2][uart_status_index[2]]=VALID;
						msg_buffer_index[2]++;
						uart_status[2]++;
						while(!(UCA2IFG&UCTXIFG));
						UCA2TXBUF = 'D';
					}
					else{
						uart_read_index[2]++;
						uart_index[2]++;
						while(!(UCA2IFG&UCTXIFG));
						UCA2TXBUF = 'C';
					}
				}
				if(uart_index[2] == MSG_LEN){
					uart_index[2] = 0;
				}
        	}
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
        	if(uart_interrupt_style[3] == NO_BUFFER){
        		RXSWFG3 = 1;
        	}
        	else if (uart_interrupt_style[3] == BUFFER){
        		uart_read_buffer[3][uart_index[3]] = UCA3RXBUF;
        		if(uart_status[3] >= PI_HAT_SENSOR_CNT){
        			uart_fsm_state[3] = MSG_BUFFER_FULL;
        		}
        	// MSG_BUFFER_FULL
				if(uart_fsm_state[3] == MSG_BUFFER_FULL){
					if(uart_status[3] < PI_HAT_SENSOR_CNT){
						uart_fsm_state[3] = LISTENING_FOR_RESPONSE;
					}
					else {
						while(!(UCA3IFG&UCTXIFG));
							UCA3TXBUF = 'F'; // TODO: Chris OK with?
							uart_index[3]++;
					}
				}
			// LISTENING_FOR_RESPONSE
				if(uart_fsm_state[3] == LISTENING_FOR_RESPONSE){
					if(uart_read_buffer[3][uart_index[3]] == 0x7B){
						// find open buffer slot and if none found, go to buffer full state
						for(uart_status_index[3] = 0; uart_status_index[3] <PI_HAT_SENSOR_CNT; uart_status_index[3]++){
							if(uart_read_message_buffer_status[3][uart_status_index[3]]==AVAILABLE){
								uart_fsm_state[3] = CAPTURING_RESPONSE;
								uart_read_index[3] = 0; // May cause overwriting in future
								msg_buffer_index[3] = uart_status_index[3];
								break;
							}
							else if(uart_status_index[3] == PI_HAT_SENSOR_CNT-1){
								uart_fsm_state[3] = MSG_BUFFER_FULL;
							}
						}
					}
					else{
						while(!(UCA3IFG&UCTXIFG));
						UCA3TXBUF = 'L';
						uart_index[3]++;
					}
				}
			// CAPTURING_RESPONSE
				if(uart_fsm_state[3] == CAPTURING_RESPONSE){
					uart_read_message_buffer[3][msg_buffer_index[3]][uart_read_index[3]] = uart_read_buffer[3][uart_index[3]];
					if(uart_read_message_buffer[3][msg_buffer_index[3]][uart_read_index[3]] == 0x7D){// TODO: If end of msg len stop reading in...
						uart_fsm_state[3] = LISTENING_FOR_RESPONSE;
						uart_read_message_buffer[3][msg_buffer_index[3]][uart_read_index[3]+1] = '\0';
						uart_read_message_buffer_status[3][uart_status_index[3]]=VALID;
						msg_buffer_index[3]++;
						uart_status[3]++;
						while(!(UCA3IFG&UCTXIFG));
						UCA3TXBUF = 'D';
					}
					else{
						uart_read_index[3]++;
						uart_index[3]++;
						while(!(UCA3IFG&UCTXIFG));
						UCA3TXBUF = 'C';
					}
				}
				if(uart_index[3] == MSG_LEN){
					uart_index[3] = 0;
				}
        	}
			break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
//*********************************************************************************************************//
//*********************************		TALK TO MOTOR MSP SPI	  *****************************************//
//*********************************************************************************************************//
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
        	spi_mst_read_buffer[spi_mst_index] = UCB0RXBUF;
             switch(spi_mst_fsm_state)
             {
                case IDLE:

                    break;
                case CHECKING_IF_SLAVE_READY:
                    if(spi_mst_read_buffer[spi_mst_index] == 'L'){
                      spi_mst_fsm_state = SENDING_COMMAND;
                      spi_mst_write_index = 0;
                    }
                    TX_B0('R')
                    break;
                case SENDING_COMMAND:
                    spi_mst_tx_data = spi_mst_send_message[spi_mst_write_index++];
                    if(spi_mst_tx_data == '}'){
                      spi_mst_fsm_state = LISTENING_FOR_RESPONSE;
                    }
                    TX_B0(spi_mst_tx_data)
                    break;
                case LISTENING_FOR_RESPONSE:
                    if(spi_mst_read_buffer[spi_mst_index] == '{'){
                      spi_mst_fsm_state = CAPTURING_RESPONSE;
                      spi_mst_read_message[0] = '{';
                      spi_mst_read_index = 1;
                      TX_B0('C')
                    }
                    else{
                        TX_B0('L')
                    }
                    break;
                case CAPTURING_RESPONSE:
                    spi_mst_read_message[spi_mst_read_index++] = spi_mst_read_buffer[spi_mst_index];
                    if(spi_mst_read_buffer[spi_mst_index] == '}'){
                      spi_mst_fsm_state = IDLE;
                      spi_mst_readDoneFG = 1;
                      TX_B0('D')
                    }
                    else {
                      TX_B0('C')
                    }
                    break;
                default:

                    break;
             }
        	if(spi_mst_index == BUFF_LEN-1){
        		spi_mst_index = 0;
        	}
             else {
                   spi_mst_index++;
             }
            // Wake up to setup next TX
            __bic_SR_register_on_exit(LPM0_bits);
            break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}
//*********************************************************************************************************//
