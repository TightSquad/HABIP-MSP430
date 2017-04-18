/*
 * UART.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
#include <msp430.h>
#include <driverlib.h>
#include "common.h"
#include "uart.h"
#include "command_interface.h"

// UART UD Variables
char uart_read_buffer[4][BUFF_LEN]={{}};
char uart_read_message[4][MSG_LEN]={{}};
char uart_read_message_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN]={{{}}};
char uart_read_message_buffer_status[4][PI_HAT_SENSOR_CNT]={{AVAILABLE}};
volatile int msg_buffer_index[4]={0};
volatile int uart_index[4] = {0};
volatile int uart_read_index[4] = {0};
volatile int uart_status_index[4] = {0};
volatile int uart_status[4] = {0};
int uart_fsm_state[4] = {{LISTENING_FOR_RESPONSE}}; // TODO: Does this work for init?
char uart_interrupt_style[4] = {BUFFER};
volatile int RXSWFG0 = 0;
volatile int RXSWFG1 = 0;
volatile int RXSWFG2 = 0;
volatile int RXSWFG3 = 0;

void config_UART_GPIO(int brd_num){
	switch(brd_num)
	{
	case 0:
		// USCI_A0 UART operation
		P2SEL1 |= (BIT0 | BIT1);
		P2SEL0 &= ~(BIT0 | BIT1);
		break;
	case 1:
		// USCI_A1 UART operation
		P2SEL1 |= (BIT5 | BIT6);
		P2SEL0 &= ~(BIT5 | BIT6);
		break;
	case 2:
		// USCI_A2 UART operation
		P5SEL1 &= ~(BIT4 | BIT5);
		P5SEL0 |= (BIT4 | BIT5);
		break;
	case 3:
		// USCI_A3 UART operation
		P6SEL1 &= ~(BIT0 | BIT1);
		P6SEL0 |= (BIT0 | BIT1);
		break;
	default: break;
	}
	activate_GPIO_config();
}
void config_UART_9600_ACLK_32768Hz(int brd_num){
	/* Dependencies
	 * config_XT1_ACLK_32768Hz();
	 */
	switch(brd_num)
	{
	case 0:
		// Configure USCI_A0 for UART mode
		UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA0CTLW0 = 0x0000;
		UCA0CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA0BRW = 3;                            // 9600 baud
		UCA0MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	case 1:
		// Configure USCI_A1 for UART mode
		UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA1CTLW0 = 0x0000;
		UCA1CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA1BRW = 3;                            // 9600 baud
		UCA1MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA1CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	case 2:
		// Configure USCI_A2 for UART mode
		UCA2CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA2CTLW0 = 0x0000;
		UCA2CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA2BRW = 3;                            // 9600 baud
		UCA2MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA2CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	case 3:
		// Configure USCI_A3 for UART mode
		UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA3CTLW0 = 0x0000;
		UCA3CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
		UCA3BRW = 3;                            // 9600 baud
		UCA3MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
												// UCBRSx value = 0x53 (See UG)
		UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		break;
	default: break;
	}
}
void config_UART_9600_SMCLK_8MHz(int brd_num){
	/* Dependencies:
	 * config_DCO_8MHz();
	 */
	// Baud Rate calculation
	// 8000000/(16*9600) = 52.083
	// Fractional portion = 0.083
	// User's Guide Table 21-4: UCBRSx = 0x04
	// UCBRFx = int ( (52.083-52)*16) = 1
	//	UCAXBRW = 52;                           // 8000000/16/9600
	switch(brd_num)
	{
	case 0:
		// Configure USCI_A0 for UART mode
		UCA0CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA0CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA0BRW = 52;
		UCA0MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA0CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
		break;
	case 1:
		// Configure USCI_A1 for UART mode
		UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA1CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA1BRW = 52;
		UCA1MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA1CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
		break;
	case 2:
		// Configure USCI_A2 for UART mode
		UCA2CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA2CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA2BRW = 52;
		UCA2MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA2CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA2IE |= UCRXIE;                       // Enable USCI_A2 RX interrupt
		break;
	case 3:
		// Configure USCI_A3 for UART mode
		UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
		UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
		UCA3BRW = 52;
		UCA3MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
		UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
		UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
		break;
	default: break;
	}
}
void UART_read_response(int brd_num, volatile int* RXSWFG){
	int index = 0;
	int Done = 0;
	int read_index = 0;
	while(Done == 0){
		while(*RXSWFG == 0) ;
		switch(brd_num)
		{
		case 0:
			uart_read_buffer[brd_num][index] = UCA0RXBUF;
			break;
		case 1:
			uart_read_buffer[brd_num][index] = UCA1RXBUF;
			break;
		case 2:
			uart_read_buffer[brd_num][index] = UCA2RXBUF;
			break;
		case 3:
			uart_read_buffer[brd_num][index] = UCA3RXBUF;
			break;
		default: break;
		}
	// LISTENING_FOR_RESPONSE
		if(uart_fsm_state[brd_num] == LISTENING_FOR_RESPONSE){
			if(uart_read_buffer[brd_num][index] == 0x7B){
				uart_fsm_state[brd_num] = CAPTURING_RESPONSE;
				read_index = 0; // May cause overwriting in future
			}
			else{
				index++;
			}
		}
	// CAPTURING_RESPONSE
		if(uart_fsm_state[brd_num] == CAPTURING_RESPONSE){
			uart_read_message[brd_num][read_index] = uart_read_buffer[brd_num][index];
			if(uart_read_message[brd_num][read_index] == 0x7D){
				uart_fsm_state[brd_num] = LISTENING_FOR_RESPONSE;
				Done = 1; // reset where?
			} // TODO: some check for '\0' in uart_response so don't overwrite next array of chars.
			else{
				read_index++;
				index++;
			}
		}
		if(index == MSG_LEN){
			index = 0;
		}
		*RXSWFG = 0;
//		UART_write_msg(brd_num,"{ACK}");
	}
}

void UART_write_msg(int brd_num, char* message){
	int i;
	i = 0;
	switch(brd_num)
	{
	case 0:
		while(message[i] != '\0'){
			while(!(UCA0IFG&UCTXIFG));
			UCA0TXBUF = message[i];
			i++;
		}
//		while(!(UCA0IFG&UCTXIFG));
//		UCA0TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	case 1:
		while(message[i] != '\0'){
			while(!(UCA1IFG&UCTXIFG));
			UCA1TXBUF = message[i];
			i++;
		}
//		while(!(UCA1IFG&UCTXIFG));
//		UCA1TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	case 2:
		while(message[i] != '\0'){
			while(!(UCA2IFG&UCTXIFG));
			UCA2TXBUF = message[i];
			i++;
		}
//		while(!(UCA2IFG&UCTXIFG));
//		UCA2TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	case 3:
		while(message[i] != '\0'){
			while(!(UCA3IFG&UCTXIFG));
			UCA3TXBUF = message[i];
			i++;
		}
//		while(!(UCA3IFG&UCTXIFG));
//		UCA3TXBUF = END_CHAR; // TODO: future dev decide on passing in {XX} or just XX
		break;
	default: break;
	}
}

//void chris_init(void){
//    // LFXT Setup
//    //Set PJ.4 and PJ.5 as Primary Module Function Input.
//    /*
//
//     * Select Port J
//     * Set Pin 4, 5 to input Primary Module Function, LFXT.
//     */
//    GPIO_setAsPeripheralModuleFunctionInputPin(
//        GPIO_PORT_PJ,
//        GPIO_PIN4 + GPIO_PIN5,
//        GPIO_PRIMARY_MODULE_FUNCTION
//        );
//
//    //Set DCO frequency to 1 MHz
//    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_0);
//    //Set external clock frequency to 32.768 KHz
//    CS_setExternalClockSource(32768,0);
//    //Set ACLK=LFXT
//    CS_initClockSignal(CS_ACLK,CS_LFXTCLK_SELECT,CS_CLOCK_DIVIDER_1);
//    //Set SMCLK = DCO with frequency divider of 1
//    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
//    //Set MCLK = DCO with frequency divider of 1
//    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
//    //Start XT1 with no time out
//    CS_turnOnLFXT(CS_LFXT_DRIVE_0);
//
//    // Configure UART pins
//    //Set P6.0 and P6.1 as Secondary Module Function Input.
//    /*
//
//     * Select Port 6
//     * Set Pin 0, 1 to input Secondary Module Function, (UCA3TXD/UCA3SIMO, UCA3RXD/UCA3SOMI).
//     */
//    GPIO_setAsPeripheralModuleFunctionInputPin(
//        GPIO_PORT_P6,
//        GPIO_PIN0 + GPIO_PIN1,
//        GPIO_PRIMARY_MODULE_FUNCTION
//        );
//
//    /*
//     * Disable the GPIO power-on default high-impedance mode to activate
//     * previously configured port settings
//     */
//    PMM_unlockLPM5();
//
//    // Configure UART
//    EUSCI_A_UART_initParam param = {0};
//    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_ACLK;
//    param.clockPrescalar = 3;
//    param.firstModReg = 0;
//    param.secondModReg = 146;
//    param.parity = EUSCI_A_UART_NO_PARITY;
//    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
//    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
//    param.uartMode = EUSCI_A_UART_MODE;
//    param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;
//
//    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A3_BASE, &param))
//    {
//        return;
//    }
//
//    EUSCI_A_UART_enable(EUSCI_A3_BASE);
//
//    EUSCI_A_UART_clearInterrupt(EUSCI_A3_BASE,
//                                EUSCI_A_UART_RECEIVE_INTERRUPT);
//
//    // Enable USCI_A3 RX interrupt
//    EUSCI_A_UART_enableInterrupt(EUSCI_A3_BASE,
//                                 EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt
//
//}
