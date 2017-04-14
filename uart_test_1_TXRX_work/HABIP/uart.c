/*
 * UART.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
// #include <uart.h>
#include <msp430.h>
#include <driverlib.h>
#include "common.h"
#include "uart.h"

// UART UD Variables
volatile char uart_read_buffer[MSG_LEN]={};
volatile char uart_read_message[MSG_LEN]={};
volatile int uart_index = 0;
volatile int uart_readDoneFG = 0;
volatile int uart_fsm_state = LISTENING_FOR_RESPONSE;
volatile int uart_read_index = 0;

void config_UART_4_GPIO(void){
	P6SEL1 &= ~(BIT0 | BIT1);
	P6SEL0 |= (BIT0 | BIT1);				// USCI_A3 UART operation
	activate_GPIO_config();
}
void config_UART_4_9600_ACLK_32768Hz(void){
// Configure USCI_A3 for UART mode
	/* Dependencies
	 * config_XT1_ACLK_32768Hz();
	 */
    UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA3CTLW0 = 0x0000;
    UCA3CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
    UCA3BRW = 3;                            // 9600 baud
    UCA3MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
                                            // UCBRSx value = 0x53 (See UG)
    UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
}
void config_UART_4_9600_SMCLK_8MHz(void){
// Configure USCI_A3 for UART mode
	/* Dependencies:
	 * config_DCO_8MHz();
	 */
	UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
	UCA3CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
	// Baud Rate calculation
	// 8000000/(16*9600) = 52.083
	// Fractional portion = 0.083
	// User's Guide Table 21-4: UCBRSx = 0x04
	// UCBRFx = int ( (52.083-52)*16) = 1
	UCA3BRW = 52;                           // 8000000/16/9600
	UCA3MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
	UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
	UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
}

void UART_read_msg(void){
	int i;
	// clear last message
	for(i=0;i<MSG_LEN;i++){
		uart_read_message[i] = 0;
	}
	uart_readDoneFG = 0;
    UCA3IE |= UCRXIE;                        // Enable USCI_A3 RX interrupt
    while(uart_readDoneFG == 0) ;
    UCA3IE &= ~UCRXIE;                       // Disable USCI_A3 RX interrupt
}

void UART_write_msg(char* message){
	int i;
	i = 0;
	while(message[i] != '\0'){
		while(!(UCA3IFG&UCTXIFG));
		UCA3TXBUF = message[i];
		i++;
	}
	while(!(UCA3IFG&UCTXIFG));
	UCA3TXBUF = END_CHAR; // future dev decide on passing in {XX} or just XX
}

void chris_init(void){
    // LFXT Setup
    //Set PJ.4 and PJ.5 as Primary Module Function Input.
    /*

     * Select Port J
     * Set Pin 4, 5 to input Primary Module Function, LFXT.
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_PJ,
        GPIO_PIN4 + GPIO_PIN5,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    //Set DCO frequency to 1 MHz
    CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_0);
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768,0);
    //Set ACLK=LFXT
    CS_initClockSignal(CS_ACLK,CS_LFXTCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    CS_turnOnLFXT(CS_LFXT_DRIVE_0);

    // Configure UART pins
    //Set P6.0 and P6.1 as Secondary Module Function Input.
    /*

     * Select Port 6
     * Set Pin 0, 1 to input Secondary Module Function, (UCA3TXD/UCA3SIMO, UCA3RXD/UCA3SOMI).
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P6,
        GPIO_PIN0 + GPIO_PIN1,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    // Configure UART
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_ACLK;
    param.clockPrescalar = 3;
    param.firstModReg = 0;
    param.secondModReg = 146;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A3_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A3_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A3_BASE,
                                EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A3 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A3_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT); // Enable interrupt

}
