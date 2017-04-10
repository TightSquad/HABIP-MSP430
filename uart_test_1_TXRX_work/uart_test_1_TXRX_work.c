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
#include <driverlib.h> //added for blink LED

// UD Functions
void activate_GPIO_config(void);

void config_XT1_GPIO(void);
void config_XT1_ACLK_32768Hz(void);
void config_DCO_8MHz(void);
void config_DCO_1MHz(void);

void config_UART_4_GPIO(void);
void config_UART_4_9600_ACLK_32768Hz(void);
void config_UART_4_9600_SMCLK_8MHz(void);
void UART_read_msg(void);
void UART_write_msg(char* message);
void chris_init(void);

void config_DS4_LED(void);
void Toggle_ON_OFF_DS4_LED(void);
void delay_LED(void);


// UART UD Constants
const int EOT=4; // End of Transmission ASCII Character
#define MSG_LEN 64 // Default for now

// UART UD Variables
volatile char uart_read_buffer[MSG_LEN]={};
volatile char uart_read_message[MSG_LEN]={};
char uart_index = 0;
volatile int uart_readDoneFG = 0;

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

//    chris_init();

// Begin Main Code
    UART_read_msg();
    UART_write_msg("Hi from MSP");
// End Main Code

	while(1) ; // catchall for debug
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
            uart_read_buffer[uart_index] = UCA3RXBUF;
//            UCA3TXBUF = uart_read_buffer[uart_index];
            if((uart_read_buffer[uart_index] == EOT)||(uart_index == MSG_LEN-1)) {
            	int i;
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
void activate_GPIO_config(void){
	/*Scope: Run after configured GPIO to activate the config*/
	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
    PMM_unlockLPM5();
	//PM5CTL0 &= ~LOCKLPM5;
}
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
void config_XT1_GPIO(void){
	PJSEL0 |= BIT4 | BIT5;                  // For XT1
	activate_GPIO_config();
}
void config_XT1_ACLK_32768Hz(void){
	/*Dependencies:
	 * config_XT1_GPIO();
	 */
// XT1 Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
    CSCTL4 &= ~LFXTOFF;
    do
    {
        CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers
}

void config_DCO_8MHz(void){
// Startup clock system with max DCO setting ~8MHz
	CSCTL0_H = CSKEY_H;                     // Unlock CS registers
	CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 1MHz
	//					ACLK			SMCLK			MCLK
	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; // select clock sources
	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
	CSCTL0_H = 0;                           // Lock CS registers
}

void config_DCO_1MHz(void){
// Startup clock system with max DCO setting ~8MHz
	CSCTL0_H = CSKEY_H;                     // Unlock CS registers
//	CSCTL1 = 0x0000;
//	CSCTL1 = DCOFSEL_3 | DCORSEL;           // Set DCO to 8MHz
//					ACLK			SMCLK			MCLK
	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;   // Set all dividers
	CSCTL0_H = 0;                           // Lock CS registers
}

void UART_read_msg(void){
	int i;
	// clear last message
	for(i=0;i<MSG_LEN;i++){
		uart_read_message[i] = 0;
	}
	uart_readDoneFG = 0;
    UCA3IE |= UCRXIE;                        // Enable USCI_A3 RX interrupt
    __bis_SR_register(GIE);
    while(uart_readDoneFG == 0) ;
    UCA3IE &= ~UCRXIE;                       // Disable USCI_A3 RX interrupt
    __bic_SR_register(GIE);
    Toggle_ON_OFF_DS4_LED(); // Optional
}

void UART_write_msg(char* message){
	int i;
	__bis_SR_register(GIE);
	i = 0;
	while(message[i] != '\0'){
		while(!(UCA3IFG&UCTXIFG));
		UCA3TXBUF = message[i];
//		EUSCI_A_UART_transmitData(EUSCI_A3_BASE,
//		                                  message[i]);
//		__no_operation();
		i++;
	}
	while(!(UCA3IFG&UCTXIFG));
	UCA3TXBUF = EOT;
//	EUSCI_A_UART_transmitData(EUSCI_A3_BASE,
//	                                  EOT);
//	__no_operation();
	Toggle_ON_OFF_DS4_LED();
	Toggle_ON_OFF_DS4_LED();
	__bic_SR_register(GIE);
}

void config_DS4_LED(void){
    GPIO_setAsOutputPin(					// config P1.0 (DS4 LED) GPIO as output
            GPIO_PORT_P1,
            GPIO_PIN0
            );
    activate_GPIO_config();
}
void Toggle_ON_OFF_DS4_LED(void){
	/*Scope: Toggles LED ON/OFF once when message received*/
	/*Dependencies:
	#include <driverlib.h>
	config_DS4_LED();
	*/
	GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
	// Delay
	delay_LED();
    // Toggle P1.0 output
	GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
	// Delay
	delay_LED();
	// Toggle P1.0 output
	GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void delay_LED(void){
	int i;
	int j;
	for(i=1000;i>0;i--){
		__no_operation();
		for(j=1000;j>0;j--) {
			__no_operation();
		}
	}
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
