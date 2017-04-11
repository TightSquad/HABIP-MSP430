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

// UART UD Variables
volatile char uart_read_buffer[MSG_LEN]={};
volatile char uart_read_message[MSG_LEN]={};
volatile char uart_index = 0;
volatile int uart_readDoneFG = 0;

// SPI UD Variables
//volatile char RXData = '\0';
//volatile char TXData = '\0';
volatile char spi_read_buffer[MSG_LEN]={};
volatile char spi_read_message[MSG_LEN]={};
volatile char spi_tx_msg[MSG_LEN]="{00:B4:ZGY}";
volatile char spi_index = 0;
volatile char msg_index = 0;
volatile int spi_readDoneFG = 0;
volatile char TXDATA ='\0';
volatile char RXDATA ='\0';

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
        	TXDATA = spi_tx_msg[++msg_index];                           // Increment transmit data
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
            if((uart_read_buffer[uart_index] == EOT)||(uart_index == MSG_LEN-1)) {
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
			if((spi_read_buffer[spi_index] == EOT)||(spi_index == MSG_LEN-1)) {
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
			if((spi_read_buffer[spi_index] == EOT)||(spi_index == MSG_LEN-1)) {
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
void activate_GPIO_config(void){
	/*Scope: Run after configured GPIO to activate the config*/
	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
    PMM_unlockLPM5();
	//PM5CTL0 &= ~LOCKLPM5;
}
void config_SPI_B0_Master_GPIO(void){
	// Configure SPI GPIO for Host MSP (MSP-MSP)
	// STE/SS & SIMO & SOMI
//	P1SEL0 &= ~(BIT3 | BIT6 | BIT7);
//	P1SEL1 |= (BIT3 | BIT6 | BIT7);
	P1SEL0 &= ~(BIT6 | BIT7);
	P1SEL1 |= (BIT6 | BIT7);
	// SCLK
	P2SEL0 &= ~(BIT2);
	P2SEL1 |= (BIT2);
	activate_GPIO_config();
}

void config_SPI_B1_Slave_GPIO(void){
    // Configure SPI GPIO for Host MSP (COMMS-MSP)
    P5SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);        // USCI_B1 SCLK, MOSI,
    P5SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3);         // STE, and MISO pin
	activate_GPIO_config();
}

void config_SPI_A0_Slave_GPIO(void){
	// Configure SPI GPIO for Slave MSP (MSP-MSP)
	// SIMO & SOMI
	P2SEL0 &= ~(BIT0 | BIT1);
	P2SEL1 |= (BIT0 | BIT1);
	// STE/SS & SCLK
	P1SEL0 &= ~(BIT4 | BIT5);
	P1SEL1 |= (BIT4 | BIT5);
	activate_GPIO_config();
}

void config_SPI_B0_Master(void){
	/*
	 * Dependencies:
	 * config_SPI_B0_Master_GPIO();
	 * config_XT1_ACLK_32768Hz_DCO_1MHz();
	 */
// Configure USCI_B0 for SPI operation
    UCB0CTLW0 = UCSWRST;                    // **Put state machine in reset**
//    UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB | UCMODE_1 | UCSTEM; // 4-pin, 8-bit SPI master
//    UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB | UCMODE_1; // 4-pin, 8-bit SPI master
    UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // 3-pin, 8-bit SPI master
                                            // Clock polarity high, MSB
    UCB0CTLW0 |= UCSSEL__ACLK;              // ACLK
    UCB0BRW = 0x02;                         // /2
    //UCB0MCTLW = 0;                          // No modulation
    UCB0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
    UCB0IE |= UCRXIE;                       // Enable USCI_B0 RX interrupt
    __bis_SR_register(GIE);
}
void config_SPI_B1_Slave(void){
	/*
	 * Dependencies:
	 * config_SPI_B1_Slave_GPIO();
	 * config_XT1_ACLK_32768Hz_DCO_1MHz();
	 */
// Configure USCI_B1 for SPI operation
   UCB1CTLW0 = UCSWRST;                    // **Put state machine in reset**
   UCB1CTLW0 |= UCSYNC | UCCKPL | UCMSB | UCMODE_1 | UCSTEM;   // 4-pin, 8-bit SPI slave
                                           // Clock polarity high, MSB
   UCB1CTLW0 |= UCSSEL__SMCLK;             // ACLK
   UCB1BRW = 0x02;                         // /2
   //UCB1MCTLW = 0;                          // No modulation
   UCB1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
   UCB1IE |= UCRXIE;                       // Enable USCI_B1 RX interrupt
   __bis_SR_register(GIE);
}
void config_SPI_A0_Slave(void){
	/*
	 * Dependencies:
	 * config_SPI_A0_Slave_GPIO();
	 * config_XT1_ACLK_32768Hz_DCO_1MHz();
	 */
// Configure USCI_A0 for SPI operation
   UCA0CTLW0 = UCSWRST;                    // **Put state machine in reset**
//   UCA0CTLW0 |= UCSYNC | UCCKPL | UCMSB | UCMODE_1 | UCSTEM;   // 4-pin, 8-bit SPI slave
   UCA0CTLW0 |= UCSYNC | UCCKPL | UCMSB | UCMODE_1;   // 4-pin, 8-bit SPI slave
                                           // Clock polarity high, MSB
   UCA0CTLW0 |= UCSSEL__SMCLK;             // ACLK
   UCA0BRW = 0x02;                         // /2
   UCA0MCTLW = 0;                          // No modulation
   UCA0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**
   UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
   __bis_SR_register(GIE);
}
void SPI_read_msg(void){
	int i;
	// clear last message
	for(i=0;i<MSG_LEN;i++){
		spi_read_message[i] = 0;
	}
	spi_readDoneFG = 0;
    UCB0IE |= UCRXIE;                        // Enable USCI_B0 RX interrupt
//    __bis_SR_register(GIE);
    while(spi_readDoneFG == 0){
        UCB0IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt
        __no_operation();                   // Remain in LPM0
        __delay_cycles(2000);               // Delay before next transmission
        TXDATA++;                           // Increment transmit data
    }
    UCB0IE &= ~UCRXIE;                       // Disable USCI_B0 RX interrupt
//    __bic_SR_register(GIE);
    Toggle_ON_OFF_DS4_LED(); // Optional
}
void SPI_write_msg(char* message){
	int i;
//	__bis_SR_register(GIE);
	i = 0;
	while(message[i] != '\0'){
		while(!(UCB0IFG&UCTXIFG));
		UCB0TXBUF = message[i];
		i++;
	}
	while(!(UCB0IFG&UCTXIFG));
	UCB0TXBUF = EOT;
//	Toggle_ON_OFF_DS4_LED();
//	Toggle_ON_OFF_DS4_LED();
//	__bic_SR_register(GIE);
}
void config_UART_4_GPIO(void){
	P6SEL1 &= ~(BIT0 | BIT1);
	P6SEL0 |= (BIT0 | BIT1);				// USCI_A3 UART operation
	activate_GPIO_config();
}
void config_UART_4_9600_ACLK_32768Hz(void){
// Configure USCI_A3 for UART mode
	/* Dependencies
	 * config_UART_4_GPIO();
	 * config_XT1_ACLK_32768Hz();
	 */
    UCA3CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA3CTLW0 = 0x0000;
    UCA3CTLW0 |= UCSSEL__ACLK;              // CLK = ACLK
    UCA3BRW = 3;                            // 9600 baud
    UCA3MCTLW |= 0x5300;                    // 32768/9600 - INT(32768/9600)=0.41
                                            // UCBRSx value = 0x53 (See UG)
    UCA3CTLW0 &= ~UCSWRST;                  // Initialize eUSCI
//    UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
}
void config_UART_4_9600_SMCLK_8MHz(void){
// Configure USCI_A3 for UART mode
	/* Dependencies:
	 * config_UART_4_GPIO();
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
//	UCA3IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
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
void config_XT1_ACLK_32768Hz_DCO_1MHz(void){
	/*Dependencies:
	 * config_XT1_GPIO();
	 */
	// XT1 Setup
	CSCTL0_H = CSKEY_H;                     // Unlock CS registers
	CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
	CSCTL1 &= ~DCORSEL;
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
	for(i=500;i>0;i--){
		__no_operation();
		for(j=500;j>0;j--) {
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
