/*
 * SPI.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
// #include <spi.h>
#include <msp430.h>
#include "common.h"

// SPI UD Variables
volatile char spi_read_buffer[MSG_LEN]={};
volatile char spi_read_message[MSG_LEN]={};
volatile char spi_tx_msg[MSG_LEN]="{00:B4:ZGY}";
volatile char spi_index = 0;
volatile char msg_index = 0;
volatile char spi_read_index = 0;
volatile int spi_read_enable = 0;
volatile int spi_readDoneFG = 0;
volatile char TXDATA ='\0';
volatile char RXDATA ='\0';

void config_SPI_B0_Master_GPIO(void){
    // Configure SPI GPIO for Host MSP (MSP-MSP)
    // STE/SS & SIMO & SOMI
//  P1SEL0 &= ~(BIT3 | BIT6 | BIT7);
//  P1SEL1 |= (BIT3 | BIT6 | BIT7);
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
//  __bis_SR_register(GIE);
    i = 0;
    while(message[i] != '\0'){
        while(!(UCB0IFG&UCTXIFG));
        UCB0TXBUF = message[i];
        i++;
    }
    while(!(UCB0IFG&UCTXIFG));
    UCB0TXBUF = END_CHAR;
//  Toggle_ON_OFF_DS4_LED();
//  Toggle_ON_OFF_DS4_LED();
//  __bic_SR_register(GIE);
}
