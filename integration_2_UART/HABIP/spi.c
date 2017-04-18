/*
 * SPI.c
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */
// #include <spi.h>
#include <msp430.h>
#include "common.h"
#include "spi.h"
#include "string.h"

// SPI UD Variables
volatile char spi_read_buffer[MSG_LEN]={};
volatile char spi_read_message[MSG_LEN]={};
char spi_send_message[MSG_LEN]={};
volatile char spi_send_buffer[MSG_LEN]={};
volatile int msg_return = 0;	//when to respond to SPI master
volatile int spi_fsm_state = LISTENING_FOR_COMMAND;
volatile int spi_index = 0;
volatile int msg_index = 0;
volatile int spi_req_data = 0;
volatile int spi_data_available = 0;
volatile int spi_write_index = 0;
volatile int spi_read_index = 0;
volatile int spi_read_enable = 0;
volatile int spi_readDoneFG = 0;
volatile char TXDATA = '\0';

void config_SPI_B0_Master_GPIO(void){
    // Configure SPI GPIO for Host MSP (MSP-MSP)
    // STE/SS & SIMO & SOMI
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
	strcpy(spi_send_message,"{00:B4:ZGY}"); //temp
    UCB0CTLW0 = UCSWRST;                    // **Put state machine in reset**
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
	strcpy(spi_send_message,"{B4:ZGY:1490}"); //temp
   UCB1CTLW0 = UCSWRST;                    // **Put state machine in reset**
   UCB1CTLW0 |= UCSYNC | UCCKPH | UCMSB | UCMODE_2;   // 4-pin, 8-bit SPI slave
                                           // Clock polarity high, MSB, SS active low
   UCB1BRW = 0x02;                         // /2
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
	strcpy(spi_send_message,"{B4:ZGY:1490}"); //temp
   UCA0CTLW0 = UCSWRST;                    // **Put state machine in reset**
   UCA0CTLW0 |= UCSYNC | UCCKPL | UCMSB | UCMODE_0;   // 3-pin, 8-bit SPI slave
                                           // Clock polarity high, MSB
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
void SPI_command_host_to_slave(char* message,volatile int* read_done,volatile char* TXDATA){
	UCB0IE |= UCRXIE;
	int msg_index = 0;
    *TXDATA = message[msg_index];
    volatile int dummy_values = 0;
    while(*read_done == 0)
    {
    	UCB0IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0, enable interrupt
        __no_operation();                   // Remain in LPM0
        __delay_cycles(2000);               // Delay before next transmission
        if(dummy_values == 0){
        	if(*TXDATA == 0x7D){
        		dummy_values = 1;
        	}
        	else{
        		*TXDATA = message[++msg_index];                           // Increment transmit data
        	}
        }
        if (dummy_values == 1){
        	*TXDATA = 0x58;
        }
    }
    *read_done = 0;
}
