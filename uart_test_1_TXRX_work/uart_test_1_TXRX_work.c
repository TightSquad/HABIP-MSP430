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
//extern char uart_b0_read_buffer[MSG_LEN];
//extern char uart_b0_read_message[MSG_LEN];
//extern char uart_b1_read_buffer[MSG_LEN];
//extern char uart_b1_read_message[MSG_LEN];
//extern char uart_b2_read_buffer[MSG_LEN];
//extern char uart_b2_read_message[MSG_LEN];
extern char uart_b3_read_buffer[MSG_LEN];
extern char uart_b3_read_message[MSG_LEN];
extern char uart_read_buffer[4][MSG_LEN];
extern char uart_read_message[4][MSG_LEN];
//extern char* uart_response;
extern int uart_index;
extern int uart_readDoneFG;
extern int uart_fsm_state;
extern int uart_read_index;
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

//temp
void test_strcmp(void);
void test_strcpy(void);
void test_strcat(void);
void test_extraction_command(void);
void test_strstr(void);
void test_removing_7B(void);
void test_removing_7D(void);
void test_strtok(void);


//temp?
extern char response_buffer_b0[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b1[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b2[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b3[PI_HAT_SENSOR_CNT][MSG_LEN];
extern char response_buffer_b4[DAQCS_SENSOR_CNT][MSG_LEN];

extern char response_status_b0[PI_HAT_SENSOR_CNT];
extern char response_status_b1[PI_HAT_SENSOR_CNT];
extern char response_status_b2[PI_HAT_SENSOR_CNT];
extern char response_status_b3[PI_HAT_SENSOR_CNT];
extern char response_status_b4[DAQCS_SENSOR_CNT];

//temp

//*********************************************************************************************************//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop Watchdog

// Configure GPIO
    config_DS4_LED();
    config_UART_GPIO(0);
    config_UART_GPIO(1);
    config_UART_GPIO(2);
    config_UART_GPIO(3);
//    config_XT1_GPIO();						// XT1 Crystal

// Configure Clock
//    config_XT1_ACLK_32768Hz();
    config_DCO_8MHz();

// Configure UART
//    config_UART_4_9600_ACLK_32768Hz();
    config_UART_9600_SMCLK_8MHz(0);
    config_UART_9600_SMCLK_8MHz(1);
    config_UART_9600_SMCLK_8MHz(2);
    config_UART_9600_SMCLK_8MHz(3);

    __bis_SR_register(GIE);

// Begin Main Code
    char UART_num[MSG_LEN];
while(1){
	if(RXSWFG0){
		UART_read_response(0,&RXSWFG0);
		strcpy(UART_num,"UART_0----");
		strcat(UART_num,uart_read_message[0]);
//		UART_write_msg(uart_read_message[0]);
		UART_write_msg(0,UART_num);
	}
	else if(RXSWFG1){
		UART_read_response(1,&RXSWFG1);
		strcpy(UART_num,"UART_1----");
		strcat(UART_num,uart_read_message[1]);
//		UART_write_msg(uart_read_message[1]);
		UART_write_msg(1,UART_num);
	}
	else if(RXSWFG2){
		UART_read_response(2,&RXSWFG2);
		strcpy(UART_num,"UART_2----");
		strcat(UART_num,uart_read_message[2]);
//		UART_write_msg(uart_read_message[2]);
		UART_write_msg(2,UART_num);
	}
	else if(RXSWFG3){
		UART_read_response(3,&RXSWFG3);
		strcpy(UART_num,"UART_3----");
		strcat(UART_num,uart_read_message[3]);
//		UART_write_msg(uart_read_message[3]);
		UART_write_msg(3,UART_num);
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

void test_strtok(void){
	strcpy(response_buffer_b0[PI_TD0],"{00:B0:TD0}");
	char* pcommand;
	char* pboard;
	char* psensor;
	char array_copy[17];
	strcpy(array_copy,response_buffer_b0[PI_TD0]);
	if(strstr(array_copy,"{")!=NULL){
		strncpy(array_copy,array_copy+1,strlen(array_copy)-1);
		array_copy[strlen(array_copy)-1]='\0';
	}
	if(strstr(array_copy,"}")!=NULL){
		array_copy[strlen(array_copy)-1]='\0';
	}
	pcommand = strtok(array_copy,":");
	pboard = strtok(NULL,":");
	psensor = strtok(NULL,":");
//	strcpy(command,pcommand);
//	strcpy(board,pboard); // TODO: Appending '\0's when writing to already written to array.
//	strcpy(sensor,psensor);
//	UART_write_msg(command);
//	UART_write_msg(board);
//	UART_write_msg(sensor);
//	if(strcmp(pcommand,"00")==0){
//		UART_write_msg(pcommand);
//	}
//	if(strcmp(pboard,"B0")==0){
//		UART_write_msg(pboard);
//	}
//	if(strcmp(psensor,"TD0")==0){
//		UART_write_msg(psensor);
//	}
	__no_operation();
}
void test_removing_7B(void){
	strcpy(response_buffer_b0[PI_TD0],"{00:B0:TD0}");
	if(strlen(strstr(response_buffer_b0[PI_TD0],"{")) == strlen(response_buffer_b0[PI_TD0])){
		strncpy(response_buffer_b0[PI_TD0],response_buffer_b0[PI_TD0]+1,strlen(response_buffer_b0[PI_TD0])-1);
	}
	else {
		strcpy(response_buffer_b0[PI_TD0],"'{' not at beginning");
	}
	UART_write_msg(0,response_buffer_b0[PI_TD0]);
}
void test_removing_7D(void){
	strcpy(response_buffer_b0[PI_TD0],"{00:B0:TD0}");
//	char* temp = strstr(response_buffer_b0[PI_TD0],"}");
//	UART_write_msg(temp);
//	char test[17];
	if(strlen(strstr(response_buffer_b0[PI_TD0],"}")) == 1){
//		strncpy(response_buffer_b0[PI_TD0],response_buffer_b0[PI_TD0],strlen(response_buffer_b0[PI_TD0])-1);
//		strncpy(test,response_buffer_b0[PI_TD0],strlen(response_buffer_b0[PI_TD0])-1);
		response_buffer_b0[PI_TD0][strlen(response_buffer_b0[PI_TD0])-1] = '\0';
	}
	else {
		strcpy(response_buffer_b0[PI_TD0],"'}' not at end");
	}
	UART_write_msg(0,response_buffer_b0[PI_TD0]);
}
void test_strstr(void){
	strcpy(response_buffer_b0[PI_TD0],"{00:B0:TD0}");
	char* leftover;
	leftover = strstr(response_buffer_b0[PI_TD0],":");
	strcpy(response_buffer_b0[PI_TD0],leftover);
//	UART_write_msg(response_buffer_b0[PI_TD0]);
}
void test_extraction_command(void){
	strcpy(response_buffer_b0[PI_TD0],"{00:B0:TD0}");
	char* leftover = strstr(response_buffer_b0[PI_TD0],":");
	char command_val [3] = {};
	if(strchr(response_buffer_b0[PI_TD0],START_CHAR)!= NULL){ // Has start char somewhere
		strncpy(command_val,response_buffer_b0[PI_TD0]+1,strlen(response_buffer_b0[PI_TD0])-strlen(leftover)-1);
	}
	else {
		strncpy(command_val,response_buffer_b0[PI_TD0],strlen(response_buffer_b0[PI_TD0])-strlen(leftover));
	}
	UART_write_msg(0,command_val);
}
void test_strcat(void){
	// Test Results
	// SUCCESS: "" to array
	// SUCCESS: array to array
	// SUCCESS: pchar to array
//	strcpy(response_buffer_b0[PI_TD0],"{00:");
//	strcpy(response_buffer_b0[PI_TB0],"TB0:");
//	strcat(response_buffer_b0[PI_TD0],response_buffer_b0[PI_TB0]);
//	strcat(response_buffer_b0[PI_TD0],"1490}");
	strcat(response_buffer_b0[PI_TD0],commands[0]);
	UART_write_msg(0,response_buffer_b0[PI_TD0]);
}
void test_strcmp(void){
	// Test Results
	// SUCCESS: Two char pointers
	// SUCCESS: Two arrays
	// SUCCESS: One array one char pointer
    int result = 1;
//    result = strcmp(commands[0],commands[1]); // SUCCESS
//    char* test = "aloha";
//    strcpy(response_buffer_b0[PI_TD0],"aloha");
//    strcpy(response_buffer_b0[PI_TB0],"aloha");
//    result = strcmp(response_buffer_b0[PI_TD0],response_buffer_b0[PI_TB0]); // SUCCESS
//    result = strcmp(response_buffer_b0[PI_TD0],test); // SUCCESS
    strcpy(response_buffer_b0[PI_TD0],commands[0]);
    result = strcmp(response_buffer_b0[PI_TD0],commands[1]); // SUCCESS
    if(result == 0) {
    	UART_write_msg(0,"Great Success YO Swag");
    }
}
void test_strcpy(void){
	// Test Results
	// Need normal chars, not volatile
	// FAIL: Two char pointers
	// SUCCESS: Copying pchar into array
	// SUCCESS: Copying "" into array
	// SUCCESS: Array to Array - same size
	// SUCCESS: Array to Array - diff msg size
	// SUCCESS: Array to Array - diff array size
	// OVERFLOW: Overflow is a thing. be careful TODO: Can I do anything about this error wise or preventative?
//	strcpy(commands[1],commands[2]); // FAIL
//	strcpy(commands[1],"Help"); // FAIL
//	commands[1] = strcpy(commands[1],commands[2]); // FAIL
//	UART_write_msg(commands[1]);
//	strcpy(response_buffer_b0[PI_TD0],"1234567890123456"); // SUCCESS
////	strcpy(response_buffer_b0[PI_TB0],"testTB0");
////	strcpy(response_buffer_b0[PI_TD0],response_buffer_b0[PI_TB0]); // SUCCESS
//	strcpy(response_buffer_b4[DQ_TB0],"abc123def456333overflow");
//	strcpy(response_buffer_b0[PI_TD0],response_buffer_b4[DQ_TB0]); // SUCCESS
////	UART_write_msg(response_buffer_b0[PI_TD0]);
//	UART_write_msg(response_buffer_b0[PI_TB0]); // Overflow happens
	strcpy(response_buffer_b0[PI_TD0],"1234567890123456");
	strcpy(response_buffer_b0[PI_TD0],commands[2]);
	UART_write_msg(0,response_buffer_b0[PI_TD0]);
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
