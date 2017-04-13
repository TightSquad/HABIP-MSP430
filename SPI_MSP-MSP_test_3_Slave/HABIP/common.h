/*
 * COMMON.h
 *
 *  Created on: Apr 11, 2017
 *      Author: Lincster
 */

#ifndef HABIP_COMMON_H_
#define HABIP_COMMON_H_
// #include <msp430.h>
// #include <driverlib.h> //added for blink LED
  // General
void activate_GPIO_config(void);
void config_DS4_LED(void);
void Toggle_ON_OFF_DS4_LED(void);
void delay_LED(void);

// Constants
// const int START_CHAR=0x7B;
// const int END_CHAR=0x7D; // End of Transmission ASCII Character as per Protocol Format
#define START_CHAR 0x7B
#define END_CHAR 0x7D // End of Transmission ASCII Character as per Protocol Format
#define MSG_LEN 64 // Default for now

#endif /* HABIP_COMMON_H_ */
