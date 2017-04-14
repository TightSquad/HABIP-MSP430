/*
 * daqcs_motor.c
 *
 *  Created on: Apr 11, 2017
 *      Author: stevy
 */
#include <msp430.h>

void setupMotor(void){
	//-------------------------------------------
	// GPIO setup for motor direction and enable
    // Set P4.0 and P4.1 to output direction
    P4DIR |= BIT0 + BIT1;
	//-------------------------------------------


	//-------------------------------------------
    // Setup GPIO to generate PWM for motor speed and current
    // Set P1.2 and P1.3 to output
    P1DIR |= BIT2 + BIT3;

    // Set P1.2 and P1.3 to TA1.1 and TA1.2 mode respectively, See Table 6-20 and TABLE 6-21 in MSP430FR4994 Data Sheet
    P1SEL1 &= ~(BIT2 + BIT3);
    P1SEL0 |= BIT2 + BIT3;


	//-------------------------------------------
    // Timer1 Set-Up

    // Set PWM Period in uSeconds
//	TA1CCR0 |= 20000;
    TA1CCR0 |= 200000;

	//Set output mode to reset/set
	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_7;

	// Set the default duty cylce of the PWM in uSeconds
	TA1CCR1 |= 0;
	TA1CCR2 |= 0;

	//Start Timer1 and PWM
	TA1CTL |= TASSEL_2 + MC_1;
	//-------------------------------------------

}

void motorON(void){
	//set the P4.0 bit high to enable motor
	P4OUT |= BIT0;
}

void motorSpeed(int duty_cycle){
	TA1CCR1 = 10*duty_cycle;
}

void motorOFF(void){
	//set the P4.0 low high to disable motor
	P4OUT &= ~BIT0;
	motorSpeed(0);
}

void motorCCW(void){
	//set the P4.1 bit low to direct motor in the counter clock-wise direction
	P4OUT &= ~BIT1;
}

void motorCW(void){
	// set the P4.1 bit to high to direct the motor in the clockwise direction
	P4OUT |= BIT1;
}



void currentControl(int current_duty_cycle){
	TA1CCR2 = current_duty_cycle;
}
