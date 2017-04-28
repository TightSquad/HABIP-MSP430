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
    TA1CCR0 |= 0x4E20;

	//Set output mode to reset/set
	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_7;

	// Set the default duty cylce of the PWM in uSeconds
	TA1CCR1 |= 0;
	TA1CCR2 |= 0;

	//Start Timer1 and PWM
	TA1CTL |= ID_3 + TASSEL_2 + MC_1;
	//-------------------------------------------

}

void motorON(void){
	//set the P4.0 bit high to enable motor
	P4OUT |= BIT0;
}

void motorSpeed(int duty_cycle){
	//some safety checks
	if (duty_cycle < 2000){
		duty_cycle = 2000;
	}
	if (duty_cycle > 18000){
		duty_cycle = 18000;
	}
	//place the duty cycle in the Timer counter
	TA1CCR1 = duty_cycle;
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

void motorRampDown(void){
	int curr_speed = TA1CCR1;

	while (curr_speed > 2000){
		curr_speed--;
		motorSpeed(curr_speed);
		__delay_cycles(1000);
	}
}

int mapMotorSpeed(double input){
	// Motor speed input ranges (RPM)
	static int in_min = 0;
	static int in_max = 2590;

	// motor controller output ranges (Duty Cycle)
	static int out_min = 2000;
	static int out_max = 18000;

	return (input - in_min) * ((out_max - out_min) / (in_max - in_min)) + out_min;
}

void motorRPM(double desired_rpm){
	if (desired_rpm < 0) {
		desired_rpm = desired_rpm * -1;
	}
	// map the desired RPM to a duty cycle
	// send duty cycle command to PWM control
	motorSpeed(mapMotorSpeed(desired_rpm));
}


void currentControl(int current_duty_cycle){
	TA1CCR2 = current_duty_cycle;
}
