/*
 * daqcs_motor.h
 *
 *  Created on: Apr 11, 2017
 *      Author: stevy
 */

#ifndef DAQCS_MOTOR_H_
#define DAQCS_MOTOR_H_

void setupMotor(void);
void motorON(void);
void motorOFF(void);
void motorCCW(void);
void motorCW(void);
void motorSpeed(int duty_cycle);
void currentControl(int current_duty_cycle);

#endif /* DAQCS_MOTOR_H_ */
