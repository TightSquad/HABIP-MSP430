/*
 * daqcs_adc.h
 *
 *  Created on: Apr 11, 2017
 *      Author: stevy
 */

#ifndef DAQCS_ADC_H_
#define DAQCS_ADC_H_

void setupADC(void);
int readADC(void);
int mapADC(int input);

#endif /* DAQCS_ADC_H_ */
