#include <msp430.h> 
#include <driverlib.h>
#include "habip.h"



extern uint16_t adc_val;
extern int adc_software_fg;
extern int cellNum;
//
//// Error raised if an individual cell is too low for a little while
//extern int low_bat_cell_error;
//
//// Keep track of how many times the average voltage for a cell was too low
//extern int low_cell_count[6];

int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
	
    // Keep track of rolling average voltage values for individual battery cells
    double adc_cell_averages[6];

    // Setup ADC
    setupBattADC();

    // Get 30 samples of individual cell voltages
    int j = 0;
    for (j=0; j<30; j++){
    	if (j==0){
    	    getCellVoltages(adc_cell_averages,1);
    	}
    	else{
    		getCellVoltages(adc_cell_averages,0);
    	}
    	__delay_cycles(1000);
    }

	return 0;
}
//*********************************************************************************************************//
//*********************************		ADC ISR MOTOR BATTERY	  *****************************************//
//*********************************************************************************************************//
#pragma vector=ADC12_B_VECTOR
__interrupt void ADC12_ISR(void)
{
	//set the software flag
	adc_software_fg = 1;

	//grab the ADC value
	switch (cellNum){
	case 1:
		adc_val = ADC12MEM12;
		break;
	case 2:
		adc_val = ADC12MEM13;
		break;
	case 3:
		adc_val = ADC12MEM14;
		break;
	case 4:
		adc_val = ADC12MEM17;
		break;
	case 5:
		adc_val = ADC12MEM18;
		break;
	case 6:
		adc_val = ADC12MEM19;
		break;
	}

	__bic_SR_register_on_exit(LPM0_bits);

	return;
}
