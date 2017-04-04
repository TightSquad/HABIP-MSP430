#include <msp430.h> 
#include <driverlib.h>

void setupADC(void);

uint16_t adc_val;
int adc_software_flag = 0;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
   // Disable the GPIO power-on default high-impedance mode
   // to activate previously configured port settings
   PMM_unlockLPM5();

   setupADC();

   int i;

   for(i = 0; i < 10; i++){
	   // Enable ADC and start acquisition
	   ADC12CTL0 |= ADC12ENC + ADC12SC;

	   //enable general interrupts
	   __bis_SR_register(LPM0_bits | GIE);

	   //wait for the adc software flag to be set
	   while(adc_software_flag == 0);

//	   //grab MEM12 value
//	   adc_val = ADC12MEM12;

	   //disable the ADC
	   ADC12CTL0 &= ~ADC12ENC;

	   //clear the adc software flag
	   adc_software_flag = 0;

   }

	return adc_val;
}


void setupADC(void){
	   // Configure P3.0 for ADC, A12
	   P3SEL1 |= BIT0;
	   P3SEL0 |= BIT0;

	   //clear and use 16clk cycles, turn ADC on
	   ADC12CTL0 = 0;
	   ADC12CTL0 |= ADC12SHT0_2 + ADC12ON;

	   //clear and turn on sampling timer
	   ADC12CTL1 = 0;
	   ADC12CTL1 = ADC12SHP;                   // Use sampling timer

	   //set 12bit resolution
	   ADC12CTL2 = 0;
	   ADC12CTL2 |= ADC12RES_2;

	   //clear and setup A12 as input source for MEM12
	   ADC12MCTL12 = 0;
	   ADC12MCTL12 |= ADC12_B_INPUT_A12;

	   //start acquistions on MEM12
	   ADC12CTL3 = 0;
	   ADC12CTL3 |= ADC12CSTARTADD_12;

	   // enable interrupts for A12
	   ADC12IER0 = 0;
	   ADC12IER0 |= ADC12IE12;
}


//ADC Interrupt Service Routine
#pragma vector=ADC12_B_VECTOR
__interrupt void ADC12_ISR(void)
{
	//set the software flag
	adc_software_flag = 1;
	//grab the ADC value
	adc_val = ADC12MEM12;			//store value of ADC in array
	__bic_SR_register_on_exit(LPM0_bits);
	return;

}
