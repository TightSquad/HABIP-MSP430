#include <msp430.h> 
#include <driverlib.h>

void setupADC(int);
int readADC(void);

uint16_t adc_val;
int adc_software_flag = 0;
double adc_cell_data[8];
int cellNum = 1;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
	
   // Disable the GPIO power-on default high-impedance mode
   // to activate previously configured port settings
   PMM_unlockLPM5();

   int i = 1;
   int current_adc_val;

   for (i=1; i<7; i++){
	   cellNum = i;
	   setupADC(i);
	   current_adc_val = readADC();
	   adc_cell_data[i] = current_adc_val/4096*2.5;
    }

	return adc_val;
}

void setupADC(int cellNum){
	// VBAT_MOD_CELL_1 is for all 6 cells and is pin P3.0_A12_C12
	// VBAT_MOD_CELL_2 is for 5 cells and is pin P3.1_A13_C13
	// VBAT_MOD_CELL_3 is for 4 cells and is pin P3.2_A14_C14
	// VBAT_MOD_CELL_4 is for 3 cells and is pin P7.5_A17
	// VBAT_MOD_CELL_5 is for 2 cells and is pin P7.6_A18
	// VBAT_MOD_CELL_6 is for 1 cell and is pin P7.7_A19

	// Configure P3.0 for ADC, A12
	P3SEL1 |= BIT0;
	P3SEL0 |= BIT0;

	// Configure P3.1 for ADC, A13
    P3SEL1 |= BIT1;
    P3SEL0 |= BIT1;

    // Configure P3.2 for ADC, A14
    P3SEL1 |= BIT2;
    P3SEL0 |= BIT2;

    // Configure P7.5 for ADC, A17
    P7SEL1 |= BIT5;
    P7SEL0 |= BIT5;

    // Configure P7.6 for ADC, A18
    P7SEL1 |= BIT6;
    P7SEL0 |= BIT6;

    // Configure P7.7 for ADC, A19
    P7SEL1 |= BIT7;
    P7SEL0 |= BIT7;

	// Clear and use 16clk cycles, turn ADC on
	ADC12CTL0 = 0;
	ADC12CTL0 |= ADC12SHT0_2 + ADC12ON;

	// Clear and turn on sampling timer
	ADC12CTL1 = 0;
	ADC12CTL1 = ADC12SHP; // Use sampling timer

	// Set 12bit resolution
	ADC12CTL2 = 0;
	ADC12CTL2 |= ADC12RES_2;

	// Clear and setup A12 as input source for MEM12
	ADC12MCTL12 = 0;
	ADC12MCTL12 |= ADC12_B_INPUT_A12;

	// Clear and setup A13 as input source for MEM13
	ADC12MCTL13 = 0;
	ADC12MCTL13 |= ADC12_B_INPUT_A13;

	// Clear and setup A14 as input source for MEM14
	ADC12MCTL14 = 0;
	ADC12MCTL14 |= ADC12_B_INPUT_A14;

	// Clear and setup A17 as input source for MEM17
	ADC12MCTL17 = 0;
	ADC12MCTL17 |= ADC12_B_INPUT_A17;

	// Clear and setup A18 as input source for MEM18
	ADC12MCTL18 = 0;
	ADC12MCTL18 |= ADC12_B_INPUT_A18;

	// Clear and setup A19 as input source for MEM19
	ADC12MCTL19 = 0;
	ADC12MCTL19 |= ADC12_B_INPUT_A19;

    // Enable interrupts for A12
    ADC12IER0 = 0;
    ADC12IER0 |= ADC12IE12;

    // Enable interrutps for A13
    ADC12IER0 |= ADC12IE13;

    // Enable interrupts for A14
    ADC12IER0 |= ADC12IE14;

    // Enable interrupts for A17
    ADC12IER1 = 0;
    ADC12IER1 |= ADC12IE17;

    // Enable interrupts for A18
    ADC12IER1 |= ADC12IE18;

    // Enable interrupts for A19
    ADC12IER1 |= ADC12IE19;

    // Start acquisitons for the MEM for the current battery cell
    ADC12CTL3 = 0;
    switch (cellNum){
    case 1:
    	ADC12CTL3 |= ADC12CSTARTADD_12; // MEM12
    	break;
    case 2:
    	ADC12CTL3 |= ADC12CSTARTADD_13; // MEM13
    	break;
    case 3:
    	ADC12CTL3 |= ADC12CSTARTADD_14; // MEM14
    	break;
    case 4:
    	ADC12CTL3 |= ADC12CSTARTADD_17; // MEM17
    	break;
    case 5:
    	ADC12CTL3 |= ADC12CSTARTADD_18; // MEM18
    	break;
    case 6:
    	ADC12CTL3 |= ADC12CSTARTADD_19; // MEM19
    	break;
    default:
    	break;
    }
}

int readADC(void){
	// Enable ADC and start acquisition
   ADC12CTL0 |= ADC12ENC + ADC12SC;

   //enable general interrupts
   __bis_SR_register(LPM0_bits | GIE);

   //wait for the adc software flag to be set
   while(adc_software_flag == 0);

   //disable the ADC
   ADC12CTL0 &= ~ADC12ENC;

   //clear the adc software flag
   adc_software_flag = 0;

   return adc_val;
}


//ADC Interrupt Service Routine
#pragma vector=ADC12_B_VECTOR
__interrupt void ADC12_ISR(void)
{
	//set the software flag
	adc_software_flag = 1;

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
