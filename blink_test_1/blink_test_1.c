#include <driverlib.h>

int main(void) {

    volatile uint32_t i;

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set P1.0 to output direction
    GPIO_setAsOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN0
        );

    // Set P1.0 to output direction
    GPIO_setAsOutputPin(
        GPIO_PORT_P4,
        GPIO_PIN1
        );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);

    while(1)
    {
        // Toggle P1.0 output
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

        // Toggle P1.0 output
		GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN1);

        // Delay
        for(i=10000; i>0; i--);
		for(i=10000; i>0; i--);
	}
}
