#include <stdint.h>
#include <nrf.h>

// Using CMSIS definitions

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

int main(void)
{
    // Configure the GPIO pin of the LED.
	// TODO
    NRF_P0->PIN_CNF[13] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output);


    // Activate the LED using the OUTCLR register.
	// TODO
    NRF_P0->OUTSET = BV(13);
    // run forever
    while (1)
        ;
}
