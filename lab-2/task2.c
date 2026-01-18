#include <stdint.h>
#include <nrf.h>

// Control LED(P0.13) using P0.11 button

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

int main(void)
{
    // Configure the GPIO pin of the LED.
    // TODO
    NRF_P0->PIN_CNF[13] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);

    // Configure the GPIO pin of the button.
	// TODO
    NRF_P0->PIN_CNF[11] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) | BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect);

    while (1)
    {
        // Read button state and control the LED.
		// TODO
        //volatile uint32_t pin_state = NRF_P0->IN & BV(11);
        if ((NRF_P0->IN & BV(11)) != 0) {   //careful about == 1 || == 0; later is used because "&" result won't be equal to 1 rather would be 0x800 =! 1
            NRF_P0->OUTCLR = BV(13); // active high
        } else {
            NRF_P0->OUTSET = BV(13); 
        }

    }
}

