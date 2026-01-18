#include <stdint.h>
#include <nrf.h>

// remember, everything is MMIO

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))



int main(void)
{
    // Configure the GPIO pin of the LED.
    NRF_P0->PIN_CNF[13] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                         (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                         (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                         (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                         (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    // Configure the timer.
	// TODO
    NRF_TIMER0->MODE = BV_BY_NAME(TIMER_MODE_MODE, Timer);
    NRF_TIMER0->PRESCALER = 4; 
    NRF_TIMER0->BITMODE = BV_BY_NAME(TIMER_BITMODE_BITMODE, 32Bit);
    // now f_timer = 1Mhz
    NRF_TIMER0->TASKS_CLEAR = 1;

    // Start the timer.
	// TODO
    NRF_TIMER0->TASKS_START = 1;

    // Read and store current counter value of timer.
	// TODO
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;   // snap values via TASKS_CAPTURE to put value in the CC[0]
    uint32_t last_val = (NRF_TIMER0->CC[0]);

    while (1)
    {
        // Compute the difference between the current and the stored (last)
        // counter value.
		// TODO
        NRF_TIMER0->TASKS_CAPTURE[0] = 1;
        uint32_t diff = (NRF_TIMER0->CC[0] - last_val);

        // Check if 1 second has passed and toggle the LED.
		// TODO
        if (diff == 1000000) {
            NRF_P0->OUT ^= BV(13);
            NRF_TIMER0->TASKS_CAPTURE[0] = 1;
            last_val = NRF_TIMER0->CC[0];
        }
    }
}
