#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))


int main(void)
{
	// Configure GPIO that controls the LED.
	NRF_P0->PIN_CNF[13] =
		BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
		BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect) |
		BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
		BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
		BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
	NRF_P0->OUTCLR = BV(13); // active high

	// Use another GPIO that mirrors the LED behavior for the logic analyzer.
	NRF_P1->PIN_CNF[8] =
		BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
		BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect) |
		BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
		BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
		BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
	NRF_P1->OUTCLR = BV(8);

	uint32_t max_count = 100000;
	uint16_t direction = 0;
	uint32_t incr = 2000;
	uint32_t dc = 0;

	while (1)
	{
		uint32_t on_time = max_count - dc;
		uint32_t off_time = max_count - on_time;

		// Turn on LED.
		NRF_P0->OUTSET = BV(13);
		NRF_P1->OUTSET = BV(8);

		// Loop variable must be volatile to avoid that the compiler optimizes away the entire loop.
		volatile uint32_t i;
		// Busy waiting.
		for (i = 0; i < on_time; i++)
			;

		// Turn off LED.
		NRF_P0->OUTCLR = BV(13);
		NRF_P1->OUTCLR = BV(8);

		// Busy waiting.
		for (i = 0; i < off_time; i++)
			;

		// Change LED duty cycle.
		if (direction == 0)
		{
			if (dc < max_count)
				dc += incr;
			else
				direction = 1;
		}
		else
		{
			if (dc > 0)
				dc -= incr;
			else
				direction = 0;
		}
	}
}
