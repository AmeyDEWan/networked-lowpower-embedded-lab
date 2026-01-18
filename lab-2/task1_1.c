#include <stdint.h>

// Green LED -> Port 0, pin 13(P0.13)
// Bare metal driving of output pin to 13

int main(void)
{
    // Store the address of the GPIO pin configuration register of the LED.
	// TODO
    uint32_t P0_base = 0x50000000; // Base address for GPIO Port 0
    uint32_t P0_led_cfg_reg_offset = 0x734; // Offset for pin configuration register
    uint32_t P0_outset = 0x508;
    
    // Configure the GPIO pin.
	// TODO
    volatile uint32_t* p0_led_cfg_reg = (uint32_t *) (P0_base + P0_led_cfg_reg_offset);
    *p0_led_cfg_reg = (1 << 0); // DIR = 0 => output

    // Store the address of the register which controls the LED.
	// TODO
    volatile uint32_t* p0_led_outset_reg = (uint32_t *)(P0_base + p0_led_outset_reg);

    // Activate the LED.
	// TODO
    *p0_led_outset_reg = (1 << 13);

    // run forever
    while (1)
        ;
}
