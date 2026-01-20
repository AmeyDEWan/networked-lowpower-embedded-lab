#include <stdint.h>
#include <nrf.h>
#include "printf.h"

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

const char* str = "hello world!";

int main(void)
{
    //*************************************************************************
    //* Timer configuration:
    //* 	- frequency: 62.500 Hz
    //* 	- bit width: 16 bit = 65.536
    //*     - interrupts are enabled and trigger every second
    //*************************************************************************
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz
    // Enable interrupts for timer events.
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos;
    // Enable interrupts for the timer peripheral in the interrupt controller.
    NVIC_EnableIRQ(TIMER0_IRQn);
    // Clear to ensure the timer counter is 0.
    NRF_TIMER0->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos;
    // Set target count value (i.e., trigger the interrupt after 1 s).
    NRF_TIMER0->CC[0] = 62500;

    //*************************************************************************
    //* UART configuration.
    //* 	- TX GPIO pin connection: P0.06
    //* 	- baudrate: 115200 Baud (bit/s)
    //* 	- hardware flow control: disabled
    //* 	- stop bit(s): 1
    //* 	- with no parity
    //*************************************************************************
    NRF_UART0->PSEL.TXD = (6 << UART_PSEL_TXD_PIN_Pos) |
                          (0 << UART_PSEL_TXD_PORT_Pos) |
                          (UART_PSEL_TXD_CONNECT_Connected << UART_PSEL_TXD_CONNECT_Pos);
    NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos;
    NRF_UART0->CONFIG = (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos) |
                        (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) |
                        (UART_CONFIG_STOP_One << UART_CONFIG_STOP_Pos);

    // Configure the TX GPIO pin according to Table 132, page 503.
    NRF_P0->PIN_CNF[6] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                         (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                         (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                         (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                         (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    // Set the output value of the GPIO pin (UART idle state).
    NRF_P0->OUTSET = GPIO_OUTSET_PIN6_Set << GPIO_OUTSET_PIN6_Pos;

    // Start timer.
    NRF_TIMER0->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos;

    // run forever
    while (1)
    {
        // Go into a low power state (i.e., sleep) until an interrupt occurs
        // (WFI = "wait for interrupt" is a processor instruction).
        __WFI();
    }
}

// This is the interrupt service routine (ISR or IRQ) that is executed when a
// timer event (from timer instance TIMER0) occurs and interrupts are enables.
void TIMER0_IRQHandler(void)
{
    // Check if our specific event triggered the interrupt.
    if ((NRF_TIMER0->EVENTS_COMPARE[0] & TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Msk) == TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Generated)
    {
        // Clear interrupt event.
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        // Update the target count value to trigger the interrupt again after one second.
        NRF_TIMER0->CC[0] += 62500;

        // Output some data via printf.
        // TODO

        printf(str);
    }
}

// This function is called internally by the printf function for each character.
// Here we can redirect the printf output stream to whereever we want.
void _putchar(char character)
{
    // Enable the UART peripheral.
    // TODO
    NRF_UART0->ENABLE = BV_BY_NAME(UART_ENABLE_ENABLE, Enabled);

    // Write the character into the TXD register.
    // TODO
    NRF_UART0->TXD = character;

    // Start the UART transfer (i.e., trigger the UART start task).
    // TODO
    NRF_UART0->TASKS_STARTTX = 1;

    // Wait until the end of the transmission by checking the TXRDY event.
    // TODO
    while(NRF_UART0->EVENTS_TXDRDY == 0);

    // Reset the event.
    // TODO
    NRF_UART0->EVENTS_TXDRDY = 0;

    // Stop the transmission by triggering the stop task.
    // TODO
    NRF_UART0->TASKS_STOPTX = 1;

    // Disable the UART peripheral.
    // TODO
    NRF_UART0->ENABLE = BV_BY_NAME(UART_ENABLE_ENABLE, Disabled);
}
