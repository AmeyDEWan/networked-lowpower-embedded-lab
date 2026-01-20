#include <stdint.h>
#include <nrf.h>
#include "printf.h"

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

int main(void)
{
    //*************************************************************************
    //* UART configuration.
    //* 	- TX GPIO pin connection: P0.06
    //* 	- baudrate: 115200 Baud (bit/s)
    //* 	- hardware flow control: disabled
    //* 	- stop bit(s): 1
    //* 	- with no parity
    //*************************************************************************
    // TODO

    // Configure the TX GPIO pin according to Table 132, page 503.
    // TODO
    NRF_UART0->PSEL.TXD = BV_BY_NAME(UART_PSEL_TXD_CONNECT, Connected) | BV_BY_VALUE(UART_PSEL_TXD_PORT, 0) | BV_BY_VALUE(UART_PSEL_TXD_PIN, 6);

    NRF_UART0->BAUDRATE = BV_BY_NAME(UART_BAUDRATE_BAUDRATE, Baud115200);

    NRF_UART0->CONFIG = BV_BY_NAME(UART_CONFIG_HWFC, Disabled) | BV_BY_NAME(UART_CONFIG_PARITY, Excluded) | BV_BY_NAME(UART_CONFIG_STOP, One);

    // Set the output value of the GPIO pin (UART idle state).
    // TODO
    NRF_P0->OUTSET = BV(6); // idle high
    NRF_P0->PIN_CNF[6] =
        BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
        BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect) |
        BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
        BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
        BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);

    // Enable the UART peripheral.
    // TODO
    NRF_UART0->ENABLE = BV_BY_NAME(UART_ENABLE_ENABLE, Enabled);

    // Start the UART transfer (i.e., trigger the UART start task).
    // TODO
    NRF_UART0->TASKS_STARTTX = 1;

    char ch = 0;
    // run forever
    while (1)
    {
        // Write the character into the TXD register.
        // TODO
        NRF_UART0->TXD = ch;

        // Wait until the end of the transmission by checking the TXRDY event.
        // TODO

        while (NRF_UART0->EVENTS_TXDRDY == 0)
            ;
        // Reset the event.
        // TODO
        NRF_UART0->EVENTS_TXDRDY = 0;

        for (volatile int i = 0; i < 10000; i++);   // a delay to see b/w frames

        // Increment the value of our character to iterate over all ASCII symbols.
        ch++;
    }
}
