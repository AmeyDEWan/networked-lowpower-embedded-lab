#include "helper.h"

const uint8_t led_pin_map[] = {13, 14, 15, 16};
const uint8_t btn_pin_map[] = {11, 12, 24, 25};

btn_t current_btn_state = BTN_NONE;
bool is_btn_pressed = false;

uint16_t response_times[MAX_RESPONSE_TIMES];
uint8_t response_time_index = 0;
int32_t current_round_points = 0;

//*************************************************************************
//* LED configuration.
//*     LED 1 to 4 on the Development Kit
//*************************************************************************
void configure_leds(void)
{
    // TODO: Configure all LEDs
    for (int i = 13; i <= 16; i++)
    {
        NRF_P0->PIN_CNF[i] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
                             BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);
    }
    NRF_P0->OUTSET = (BV(13) | BV(14) | BV(15) | BV(16));
}

//*************************************************************************
//* Set LED status
//*************************************************************************
void set_led(uint16_t led_nr, bool on)
{
    /* TODO: You can implement this helper function to simply write e.g. set_led(0, true) to enable LED 0
     * You can decide for yourself what index you want to use for what LED, but remember, that each LED
     * should correspond to a button
     */
    if (on)
    {
        NRF_P0->OUTCLR = BV(led_pin_map[led_nr]);
    }
    else
    {
        NRF_P0->OUTSET = BV(led_pin_map[led_nr]);
    }
}

//*************************************************************************
//* Buttons configuration.
//*     Buttons 1 to 4 on the Development Kit
//*************************************************************************
void configure_buttons(void)
{
    // TODO: Configure all buttons
    // Configure all btns as input + pullup
    for (int i = 0; i < 4; i++)
    {
        NRF_P0->PIN_CNF[btn_pin_map[i]] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) | BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect);
    }

    // Enable interrupts on Buttons - using peripheral GPIOE

    for (int i = 0; i < 4; i++)
    {
        // Configure channels of GPIOE
        NRF_GPIOTE->CONFIG[i] = BV_BY_NAME(GPIOTE_CONFIG_MODE, Event) | BV_BY_VALUE(GPIOTE_CONFIG_PSEL, btn_pin_map[i]) | BV_BY_NAME(GPIOTE_CONFIG_POLARITY, HiToLo);
        // Enable peripheral level interrupt
        NRF_GPIOTE->INTENSET |= BV(i);
        // Clear any interrupt flags
        NRF_GPIOTE->EVENTS_IN[i] = 0;
    }

    // Enable interrupt for GPIOTE on CPU level
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

//*************************************************************************
//* Check if any button is pressed
//*************************************************************************
btn_t which_button_pressed(void)
{
    /* TODO: You can implement this helper function to check if any button was  pressed and also return the index of what button was pressed
     */

    if (is_btn_pressed)
    {
        return current_btn_state;
    }
    return BTN_NONE;
}

void GPIOTE_IRQHandler(void)
{
    for (int i = 0; i < 4; i++)
    {
        // check EVENTS_IN register for pin that generated event
        if (NRF_GPIOTE->EVENTS_IN[i] == 1)
        {
            NRF_GPIOTE->EVENTS_IN[i] = 0;
            current_btn_state = (btn_t)i;
            is_btn_pressed = true;
            break;
        }
    }
}

//*************************************************************************
//* UART configuration.
//* 	- TX GPIO pin connection: P0.06
//* 	- baudrate: 115200 Baud (bit/s)
//* 	- hardware flow control: disabled
//* 	- stop bit(s): 1
//* 	- with no parity
//*************************************************************************
void configure_uart(void)
{
    NRF_UART0->PSEL.TXD = BV_BY_NAME(UART_PSEL_TXD_CONNECT, Connected) | BV_BY_VALUE(UART_PSEL_TXD_PORT, 0) | BV_BY_VALUE(UART_PSEL_TXD_PIN, 6);

    NRF_UART0->BAUDRATE = BV_BY_NAME(UART_BAUDRATE_BAUDRATE, Baud115200);
    NRF_UART0->CONFIG = BV_BY_NAME(UART_CONFIG_HWFC, Disabled) | BV_BY_NAME(UART_CONFIG_PARITY, Excluded) | BV_BY_NAME(UART_CONFIG_STOP, One);

    // Configure the TX GPIO pin according to Table 132, page 503.
    NRF_P0->PIN_CNF[6] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
                         BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect) |
                         BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
                         BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
                         BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);

    // Enable UART
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->TASKS_STARTTX = 1;

    // Set the output value of the GPIO pin (UART idle state).
    NRF_P0->OUTSET = BV(6);
}

//*************************************************************************
//* Configure random number generator RNG
//*************************************************************************
void configure_rng(void)
{
    // TODO: Configure the RNG peripheral. Look in the specifications document on what registers to set and how to configure them
    NRF_RNG->CONFIG = BV_BY_NAME(RNG_CONFIG_DERCEN, Enabled);
    NRF_RNG->TASKS_START = BV_BY_NAME(RNG_TASKS_START_TASKS_START, Trigger);
}

//*************************************************************************
//* Get RNG value
//*************************************************************************
uint8_t get_rng_value(void)
{
    // TODO: Generate a random value. Look in the specifications document on what register to read out

    while (NRF_RNG->EVENTS_VALRDY == 0)
        ;
    NRF_RNG->EVENTS_VALRDY = 0;

    uint16_t rng_val = NRF_RNG->VALUE;

    return rng_val;
}

//*************************************************************************
//* Configure millis timer
//*************************************************************************
void configure_milli_timer(void)
{
    // TODO: Configure a timer to count milliseconds or another format which suits you most

    // Configure the timer - prescalar, bitmode, mode
    // f_timer = 62,5Khz
    NRF_TIMER0->MODE = BV_BY_NAME(TIMER_MODE_MODE, Timer);
    NRF_TIMER0->PRESCALER = BV_BY_VALUE(TIMER_PRESCALER_PRESCALER, 8);
    NRF_TIMER0->BITMODE = BV_BY_NAME(TIMER_BITMODE_BITMODE, 32Bit);

    // TIMER1 for delay method
    NRF_TIMER1->MODE = BV_BY_NAME(TIMER_MODE_MODE, Timer);
    NRF_TIMER1->PRESCALER = BV_BY_VALUE(TIMER_PRESCALER_PRESCALER, 8);
    NRF_TIMER1->BITMODE = BV_BY_NAME(TIMER_BITMODE_BITMODE, 32Bit);

    // Clear timers to reset their values
    NRF_TIMER0->TASKS_CLEAR = BV_BY_NAME(TIMER_TASKS_CLEAR_TASKS_CLEAR, Trigger);
    NRF_TIMER1->TASKS_CLEAR = BV_BY_NAME(TIMER_TASKS_CLEAR_TASKS_CLEAR, Trigger);

    NRF_TIMER0->TASKS_START = BV_BY_NAME(TIMER_TASKS_START_TASKS_START, Trigger);
}

//*************************************************************************
//* Configure millis timer
//*************************************************************************
uint32_t get_millis(void)
{
    // TODO: Return the current timer value. Convert it to milliseconds or another format which suits you most

    // some ticks will be passed, capture them in a register
    NRF_TIMER0->TASKS_CAPTURE[0] = BV_BY_NAME(TIMER_TASKS_CAPTURE_TASKS_CAPTURE, Trigger);

    uint32_t current_val = NRF_TIMER0->CC[0];

    return (current_val >> 6);
}

//*************************************************************************
//* Wait given amount of millis
//*************************************************************************
void wait_millis(uint32_t milli_amount)
{
    // TODO: This is a helper function that helps to wait for the given amount of time in milliseconds. You can use another format which suits you most

    // Clear the timer
    NRF_TIMER1->TASKS_CLEAR = BV_BY_NAME(TIMER_TASKS_CLEAR_TASKS_CLEAR, Trigger);

    // Start the timer
    NRF_TIMER1->TASKS_START = BV_BY_NAME(TIMER_TASKS_START_TASKS_START, Trigger);

    // calculate ticks
    uint32_t target_ticks = milli_amount * TICKS_PER_MS;
    uint32_t current_ticks = 0;

    while (current_ticks < target_ticks)
    {
        NRF_TIMER1->TASKS_CAPTURE[0] = BV_BY_NAME(TIMER_TASKS_CAPTURE_TASKS_CAPTURE, Trigger);
        current_ticks = NRF_TIMER1->CC[0];
    }

    // Stop the timer
    NRF_TIMER1->TASKS_STOP = BV_BY_NAME(TIMER_TASKS_STOP_TASKS_STOP, Trigger);
}

void handle_score(round_res_t res)
{
    if (res == HIT)
    {
        current_round_points += 2;
    }
    else if (res == LATE_HIT)
    {
        current_round_points += 1;
    }
    else if (res == MISS)
    {
        current_round_points -= 1;
    }
}