#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

// state machine
typedef enum
{
    SYS_STATE_DEFAULT_PATTERN,
    SYS_STATE_PATTERN_1,
    SYS_STATE_PATTERN_2
} system_state_t;

// // state machine
typedef enum
{
    STATE_ALL_ON,
    STATE_LED_1,
    STATE_LED_2,
    STATE_LED_3,
    STATE_LED_4,
    STATE_ALL_OFF
} led_state_t;

volatile bool timer_event_flag = false;

typedef struct
{
    volatile bool btn1_pressed;
    volatile bool btn2_pressed;
} btn_press_t;

void default_led_pattern(void);
void led_pattern_1(void);
void led_pattern_2(void);

btn_press_t btn_state = {false, false};
system_state_t current_sys_state = SYS_STATE_DEFAULT_PATTERN;
led_state_t current_led_state = STATE_ALL_OFF;

int main(void)
{
    // Configure button 1 and 2 for the input
    NRF_P0->PIN_CNF[11] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) | BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect);
    NRF_P0->PIN_CNF[12] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) | BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect);

    // Configure LEDs
    NRF_P0->PIN_CNF[13] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);
    NRF_P0->PIN_CNF[14] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);
    NRF_P0->PIN_CNF[15] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);
    NRF_P0->PIN_CNF[16] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);
    NRF_P0->OUTSET = BV(13);
    NRF_P0->OUTSET = BV(14);
    NRF_P0->OUTSET = BV(15);
    NRF_P0->OUTSET = BV(16);

    // Add interrupt for the button presses
    NRF_GPIOTE->CONFIG[0] = BV_BY_NAME(GPIOTE_CONFIG_MODE, Event) | BV_BY_VALUE(GPIOTE_CONFIG_PSEL, 11) | BV_BY_NAME(GPIOTE_CONFIG_POLARITY, HiToLo); // btn1
    NRF_GPIOTE->CONFIG[1] = BV_BY_NAME(GPIOTE_CONFIG_MODE, Event) | BV_BY_VALUE(GPIOTE_CONFIG_PSEL, 12) | BV_BY_NAME(GPIOTE_CONFIG_POLARITY, HiToLo); // btn2

    // Enable interrupts
    // Enable on peripheral level : enable interrupt on channel 0 that is watching pin defined in Config(P0.11)
    NRF_GPIOTE->INTENSET = BV_BY_NAME(GPIOTE_INTENSET_IN0, Set);
    NRF_GPIOTE->INTENSET = BV_BY_NAME(GPIOTE_INTENSET_IN1, Set);

    // Enable on CPU level
    NVIC_EnableIRQ(GPIOTE_IRQn);

    // Enable timer module with interrupt
    // Configure the timer.
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz

    // Enable interrupts for timer events.
    NRF_TIMER0->INTENSET = BV_BY_NAME(TIMER_INTENSET_COMPARE0, Enabled);

    // Enable interrupts for the timer peripheral in the interrupt controller.
    NVIC_EnableIRQ(TIMER0_IRQn);

    // using short to clear timer automatically when compare event generated
    NRF_TIMER0->SHORTS = BV_BY_NAME(TIMER_SHORTS_COMPARE0_CLEAR, Enabled);

    // Set target count value (when to trigger the interrupt).
    uint32_t count = 62500;
    NRF_TIMER0->CC[0] = count;
    // Start timer.
    NRF_TIMER0->TASKS_START = 1;

    while (1)
    {
        // check if button pressed
        __WFI();

        if (timer_event_flag)
        {
            timer_event_flag = false;
            system_state_t next_sys_state = current_sys_state;
            if (!btn_state.btn1_pressed && !btn_state.btn2_pressed)
            {
                // idle state
                next_sys_state = SYS_STATE_DEFAULT_PATTERN;
            }
            else if (btn_state.btn1_pressed)
            {
                // current_led_state = STATE_ALL_OFF;
                next_sys_state = SYS_STATE_PATTERN_1;
            }
            else if (btn_state.btn2_pressed)
            {
                // current_led_state = STATE_ALL_OFF;
                next_sys_state = SYS_STATE_PATTERN_2;
            }

            if (next_sys_state != current_sys_state)
            {
                // reset the "sticky" variable - neat
                current_led_state = STATE_ALL_OFF;
                current_sys_state = next_sys_state;
            }

            switch (current_sys_state)
            {
            case SYS_STATE_DEFAULT_PATTERN:
                default_led_pattern();
                break;
            case SYS_STATE_PATTERN_1:
                led_pattern_1();
                break;
            case SYS_STATE_PATTERN_2:
                led_pattern_2();
                break;
            }
        }
    }
}

// This is the interrupt service routine (ISR or IRQ) that is executed when a
// timer event occurs and interrupts are enables.
void TIMER0_IRQHandler(void)
{
    if (NRF_TIMER0->EVENTS_COMPARE[0])
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        timer_event_flag = true; // indication that timer event happend
    }
}

void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_IN[0])
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        btn_state.btn2_pressed = false;
        btn_state.btn1_pressed = true;
    }
    else if (NRF_GPIOTE->EVENTS_IN[1])
    {
        NRF_GPIOTE->EVENTS_IN[1] = 0;
        btn_state.btn1_pressed = false;
        btn_state.btn2_pressed = true;
    }
}

void default_led_pattern()
{
    switch (current_led_state)
    {
    case STATE_ALL_ON:
        // turn all on
        NRF_P0->OUTCLR = (BV(13) | BV(14) | BV(15) | BV(16));
        current_led_state = STATE_ALL_OFF;
        break;
    case STATE_ALL_OFF:
        NRF_P0->OUTSET = (BV(13) | BV(14) | BV(15) | BV(16));
        current_led_state = STATE_ALL_ON;
        break;
    }
}

void led_pattern_1()
{
    switch (current_led_state)
    {
    case STATE_ALL_OFF:
        // turn all off
        NRF_P0->OUTSET = (BV(13) | BV(14) | BV(15) | BV(16));
        current_led_state = STATE_LED_1;
        break;
    case STATE_LED_1:
        NRF_P0->OUTCLR = (BV(13) | BV(14));
        current_led_state = STATE_LED_2;
        break;
    case STATE_LED_2:
        NRF_P0->OUTSET = (BV(13) | BV(14));
        NRF_P0->OUTCLR = (BV(14) | BV(16));
        current_led_state = STATE_LED_3;
        break;
    case STATE_LED_3:
        NRF_P0->OUTSET = (BV(14) | BV(16));
        NRF_P0->OUTCLR = (BV(16) | BV(15));
        current_led_state = STATE_LED_4;
        break;
    case STATE_LED_4:
        NRF_P0->OUTSET = (BV(16) | BV(15));
        NRF_P0->OUTCLR = (BV(15) | BV(13));
        current_led_state = STATE_ALL_ON;
        break;
    case STATE_ALL_ON:
        NRF_P0->OUTSET = (BV(16) | BV(13));
        NRF_P0->OUTCLR = (BV(13) | BV(14) | BV(15) | BV(16));
        current_led_state = STATE_ALL_OFF;
        break;
    }
}

void led_pattern_2()
{
    switch (current_led_state)
    {
    case STATE_ALL_OFF:
        // turn all off
        NRF_P0->OUTSET = (BV(13) | BV(14) | BV(15) | BV(16));
        current_led_state = STATE_LED_1;
        break;
    case STATE_LED_1:
        NRF_P0->OUTCLR = BV(13);
        current_led_state = STATE_LED_2;
        break;
    case STATE_LED_2:
        NRF_P0->OUTSET = BV(13);
        NRF_P0->OUTCLR = BV(14);
        current_led_state = STATE_LED_3;
        break;
    case STATE_LED_3:
        NRF_P0->OUTSET = BV(14);
        NRF_P0->OUTCLR = BV(16);
        current_led_state = STATE_LED_4;
        break;
    case STATE_LED_4:
        NRF_P0->OUTSET = BV(16);
        NRF_P0->OUTCLR = BV(15);
        current_led_state = STATE_ALL_ON;
        break;
    case STATE_ALL_ON:
        NRF_P0->OUTSET = BV(15);
        NRF_P0->OUTCLR = (BV(13) | BV(14) | BV(15) | BV(16));
        current_led_state = STATE_ALL_OFF;
        break;
    }
}