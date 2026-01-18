#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

// state machine
typedef enum {
    STATE_INIT,
    STATE_LED_1,
    STATE_LED_2,
    STATE_LED_3,
    STATE_LED_4,
    STATE_ALL_OFF
} system_state_t;

volatile bool timer_event_flag = false;


int main(void)
{
    // Configure the GPIO pin of the LED.
    // TODO
    NRF_P0->PIN_CNF[13] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output);
    NRF_P0->PIN_CNF[14] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output);
    NRF_P0->PIN_CNF[15] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output);
    NRF_P0->PIN_CNF[16] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output);

    // Mirror LED behavior to another pin (e.g. P1.08) that we can inspect with the LA.
    NRF_P1->PIN_CNF[8] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output);

    // Configure the timer.
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz

    // Enable interrupts for timer events.
    // TODO
    NRF_TIMER0->INTENSET = BV_BY_NAME(TIMER_INTENSET_COMPARE0, Enabled);

    // Enable interrupts for the timer peripheral in the interrupt controller.
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Clear to ensure the timer counter is 0.
    // TODO
    //using short to clear timer automatically when compare event generated
    NRF_TIMER0->SHORTS = BV_BY_NAME(TIMER_SHORTS_COMPARE0_CLEAR, Enabled);

    // Set target count value (when to trigger the interrupt).
    // TODO
    uint32_t count = 62500;
    NRF_TIMER0->CC[0] = count;
    // Start timer.
    // TODO
    NRF_TIMER0->TASKS_START = 1;

    system_state_t current_state = STATE_INIT;

    while (1)
    {
        // Go into a low power state (i.e., sleep) until an interrupt occurs
        // (WFI = "wait for interrupt" is a processor instruction).
        __WFI();

        if (timer_event_flag == true) {
            timer_event_flag = false;

            switch(current_state) {
                case STATE_INIT:
                    //turn all off
                    NRF_P0->OUTSET = (BV(13) | BV(14) | BV(15) | BV(16));
                    current_state = STATE_LED_1;
                    break;
                case STATE_LED_1:
                    NRF_P0->OUTCLR = BV(13);
                    current_state = STATE_LED_2;
                    break;
                case STATE_LED_2:
                    NRF_P0->OUTSET = BV(13);
                    NRF_P0->OUTCLR = BV(14);
                    current_state = STATE_LED_3;
                    break;
                case STATE_LED_3:
                    NRF_P0->OUTSET = BV(14);
                    NRF_P0->OUTCLR = BV(15);
                    current_state = STATE_LED_4;
                    break;
                case STATE_LED_4:
                    NRF_P0->OUTSET = BV(15);
                    NRF_P0->OUTCLR = BV(16);
                    current_state = STATE_ALL_OFF;
                    break;  
                case STATE_ALL_OFF:
                    NRF_P0->OUTSET = BV(16);
                    NRF_P0->OUTCLR = (BV(13) | BV(14) | BV(15) | BV(16));
                    current_state = STATE_INIT;
                    break;     
            }
        }


    }
}

// This is the interrupt service routine (ISR or IRQ) that is executed when a
// timer event occurs and interrupts are enables.
void TIMER0_IRQHandler(void)
{
    // // Check if our specific event triggered the interrupt.

    // // Clear interrupt events if necessary

    // // TODO
    // if (NRF_TIMER0->EVENTS_COMPARE[0]) {
    //     NRF_TIMER0->EVENTS_COMPARE[0] = 0;

    //     NRF_P0->OUT ^= BV(13);
    //     NRF_P1->OUT ^= BV(8);

    //     //NRF_TIMER0->TASKS_CLEAR = 1;
    // }
    // else if (NRF_TIMER0->EVENTS_COMPARE[1]) {
    //     NRF_TIMER0->EVENTS_COMPARE[1] = 0;

    //     NRF_P0->OUT ^= BV(14);

    //     //NRF_TIMER0->TASKS_CLEAR = 1;
    // }
    // else if (NRF_TIMER0->EVENTS_COMPARE[2]) {
    //     NRF_TIMER0->EVENTS_COMPARE[2] = 0;

    //     NRF_P0->OUT ^= BV(15);

    //     //NRF_TIMER0->TASKS_CLEAR = 1;
    // }
    // else if (NRF_TIMER0->EVENTS_COMPARE[3]) {
    //     NRF_TIMER0->EVENTS_COMPARE[3] = 0;

    //     NRF_P0->OUT ^= BV(16);

    //     NRF_TIMER0->TASKS_CLEAR = 1;
    // }


    // keeping ISR very small by just a shared bool flag
    if (NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        timer_event_flag = true; //indication that timer event happend
    }
}