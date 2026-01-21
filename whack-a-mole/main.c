#include "printf.h"
#include "helper.h"

void store_response_time(uint16_t response_time);
float get_average_response_time(void);

// SCORE & RESPONSE_TIME

static game_state_t current_game_state = STATE_START;

int main(void)
{
    // Configure LEDs
    configure_leds();
    // Configure buttons
    configure_buttons();
    // Configure the UART peripheral for printf.
    configure_uart();
    // Configure RNG peripheral
    configure_rng();
    // Configure timer for millis
    configure_milli_timer();

    // TODO: Implement the game logic

    // Run forever.
    static uint32_t round_count = 0;
    while (1)
    {
        // TODO: Implement the game logic

        switch (current_game_state)
        {
        case STATE_START:
            round_count = 0;
            current_round_points = 0;
            response_time_index = 0;
            const char *welcome_msg = "Welcome to the Embedded whack a 'led'\n";
            for (int i = 0; welcome_msg[i] != '\0'; i++)
            {
                NRF_UART0->TXD = welcome_msg[i];
                while (NRF_UART0->EVENTS_TXDRDY == 0)
                    ;
                NRF_UART0->EVENTS_TXDRDY = 0;
            }
            current_game_state = STATE_WAIT;
            break;
        case STATE_WAIT:
        {
            // a random delay

            if (round_count == MAX_RESPONSE_TIMES) // Maximum rounds reached
            {
                current_game_state = STATE_END;
                break;
            }

            uint16_t random_num = get_rng_value();
            wait_millis(random_num << 3);
            round_count++;
            current_game_state = STATE_ACTIVE;
            break;
        }
        case STATE_ACTIVE:
        {
            // main game logic

            // turn on random led
            uint16_t random_num = get_rng_value();
            uint16_t led_num = random_num >> 6;
            set_led(led_num, ON);

            uint32_t start_ms = get_millis();
            is_btn_pressed = false;

            // waiting for btn press in MAXIMUM_BUTTON_TIME time block
            while ((get_millis() - start_ms <= MAXIMUM_BUTTON_TIME) && !is_btn_pressed)
            {
                // do nothing
            }

            // reset LED state
            set_led(led_num, OFF);

            if (!is_btn_pressed)
            { // loop exit due to timeout
                handle_score(MISS);
            }
            else if (is_btn_pressed)
            { // loop exit due to btn_pressed withing window
                uint32_t current_time_ms = get_millis();
                uint32_t current_response_time_ms = (current_time_ms - start_ms);

                btn_t btn_pressed = which_button_pressed();

                if (btn_pressed != led_num)
                {
                    handle_score(MISS);
                    store_response_time(current_response_time_ms);
                }
                else if (current_response_time_ms <= INTERVAL_FOR_SCORING_POINTS)
                {
                    handle_score(HIT);
                    store_response_time(current_response_time_ms);
                }
                else // 700ms < response time < 2000ms
                {
                    handle_score(LATE_HIT);
                }
            }
            current_game_state = STATE_WAIT;
            break;
        }
        case STATE_END:
            // show response time on terminal via UART
            float user_response_time = get_average_response_time();
            response_time_index = 0;
            // sending byte by byte or convert to character array
            char buffer[64];

            snprintf(buffer, sizeof(buffer), "Avg res time: %.2f ms \n, Total Points: %d \n", user_response_time, current_round_points);

            for (int i = 0; buffer[i] != '\0'; i++)
            {
                NRF_UART0->TXD = buffer[i];
                while (NRF_UART0->EVENTS_TXDRDY == 0)
                    ;
                NRF_UART0->EVENTS_TXDRDY = 0;
            }

            const char *reset_msg = "Game Over! Press 'reset' to start game again.\n";
            for (int i = 0; reset_msg[i] != '\0'; i++)
            {
                NRF_UART0->TXD = reset_msg[i];
                while (NRF_UART0->EVENTS_TXDRDY == 0)
                    ;
                NRF_UART0->EVENTS_TXDRDY = 0;
            }

            while (1)
            {
                __WFI();
            }

            current_game_state = STATE_START;
            break;
        }
    }
    return 0;
}

void store_response_time(uint16_t response_time)
{
    if (response_time_index >= MAX_RESPONSE_TIMES)
    {
        response_time_index = 0;
    }
    response_times[response_time_index++] = response_time;
}

float get_average_response_time(void)
{
    float average_response_time = 0;
    if (response_time_index > 0)
    {
        for (uint8_t i = 0; i < response_time_index; i++)
        {
            average_response_time += response_times[i];
        }
        average_response_time /= response_time_index;
    }
    return average_response_time;
}
