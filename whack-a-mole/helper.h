#ifndef _HELPER_H_
#define _HELPER_H_

#include <stdint.h>
#include <nrf.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include "nrf.h"

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

// Macros
#define MAX_RESPONSE_TIMES 7
#define MAXIMUM_BUTTON_TIME 2000
#define INTERVAL_FOR_SCORING_POINTS 700
#define TICKS_PER_MS 64 // rounded (62,5) - closeset value that is pow(2)
#define ON true
#define OFF false

// Result of a round
typedef enum
{
    MISS,
    HIT,
    LATE_HIT
} round_res_t;

// LED number
typedef enum
{
    LED_1 = 0,
    LED_2,
    LED_3,
    LED_4
} led_t;

// Button Number
typedef enum
{
    BTN_1 = 0,
    BTN_2,
    BTN_3,
    BTN_4,
    BTN_NONE
} btn_t;

// Game state
typedef enum
{
    STATE_START,
    STATE_WAIT,
    STATE_ACTIVE,
    STATE_END
} game_state_t;

// Variable declarations
extern const uint8_t led_pin_map[];
extern const uint8_t btn_pin_map[];
extern btn_t current_btn_state;
extern bool is_btn_pressed;
extern uint16_t response_times[]; // Circular buffer for response times
extern uint8_t response_time_index;
extern int32_t current_round_points;

// Function prototypes
void configure_leds(void);
void set_led(uint16_t led_nr, bool on);
void configure_buttons(void);
btn_t which_button_pressed(void);
void configure_uart(void);
void configure_rng(void);
uint8_t get_rng_value(void);
void configure_milli_timer(void);
uint32_t get_millis(void);
void wait_millis(uint32_t milli_amount);
void handle_score(round_res_t res);

#endif // _HELPER_H_