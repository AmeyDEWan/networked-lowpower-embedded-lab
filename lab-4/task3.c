#include <stdint.h>
#include <nrf.h>
#include <string.h>
#include <inttypes.h>
#include "printf.h"

// Helper macros to access registers.
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)
#define BV(pos) (1u << (pos))

// Forward declarations.
static void twi_write(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data);
static void twi_read(uint8_t dev_addr, volatile uint8_t *rx_buffer, unsigned int n_rx_data);
static void twi_write_read(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data, volatile uint8_t *rx_buffer, unsigned int n_rx_data);
static void wait_ms(uint16_t ms);
static void configure_timer(void);
static void configure_uart(void);

// Defines.
#define TICKS_PER_MS 63 // rounded (62,5)

// TODO: define device address according to the datasheet
#define DEV_ADDR UINT8_C(0x1D)

// TODO: define chip id register address according to the datasheet
#define REG_CHIP_ID UINT8_C(0x0D)

// TODO: define control register address according to the datasheet
#define REG_CONTROL UINT8_C(0x2A)

// TODO: define control mode active and inactive according to the datasheet
#define MODE_INACTIVE UINT8_C(0x00)
#define MODE_ACTIVE UINT8_C(0x01)

// TODO: define configuration register address according to the datasheet
#define REG_CONFIG UINT8_C(0x0E)

// TODO: define configured range value to default of +/- 2g
#define RANGE_2G UINT8_C(0x00)

// TODO: define register address of first data (OUT_X_MSB)
#define REG_OUT_X_MSB UINT8_C(0x01)

// define LED states wrt Orientation
#define X_UP (BV(13) | BV(14))
#define X_DOWN (BV(15) | BV(16))
#define Y_UP (BV(13) | BV(15))
#define Y_DOWN (BV(14) | BV(16))
#define Z_UP (BV(13) | BV(16))
#define Z_DOWN (BV(15) | BV(14))
#define ALL_LEDS (BV(13) | BV(14) | BV(15) | BV(16))

// Read and write buffers.
// TODO
#define N_TX_DATA 64
#define N_RX_DATA 64
volatile uint8_t rx_buffer[N_RX_DATA];
volatile uint8_t tx_buffer[N_TX_DATA];

int main(void)
{
    // Configure the UART peripheral for printf.
    configure_uart();
    // Configure the TIMER peripheral for the wait_ms() function and an interrupt
    // every 0.5 seconds.
    configure_timer();

    //*************************************************************************
    //* I2C / TWI configuration.
    //*************************************************************************

    // Configure the PSEL.SCL, PSEL.SDA, and FREQUENCY register.
    // TODO (Copy the code from task 1.)
    NRF_TWIM0->PSEL.SCL = BV_BY_NAME(TWIM_PSEL_SCL_CONNECT, Connected) | BV_BY_VALUE(TWIM_PSEL_SCL_PORT, 0) | BV_BY_VALUE(TWIM_PSEL_SCL_PIN, 27); // P0.27
    NRF_TWIM0->PSEL.SDA = BV_BY_NAME(TWIM_PSEL_SDA_CONNECT, Connected) | BV_BY_VALUE(TWIM_PSEL_SDA_PORT, 0) | BV_BY_VALUE(TWIM_PSEL_SDA_PIN, 26); // P0.26
    NRF_TWIM0->FREQUENCY = BV_BY_NAME(TWIM_FREQUENCY_FREQUENCY, K250);
    // Configure the GPIO pins used with the TWIM peripheral according to Table 124, page 473.
    // TODO (Copy the code from task 1.)
    NRF_P0->PIN_CNF[27] = BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled) |
                          BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0D1) |
                          BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
                          BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
                          BV_BY_NAME(GPIO_PIN_CNF_DIR, Input); // Peripheral takes over direction, but Input is safe default

    // Configure SDA (Pin 26)
    NRF_P0->PIN_CNF[26] = BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled) |
                          BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0D1) |
                          BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
                          BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
                          BV_BY_NAME(GPIO_PIN_CNF_DIR, Input);

    // Debug LEDs
    for (int i = 13; i <= 16; i++)
    {
        NRF_P0->PIN_CNF[i] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
                             BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
                             BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect);
    }
    NRF_P0->OUTSET = ALL_LEDS;

    //*************************************************************************
    //* Sensor configuration.
    //*************************************************************************
    // Writing to certain registers including the one configuring the dynamic
    // range requires the sensor to be in standby (inactive) mode. Set the
    // sensor to standby mode, set the dynamic range to 2G and then put it in
    // active mode.

    // Store the address of the control register in our TX buffer.
    // TODO
    tx_buffer[0] = REG_CONTROL;

    // Store the value representing the inactive operation mode in our TX buffer.
    // TODO
    tx_buffer[1] = MODE_INACTIVE;

    // Write the buffer contents to the accelerometer.
    // TODO
    twi_write(DEV_ADDR, tx_buffer, 2);

    // Store the address of the configuration register in our TX buffer.
    // TODO
    tx_buffer[0] = REG_CONFIG;

    // Store the value representing the +/- 2g range in our TX buffer.
    // TODO
    tx_buffer[1] = RANGE_2G;

    // Write the buffer contents to the accelerometer.
    // TODO
    twi_write(DEV_ADDR, tx_buffer, 2);

    // Store the address of the control register in our TX buffer.
    // TODO
    tx_buffer[0] = REG_CONTROL;

    // Store the value representing the active operation mode in our TX buffer.
    // TODO
    tx_buffer[1] = MODE_ACTIVE;

    // Write the buffer contents to the accelerometer.
    // TODO
    twi_write(DEV_ADDR, tx_buffer, 2);

    // Run forever.
    while (1)
    {
        // Go into a low power state (i.e., sleep) until an interrupt occurs
        // (WFI = "wait for interrupt" is a processor instruction).
        __WFI();
    }
}

// This is the interrupt service routine (ISR or IRQ) that is executed when a
// timer event occurs and interrupts are enables.
void TIMER1_IRQHandler(void)
{
    // Check if our specific event triggered the interrupt.
    if ((NRF_TIMER1->EVENTS_COMPARE[0] & TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Msk) == TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Generated)
    {
        // Clear interrupt event.
        NRF_TIMER1->EVENTS_COMPARE[0] = TIMER_EVENTS_COMPARE_EVENTS_COMPARE_NotGenerated << TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Pos;

        // Store the address of the acceleration data (starts at register OUT_X_MSB)
        // in our TX buffer.
        // TODO
        tx_buffer[0] = REG_OUT_X_MSB;

        // Write to the accelerometer in order to setup the read transfer.
        // Use twi_read(), twi_write(), or twi_write_read() as necessary according to the datasheet.
        // TODO
        // do it with the twi_write_read()

        // Start a read transfer to receive 6 bytes of acceleration data.
        // TODO
        twi_write_read(DEV_ADDR, tx_buffer, 1, rx_buffer, 6);

        // Combine LSB and MSB sensor values for the X axis.
        // TODO
        int16_t x = ((int16_t)rx_buffer[0] << 4) | (rx_buffer[1] >> 4);
        if (x > 2047)
            x -= 4096;

        // Combine LSB and MSB sensor values for the Y axis.
        // TODO
        int16_t y = ((int16_t)rx_buffer[2] << 4) | (rx_buffer[3] >> 4);
        if (y > 2047)
            y -= 4096;

        // Combine LSB and MSB sensor values for the Z axis.
        // TODO
        int16_t z = ((int16_t)rx_buffer[4] << 4) | (rx_buffer[5] >> 4);
        if (z > 2047)
            z -= 4096;

        // Fancy
        // Determine orientation
        int16_t abs_x = (x < 0) ? -x : x;
        int16_t abs_y = (y < 0) ? -y : y;
        int16_t abs_z = (z < 0) ? -z : z;

        // Determine the current orientation.
        NRF_P0->OUTSET = ALL_LEDS;

        const char *orient_str;

        // Print acceleration values.
        // TODO
        printf("X: %5d, Y: %5d, Z: %5d | %s\r\n", x, y, z, orient_str);

        if (abs_x > abs_y && abs_x > abs_z)
        {
            if (x > 0)
            {
                orient_str = "X UP";
                NRF_P0->OUTCLR = X_UP; // Turn ON
            }
            else
            {
                orient_str = "X DOWN";
                NRF_P0->OUTCLR = X_DOWN; // Turn ON
            }
        }
        else if (abs_y > abs_x && abs_y > abs_z)
        {
            if (y > 0)
            {
                orient_str = "Y UP";
                NRF_P0->OUTCLR = Y_UP; // Turn ON
            }
            else
            {
                orient_str = "Y DOWN";
                NRF_P0->OUTCLR = Y_DOWN; // Turn ON
            }
        }
        else
        {
            if (z > 0)
            {
                orient_str = "Z UP";
                NRF_P0->OUTCLR = Z_UP; // Turn ON
            }
            else
            {
                orient_str = "Z DOWN";
                NRF_P0->OUTCLR = Z_DOWN; // Turn ON
            }
        }

        // Clear to ensure the timer counter is 0.
        NRF_TIMER1->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos;
        // Update the target count value to trigger the interrupt again after 0.5 seconds.
        NRF_TIMER1->CC[0] = 31250;
    }
}

// Write n_data bytes of the buffer pointed to by data to the slave device with
// the address dev_addr.
static void twi_write(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data)
{
    // Enable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = BV_BY_NAME(TWIM_ENABLE_ENABLE, Enabled);

    // Set the shortcut to stop transmitting after the last byte.
    // TODO
    NRF_TWIM0->SHORTS = BV_BY_NAME(TWIM_SHORTS_LASTTX_STOP, Enabled);

    // Set the device address in the ADDRESS register.
    // TODO
    NRF_TWIM0->ADDRESS = dev_addr;

    // Set the number of bytes we want to transmit.
    // TODO
    NRF_TWIM0->TXD.MAXCNT = n_tx_data;

    // Provide a pointer to a buffer where the transmit data is stored.
    // TODO
    NRF_TWIM0->TXD.PTR = (uint32_t)tx_data;

    // Clear events.
    NRF_TWIM0->EVENTS_STOPPED = TWIM_EVENTS_STOPPED_EVENTS_STOPPED_NotGenerated << TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Pos;
    NRF_TWIM0->EVENTS_ERROR = TWIM_EVENTS_ERROR_EVENTS_ERROR_NotGenerated << TWIM_EVENTS_ERROR_EVENTS_ERROR_Pos;

    // Start transmit task.
    // TODO
    NRF_TWIM0->TASKS_STARTTX = 1;

    // Wait until the TWIM peripheral has stopped (STOP event after the last byte).
    while (NRF_TWIM0->EVENTS_STOPPED != TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Generated)
        ;

    // Disable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = 0;
}

// Read n_data bytes from the slave device with the address dev_addr and store the
// data at the location pointed to by buffer.
static void twi_read(uint8_t dev_addr, volatile uint8_t *rx_buffer, unsigned int n_rx_data)
{
    // Enable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = BV_BY_NAME(TWIM_ENABLE_ENABLE, Enabled);
    // Set the shortcut to stop receiving after the last byte.
    // TODO
    NRF_TWIM0->SHORTS = BV_BY_NAME(TWIM_SHORTS_LASTRX_STOP, Enabled);

    // Set the device address in the ADDRESS register.
    // TODO
    NRF_TWIM0->ADDRESS = dev_addr;

    // Set the number of bytes we want to receive.
    // TODO
    NRF_TWIM0->RXD.MAXCNT = n_rx_data;

    // Provide a pointer to a buffer where the received data can be stored.
    // TODO
    NRF_TWIM0->RXD.PTR = (uint32_t)rx_buffer;

    // Clear events.
    NRF_TWIM0->EVENTS_STOPPED = TWIM_EVENTS_STOPPED_EVENTS_STOPPED_NotGenerated << TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Pos;
    NRF_TWIM0->EVENTS_ERROR = TWIM_EVENTS_ERROR_EVENTS_ERROR_NotGenerated << TWIM_EVENTS_ERROR_EVENTS_ERROR_Pos;

    // Start receive task.
    // TODO
    NRF_TWIM0->TASKS_STARTRX = 1;

    // Wait until the TWIM peripheral has stopped (STOP event after the last byte).
    while (NRF_TWIM0->EVENTS_STOPPED != TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Generated)
        ;

    // Disable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = 0;
}

// Write n_data bytes of the buffer pointed to by data to the slave device with
// the address dev_addr without sending the stop condition.
// Then read n_data bytes from the slave device with the address dev_addr and store the
// data at the location pointed to by buffer.
static void twi_write_read(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data, volatile uint8_t *rx_buffer, unsigned int n_rx_data)
{
    // Enable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = BV_BY_NAME(TWIM_ENABLE_ENABLE, Enabled);

    // Set the shortcut to start receiving after the last byte is transmitted
    // and to stop receiving after the last byte.
    // TODO
    NRF_TWIM0->SHORTS = BV_BY_NAME(TWIM_SHORTS_LASTTX_STARTRX, Enabled) | BV_BY_NAME(TWIM_SHORTS_LASTRX_STOP, Enabled);

    // Set the device address in the ADDRESS register.
    // TODO
    NRF_TWIM0->ADDRESS = dev_addr;

    // Set the number of bytes we want to transmit.
    // TODO
    NRF_TWIM0->TXD.MAXCNT = n_tx_data;

    // Provide a pointer to a buffer where the transmit data is stored.
    // TODO
    NRF_TWIM0->TXD.PTR = (uint32_t)tx_data;

    // Set the number of bytes we want to receive.
    // TODO
    NRF_TWIM0->RXD.MAXCNT = n_rx_data;

    // Provide a pointer to a buffer where the received data can be stored.
    // TODO
    NRF_TWIM0->RXD.PTR = (uint32_t)rx_buffer;

    // Clear events.
    NRF_TWIM0->EVENTS_STOPPED = TWIM_EVENTS_STOPPED_EVENTS_STOPPED_NotGenerated << TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Pos;
    NRF_TWIM0->EVENTS_ERROR = TWIM_EVENTS_ERROR_EVENTS_ERROR_NotGenerated << TWIM_EVENTS_ERROR_EVENTS_ERROR_Pos;

    // Start transmit task.
    // TODO
    NRF_TWIM0->TASKS_STARTTX = 1;

    // Wait until the TWIM peripheral has stopped (STOP event after the last byte).
    while (NRF_TWIM0->EVENTS_STOPPED != TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Generated)
        ;

    // Disable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = 0;
}

//*************************************************************************
//* UART configuration.
//* 	- TX GPIO pin connection: P0.06
//* 	- baudrate: 115200 Baud (bit/s)
//* 	- hardware flow control: disabled
//* 	- stop bit(s): 1
//* 	- with no parity
//*************************************************************************
static void configure_uart(void)
{
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
}

//*************************************************************************
//* Timer0 configuration:
//* 	- frequency: 62.500 Hz
//* 	- bit width: 16 bit = 65.536
//
//* Timer1 configuration:
//* 	- frequency: 62.500 Hz
//* 	- bit width: 16 bit = 65.536
//*     - interrupts are enabled and trigger every 0.5 seconds
//*************************************************************************
static void configure_timer(void)
{
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz

    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER1->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz

    // Enable interrupts for timer events.
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos;
    // Enable interrupts for the timer peripheral in the interrupt controller.
    NVIC_EnableIRQ(TIMER1_IRQn);

    // Clear to ensure the timer counter is 0.
    NRF_TIMER1->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos;

    // Set target count value (when to trigger the interrupt).
    NRF_TIMER1->CC[0] = 31250; // 500ms

    // Start timer.
    NRF_TIMER1->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos;
}

// Delays the execution between 1 and 999 ms.
static void wait_ms(uint16_t ms)
{
    if (ms > 1000)
        return;

    // Clear and start timer.
    NRF_TIMER0->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos;
    NRF_TIMER0->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos;

    // Set target counter value.
    uint16_t counter = 0;
    uint16_t target_count = TICKS_PER_MS * ms;

    // Check timer counter and wait until target_count value ticks have passed.
    while (counter < target_count)
    {
        NRF_TIMER0->TASKS_CAPTURE[0] = TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Trigger << TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Pos;
        counter = (uint16_t)NRF_TIMER0->CC[0];
    }

    // Stop timer.
    NRF_TIMER0->TASKS_STOP = TIMER_TASKS_STOP_TASKS_STOP_Trigger << TIMER_TASKS_STOP_TASKS_STOP_Pos;
}
