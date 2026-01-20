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
static void configure_uart(void);

int main(void)
{
    // Configure the UART peripheral for printf.
    configure_uart();

    //*************************************************************************
    //* I2C / TWI configuration.
    //*************************************************************************

    // Configure the PSEL.SCL, PSEL.SDA, and FREQUENCY register.
    // TODO
    NRF_TWIM0->PSEL.SCL = BV_BY_NAME(TWIM_PSEL_SCL_CONNECT, Connected) | BV_BY_VALUE(TWIM_PSEL_SCL_PORT, 0) | BV_BY_VALUE(TWIM_PSEL_SCL_PIN, 27); // P0.27
    NRF_TWIM0->PSEL.SDA = BV_BY_NAME(TWIM_PSEL_SDA_CONNECT, Connected) | BV_BY_VALUE(TWIM_PSEL_SDA_PORT, 0) | BV_BY_VALUE(TWIM_PSEL_SDA_PIN, 26); // P0.26
    NRF_TWIM0->FREQUENCY = BV_BY_NAME(TWIM_FREQUENCY_FREQUENCY, K250);

    // Configure SCL (Pin 27)
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

    // Run forever.
    while (1)
        ;
}

// Write n_data bytes of the buffer pointed to by data to the slave device with
// the address dev_addr.
static void twi_write(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data)
{
    // Enable the TWIM peripheral.
    // TODO
    NRF_TWIM0->ENABLE = 1;

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
    NRF_TWIM0->ENABLE = 1;

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
    NRF_TWIM0->ENABLE = 1;

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
