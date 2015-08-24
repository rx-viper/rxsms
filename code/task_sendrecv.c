/*
 *   This file is part of RXSMS.
 *   Copyright 2014, 2015  Nicolas Benes
 *
 *   RXSMS is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   RXSMS is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with RXSMS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include "task_sendrecv.h"
#include "task_ctrl.h"

#define STATUS_LED_PORT         PORTC
#define STATUS_LED_UPLINK_bm    PIN0_bm
#define STATUS_LED_DOWNLINK_bm  PIN1_bm

#define UART_GROUNDSTATION_PORT     PORTD
#define UART_GROUNDSTATION_RX_bm    PIN2_bm
#define UART_GROUNDSTATION_TX_bm    PIN3_bm
#define UART_GROUNDSTATION          USARTD0
#define UART_GROUNDSTATION_PWRUP    PR.PRPD &= ~PR_USART0_bm

#define UART_EXPERIMENT_PORT        PORTC
#define UART_EXPERIMENT_RX_bm       PIN2_bm
#define UART_EXPERIMENT_TX_bm       PIN3_bm
#define UART_EXPERIMENT             USARTC0
#define UART_EXPERIMENT_PWRUP       PR.PRPC &= ~PR_USART0_bm

#define UART_BSEL       (12)
#define UART_BSCALE     (2)

static void init(void);
static void recv(void);
static void send(void);
// common init routine for both as they access the same hardware
const struct task task_recv = { .init = &init, .run = &recv };
const struct task task_send = { .init = &init, .run = &send };

static struct uart_status {
    uint8_t last_recv_ago;
    uint8_t led_toggle_interval;
} status_from_gnd, status_from_exp;

static void
init_uart(PORT_t * port, uint8_t rx_pin, uint8_t tx_pin, USART_t * uart)
{
    port->OUTSET = tx_pin;
    port->DIRSET = tx_pin;
    port->DIRCLR = rx_pin;
    uart->CTRLA = 0;
    uart->CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    uart->CTRLC = USART_CMODE_ASYNCHRONOUS_gc
        | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
    uart->BAUDCTRLA = (uint8_t) UART_BSEL;
    uart->BAUDCTRLB = (UART_BSCALE << 4) | (UART_BSEL >> 8);
}

static void
init(void)
{
    static uint8_t is_initialized = 0;
    if (is_initialized)
        return;                 // leave if init() is called multiple times
    is_initialized = 1;

    UART_GROUNDSTATION_PWRUP;
    UART_EXPERIMENT_PWRUP;

    STATUS_LED_PORT.OUTCLR = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;
    STATUS_LED_PORT.DIRSET = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;

    init_uart(&UART_GROUNDSTATION_PORT, UART_GROUNDSTATION_RX_bm,
              UART_GROUNDSTATION_TX_bm, &UART_GROUNDSTATION);
    init_uart(&UART_EXPERIMENT_PORT, UART_EXPERIMENT_RX_bm,
              UART_EXPERIMENT_TX_bm, &UART_EXPERIMENT);

    const struct task_recv_uart_data initializer = { 0 };
    task_recv_from_gnd = initializer;
    task_recv_from_exp = initializer;
    const struct uart_status status_init = { 0 };
    status_from_gnd = status_init;
    status_from_exp = status_init;
}

static void
recv_uart(USART_t * uart, struct task_recv_uart_data *data)
{
    uint8_t err_flags = USART_FERR_bm | USART_BUFOVF_bm | USART_PERR_bm;
    if (uart->STATUS & err_flags) {
        // clear error flags if set
        uint8_t ignored __attribute__ ((unused));
        ignored = uart->DATA;
        ignored = uart->DATA;
        return;
    }

    if (!(uart->STATUS & USART_RXCIF_bm))
        return;

    data->data = uart->DATA;
    data->updated = 1;
}

static void
recv(void)
{
    recv_uart(&UART_EXPERIMENT, &task_recv_from_exp);
    recv_uart(&UART_GROUNDSTATION, &task_recv_from_gnd);
}

static void
update_led(uint8_t has_received, struct uart_status *status,
           uint8_t ledmask)
{
    // switch off LED after this time of inactivity
    const uint8_t LED_TIMEOUT = 100;
    // period of flashing LED to indicate active communication
    const uint8_t LED_DURATION = LED_TIMEOUT;

    if (has_received) {
        status->last_recv_ago = 0;
        if (++status->led_toggle_interval >= LED_DURATION)
            status->led_toggle_interval = 0;    // wrap-around
        if (LED_DURATION / 2 == status->led_toggle_interval)
            STATUS_LED_PORT.OUTTGL = ledmask;   // a half square wave passed
    } else {
        // saturating add, clear LED due to inactivity when timeout reached
        if (++status->last_recv_ago >= LED_TIMEOUT) {
            status->last_recv_ago = LED_TIMEOUT;
            status->led_toggle_interval = 0;
            STATUS_LED_PORT.OUTCLR = ledmask;
        }
    }
}

static void
send_uart(USART_t * uart, struct task_recv_uart_data *data)
{
    if (!data->updated)
        return;                 // nothing to send

    if (!(uart->STATUS & USART_DREIF_bm))
        return;                 // USART busy, cannot send now, retry later

    uart->DATA = data->data;
    data->updated = 0;
}

static void
send(void)
{
    // after LO/umbilical disconnect, drop all incoming traffic from ground
    if (task_ctrl_signals.lo_asserted)
        task_recv_from_gnd.updated = 0;

    update_led(task_recv_from_exp.updated, &status_from_exp,
               STATUS_LED_DOWNLINK_bm);
    update_led(task_recv_from_gnd.updated, &status_from_gnd,
               STATUS_LED_UPLINK_bm);

    send_uart(&UART_GROUNDSTATION, &task_recv_from_exp);
    send_uart(&UART_EXPERIMENT, &task_recv_from_gnd);
}
