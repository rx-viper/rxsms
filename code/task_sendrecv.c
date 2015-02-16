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
#include <stdlib.h>
#include "task_sendrecv.h"
#include "task_ctrl.h"
#include "task_sample_adc_inputs.h"  /* FIXME remove when not needed */

#define STATUS_LED_PORT         PORTD
#define STATUS_LED_UPLINK_bm    PIN4_bm
#define STATUS_LED_DOWNLINK_bm  PIN5_bm

#define UART_GROUNDSTATION_PORT     PORTD
#define UART_GROUNDSTATION_RX_bm    PIN6_bm
#define UART_GROUNDSTATION_TX_bm    PIN7_bm
#define UART_GROUNDSTATION          USARTD1

#define UART_EXPERIMENT_PORT        PORTD
#define UART_EXPERIMENT_RX_bm       PIN2_bm
#define UART_EXPERIMENT_TX_bm       PIN3_bm
#define UART_EXPERIMENT             USARTD0

#define UART_BSEL       (12)
#define UART_BSCALE     (2)

#define LED_DURATION    (250)
#if LED_DURATION > 255
#error task_recv_uart_data.inactivity and .led_toggle_interval are 8 bits only.
#endif

static void init(void);
static void recv(void);
static void send(void);
/* common init routine for both as they access the same hardware */
const struct task task_recv = { .init = &init, .run = &recv };
const struct task task_send = { .init = &init, .run = &send };

static void
init_uart(PORT_t *port, uint8_t rx_pin, uint8_t tx_pin, USART_t *uart)
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
        return;     /* leave if init() is called multiple times */
    is_initialized = 1;

    STATUS_LED_PORT.OUTCLR = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;
    STATUS_LED_PORT.DIRSET = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;

    init_uart(&UART_GROUNDSTATION_PORT, UART_GROUNDSTATION_RX_bm,
              UART_GROUNDSTATION_TX_bm, &UART_GROUNDSTATION);
    init_uart(&UART_EXPERIMENT_PORT, UART_EXPERIMENT_RX_bm,
              UART_EXPERIMENT_TX_bm, &UART_EXPERIMENT);

    const struct task_recv_uart_data initializer = { .data = 0, .updated = 0,
        .inactivity = LED_DURATION, .led_toggle_interval = LED_DURATION / 2,
        .biterr_remaining_bytes = 0, .biterr_flip_index = 0 };
    task_recv_from_gnd = initializer;
    task_recv_from_exp = initializer;
}

static void
recv_uart(USART_t *uart, struct task_recv_uart_data *data)
{
    --data->inactivity;
    uint8_t err_flags = USART_FERR_bm | USART_BUFOVF_bm | USART_PERR_bm;
    if (uart->STATUS & err_flags) {
        /* clear error flags if set */
        uint8_t ignored __attribute__((unused));
        ignored = uart->DATA;
        ignored = uart->DATA;
        return;
    }

    if (!(uart->STATUS & USART_RXCIF_bm))
        return;

    data->data = uart->DATA;
    data->updated = 1;
    data->inactivity = LED_DURATION;
    --data->led_toggle_interval;
}

static void
ignore_recv(struct task_recv_uart_data *data)
{
    data->updated = 0;
    data->inactivity = 0;
}

static void
recv(void)
{
    static uint16_t duration = 0;
    static uint16_t interval = 0;

    recv_uart(&UART_EXPERIMENT, &task_recv_from_exp);

    struct task_recv_uart_data from_gnd_copy = task_recv_from_gnd;
    recv_uart(&UART_GROUNDSTATION, &from_gnd_copy);
    if (task_ctrl_signals.lo_active) {
        /* after LO we just ignore all incoming traffic from groundstation */
        ignore_recv(&task_recv_from_gnd);
    } else {
        task_recv_from_gnd = from_gnd_copy;
    }

    if (task_ctrl_signals.error_inhibit)
        interval = 0;
    --interval;
    if (!interval) {
        interval = 1;
        --duration;
        if (rand() & 1) {
            ignore_recv(&task_recv_from_gnd);
            ignore_recv(&task_recv_from_exp);
        }
    }


    const uint8_t upd = task_sample_adc_inputs_blocking_generator.force_update;
    task_sample_adc_inputs_blocking_generator.force_update = 0;
    if (!duration || upd) {
        duration = task_sample_adc_inputs_blocking_generator.duration;
        interval = task_sample_adc_inputs_blocking_generator.interval;
    }
}

static void
update_led(struct task_recv_uart_data *data, uint8_t ledmask)
{
    if (0 == data->inactivity) {
        /* we have not seen any activity for some time, timeout */
        data->inactivity = LED_DURATION;
        STATUS_LED_PORT.OUTCLR = ledmask;
    } else {
        if (0 == data->led_toggle_interval) {
            /* a normal period of activity passed */
            data->led_toggle_interval = LED_DURATION / 2;
            STATUS_LED_PORT.OUTTGL = ledmask;
        }
    }
}

static void
send_uart(USART_t *uart, struct task_recv_uart_data *data)
{
    if (!data->updated)
        return; /* nothing to send */

    if (!(uart->STATUS & USART_DREIF_bm))
        return; /* USART busy, cannot send now, retry later */

    uart->DATA = data->data;
    data->updated = 0;
}

static void
send(void)
{
    update_led(&task_recv_from_exp, STATUS_LED_DOWNLINK_bm);
    update_led(&task_recv_from_gnd, STATUS_LED_UPLINK_bm);

    send_uart(&UART_GROUNDSTATION, &task_recv_from_exp);
    send_uart(&UART_EXPERIMENT, &task_recv_from_gnd);
}
