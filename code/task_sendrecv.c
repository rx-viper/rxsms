/*
 *   This file is part of RXSMS.
 *   Copyright 2014  Nicolas Benes
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
#include "task_sample_adc_inputs.h"
#include "task_ctrl.h"

#define STATUS_LED_PORT         PORTD
#define STATUS_LED_UPLINK_bm    PIN4_bm
#define STATUS_LED_DOWNLINK_bm  PIN5_bm

#define UART_GROUNDSTATION_PORT     PORTD
#define UART_GROUNDSTATION_RX_bm    PIN6_bm
#define UART_GROUNDSTATION_TX_bm    PIN7_bm
#define UART_EXPERIMENT_PORT        PORTD
#define UART_EXPERIMENT_RX_bm       PIN2_bm
#define UART_EXPERIMENT_TX_bm       PIN3_bm

#define UART_GROUNDSTATION  USARTD1
#define UART_EXPERIMENT     USARTD0
#define UART_BSEL       (12)
#define UART_BSCALE     (2)

#define LED_DURATION    500

static void init(void);
static void recv(void);
static void send(void);
/* common init routine for both as they access the same hardware */
const struct task task_recv = { .init = &init, .run = &recv };
const struct task task_send = { .init = &init, .run = &send };

static struct uart_data
{
    uint8_t data;
    uint8_t updated : 1;
    uint16_t inactivity;
    uint16_t led_toggle_interval;
    uint32_t biterr_total_count;
    uint32_t biterr_flip_index;
} from_gnd, from_exp;

static void
init(void)
{
    static uint8_t is_initialized = 0;
    if (is_initialized)
        return;     /* leave if init() is called multiple times */
    is_initialized = 1;

    STATUS_LED_PORT.OUTCLR = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;
    STATUS_LED_PORT.DIRSET = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;

    UART_GROUNDSTATION_PORT.OUTSET = UART_GROUNDSTATION_TX_bm;
    UART_GROUNDSTATION_PORT.DIRSET = UART_GROUNDSTATION_TX_bm;
    UART_GROUNDSTATION_PORT.DIRCLR = UART_GROUNDSTATION_RX_bm;

    UART_EXPERIMENT_PORT.OUTSET = UART_EXPERIMENT_TX_bm;
    UART_EXPERIMENT_PORT.DIRSET = UART_EXPERIMENT_TX_bm;
    UART_EXPERIMENT_PORT.DIRCLR = UART_EXPERIMENT_RX_bm;

    UART_GROUNDSTATION.CTRLA = 0;
    UART_GROUNDSTATION.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    UART_GROUNDSTATION.CTRLC = USART_CMODE_ASYNCHRONOUS_gc
        | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
    UART_GROUNDSTATION.BAUDCTRLA = (uint8_t) UART_BSEL;
    UART_GROUNDSTATION.BAUDCTRLB = (UART_BSCALE << 4) | (UART_BSEL >> 8);

    UART_EXPERIMENT.CTRLA = 0;
    UART_EXPERIMENT.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    UART_EXPERIMENT.CTRLC = USART_CMODE_ASYNCHRONOUS_gc
        | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
    UART_EXPERIMENT.BAUDCTRLA = (uint8_t) UART_BSEL;
    UART_EXPERIMENT.BAUDCTRLB = (UART_BSCALE << 4) | (UART_BSEL >> 8);

    const struct uart_data initializer = { .data = 0, .updated = 0,
        .inactivity = LED_DURATION, .led_toggle_interval = LED_DURATION / 2,
        .biterr_total_count = 0, .biterr_flip_index = 0 };
    from_gnd = initializer;
    from_exp = initializer;
}

static void
recv_uart(USART_t *uart, struct uart_data *data)
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
update_led(struct uart_data *data, uint8_t ledmask)
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
ignore_recv(struct uart_data *data)
{
    data->updated = 0;
    data->inactivity = 0;
}

static void
recv(void)
{
    static uint16_t duration = 0;
    static uint16_t interval = 0;

    recv_uart(&UART_EXPERIMENT, &from_exp);

    struct uart_data from_gnd_copy = from_gnd;
    recv_uart(&UART_GROUNDSTATION, &from_gnd_copy);
    if (task_ctrl_signals.lo_active) {
        /* after LO we just ignore all incoming traffic from groundstation */
        ignore_recv(&from_gnd);
    } else {
        from_gnd = from_gnd_copy;
    }

    if (task_ctrl_signals.error_inhibit)
        interval = 0;
    --interval;
    if (!interval) {
        interval = 1;
        --duration;
        if (rand() & 1) {
            ignore_recv(&from_gnd);
            ignore_recv(&from_exp);
        }
    }

    update_led(&from_exp, STATUS_LED_DOWNLINK_bm);
    update_led(&from_gnd, STATUS_LED_UPLINK_bm);

    const uint8_t upd = task_sample_adc_inputs_blocking_generator.force_update;
    task_sample_adc_inputs_blocking_generator.force_update = 0;
    if (!duration || upd) {
        duration = task_sample_adc_inputs_blocking_generator.duration;
        interval = task_sample_adc_inputs_blocking_generator.interval;
    }
}

static void
send_uart(USART_t *uart, struct uart_data *data)
{
    if (!data->updated)
        return; /* nothing to send */

    if (!(uart->STATUS & USART_DREIF_bm))
        return; /* cannot send, USART busy, drop byte */

    /* apply error pattern */
    uint8_t to_send = data->data;
    if (task_ctrl_signals.error_inhibit) {
        data->biterr_total_count = 0;
    } else {
        uint32_t count = data->biterr_total_count;

        if (count) {
            count -= 8;
            data->biterr_total_count = count;

            int32_t flip = data->biterr_flip_index;
            if (flip >= 8) {            /* bit to flip outside current byte */
                flip -= 8;
            } else if (flip >= 0) {     /* ok, flip bit */
                to_send ^= (uint8_t) flip;
                flip = -1;              /* mark as bit already flipped */
            }
            data->biterr_flip_index = flip;
        }
    }
    uart->DATA = to_send;
    data->updated = 0;
}

static void
send(void)
{
    send_uart(&UART_GROUNDSTATION, &from_exp);
    send_uart(&UART_EXPERIMENT, &from_gnd);

    /* load new bit error patterns/settings */
    uint8_t upd = task_sample_adc_inputs_biterror_generator.force_update;
    task_sample_adc_inputs_biterror_generator.force_update = 0;
    if (0 == from_exp.biterr_total_count || upd) {
        from_exp.biterr_total_count =
            task_sample_adc_inputs_biterror_generator.total_bit_count;
        from_exp.biterr_flip_index =
            task_sample_adc_inputs_biterror_generator.from_exp_flip;
    }
    if (0 == from_gnd.biterr_total_count || upd) {
        from_gnd.biterr_total_count =
            task_sample_adc_inputs_biterror_generator.total_bit_count;
        from_gnd.biterr_flip_index =
            task_sample_adc_inputs_biterror_generator.from_gnd_flip;
    }
}
