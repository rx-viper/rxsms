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
#include "task_ctrl.h"
#include "task_buttons.h"
#include "task_adc.h"

#define STATUS_LED_PORT         PORTA
#define STATUS_LED_LO_bm        PIN7_bm
#define STATUS_LED_ERRINH_bm    PIN6_bm

#define RXSM_LO_PORT        PORTD
#define RXSM_LO_bm          PIN0_bm
#define RXSM_SOE_PORT       PORTD
#define RXSM_SOE_bm         PIN1_bm
#define RXSM_SODS_PORT      PORTC
#define RXSM_SODS_bm        PIN5_bm

#define RXSM_EXPPWR_PORT    PORTC
#define RXSM_EXPPWR_bm      PIN4_bm

static void init(void);
static void run(void);
const struct task task_ctrl = { .init = &init, .run = &run };

// TODO actually read the LO pin's value back

static void
apply_state(void)
{
    if (task_ctrl_signals.lo_active) {
        STATUS_LED_PORT.OUTSET = STATUS_LED_LO_bm;
        RXSM_LO_PORT.OUTCLR = RXSM_LO_bm;
    } else {
        STATUS_LED_PORT.OUTCLR = STATUS_LED_LO_bm;
        RXSM_LO_PORT.OUTSET = RXSM_LO_bm;
    }

    if (task_ctrl_signals.soe_active)
        RXSM_SOE_PORT.OUTCLR = RXSM_SOE_bm;
    else
        RXSM_SOE_PORT.OUTSET = RXSM_SOE_bm;

    if (task_ctrl_signals.sods_active)
        RXSM_SODS_PORT.OUTCLR = RXSM_SODS_bm;
    else
        RXSM_SODS_PORT.OUTSET = RXSM_SODS_bm;

    if (task_ctrl_signals.error_inhibit ||
        (!task_adc_biterror_generator.stream_len_bytes &&
         !task_adc_drop_generator.interval))
        STATUS_LED_PORT.OUTSET = STATUS_LED_ERRINH_bm;
    else
        STATUS_LED_PORT.OUTCLR = STATUS_LED_ERRINH_bm;

    if (task_ctrl_signals.pwr_on)
        RXSM_EXPPWR_PORT.OUTSET = RXSM_EXPPWR_bm;
    else
        RXSM_EXPPWR_PORT.OUTCLR = RXSM_EXPPWR_bm;
}

static void
init(void)
{
    /* by default, the error inhibit is ON, all others OFF/INACTIVE */
    task_ctrl_signals.lo_active = 0;
    task_ctrl_signals.soe_active = 0;
    task_ctrl_signals.sods_active = 0;
    task_ctrl_signals.pwr_on = 0;
    task_ctrl_signals.error_inhibit = 1;

    /* first set the outputs, then the directions.
     * this way, we can ensure that the outputs are undriven/passive but the
     * OUTPUT register is already set correctly. Then the pins are switched to
     * drive loads, and already have the correct output value. */
    apply_state();
    STATUS_LED_PORT.DIRSET = STATUS_LED_LO_bm | STATUS_LED_ERRINH_bm;
#define PORTINIT(port)  RXSM_##port##_PORT.DIRSET = RXSM_##port##_bm
    PORTINIT(LO);
    PORTINIT(SOE);
    PORTINIT(SODS);
    PORTINIT(EXPPWR);
#undef PORTINIT
}

static void
run(void)
{
    /* TODO: at a later point in time, we don't want to use
     * the button requests directly. instead task_sync should
     * send the button requests to others and should receive
     * requests as well, and finally when all simulators are
     * synchronized, we use the requests from task_sync to
     * synchronously switch the signals for all simulators.
     */
    /* update our state according task_buttons */
    if (task_buttons_toggle_request.lo)
        task_ctrl_signals.lo_active ^= 1;
    if (task_buttons_toggle_request.soe)
        task_ctrl_signals.soe_active ^= 1;
    if (task_buttons_toggle_request.sods)
        task_ctrl_signals.sods_active ^= 1;
    if (task_buttons_toggle_request.errinh)
        task_ctrl_signals.error_inhibit ^= 1;
    if (task_buttons_toggle_request.pwr)
        task_ctrl_signals.pwr_on ^= 1;

    apply_state();
}
