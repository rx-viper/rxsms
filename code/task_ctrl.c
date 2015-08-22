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
#define RXSM_LO_bp          PIN0_bp

#define RXSM_SOE_PORT       PORTD
#define RXSM_SOE_bm         PIN1_bm

#define RXSM_SODS_PORT      PORTC
#define RXSM_SODS_bm        PIN5_bm

#define RXSM_EXPPWR_PORT    PORTC
#define RXSM_EXPPWR_bm      PIN4_bm

static void init(void);
static void run(void);
const struct task task_ctrl = { .init = &init, .run = &run };

static void
update_lo_led(void)
{
    // For multi-channel operation, LO is connected between all simulators.
    // There are four cases for the LO LED:
    //
    //                       | our LO unasserted | our LO asserted
    // ----------------------+-------------------+----------------
    // other's LO unasserted |       LED OFF     |  LED cont. ON
    // other's LO   asserted |    LED flashing   |  LED cont. ON
    //
    // "our LO": the LO line we control (local)
    // "other's LO": the LO line another simulator controls (remote)
    const uint16_t led_period = 5000;
    static uint16_t led_cnt = led_period;

    if (!(RXSM_LO_PORT.OUT & RXSM_LO_bm)) {         // our LO is asserted
        led_cnt = led_period;
        STATUS_LED_PORT.OUTSET = STATUS_LED_LO_bm;  // LED always ON
        return;
    }

    if (!task_ctrl_signals.lo_asserted) {           // LO is unasserted
        led_cnt = led_period;
        STATUS_LED_PORT.OUTCLR = STATUS_LED_LO_bm;  // LED always OFF
        return;
    }

    // LO is asserted by another simulator          -- LED flashing
    if (led_cnt > led_period / 2)
        STATUS_LED_PORT.OUTSET = STATUS_LED_LO_bm;
    else
        STATUS_LED_PORT.OUTCLR = STATUS_LED_LO_bm;
    // decrement with underflow wrap-around
    if (0 == --led_cnt)
        led_cnt = led_period;
}

static void
update_error_inhibit_led(void)
{
    // There are four cases for the Error Inhibit LED:
    //
    //                       |   error inhibit ON  |   error inhibit OFF
    // ----------------------+---------------------+--------------------
    // poti errors disabled  |     LED cont. ON    |   LED flashing
    // poti errors  enabled  |     LED cont. ON    |      LED OFF
    const uint16_t led_period = 5000;
    static uint16_t led_cnt = led_period;

    if (task_ctrl_signals.error_inhibit) {              // error inhibit ON
        led_cnt = led_period;
        STATUS_LED_PORT.OUTSET = STATUS_LED_ERRINH_bm;  // LED always OFF
        return;
    }

    if (task_adc_biterror_generator.stream_len_bytes || // error inhibit OFF
        task_adc_drop_generator.interval) {             // and poti errors ON
        led_cnt = led_period;
        STATUS_LED_PORT.OUTCLR = STATUS_LED_ERRINH_bm;  // LED always OFF
        return;
    }

    // error inhibit is OFF but NO poti errors are set  -- LED flashing
    if (led_cnt > led_period / 2)
        STATUS_LED_PORT.OUTSET = STATUS_LED_ERRINH_bm;
    else
        STATUS_LED_PORT.OUTCLR = STATUS_LED_ERRINH_bm;

    // decrement with underflow wrap-around
    if (0 == --led_cnt)
        led_cnt = led_period;
}

static void
read_signal_states(void)
{
    task_ctrl_signals.lo_asserted = !(RXSM_LO_PORT.IN & RXSM_LO_bm);
    task_ctrl_signals.soe_asserted = !(RXSM_SOE_PORT.IN & RXSM_SOE_bm);
    task_ctrl_signals.sods_asserted = !(RXSM_SODS_PORT.IN & RXSM_SODS_bm);
    task_ctrl_signals.pwr_on = 0 != (RXSM_EXPPWR_PORT.IN & RXSM_EXPPWR_bm);
}

static void
init(void)
{
    // first set the outputs, then the directions.
    // this way, we can ensure that the outputs are undriven/passive but the
    // OUTPUT register is already set correctly. Then the pins are switched to
    // drive loads and already have the correct output value.
#define PINCTRL_CONCAT(num) PIN##num##CTRL
#define PINCTRL(num)        PINCTRL_CONCAT(num)
    RXSM_LO_PORT.PINCTRL(RXSM_LO_bp) = PORT_OPC_WIREDAND_gc;
#undef PINCTRL
#undef PINCTRL_CONCAT

    // by default, the error inhibit is ON, all others OFF/UNASSERTED
#define OUTINIT(port, out)  RXSM_##port##_PORT.OUT##out = RXSM_##port##_bm
    OUTINIT(LO, SET);
    OUTINIT(SOE, SET);
    OUTINIT(SODS, SET);
    OUTINIT(EXPPWR, CLR);
#undef OUTINIT

#define DIRINIT(port)  RXSM_##port##_PORT.DIRSET = RXSM_##port##_bm
    DIRINIT(LO);
    DIRINIT(SOE);
    DIRINIT(SODS);
    DIRINIT(EXPPWR);
#undef DIRINIT

    // update our internal state
    read_signal_states();
    task_ctrl_signals.error_inhibit = 1;

    // update the LEDs
    update_lo_led();
    update_error_inhibit_led();
    STATUS_LED_PORT.DIRSET = STATUS_LED_LO_bm | STATUS_LED_ERRINH_bm;
}

static void
run(void)
{
    // update output signals according to task_buttons
    if (task_buttons_toggle_request.lo)
        RXSM_LO_PORT.OUTTGL = RXSM_LO_bm;
    if (task_buttons_toggle_request.soe)
        RXSM_SOE_PORT.OUTTGL = RXSM_SOE_bm;
    if (task_buttons_toggle_request.sods)
        RXSM_SODS_PORT.OUTTGL = RXSM_SODS_bm;
    if (task_buttons_toggle_request.pwr)
        RXSM_EXPPWR_PORT.OUTTGL = RXSM_EXPPWR_bm;
    if (task_buttons_toggle_request.errinh)
        task_ctrl_signals.error_inhibit ^= 1;

    read_signal_states();

    update_lo_led();
    update_error_inhibit_led();
}
