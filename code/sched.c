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


#include <avr/interrupt.h>
#include <stddef.h>     /* for NULL */
#include "sched.h"
#include "task.h"
#include "task_buttons.h"
#include "task_rng.h"
#include "task_adc.h"
#include "task_ctrl.h"
#include "task_sendrecv.h"
#include "task_error.h"
#include "task_debug.h"

#define SCHED_TIMER             TCC0
#define SCHED_TIMER_OVF_vect    TCC0_OVF_vect
#define SCHED_TIMER_PWRUP       PR.PRPC &= ~PR_TC0_bm
#define SCHED_CLKSEL_gc     TC_CLKSEL_DIV8_gc
#define SCHED_CLKSEL_DEC    8
#define SCHED_PERIOD ((SCHED_SLOT_PERIOD_US * 1e-6 * F_CPU / SCHED_CLKSEL_DEC))


/// Scheduling plan.
///
/// Empty/unused slots must be set to NULL.
static const struct task *const scheduling_map[SCHED_SLOT_COUNT] = {
    &task_rng, &task_buttons, &task_adc, &task_recv,
    &task_ctrl, &task_error, &task_send, &task_debug
};

/// The current slot number the scheduler is in.
static uint8_t next_slot;

/// Configure scheduling timer interrupt.
void
sched_init(void)
{
    SCHED_TIMER_PWRUP;
    SCHED_TIMER.PERBUF = SCHED_PERIOD;
    SCHED_TIMER.PER = SCHED_PERIOD;
    SCHED_TIMER.CTRLB = TC_WGMODE_SINGLESLOPE_gc;
    SCHED_TIMER.INTCTRLA =
        TC_OVFINTLVL_LO_gc | TC_OVFINTLVL_MED_gc | TC_OVFINTLVL_HI_gc;
    PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

    for (uint8_t i = 0; i < SCHED_SLOT_COUNT; ++i) {
        const struct task *t = scheduling_map[i];
        if (NULL != t)
            t->init();
    }
}

/// Starts scheduler timer and resets slot counter.
void
sched_start(void)
{
    next_slot = 0;
    SCHED_TIMER.CTRLA = SCHED_CLKSEL_gc;
}

/// Scheduler ISR to load the task for the current slot.
ISR(SCHED_TIMER_OVF_vect)
{
    uint8_t slot = next_slot;
    const struct task *t = scheduling_map[slot];
#if SCHED_SLOT_COUNT % 2 == 0
    next_slot = ((uint8_t) (slot + 1)) % SCHED_SLOT_COUNT;
#else
    if (++slot >= SCHED_SLOT_COUNT)
        slot = 0;
    next_slot = slot;
#endif

    if (NULL != t)
        t->run();
}
