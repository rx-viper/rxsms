/*
 *   This file is part of RXSMS.
 *   Copyright 2015  Nicolas Benes
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
#include "task_error.h"
#include "task_sendrecv.h"
#include "task_ctrl.h"
#include "task_adc.h"

/*
 * Ressources used _once_ by this task:
 * (none)
 *
 * Ressources _continously_ used by this task:
 * (none)
 */

static void init(void);
static void run(void);

const struct task task_error = { .init = &init, .run = &run };

static struct
{
    uint32_t begin;
    uint32_t end;
    uint32_t reload;
} drop_error;

static void
init(void)
{
    drop_error.begin = 0;
    drop_error.end = 0;
    drop_error.reload = 0;
}

/// flip a single bit in data stream
static void
flip_bit(struct task_recv_uart_data *const data)
{
    if (!data->updated)
        return;                 /* no pending data */

    if (task_ctrl_signals.error_inhibit) {
        data->biterr_remaining_bytes = 0;
        return;                 /* global error inhibit, no errors should be generated */
    }

    if (0 == data->biterr_remaining_bytes)
        return;                 /* specifically bit flip errors disabled */

    --data->biterr_remaining_bytes;

    int32_t flip = data->biterr_flip_index;
    if (flip >= 8) {            /* bit to flip outside current byte */
        flip -= 8;
    } else if (flip >= 0) {     /* ok, flip bit */
        data->data ^= 1 << ((uint8_t) flip);
        flip = -1;              /* mark as bit already flipped */
    }
    data->biterr_flip_index = flip;
}

static void
drop_communication(struct task_recv_uart_data *a,
                   struct task_recv_uart_data *b)
{
    if (drop_error.reload)
        --drop_error.reload;
    if (drop_error.begin)
        --drop_error.begin;
    if (!drop_error.end)
        return; /* dropping is over */
    --drop_error.end;
    if (drop_error.begin)
        return; /* dropping did not yet start */
    /* now really drop */
    a->updated = 0;
    b->updated = 0;
}

static void
reload_settings(const uint8_t global_update)
{
    /*
       Load new stream length in bytes and the bit to flip if the bit error
       rate has been changed (forced update) or the old run for bit errors has
       been completed (regular update).
     */
    uint8_t biterr_updated = task_adc_biterror_generator.force_update
                           | global_update;
    if (0 == task_recv_from_exp.biterr_remaining_bytes || biterr_updated) {
        task_recv_from_exp.biterr_remaining_bytes =
            task_adc_biterror_generator.stream_len_bytes;
        task_recv_from_exp.biterr_flip_index =
            task_adc_biterror_generator.from_exp_flip;
    }
    if (0 == task_recv_from_gnd.biterr_remaining_bytes || biterr_updated) {
        task_recv_from_gnd.biterr_remaining_bytes =
            task_adc_biterror_generator.stream_len_bytes;
        task_recv_from_gnd.biterr_flip_index =
            task_adc_biterror_generator.from_gnd_flip;
    }
    task_adc_biterror_generator.force_update = 0;

    /*
       Load new point in time when to start dropping and load the duration for
       the communication outage.
     */
    uint8_t drop_updated = task_adc_drop_generator.force_update
                         | global_update;
    if ((0 == drop_error.end && 0 == drop_error.reload) || drop_updated) {
        drop_error.reload = task_adc_drop_generator.interval;
        if (!drop_error.reload) {
            /* communication drop disabled */
            drop_error.begin = 0;
            drop_error.end = 0;
        } else {
            drop_error.begin = task_adc_drop_generator.start_of_drop;
            drop_error.end = drop_error.begin
                           + task_adc_drop_generator.drop_duration + 1;
        }
    }
    task_adc_drop_generator.force_update = 0;
}

static void
run(void)
{
    static uint8_t global_update = 0;
    if (task_ctrl_signals.error_inhibit) {
        global_update = 1;
        return;
    }

    reload_settings(global_update);
    global_update = 0;

    flip_bit(&task_recv_from_gnd);
    flip_bit(&task_recv_from_exp);

    drop_communication(&task_recv_from_gnd, &task_recv_from_exp);
}
