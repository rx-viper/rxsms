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
#include "task_error.h"
#include "task_error_tables.h"
#include "task_sendrecv.h"
#include "task_ctrl.h"
#include "task_adc.h"
#include "task_rng.h"

// Ressources used _once_ by this task:
// (none)
//
// Ressources _continously_ used by this task:
// (none)

static void init(void);
static void run(void);

const struct task task_error = { .init = &init, .run = &run };

static struct
{
    __uint24 begin;
    __uint24 end;
    __uint24 reload;
} drop_error;

static struct biterr_status
{
    __uint24 remaining_bytes;
    __uint24 flip_index;
} biterr_from_gnd, biterr_from_exp;

static void
init(void)
{
}

// get 3 B of random data
static __uint24
get_random(void)
{
    union {
        __uint24 retval;
        uint8_t ui8[sizeof(__uint24)];
    } val;
    uint8_t rng_next = task_rng_random.next;
    task_rng_random.next = rng_next + sizeof(val);
    for (uint8_t i = 0; i < sizeof(val); ++i)
        val.ui8[i] = task_rng_random.ui8[rng_next + i];
    return val.retval;
}

static void
reload_biterror(const uint8_t global_update)
{
    // Load new stream length in bytes and the bit to flip if the bit error
    // rate has been changed (forced update) or the old run for bit errors has
    // been completed (regular update).
    uint8_t bin = task_adc_generator.biterror_rate_bin;
    __uint24 stream_len = bit_error_rate_bin_map[bin];
    __uint24 flipmask = (stream_len << 3) - 1;

    uint8_t update = task_adc_generator.biterror_force_update || global_update;
    if (!biterr_from_exp.remaining_bytes || update) {
        biterr_from_exp.remaining_bytes = stream_len;
        biterr_from_exp.flip_index = flipmask & get_random();
    }
    if (!biterr_from_gnd.remaining_bytes || update) {
        biterr_from_gnd.remaining_bytes = stream_len;
        biterr_from_gnd.flip_index = flipmask & get_random();
    }
    task_adc_generator.biterror_force_update = 0;
}

/// Load new point in time when to start dropping and load the duration for
/// the communication outage.
static void
reload_dropout(const uint8_t global_update)
{
    // we can leave directly iff we do not have a "forced update" condition
    // and iff we do not have a "regular" update (which happens when both
    // end and reload are 0)
    if (!global_update && !task_adc_generator.dropout_force_update
        && (drop_error.end || drop_error.reload))
        return;

    uint8_t bin = task_adc_generator.dropout_rate_bin;
    __uint24 interval = drop_rate_bin_map[bin];

    drop_error.reload = interval;
    if (0 == interval) {
        // communication drop disabled
        drop_error.begin = 0;
        drop_error.end = 0;
    } else {
        drop_error.begin = (interval - 1) & get_random();
        bin = task_adc_generator.dropout_duration_bin;
        drop_error.end = drop_error.begin + drop_duration_bin_map[bin] + 1;
    }
    task_adc_generator.dropout_force_update = 0;
}

/// flip a single bit in data stream
static void
flip_bit(struct task_recv_uart_data *const data,
         struct biterr_status *const biterr)
{
    if (!data->updated)
        return;                 // no pending data

    if (0 == biterr->remaining_bytes)
        return;                 // specifically bit flip errors disabled

    --biterr->remaining_bytes;

    __int24 flip = biterr->flip_index;
    if (flip >= 8) {            // bit to flip outside current byte
        flip -= 8;
    } else if (flip >= 0) {     // ok, flip bit
        data->data ^= 1 << ((uint8_t) flip);
        flip = -1;              // mark as bit already flipped
    }
    biterr->flip_index = flip;
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
        return; // dropping is over
    --drop_error.end;
    if (drop_error.begin)
        return; // dropping did not yet start
    // now really drop
    a->updated = 0;
    b->updated = 0;
}

static void
run(void)
{
    static uint8_t global_update = 0;
    if (task_ctrl_signals.error_inhibit) {
        global_update = 1;
        return;
    }

    reload_biterror(global_update);
    reload_dropout(global_update);
    global_update = 0;

    flip_bit(&task_recv_from_gnd, &biterr_from_gnd);
    flip_bit(&task_recv_from_exp, &biterr_from_exp);

    drop_communication(&task_recv_from_gnd, &task_recv_from_exp);
}
