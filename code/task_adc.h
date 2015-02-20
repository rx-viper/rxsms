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

#ifndef TASK_ADC_H
#define TASK_ADC_H

#include <inttypes.h>
#include "task.h"

const struct task task_adc;

/*
   note:  when changing the layout of this structure, concult with all other
          uses and verify correct behaviour.
 */
union {
    int16_t i16[8];
    struct {
        int16_t poti_bit_error_rate;
        int16_t poti_blocking_rate;
        int16_t poti_blocking_duration;
        int16_t current_sense;
        int16_t tempsense[4];
    } e;
} task_adc_raw;

struct {
    uint32_t stream_len_bytes;
    int32_t from_exp_flip;
    int32_t from_gnd_flip;
    uint8_t force_update:1;
} task_adc_biterror_generator;

// FIXME rename *blocking_* with more accurate *drop_*
struct {
    uint32_t interval;
    uint32_t start_of_drop;
    uint16_t drop_duration;
    uint8_t force_update:1;
} task_adc_blocking_generator;

#endif                          /* TASK_ADC_H */
