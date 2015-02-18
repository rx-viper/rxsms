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

struct {
    int16_t poti_bit_error_rate;
    int16_t poti_blocking_rate;
    int16_t poti_blocking_duration;
    int16_t current_sense;
    int16_t tempsense[4];
} task_adc_raw;

struct {
    uint32_t stream_len_bytes;
    int32_t from_exp_flip;
    int32_t from_gnd_flip;
    uint8_t force_update:1;
} task_adc_biterror_generator;

// FIXME rename *blocking_* with more accurate *drop_*
struct {
    uint16_t duration;          ///< number of scheduling cycles to block transmission
    uint16_t interval;          ///< interval in scheduling cycles to throw coin
    uint8_t force_update:1;
} task_adc_blocking_generator;

#endif                          /* TASK_ADC_H */
