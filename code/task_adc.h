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
    __uint24 stream_len_bytes;
    __int24 from_exp_flip;
    __int24 from_gnd_flip;
    uint8_t force_update:1;
} task_adc_biterror_generator;

struct {
    __uint24 interval;
    __uint24 start_of_drop;
    __uint24 drop_duration;
    uint8_t force_update:1;
} task_adc_drop_generator;

#endif                          /* TASK_ADC_H */
