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

#ifndef TASK_SAMPLE_ADC_INPUTS_H
#define TASK_SAMPLE_ADC_INPUTS_H

#include <inttypes.h>
#include "task.h" /* for struct task */

const struct task task_sample_adc_inputs;

// TODO maybe remove entirely or move to task_$FOO.c file if not needed
struct
{
    int16_t poti_bit_error_rate;
    int16_t poti_blocking_rate;
    int16_t poti_blocking_duration;
    int16_t current_sense;
} adc_sense_buffer;

struct
{
    uint32_t total_bit_count;
    int32_t from_exp_flip;
    int32_t from_gnd_flip;
    uint8_t force_update;
} task_sample_adc_inputs_biterror_generator;

struct
{
    uint16_t duration;  ///< number of scheduling cycles to block transmission
    uint16_t interval; ///< interval in scheduling cycles to throw coin
    uint8_t force_update;
} task_sample_adc_inputs_blocking_generator;

#endif /* TASK_SAMPLE_ADC_INPUTS_H */
