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

#ifndef BGTASK_SAMPLE_ADC_INPUTS_H
#define BGTASK_SAMPLE_ADC_INPUTS_H

#include <inttypes.h>
#include "task.h" /* for struct bgtask */

// TODO maybe set as const?
const struct bgtask bgtask_sample_adc_inputs;

struct
{
    uint16_t poti_bit_error_rate;
    uint16_t poti_blocking_rate;
    uint16_t poti_blocking_duration;
    uint16_t current_sense;
} adc_sense_buffer;

#endif /* BGTASK_SAMPLE_ADC_INPUTS_H */
