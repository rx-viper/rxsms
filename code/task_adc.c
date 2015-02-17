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
#include <stdlib.h>
#include "task_adc.h"
#include "util.h"

// TODO configure analog input pins with INTPUT_DISABLE
//      in the respective PINnCTRL register of the IO port

/*
 * Ressources used _once_ by this task:
 * (none)
 *
 * Ressources _continously_ used by this task:
 * ADCA
 * ADCA.CH0..CH1
 * ADC TEMPREF
 * PORTA PIN 0..4
 */

// FIXME - remove when everyone uses avr-libc >= 1.8.1
// These symbols are not defined in io.h or have other names than in pre 1.8.1
#if __AVR_LIBC_VERSION__ < 10801UL
#define ADC_CURRLIMIT_HIGH_gc ADC_CURRLIMIT_LARGE_gc
#define ADC_CH_MUXNEG_GND_MODE3_gc 0x05
#endif

#define ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc    ADC_CH_MUXPOS_PIN1_gc
#define ADC_CH_MUXPOS_POTI_BLOCKING_RATE_gc     ADC_CH_MUXPOS_PIN2_gc
#define ADC_CH_MUXPOS_POTI_BLOCKING_DURATION_gc ADC_CH_MUXPOS_PIN3_gc
#define ADC_CH_MUXPOS_CURRENT_SENSE_gc          ADC_CH_MUXPOS_PIN4_gc

#define ADC_PORT                        PORTA
#define ADC_REF_bm                      PIN0_bm
#define ADC_POTI_BIT_ERROR_RATE_bm      PIN1_bm
#define ADC_POTI_BLOCKING_RATE_bm       PIN2_bm
#define ADC_POTI_BLOCKING_DURATION_bm   PIN3_bm
#define ADC_CURRENT_SENSE_bm            PIN4_bm

static void init(void);
static void run(void);
const struct task task_adc = { .init = &init, .run = &run };

/// Reads the ADC calibration value and TEMPSENSE calibration value.
static void
production_signature_row_read_calibration(uint16_t * adca_calibration,
                                          uint16_t * tempsense_calibration)
{
    uint8_t adcacal0_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_ADCACAL0;
    uint8_t adcacal1_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_ADCACAL1;
    uint8_t tempsense0_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_TEMPSENSE0;
    uint8_t tempsense1_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_TEMPSENSE1;

    *adca_calibration = (prodsigrow_read_byte(adcacal1_addr) << 8)
        | prodsigrow_read_byte(adcacal0_addr);
    *tempsense_calibration = (prodsigrow_read_byte(tempsense1_addr) << 8)
        | prodsigrow_read_byte(tempsense0_addr);
}

/// Init the ADC to scan the potis and optional current sense input.
static void
init(void)
{
    adc_sense_buffer.poti_bit_error_rate = 0;
    adc_sense_buffer.poti_blocking_rate = 0;
    adc_sense_buffer.poti_blocking_duration = 0;
    adc_sense_buffer.current_sense = 0;
    task_adc_biterror_generator.stream_len_bytes = 0;
    task_adc_biterror_generator.from_exp_flip = 0;
    task_adc_biterror_generator.from_gnd_flip = 0;
    task_adc_biterror_generator.force_update = 0;
    task_adc_blocking_generator.duration = 0;
    task_adc_blocking_generator.interval = 0;
    task_adc_blocking_generator.force_update = 0;

    /* configure pins for input */
    const uint8_t pins = ADC_REF_bm | ADC_POTI_BIT_ERROR_RATE_bm
        | ADC_POTI_BLOCKING_RATE_bm | ADC_POTI_BLOCKING_DURATION_bm
        | ADC_CURRENT_SENSE_bm;
    ADC_PORT.DIRCLR = pins;
    ADC_PORT.OUTCLR = pins;

    uint16_t adca_calibration, tempsense_calibration;
    production_signature_row_read_calibration(&adca_calibration,
                                              &tempsense_calibration);

    /* init ADCA CH0 and CH1 */
    ADCA.CTRLA = 0;
    ADCA.CTRLB =
        ADC_CURRLIMIT_HIGH_gc | ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc | ADC_TEMPREF_bm;
    ADCA.EVCTRL = 0;
    ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;
    ADCA.INTFLAGS =
        ADC_CH3IF_bm | ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;
    ADCA.CAL = adca_calibration;
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH0.MUXCTRL =
        ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc | ADC_CH_MUXNEG_GND_MODE3_gc;
    ADCA.CH0.INTCTRL = 0;
    ADCA.CH0.SCAN = 3;

    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXINT_TEMP_gc | ADC_CH_MUXNEG_GND_MODE3_gc;
    ADCA.CH1.INTCTRL = 0;
    ADCA.CTRLA |=
        ADC_CH1START_bm | ADC_CH0START_bm | ADC_FLUSH_bm | ADC_ENABLE_bm;
}

// TODO check if uint32_t is correct. at other places I've seen int32_t...
static uint32_t
generate_bit_flip(const uint32_t stream_len_bytes)
{
    union {
        uint32_t flip;
        struct {
            uint8_t ui8[4];
        } e;
    } random_number;

    for (uint8_t i = 0; i < sizeof(random_number); ++i)
        random_number.e.ui8[i] = (uint8_t) rand();
    /* mask the random number with range 0..2^32-1
       to range 0..(stream_len_bytes * 8 - 1) */
    if (stream_len_bytes)
        return random_number.flip & (stream_len_bytes * 8 - 1);
    return 0;
}

static void
update_biterror_generators(void)
{
    int16_t bit_error_rate = adc_sense_buffer.poti_bit_error_rate;
    if (bit_error_rate < 0)
        bit_error_rate = 0;

    /* partition the ADC range 2047..0 into 16 bins
       with respective probabilities 1/N:
       bin        | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
       N=2^x bit  |  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21  -
       N=2^x byte |  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18  -
       except bin 0 with probability 0 (error gen disabled)

       The value range is |0..2047| = 2048 and we want 16 bins out of it.
     */
    uint8_t bin = bit_error_rate / (2048 / 16);
    uint32_t stream_len_bytes;
    if (0 == bin)
        stream_len_bytes = 0;
    else
        stream_len_bytes = ((uint32_t) 1) << (15 - bin + 4);

    if (task_adc_biterror_generator.stream_len_bytes != stream_len_bytes)
        task_adc_biterror_generator.force_update = 1;
    task_adc_biterror_generator.stream_len_bytes = stream_len_bytes;
    task_adc_biterror_generator.from_exp_flip =
        generate_bit_flip(stream_len_bytes);
    task_adc_biterror_generator.from_gnd_flip =
        generate_bit_flip(stream_len_bytes);
}

static void
update_blocking_generators(void)
{
    int16_t duration_poti = adc_sense_buffer.poti_blocking_duration;
    int16_t frequency_poti = adc_sense_buffer.poti_blocking_rate;

    if (duration_poti < 0)
        duration_poti = 0;
    if (frequency_poti < 0)
        frequency_poti = 0;

    /* 32 bins with 2000 sched cycles (=0.5s) each: 0s..15.5s */
    uint16_t duration = 2000 * (duration_poti / 64);
    /* 32 bins with 2000 sched cycles (=0.5s) each: 0s..15.5s */
    uint8_t interval_bin = frequency_poti / 64;
    uint16_t interval;
    if (interval_bin)
        interval = 2000 * interval_bin;
    else
        interval = 0;

    if (task_adc_blocking_generator.duration != duration ||
        task_adc_blocking_generator.interval != interval)
        task_adc_blocking_generator.force_update = 1;
    task_adc_blocking_generator.duration = duration;
    task_adc_blocking_generator.interval = interval;
}

static void
run(void)
{
    static enum { INIT, BITERR, BLOCKRATE, BLOCKDUR, CURRSENSE } state = INIT;
    const uint8_t s = state;
    if (INIT == s) {
        /* throw away the first measurement, as it might be wrong */
        ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
        ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc
            | ADC_CH_MUXNEG_GND_MODE3_gc;
        ADCA.CH0.SCAN = 3;
        ADCA.CTRLA |= ADC_CH1START_bm | ADC_CH0START_bm;
        state = BITERR;
    } else if (BITERR <= s && CURRSENSE >= s) {
        if (!(ADCA.INTFLAGS & ADC_CH0IF_bm)
            || !(ADCA.INTFLAGS & ADC_CH1IF_bm))
            return;

        int16_t adc_value = ADCA.CH0RES;

        /* update adc_sense_buffer only iff the new value has
           a significant difference
           this hysteresis prevents updates of the generator probabilities at
           the boundary of a setting
         */
        const int8_t MIN_DIFF = 50;
        if (BITERR == s) {
            int16_t diff =
                adc_sense_buffer.poti_bit_error_rate - adc_value;
            if (diff < -MIN_DIFF || diff > MIN_DIFF)
                adc_sense_buffer.poti_bit_error_rate = adc_value;
            // TODO where to store the tempsense result?
            state = BLOCKRATE;
        } else if (BLOCKRATE == s) {
            int16_t diff = adc_sense_buffer.poti_blocking_rate - adc_value;
            if (diff < -MIN_DIFF || diff > MIN_DIFF)
                adc_sense_buffer.poti_blocking_rate = adc_value;
            // TODO where to store the tempsense result?
            state = BLOCKDUR;
        } else if (BLOCKDUR == s) {
            int16_t diff =
                adc_sense_buffer.poti_blocking_duration - adc_value;
            if (diff < -MIN_DIFF || diff > MIN_DIFF)
                adc_sense_buffer.poti_blocking_duration = adc_value;
            // TODO where to store the tempsense result?
            state = CURRSENSE;
        } else if (CURRSENSE == s) {
            adc_sense_buffer.current_sense = adc_value;
            // TODO where to store the tempsense result?
            ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc
                | ADC_CH_MUXNEG_GND_MODE3_gc;
            ADCA.CH0.SCAN = 3;
            state = BITERR;

            update_biterror_generators();
            update_blocking_generators();
        }
        ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
        ADCA.CTRLA |= ADC_CH1START_bm | ADC_CH0START_bm;
    } else {
        /* this should not happen */
        init();
        state = INIT;
    }
}