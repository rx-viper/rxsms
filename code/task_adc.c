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
    uint8_t addr0, addr1;

    addr0 = (uint8_t) (uint16_t) & PRODSIGNATURES_ADCACAL0;
    addr1 = (uint8_t) (uint16_t) & PRODSIGNATURES_ADCACAL1;
    *adca_calibration = (prodsigrow_read_byte(addr1) << 8)
        | prodsigrow_read_byte(addr0);

    addr0 = (uint8_t) (uint16_t) & PRODSIGNATURES_TEMPSENSE0;
    addr1 = (uint8_t) (uint16_t) & PRODSIGNATURES_TEMPSENSE1;
    *tempsense_calibration = (prodsigrow_read_byte(addr1) << 8)
        | prodsigrow_read_byte(addr0);
}

/// Init the ADC to scan the potis and optional current sense input.
static void
init(void)
{
    for (uint8_t i = 0; i < sizeof(task_adc_raw); ++i)
        ((uint8_t*) &task_adc_raw)[i] = 0;
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

/**
 *  Partitions a value range 0..2047 into 16 regions ("bins") and returns
 *  a bitmask with all bits cleared (bin 0) or the 15th bit set (bin 1), the
 *  14th bit set (bit 2) and so on.
 *
 *  bin     | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1    0
 *  bit set |  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 none
 *
 *  It allows for generation of a (mask - 1) value that can be and-ed with a
 *  uniformly distributed random number with binary range, i.e. 0..2^x-1, to
 *  create a new uniform distributed random number of range 0..2^16-1.
 */
static uint16_t partition_range(int16_t adc) __ATTR_PURE__;
static uint16_t
partition_range(int16_t adc)
{
    if (adc < 0)
        adc = 0;
    else if (adc >= 2048)
        adc = 2047;

    uint8_t bin = adc / (2048 / 16);
    if (0 == bin)
        return 0;

    uint8_t set_bit = 15 - bin;
    if (set_bit > 7)
        return (1 << (set_bit - 8)) << 8;
    return 1 << set_bit;
}

static void
update_biterror_generators(void)
{
    /* get a mask for the probability with either p=0, or p = 1 / 2^N
       then decrease the probability by 2^(-4) to get the following values:

       bin          | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
       2^N bytes    |  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18  -
       2^(N+3) bits |  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21  -

       Therefore, the bit error rate with the poti in bin=13 is
           p = (1 bit error) / (2^9 bits)
     */
    uint32_t stream_len = partition_range(task_adc_raw.e.poti_bit_error_rate);
    stream_len <<= 4;

    if (task_adc_biterror_generator.stream_len_bytes != stream_len)
        task_adc_biterror_generator.force_update = 1;
    task_adc_biterror_generator.stream_len_bytes = stream_len;
    task_adc_biterror_generator.from_exp_flip = generate_bit_flip(stream_len);
    task_adc_biterror_generator.from_gnd_flip = generate_bit_flip(stream_len);
}

static void
update_blocking_generators(void)
{
    int16_t duration_poti = task_adc_raw.e.poti_blocking_duration;
    int16_t frequency_poti = task_adc_raw.e.poti_blocking_rate;

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

        task_adc_raw.e.tempsense[s - BITERR] = ADCA.CH1RES;

        const int16_t adc_value = ADCA.CH0RES;
        /* For the potentiometer inputs:
             Update task_adc_raw only iff the new value exceeds a hysteresis.
             This hysteresis prevents updates of the generator probabilities at
             the boundary of a setting and cancels out noise.
           For the current sensor:
             Ignore the hysteresis since we want the exact sensor readings.  */
        const int8_t MIN_DIFF = 8;
        int16_t diff = task_adc_raw.i16[s - BITERR] - adc_value;
        if (diff < -MIN_DIFF || diff > MIN_DIFF || CURRSENSE == s)
            task_adc_raw.i16[s - BITERR] = adc_value;

        ++state;
        if (state > CURRSENSE) {
            ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc
                | ADC_CH_MUXNEG_GND_MODE3_gc;
            ADCA.CH0.SCAN = 3;

            update_biterror_generators();
            update_blocking_generators();

            state = BITERR;
        }

        ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
        ADCA.CTRLA |= ADC_CH1START_bm | ADC_CH0START_bm;
    } else {
        /* this should not happen */
        init();
        state = INIT;
    }
}
