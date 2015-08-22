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
#include "sched.h"

// TODO configure analog input pins with INTPUT_DISABLE
//      in the respective PINnCTRL register of the IO port

/*
 * Ressources used _once_ by this task:
 * (none)
 *
 * Ressources _continously_ used by this task:
 * ADCA
 * ADCA.CH0
 * PORTA PIN 0..3
 * PORTB PIN 0
 */

// FIXME - remove when everyone uses avr-libc >= 1.8.1
// These symbols are not defined in io.h or have other names than in pre 1.8.1
#define GCC_VERSION (__GNUC__ * 10000 \
		     + __GNUC_MINOR__ * 100 \
		     + __GNUC_PATCHLEVEL__)
#if GCC_VERSION == 40702
#define ADC_CURRLIMIT_HIGH_gc ADC_CURRLIMIT_LARGE_gc
#define ADC_CH_MUXNEG_GND_MODE3_gc 0x05
#endif
#undef GCC_VERSION

#define ADC_REF_PORT        PORTB
#define ADC_REF_bm          PIN0_bm
#define ADC_REFSEL_gc       ADC_REFSEL_AREFB_gc

#define ADC_PWRUP           PR.PRPA &= ~PR_ADC_bm
#define ADC_PORT            PORTA
#define ADC_PORT_PINS_gc    (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)

#define FIRST_CHANNEL_NAME  DROPDUR
#define ADC_CH_MUXPOS_FIRST ADC_CH_MUXPOS_PIN0_gc
#define ADC_CH_MUXPOS_LAST  ADC_CH_MUXPOS_PIN3_gc
#define INPUT_SCAN_COUNT    3

static void init(void);
static void run(void);
const struct task task_adc = { .init = &init, .run = &run };

static struct {
    int16_t poti_bit_error_rate;
    int16_t poti_drop_rate;
    int16_t poti_drop_duration;
    int16_t current_sense;
} adc_raw;

static enum
{
    INIT,
    DROPDUR,
    DROPRATE,
    BITERR,
    CURRSENSE,
    _END
} state = INIT;

/// Converts ms to the number of complete scheduler runs
#define TIME_TO_INTVAL(ms) \
	((ms) * (1000 / (SCHED_SLOT_PERIOD_US * SCHED_SLOT_COUNT)))
const __flash __uint24 duration_bin_map[] =
{
	1, 2, /* very short dropout durations */
	3, 7, /* prime numbers */
	24, /* max. packet len permitted by RX specs */
	36, /* max. packet len w/o RX required pause */
	/* 50ms .. 500ms delta 50ms */
	TIME_TO_INTVAL(50),	TIME_TO_INTVAL(100),
	TIME_TO_INTVAL(150),	TIME_TO_INTVAL(200),
	TIME_TO_INTVAL(250),	TIME_TO_INTVAL(300),
	TIME_TO_INTVAL(350),	TIME_TO_INTVAL(400),
	TIME_TO_INTVAL(450),	TIME_TO_INTVAL(500),
	/* 600ms .. 1.5s delta 100ms */
	TIME_TO_INTVAL(600),	TIME_TO_INTVAL(700),
	TIME_TO_INTVAL(800),	TIME_TO_INTVAL(900),
	TIME_TO_INTVAL(1000),	TIME_TO_INTVAL(1100),
	TIME_TO_INTVAL(1200),	TIME_TO_INTVAL(1300),
	TIME_TO_INTVAL(1400),	TIME_TO_INTVAL(1500),
	/* 2s .. 5s delta 1s */
	TIME_TO_INTVAL(2000),	TIME_TO_INTVAL(3000),
	TIME_TO_INTVAL(4000),	TIME_TO_INTVAL(5000),
	/* 10s and 20s */
	TIME_TO_INTVAL(10000UL),
	TIME_TO_INTVAL(20000UL)
};
#undef TIME_TO_INTVAL

const __flash __uint24 drop_rate_bin_map[] =
{
          0, (1UL<<17), (1UL<<16), _BV(15), _BV(14), _BV(13), _BV(12), _BV(11),
    _BV(10),    _BV(9),    _BV(8),  _BV(7),  _BV(6),  _BV(5),  _BV(4),       1
};

const __flash __uint24 bit_error_rate_bin_map[] =
{
          0, (1UL<<18), (1UL<<17), (1UL<<16),
    _BV(15), _BV(14), _BV(13), _BV(12), _BV(11), _BV(10), _BV(9), _BV(8),
     _BV(7),  _BV(6),  _BV(5),  _BV(4)
};

/// Reads the ADC calibration value.
static void
production_signature_row_read_calibration(uint16_t * adca_calibration)
{
    uint8_t addr0, addr1;

    addr0 = (uint8_t) (uint16_t) &PRODSIGNATURES_ADCACAL0;
    addr1 = (uint8_t) (uint16_t) &PRODSIGNATURES_ADCACAL1;
    *adca_calibration = (prodsigrow_read_byte(addr1) << 8)
        | prodsigrow_read_byte(addr0);
}

/// Init the ADC to scan the potis and optional current sense input.
static void
init(void)
{
    for (uint8_t i = 0; i < sizeof(adc_raw); ++i)
        ((uint8_t*) &adc_raw)[i] = 0;
    task_adc_biterror_generator.stream_len_bytes = 0;
    task_adc_biterror_generator.from_exp_flip = 0;
    task_adc_biterror_generator.from_gnd_flip = 0;
    task_adc_biterror_generator.force_update = 0;
    task_adc_drop_generator.interval = 0;
    task_adc_drop_generator.start_of_drop = 0;
    task_adc_drop_generator.drop_duration = 0;
    task_adc_drop_generator.force_update = 0;
    state = INIT;

    ADC_PWRUP;

    /* configure pins for input */
    ADC_REF_PORT.DIRCLR = ADC_REF_bm;
    ADC_REF_PORT.OUTCLR = ADC_REF_bm;
    ADC_PORT.DIRCLR = ADC_PORT_PINS_gc;
    ADC_PORT.OUTCLR = ADC_PORT_PINS_gc;

    uint16_t adca_calibration;
    production_signature_row_read_calibration(&adca_calibration);

    /* init ADCA CH0 */
    ADCA.CTRLA = 0;
    ADCA.CTRLB =
        ADC_CURRLIMIT_HIGH_gc | ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
    ADCA.REFCTRL = ADC_REFSEL_gc;
    ADCA.EVCTRL = 0;
    ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;
    ADCA.INTFLAGS =
        ADC_CH3IF_bm | ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;
    ADCA.CAL = adca_calibration;
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_FIRST | ADC_CH_MUXNEG_GND_MODE3_gc;
    ADCA.CH0.INTCTRL = 0;
    ADCA.CH0.SCAN = INPUT_SCAN_COUNT;

    ADCA.CTRLA |= ADC_CH0START_bm | ADC_FLUSH_bm | ADC_ENABLE_bm;
}

static __uint24
get_random(const __uint24 mask)
{
    if (!mask)
        return 0;

    union {
        __uint24 flip;
        struct {
            uint8_t ui8[3];
        } e;
    } random_number;

    for (uint8_t i = 0; i < sizeof(random_number); ++i)
        random_number.e.ui8[i] = (uint8_t) rand();
    /* mask the random number with range 0..2^24-1 to range 0..(mask - 1)
       mask has to be 0 or 2^N, N in {0, ... , 23}                      */
    return random_number.flip & (mask - 1);
}

/// Limit the adc values to the allowed range 0..2047.
static int16_t
limit_adc(const int16_t adc)
{
    if (adc < 0)
        return 0;
    else if (adc > 2047)
        return 2047;
    return adc;
}

static void
update_biterror_generators(void)
{
    /* get a mask for the probability of a bit error with either
       p=0, or p = 1 / 2^N, then decrease the probability by 2^(-4)
       to get the following values:

       bin            | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
       2^N bytes      |  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18  -
       2^(N+3) bits   |  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21  -

       The get_random(mask) function will generate a random uniform
       distributed number in the range
           0..(2^stream_len bytes * 8 bits/byte)
           = 0..2^(stream_len_bytes + 3)

       Therefore, the bit error rate with the poti set to bin=13 is
           p = (1 bit error) / (2^9 bits)
       hence, a bit flip occurs once per
           n = 1/p = 2^9 bits
     */
    int16_t adc = limit_adc(adc_raw.poti_bit_error_rate);
    __uint24 stream_len = bit_error_rate_bin_map[adc / (2048 / 16)];

    if (task_adc_biterror_generator.stream_len_bytes != stream_len)
        task_adc_biterror_generator.force_update = 1;
    task_adc_biterror_generator.stream_len_bytes = stream_len;
    task_adc_biterror_generator.from_exp_flip = get_random(stream_len << 3);
    task_adc_biterror_generator.from_gnd_flip = get_random(stream_len << 3);
}

static void
update_drop_generators(void)
{
    /* get a mask for the probability of a drop event with either
       p=0, or p = 1 / 2^N, then decrease the probability by 2^(-3)
       to get the following values:

       bin            | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
       2^N time units |  0  4  5  6  7  8  9 10 11 12 13 14 15 16 17  -

       At bin=0 communication drop is disabled and at bin=15 all
       communication is dropped entirely (infinite blocking, p=1).

       A "time unit" is fixed to the scheduler interval, i.e. the time
       for a complete run through the scheduler task list:
           time unit = SCHED_SLOT_PERIOD_US * SCHED_SLOT_COUNT
       In case of 25us per slots and a total of 8 slots
           time unit = 200us

       Therefore, if the drop rate poti is set to bin=6 a byte drop
       event in the communication should have probability
           p = (1 drop event) / (2^12 time units)
       and, thus, occur once every
           t = 1 / p = 2^12 * 200us = 0.8192s
     */
    int16_t adc = limit_adc(adc_raw.poti_drop_rate);
    __uint24 interval = drop_rate_bin_map[adc / (2048 / 16)];

    if (task_adc_drop_generator.interval != interval)
        task_adc_drop_generator.force_update = 1;
    task_adc_drop_generator.interval = interval;
    if (0 == interval)                  /* communication drop disabled */
        return;
    task_adc_drop_generator.start_of_drop = get_random(interval);

    adc = limit_adc(adc_raw.poti_drop_duration);
    /* partition range into 32 bins */
    __uint24 duration = duration_bin_map[adc / (2048 / 32)];

    if (task_adc_drop_generator.drop_duration != duration)
        task_adc_drop_generator.force_update = 1;
    task_adc_drop_generator.drop_duration = duration;
}

static void
run(void)
{
    const uint8_t s = state;
    if (INIT == s) {
        /* throw away the first measurement, as it might be wrong */
        ADCA.INTFLAGS = ADC_CH0IF_bm;
        ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_FIRST | ADC_CH_MUXNEG_GND_MODE3_gc;
        ADCA.CH0.SCAN = INPUT_SCAN_COUNT;
        ADCA.CTRLA |= ADC_CH0START_bm;
        state = FIRST_CHANNEL_NAME;
    } else if (FIRST_CHANNEL_NAME <= s && s < _END) {
        if (!(ADCA.INTFLAGS & ADC_CH0IF_bm))
            return;

        const int16_t adc_value = ADCA.CH0RES;
        /* For the potentiometer inputs:
             Update adc_raw only iff the new value exceeds a hysteresis.
             This hysteresis prevents updates of the generator probabilities at
             the boundary of a setting and cancels out noise.
           For the current sensor:
             Ignore the hysteresis since we want the exact sensor readings.  */
        if (CURRSENSE == s) {
            adc_raw.current_sense = adc_value;
        } else {
            int16_t *old_value;
            if (DROPDUR == s)
                old_value = &adc_raw.poti_drop_duration;
            else if (DROPRATE == s)
                old_value = &adc_raw.poti_drop_rate;
            else
                old_value = &adc_raw.poti_bit_error_rate;
            const int8_t MIN_DIFF = 16;
            int16_t diff = *old_value - adc_value;
            if (diff < -MIN_DIFF || diff > MIN_DIFF)
                *old_value = adc_value;
        }

        ++state;
        if (_END == state) {
            ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_FIRST
                | ADC_CH_MUXNEG_GND_MODE3_gc;
            ADCA.CH0.SCAN = INPUT_SCAN_COUNT;

            update_biterror_generators();
            update_drop_generators();

            state = FIRST_CHANNEL_NAME;
        }

        ADCA.INTFLAGS = ADC_CH0IF_bm;
        ADCA.CTRLA |= ADC_CH0START_bm;
    } else {
        /* this should not happen */
        init();
        state = INIT;
    }
}
