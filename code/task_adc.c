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
    for (uint8_t i = 0; i < sizeof(task_adc_generator); ++i)
        ((uint8_t*) &task_adc_generator)[i] = 0;
    srandom(0);
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
update_generator(void)
{
#define GEN task_adc_generator
    uint8_t bin;
    bin = limit_adc(adc_raw.poti_bit_error_rate) / (2048 / 16);
    if (GEN.biterror_rate_bin != bin)
        GEN.biterror_force_update = 1;
    GEN.biterror_rate_bin = bin;

    bin = limit_adc(adc_raw.poti_drop_rate) / (2048 / 16);
    if (GEN.dropout_rate_bin != bin)
        GEN.dropout_force_update = 1;
    GEN.dropout_rate_bin = bin;

    bin = limit_adc(adc_raw.poti_drop_duration) / (2048 / 32);
    if (GEN.dropout_duration_bin != bin)
        GEN.dropout_force_update = 1;
    GEN.dropout_duration_bin = bin;

#if RANDOM_MAX < 0xFFFFFF
#error "Need at least 3B of random data for __uint24 per random() call"
#endif

    const uint8_t array_len = sizeof(GEN.random) / sizeof(GEN.random[0]);
    for (uint8_t i = 0; i < array_len; ++i)
        GEN.random[i] = random();
#undef GEN
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
        // For the potentiometer inputs:
        //   Update adc_raw only iff the new value exceeds a hysteresis.
        //   This hysteresis prevents updates at the boundary of a bin and
        //   cancels out noise.
        // For the current sensor:
        //   Ignore the hysteresis since we want the exact sensor readings.
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
            update_generator();
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
