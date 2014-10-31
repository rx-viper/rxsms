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

#include <avr/io.h>
#include "task_sample_adc_inputs.h"
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

/* "ADC_CH_NOGAIN_MUXNEG_PADGND_gc" missing in avr/io.h corresponds to 0x05 */
#define ADC_CH_NOGAIN_MUXNEG_PADGND_gc          0x05
/* "ADC_CH_WGAIN_MUXNEG_PADGND_gc" missing in avr/io.h corresponds to 0x07 */
#define ADC_CH_WGAIN_MUXNEG_PADGND_gc           0x07

#define ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc    ADC_CH_MUXPOS_PIN1_gc
#define ADC_CH_MUXPOS_POTI_BLOCKING_RATE_gc     ADC_CH_MUXPOS_PIN2_gc
#define ADC_CH_MUXPOS_POTI_BLOCKING_DURATION_gc ADC_CH_MUXPOS_PIN3_gc
#define ADC_CH_MUXPOS_CURRENT_SENSE_gc          ADC_CH_MUXPOS_PIN4_gc


static void init(void);
static void run(void);
const struct task task_sample_adc_inputs = { .init = &init, .run = &run };

/// Reads the ADC calibration value and TEMPSENSE calibration value.
static void
production_signature_row_read_calibration(uint16_t *adca_calibration, uint16_t *tempsense_calibration)
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
///
/// It uses the first ADC channel CH0 in FREERUNning mode and sets the
/// SCAN register to consecutively sample the inputs for the three potis
/// and the current sense input.
/// The results are written to the given buffer through DMA CH0.
static void
init(void)
{
    adc_sense_buffer.is_updated = 0;

    /* configure pins for input */
    PORTA.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm;
    PORTA.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm;

    uint16_t adca_calibration, tempsense_calibration;
    production_signature_row_read_calibration(&adca_calibration,
           &tempsense_calibration);

    /* init ADCA and ADC CH0 to be used in FREERUN and SCAN mode */
    ADCA.CTRLA = 0;
    ADCA.CTRLB = ADC_CURRLIMIT_LARGE_gc | ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc | ADC_TEMPREF_bm;
    ADCA.EVCTRL = 0;
    ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;
    ADCA.INTFLAGS = ADC_CH3IF_bm | ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;
    ADCA.CAL = adca_calibration;

    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc | ADC_CH_NOGAIN_MUXNEG_PADGND_gc;
    ADCA.CH0.INTCTRL = 0;
    ADCA.CH0.SCAN = 3;

    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXINT_TEMP_gc | ADC_CH_NOGAIN_MUXNEG_PADGND_gc;
    ADCA.CH1.INTCTRL = 0;
    ADCA.CH1.SCAN = 0;

    ADCA.CTRLA |= ADC_CH1START_bm | ADC_CH0START_bm | ADC_FLUSH_bm | ADC_ENABLE_bm;
}


static void
run(void)
{
    static enum { INIT, BITERR, BLOCKRATE, BLOCKDUR, CURRSENSE } state = INIT;
    switch (state)
    {
        case INIT:  /* throw away the first measurement, as it might be wrong */
            ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
            ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc | ADC_CH_NOGAIN_MUXNEG_PADGND_gc;
            ADCA.CH0.SCAN = 3;
            ADCA.CTRLA |= ADC_CH1START_bm | ADC_CH0START_bm;
            state = BITERR;
            break;
        case BITERR:
            if (!(ADCA.INTFLAGS & ADC_CH0IF_bm) ||
                !(ADCA.INTFLAGS & ADC_CH1IF_bm))
                break;
            adc_sense_buffer.poti_bit_error_rate = ADCA.CH0RES;
            adc_sense_buffer.is_updated = 0;
            // TODO where to store the tempsense result?
            ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
            state = BLOCKRATE;
            break;
        case BLOCKRATE:
            if (!(ADCA.INTFLAGS & ADC_CH0IF_bm) ||
                !(ADCA.INTFLAGS & ADC_CH1IF_bm))
                break;
            adc_sense_buffer.poti_blocking_rate = ADCA.CH0RES;
            // TODO where to store the tempsense result?
            ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
            state = BLOCKDUR;
            break;
        case BLOCKDUR:
            if (!(ADCA.INTFLAGS & ADC_CH0IF_bm) ||
                !(ADCA.INTFLAGS & ADC_CH1IF_bm))
                break;
            adc_sense_buffer.poti_blocking_duration = ADCA.CH0RES;
            // TODO where to store the tempsense result?
            ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
            state = CURRSENSE;
            break;
        case CURRSENSE:
            if (!(ADCA.INTFLAGS & ADC_CH0IF_bm) ||
                !(ADCA.INTFLAGS & ADC_CH1IF_bm))
                break;
            adc_sense_buffer.current_sense = ADCA.CH0RES;
            adc_sense_buffer.is_updated = 0xff;
            // TODO where to store the tempsense result?
            ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc | ADC_CH_NOGAIN_MUXNEG_PADGND_gc;
            ADCA.CH0.SCAN = 3;
            ADCA.INTFLAGS = ADC_CH1IF_bm | ADC_CH0IF_bm;
            state = BITERR;
            break;
        default:
            /* this should not happen */
            init();
            state = INIT;
    }
}
