/*
 *   This file is part of RXSMS.
 *   Copyright 2014  Kholodkov Jakov (original RXSMS author),
 *                   Thomas Grübler, Nicolas Benes
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
#include <avr/interrupt.h>
#include "init.h"
#include "sched.h"

#if 0
// TODO delete
volatile uint8_t stop_data;
volatile uint32_t rando, compare;       // randomizer values
volatile uint16_t poti1, poti2, poti3, counter; // POTENIOMETER values
volatile uint8_t disable_noise_generation = 0;
#endif

/*! \brief Main function
 *
 *	Main function, speaks for itself :P
 *
 *	\todo clean up, put more code outside of this function
 */
int
main(void)
{
    init_clock();
    init_prr();
    init_io();

//XXX    FUSE_FUSEBYTE5 |= BODACT_CONTINUOUS_gc | BODLVL_2V8_gc;     // initialise BROWN-OUT detection, tg: maybe try different values if reset occurs

    sched_init();
    sched_start();
    sei();

    // TODO Initialize seed for Random - variable
    while (1) {
#if 0
        ADCA.CTRLA |= ADC_CH0START_bm | ADC_CH1START_bm | ADC_CH2START_bm | ADC_CH3START_bm;
        poti1 = 2.3 * adc_sense_buffer.poti_bit_error_rate - 400;
        poti2 = (adc_sense_buffer.poti_blocking_rate - 180) >> 4;
        poti3 = (adc_sense_buffer.poti_blocking_duration - 180) << 4;

        //generate compare Value for error randomizer : from 12 bit into 32 bit
        if (poti1 <= 80) {
            compare = 0;
        } else {
            compare = ((uint32_t) poti1 << 6);
        }

        // Checking for error number(poti2) , If bigger : Start timer and wait a certain time(poti3) while not transmitting data.
        if (counter >= poti2) {
            stop_data = true;
            start_timer(poti3);
            LED_RXTX_PORT.OUTSET = LRX_bm | LTX_bm;
        }
#endif
    }
}
