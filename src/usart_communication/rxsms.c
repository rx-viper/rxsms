/*! \file *********************************************************************
 *
 * \brief  XMEGA USART interrupt driven driver example source.
 *
 *      This file contains an application that simulates an Radio connection between the rexus rocket and the Ground statioin.
 *		For the communication it uses 2 Uarts, potis and swiches as control.
 *
 * \par Simulator for Radio Connection
 *
 * \author
 *      Kholodkov Jakov, Thomas Gruebler \n
 *      Support email: jakov.kholodkov@tum.de, thomas.gruebler@tum.de
 *
 * $Date: 2013-06-25 13:20 +0200   \n
 * \license
 * ----------------------------------------------------------------------------
 * "THE BEERWARE LICENSE" (Revision 42):
 * Mister X wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can give me something back in return. A beer for example.
 * Kholodkov Jakov
 * Copyleft !
 * ----------------------------------------------------------------------------
 *****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "init.h"
#include "sched.h"
#include "bgtask_sample_adc_inputs.h"

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

    bgtask_sample_adc_inputs.init();

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
