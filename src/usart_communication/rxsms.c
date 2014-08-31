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

#include "main.h"
#include "init.h"
#include "bgtask_sample_adc_inputs.h"

/*! USART data struct used in example. */
USART_data_t UsartDataGnd;      // D1 USART - Data
USART_data_t UsartDataExp;      // D0 Usart - Data

volatile uint8_t stop_data;
volatile uint32_t rando, compare;       // randomizer values
volatile uint16_t poti1, poti2, poti3, counter; // POTENIOMETER values
volatile uint8_t disable_noise_generation = 0;

/*! \brief Interrupt init
 *
 *  Enable all interrupts
 *
 *	\todo choose what to enable (high, low, mid)
 */
void
interrupt_init()
{
    PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
}

/*! \brief Set PIN directions
 *
 *  All output pins to output
 *
 */
void
set_directions(void)
{
    LED_PORT.DIRSET = LLO_bm | LSOE_bm | LSODS_bm | LPS_bm;
    LED5_PORT.DIRSET = LPWR_bm;

    RXSIG_PORT.DIRSET = LO_bm | SOE_bm | SODS_bm;

    SUPPLY_CTRL_PORT.DIRSET = SUPPLY_CTRL_bm;

    LED_RXTX_PORT.DIRSET = LTX_bm | LRX_bm;
}

/*! \brief UART Init
 *
 *  Initializes+enables both USARTs
 *
 *	\todo maybe more beautiful if splitted into two functions with parameters
 */
void
uart_init()
{
    PORTD.DIRSET = PIN7_bm;     //TX - USARTD1 As GroundUsart (tg: checked)
    PORTD.DIRCLR = PIN6_bm;     //RX

    /* Use USARTD1 and initialize buffers. */
    USART_InterruptDriver_Initialize(&UsartDataGnd, &USARTGROUND,
                                     USART_DREINTLVL_HI_gc);

    /* USARTD1, 8 Data bits, No Parity, 1 Stop bit. */
    USART_Format_Set(UsartDataGnd.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

    /* Enable RXC interrupt. */
    USART_RxdInterruptLevel_Set(UsartDataGnd.usart, USART_RXCINTLVL_HI_gc);

    // Set Baudrate to 38,4 Kbps:
    USART_Baudrate_Set(&USARTGROUND, 3269, -6);

    /* Enable both RX and TX. */
    USART_Rx_Enable(UsartDataGnd.usart);
    USART_Tx_Enable(UsartDataGnd.usart);

    PORTD.DIRSET = PIN3_bm;     //TX - USARTD0 As ExperimentUsart (tg: checked)
    PORTD.DIRCLR = PIN2_bm;     //RX

    /* Use USARTD0 and initialize buffers. */
    USART_InterruptDriver_Initialize(&UsartDataExp, &USARTROCKET,
                                     USART_DREINTLVL_HI_gc);

    /* USARTD0, 8 Data bits, No Parity, 1 Stop bit. */
    USART_Format_Set(UsartDataExp.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

    /* Enable RXC interrupt. */
    USART_RxdInterruptLevel_Set(UsartDataExp.usart, USART_RXCINTLVL_HI_gc);

    // Set Baudrate to 38.4 Kbps:
    USART_Baudrate_Set(&USARTROCKET, 3269, -6); // eventually changeable Baud for Limit testing

    /* Enable both RX and TX. */
    USART_Rx_Enable(UsartDataExp.usart);
    USART_Tx_Enable(UsartDataExp.usart);
}

/*! \brief Timer init
 *
 *  Enable Timer/Counter 0.
 *
 *	\see stop_timer()
 *	\see start_timer()
 */
void
timer_init()
{
    TC0_ConfigWGM(&TCC0, TC_WGMODE_NORMAL_gc);
    TC0_SetOverflowIntLevel(&TCC0, PMIC_MEDLVLEN_bm);
}

/*! \brief Start timer
 *
 *  Start TC0
 *
 *	\param max Maximum timer value
 *
 */
void
start_timer(uint16_t max)
{
    TCC0.PER = max;
    TC0_ConfigClockSource(&TCC0, TC_CLKSEL_DIV1024_gc);
}

/*! \brief Stop timer
 *
 *  Stop TC0 by setting max value to 0
 *
 */
void
stop_timer()
{
    TC0_ConfigClockSource(&TCC0, TC_CLKSEL_OFF_gc);
    TCC0.PER = 0;
}

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

    uint16_t seed;
    cli();
    FUSE_FUSEBYTE5 |= BODACT_CONTINUOUS_gc | BODLVL_2V8_gc;     // initialise BROWN-OUT detection, tg: maybe try different values if reset occurs
    uart_init();
    set_directions();
    interrupt_init();
    timer_init();

    // Initialise interrupts for switches
    PORTCFG.MPCMASK |= SWLO_bm | SWSOE_bm | SWSODS_bm | SWPS_bm;
    SW_PORT.PIN5CTRL |= PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;       // First 4 switches

    SW_PORT.INT0MASK |= SWLO_bm | SWSOE_bm | SWSODS_bm | SWPS_bm;
    SW_PORT.INTCTRL |= PORT_INT0LVL_LO_gc;

    SWPWR_PORT.PIN1CTRL |= PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;
    SWPWR_PORT.INT0MASK |= SWPWR_bm;
    SWPWR_PORT.INTCTRL |= PORT_INT0LVL_LO_gc;
    sei();

    // TODO Initialize seed for Random - variable
    while (1) {
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


    }
}

/*! \brief Experiment USART Interrupt
 *
 *  Interrupt called when experiment sends data to ground
 *
 *	\todo write more beautiful code, e.g. use stdlib stuff or create a randr function instead of this calculation
 *	\todo hardware upgrade idea: use noise of a zener diode as random value
 *
 */
ISR(USARTD0_RXC_vect)           // DATA DIRECTION FROM experiment to GROUND
{
    uint8_t data = USARTROCKET.DATA;
    uint8_t err_var = 0;
    if (disable_noise_generation == 0) {
        if (stop_data == 0) {
            for (uint8_t i = 0; i < 8; i++) {
                rando = (rando * 217 + 667);
                if (rando < compare) {
                    err_var |= (1 << i);
                }
            }
            if (err_var != 0) {
                counter++;
            }

            data ^= err_var;
            USARTGROUND.DATA = data;
            LED_RXTX_PORT.OUTTGL = LRX_bm;
        }

    } else {
        USARTGROUND.DATA = data;
        LED_RXTX_PORT.OUTTGL = LRX_bm;
    }

}

/*! \brief Ground USART Interrupt
 *
 *  Interrupt called when ground station sends data to experiment. No data loss simulated.
 *
 * \todo implement deactivateable data loss feature for experiments that use real uplink
 *
 */
ISR(USARTE0_RXC_vect)           // DATA DIRECTION FROM GROUND to experiment
{
    uint8_t data = USARTGROUND.DATA;    // Transit without Interrupting data line but listening to magic strings
    USARTROCKET.DATA = data;
    LED_RXTX_PORT.OUTTGL = LTX_bm;
}

/*! \brief Button press interrupt
 *
 *  Called for pressed buttons
 *
 *	\todo change to switch/case format
 *
 */
ISR(PORTB_INT0_vect)
{
    const uint8_t pins = ~(SW_PORT.IN);

    if ((pins & SWLO_bm))       // readout switches and set a job variable for each switch
    {
        LED_PORT.OUTTGL = LLO_bm;       //switch led indicator on/off
        RXSIG_PORT.OUTTGL = LO_bm;      //switch rexus rocket signal on/off
    }
    if ((pins & SWSOE_bm)) {
        LED_PORT.OUTTGL = LSOE_bm;      // -- "" --
        RXSIG_PORT.OUTTGL = SOE_bm;
    }
    if ((pins & SWSODS_bm)) {
        LED_PORT.OUTTGL = LSODS_bm;
        RXSIG_PORT.OUTTGL = SODS_bm;
    }
    if ((pins & SWPS_bm)) {
        LED_PORT.OUTTGL = LPS_bm;       //override indicator led
        disable_noise_generation ^= 1;  //XORG/Toogle bit 0.
    }
}

/*! \brief Button press interrupt
 *
 *  Called for pressed buttons
 *
 *	\todo change to switch/case format
 *
 */
ISR(PORTC_INT0_vect)
{
    const uint8_t pins = ~(SWPWR_PORT.IN);      // same for power supply

    if (pins & SWPWR_bm) {
        LED5_PORT.OUTTGL = LPWR_bm;
        SUPPLY_CTRL_PORT.OUTTGL = SUPPLY_CTRL_bm;
    }
}

/*! \brief Timer overflow interrupt
 *
 *  Called on timer overflow. Deactivates stop_data so data will be transmitted again.
 *
 */
ISR(TCC0_OVF_vect)
{
    counter = 0;
    stop_data = false;
    stop_timer();
    LED_RXTX_PORT.OUTCLR = LTX_bm | LRX_bm;     // i think we should switch them off earlier
}
