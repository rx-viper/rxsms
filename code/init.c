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
#include "init.h"

/// Set the CPU clock to F_CPU Hz.
void
init_clock(void)
{
    // enable 32.768 kHz and 32 MHz clock
    const uint8_t enable_clk = OSC.CTRL | OSC_RC32MEN_bm | OSC_RC32KEN_bm;
    CCP = CCP_IOREG_gc;
    OSC.CTRL = enable_clk;
    do {
        asm volatile ("");      // wait for MHz and kHz Osc
    } while (!(OSC.STATUS & OSC_RC32MRDY_bm)
             || !(OSC.STATUS & OSC_RC32KRDY_bm));

    // RC32K calibration not needed (done automatically by hardware after
    // reset), but here could go the CAL value
    //OSC.RC32KCAL = <calibration value>;

    // select RC32K as DFLL reference and set DFLL auto run-time CAL value
    OSC.DFLLCTRL |= OSC_RC32MCREF_RC32K_gc;
    DFLLRC32M.COMP1 = (F_CPU / 1024) & 0xff;
    DFLLRC32M.COMP2 = ((F_CPU / 1024) >> 8) & 0xff;
    DFLLRC32M.CTRL |= DFLL_ENABLE_bm;

    // select 32 MHz clock as system clock
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

    // disable all prescalers (this is the default, but to make it sure)
    CCP = CCP_IOREG_gc;
    CLK.PSCTRL = CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;

    // disable 2MHz internal osc
    const uint8_t disable_clk = OSC.CTRL & (~OSC_RC2MEN_bm);
    CCP = CCP_IOREG_gc;
    OSC.CTRL = disable_clk;

    // disable all further changes to the clock system
    CCP = CCP_IOREG_gc;
    CLK.LOCK = CLK_LOCK_bm;
}

/// Power down all unneeded clocks and modules.
void
init_prr(void)
{
    PR.PRGEN = PR_USB_bm | PR_AES_bm | PR_EBI_bm | PR_RTC_bm | PR_EVSYS_bm
        | PR_DMA_bm;
    /* enable ADC */
    PR.PRPA = PR_DAC_bm | PR_AC_bm;
    /* enable TCC0 */
    PR.PRPC = PR_TWI_bm | PR_USART1_bm | PR_USART0_bm | PR_SPI_bm
        | PR_HIRES_bm | PR_TC1_bm;
    /* enable USARTD0, USARTD1 */
    PR.PRPD = PR_TWI_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC1_bm | PR_TC0_bm;
    PR.PRPE = PR_TWI_bm | PR_USART1_bm | PR_USART0_bm | PR_SPI_bm
        | PR_HIRES_bm | PR_TC1_bm | PR_TC0_bm;
    PR.PRPF = PR_TWI_bm | PR_USART1_bm | PR_USART0_bm | PR_SPI_bm
        | PR_HIRES_bm | PR_TC1_bm | PR_TC0_bm;
}
