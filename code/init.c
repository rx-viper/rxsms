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
#include "init.h"

/// Set the CPU clock to F_CPU Hz.
void init_clock(void)
{
	// enable 32.768 kHz and 32 MHz clock
	const uint8_t enable_clk = OSC.CTRL | OSC_RC32MEN_bm | OSC_RC32KEN_bm;
	CCP = CCP_IOREG_gc;
	OSC.CTRL = enable_clk;
	do {
		asm volatile ("");		// wait for MHz and kHz Osc
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

/// Init all IO ports.
void init_io(void)
{
#if 0
        // Status LEDs: output and off
	STATUS_LED_PORT.DIRSET =
	    STATUS_LED0_bm | STATUS_LED1_bm | STATUS_LED2_bm;
	STATUS_LED_PORT.OUTCLR =
	    STATUS_LED0_bm | STATUS_LED1_bm | STATUS_LED2_bm;
	// Fiber PCB LEDs: output and off
	LED_CHIP_PORT.DIRSET = LED_CHIPA_bm | LED_CHIPB_bm
	    | LED_CHIPC_bm | LED_CHIPD_bm;
	LED_CHIP_PORT.OUTCLR = LED_CHIPA_bm | LED_CHIPB_bm
	    | LED_CHIPC_bm | LED_CHIPD_bm;

	// Laser On/Off signal to OFF by default
	LASER_PORT.DIRSET = LASER_PIN_bm;
	LASER_PORT.OUTSET = LASER_PIN_bm;

	// RXSM control interface: input
	RXSM_CTRL.DIRCLR = RXSM_SOE_bm | RXSM_LO_bm | RXSM_SODS_bm;
	// RXSM serial interface: in/out
	RXSM_UART_PORT.DIRCLR = RXSM_UART_RX_bm;
	RXSM_UART_PORT.DIRSET = RXSM_UART_TX_bm;
	RXSM_UART_PORT.OUTSET = RXSM_UART_TX_bm;

	// Fiber ADCs, acceleration sensors: SCK, SS, MOSI, CNV: out; MISO: in
	PCACCEL_PORT.DIRSET = PCACCEL_CNV_bm | PCACCEL_SCK_bm
	    | PCACCEL_MOSI_bm | PCACCEL_SS_bm;
	PCACCEL_PORT.DIRCLR = PCACCEL_MISO_bm;
	PCACCEL_PORT.OUTCLR = PCACCEL_CNV_bm | PCACCEL_MOSI_bm;
	PCACCEL_PORT.OUTSET = PCACCEL_SCK_bm | PCACCEL_SS_bm;
	// Fiber temperature sensors, aux. ADC
	//  CNV, MOSI, SS, SCK: out
	//  MISO: in
	PDTEMP_PORT.DIRSET = PDTEMP_CNV_bm | PDTEMP_SCK_bm
	    | PDTEMP_MOSI_bm | PDTEMP_SS_bm;
	PDTEMP_PORT.DIRCLR = PDTEMP_MISO_bm;
	PDTEMP_PORT.OUTCLR = PDTEMP_CNV_bm | PDTEMP_MOSI_bm;
	PDTEMP_PORT.OUTSET = PDTEMP_SCK_bm | PDTEMP_SS_bm;

	// Chain C, aux. temperature sensors, ADXL accel. sensor
	ADXL_CS_PORT.OUTSET = ADXL_CS_bm;
	ADXL_CS_PORT.DIRSET = ADXL_CS_bm;
	LM74LOCAL_CS_PORT.OUTSET = LM74LOCAL_CS_bm;
	LM74LOCAL_CS_PORT.DIRSET = LM74LOCAL_CS_bm;
	LM74ADXL_CS_PORT.OUTSET = LM74ADXL_CS_bm;
	LM74ADXL_CS_PORT.DIRSET = LM74ADXL_CS_bm;

	AUXBUS_PORT.DIRSET = AUXBUS_SCK_bm | AUXBUS_MOSI_bm;
	AUXBUS_PORT.OUTCLR = AUXBUS_SCK_bm;
	AUXBUS_PORT.OUTSET = AUXBUS_MOSI_bm;
	AUXBUS_PORT.DIRCLR = AUXBUS_MISO_bm;

// XXX check whether a connection (even unused) exists. MUST init if it exists.
//      // ADXL interrupt inputs: currently unused but need to be initialized
//      ADXL_INT_PORT.DIRCLR = ADXL_INT0_bm | ADXL_INT1_bm;

	// SD-Card
	SDCARD_PORT.OUTSET = SDCARD_SS_bm | SDCARD_MOSI_bm;
	SDCARD_PORT.OUTCLR = SDCARD_RESET_bm;
	SDCARD_PORT.DIRSET = SDCARD_SCK_bm | SDCARD_MOSI_bm | SDCARD_SS_bm
	    | SDCARD_RESET_bm;
	SDCARD_PORT.DIRCLR = SDCARD_BUSY_bm | SDCARD_MISO_bm | SDCARD_ACTIVE_bm;
	SDCARD_PORT.OUTCLR = SDCARD_SCK_bm;
#endif
}

/// Power down all unneeded clocks and modules.
void init_prr(void)
{
        /* enable DMA */
	PR.PRGEN = PR_USB_bm | PR_AES_bm | PR_EBI_bm | PR_RTC_bm | PR_EVSYS_bm;
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
