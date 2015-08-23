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
#include "task_debug.h"
#include "task_adc.h"
#include "task_error_tables.h"
#include "task_ctrl.h"

/*
 * Ressources used _once_ by this task:
 * (none)
 *
 * Ressources _continously_ used by this task:
 * UART_DEBUG_PORT: UART_DEBUG_RX_bm, UART_DEBUG_TX_bm
 * UART_DEBUG
 * DMA.CH0
 */

#define UART_DEBUG_PORT     PORTD
#define UART_DEBUG_RX_bm    PIN6_bm
#define UART_DEBUG_TX_bm    PIN7_bm
#define UART_DEBUG          USARTD1
#define UART_DEBUG_PWRUP    PR.PRPD &= ~PR_USART1_bm

#define DMA_TRIGSRC         DMA_CH_TRIGSRC_USARTD1_DRE_gc

#define UART_BSEL       (131)
#define UART_BSCALE     (-3)

static void init(void);
static void run(void);
const struct task task_debug = { .init = &init, .run = &run };

static uint8_t buffer[] =   "Error Inhibit: "       "XXX"   "        " \
                            "Byte Dropout Rate: "   "01234" "        " \
                            "Dropout Duration: "    "01234" "        " \
                            "Bit Error Rate: "      "01234" "\n\r";

static void
init_uart(PORT_t * port, uint8_t rx_pin, uint8_t tx_pin, USART_t * uart)
{
    port->OUTSET = tx_pin;
    port->DIRSET = tx_pin;
    port->DIRCLR = rx_pin;
    uart->CTRLA = 0;
    uart->CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    uart->CTRLC = USART_CMODE_ASYNCHRONOUS_gc
        | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
    uart->BAUDCTRLA = (uint8_t) UART_BSEL;
    uart->BAUDCTRLB = (UART_BSCALE << 4) | (UART_BSEL >> 8);
}

static void
start_dma(void)
{
    DMA.CH0.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_REPEAT_bm | DMA_CH_TRFREQ_bm
                  | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
}

static void
init(void)
{
    UART_DEBUG_PWRUP;
    init_uart(&UART_DEBUG_PORT, UART_DEBUG_RX_bm, UART_DEBUG_TX_bm,
              &UART_DEBUG);

    // configure our DMA channel CH0
    DMA.CH0.ADDRCTRL =
        DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc |
        DMA_CH_DESTRELOAD_NONE_gc  | DMA_CH_DESTDIR_FIXED_gc;
    uint16_t src_addr = (uint16_t) buffer;
    DMA.CH0.SRCADDR0 = (uint8_t) src_addr;
    DMA.CH0.SRCADDR1 = (uint8_t) (src_addr >> 8);
    DMA.CH0.SRCADDR2 = 0;
    uint16_t dest_addr = (uint16_t) &UART_DEBUG.DATA;
    DMA.CH0.DESTADDR0 = (uint8_t) dest_addr;
    DMA.CH0.DESTADDR1 = (uint8_t) (dest_addr >> 8);
    DMA.CH0.DESTADDR2 = 0;
    DMA.CH0.TRIGSRC = DMA_TRIGSRC;
    DMA.CH0.TRFCNT = sizeof(buffer) - 1;    // -1 due to '\0' at end of string
    DMA.CH0.REPCNT = 1;

    start_dma();
}


static void
memcpy_flash(uint8_t *dest, const __flash uint8_t *src, uint8_t n)
{
    if (!n)
        return;
    do {
        *dest++ = *src++;
    } while(--n);
}

const __flash uint8_t on[3] = "ON ";
const __flash uint8_t off[3] = "OFF";

static void
run(void)
{
    // do nothing as long as DMA is running
    if ((DMA.CH0.CTRLB & (DMA_CH_CHBUSY_bm | DMA_CH_CHPEND_bm))
        || !(DMA.INTFLAGS & DMA_CH0TRNIF_bm))
        return;

    // update buffer
    uint8_t biterror_bin = task_adc_generator.biterror_rate_bin;
    uint8_t drop_rate_bin = task_adc_generator.dropout_rate_bin;
    uint8_t drop_duration_bin = task_adc_generator.dropout_duration_bin;

    const __flash uint8_t *src;
    if (task_ctrl_signals.error_inhibit) {
        src = on;
        biterror_bin = 0;
        drop_rate_bin = 0;
    } else {
        src = off;
    }
    memcpy_flash(&buffer[15], src, sizeof(on));

    src = (const __flash uint8_t *) &drop_rate_bin_string_map[drop_rate_bin];
    memcpy_flash(&buffer[45], src, sizeof(struct value_as_string));
    src = (const __flash uint8_t *) &drop_duration_bin_string_map[drop_duration_bin];
    memcpy_flash(&buffer[76], src, sizeof(struct value_as_string));
    src = (const __flash uint8_t *) &bit_error_rate_bin_string_map[biterror_bin];
    memcpy_flash(&buffer[105], src, sizeof(struct value_as_string));

    // wait until our UART is done
    if (!(UART_DEBUG.STATUS & USART_DREIF_bm))
        return;
    start_dma();
}
