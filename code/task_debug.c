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

#define UART_DEBUG_PORT     PORTD
#define UART_DEBUG_RX_bm    PIN6_bm
#define UART_DEBUG_TX_bm    PIN7_bm
#define UART_DEBUG          USARTD1
#define UART_DEBUG_PWRUP    PR.PRPD &= ~PR_USART1_bm

#define UART_BSEL       (131)
#define UART_BSCALE     (-3)

static void init(void);
static void run(void);
const struct task task_debug = { .init = &init, .run = &run };

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
init(void)
{
    UART_DEBUG_PWRUP;
    init_uart(&UART_DEBUG_PORT, UART_DEBUG_RX_bm, UART_DEBUG_TX_bm,
              &UART_DEBUG);

}

static void
run(void)
{
    static uint8_t current_byte = 0;
    static uint8_t buffer[16];

    // TODO use DMA for sending

    if (!(UART_DEBUG.STATUS & USART_DREIF_bm))
        return;

    UART_DEBUG.DATA = buffer[current_byte];

    ++current_byte;
    if (current_byte >= sizeof(buffer)) {
        current_byte = 0;
    }
}
