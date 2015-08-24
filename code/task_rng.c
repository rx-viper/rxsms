/*
 *   This file is part of RXSMS.
 *   Copyright 2015  Nicolas Benes
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
#include "task_rng.h"

// Ressources used _once_ by this task:
// (none)
//
// Ressources _continously_ used by this task:
// AES

#define AES_PWRUP           PR.PRGEN &= ~PR_AES_bm

static void init(void);
static void run(void);
const struct task task_rng = { .init = &init, .run = &run };

const __flash uint8_t initial_key[] = {
    0x4c, 0x6e, 0x4e, 0x17, 0x80, 0x4d, 0x3a, 0x63,
    0x78, 0xf0, 0xbd, 0x2a, 0x9f, 0x7c, 0x31, 0x12
};

const __flash uint8_t initial_data[] = {
    0xed, 0x30, 0xfb, 0xde, 0xc8, 0x2f, 0x8e, 0xe2,
    0x24, 0x62, 0x54, 0x93, 0xf2, 0x49, 0x44, 0xd4
};

static void
start(void)
{
    AES.CTRL = AES_START_bm | AES_XOR_bm;
}

static void
init(void)
{
    AES_PWRUP;
    AES.CTRL = AES_RESET_bm;
    for (uint8_t i = 0; i < sizeof(initial_key); ++i)
        AES.KEY = initial_key[i];
    for (uint8_t i = 0; i < sizeof(initial_data); ++i)
        AES.STATE = initial_data[i];
    start();
    task_rng_random.next = 0;
}

static void
run(void)
{
    if (AES.STATUS & AES_ERROR_bm) {
        // apparently there was an error. this should not happen!
        init();
        return;
    }
    if (!(AES.STATUS & AES_SRIF_bm))
        return;
    task_rng_random.next = 0;
    for (uint8_t i = 0; i < sizeof(task_rng_random.ui8); ++i)
        task_rng_random.ui8[i] = AES.STATE;
    // AES_XOR_bm is set to writing 0 keeps the old state
    for (uint8_t i = 0; i < sizeof(task_rng_random.ui8); ++i)
        AES.STATE = 0;
    start();
}
