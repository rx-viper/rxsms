/*
 *   This file is part of RXSMS.
 *   Copyright 2014  Kholodkov Jakov (original RXSMS author),
 *                   Thomas Grübler (Mentor)
 *             2014, 2015  Nicolas Benes
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

/*! \brief Main function
 *
 *	Main function, speaks for itself :P
 *
 *	\todo clean up, put more code outside of this function
 */
int
main(void)
{
    init();
//XXX    FUSE_FUSEBYTE5 |= BODACT_CONTINUOUS_gc | BODLVL_2V8_gc;     // initialise BROWN-OUT detection, tg: maybe try different values if reset occurs

    sched_init();
    sched_start();
    sei();

    // TODO Initialize seed for Random - variable
    while (1) {
    }
}
