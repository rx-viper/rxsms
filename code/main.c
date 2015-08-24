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

#include <avr/interrupt.h>
#include "init.h"
#include "sched.h"

// NOTE:
// We may use other FUSE settings, for instance to enable brown-out detection,
// but it works fine without them now.
// If they become necessary at a later point in time, we can add them then.
//
//      see FUSE_FUSEBYTE1..5 in the uC's io.h (ATxmega32A4u: iox32a4u.h)

/// Initialize and start the scheduler, and as a result all tasks.
int
main(void)
{
    init();

    sched_init();
    sched_start();
    sei();

    while (1) {
        // nothing much to do here:
        // the fun happens in the tasks executed by the scheduler ISR
        //      see sched.c and task_*.c
    }
}
