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

#ifndef TASK_RNG_H
#define TASK_RNG_H

#include <inttypes.h>
#include "task.h"

const struct task task_rng;

struct {
    uint8_t ui8[16]; ///< the random bytes
    uint8_t next:4; ///< the index of the random byte that should be read next
} task_rng_random;

#endif                          /* TASK_RNG_H */
