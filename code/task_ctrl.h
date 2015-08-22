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

#ifndef TASK_CTRL_H
#define TASK_CTRL_H

#include <inttypes.h>
#include "task.h"

const struct task task_ctrl;

struct {
    uint8_t lo_asserted:1;
    uint8_t soe_asserted:1;
    uint8_t sods_asserted:1;
    uint8_t pwr_on:1;
    uint8_t error_inhibit:1;
} task_ctrl_signals;

#endif                          /* TASK_CTRL_H */
