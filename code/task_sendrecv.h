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

#ifndef TASK_SENDRECV_H
#define TASK_SENDRECV_H

#include <inttypes.h>
#include "task.h"

const struct task task_recv;
const struct task task_send;

struct task_recv_uart_data {
    uint8_t data;
    uint8_t updated:1;
} task_recv_from_gnd, task_recv_from_exp;

#endif                          /* TASK_SENDRECV_H */
