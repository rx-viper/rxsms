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

#ifndef TASK_BUTTONS_H
#define TASK_BUTTONS_H

#include "task.h"

const struct task task_buttons;

struct
{
    uint8_t lo : 1;
    uint8_t soe : 1;
    uint8_t sods : 1;
    uint8_t errinh : 1;
    uint8_t pwr : 1;
} task_buttons_toggle_request;


#endif /* TASK_BUTTONS_H */
