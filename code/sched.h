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

#ifndef SCHED_H
#define SCHED_H

#define SCHED_SLOT_COUNT     8
#define SCHED_SLOT_PERIOD_US 25

void sched_init(void);
void sched_start(void);

#endif                          /* SCHED_H */
