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

#ifndef TASK_ERROR_TABLES_H
#define TASK_ERROR_TABLES_H

#include <inttypes.h>
#include "sched.h"  /* for SCHED_SLOT_COUNT and SCHED_SLOT_PERIOD_US */

/// Type to represent tables as string to be printed by task_debug.
struct value_as_string
{
    char s[5];  // use array instead of char* so type has fixed size
};

/// Converts ms to the number of complete scheduler runs
#define TIME_TO_INTVAL(ms) \
	((ms) * (1000 / (SCHED_SLOT_PERIOD_US * SCHED_SLOT_COUNT)))
const __flash __uint24 drop_duration_bin_map[] =
{
	1, 2, /* very short dropout durations */
	3, 7, /* prime numbers */
	24, /* max. packet len permitted by RX specs */
	36, /* max. packet len w/o RX required pause */
	/* 50ms .. 500ms delta 50ms */
	TIME_TO_INTVAL(50),	TIME_TO_INTVAL(100),
	TIME_TO_INTVAL(150),	TIME_TO_INTVAL(200),
	TIME_TO_INTVAL(250),	TIME_TO_INTVAL(300),
	TIME_TO_INTVAL(350),	TIME_TO_INTVAL(400),
	TIME_TO_INTVAL(450),	TIME_TO_INTVAL(500),
	/* 600ms .. 1.5s delta 100ms */
	TIME_TO_INTVAL(600),	TIME_TO_INTVAL(700),
	TIME_TO_INTVAL(800),	TIME_TO_INTVAL(900),
	TIME_TO_INTVAL(1000),	TIME_TO_INTVAL(1100),
	TIME_TO_INTVAL(1200),	TIME_TO_INTVAL(1300),
	TIME_TO_INTVAL(1400),	TIME_TO_INTVAL(1500),
	/* 2s .. 5s delta 1s */
	TIME_TO_INTVAL(2000),	TIME_TO_INTVAL(3000),
	TIME_TO_INTVAL(4000),	TIME_TO_INTVAL(5000),
	/* 10s and 20s */
	TIME_TO_INTVAL(10000UL),
	TIME_TO_INTVAL(20000UL)
};
#undef TIME_TO_INTVAL

const __flash struct value_as_string drop_duration_bin_string_map[] =
{
    {" 1 B "}, {" 2 B "}, {" 3 B "}, {" 7 B "},
    {"24 B "}, {"36 B "},
    {" 50ms"}, {"100ms"}, {"150ms"}, {"200ms"},
    {"250ms"}, {"300ms"}, {"350ms"}, {"400ms"},
    {"450ms"}, {"500ms"},
    {"600ms"}, {"700ms"}, {"800ms"}, {"900ms"},
    {"1.0 s"}, {"1.1 s"}, {"1.2 s"}, {"1.3 s"},
    {"1.4 s"}, {"1.5 s"},
    {" 2 s "}, {" 3 s "}, {" 4 s "}, {" 5 s "},
    {"10 s "}, {"20 s "}
};

// get a mask for the probability of a drop event with either
// p=0, or p = 1 / 2^N, then decrease the probability by 2^(-3)
// to get the following values:
//
// bin            | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// 2^N time units |  0  4  5  6  7  8  9 10 11 12 13 14 15 16 17  -
//
// At bin=0 communication drop is disabled and at bin=15 all
// communication is dropped entirely (infinite blocking, p=1).
//
// A "time unit" is fixed to the scheduler interval, i.e. the time
// for a complete run through the scheduler task list:
//     time unit = SCHED_SLOT_PERIOD_US * SCHED_SLOT_COUNT
// In case of 25us per slots and a total of 8 slots
//     time unit = 200us
//
// Therefore, if the drop rate poti is set to bin=6 a byte drop
// event in the communication should have probability
//     p = (1 drop event) / (2^12 time units)
// and, thus, occur once every
//     t = 1 / p = 2^12 * 200us = 0.8192s
const __flash __uint24 drop_rate_bin_map[] =
{
          0, (1UL<<17), (1UL<<16), _BV(15), _BV(14), _BV(13), _BV(12), _BV(11),
    _BV(10),    _BV(9),    _BV(8),  _BV(7),  _BV(6),  _BV(5),  _BV(4),       1
};

const __flash struct value_as_string drop_rate_bin_string_map[] =
{
    {"  0  "}, {"2^-17"}, {"2^-16"}, {"2^-15"},
    {"2^-14"}, {"2^-13"}, {"2^-12"}, {"2^-11"},
    {"2^-10"}, {"2^ -9"}, {"2^ -8"}, {"2^ -7"},
    {"2^ -6"}, {"2^ -5"}, {"2^ -4"}, {"  1  "},
};

// get a mask for the probability of a bit error with either
// p=0, or p = 1 / 2^N, then decrease the probability by 2^(-4)
// to get the following values:
//
// bin            | 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// 2^N bytes      |  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18  -
// 2^(N+3) bits   |  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21  -
//
// The get_random(mask) function will generate a random uniform
// distributed number in the range
//     0..(2^stream_len bytes * 8 bits/byte)
//     = 0..2^(stream_len_bytes + 3)
//
// Therefore, the bit error rate with the poti set to bin=13 is
//     p = (1 bit error) / (2^9 bits)
// hence, a bit flip occurs once per
//     n = 1/p = 2^9 bits
const __flash __uint24 bit_error_rate_bin_map[] =
{
          0, (1UL<<18), (1UL<<17), (1UL<<16),
    _BV(15), _BV(14), _BV(13), _BV(12), _BV(11), _BV(10), _BV(9), _BV(8),
     _BV(7),  _BV(6),  _BV(5),  _BV(4)
};

const __flash struct value_as_string bit_error_rate_bin_string_map[] =
{
    {"  0  "}, {"2^-18"}, {"2^-17"}, {"2^-16"},
    {"2^-15"}, {"2^-14"}, {"2^-13"}, {"2^-12"},
    {"2^-11"}, {"2^-10"}, {"2^ -9"}, {"2^ -8"},
    {"2^ -7"}, {"2^ -6"}, {"2^ -5"}, {"2^ -4"},
};

#endif                          /* TASK_ERROR_TABLES_H */
