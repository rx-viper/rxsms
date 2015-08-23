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

/// Type to represent tables as string to be printed by task_debug.
struct value_as_string
{
    char s[5];  // use array instead of char* so type has fixed size
};

extern const __flash __uint24 drop_duration_bin_map[];
extern const __flash struct value_as_string drop_duration_bin_string_map[];
extern const __flash __uint24 drop_rate_bin_map[];
extern const __flash struct value_as_string drop_rate_bin_string_map[];
extern const __flash __uint24 bit_error_rate_bin_map[];
extern const __flash struct value_as_string bit_error_rate_bin_string_map[];

#endif                          /* TASK_ERROR_TABLES_H */
