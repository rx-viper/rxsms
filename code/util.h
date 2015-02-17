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

#ifndef UTIL_H
#define UTIL_H

#include <inttypes.h>

#define FIX_POINTER(_ptr) __asm__ __volatile__("" : "=e" (_ptr) : "0" (_ptr))

#define STORE_POSTINC(to_addr, val) \
	asm volatile( \
	    "st %a[to]+, %[reg]  \n\t" \
	    : [to] "+e" (to_addr) \
	    : "0" (to_addr), [reg] "r" (val) \
	    : "memory" \
	    )

#define LOAD_STORE_POSTINC_CRC(from_addr, crc_datain_addr, to_addr) \
	asm volatile( \
	    "ld __tmp_reg__, %a[from]+  \n\t" \
	    "st %a[to]+, __tmp_reg__ \n\t" \
	    "st %a[crc], __tmp_reg__  \n\t" \
	    : [from] "+e" (from_addr), [to] "+e" (to_addr) \
	    : "0" (from_addr), "1" (to_addr), [crc] "e" (crc_datain_addr) \
	    : "memory" \
	    )

static void
software_reset(void)
{
    CCP = CCP_IOREG_gc;
    RST.CTRL = RST_SWRST_bm;
}

static uint8_t
prodsigrow_read_byte(const uint8_t byte_offs)
{
    uint8_t byte;
    const uint16_t flash_addr = PROD_SIGNATURES_START + byte_offs;
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    asm volatile ("lpm  %[val], Z  \n\t"
                  : [val] "=&r"(byte)
                  : "z"(flash_addr)
                  : );
    NVM.CMD = NVM_CMD_NO_OPERATION_gc;
    return byte;
}

#endif                          /* UTIL_H */
