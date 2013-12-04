/*
 * arch/arm/mach-meson8/clockgate.c
 *
 * Copyright (C) 2013-2014 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


/*
 * The control routines here are for CBUS register 0x1050, 0x1051, 0x1052, 0x1054
 * for CLK81 and Miscellaneous Clock Gates
 */

#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>

#include <mach/am_regs.h>
#include <mach/clkgate.h>

/* we could add lock for each bit but the operation here
 * is not often so just use a single lock for now.
 */
static DEFINE_SPINLOCK(lock);

static unsigned long reg_addr[GCLK_MAX] = {
    P_HHI_GCLK_MPEG0, P_HHI_GCLK_MPEG1, P_HHI_GCLK_MPEG2, P_HHI_GCLK_OTHER
};

static atomic_t use_count[GCLK_MAX][32];

void clkgate_get(gate_type_t type, unsigned int bit)
{
    if ((type <= GCLK_MAX) && (bit < 32)) {
        spin_lock(&lock);
        if (atomic_inc_return(&use_count[type][bit]) == 1) {
            aml_set_reg32_bits(reg_addr[type], 1, bit, 1);
        }
        spin_unlock(&lock);
    }
}

void clkgate_put(gate_type_t type, unsigned int bit)
{
    if ((type <= GCLK_MAX) && (bit < 32)) {
        if (atomic_dec_and_lock(&use_count[type][bit], &lock)) {
            aml_set_reg32_bits(reg_addr[type], 0, bit, 1);
            spin_unlock(&lock);
        }
    }
}

EXPORT_SYMBOL(clkgate_get);
EXPORT_SYMBOL(clkgate_put);

