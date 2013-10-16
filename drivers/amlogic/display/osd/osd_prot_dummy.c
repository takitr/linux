/*
 * Amlogic Meson
 * frame buffer driver
 *
 * Copyright (C) 2009 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Amlogic Platform Group
 *
 */

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <linux/irqreturn.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/osd/osd.h>
#include <linux/vout/vout_notify.h>
#include <linux/amports/canvas.h>
#include "osd_log.h"
#include <linux/amlog.h>
#include "osd_prot.h"


int osd_set_prot(unsigned char   x_rev,
                unsigned char   y_rev,
                unsigned char   bytes_per_pixel,
                unsigned char   conv_422to444,
                unsigned char   little_endian,
                unsigned int    hold_lines,
                unsigned int    x_start,
                unsigned int    x_end,
                unsigned int    y_start,
                unsigned int    y_end,
                unsigned int    y_len_m1,
                unsigned char   y_step,
                unsigned char   pat_start_ptr,
                unsigned char   pat_end_ptr,
                unsigned long   pat_val,
                unsigned int    canv_addr,
                unsigned int    cid_val,
                unsigned char   cid_mode,
                unsigned char   cugt,
                unsigned char   req_onoff_en,
                unsigned int    req_on_max,
                unsigned int    req_off_min,
                unsigned char   osd_index,
                unsigned char   on)
{
    printk("PROT unsupported for this platform\r\n");
    return 0;
}   

