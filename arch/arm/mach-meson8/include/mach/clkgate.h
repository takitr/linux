/*
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

#ifndef CLKGATE_H
#define CLKGATE_H

typedef enum {
    GCLK_MPEG0 = 0,
    GCLK_MPEG1 = 1,
    GCLK_MPEG2 = 2,
    GCLK_OTHER = 3,
    GCLK_MAX   = 4
} gate_type_t;

#define CLKGATE_BIT_U_DOS_TOP	        1
#define CLKGATE_BIT_U_PARSER_TOP        25

extern void clkgate_get(gate_type_t type, unsigned int bit);
extern void clkgate_put(gate_type_t type, unsigned int bit);

#endif /* CLKGATE_H */

