/*
 * This file is part of the coreboot project.
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Copyright 2014 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <memlayout.h>
#include <vendorcode/google/chromeos/memlayout.h>

#include <arch/header.ld>

SECTIONS
{
	REGION(rpm, 0x00020000,  160K, 8K)
	SRAM_START(0x2A000000)
	/* This includes bootblock image, can be reused after bootblock starts */
/*	UBER_SBL(0x2A000000, 48K) */
/*	DDR(0x2A000000, 48K) */
	BOOTBLOCK(0x2A00C000, 24K)
	OVERLAP_VERSTAGE_ROMSTAGE(0x2A012000, 64K)
	VBOOT2_WORK(0x2A022000, 16K)
	PRERAM_CBMEM_CONSOLE(0x2A026000, 32K)
	CBFS_HEADER_OFFSET(0x2A02E400)

/*	0x2e404..0x3F000 4 bytes shy of 67KB free */

/* Keep the below area reserved at all times, it is used by various QCA
   components as shared data
	QCA_SHARED_RAM(2A03F000, 4K)
*/
	STACK(0x2A040000, 16K)
#ifdef __PRE_RAM__
       /*
        * ipq8064 is different from most other ARM platforms: it loads the
	* proprietary DRAM initialization code from CBFS (as opposed to compiling
	* it in into rombase). As a result CBFS needs to be used before DRAM is
	* availale, which means CBFS cache must be in SRAM, which in turn means
	* that PRERAM_CBFS_CACHE description can not be used here.
	*/
	CBFS_CACHE(0x2A044000, 93K)
#endif
	TTB_SUBTABLES(0x2A05B800, 2K)
	TTB(0x2A05C000, 16K)
	SRAM_END(0x2A060000)

	DRAM_START(0x40000000)
	RAMSTAGE(0x40640000, 128K)
	SYMBOL(memlayout_cbmem_top, 0x59F80000)
#ifndef __PRE_RAM__
	CBFS_CACHE(0x59F80000, 384K)
#endif
	DMA_COHERENT(0x5A000000, 2M)
}
