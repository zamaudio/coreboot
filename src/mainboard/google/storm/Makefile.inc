##
## This file is part of the coreboot project.
##
## Copyright 2014 Google Inc.
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; version 2 of the License.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
##

bootblock-y += bootblock.c
bootblock-y += cdp.c
bootblock-y += mmu.c
bootblock-y += reset.c

verstage-y += cdp.c
verstage-y += chromeos.c
verstage-y += gsbi.c
verstage-y += memlayout.ld
verstage-y += reset.c

romstage-y += romstage.c
romstage-y += cdp.c
romstage-y += mmu.c
romstage-y += reset.c

ramstage-y += boardid.c
ramstage-y += cdp.c
ramstage-y += chromeos.c
ramstage-y += mainboard.c
ramstage-y += mmu.c
ramstage-y += reset.c

bootblock-y += memlayout.ld
romstage-y += memlayout.ld
ramstage-y += memlayout.ld
