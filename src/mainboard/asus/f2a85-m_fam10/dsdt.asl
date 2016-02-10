/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2012 Advanced Micro Devices, Inc.
 * Copyright (C) 2013 Sage Electronic Engineering, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* DefinitionBlock Statement */
DefinitionBlock (
	"DSDT.AML",		/* Output filename */
	"DSDT",			/* Signature */
	0x02,			/* DSDT Revision, needs to be 2 for 64bit */
	"ASUS  ",		/* OEMID */
	"COREBOOT",		/* TABLE ID */
	0x00010001		/* OEM Revision */
	)
{	/* Start of ASL file */
	/* #include <arch/x86/acpi/debug.asl> */	/* Include global debug methods if needed */

	//#include "acpi/usb_oc.asl"

	//#include "northbridge/amd/amdfam10/amdfam10_util.asl"

	/* Globals for the platform */
	//#include "acpi/mainboard.asl"

	/* Describe the Sleep Methods (WAK, PTS, GTS, etc.) for this platform */
	//#include "acpi/sleep.asl"

	Scope(\_SB) {
		/* global utility methods expected within the \_SB scope */
		//#include <arch/x86/acpi/globutil.asl>

		/* Describe IRQ Routing mapping for this platform (within the \_SB scope) */
		//#include "acpi/routing.asl"

	}   /* End Scope(_SB)  */
}
/* End of ASL file */
