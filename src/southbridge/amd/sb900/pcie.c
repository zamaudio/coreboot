/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2010 Advanced Micro Devices, Inc.
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

#include <console/console.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ids.h>
#include <device/pci_ops.h>
#include "sb900.h"

static struct pci_operations lops_pci = {
	.set_subsystem = 0,
};

static struct device_operations pci_ops = {
	.read_resources = pci_bus_read_resources,
	.set_resources = pci_dev_set_resources,
	.enable_resources = pci_bus_enable_resources,
	.scan_bus = pci_scan_bridge,
	.reset_bus = pci_bus_reset,
	.ops_pci = &lops_pci,
};

static const struct pci_driver pciea_driver __pci_driver = {
	.ops = &pci_ops,
	.vendor = PCI_VENDOR_ID_AMD,
	.device = PCI_DEVICE_ID_ATI_SB900_PCIEA,
};

static const struct pci_driver pcieb_driver __pci_driver = {
	.ops = &pci_ops,
	.vendor = PCI_VENDOR_ID_AMD,
	.device = PCI_DEVICE_ID_ATI_SB900_PCIEB,
};
static const struct pci_driver pciec_driver __pci_driver = {
	.ops = &pci_ops,
	.vendor = PCI_VENDOR_ID_AMD,
	.device = PCI_DEVICE_ID_ATI_SB900_PCIEC,
};
static const struct pci_driver pcied_driver __pci_driver = {
	.ops = &pci_ops,
	.vendor = PCI_VENDOR_ID_AMD,
	.device = PCI_DEVICE_ID_ATI_SB900_PCIED,
};
