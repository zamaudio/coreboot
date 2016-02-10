/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2012 Advanced Micro Devices, Inc.
 * Copyright (C) 2012 Rudolf Marek <r.marek@assembler.cz>
 * Copyright (C) 2017 Damien Zammit <damien@zamaudio.com>
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

#include <arch/acpi.h>
#include <arch/cpu.h>
#include <arch/io.h>
#include <arch/stages.h>
#include <cbmem.h>
#include <console/console.h>
#include <cpu/amd/car.h>
#include <cpu/x86/bist.h>
#include <cpu/x86/lapic.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <device/pnp_def.h>
#include <spd.h>

#include <cpu/amd/model_10xxx_rev.h>
#include <lib.h>
#include <cpu/x86/lapic.h>
#include <commonlib/loglevel.h>
#include <cpu/x86/bist.h>
#include <cpu/amd/mtrr.h>
#include <southbridge/amd/sb900/smbus.h>
#include <northbridge/amd/amdfam10/raminit.h>
#include <northbridge/amd/amdht/ht_wrapper.h>
#include <cpu/amd/family_10h-family_15h/init_cpus.h>
#include <arch/early_variables.h>
#include <cbmem.h>
#include <superio/ite/common/ite.h>
#include <superio/ite/it8728f/it8728f.h>
#include <southbridge/amd/sb900/sb900.h>
#include <southbridge/amd/sb900/smbus.h>
#include <delay.h>

#include "cpu/amd/quadcore/quadcore.c"

#define MMIO_NON_POSTED_START 0xfed00000
#define MMIO_NON_POSTED_END   0xfedfffff
#define SB_MMIO 0xFED80000
#define SB_MMIO_MISC32(x) *(volatile u32 *)(SB_MMIO + 0xE00 + (x))

#define SMBUS_IO_BASE 0xb00
#define SMBUS_AUX_IO_BASE 0xb20

#define SERIAL_DEV PNP_DEV(0x2e, IT8728F_SP1)
#define GPIO_DEV PNP_DEV(0x2e, IT8728F_GPIO)

extern struct sys_info sysinfo_car;
void activate_spd_rom(const struct mem_controller *ctrl);
int spd_read_byte(unsigned device, unsigned address);

static void writepmreg (int reg, int data)
{
	outb(reg, 0xCD6);
	outb(data, 0xCD7);
}

void activate_spd_rom(const struct mem_controller *ctrl)
{
	u32 iobase = SMBUS_IO_BASE;

	writepmreg (0x2D, iobase >> 8);
	writepmreg (0x2C, iobase | 1);
	/* Set SMBUS clock to 400KHz */
	outb(66000000 / 400000 / 4, iobase + 0x0E);
}

int spd_read_byte(unsigned device, unsigned address)
{
	return do_smbus_read_byte(SMBUS_IO_BASE, device, address);
}

static const uint8_t spd_addr_fam15[] = {
	// Socket 0 Node 0 ("Node 0")
	RC00, DIMM0, DIMM2, 0, 0, DIMM1, DIMM3, 0, 0,
};

static void sbxxx_enable_48mhzout(void)
{
	/* most likely programming to 48MHz out signal */
	u32 reg32;
	reg32 = SB_MMIO_MISC32(0x28);
	reg32 &= 0xffc7ffff;
	reg32 |= 0x00100000;
	SB_MMIO_MISC32(0x28) = reg32;

	reg32 = SB_MMIO_MISC32(0x40);
	reg32 &= ~0x80u;
	SB_MMIO_MISC32(0x40) = reg32;
}

static void set_ddr3_voltage(uint8_t node, uint8_t index) {
	uint8_t byte;
	uint8_t value = 0;

	if (index == 0)
		value = 0x0;
	else if (index == 1)
		value = 0x1;
	else if (index == 2)
		value = 0x4;
	else if (index == 3)
		value = 0x5;
	if (node == 1)
		value <<= 1;

	/* Set GPIOs */
	byte = pci_read_config8(PCI_DEV(0, 0x14, 3), 0xd1);
	if (node == 0)
		byte &= ~0x5;
	if (node == 1)
		byte &= ~0xa;
	byte |= value;
	pci_write_config8(PCI_DEV(0, 0x14, 3), 0xd1, byte);

	/* Enable GPIO output drivers */
	byte = pci_read_config8(PCI_DEV(0, 0x14, 3), 0xd0);
	byte &= 0x0f;
	pci_write_config8(PCI_DEV(0, 0x14, 3), 0xd0, byte);
}

void DIMMSetVoltages(struct MCTStatStruc *pMCTstat,
				struct DCTStatStruc *pDCTstatA) {
	u8 dimm;
	u8 socket = 0;
	u8 set_voltage = 0x1;

	set_ddr3_voltage(socket, set_voltage);

	/* Save 1.5V DIMM voltages for MCT and SMBIOS use */
	for (dimm = 0; dimm < MAX_DIMMS_SUPPORTED; dimm++) {
		pDCTstatA->DimmConfiguredVoltage[dimm] = set_voltage;
	}
	/* Allow the DDR supply voltages to settle */
	udelay(100000);
	printk(BIOS_DEBUG, "DIMM voltage set to index %02x\n", set_voltage);
}

unsigned int get_sbdn(unsigned int bus)
{
	device_t dev;

	dev = PCI_DEV(0, 0x14, 0);
	return (dev >> 15) & 0x1f;
}

static void execute_memory_test(void)
{
	/* Test DRAM functionality */
	uint32_t i;
	uint32_t* dataptr;
	printk(BIOS_DEBUG, "Writing test patterns to memory...\n");
	for (i=0; i < 0x1000000; i = i + 8) {
		dataptr = (void *)(0x300000 + i);
		*dataptr = 0x55555555;
		dataptr = (void *)(0x300000 + i + 4);
		*dataptr = 0xaaaaaaaa;
	}
	printk(BIOS_DEBUG, "Done!\n");
	printk(BIOS_DEBUG, "Testing memory...\n");
	uint32_t readback;
	for (i=0; i < 0x1000000; i = i + 8) {
		dataptr = (void *)(0x300000 + i);
		readback = *dataptr;
		if (readback != 0x55555555)
			printk(BIOS_DEBUG, "%p: INCORRECT VALUE %08x (should have been %08x)\n", dataptr, readback, 0x55555555);
		dataptr = (void *)(0x300000 + i + 4);
		readback = *dataptr;
		if (readback != 0xaaaaaaaa)
			printk(BIOS_DEBUG, "%p: INCORRECT VALUE %08x (should have been %08x)\n", dataptr, readback, 0xaaaaaaaa);
	}
	printk(BIOS_DEBUG, "Done!\n");
}

void cache_as_ram_main(unsigned long bist, unsigned long cpu_init_detectedx)
{
	u32 val;
	u8 byte;
	device_t dev;
	uint32_t bsp_apicid = 0;
	msr_t msr;

	struct sys_info *sysinfo = &sysinfo_car;

	/* Limit the maximum HT speed to 2.6GHz to prevent lockups
	* due to HT CPU <--> CPU wiring not being validated to 3.2GHz
	*/
	sysinfo->ht_link_cfg.ht_speed_limit = 2600;

	if (!cpu_init_detectedx && boot_cpu()) {

		//set_bsp_node_CHtExtNodeCfgEn();
		//enumerate_ht_chain();

		/* enable SIO LPC decode */
		dev = PCI_DEV(0, 0x14, 3);
		byte = pci_read_config8(dev, 0x48);
		byte |= 3;		/* 2e, 2f */
		pci_write_config8(dev, 0x48, byte);

		/* enable serial decode */
		byte = pci_read_config8(dev, 0x44);
		byte |= (1 << 6);  /* 0x3f8 */
		pci_write_config8(dev, 0x44, byte);

		sb900_pci_port80();

		post_code(0x30);

                /* enable SB MMIO space */
		outb(0x24, 0xcd6);
		outb(0x1, 0xcd7);

		/* enable SIO clock */
		sbxxx_enable_48mhzout();
		ite_kill_watchdog(GPIO_DEV);
		ite_enable_serial(SERIAL_DEV, CONFIG_TTYS0_BASE);
		ite_enable_3vsbsw(GPIO_DEV);
		console_init();

		printk(BIOS_SPEW, "CONSOLE: Hello\n");

		/* turn on secondary smbus at b20 */
		outb(0x28, 0xcd6);
		byte = inb(0xcd7);
		byte |= 1;
		outb(byte, 0xcd7);
	}

	if (bist == 0)
		bsp_apicid = init_cpus(cpu_init_detectedx, sysinfo);

	post_code(0x34);

	/* Halt if there was a built in self test failure */
	report_bist_failure(bist);

	/* Load MPB */
	val = cpuid_eax(1);
	printk(BIOS_DEBUG, "BSP Family_Model: %08x\n", val);
	printk(BIOS_DEBUG, "cpu_init_detectedx = %08lx\n", cpu_init_detectedx);

	set_sysinfo_in_ram(0);

	//update_microcode(val);

	cpuSetAMDMSR(0);

	/* Pass NULL to this function to skip */
	amd_ht_init(NULL);
	//amd_ht_fixup(sysinfo);

	post_code(0x35);

	finalize_node_setup(sysinfo);
	//setup_mb_resource_map();
	post_code(0x36);

	/* Wait for all the APs core0 started by finalize_node_setup. */
	wait_all_core0_started();

	if (IS_ENABLED(CONFIG_LOGICAL_CPUS)) {
		/* Core0 on each node is configured. Now setup any additional cores. */
		printk(BIOS_DEBUG, "start_other_cores()\n");
		start_other_cores(bsp_apicid);
		post_code(0x37);
		wait_all_other_cores_started(bsp_apicid);
	}

	if (IS_ENABLED(CONFIG_SET_FIDVID)) {
		msr = rdmsr(0xc0010071);
		printk(BIOS_DEBUG, "\nBegin FIDVID MSR 0xc0010071 0x%08x 0x%08x\n", msr.hi, msr.lo);

		/* FIXME: The sb fid change may survive the warm reset and only need to be done once */
		//enable_fid_change_on_sb(sysinfo->sbbusn, sysinfo->sbdn);

		post_code(0x38);

		if (!warm_reset_detect(0)) {	// BSP is node 0
			init_fidvid_bsp(bsp_apicid, sysinfo->nodes);
		} else {
			init_fidvid_stage2(bsp_apicid, 0);	// BSP is node 0
		}

		post_code(0x39);

		/* show final fid and vid */
		msr=rdmsr(0xc0010071);
		printk(BIOS_DEBUG, "End FIDVIDMSR 0xc0010071 0x%08x 0x%08x\n", msr.hi, msr.lo);
	}

	post_code(0x3a);

	set_ddr3_voltage(0, 1);

	post_code(0x3b);

	///* Set up peripheral control lines */
	//set_peripheral_control_lines();
	fill_mem_ctrl(sysinfo->nodes, sysinfo->ctrl, spd_addr_fam15);

	post_code(0x40);
	printk(BIOS_DEBUG, "do raminit...");
	raminit_amdmct(sysinfo);
	printk(BIOS_DEBUG, "done\n");
	post_code(0x41);

	execute_memory_test();

	post_code(0x42);

	cbmem_initialize_empty();

	amdmct_cbmem_store_info(sysinfo);

	post_cache_as_ram();

	post_code(0xee);  /* Should never see this post code. */
}

BOOL AMD_CB_ManualBUIDSwapList (u8 node, u8 link, const u8 **List)
{
	/* Force BUID to 0 */
	static const u8 swaplist[] = { 0xFF, CONFIG_HT_CHAIN_UNITID_BASE,
				CONFIG_HT_CHAIN_END_UNITID_BASE, 0xFF };
	if ((node == 0) && (link == 0)) {       /* BSP SB link */
		*List = swaplist;
		return 1;
	}

	return 0;
}
