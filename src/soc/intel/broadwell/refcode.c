/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2014 Google Inc.
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

#include <string.h>
#include <arch/acpi.h>
#include <cbmem.h>
#include <console/console.h>
#include <console/streams.h>
#include <cpu/x86/tsc.h>
#include <program_loading.h>
#include <rmodule.h>
#include <ramstage_cache.h>
#include <string.h>
#if IS_ENABLED(CONFIG_CHROMEOS)
#include <vendorcode/google/chromeos/vboot_handoff.h>
#endif
#include <soc/pei_data.h>
#include <soc/pei_wrapper.h>
#include <soc/ramstage.h>

static inline struct ramstage_cache *next_cache(struct ramstage_cache *c)
{
	return (struct ramstage_cache *)&c->program[c->size];
}

static pei_wrapper_entry_t load_refcode_from_cache(void)
{
	struct ramstage_cache *c;
	long cache_size;

	printk(BIOS_DEBUG, "refcode loading from cache.\n");

	c = ramstage_cache_location(&cache_size);

	if (!ramstage_cache_is_valid(c)) {
		printk(BIOS_DEBUG, "Invalid ramstage cache descriptor.\n");
		return NULL;
	}

	c = next_cache(c);
	if (!ramstage_cache_is_valid(c)) {
		printk(BIOS_DEBUG, "Invalid refcode cache descriptor.\n");
		return NULL;
	}

	printk(BIOS_DEBUG, "Loading cached reference code from 0x%08x(%x)\n",
	       c->load_address, c->size);
	memcpy((void *)c->load_address, &c->program[0], c->size);

	return (pei_wrapper_entry_t)c->entry_point;
}

static void cache_refcode(const struct rmod_stage_load *rsl)
{
	struct ramstage_cache *c;
	long cache_size;

	c = ramstage_cache_location(&cache_size);

	if (!ramstage_cache_is_valid(c)) {
		printk(BIOS_DEBUG, "No place to cache reference code.\n");
		return;
	}

	/* Determine how much remaining cache available. */
	cache_size -= c->size + sizeof(*c);

	if (cache_size < (sizeof(*c) + prog_size(rsl->prog))) {
		printk(BIOS_DEBUG, "Not enough cache space for ref code.\n");
		return;
	}

	c = next_cache(c);
	c->magic = RAMSTAGE_CACHE_MAGIC;
	c->entry_point = (uint32_t)(uintptr_t)prog_entry(rsl->prog);
	c->load_address = (uint32_t)(uintptr_t)prog_start(rsl->prog);
	c->size = prog_size(rsl->prog);

	printk(BIOS_DEBUG, "Caching refcode at 0x%p(%x)\n",
	       &c->program[0], c->size);
	memcpy(&c->program[0], (void *)c->load_address, c->size);
}

#if IS_ENABLED(CONFIG_CHROMEOS)
static int load_refcode_from_vboot(struct rmod_stage_load *refcode)
{
	struct vboot_handoff *vboot_handoff;
	const struct firmware_component *fwc;
	struct cbfs_stage *stage;

	vboot_handoff = cbmem_find(CBMEM_ID_VBOOT_HANDOFF);
	fwc = &vboot_handoff->components[CONFIG_VBOOT_REFCODE_INDEX];

	if (vboot_handoff == NULL ||
	    vboot_handoff->selected_firmware == VB_SELECT_FIRMWARE_READONLY ||
	    CONFIG_VBOOT_REFCODE_INDEX >= MAX_PARSED_FW_COMPONENTS ||
	    fwc->size == 0 || fwc->address == 0)
		return -1;

	printk(BIOS_DEBUG, "refcode loading from vboot rw area.\n");
	stage = (void *)(uintptr_t)fwc->address;

	if (rmodule_stage_load(refcode, stage)) {
		printk(BIOS_DEBUG, "Error loading reference code.\n");
		return -1;
	}
	return 0;
}
#else
static int load_refcode_from_vboot(struct rmod_stage_load *refcode)
{
	return -1;
}
#endif

static int load_refcode_from_cbfs(struct rmod_stage_load *refcode)
{
	printk(BIOS_DEBUG, "refcode loading from cbfs.\n");

	if (rmodule_stage_load_from_cbfs(refcode)) {
		printk(BIOS_DEBUG, "Error loading reference code.\n");
		return -1;
	}

	return 0;
}

static pei_wrapper_entry_t load_reference_code(void)
{
	struct prog prog = {
		.name = CONFIG_CBFS_PREFIX "/refcode",
	};
	struct rmod_stage_load refcode = {
		.cbmem_id = CBMEM_ID_REFCODE,
		.prog = &prog,
	};

	if (acpi_is_wakeup_s3()) {
		return load_refcode_from_cache();
	}

	if (load_refcode_from_vboot(&refcode) ||
		load_refcode_from_cbfs(&refcode))
			return NULL;

	/* Cache loaded reference code. */
	cache_refcode(&refcode);

	return prog_entry(&prog);
}

void broadwell_run_reference_code(void)
{
	int ret, dummy;
	struct pei_data pei_data;
	pei_wrapper_entry_t entry;

	memset(&pei_data, 0, sizeof(pei_data));
	mainboard_fill_pei_data(&pei_data);
	broadwell_fill_pei_data(&pei_data);

	pei_data.boot_mode = acpi_slp_type;
	pei_data.saved_data = (void *) &dummy;

	entry = load_reference_code();
	if (entry == NULL) {
		printk(BIOS_ERR, "Reference code not found\n");
		return;
	}

	/* Call into reference code. */
	ret = entry(&pei_data);
	if (ret != 0) {
		printk(BIOS_ERR, "Reference code returned %d\n", ret);
		return;
	}
}
