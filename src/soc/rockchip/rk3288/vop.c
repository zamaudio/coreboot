/*
 * This file is part of the coreboot project.
 *
 * Copyright 2014 Rockchip Inc.
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

#include <arch/io.h>
#include <console/console.h>
#include <delay.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <soc/addressmap.h>
#include <soc/clock.h>
#include <soc/edp.h>
#include <soc/vop.h>

#include "chip.h"

static struct rk3288_vop_regs * const vop_regs[] = {
	(struct rk3288_vop_regs *)VOP_BIG_BASE,
	(struct rk3288_vop_regs *)VOP_LIT_BASE
};

void rkvop_enable(u32 vop_id, u32 fbbase, const struct edid *edid)
{
	u32 lb_mode;
	u32 rgb_mode;
	u32 hactive = edid->ha;
	u32 vactive = edid->va;
	u32 hsync_len = edid->hspw;
	u32 hback_porch = edid->hbl - edid->hso - edid->hspw;
	u32 vsync_len = edid->vspw;
	u32 vback_porch = edid->vbl - edid->vso - edid->vspw;
	u32 xpos = 0, ypos = 0;
	struct rk3288_vop_regs *preg = vop_regs[vop_id];

	write32(&preg->win0_act_info,
		V_ACT_WIDTH(hactive - 1) | V_ACT_HEIGHT(vactive - 1));

	write32(&preg->win0_dsp_st, V_DSP_XST(xpos + hsync_len + hback_porch) |
				    V_DSP_YST(ypos + vsync_len + vback_porch));

	write32(&preg->win0_dsp_info, V_DSP_WIDTH(hactive - 1) |
				      V_DSP_HEIGHT(vactive - 1));

	clrsetbits_le32(&preg->win0_color_key, M_WIN0_KEY_EN | M_WIN0_KEY_COLOR,
						V_WIN0_KEY_EN(0) |
						V_WIN0_KEY_COLOR(0));

	switch (edid->framebuffer_bits_per_pixel) {
	case 16:
		rgb_mode = RGB565;
		write32(&preg->win0_vir, V_RGB565_VIRWIDTH(hactive));
		break;
	case 24:
		rgb_mode = RGB888;
		write32(&preg->win0_vir, V_RGB888_VIRWIDTH(hactive));
		break;
	case 32:
	default:
		rgb_mode = ARGB8888;
		write32(&preg->win0_vir, V_ARGB888_VIRWIDTH(hactive));
		break;
	}

	if (hactive > 2560)
		lb_mode = LB_RGB_3840X2;
	else if (hactive > 1920)
		lb_mode = LB_RGB_2560X4;
	else if (hactive > 1280)
		lb_mode = LB_RGB_1920X5;
	else
		lb_mode = LB_RGB_1280X8;

	clrsetbits_le32(&preg->win0_ctrl0,
			M_WIN0_LB_MODE | M_WIN0_DATA_FMT | M_WIN0_EN,
			V_WIN0_LB_MODE(lb_mode) |
			V_WIN0_DATA_FMT(rgb_mode) | V_WIN0_EN(1));

	write32(&preg->win0_yrgb_mst, fbbase);

	write32(&preg->reg_cfg_done, 0x01); /* enable reg config */
}

void rkvop_mode_set(u32 vop_id, const struct edid *edid)
{
	u32 hactive = edid->ha;
	u32 vactive = edid->va;
	u32 hfront_porch = edid->hso;
	u32 hsync_len = edid->hspw;
	u32 hback_porch = edid->hbl - edid->hso - edid->hspw;
	u32 vfront_porch = edid->vso;
	u32 vsync_len = edid->vspw;
	u32 vback_porch = edid->vbl - edid->vso - edid->vspw;
	struct rk3288_vop_regs *preg = vop_regs[vop_id];

	clrsetbits_le32(&preg->sys_ctrl, M_ALL_OUT_EN, V_EDP_OUT_EN(1));
	clrsetbits_le32(&preg->dsp_ctrl0, M_DSP_OUT_MODE,
					 V_DSP_OUT_MODE(15));
	write32(&preg->dsp_htotal_hs_end, V_HSYNC(hsync_len) |
		V_HORPRD(hsync_len + hback_porch + hactive + hfront_porch));

	write32(&preg->dsp_hact_st_end,
		V_HEAP(hsync_len + hback_porch + hactive) |
		V_HASP(hsync_len + hback_porch));

	write32(&preg->dsp_vtotal_vs_end, V_VSYNC(vsync_len) |
		V_VERPRD(vsync_len + vback_porch + vactive + vfront_porch));

	write32(&preg->dsp_vact_st_end,
		V_VAEP(vsync_len + vback_porch + vactive) |
		V_VASP(vsync_len + vback_porch));

	write32(&preg->post_dsp_hact_info,
		V_HEAP(hsync_len + hback_porch + hactive) |
		V_HASP(hsync_len + hback_porch));

	write32(&preg->post_dsp_vact_info,
		V_VAEP(vsync_len + vback_porch + vactive) |
		V_VASP(vsync_len + vback_porch));

	write32(&preg->reg_cfg_done, 0x01); /* enable reg config */
}
