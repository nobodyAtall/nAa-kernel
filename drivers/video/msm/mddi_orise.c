/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#define MDDI_ORISE_1_2 1
#define write_client_reg(__X, __Y, __Z) {\
	mddi_queue_register_write(__X, __Y, TRUE, 0);\
}

static int mddi_orise_lcd_on(struct platform_device *pdev);
static int mddi_orise_lcd_off(struct platform_device *pdev);
static int __init mddi_orise_probe(struct platform_device *pdev);
static int __init mddi_orise_init(void);

/* function used to turn on the display */
static void mddi_orise_prim_lcd_init(void)
{
	write_client_reg(0x00110000, 0, TRUE);
	mddi_wait(150);
	write_client_reg(0x00290000, 0, TRUE);
}

static struct platform_driver this_driver = {
	.probe  = mddi_orise_probe,
	.driver = {
		.name   = "mddi_orise",
	},
};

static struct msm_fb_panel_data mddi_orise_panel_data = {
	.on = mddi_orise_lcd_on,
	.off = mddi_orise_lcd_off,
};

static struct platform_device this_device = {
	.name	= "mddi_orise",
	.id	= MDDI_ORISE_1_2,
	.dev	= {
		.platform_data = &mddi_orise_panel_data,
	}
};

static int mddi_orise_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mddi_orise_prim_lcd_init();

	return 0;
}

static int mddi_orise_lcd_off(struct platform_device *pdev)
{
	return 0;
}

static int __init mddi_orise_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
	return 0;
}

static int __init mddi_orise_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	u32 id;
	ret = msm_fb_detect_client("mddi_orise");
	if (ret == -ENODEV)
		return 0;

	if (ret) {
		id = mddi_get_client_id();
		if (((id >> 16) != 0xbe8d) || ((id & 0xffff) != 0x8031))
			return 0;
	}
#endif
	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &mddi_orise_panel_data.panel_info;
		pinfo->xres = 480;
		pinfo->yres = 800;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 18;
		pinfo->fb_num = 2;
		pinfo->clk_rate = 192000000;
		pinfo->clk_min = 192000000;
		pinfo->clk_max = 192000000;
		pinfo->lcd.vsync_enable = FALSE;
		pinfo->lcd.refx100 = 6050;
		pinfo->lcd.v_back_porch = 2;
		pinfo->lcd.v_front_porch = 2;
		pinfo->lcd.v_pulse_width = 105;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;

		ret = platform_device_register(&this_device);
		if (ret)
			platform_driver_unregister(&this_driver);
	}
	return ret;
}
module_init(mddi_orise_init);
