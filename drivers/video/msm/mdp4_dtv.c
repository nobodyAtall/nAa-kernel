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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_qos_params.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "msm_fb.h"

static int dtv_probe(struct platform_device *pdev);
static int dtv_remove(struct platform_device *pdev);

static int dtv_off(struct platform_device *pdev);
static int dtv_on(struct platform_device *pdev);

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;

static struct clk *tv_src_clk;
static struct clk *tv_enc_clk;
static struct clk *tv_dac_clk;
static struct clk *hdmi_clk;

static struct platform_driver dtv_driver = {
	.probe = dtv_probe,
	.remove = dtv_remove,
	.suspend = NULL,
	.suspend_late = NULL,
	.resume_early = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "dtv",
		   },
};

static struct lcdc_platform_data *dtv_pdata;

static int dtv_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = panel_next_off(pdev);

	clk_disable(tv_enc_clk);
	clk_disable(tv_dac_clk);

	if (dtv_pdata && dtv_pdata->lcdc_power_save)
		dtv_pdata->lcdc_power_save(0);

	if (dtv_pdata && dtv_pdata->lcdc_gpio_config)
		ret = dtv_pdata->lcdc_gpio_config(0);

	pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "dtv",
					PM_QOS_DEFAULT_VALUE);

	return ret;
}

static int dtv_on(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_data_type *mfd;
	unsigned long panel_pixclock_freq , pm_qos_freq;

	mfd = platform_get_drvdata(pdev);
	panel_pixclock_freq = mfd->fbi->var.pixclock;

	if (panel_pixclock_freq > 58000000)
		/* pm_qos_freq should be in Khz */
		pm_qos_freq = panel_pixclock_freq / 1000 ;
	else
		pm_qos_freq = 58000;

	pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ , "dtv",
						pm_qos_freq);
	mfd = platform_get_drvdata(pdev);

	clk_enable(tv_enc_clk);
	clk_enable(tv_dac_clk);
	clk_enable(hdmi_clk);

	if (dtv_pdata && dtv_pdata->lcdc_power_save)
		dtv_pdata->lcdc_power_save(1);
	if (dtv_pdata && dtv_pdata->lcdc_gpio_config)
		ret = dtv_pdata->lcdc_gpio_config(1);

	clk_set_rate(tv_src_clk, mfd->fbi->var.pixclock);

	ret = panel_next_on(pdev);
	return ret;
}

static int dtv_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;

	if (pdev->id == 0) {
		dtv_pdata = pdev->dev.platform_data;
		if (dtv_pdata && dtv_pdata->lcdc_power_save)
			dtv_pdata->lcdc_power_save(1);
		return 0;
	}

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_LCDC;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		printk(KERN_ERR "dtv_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = (struct msm_fb_panel_data *)mdp_dev->dev.platform_data;
	pdata->on = dtv_on;
	pdata->off = dtv_off;
	pdata->next = pdev;

	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;
	mfd->fb_imgType = MDP_RGB_565;

	fbi = mfd->fbi;
	fbi->var.pixclock = mfd->panel_info.clk_rate;
	fbi->var.left_margin = mfd->panel_info.lcdc.h_back_porch;
	fbi->var.right_margin = mfd->panel_info.lcdc.h_front_porch;
	fbi->var.upper_margin = mfd->panel_info.lcdc.v_back_porch;
	fbi->var.lower_margin = mfd->panel_info.lcdc.v_front_porch;
	fbi->var.hsync_len = mfd->panel_info.lcdc.h_pulse_width;
	fbi->var.vsync_len = mfd->panel_info.lcdc.v_pulse_width;

	/*
	 * set driver data
	 */
	platform_set_drvdata(mdp_dev, mfd);

	/*
	 * register in mdp driver
	 */
	rc = platform_device_add(mdp_dev);
	if (rc)
		goto dtv_probe_err;

	pdev_list[pdev_list_cnt++] = pdev;
		return 0;

dtv_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

static int dtv_remove(struct platform_device *pdev)
{
	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ , "dtv");
	return 0;
}

static int dtv_register_driver(void)
{
	return platform_driver_register(&dtv_driver);
}

static int __init dtv_driver_init(void)
{
	tv_enc_clk = clk_get(NULL, "tv_enc_clk");
	if (IS_ERR(tv_enc_clk)) {
		printk(KERN_ERR "error: can't get tv_enc_clk!\n");
		return IS_ERR(tv_enc_clk);
	}

	tv_dac_clk = clk_get(NULL, "tv_dac_clk");
	if (IS_ERR(tv_dac_clk)) {
		printk(KERN_ERR "error: can't get tv_dac_clk!\n");
		return IS_ERR(tv_dac_clk);
	}

	tv_src_clk = clk_get(NULL, "tv_src_clk");
	if (IS_ERR(tv_src_clk))
		tv_src_clk = tv_dac_clk; /* Fallback to slave */

	hdmi_clk = clk_get(NULL, "hdmi_clk");
	if (IS_ERR(hdmi_clk)) {
		printk(KERN_ERR "error: can't get hdmi_clk!\n");
		return IS_ERR(hdmi_clk);
	}

	pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ , "dtv",
				PM_QOS_DEFAULT_VALUE);

	return dtv_register_driver();
}

module_init(dtv_driver_init);
