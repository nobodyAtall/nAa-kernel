/*
 * MSM PMIC Flash Led driverller
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */
#ifndef _LEDS_MSM_PMIC_FLASHLED_H
#define _LEDS_MSM_PMIC_FLASHLED_H

#include <mach/pmic.h>

struct msm_pmic_flashled_platform_data {
	int (*set_boost_voltage)(struct platform_device *pdev, u16 mv);
	int (*enable_boost_voltage)(struct platform_device *pdev, int enable);
	u16 def_camflash_boost_v;
	u16 def_spotlight_boost_v;
	u16 def_camflash_current;
	u16 def_spotlight_curtent;
	u16 enable_time_limit;
	int sync_enable;
	enum mpp_which sync_mpp;
	enum mpp_dlogic_level sync_level;
	enum flash_led_pol sync_polarity;
	enum flash_led_mode flash_ctl_dbus;
	enum mpp_dlogic_in_dbus sync_in_dbus;
};
#endif

