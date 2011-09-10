/*
 * MSM PMIC LED driver
 *
 * Copyright (C) 2010 SonyEricsson Mobile Communications AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 *
 */
#ifndef _LEDS_MSM_PMIC_MISC_H
#define _LEDS_MSM_PMIC_MISC_H

#include <linux/platform_device.h>
#include <mach/pmic.h>

#define MSM_PMIC_MISC_DRV_NAME "msm_pmic_misc_led"

union pmic_led_id {
	enum ledtype led_id;
	enum mpp_which mpp;
};

struct msm_pmic_misc_led_platform_data {
	int (*currnet_to_brightness)(int current_ma);
	int (*set_brightness)(struct msm_pmic_misc_led_platform_data *pdata,
						  u8 brightness);
	char *name;
	union pmic_led_id id;
};

#endif

