/*
 * MSM PMIC vibrator driver
 *
 *
 * Copyright (C) 2009 SonyEricsson Mobile Communications AB
 * Copyright (C) 2010 SonyEricsson Mobile Communications AB
 *
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 *
 */

#ifndef _MSM_PMIC_VIBRATOR_H_
#define _MSM_PMIC_VIBRATOR_H_

struct msm_pmic_vibrator_platform_data {
	u16 min_voltage;
	u16 max_voltage;
	u16 off_voltage;
	u16 default_voltage;
	u16 mimimal_on_time;
	int (*platform_set_vib_voltage)(u16 volt_mv);
	int (*platform_init_vib_hw)(void);
};

#endif
