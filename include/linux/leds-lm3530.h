/*
 * LM3530.h - data structure for LM3530 led controller
 *
 * Copyright (C) 2009 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */
#ifndef __LINUX_LM3530_H
#define __LINUX_LM3530_H

#include <linux/types.h>

enum lm353_als_input {
	ALS_INPUT_1,
	ALS_INPUT_2,
	ALS_INPUT_BOTH,
};

struct lm3530_platform_data {
	int (*power_up)(int enable);
	int zone_irq_gpio;
	int hw_enable_gpio;
	enum lm353_als_input als_input;
	int pwm_polarity:1;

	void *chip_data;
	void (*set_backlight)(void *chip_data, u8 brightness);
	int (*get_backlight)(void *chip_data, u8 *brightness);
};

#endif /* __LINUX_LM3530_H */

