/* drivers/gpio/semc_gpio_extr.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Alexandar Rodzevski <alexandar.rodzevski@sonyericsson.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SEMC_GPIO_EXTR_H_
#define _SEMC_GPIO_EXTR_H_

#define SEMC_GPIO_EXTR_DRV_NAME		"semc_gpio_extractor"
#define SEMC_GPIO_EXTR_NO_SELECT	-1

struct semc_gpio_extr_platform_data {
	u8 *gpio_list;
	u8 list_size;
	int gpio_select;
	int (*init_gpios)(void);
	int (*get_gpio)(u8 gpio);
	int (*set_gpio)(u8 gpio, u8 val);
};

#endif
