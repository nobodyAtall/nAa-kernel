/*
 * Copyright (C) 2009 SEMC
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Daniel Nygren <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include <linux/semc/gpio_event.h>
#include "board-delta-keypad.h"
#include <mach/semc_rpc_server_handset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>

#define DBG(X)

static unsigned int keypad_row_gpios[] = { 31 };
static unsigned int keypad_col_gpios[] = { 36, 37, 38, 39, 40, 41, 42 };
static atomic_t driver_up_and_running = ATOMIC_INIT(0);
static struct gpio_event_input_devs *kbd_input_dev;

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))

static const unsigned short keypad_keymap_robyn[ARRAY_SIZE(keypad_col_gpios) *
					  ARRAY_SIZE(keypad_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_BACK,		/* right */
	[KEYMAP_INDEX(0, 1)] = KEY_HOME,		/* Middle */
	[KEYMAP_INDEX(0, 2)] = KEY_MENU,		/* left */
	[KEYMAP_INDEX(0, 3)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 4)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(0, 5)] = KEY_CAMERA,		/* Camera Snap */
	[KEYMAP_INDEX(0, 6)] = KEY_EMAIL,		/* Camera AF */
};

static const unsigned short wakeup_inputs[ARRAY_SIZE(keypad_col_gpios)] = {
	0,
	1,  /* KEY_HOME */
	0,
	1,  /* KEY_VOLUMEUP */
	1,  /* KEY_VOLUMEDOWN  */
	0,
	0,
};

static unsigned short w_enabled_inputs[ARRAY_SIZE(keypad_col_gpios)];

static void configure_wakeup_inputs(const unsigned short *wi)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(keypad_col_gpios); i++) {
		if (w_enabled_inputs[i] == wi[i])
			continue;
		if (set_irq_wake(gpio_to_irq(keypad_col_gpios[i]), wi[i])) {
			printk(KERN_ERR "%s: set_irq_wake failed for GPIO %d\n",
					__func__, keypad_col_gpios[i]);
		} else {
			DBG(printk(KERN_DEBUG
				   "%s: set_irq_wake=%d  for GPIO %d\n",
				   __func__, wi[i],  keypad_col_gpios[i]);)
			w_enabled_inputs[i] = wi[i];
		}
	}
}

static int robyn_gpio_event_matrix_func(
			struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info,
			void **data, int func)
{
	if (func == GPIO_EVENT_FUNC_INIT) {
		struct input_dev *input_dev = input_devs->dev[0];

		kbd_input_dev = input_devs;
		input_set_capability(input_dev, EV_KEY, KEY_POWER);
		input_set_capability(input_dev, EV_KEY, KEY_END);
		atomic_set(&driver_up_and_running, 1);
	}
	if (func == GPIO_EVENT_FUNC_UNINIT)
		atomic_set(&driver_up_and_running, 0);

	configure_wakeup_inputs(wakeup_inputs);
	return semc_gpio_event_matrix_func(input_devs, info, data, func);
}

/* Robyn keypad platform device information */
static struct gpio_event_matrix_info robyn_keypad_matrix_info = {
	.info.func	= robyn_gpio_event_matrix_func,
	.keymap		= keypad_keymap_robyn,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 0,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *robyn_keypad_info[] = {
	&robyn_keypad_matrix_info.info
};

static struct gpio_event_platform_data robyn_keypad_data = {
	.name		= "robyn_keypad",
	.info		= robyn_keypad_info,
	.info_count	= ARRAY_SIZE(robyn_keypad_info)
};

struct platform_device keypad_device_robyn = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &robyn_keypad_data,
	},
};

struct platform_device *get_keypad_device_delta(void)
{
	return &keypad_device_robyn;
}

void keypad_rpc_key_callback(uint32_t key, uint32_t event)
{
	if (!atomic_read(&driver_up_and_running)) {
		DBG(printk(KERN_DEBUG "%s: Power key ignored, driver"
				" not ready\n", __func__));
		return;
	}

	switch (key) {
	case HS_PWR_K:
		key = KEY_POWER;
		break;
	case HS_END_K:
		key = KEY_END;
		break;
	default:
		return;
	}
	DBG(printk(KERN_DEBUG "%s: reporting key (code %d, value %d)\n",
			__func__, key, event != HS_REL_K);)
	input_report_key(kbd_input_dev->dev[0], key, event != HS_REL_K);
}
