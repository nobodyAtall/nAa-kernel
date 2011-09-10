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
#include <linux/timer.h>
#include <asm/atomic.h>
#include <linux/semc/gpio_event.h>
#include "board-delta-keypad.h"
#include <mach/semc_rpc_server_handset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>

#define MIMMI_GPIO_SLIDING_DET 17
#define DBG(X)

static unsigned int keypad_row_gpios[] = { 31, 32, 33, 34, 35, 29, 76 };
static unsigned int keypad_col_gpios[] = { 36, 37, 38, 39, 40, 41, 42 };
static struct timer_list ignore_timer;
static atomic_t key_ignored;
static atomic_t driver_up_and_running = ATOMIC_INIT(0);
static atomic_t keyboard_closed = ATOMIC_INIT(1);
static struct gpio_event_input_devs *kbd_input_dev;

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))

static const unsigned short keypad_keymap_mimmi[ARRAY_SIZE(keypad_col_gpios) *
					  ARRAY_SIZE(keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_ZENKAKUHANKAKU, //Camera auto focus
	[KEYMAP_INDEX(0, 1)] = KEY_CAMERA,
//	[KEYMAP_INDEX(0, 2)] = ,     //Not used
//	[KEYMAP_INDEX(0, 3)] = ,     //Not used
//	[KEYMAP_INDEX(0, 4)] = ,     //Not used
	[KEYMAP_INDEX(0, 5)] = KEY_TAB,
	[KEYMAP_INDEX(0, 6)] = KEY_QUESTION,

	[KEYMAP_INDEX(1, 0)] = KEY_O,
	[KEYMAP_INDEX(1, 1)] = KEY_U,
	[KEYMAP_INDEX(1, 2)] = KEY_Y,
	[KEYMAP_INDEX(1, 3)] = KEY_R,
	[KEYMAP_INDEX(1, 4)] = KEY_E,
	[KEYMAP_INDEX(1, 5)] = KEY_Q,
	[KEYMAP_INDEX(1, 6)] = KEY_LEFTALT,

	[KEYMAP_INDEX(2, 0)] = KEY_P,
	[KEYMAP_INDEX(2, 1)] = KEY_I,
	[KEYMAP_INDEX(2, 2)] = KEY_H,
	[KEYMAP_INDEX(2, 3)] = KEY_T,
	[KEYMAP_INDEX(2, 4)] = KEY_D,
	[KEYMAP_INDEX(2, 5)] = KEY_W,
	[KEYMAP_INDEX(2, 6)] = KEY_RIGHT,

	[KEYMAP_INDEX(3, 0)] = KEY_L,
	[KEYMAP_INDEX(3, 1)] = KEY_J,
	[KEYMAP_INDEX(3, 2)] = KEY_G,
	[KEYMAP_INDEX(3, 3)] = KEY_F,
	[KEYMAP_INDEX(3, 4)] = KEY_S,
	[KEYMAP_INDEX(3, 5)] = KEY_A,
	[KEYMAP_INDEX(3, 6)] = KEY_SPACE,

	[KEYMAP_INDEX(4, 0)] = KEY_HOME,
	[KEYMAP_INDEX(4, 1)] = KEY_K,
	[KEYMAP_INDEX(4, 2)] = KEY_M,
	[KEYMAP_INDEX(4, 3)] = KEY_V,
	[KEYMAP_INDEX(4, 4)] = KEY_C,
	[KEYMAP_INDEX(4, 5)] = KEY_MENU,
	[KEYMAP_INDEX(4, 6)] = KEY_APOSTROPHE,

	[KEYMAP_INDEX(5, 0)] = KEY_ENTER,
	[KEYMAP_INDEX(5, 1)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(5, 2)] = KEY_N,
	[KEYMAP_INDEX(5, 3)] = KEY_B,
	[KEYMAP_INDEX(5, 4)] = KEY_X,
	[KEYMAP_INDEX(5, 5)] = KEY_Z,
	[KEYMAP_INDEX(5, 6)] = KEY_CHAT, //Symbol

	[KEYMAP_INDEX(6, 0)] = KEY_BACK,
	[KEYMAP_INDEX(6, 1)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(6, 2)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(6, 3)] = KEY_COMMA,
	[KEYMAP_INDEX(6, 4)] = KEY_DOT,
	[KEYMAP_INDEX(6, 5)] = KEY_LEFT,
	[KEYMAP_INDEX(6, 6)] = KEY_VOLUMEUP,
};

static const unsigned short button_codes[] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU,
	KEY_VOLUMEDOWN,
	KEY_VOLUMEUP,
	KEY_CAMERA,
	KEY_ZENKAKUHANKAKU, /* CAMERA FOCUS */
};

static const unsigned short
wakeup_inputs_kbd_closed[ARRAY_SIZE(keypad_col_gpios)] = {
	1,  /* KEY_HOME + KEY_BACK */
	0,
	1,  /* KEY_VOLUMEDOWN */
	0,
	0,
	0,
	1,  /* KEY_VOLUMEUP  */
};

static const unsigned short
wakeup_inputs_kbd_opened[ARRAY_SIZE(keypad_col_gpios)] = {
	1,
	1,
	1,
	1,
	1,
	1,
	1,
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

static int is_a_button(unsigned short code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(button_codes); i++)
		if (code == button_codes[i])
			return 1;
	return 0;
}

static int mimmi_matrix_event(struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info,
			void **data, unsigned int dev, unsigned int type,
			unsigned int code, int value)
{
	struct input_dev *input_dev = input_devs->dev[dev];

	DBG(printk(KERN_DEBUG "%s: code %d value %d\n", __func__, code, value);)

	if (atomic_read(&keyboard_closed)) {
		if (is_a_button(code))
			input_report_key(input_dev, code, value);
		else
			DBG(printk(KERN_DEBUG "%s: KBD is closed,"
				" ignoring code %d\n", __func__, code);)
		return 0;
	}

	if (code == KEY_SPACE || code == KEY_TAB) {
		int key_i = atomic_read(&key_ignored);

		if (value) {
			if (key_i == -1) {
				del_timer_sync(&ignore_timer);
				atomic_set(&key_ignored, code == KEY_TAB ?
						KEY_SPACE : KEY_TAB);
			} else if (code == key_i) {
				DBG(printk(KERN_DEBUG "%s: KEY %d keypress"
						" ignored\n", __func__, code));
				return 0;
			}
		} else {
			if (code == key_i) {
				DBG(printk(KERN_DEBUG "%s: KEY %d release"
						" ignored\n", __func__, code));
				return 0;
			}
			mod_timer(&ignore_timer, jiffies +
					msecs_to_jiffies(100));
		}
		DBG(printk(KERN_DEBUG "%s: reporting code %d value %d\n",
			   __func__, code, value);)
	}
	input_report_key(input_dev, code, value);
	return 0;
}

static void timer_callback(unsigned long d)
{
	atomic_set(&key_ignored, -1);
	DBG(printk(KERN_DEBUG "%s: reset key ignoring\n", __func__));
}

static int mimmi_gpio_event_matrix_func(
			struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info,
			void **data, int func)
{
	if (func == GPIO_EVENT_FUNC_INIT) {
		struct input_dev *input_dev = input_devs->dev[0];

		setup_timer(&ignore_timer, timer_callback, 0);
		atomic_set(&key_ignored, -1);
		kbd_input_dev = input_devs;
		input_set_capability(input_dev, EV_KEY, KEY_POWER);
		input_set_capability(input_dev, EV_KEY, KEY_END);
		atomic_set(&driver_up_and_running, 1);

	} else if (func == GPIO_EVENT_FUNC_UNINIT) {
		atomic_set(&driver_up_and_running, 0);
		del_timer_sync(&ignore_timer);
	}

	return semc_gpio_event_matrix_func(input_devs, info, data, func);
}

/* Mimmi keypad platform device information */
static struct gpio_event_matrix_info mimmi_keypad_matrix_info = {
	.info.func	= mimmi_gpio_event_matrix_func,
	.info.event	= mimmi_matrix_event,
	.keymap		= keypad_keymap_mimmi,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 0,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};


static int mimmi_input_event(struct gpio_event_input_devs *input_devs,
		     struct gpio_event_info *info,
		     void **data, unsigned int dev, unsigned int type,
		     unsigned int code, int value)
{
	DBG(printk(KERN_DEBUG "%s: type %d, code %d value %d\n", __func__,
		   type, code, value);)
	atomic_set(&keyboard_closed, value);
	configure_wakeup_inputs(value ? wakeup_inputs_kbd_closed
				: wakeup_inputs_kbd_opened);
	input_event(input_devs->dev[dev], type, code, value);
	return 0;
}

static int mimmi_gpio_event_input_func(
			struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info,
			void **data, int func)
{
	if (func == GPIO_EVENT_FUNC_INIT) {
		int kbd;

		gpio_direction_input(MIMMI_GPIO_SLIDING_DET);
		kbd = !gpio_get_value(MIMMI_GPIO_SLIDING_DET);
		atomic_set(&keyboard_closed, kbd);
		configure_wakeup_inputs(kbd ? wakeup_inputs_kbd_closed
					: wakeup_inputs_kbd_opened);
		DBG(printk(KERN_DEBUG "%s: KBD is %s\n", __func__,
			   kbd ? "closed" : "opened"));
	}
	return semc_gpio_event_input_func(input_devs, info, data, func);
}


static struct gpio_event_direct_entry mimmi_keypad_switch_map[] = {
	{ MIMMI_GPIO_SLIDING_DET,       SW_LID       }
};

static struct gpio_event_input_info mimmi_keypad_switch_info = {
	.info.func = mimmi_gpio_event_input_func,
	.info.event = mimmi_input_event,
	.flags = 0,
	.type = EV_SW,
	.keymap = mimmi_keypad_switch_map,
	.keymap_size = ARRAY_SIZE(mimmi_keypad_switch_map)
};


static struct gpio_event_info *mimmi_keypad_info[] = {
	&mimmi_keypad_matrix_info.info,
	&mimmi_keypad_switch_info.info
};

static struct gpio_event_platform_data mimmi_keypad_data = {
	.name		= "mimmi_keypad",
	.info		= mimmi_keypad_info,
	.info_count	= ARRAY_SIZE(mimmi_keypad_info)
};

struct platform_device keypad_device_mimmi = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &mimmi_keypad_data,
	},
};

struct platform_device *get_keypad_device_delta(void)
{
	return &keypad_device_mimmi;
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
