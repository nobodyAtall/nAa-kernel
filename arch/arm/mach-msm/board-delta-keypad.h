/*
 * Copyright (c) 2008-2009, SonyEricsson. All rights reserved.
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

#ifndef _BOARD_DELTA_KEYPAD_H
#define _BOARD_DELTA_KEYPAD_H

#include <linux/input.h>
#include <linux/platform_device.h>

struct platform_device *get_keypad_device_delta(void);
void keypad_rpc_key_callback(uint32_t key, uint32_t event);
#endif
