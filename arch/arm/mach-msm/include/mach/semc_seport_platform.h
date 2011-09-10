/* arch/arm/mach-msm/include/mach/semc_seport_platform.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SEPORT_PLATFORM_H_
#define _SEPORT_PLATFORM_H_


/**
 * The name here and in board-*.c must be the same.
 * use a define in this file since it's included in
 * board-*.c */
#define SEPORT_DRIVER_NAME "seport-plf-drv"

#include <asm/atomic.h>
#include <linux/workqueue.h>

struct seport_plug_detect_data {
	atomic_t status;
	struct work_struct local_work;
	u8 first_run;
};

struct seport_config {
	int headset_detect_enable_pin;
	int plug_detect_read_pin;
	int button_detect_pin;
	int rid_ping_enable_pin;
#if defined(CONFIG_SEPORT_VIDEO_OUT)
	int video_out_switch;
#endif

	int (*initialize)(struct seport_config *);
};

int seport_platform_get_phf_value(u16 *phf_val);
int seport_platfrom_get_rid_value(u16 *ridVal);
int seport_platform_register_button_gpio_callback(int (*func)
						  (int, void *arg),
						  void *data);
int seport_platform_get_button_id(u16 *button_id);
void seport_platform_unregister_button_gpio_callback(void);
int seport_platform_register_plug_detect_gpio_callback(int (*func)
						       (int, void *arg),
						       void *data);

void seport_platform_unregister_plug_detect_gpio_callback(void *data);

#if defined(CONFIG_SEPORT_VIDEO_OUT)
int seport_platform_enable_video_out_switch(int enable);
#endif /* CONFIG_SEPORT_VIDEO_OUT */

int seport_platform_read_detect_pin_value(void);
void seport_platform_disable_button_detect_interrupt(void);
void seport_platform_enable_button_detect_interrupt(void);
void seport_platform_disable_detect_interrupt(void);
void seport_platform_enable_detect_interrupt(void);

int seport_platform_register_vad_button_callback(
	int (*func)(int, void *arg), void *data);
void seport_platform_unregister_vad_button_callback(void);
int seport_platform_enable_hp_amp(u8 enable);
int seport_platform_enable_mic_bias(u8 enable);
int seport_platform_enable_mic_bias_measurement(u8 enable);

int seport_platform_get_hssd_threshold(u8 max_val, int *value);
int seport_platform_set_hssd_threshold(u8 max, int value);

void seport_platform_vad_callback(uint32_t key, uint32_t event);

#endif /* _SEPORT_PLATFORM_H_ */
