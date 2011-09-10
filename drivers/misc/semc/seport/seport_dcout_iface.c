/* drivers/misc/semc/seport/seport_dcout_iface.c
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

/*
 * This file exists to abstract handling of DCout.
 * We may have several solutions for handling DCout.
 * The MSM7627 platform will use a BQ chip. Others
 * may use something else.
 *
 * In order for this driver to handle all variants,
 * the handling of DCout has been abstracted so we only
 * need to modify one file if charging/DCout solution
 * changes.
 */

#include <linux/module.h>
#include <linux/semc/seport/seport_dcout_iface.h>
#include <linux/semc/seport/seport.h>

#ifdef CONFIG_SEMC_POWER_BQ24180
#include <linux/semc/power/semc_power.h>
#endif

#define SEPORT_DCOUT_INITIAL_STATE 0

/** Global variables needed for all solutions **/
static int seport_high_power_mode = SEPORT_DCOUT_INITIAL_STATE;
static int seport_dcout_status = SEPORT_DCOUT_INITIAL_STATE;

int seport_set_dcout(u8 enable)
{
#ifdef CONFIG_SEMC_POWER
	int retval = semc_power_dcio(enable, 1000);

	if (retval) {
		printk(KERN_ERR "Failed to change DC-out state\n");
		return -ERROR_HWIF_DCOUT_FAILED;
	}
#endif
	seport_dcout_status = enable;

	return retval;
}

int seport_get_dcout_status(void)
{
	return seport_dcout_status;
}

void seport_enable_high_power(int enable)
{
	seport_high_power_mode = enable;
}

int seport_get_high_power_mode(void)
{
	return seport_high_power_mode;
}

void seport_register_overcurrent_callback(void (*func)(u8))
{
#ifdef CONFIG_SEMC_POWER
	semc_power_register_dcio_over_current_callback(func);
#endif
}
