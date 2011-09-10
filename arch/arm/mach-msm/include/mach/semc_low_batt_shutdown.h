/* arch/arm/mach-msm/include/mach/semc_low_batt_shutdown.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB. All Rights Reserved.
 *
 * Author: Yukito Naganuma <Yukito.X.Naganuma@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
*/

#ifndef _SEMC_LOW_BATT_SHUTDOWN_H_
#define _SEMC_LOW_BATT_SHUTDOWN_H_

struct lbs_platform_data {
	int threshold_vol;
};

int lbs_register_cb_func(void (*lbs_callback_func)(void));

#endif /* _SEMC_LOW_BATT_SHUTDOWN_H_ */

