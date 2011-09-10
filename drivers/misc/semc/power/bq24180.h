/* drivers/misc/semc/power/bq24180.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Imre Sunyi <imre.sunyi@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _BQ24180_H_
#define _BQ24180_H_

#include <linux/semc/power/semc_power.h>

int setup_hw(struct power_ops *hw_ops, event_callback_t fn);
int teardown_hw(void);

#endif /* _BQ24180_H_ */
