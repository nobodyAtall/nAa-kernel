/* include/linux/semc/seport/seport_dcout_iface.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Joakim Holst <joakim.holst@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SEPORT_DCOUT_IFACE_H_
#define _SEPORT_DCOUT_IFACE_H_

#include <linux/types.h>

int seport_set_dcout(u8 enable);
int seport_get_dcout_status(void);
void seport_enable_high_power(int enable);
int seport_get_high_power_mode(void);
void seport_register_overcurrent_callback(void (*func)(u8));

#endif /* _SEPORT_DCOUT_IFACE_H_ */
