/* drivers/misc/semc/seport/seport_attrs.h
 *
 * Portable Headphone (phf) sensing for SEMC custom 3.5mm
 * audio jack with 3 extra pins.
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

#ifndef __SEPORT_ATTRS_H_
#define __SEPORT_ATTRS_H_

#include <linux/semc/seport/seport.h>
#include <linux/device.h>

int seport_attrs_create_attrs(struct device *dev);
void seport_attrs_destroy_attrs(struct device *dev);
void seport_attrs_init(void);
void seport_attrs_parse_button_value(u16 *value, int hp_mode);

int seport_attrs_get_rid_max_vals(struct seport_trim_param_data *pd);
int seport_attrs_get_rid_min_vals(struct seport_trim_param_data *pd);
int seport_attrs_get_cco_max_vals(struct seport_trim_param_data *pd);
int seport_attrs_get_cco_min_vals(struct seport_trim_param_data *pd);
int seport_attrs_get_vad_max_vals(struct seport_trim_param_data *pd);
int seport_attrs_get_vad_min_vals(struct seport_trim_param_data *pd);

#endif
