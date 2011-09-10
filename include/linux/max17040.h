/*

   Copyright (C) 2010 Sony Ericsson Mobile Communications Japan, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License, version 2, as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/
#ifndef __MAX17040_H
#define __MAX17040_H

#include <linux/types.h>

struct max17040_load_result {
	u8	max;
	u8	min;
};

struct max17040_rcomp_data {
	int	init;
	int	temp_co_hot;
	int	temp_co_cold;
	int	temp_div;
};

struct max17040_voltage_value {
	int	max;		/* mV */
	int	over_voltage;	/* mV */
	int	dead;		/* mV */
};

struct max17040_device_data {
	u8				model[4][16];
	u8				lock[2];
	u8				unlock[2];
	u8				greatest_ocv[2];
	struct max17040_load_result	load_result;
	struct max17040_rcomp_data	rcomp;
	struct max17040_voltage_value	voltage;
};


struct max17040_i2c_platform_data {
	struct max17040_device_data *data;
};

#endif /* __MAX17040_H */
