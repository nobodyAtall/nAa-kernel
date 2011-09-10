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

#include <linux/max17040.h>

#if defined(CONFIG_MACH_ES209RA)
struct max17040_device_data max17040_dev_data = {
	.model = {
		{
			(u8)0x8A, (u8)0xF0, (u8)0xB6, (u8)0x50,
			(u8)0xB7, (u8)0xA0, (u8)0xB9, (u8)0xF0,
			(u8)0xBB, (u8)0xA0, (u8)0xBC, (u8)0x80,
			(u8)0xBC, (u8)0xE0, (u8)0xBD, (u8)0x40
		},
		{
			(u8)0xBD, (u8)0xA0, (u8)0xBE, (u8)0x90,
			(u8)0xBF, (u8)0x90, (u8)0xC0, (u8)0xE0,
			(u8)0xC1, (u8)0x50, (u8)0xC6, (u8)0x30,
			(u8)0xC7, (u8)0x40, (u8)0xCA, (u8)0xC0
		},
		{
			(u8)0x00, (u8)0x40, (u8)0x34, (u8)0x40,
			(u8)0x1A, (u8)0x00, (u8)0x1E, (u8)0x00,
			(u8)0x4C, (u8)0xC0, (u8)0x78, (u8)0x20,
			(u8)0x47, (u8)0xA0, (u8)0x80, (u8)0x00
		},
		{
			(u8)0x1A, (u8)0xE0, (u8)0x2A, (u8)0x40,
			(u8)0x23, (u8)0x40, (u8)0x14, (u8)0x60,
			(u8)0x1B, (u8)0x80, (u8)0x2D, (u8)0xA0,
			(u8)0x0E, (u8)0xC0, (u8)0x0E, (u8)0xC0
		},
	},
	.lock		= { (u8)0x00, (u8)0x00 },
	.unlock		= { (u8)0x4A, (u8)0x57 },
	.greatest_ocv	= { (u8)0xD4, (u8)0xC0 },
	.load_result = {
		.max		= 0xDD,
		.min		= 0xDB,
	},
	.rcomp = {
		.init		= 88,
		.temp_co_hot	= -155,
		.temp_co_cold	= -355,
		.temp_div	= 100,
	},
	.voltage = {
		.max		= 5000,
		.over_voltage	= 4260,
		.dead		= 3300,
	},
};
#endif
