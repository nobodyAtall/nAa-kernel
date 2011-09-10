/*
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#ifndef CAMSENSOR_DLT002_CAM_DEVDRV
#define CAMSENSOR_DLT002_CAM_DEVDRV

#define DLT002_DEFAULT_CLOCK_RATE  19200000
#define DLT002_I2C_WRITE_FOOTPRINT 2
#define DLT002_I2C_MAX_BYTES       60

#define TRUE  1
#define FALSE 0
#define boolean uint8_t

#define HIGH_HIGH_BIT_32  0xFF000000
#define HIGH_LOW_BIT_32   0x00FF0000
#define LOW_HIGH_BIT_32   0x0000FF00
#define LOW_LOW_BIT_32    0x000000FF
#define LOW_BIT_32        0x0000FFFF
#define HIGH_BIT_32       0xFFFF0000

#define BYTE_1  1
#define BYTE_2  2
#define BYTE_4  4

#define SHIFT_8  8
#define SHIFT_16 16
#define SHIFT_24 24

#define DLT002_USERCTRL_0000	0x0000
#define DLT002_USERCTRL_0004	0x0004
#define DLT002_USERCTRL_0010	0x0010
#define DLT002_USERCTRL_022C	0x022C
#define DLT002_USERCTRL_0238	0x0238
#define DLT002_USERCTRL_0230	0x0230
#define DLT002_USERCTRL_023C	0x023C

#define DLT002_MSTS_MON_VAL 0x00

#define DLT002_V_FLAG_OTP_MASK        0x00000010
#define DLT002_VENDOR_ID_OTP_MASK     0x00008000

#define DLT002_POLLING_SLEEP_MSEC     20
#define DLT002_POLLING_TIMEOUT_MSEC 1000

#define DLT002_BSTS_VAL 0x00

enum dlt002_vendor {
	VENDOR_ID_0,
	VENDOR_ID_1
};

enum dlt002_otp {
	OTP_0 = 0,
	OTP_1,
	OTP_NO_DATA_WRITTEN
};

enum camera_devmode_type {
	CAMERA_MODE_STANDBY,
	CAMERA_MODE_MONITOR,
	CAMERA_MODE_CAPTURE,
};

struct dlt002_work {
	struct work_struct work;
};

struct dlt002_ctrl {
	int8_t opened;
	struct msm_camera_sensor_info *sensordata;
	struct dlt002_work *sensorw;
	struct i2c_client *client;
	struct msm_sensor_resp *resp;

	uint16_t camera_revision;
	enum dlt002_vendor vendor_id;

	int8_t init_complete;

	enum camera_devmode_type dev_mode;
	enum camera_scene scene;

};
#endif				/* CAMSENSOR_DLT002_CAM_DEVDRV */
