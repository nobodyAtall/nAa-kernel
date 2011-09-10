/* include/linux/semc/seport/seport.h
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

#ifndef _SEMC_SEPORT_H_
#define _SEMC_SEPORT_H_

#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <time.h> /* struct tm */
#endif

/*
 * Define the number of values regarding
 * RID and 3.5mm detection here. If everyone
 * uses these parameters for this purpose,
 * we will always have up to date length
 * of these parameters.
 */

enum seport_rid_values {
	RID_VAL_0,
	RID_VAL_1,
	RID_VAL_2,
	RID_VAL_3,
	RID_VAL_4,
	RID_VAL_5,
	RID_VAL_6,
	RID_VAL_7,
	RID_VAL_8,
	RID_VAL_9,
	RID_VAL_10,
	RID_VAL_11,
	RID_VAL_12,
	RID_VAL_13,
	RID_VAL_14,
	RID_VAL_15,
	RID_VAL_16,
	RID_VAL_17,
	RID_NONE_VAL, /*TODO: Rename me when last two entries are removed */
	RID_NUM_VALS,
	RID_VAL_NONE = 50,
	RID_VAL_ERROR = 100,
};


enum seport_button_values {
	BUTTON_VAL_0,
	BUTTON_VAL_1,
	BUTTON_VAL_2,
	BUTTON_VAL_3,
	BUTTON_VAL_4,
	BUTTON_VAL_5,
	BUTTON_VAL_6,
	BUTTON_VAL_7,
	BUTTON_VAL_8,
	BUTTON_VAL_9,
	BUTTON_VAL_10,
	BUTTON_VAL_11,
	BUTTON_VAL_12,
	BUTTON_VAL_13,
	BUTTON_VAL_14,
	BUTTON_VAL_15,
	BUTTON_VAL_16,
	BUTTON_VAL_17,
	BUTTON_VAL_18,
	BUTTON_VAD = 30,
	BUTTON_VAL_NONE = 50,
	BUTTON_VAL_ERROR = 100,
};

enum seport_cco_types {
	CCO_HEADPHONE,
	CCO_TV_OUT,
	CCO_OMTP_HEADSET,
	CCO_HEADSET,
	CCO_NONE_VAL,
	CCO_NUM_VALS,
	CCO_UNDEFINED = 50,
	CCO_ERROR = 100,
};


struct seport_connection_event {
	struct timespec tm;
	int plug_status;
	enum seport_rid_values rid_value;
	enum seport_cco_types cco_type;
	int dcout;
};

struct seport_button_event {
	struct timespec tm;
	int pressed;
	enum seport_button_values button;
	int high_power;
};

enum seport_error_values {
	ERROR_NO_ERROR = 0, /* No error */
	ERROR_SHUT_DOWN, /* Driver is going down. Used to release clients. */
	ERROR_DCOUT_REQUIRED, /* DCOUT Not enabled for accessory. */
	/* Accessory is not an AIN compatible accessory */
	ERROR_NO_AIN_ACCESSORY,
	/* Too much current drawn over DC-out. DCout terminated. */
	ERROR_DCOUT_OVERLOAD,
	ERROR_BDTN_MALLOC_FAILED,
	ERROR_BDTN_INT_REQ_FAILED,
	ERROR_HWIF_RID_FAILED,
	ERROR_HWIF_BTN_READ_FAILED,
	ERROR_HWIF_DCOUT_FAILED,
	ERROR_HWIF_PHF_VAL_FAILED,
	ERROR_PDET_ALREADY_OPEN,
	ERROR_PDET_OPEN_FAILED,
};

struct seport_trim_param_data {
	short *values;
	int value_length;
};

/* Names for the different devices. Defined once and this is here! */
/* This device is used for IOCTL commands to and from the driver.
 *  This device is also used for retrieveing error codes from the driver */
#define CONTROL_DEVICE_NAME       "seport_control"

/* This device is used for plug detection events from the driver */
/* IOCTL is not supported on this device */
#define PLUG_DETECT_DEVICE_NAME   "seport_plg_det"

/* This device is used for button detection events. */
/* IOCTL is not supported on this device. */
#define BUTTON_DETECT_DEVICE_NAME "seport_btn_det"

/* IOCTL Magic */
#define SEPORT_IOC_MAGIC  'S'

/* IOCTL commands
 */

/* Get RID value. Value returned as return value. */
#define SEPORT_IOCQRIDVAL   _IO(SEPORT_IOC_MAGIC,  0)
/* Get detect status. Returns 0/1 or -2 if detection is not enabled. */
#define SEPORT_IOCQDETSTAT  _IO(SEPORT_IOC_MAGIC,  1)
/* Get analog value from MIC (accessory detection) */
#define SEPORT_IOCGPHFVAL   _IO(SEPORT_IOC_MAGIC,  2)
/* Enable/Disable polling of VAD button (Mic ADC) */
#define SEPORT_IOCSPHFPOLL  _IOW(SEPORT_IOC_MAGIC,  3, int)
/* Enable/Disable detection of buttons from AIN. */
#define SEPORT_IOCSBDTCT    _IOW(SEPORT_IOC_MAGIC,  4, int)
/* Enable/Disable Video out switch.*/
#define SEPORT_IOCSVIDOUT   _IOW(SEPORT_IOC_MAGIC,  5, int)
/* Enable DCOut */
#define SEPORT_IOCSDCOUT   _IOW(SEPORT_IOC_MAGIC,  6, int)
/* Enable/Disable DCOut High power. */
#define SEPORT_IOCSDCOHPWR   _IOW(SEPORT_IOC_MAGIC, 7, int)
/* Set poll delay */
#define SEPORT_IOCSPOLLDEL   _IOW(SEPORT_IOC_MAGIC, 8, long)
/* Get currently set poll delay */
#define SEPORT_IOCQPOLLDEL   _IO(SEPORT_IOC_MAGIC, 9)
/* Reset RID readings. This should not be useful! */
#define SEPORT_IOCTRIDREST   _IO(SEPORT_IOC_MAGIC, 10)

/* Retrieve rid max values */
#define SEPORT_IOCRRIDMAX   _IOR(SEPORT_IOC_MAGIC, 11, \
				 struct seport_trim_param_data*)
/* Retrieve rid min values */
#define SEPORT_IOCRRIDMIN   _IOR(SEPORT_IOC_MAGIC, 12, \
				 struct seport_trim_param_data*)
/* Retrieve cco max values */
#define SEPORT_IOCRCCOMAX   _IOR(SEPORT_IOC_MAGIC, 13, \
				 struct seport_trim_param_data*)
/* Retrieve cco min values */
#define SEPORT_IOCRCCOMIN   _IOR(SEPORT_IOC_MAGIC, 14, \
				 struct seport_trim_param_data*)
/* Enable/Disable MIC Bias */
#define SEPORT_IOCSMICBIAS   _IOW(SEPORT_IOC_MAGIC,  15, uint)

/* Enable/Disable Headphone AMP */
#define SEPORT_IOCSHPAMP   _IOW(SEPORT_IOC_MAGIC,  16, uint)

/* Enable/Disable MIC Bias measurement */
#define SEPORT_IOCSMBMEAS   _IOW(SEPORT_IOC_MAGIC,  17, uint)

#endif /* _SEMC_SEPORT_H_ */
