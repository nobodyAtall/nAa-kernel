/* drivers/misc/semc/seport/seport_attrs.c
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

/*
 * This file is mostly used to store and handle configurable
 * parameters that have to do with detection and button handling.
 *
 */

#include <linux/string.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/kernel.h>  /* strtol() */
#include <linux/ctype.h>   /* isdigit() */
#include <linux/semc/seport/seport.h>
#include <mach/semc_seport_platform.h>

#include "seport_attrs.h"

#define SEPORT_VAD_MIN_VAL_NAME "vad_min_val"
#define SEPORT_VAD_MAX_VAL_NAME "vad_max_val"

#define HSSD_MAX_VAL 1
#define HSSD_MIN_VAL 0

static struct mutex seport_attr_rw_mutex;

short seport_rid_max[] = {
	5,     /* RID_0  */
	17,    /* RID_1  */
	28,    /* RID_2  */
	42,    /* RID_3  */
	56,    /* RID_4  */
	69,    /* RID_5  */
	83,    /* RID_6  */
	100,   /* RID_7  */
	118,   /* RID_8  */
	136,   /* RID_9  */
	152,   /* RID_10 */
	167,   /* RID_11 */
	183,   /* RID_12 */
	199,   /* RID_13 */
	212,   /* RID_14 */
	225,   /* RID_15 */
	237,   /* RID_16 */
	249,   /* RID_17 */
	255,   /* NO_RID */
};

short seport_rid_min[] = {
	0,     /* RID_0  */
	6,     /* RID_1  */
	18,    /* RID_2  */
	29,    /* RID_3  */
	43,    /* RID_4  */
	57,    /* RID_5  */
	70,    /* RID_6  */
	84,    /* RID_7  */
	101,   /* RID_8  */
	119,   /* RID_9  */
	137,   /* RID_10 */
	153,   /* RID_11 */
	168,   /* RID_12 */
	184,   /* RID_13 */
	200,   /* RID_14 */
	213,   /* RID_15 */
	226,   /* RID_16 */
	238,   /* RID_17 */
	250,   /* NO_RID */
};

short seport_ain_low_max[] = {
	7,     /* AIN_0  */
	21,    /* AIN_1  */
	35,    /* AIN_2  */
	50,    /* AIN_3  */
	64,    /* AIN_4  */
	79,    /* AIN_5  */
	93,    /* AIN_6  */
	108,   /* AIN_7  */
	123,   /* AIN_8  */
	139,   /* AIN_9  */
	153,   /* AIN_10 */
	167,   /* AIN_11 */
	181,   /* AIN_12 */
	195,   /* AIN_13 */
	207,   /* AIN_14 */
	222,   /* AIN_15 */
	234,   /* AIN_16 */
	246,   /* AIN_17 */
	255,   /* NO_AIN */
};

short seport_ain_low_min[] = {
	0,    /* AIN_0  */
	8,    /* AIN_1  */
	22,   /* AIN_2  */
	36,   /* AIN_3  */
	51,   /* AIN_4  */
	65,   /* AIN_5  */
	80,   /* AIN_6  */
	94,   /* AIN_7  */
	109,  /* AIN_8  */
	124,  /* AIN_9  */
	140,  /* AIN_10 */
	154,  /* AIN_11 */
	168,  /* AIN_12 */
	182,  /* AIN_13 */
	196,  /* AIN_14 */
	208,  /* AIN_15 */
	223,  /* AIN_16 */
	235,  /* AIN_17 */
	247,  /* NO_AIN */
};

short seport_ain_high_max[] = {
	31,   /* AIN_0  */
	66,   /* AIN_1  */
	100,  /* AIN_2  */
	129,  /* AIN_3  */
	155,  /* AIN_4  */
	178,  /* AIN_5  */
	199,  /* AIN_6  */
	217,  /* AIN_7  */
	232,  /* AIN_8  */
	246,  /* AIN_9  */
	255,  /* NO_AIN */
};

short seport_ain_high_min[] = {
	0,    /* AIN_0  */
	32,   /* AIN_1  */
	67,   /* AIN_2  */
	101,  /* AIN_3  */
	130,  /* AIN_4  */
	156,  /* AIN_5  */
	179,  /* AIN_6  */
	200,  /* AIN_7  */
	218,  /* AIN_8  */
	233,  /* AIN_9  */
	247,  /* NO_AIN */
};

short seport_cco_max[] = {
	4,   /* Headphone */
	6,   /* TV-Out */
	9,   /* OMTP PHF */
	175, /* SEMC PHF */
	205, /* Nothing connected */
};

short seport_cco_min[] = {
	0,   /* Headphone */
	5,   /* TV-Out */
	7,   /* OMTP PHF */
	100, /* SEMC PHF */
	195, /* Nothing connected */
};

short seport_vad_max[] = {
	150,
};

short seport_vad_min[] = {
	0,
};

struct array_info {
	short *array;
	int array_len;
};

static struct array_info array_data[] = {
	{
		.array = seport_rid_max,
		.array_len = ARRAY_SIZE(seport_rid_max)
	},
	{
		.array = seport_rid_min,
		.array_len = ARRAY_SIZE(seport_rid_min)
	},
	{
		.array = seport_ain_low_max,
		.array_len = ARRAY_SIZE(seport_ain_low_max)
	},
	{
		.array = seport_ain_low_min,
		.array_len = ARRAY_SIZE(seport_ain_low_min)
	},
	{
		.array = seport_ain_high_max,
		.array_len = ARRAY_SIZE(seport_ain_high_max)
	},
	{
		.array = seport_ain_high_min,
		.array_len = ARRAY_SIZE(seport_ain_high_min)
	},
	{
		.array = seport_cco_max,
		.array_len = ARRAY_SIZE(seport_cco_max)
	},
	{
		.array = seport_cco_min,
		.array_len = ARRAY_SIZE(seport_cco_min)
	},
	{
		.array = seport_vad_max,
		.array_len = ARRAY_SIZE(seport_vad_max)
	},
	{
		.array = seport_vad_min,
		.array_len = ARRAY_SIZE(seport_vad_min)
	},
};

static ssize_t seport_attrs_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static ssize_t seport_attrs_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count);

#define SEPORT_ATTR(_name) \
	{								\
		.attr = { .name = #_name,				\
			  .mode = (S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH), \
			  .owner = THIS_MODULE				\
		},							\
			.show = seport_attrs_show_property,		\
				 .store = seport_attrs_store_property,	\
				 }

/*
 * If we need more information to be readable from
 * userspace, we can extend this vector. Dont forget
 * to update the corresponding enum in the header file
 */
static struct device_attribute seport_attrs[] = {
	SEPORT_ATTR(rid_max_vals),
	SEPORT_ATTR(rid_min_vals),
	SEPORT_ATTR(rain_low_max_vals),
	SEPORT_ATTR(rain_low_min_vals),
	SEPORT_ATTR(rain_high_max_vals),
	SEPORT_ATTR(rain_high_min_vals),
	SEPORT_ATTR(cco_max_vals),
	SEPORT_ATTR(cco_min_vals),
	SEPORT_ATTR(vad_max_val),
	SEPORT_ATTR(vad_min_val)
};

static int seport_attrs_create_data_buffer(char *buf, int buf_len,
					   short *array, int array_len)
{
	char tmp_buff[10];
	int j = -1;
	int len = 0;

	if (NULL != array && NULL != buf) {
		for (j = 0; j < array_len && buf_len > 0; j++) {
			(void)scnprintf(tmp_buff, sizeof(tmp_buff), "%d ",
					array[j]);
			len = strlcat(buf, tmp_buff, buf_len);
			if (len > buf_len) {
				printk(KERN_ERR "*** %s - Ooops! len " \
				       "(%d) > buf_len (%d)\n",
				       __func__, len, buf_len);
				return -1;
			}
		}
		len = strlcat(buf, "\n", buf_len);
	}

	return len;
}


/* We know that the buffer received here is of PAGE_SIZE (4096 bytes) size. */
static ssize_t seport_attrs_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf) {
	int retval = 0;
	int val = 0;
	const ptrdiff_t off = attr - seport_attrs;

	mutex_lock(&seport_attr_rw_mutex);
	/* Special handling required for HSSD min/max vals */
	if (0 == strcmp(SEPORT_VAD_MIN_VAL_NAME, attr->attr.name)) {
		if (!seport_platform_get_hssd_threshold(HSSD_MIN_VAL, &val))
			retval = scnprintf(buf, PAGE_SIZE, "%u\n", val);
		else
			retval = -EIO;
	} else if (0 == strcmp(SEPORT_VAD_MAX_VAL_NAME, attr->attr.name)) {
		if (!seport_platform_get_hssd_threshold(HSSD_MAX_VAL, &val))
			retval = scnprintf(buf, PAGE_SIZE, "%u\n", val);
		else
			retval = -EIO;
	} else {
		if (off >= 0 && off < ARRAY_SIZE(seport_attrs)) {
			retval =
				seport_attrs_create_data_buffer(
					buf, PAGE_SIZE,
					array_data[off].array,
					array_data[off].array_len);
		} else {
			retval = -EIO;
		}
	}

	mutex_unlock(&seport_attr_rw_mutex);

	return retval;
}

/*
 * There is no need to check char* here since it has already
 * been checked by the calling function.
 * There is only one function that should call here.
 * Returns number of successfully stored values.
 */
static int seport_attrs_update_data(const char *buf, size_t count,
				    short *array, int array_len)
{
	char tmp_buf[10];
	int i, stepper;
	int value_count = 0;
	long conversion = 0;
	char *q = (char *)buf;

	do {
		i = strcspn(q, " \t,\r\n");
		/*
		 * Magic number!
		 * A short can handle 6 digits in length including - sign.
		 * More than that and we have a too large value and can not
		 * store it! */
		if (6 >= i && 0 != i) {
			for (stepper = 0; stepper < i &&
				     stepper < sizeof(tmp_buf); stepper++) {
				/* First char could be - sign.
				   Store it anyway */
				if (0 == stepper)
					tmp_buf[stepper] = q[stepper];
				else if (isdigit(q[stepper]))
					tmp_buf[stepper] = q[stepper];
				else if (q[stepper] == 10 || q[stepper] == 13)
					break;
				else
					goto leave;
			}
		} else {
			printk(KERN_ERR "*** %s - Received out of range " \
			       "value. Aborting\n", __func__);
			goto leave;
		}


		tmp_buf[stepper] = '\0';

		conversion = simple_strtol(tmp_buf, NULL, 0);

		/* Making sure that we get values that are in range */
		if (conversion <= SHORT_MAX &&
		    conversion >= SHORT_MIN) {
			array[value_count] = (short)conversion;
		} else {
			printk(KERN_ERR "%s - Value out of range. "	\
			       "Aborting change!\n", __func__);
			goto leave;
		}

		value_count++;
		q += i+1;
	} while (q < buf + count && value_count < array_len);

leave:
	return value_count;
}

static ssize_t seport_attrs_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	const ptrdiff_t off = attr - seport_attrs;
	int ret = 0;
	int val;

	mutex_lock(&seport_attr_rw_mutex);

	/* Special handling required for HSSD min/max vals */
	if (0 == strcmp(SEPORT_VAD_MIN_VAL_NAME, attr->attr.name)) {
		sscanf(buf, "%d", &val);
		seport_platform_set_hssd_threshold(HSSD_MIN_VAL, val);
	} else if (0 == strcmp(SEPORT_VAD_MAX_VAL_NAME, attr->attr.name)) {
		sscanf(buf, "%d", &val);
		seport_platform_set_hssd_threshold(HSSD_MAX_VAL, val);
	} else {
		if (!buf || !*buf || 0 > off ||
		    ARRAY_SIZE(seport_attrs) <= off) {
			goto error;
		}
		ret = seport_attrs_update_data(buf, count,
					       array_data[off].array,
					       array_data[off].array_len);

		if (array_data[off].array_len != ret)
			printk(KERN_WARNING "%s - Failed to update complete " \
			       " data set. To few parameters?\n",
			       __func__);
	}

error:
	mutex_unlock(&seport_attr_rw_mutex);

	/*
	 * Because we can have more parameters than we need or less
	 * parameters than we need, we just return count when the
	 * data has been handled to make sure that this function
	 * will not be called when not required.
	 */
	return count;
}

/*
 * The dev pointer should be initialized when it gets here since the device
 * has already been registered. If this pointer happens to be NULL, something is
 * very wrong, and I belive that we should make this known via a kernel panic!
 */
int seport_attrs_create_attrs(struct device *dev)
{
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(seport_attrs) && !ret; i++) {
		ret = device_create_file(dev, &seport_attrs[i]);
		if (ret)
			break;
	}

	if (ret) {
		printk(KERN_ERR "*** %s - Failed to create attribute entries\n",
		       __func__);
		while (i--)
			device_remove_file(dev, &seport_attrs[i]);
	}

	return ret;
}

/*
 * The dev pointer should be initialized when it gets here since the device
 * has already been registered. If this pointer happens to be NULL, something is
 * very wrong, and I belive that we should make this know via a kernel panic!
 */
void seport_attrs_destroy_attrs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(seport_attrs); i++)
		device_remove_file(dev, &seport_attrs[i]);

}

void seport_attrs_init(void)
{
	mutex_init(&seport_attr_rw_mutex);
}

void seport_attrs_parse_button_value(u16 *value, int hp_mode)
{
	int i;
	if (!value)
		return;

	mutex_lock(&seport_attr_rw_mutex);

	if (hp_mode) {
		/* Check if button value is off scale */
		if (*value >= seport_ain_high_min[ARRAY_SIZE(
				    seport_ain_high_min)-1]) {
			*value = BUTTON_VAL_ERROR;
			goto check_done;
		}
		for (i = 0; i < ARRAY_SIZE(seport_ain_high_max); i++) {
			if (*value <= seport_ain_high_max[i]) {
				if (*value >= seport_ain_high_min[i]) {
					*value = (u16)i;
					goto check_done;
				} else {
					*value = BUTTON_VAL_ERROR;
					goto check_done;
				}
			}
		}
	} else {
		/* Check if button value is off scale */
		if (*value >= seport_ain_low_min[ARRAY_SIZE(seport_ain_low_min)
						 -1]) {
			*value = BUTTON_VAL_ERROR;
			goto check_done;
		}
		for (i = 0; i < ARRAY_SIZE(seport_ain_low_max); i++) {
			if (*value <= seport_ain_low_max[i]) {
				if (*value >= seport_ain_low_min[i]) {
					*value = (u16)i;
					goto check_done;
				} else {
					*value = BUTTON_VAL_ERROR;
					goto check_done;
				}
			}
		}
	}

check_done:
	mutex_unlock(&seport_attr_rw_mutex);
	return;
}

int seport_attrs_get_rid_max_vals(
	struct seport_trim_param_data *pd)
{
	if (sizeof(seport_rid_max) == pd->value_length) {
		memcpy((void *)pd->values,
		       (void *)seport_rid_max,
		       pd->value_length);
		return 0;
	}
	printk(KERN_ERR "%s - Invalid data lenght. Expected %d but got %d\n",
	       __func__, sizeof(seport_rid_max), pd->value_length);

	return -1;
}

int seport_attrs_get_rid_min_vals(
	struct seport_trim_param_data *pd)
{
	if (sizeof(seport_rid_min) == pd->value_length) {
		memcpy((void *)pd->values,
		       (void *)seport_rid_min,
		       pd->value_length);
		return 0;
	}
	printk(KERN_ERR "%s - Invalid data lenght. Expected %d but got %d\n",
	       __func__, sizeof(seport_rid_min), pd->value_length);
	return -1;
}

int seport_attrs_get_cco_max_vals(
	struct seport_trim_param_data *pd)
{
	if (sizeof(seport_cco_max) == pd->value_length) {
		memcpy((void *)pd->values,
		       (void *)seport_cco_max,
		       pd->value_length);
		return 0;
	}
	printk(KERN_ERR "%s - Invalid data lenght. Expected %d but got %d\n",
	       __func__, sizeof(seport_cco_max), pd->value_length);

	return -1;
}

int seport_attrs_get_cco_min_vals(
	struct seport_trim_param_data *pd)
{
	if (sizeof(seport_cco_min) == pd->value_length) {
		memcpy((void *)pd->values,
		       (void *)seport_cco_min,
		       pd->value_length);
		return 0;
	}
	printk(KERN_ERR "%s - Invalid data lenght. Expected %d but got %d\n",
	       __func__, sizeof(seport_cco_min), pd->value_length);

	return -1;
}

int seport_attrs_get_vad_max_vals(
	struct seport_trim_param_data *pd)
{
	if (sizeof(seport_vad_max) == pd->value_length) {
		memcpy((void *)pd->values,
		       (void *)seport_vad_max,
		       pd->value_length);
		return 0;
	}
	printk(KERN_ERR "%s - Invalid data lenght. Expected %d but got %d\n",
	       __func__, sizeof(seport_cco_min), pd->value_length);

	return -1;
}

int seport_attrs_get_vad_min_vals(
	struct seport_trim_param_data *pd)
{
	if (sizeof(seport_vad_min) == pd->value_length) {
		memcpy((void *)pd->values,
		       (void *)seport_vad_min,
		       pd->value_length);
		return 0;
	}
	printk(KERN_ERR "%s - Invalid data lenght. Expected %d but got %d\n",
	       __func__, sizeof(seport_cco_min), pd->value_length);

	return -1;
}
