/* arch/arm/mach-msm/proximity_platform_delta.c
 *
 * Proximity sensor driver: platform independant part.
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/semcclass.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/earlysuspend.h>
#include <mach/oem_rapi_client.h>

#define DBG(X)  /* X */
#define LOCK() mutex_lock(&proximity.update_lock)
#define UNLOCK() mutex_unlock(&proximity.update_lock)
#define PROXIMITY_ADC_VALUE_MAX 255
#define DEV_NAME "proximity-sensor"

enum proximity_state_t {
	PROX_STATE_INIT = 0x00,
	PROX_RAPI_READY = 0x01,
	PROX_SUSPEND    = 0x80
};

struct proximity_data_t {
	struct semc_classdev semc_dev;
	int sensor_value;
	int adc_value;
	int enable_counter;
	enum proximity_state_t proximity_state;
	struct msm_rpc_client *rpc_client;
	struct mutex update_lock;
	u16 led_on_time_ms;
	u8 threshold;
	u8 hysteresis;
};

static struct proximity_data_t proximity;

static 	int proximity_sensor_value_changed_cb(
			   struct oem_rapi_client_streaming_func_cb_arg *arg,
		       struct oem_rapi_client_streaming_func_cb_ret *ret)
{
	struct prox_value_from_rpc_t {
		u32 sensor_value;
		u32 adc_value;
	} *args;

	if (arg && arg->event == OEM_RAPI_SERVER_EVENT_PROXIMITY_VALUE_CB &&
			arg->in_len == sizeof(struct prox_value_from_rpc_t) && arg->input) {

		args = (struct prox_value_from_rpc_t *)arg->input;

		DBG(printk(KERN_DEBUG
				"%s (%s): sensor adc_value = %d, sensor value = %d.\n",
				DEV_NAME, __func__, args->adc_value, args->sensor_value));

		LOCK();
		proximity.sensor_value = args->sensor_value;
		proximity.adc_value = args->adc_value;
		UNLOCK();
		sysfs_notify(&proximity.semc_dev.dev->kobj, NULL, "sensor");
	} else {
		if (arg)
			printk(KERN_ERR "%s (%s): "
					"got event %d, in_len %d, input->0x%x\n",
					DEV_NAME, __func__, arg->event, arg->in_len,
					(unsigned int)arg->input);
		else
			printk(KERN_ERR "%s (%s): Null arg pointer received\n",
					DEV_NAME, __func__);
	}
	return 0;
}

static int start_oem_rapi_client(void)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret;

	proximity.rpc_client = oem_rapi_client_init();
	if (!proximity.rpc_client || IS_ERR(proximity.rpc_client)) {
		printk(KERN_ERR "%s (%s): oem_rapi_client_init failed.\n",
			   DEV_NAME, __func__);
		return -1;
	}

	memset(&client_arg, 0, sizeof(client_arg));
	memset(&client_ret, 0, sizeof(client_ret));

	client_arg.event = OEM_RAPI_CLIENT_EVENT_PROXIMITY_VALUE_CB_REGISTER;
	client_arg.cb_func = proximity_sensor_value_changed_cb;
	/*
	* client_arg.input is unused, but required by
	* oem_rapi_client_streaming_function to be not NULL
	*/
	client_arg.input = (char *)&ret;
	ret = oem_rapi_client_streaming_function(proximity.rpc_client,
											 &client_arg, &client_ret);
	if (ret < 0) {
		printk(KERN_ERR
				"%s (%s): Failed to register oem_rapi callback (rc=%d).\n",
				DEV_NAME, __func__, ret);
		return ret;
	}
	proximity.proximity_state |= PROX_RAPI_READY;
	return 0;
}

static int proximity_sensor_set_configuration(int conf, int value)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret;

	if (!(proximity.proximity_state & PROX_RAPI_READY)) {
		if (start_oem_rapi_client() < 0)
			return -1;
	}

	memset(&client_arg, 0, sizeof client_arg);
	client_arg.event = conf;
	client_arg.in_len = sizeof(value);
	client_arg.input = (char *)&value;
	ret = oem_rapi_client_streaming_function(proximity.rpc_client,
											 &client_arg, &client_ret);
	if (ret < 0)
		printk(KERN_ERR
				"%s (%s): failed to set configuration %d (rc=%d).\n",
				DEV_NAME, __func__, conf, ret);
	return ret < 0 ? ret : 0;
}

static int proximity_sensor_set_configuration_locked(int conf, int value)
{
	int ret;
	LOCK();
	ret = proximity_sensor_set_configuration(conf, value);
	UNLOCK();
	return ret;
}

static void proximity_enable_sensing(int enable)
{
	enum proximity_state_t state;
	int current_sensor_value;
	int start_stop_cmd = 0;

	LOCK();
	current_sensor_value = proximity.sensor_value;
	if (enable) {
		if (1 == ++proximity.enable_counter)
			start_stop_cmd = OEM_RAPI_CLIENT_EVENT_PROXIMITY_ACTIVATE;
	} else if (proximity.enable_counter) {
		if (0 == --proximity.enable_counter)
			start_stop_cmd = OEM_RAPI_CLIENT_EVENT_PROXIMITY_DEACTIVATE;
	}
	state = proximity.proximity_state;
	UNLOCK();

	if (!start_stop_cmd)
		return;

	if (state & PROX_SUSPEND) {
		DBG(printk(KERN_DEBUG "%s (%s): sensor enable=%d (pending))\n",
				   DEV_NAME, __func__, enable);)
		return;
	}
	proximity_sensor_set_configuration(start_stop_cmd, 0);

	DBG(printk(KERN_DEBUG "%s (%s): sensor enable=%d)\n",
				   DEV_NAME, __func__, enable);)
}

void suspend_sensing(void)
{
	LOCK();
	if (proximity.enable_counter) {
		proximity_sensor_set_configuration(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_DEACTIVATE, 0);
		DBG(printk(KERN_DEBUG "%s (%s): sensing suspended\n",
				   DEV_NAME, __func__);)
	}
	UNLOCK();
}

void resume_sensing(void)
{
	LOCK();
	if (proximity.enable_counter) {
		proximity_sensor_set_configuration(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_ACTIVATE, 0);
		DBG(printk(KERN_DEBUG "%s (%s): sensing  resumed\n",
				   DEV_NAME, __func__);)
	} else {
		DBG(printk(KERN_DEBUG "%s (%s): Nothing to resume\n",
				   DEV_NAME, __func__);)
	}
	UNLOCK();
}


static ssize_t th_set(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	u32 value;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if (-EINVAL == ret)
		return -EINVAL;
	if (value <= PROXIMITY_ADC_VALUE_MAX) {
		int rc = proximity_sensor_set_configuration_locked(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_THRESHOLD_SET, value);
		if (!rc)
			printk(KERN_INFO "%s (%s): threshold=%d\n",
					DEV_NAME, __func__, value);
		LOCK();
		proximity.threshold = value;
		UNLOCK();
	} else
		printk(KERN_ERR "%s: Couldn't set threshold %d\n",
			   DEV_NAME, value);
	return ret;
}

static ssize_t th_get(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret;

	LOCK();
	ret = sprintf(buf, "%u\n", proximity.threshold);
	UNLOCK();
	return ret;
}


static ssize_t hyst_set(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	u32 value;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if (-EINVAL == ret)
		return -EINVAL;
	if (value < PROXIMITY_ADC_VALUE_MAX) {
		int rc = proximity_sensor_set_configuration_locked(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_HYSTERESIS_SET, value);
		if (!rc)
			printk(KERN_INFO "%s (%s): hysteresis=%d\n",
					DEV_NAME, __func__, value);
		LOCK();
		proximity.hysteresis = value;
		UNLOCK();
	} else
		printk(KERN_ERR "%s: Couldn't set hysteresis %d\n",
			   DEV_NAME, value);
	return ret;
}

static ssize_t hyst_get(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret;

	LOCK();
	ret = sprintf(buf, "%u\n", proximity.hysteresis);
	UNLOCK();
	return ret;
}

static ssize_t ledon_set(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	u32 value;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if (-EINVAL == ret)
		return ret;
	proximity_sensor_set_configuration_locked(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_ON_TIME_SET, value);
	LOCK();
	proximity.led_on_time_ms = value;
	UNLOCK();

	return ret;
}

static ssize_t ledon_get(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret;

	LOCK();
	ret = sprintf(buf, "%u\n", proximity.led_on_time_ms);
	UNLOCK();
	return ret;
}


static ssize_t ledoff_set(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	u32 value;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if (-EINVAL == ret)
		return ret;
	proximity_sensor_set_configuration_locked(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_OFF_TIME_SET, value);
	return ret;
}

static ssize_t enable_set(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	u32 enable;
	ssize_t ret = semc_classdev_read_interface(buf, size, &enable, 10);

	if (-EINVAL == ret)
		return -EINVAL;
	proximity_enable_sensing(!!enable);
	return ret;
}

static ssize_t use_dout_set(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	u32 use;
	ssize_t ret = semc_classdev_read_interface(buf, size, &use, 10);

	if (-EINVAL == ret)
		return -EINVAL;
	proximity_sensor_set_configuration_locked(
				OEM_RAPI_CLIENT_EVENT_PROXIMITY_USE_DOUT_SET, !!use);
	return ret;
}

static ssize_t enable_get(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret;
	int enable;

	LOCK();
	enable = proximity.enable_counter;
	UNLOCK();
	ret = sprintf(buf, "%d\n", enable);
	return ret;
}

static ssize_t sensor_get(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret;
	int value;

	LOCK();
	if (proximity.enable_counter)
		value = proximity.sensor_value;
	else
		value = 0;
	UNLOCK();
	ret = sprintf(buf, "%u\n", value);
	return ret;
}

static ssize_t sensor_adc_get(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	ssize_t ret;
	int value;

	LOCK();
	if (proximity.enable_counter)
		value = proximity.adc_value;
	else
		value = 0;
	UNLOCK();
	ret = sprintf(buf, "%u\n", value);
	return ret;
}

static struct device_attribute attributes[] = {
	__ATTR(sensor,  0444, sensor_get,  NULL),
	__ATTR(adc,  0444, sensor_adc_get,  NULL),
	__ATTR(enable,  0666, enable_get,  enable_set),
	__ATTR(led_on_ms,  0666, ledon_get,  ledon_set),
	__ATTR(led_off_ms,  0200, NULL,  ledoff_set),
	__ATTR(threshold,  0666, th_get,  th_set),
	__ATTR(hysteresis,  0666, hyst_get,  hyst_set),
	__ATTR(use_dout,  0200, NULL,  use_dout_set),
};


static int proximity_probe(struct platform_device *pdev)
{
	int res;

	printk(KERN_INFO "%s: %s called.\n", DEV_NAME, __func__);

	mutex_init(&proximity.update_lock);

	proximity.semc_dev.name = DEV_NAME;
	proximity.semc_dev.attributes = attributes;
	proximity.semc_dev.attr_number = sizeof(attributes) /
			sizeof(struct device_attribute);
	res = semc_classdev_register(&pdev->dev, &proximity.semc_dev);
	if (res < 0) {
		printk(KERN_ERR "%s: Failed to register interface\n", DEV_NAME);
		goto err_exit;
	}

	printk(KERN_INFO "%s: driver installation succeeded\n", DEV_NAME);

	return 0;

err_exit:
	printk(KERN_ERR "%s: driver installation failed\n", DEV_NAME);
	return res;
}

static int __devexit proximity_remove(struct platform_device *pdev)
{
	suspend_sensing();
	semc_classdev_unregister(&proximity.semc_dev);
	oem_rapi_client_close();
	printk(KERN_INFO "%s: driver removed\n", DEV_NAME);
	return 0;
}

static struct platform_driver proximity_driver = {
	.probe = proximity_probe,
	.remove = __devexit_p(proximity_remove),
	.driver = {
	.name = DEV_NAME,
	.owner = THIS_MODULE,
	},
};

static int __init proximity_driver_init(void)
{
	int rc = platform_driver_register(&proximity_driver);
	DBG(printk(KERN_DEBUG "%s: %s rc=%d\n", DEV_NAME, __func__, rc);)
	return rc;
}
module_init(proximity_driver_init);

static void __exit proximity_driver_exit(void)
{
	platform_driver_unregister(&proximity_driver);
}
module_exit(proximity_driver_exit);

MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_DESCRIPTION("delta proximity sensor driver");
MODULE_LICENSE("GPL");

