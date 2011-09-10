/* drivers/misc/semc/power/power.c
 *
 * SEMC power driver
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Imre Sunyi <imre.sunyi.x@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/semc/power/semc_power.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <mach/semc_power_platform.h>
#include "bq24180.h"

#define POWER_DRV_NAME "semc_power"
/* Enable/disable sys fs debug */
/* #define DEBUG_FS */

#define MIN(x,y) ((x) < (y) ? (x) : (y))

struct charger_info {
	struct charger_status status;
	u8 overvoltage;
};

struct override_value {
	u8 active;
	int value;
};

struct power_info {
	unsigned int ac_online;
	unsigned int usb_online;
	int charge_status;       /* POWER_SUPPLY_STATUS_* */
	u8 power_status_updated;
	struct battery_status battery_status;
	struct charger_info charger_info;
	struct battery_data bdata;
	u8 shutdown_system;
	u8 over_voltage;
	u8 safety_timer_fault;
	u8 dcout_fault;

	wait_queue_head_t wq;    /* wait queue for poll */

	struct mutex lock;

	struct power_param charge_params;
	struct mutex charge_param_lock;
	unsigned int status;
	u16 max_usb_current;

	struct power_supply bat;
	struct power_supply usb;
	struct power_supply ac;

	struct semc_notify_platform notify;

	u8 disable_charging;

	u8 clients_opened;

	/* For debug */
#ifdef DEBUG_FS
	struct override_value capacity_debug;
	struct override_value temperature_debug;
	struct override_value voltage_debug;
	struct override_value technology_debug;
#endif
};

static struct power_info power_info;
static struct power_ops power_ops;
static dcio_over_current_callback_t dcio_over_current_fn = NULL;
static atomic_t power_init_ok = ATOMIC_INIT(0);
static struct wake_lock power_supply_lock;

static enum power_supply_property semc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
#ifndef CONFIG_ANDROID
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
#endif
};

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH
};

static ssize_t semc_battery_show_property(struct device *dev,
					     struct device_attribute *attr,
					     char *buf);
static ssize_t store_disable_charging(struct device *pdev,
				      struct device_attribute *attr,
				      const char* pbuf,
				      size_t count);
static void power_event_handler(enum event event, void *data);

#ifdef DEBUG_FS
static ssize_t store_usb_connected(struct device *pdev,
				   struct device_attribute *attr,
				   const char* pbuf,
				   size_t count);
static ssize_t store_usb_current(struct device *pdev,
				 struct device_attribute *attr,
				 const char* pbuf,
				 size_t count);
static ssize_t store_timer(struct device *pdev,
				 struct device_attribute *attr,
				 const char* pbuf,
				 size_t count);
static ssize_t store_capacity(struct device *pdev,
			      struct device_attribute *attr,
			      const char* pbuf,
			      size_t count);
static ssize_t store_temperature(struct device *pdev,
				 struct device_attribute *attr,
				 const char* pbuf,
				 size_t count);
static ssize_t store_voltage(struct device *pdev,
			     struct device_attribute *attr,
			     const char* pbuf,
			     size_t count);
static ssize_t store_technology(struct device *pdev,
				struct device_attribute *attr,
				const char* pbuf,
				size_t count);
#endif /* DEBUG_FS */


#define SEMC_BATTERY_ATTR(_name)						\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },	\
	.show = semc_battery_show_property,					\
	.store = NULL,								\
}

static struct device_attribute semc_battery_attrs[] = {
#ifdef CONFIG_ANDROID
	SEMC_BATTERY_ATTR(batt_vol),
	SEMC_BATTERY_ATTR(batt_temp),
#endif
	SEMC_BATTERY_ATTR(batt_id_unknown),
	SEMC_BATTERY_ATTR(batt_id_type1),
	SEMC_BATTERY_ATTR(batt_id_type2),

	/* TODO: Check that service menu can access this with these rights */
	__ATTR(disable_charging, 0644,
	       semc_battery_show_property, store_disable_charging),

#ifdef DEBUG_FS
	__ATTR(set_usb_connected, 0222,
	       semc_battery_show_property, store_usb_connected),
	__ATTR(set_usb_current, 0222,
	       semc_battery_show_property, store_usb_current),
	__ATTR(set_timer, 0222,
	       semc_battery_show_property, store_timer),
	__ATTR(set_capacity, 0222,
	       semc_battery_show_property, store_capacity),
	__ATTR(set_temperature, 0222,
	       semc_battery_show_property, store_temperature),
	__ATTR(set_voltage, 0222,
	       semc_battery_show_property, store_voltage),
	__ATTR(set_technology, 0222,
	       semc_battery_show_property, store_technology),
#endif /* DEBUG_FS */
};

enum {
#ifdef CONFIG_ANDROID
	BATT_VOL = 0,
	BATT_TEMP,
#endif
	BATT_ID_UNKNOWN,
	BATT_ID_TYPE1,
	BATT_ID_TYPE2,
	DISABLE_CHARGING,
#ifdef DEBUG_FS
	SET_USB_CONNECTED,
	SET_USB_CURRENT,
	SET_TIMER,
	SET_CAPACITY,
	SET_TEMPERATURE,
	SET_VOLTAGE,
	SET_TECHNOLOGY,
#endif /* DEBUG_FS */
};

static int power_ioctl(struct inode *inode, struct file *filp,
			    unsigned int cmd, unsigned long arg);
static unsigned int power_poll(struct file *filp,
			       struct poll_table_struct *wait);
static int power_open(struct inode *inode, struct file *filp);
static int power_release(struct inode *inode, struct file *filp);

static char *power_supplied_to[] = {
	"battery"
};

static struct file_operations power_fops = {
	.owner   = THIS_MODULE,
	.ioctl   = power_ioctl,
	.poll    = power_poll,
	.open    = power_open,
	.release = power_release,
};

static struct miscdevice power_misc = {
	MISC_DYNAMIC_MINOR,
	SEMC_POWER_DEVICE_NAME,
	&power_fops,
};

static int semc_battery_create_attrs(struct device *dev)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(semc_battery_attrs); i++) {
		ret = device_create_file(dev, &semc_battery_attrs[i]);
		if (ret)
			goto semc_create_attrs_failed;
	}

	goto succeed;

semc_create_attrs_failed:
	while (i--)
		device_remove_file(dev, &semc_battery_attrs[i]);

succeed:
	return ret;
}

static void semc_battery_remove_attrs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(semc_battery_attrs); i++) {
		(void)device_remove_file(dev, &semc_battery_attrs[i]);
	}
}

static ssize_t semc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int ret = 0;
	const ptrdiff_t off = attr - semc_battery_attrs;

	if (!atomic_read(&power_init_ok))
		return -EBUSY;

	switch (off) {
#ifdef CONFIG_ANDROID
	case BATT_VOL:
	{
		u16 voltage;
#ifdef DEBUG_FS
		if (power_info.voltage_debug.active)
			voltage = (u16)power_info.voltage_debug.value;
		else
			ret = power_ops.get_battery_voltage(&voltage);
#else
		ret = power_ops.get_battery_voltage(&voltage);
#endif /* DEBUG_FS */
		if (!ret)
			ret = scnprintf(buf, PAGE_SIZE, "%u\n", voltage);
		break;
	}
	case BATT_TEMP:
	{
		int temp_celsius;

		mutex_lock(&power_info.lock);
#ifdef DEBUG_FS
		if (power_info.temperature_debug.active)
			temp_celsius = power_info.temperature_debug.value;
		else
			temp_celsius = power_info.bdata.temp_celsius;
#else
		temp_celsius = power_info.bdata.temp_celsius;
#endif /* DEBUG_FS */
		ret = scnprintf(buf, PAGE_SIZE, "%d\n",
				temp_celsius * 10);
		mutex_unlock(&power_info.lock);
		break;
	}
#endif /* CONFIG_ANDROID */
	case BATT_ID_UNKNOWN:
	{
		u32 id;
		ret = power_ops.get_battery_thermistor(
			BATTERY_TECHNOLOGY_UNKNOWN, UNIT_MV, &id);
		if (!ret)
			ret = scnprintf(buf, PAGE_SIZE, "%u\n", id);
		break;
	}
	case BATT_ID_TYPE1:
	{
		u32 id;
		ret = power_ops.get_battery_thermistor(BATTERY_TECHNOLOGY_TYPE1,
						       UNIT_MV, &id);
		if (!ret)
			ret = scnprintf(buf, PAGE_SIZE, "%u\n", id);
		break;
	}
	case BATT_ID_TYPE2:
	{
		u32 id;
		ret = power_ops.get_battery_thermistor(BATTERY_TECHNOLOGY_TYPE2,
						       UNIT_MV, &id);
		if (!ret)
			ret = scnprintf(buf, PAGE_SIZE, "%u\n", id);
		break;
	}
	case DISABLE_CHARGING:
	{
		mutex_lock(&power_info.lock);
		ret = scnprintf(buf, PAGE_SIZE, "%d\n",
				power_info.disable_charging);
		mutex_unlock(&power_info.lock);
		break;
	}
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int read_sysfs_interface(const char *pbuf, int size, s32 *pvalue, u8 base)
{
	ssize_t ret = -EINVAL;
	char *pend;
	int count;

	if (!pbuf || !pvalue)
	  return ret;

	*pvalue =  simple_strtoll(pbuf, &pend, base);
	count = pend - pbuf;

	if (*pend && isspace(*pend))
		count++;

	if (count == size)
		ret = count;

	return ret;
}

/*
 * Sysfs interface to enable/disable charging
 * Writing 1 means disable charging
 * Writing 0 means enable charging
 *
 * Returns -1 if failed
 */
static ssize_t store_disable_charging(struct device *pdev,
				      struct device_attribute *attr,
				      const char* pbuf,
				      size_t count)
{
	s32 input;
	ssize_t ret = read_sysfs_interface(pbuf, count, &input, 10);

	if (ret > 0 && ((input == 0) || (input == 1))) {
		unsigned int current_to_set;
		unsigned int eoc_level = 0;
		int retval;

		mutex_lock(&power_info.lock);

		if (input == 0) {
			power_info.disable_charging = 0;
			current_to_set =
				power_info.charge_params.battery_charge_current;
			eoc_level =
				power_info.charge_params.charging_completed_current_level;
		} else {
			power_info.disable_charging = 1;
			current_to_set = 0;
		}

		mutex_unlock(&power_info.lock);

		/* Before enabling charging turn off the EOC to not trigger
		 * false charge complete.
		 */
		if (!input)
			power_ops.set_charger_current_termination(0);

		retval = power_ops.set_charger_current(current_to_set);
		if (retval < 0) {
			printk(KERN_ERR POWER_DRV_NAME
			 ": Failed to %sable charging\n", input ? "dis" : "en");
			ret = retval;
		}

		/* Charging is enabled and now turn on EOC */
		if (!input) {
			/* Give some time to settle before turning on.
			 * 50 ms is recommendation from TI.
			 */
			msleep(50);
			power_ops.set_charger_current_termination(eoc_level);
		}
	} else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs disable_charging. "
		       "Expect [0,1]\n");
		ret = -EINVAL;
	}

	return ret;
}

#ifdef DEBUG_FS
static ssize_t store_usb_connected(struct device *pdev,
				   struct device_attribute *attr,
				   const char* pbuf,
				   size_t count)
{
	s32 input;
	ssize_t ret = read_sysfs_interface(pbuf, count, &input, 10);

	if (ret < 0)
		return ret;

	if (!input)
		power_event_handler(SEMC_POWER_USB_DISCONNECTED, NULL);
	else if (input == 1) {
		u16 usb_current = 100;
		power_event_handler(SEMC_POWER_USB_CONNECTED, (void *)&usb_current);
	} else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_usb_connected. Expect [0,1]\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t store_usb_current(struct device *pdev,
				 struct device_attribute *attr,
				 const char* pbuf,
				 size_t count)
{
	s32 input;
	ssize_t ret = read_sysfs_interface(pbuf, count, &input, 10);

	if (ret > 0 && input >= 0 && input <= 1800)
		power_event_handler(SEMC_POWER_USB_CURRENT_AVAILABLE, (void *)&input);
	else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_usb_current. Expect [0..1800]\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t store_timer(struct device *pdev,
			   struct device_attribute *attr,
			   const char* pbuf,
			   size_t count)
{
	s32 input;
	ssize_t ret = read_sysfs_interface(pbuf, count, &input, 10);

	if (ret > 0 && input >= 0)
		power_ops.set_safety_timer((u16)input);
	else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_timer. Expect >=0\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t store_capacity(struct device *pdev,
			      struct device_attribute *attr,
			      const char* pbuf,
			      size_t count)
{
	s32 cap;
	ssize_t ret = read_sysfs_interface(pbuf, count, &cap, 10);

	if (ret > 0 && (cap >= -1 && cap <= 100)) {
		mutex_lock(&power_info.lock);

		if (cap >= 0) {
			power_info.capacity_debug.active = 1;
			power_info.capacity_debug.value = cap;
		} else
			power_info.capacity_debug.active = 0;

		power_info.power_status_updated = 1;
		power_info.status |= STATUS_BDATA_CHANGED;
		mutex_unlock(&power_info.lock);

		power_supply_changed(&power_info.bat);
		wake_up_interruptible(&power_info.wq);
	} else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_capacity. Expect [-1..100]. "
		       " -1 releases the debug value\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t store_temperature(struct device *pdev,
				 struct device_attribute *attr,
				 const char* pbuf,
				 size_t count)
{
	s32 temp;
	ssize_t ret = read_sysfs_interface(pbuf, count, &temp, 10);

	if (ret > 0 && temp >= -21 && temp <= 100) {
		mutex_lock(&power_info.lock);

		if (temp >= -20) {
			power_info.temperature_debug.active = 1;
			power_info.temperature_debug.value = temp;
		} else
			power_info.temperature_debug.active = 0;

		power_info.power_status_updated = 1;
		power_info.status |= STATUS_BDATA_CHANGED;
		mutex_unlock(&power_info.lock);

		power_supply_changed(&power_info.bat);
		wake_up_interruptible(&power_info.wq);
	} else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_temperature. "
		       "Expect [-21..100]. -21 releases the debug value\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t store_voltage(struct device *pdev,
			     struct device_attribute *attr,
			     const char* pbuf,
			     size_t count)
{
	s32 volt;
	ssize_t ret = read_sysfs_interface(pbuf, count, &volt, 10);

	if (ret > 0 && volt >= -1 && volt <= 5000) {
		mutex_lock(&power_info.lock);

		if (volt >= 0) {
			power_info.voltage_debug.active = 1;
			power_info.voltage_debug.value = volt;
		} else
			power_info.voltage_debug.active = 0;

		mutex_unlock(&power_info.lock);

		power_supply_changed(&power_info.bat);

	} else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_voltage. Expect [-1..5000] "
		       " -1 releases the debug value\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t store_technology(struct device *pdev,
				struct device_attribute *attr,
				const char* pbuf,
				size_t count)
{
	s32 tech;
	ssize_t ret = read_sysfs_interface(pbuf, count, &tech, 10);

	if (ret > 0 && tech >= -1 &&
	    tech <= BATTERY_TECHNOLOGY_TYPE2) {
		mutex_lock(&power_info.lock);

		if (tech >= BATTERY_TECHNOLOGY_UNKNOWN) {
			power_info.technology_debug.active = 1;
			power_info.technology_debug.value =
				(enum battery_technology)tech;
		} else
			power_info.technology_debug.active = 0;

		power_info.power_status_updated = 1;
		power_info.status |= STATUS_BDATA_CHANGED;
		mutex_unlock(&power_info.lock);

		power_supply_changed(&power_info.bat);
		wake_up_interruptible(&power_info.wq);
	} else {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Wrong input to sysfs set_technology. Expect [-1..2] "
		       " -1 releases the debug value\n");
		ret = -EINVAL;
	}

	return ret;
}
#endif /* DEBUG_FS */

static int power_get_health(void)
{
	int health;

	mutex_lock(&power_info.lock);

	if (power_info.shutdown_system)
		health = POWER_SUPPLY_HEALTH_DEAD;
	else if (power_info.safety_timer_fault)
		health =  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	else if (power_info.over_voltage)
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else {
		switch (power_info.battery_status.temp_status) {
		case BATTERY_TEMP_STATUS_COLD:
			health = POWER_SUPPLY_HEALTH_COLD;
			break;
		case BATTERY_TEMP_STATUS_NORMAL:
			/* fall through */
		case BATTERY_TEMP_STATUS_WARM:
			health = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case BATTERY_TEMP_STATUS_OVERHEAT:
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
		default:
			health = POWER_SUPPLY_HEALTH_UNKNOWN;
			break;
		}

	}

	mutex_unlock(&power_info.lock);

	return health;
}

static int power_get_technology(void)
{
	int technology;
	enum battery_technology batt_tech;

	mutex_lock(&power_info.lock);

#ifdef DEBUG_FS
	if (power_info.technology_debug.active)
		batt_tech = (enum battery_technology)
			power_info.technology_debug.value;
	else
		batt_tech = power_info.bdata.technology;
#else
	batt_tech = power_info.bdata.technology;
#endif /* DEBUG_FS */

	switch (batt_tech) {
	case BATTERY_TECHNOLOGY_TYPE1:
		technology = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case BATTERY_TECHNOLOGY_TYPE2:
		technology = POWER_SUPPLY_TECHNOLOGY_LiMn;
		break;
	case BATTERY_TECHNOLOGY_UNKNOWN:
		/* fall through */
	default:
		technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	mutex_unlock(&power_info.lock);

	return technology;
}

static int power_get_charger_status(enum power_supply_type supply)
{
	int status;

	mutex_lock(&power_info.lock);

	if ((supply == POWER_SUPPLY_TYPE_MAINS && power_info.ac_online) ||
	    (supply == POWER_SUPPLY_TYPE_USB && power_info.usb_online)) {
		switch (power_info.charger_info.status.strength) {
		case CHARGER_STRENGTH_GOOD:
			status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case CHARGER_STRENGTH_WEAK:
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			status = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
	} else
		status = POWER_SUPPLY_STATUS_UNKNOWN;

	mutex_unlock(&power_info.lock);

	return status;
}

static int power_get_charger_health(enum power_supply_type supply)
{
	int health;

	mutex_lock(&power_info.lock);

	if ((supply == POWER_SUPPLY_TYPE_MAINS && power_info.ac_online) ||
	    (supply == POWER_SUPPLY_TYPE_USB && power_info.usb_online)) {
		if (power_info.charger_info.overvoltage)
			health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			health = POWER_SUPPLY_HEALTH_GOOD;
	} else
		health = POWER_SUPPLY_HEALTH_UNKNOWN;

	mutex_unlock(&power_info.lock);

	return health;
}

static int power_handle_set_params(unsigned long arg)
{
	int ret;
	int error;

	struct power_param param;
	struct power_param *old_param;
	struct semc_notify_platform old_notify;

	/* Do not use &power_info.lock here since parameter settings may cause
	 * deadlock if the charger driver is also using mutexes and is calling
	 * the callback to power_event_handler() in this file.
	 */
	mutex_lock(&power_info.charge_param_lock);

	ret = 0;

	old_notify = power_info.notify;
	old_param = &power_info.charge_params;
	error = copy_from_user(&param, (struct power_param *)arg, sizeof param);
	if (error) {
		ret = -EFAULT;
		printk(KERN_ERR POWER_DRV_NAME
		       ": Failed get charging params from user-space!\n");
		goto set_params_exit;
	}

	if (param.max_battery_charge_voltage !=
	    old_param->max_battery_charge_voltage) {
		ret = power_ops.set_charger_maximum_voltage(
			param.max_battery_charge_voltage);
		if (ret)
			goto set_params_exit;

		old_param->max_battery_charge_voltage =
			param.max_battery_charge_voltage;
	}

	if (param.max_battery_charge_current !=
	    old_param->max_battery_charge_current) {
		ret = power_ops.set_charger_maximum_current(
			param.max_battery_charge_current);
		if (ret)
			goto set_params_exit;

		old_param->max_battery_charge_current =
			param.max_battery_charge_current;
		power_info.notify.battery_charge_current =
			MIN(param.battery_charge_current,
			    param.max_battery_charge_current);
	}

	if (param.charging_completed_current_level !=
	    old_param->charging_completed_current_level) {
		ret = power_ops.set_charger_current_termination(
			param.charging_completed_current_level);
		if (ret)
			goto set_params_exit;

		old_param->charging_completed_current_level =
			param.charging_completed_current_level;
	}

	if (param.battery_charge_voltage != old_param->battery_charge_voltage) {
		ret = power_ops.set_charger_voltage(
			param.battery_charge_voltage);
		if (ret)
			goto set_params_exit;

		old_param->battery_charge_voltage = param.battery_charge_voltage;
	}

	if (param.charger_charge_current != old_param->charger_charge_current) {
		ret = power_ops.set_input_charger_current(
			param.charger_charge_current);
		if (ret)
			goto set_params_exit;

		old_param->charger_charge_current = param.charger_charge_current;
		power_info.notify.charger_charge_current =
			param.charger_charge_current;
	}

	if (param.battery_charge_current != old_param->battery_charge_current) {
		/* Make sure no one has stopped charging */
		if (power_info.disable_charging == 0) {
			ret = power_ops.set_charger_current(
				param.battery_charge_current);
			if (ret)
				goto set_params_exit;
		} else {
			printk(KERN_INFO POWER_DRV_NAME
				": Charging not allowed (stopped by user)\n");
		}

		old_param->battery_charge_current = param.battery_charge_current;
		power_info.notify.battery_charge_current =
			MIN(param.battery_charge_current,
			    param.max_battery_charge_current);
	}

set_params_exit:
	if (memcmp(&old_notify, &power_info.notify, sizeof power_info.notify))
		power_ops.notify_platform(&power_info.notify);

	mutex_unlock(&power_info.charge_param_lock);

	return ret;
}

static unsigned int power_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int ret = 0;

	if (!atomic_read(&power_init_ok))
		return ret;

	poll_wait(filp, &power_info.wq, wait);

	mutex_lock(&power_info.lock);

	if (power_info.power_status_updated) {
		power_info.power_status_updated = 0;
		ret = (POLLIN | POLLRDNORM);
	}

	mutex_unlock(&power_info.lock);

	return ret;
}

static int power_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	if (!atomic_read(&power_init_ok))
		return -EBUSY;

	mutex_lock(&power_info.lock);

	if (power_info.clients_opened) {
		printk(KERN_INFO POWER_DRV_NAME
		       ": Not allowed to open. Another client is using"
		       " this file\n");
		ret = -EMFILE;
	} else
		power_info.clients_opened++;

	mutex_unlock(&power_info.lock);

	return ret;
}

static int power_release(struct inode *inode, struct file *filp)
{
	if (!atomic_read(&power_init_ok))
		return -EBUSY;

	mutex_lock(&power_info.lock);
	if (power_info.clients_opened)
		power_info.clients_opened--;
	mutex_unlock(&power_info.lock);

	return 0;
}

static int power_ioctl(struct inode *inode, struct file *filp,
			    unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int ret = 0;

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != SEMC_POWER_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > SEMC_POWER_IOC_MAXNR)
		return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	if (!atomic_read(&power_init_ok))
		return -EBUSY;

	switch (cmd) {
	case SEMC_POWER_IOC_SET_PARAMS:
		ret = power_handle_set_params(arg);
		break;
	case SEMC_POWER_IOC_SET_BATT_STATUS:
	{
		u8 prev_ovp;

		mutex_lock(&power_info.lock);
		prev_ovp = power_info.battery_status.ovp;

		if (copy_from_user(&power_info.battery_status,
				   (struct battery_status *)arg,
				   sizeof power_info.battery_status))
			ret = -EFAULT;
		else if (prev_ovp != power_info.battery_status.ovp) {
			power_info.over_voltage =
				(u8)power_info.battery_status.ovp;
			mutex_unlock(&power_info.lock);
			ret = power_ops.set_ovp(power_info.over_voltage);
			if (ret)
				printk(KERN_INFO POWER_DRV_NAME
					": Failed setting OVP switch!\n");
		} else
			mutex_unlock(&power_info.lock);

		power_supply_changed(&power_info.bat);
		break;
	}
	case SEMC_POWER_IOC_SET_SAFETY_TIMER:
		ret = power_ops.set_safety_timer((u16)arg);
		break;
	case SEMC_POWER_IOC_SET_CHG_STATUS:
	{
		struct charger_status old_status;

		mutex_lock(&power_info.lock);
		old_status = power_info.charger_info.status;

		if (copy_from_user(&power_info.charger_info.status,
				   (struct charger_status *)arg,
				   sizeof power_info.charger_info.status))
			ret = -EFAULT;
		else if (memcmp(&old_status,
				&power_info.charger_info.status,
				sizeof power_info.charger_info.status)) {
			if (power_info.usb_online)
				power_supply_changed(&power_info.usb);
			else if (power_info.ac_online)
				power_supply_changed(&power_info.ac);

			power_supply_changed(&power_info.bat);
		}
		mutex_unlock(&power_info.lock);
		break;
	}
	case SEMC_POWER_IOC_GET_STATUS:
		mutex_lock(&power_info.lock);
		if (copy_to_user((unsigned int *)arg,
				 &power_info.status,
				 sizeof power_info.status))
			ret = -EFAULT;
		else {
			/* Clear status */
			power_info.status = 0x00;
		}
		mutex_unlock(&power_info.lock);
		break;
	case SEMC_POWER_IOC_GET_USB_MAX_CURRENT:
		mutex_lock(&power_info.lock);

		if (copy_to_user((unsigned int *)arg,
				 &power_info.max_usb_current,
				 sizeof(unsigned int)))
			ret = -EFAULT;

		mutex_unlock(&power_info.lock);
		break;
	default:
		printk(KERN_INFO POWER_DRV_NAME
				": Unknown IOCTL command received\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

static int power_get_battery_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	int ret = 0;

	if (!atomic_read(&power_init_ok))
		return -EBUSY;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&power_info.lock);
		val->intval = power_info.charge_status;
		mutex_unlock(&power_info.lock);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = power_get_health();
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = power_get_technology();
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&power_info.lock);
#ifdef DEBUG_FS
		if (power_info.capacity_debug.active)
			val->intval = power_info.capacity_debug.value;
		else
			val->intval = power_info.bdata.cap_percent;
#else
		val->intval = power_info.bdata.cap_percent;
#endif /* DEBUG_FS */
		mutex_unlock(&power_info.lock);
		break;
#ifndef CONFIG_ANDROID
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	{
		u16 value;

#ifdef DEBUG_FS
		if (power_info.voltage_debug.active)
			value = (u16)power_info.voltage_debug.value;
		else
			ret = power_ops.get_battery_voltage(&value);
#else
		ret = power_ops.get_battery_voltage(&value);
#endif /* DEBUG_FS */

		if (!ret) {
			/* Voltage in uV */
			val->intval = (int)value * 1000;
		}

		break;
	}
	case POWER_SUPPLY_PROP_TEMP:
		/* Temperature in tenths of degree Celsius */
		mutex_lock(&power_info.lock);
#ifdef DEBUG_FS
		if (power_info.temperature_debug.active)
			val->intval = power_info.temperature_debug.value * 10;
		else
			val->intval = power_info.bdata.temp_celsius * 10;
#else
		val->intval = power_info.bdata.temp_celsius * 10;
#endif /* DEBUG_FS */
		mutex_unlock(&power_info.lock);

		break;
#endif /* CONFIG_ANDROID */
	default:
		ret = -EINVAL;
		break;
	}


	return ret;
}

static int power_get_power_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		mutex_lock(&power_info.lock);
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = power_info.ac_online;
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = power_info.usb_online;
		else
			val->intval = 0;
		mutex_unlock(&power_info.lock);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = power_get_charger_status(psy->type);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = power_get_charger_health(psy->type);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void power_event_handler(enum event event, void *data)
{
	unsigned int charger_on_before;
	unsigned int charger_on_after;
	struct semc_notify_platform old_notify;
	u8 update_bat = 0;
	u8 update_usb = 0;
	u8 update_ac = 0;

	if (!atomic_read(&power_init_ok)) {
		printk(KERN_INFO POWER_DRV_NAME
			": Got event 0x%x but not fully init\n", event);
		return;
	}

	mutex_lock(&power_info.lock);

	old_notify = power_info.notify;

	if (power_info.usb_online || power_info.ac_online)
		charger_on_before = 1;
	else
		charger_on_before = 0;

	switch (event) {
	case SEMC_POWER_USB_CONNECTED:
	{
		u16 usb_current = power_info.max_usb_current;

		if (data != NULL)
			usb_current = *(u16 *)data;

		if (power_info.max_usb_current != usb_current) {
			power_info.max_usb_current = usb_current;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_USB_MAX_CURRENT_CHANGED;
		}

		if (!power_info.usb_online) {
			power_info.usb_online = 1;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGER_CONNECTION_CHANGED;
			update_usb = 1;
		}
		break;
	}
	case SEMC_POWER_USB_DISCONNECTED:
		if (power_info.usb_online) {
			power_info.usb_online = 0;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGER_CONNECTION_CHANGED;
			update_usb = 1;
		}
		break;
	case SEMC_POWER_USB_CURRENT_AVAILABLE:
	{
		u16 usb_current = power_info.max_usb_current;

		if (data != NULL)
			usb_current = *(u16 *)data;

		if (power_info.max_usb_current != usb_current) {
			power_info.max_usb_current = usb_current;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_USB_MAX_CURRENT_CHANGED;
		}
		break;
	}
	case SEMC_POWER_AC_CONNECTED:
	{
		u16 usb_current = power_info.max_usb_current;

		if (data != NULL)
			usb_current = *(u16 *)data;

		if (power_info.max_usb_current != usb_current) {
			power_info.max_usb_current = usb_current;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_USB_MAX_CURRENT_CHANGED;
		}

		if (!power_info.ac_online) {
			power_info.ac_online = 1;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGER_CONNECTION_CHANGED;
			update_ac = 1;
		}
		break;
	}
	case SEMC_POWER_AC_DISCONNECTED:
		if (power_info.ac_online) {
			power_info.ac_online = 0;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGER_CONNECTION_CHANGED;
			update_ac = 1;
		}
		break;
	case SEMC_POWER_CHARGING_READY:
		if (power_info.charge_status != POWER_SUPPLY_STATUS_DISCHARGING)
		{
			power_info.charge_status =
				POWER_SUPPLY_STATUS_DISCHARGING;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGING_CHANGED;
			update_bat = 1;
		}
		break;
	case SEMC_POWER_CHARGING:
		if (power_info.charge_status != POWER_SUPPLY_STATUS_CHARGING) {
			power_info.charge_status = POWER_SUPPLY_STATUS_CHARGING;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGING_CHANGED;
			update_bat = 1;
		}
		break;
	case SEMC_POWER_CHARGING_DONE:
		if (power_info.charge_status != POWER_SUPPLY_STATUS_FULL) {
			power_info.charge_status = POWER_SUPPLY_STATUS_FULL;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGING_CHANGED;
			power_info.bdata.cap_percent = 100;
			update_bat = 1;
		}
		break;
	case SEMC_POWER_ERROR_CHARGER_OVERVOLTAGE:
		if (power_info.charge_status !=
		    POWER_SUPPLY_STATUS_NOT_CHARGING) {
			power_info.charge_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGING_CHANGED;
			update_bat = 1;
		}

		if (!power_info.charger_info.overvoltage) {
			power_info.charger_info.overvoltage = 1;
			if (power_info.usb_online)
				update_usb = 1;
			else if (power_info.ac_online)
				update_ac = 1;
		}
		break;
	case SEMC_POWER_ERROR_SAFETY_TIMER:
		if (power_info.charge_status !=
		    POWER_SUPPLY_STATUS_NOT_CHARGING) {
			power_info.charge_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGING_CHANGED;
			update_bat = 1;
		}

		if (!power_info.safety_timer_fault) {
			power_info.safety_timer_fault = 1;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_SAFETY_TIMER_FAULT;
			update_bat = 1;
		}
	  break;
	case SEMC_POWER_ERROR_DCOUT:
		if (!power_info.dcout_fault) {
			power_info.dcout_fault = 1;

			if (dcio_over_current_fn)
				dcio_over_current_fn(power_info.dcout_fault);
		}
		break;
	case SEMC_POWER_ERROR_OTHER:
		if (power_info.charge_status !=
		    POWER_SUPPLY_STATUS_NOT_CHARGING) {
			power_info.charge_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			power_info.power_status_updated = 1;
			power_info.status |= STATUS_CHARGING_CHANGED;
			update_bat = 1;
		}
		break;
	case SEMC_POWER_ERROR_NONE:
		if (power_info.charger_info.overvoltage) {
			power_info.charger_info.overvoltage = 0;
			if (power_info.usb_online)
				update_usb = 1;
			else if (power_info.ac_online)
				update_ac = 1;
		}

		if (power_info.dcout_fault) {
			power_info.dcout_fault = 0;

			if (dcio_over_current_fn)
				dcio_over_current_fn(power_info.dcout_fault);
		}

		if (power_info.safety_timer_fault) {
			power_info.safety_timer_fault = 0;
			update_bat = 1;
		}
		break;
	case SEMC_POWER_VBATT_CUTOFF:
		if (!power_info.shutdown_system) {
			power_info.shutdown_system = 1;
			power_ops.platform_callbacks(0);
			update_bat = 1;
		}
		break;
	case SEMC_POWER_BDATA_UPDATE:
		if (data != NULL) {
			power_info.bdata = *(struct battery_data *)data;

			/* As long as a charger is inserted and the charging
			 * status is reported full keep the capacity to show
			 * full, i.e. 100 %.
			 */
			if (power_info.bdata.cap_percent != 100 &&
			    power_info.charge_status ==
			    POWER_SUPPLY_STATUS_FULL &&
			    (power_info.usb_online || power_info.ac_online))
				power_info.bdata.cap_percent = 100;

			power_info.power_status_updated = 1;
			power_info.status |= STATUS_BDATA_CHANGED;
			update_bat = 1;
		}
		break;
	case SEMC_POWER_CPU_ON:
		power_info.notify.power_collapse = 0;
		break;
	case SEMC_POWER_CPU_OFF:
		power_info.notify.power_collapse = 1;
		break;
	default:
		break;
	}

	if (power_info.usb_online || power_info.ac_online)
		charger_on_after = 1;
	else
		charger_on_after = 0;

	if (charger_on_before && !charger_on_after &&
	    power_info.bdata.cap_percent == 100)
		power_info.notify.battery_full = 1;
	else
		power_info.notify.battery_full =
			(power_info.charge_status == POWER_SUPPLY_STATUS_FULL);

	/* Show discharging if no charger is attached */
	if (!charger_on_after &&
	    power_info.charge_status != POWER_SUPPLY_STATUS_DISCHARGING) {
		power_info.charge_status =
			POWER_SUPPLY_STATUS_DISCHARGING;
		power_info.power_status_updated = 1;
		power_info.status |= STATUS_CHARGING_CHANGED;
		update_bat = 1;
	}

	power_info.notify.charging =
		(power_info.charge_status == POWER_SUPPLY_STATUS_CHARGING);

	if (memcmp(&old_notify, &power_info.notify, sizeof power_info.notify))
		power_ops.notify_platform(&power_info.notify);

	if (power_info.power_status_updated) {
		if ((power_info.status & STATUS_CHARGER_CONNECTION_CHANGED) ==
		    STATUS_CHARGER_CONNECTION_CHANGED) {
			if (!charger_on_before && charger_on_after) {
				unsigned int usb_online = power_info.usb_online;
				/* Reset the charging parameters */
				memset(&power_info.charge_params, 0xFF,
				       sizeof power_info.charge_params);
				mutex_unlock(&power_info.lock);
				power_ops.turn_on_charger(usb_online);
			}
			else if (charger_on_before && !charger_on_after) {
				mutex_unlock(&power_info.lock);
				power_ops.turn_off_charger();
			} else {
				mutex_unlock(&power_info.lock);
			}
		} else {
			mutex_unlock(&power_info.lock);
		}

		wake_up_interruptible(&power_info.wq);
	} else
		mutex_unlock(&power_info.lock);

	if (update_bat || update_usb || update_ac)
		wake_lock_timeout(&power_supply_lock, HZ * 2);
	if (update_bat)
		power_supply_changed(&power_info.bat);
	if (update_usb)
		power_supply_changed(&power_info.usb);
	if (update_ac)
		power_supply_changed(&power_info.ac);
}

static int semc_power_check_power_ops(void)
{
	u8 missing_ops = 0;

	if (!power_ops.get_boot_charging_info) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops get_boot_charging_info not initialized\n");
		missing_ops++;
	}

	if (!power_ops.turn_on_charger) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops turn_on_charger not initialized\n");
		missing_ops++;
	}

	if (!power_ops.turn_off_charger) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops turn_off_charger not initialized\n");
		missing_ops++;
	}

	if (!power_ops.enable_charger) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops enable_charger not initialized\n");
		missing_ops++;
	}

	if (!power_ops.disable_charger) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops disable_charger not initialized\n");
		missing_ops++;
	}

	if (!power_ops.enable_dcout) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops enable_dcout not initialized\n");
		missing_ops++;
	}

	if (!power_ops.disable_dcout) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops disable_dcout not initialized\n");
		missing_ops++;
	}

	if (!power_ops.get_dcout_currents) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops get_dcout_currents not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_ovp) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops set_ovp not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_safety_timer) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops set_safety_timer not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_charger_voltage) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops set_charger_voltage not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_charger_current) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops set_charger_current not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_input_charger_current) {
		printk(KERN_ERR POWER_DRV_NAME
		": File ops set_input_charger_current not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_charger_current_termination) {
		printk(KERN_ERR POWER_DRV_NAME
		": File ops set_charger_current_termination not inited\n");
		missing_ops++;
	}

	if (!power_ops.set_charger_maximum_voltage) {
		printk(KERN_ERR POWER_DRV_NAME
		": File ops set_charger_maximum_voltage not initialized\n");
		missing_ops++;
	}

	if (!power_ops.set_charger_maximum_current) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops set_charger_maximum_current not inited\n");
		missing_ops++;
	}

	if (!power_ops.get_battery_voltage) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops get_battery_voltage not initialized\n");
		missing_ops++;
	}

	if (!power_ops.get_battery_thermistor) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops get_battery_thermistor not initialized\n");
		missing_ops++;
	}

	if (!power_ops.sync_hw) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops sync_hw not initialized\n");
		missing_ops++;
	}

	if (!power_ops.sync_platform) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops sync_platform not initialized\n");
		missing_ops++;
	}

	if (!power_ops.notify_platform) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops notify_platform not initialized\n");
		missing_ops++;
	}

	if (!power_ops.platform_callbacks) {
		printk(KERN_ERR POWER_DRV_NAME
		       ": File ops platform_callbacks not initialized\n");
		missing_ops++;
	}

	if (!power_ops.notify_platform_boot_charging_info) {
		printk(KERN_ERR POWER_DRV_NAME
			": File ops notify_platform_boot_charging_info "
			"not initialized\n");
		missing_ops++;
	}

	if (missing_ops)
		return -ENOSYS;
	else
		return 0;
}

int semc_power_get_dcout_currents(struct semc_power_dcout_currents *currents)
{
	if (atomic_read(&power_init_ok))
		return power_ops.get_dcout_currents(currents);
	else
		return -EBUSY;
}
EXPORT_SYMBOL(semc_power_get_dcout_currents);

int semc_power_dcio(u8 enable, u16 current_ma)
{
	int ret = 0;

	printk(KERN_INFO POWER_DRV_NAME ": %s DCOUT. Current: %u\n",
	       enable ? "Enabling" : "Disabling", current_ma);

	if (atomic_read(&power_init_ok)) {
		if (enable)
			ret = power_ops.enable_dcout(current_ma);
		else
			ret = power_ops.disable_dcout();
	} else
		ret = -EBUSY;

	return ret;
}
EXPORT_SYMBOL(semc_power_dcio);

int semc_power_register_dcio_over_current_callback(
	dcio_over_current_callback_t callback)
{
	int ret = 0;

	if (atomic_read(&power_init_ok)) {
		mutex_lock(&power_info.lock);
		dcio_over_current_fn = callback;
		mutex_unlock(&power_info.lock);
	} else
		ret = -EBUSY;

	return ret;
}
EXPORT_SYMBOL(semc_power_register_dcio_over_current_callback);

int semc_power_unregister_dcio_over_current_callback(void)
{
	int ret = 0;

	if (atomic_read(&power_init_ok)) {
		mutex_lock(&power_info.lock);
		dcio_over_current_fn = NULL;
		mutex_unlock(&power_info.lock);
	} else
		ret = -EBUSY;

	return ret;
}
EXPORT_SYMBOL(semc_power_unregister_dcio_over_current_callback);

static int __init power_init(void)
{
	int ret = 0;
	struct semc_power_boot_charging_info charging;

	printk(KERN_INFO POWER_DRV_NAME ": Loading...\n");

	memset(&power_info, 0, sizeof(power_info));

	/* Set to known data. Need to check after setup_[hw, platform]
	 * that everything is initialized.
	 */
	memset(&power_ops, 0, sizeof(power_ops));

	init_waitqueue_head(&power_info.wq);
	mutex_init(&power_info.lock);
	mutex_init(&power_info.charge_param_lock);

	ret = setup_hw(&power_ops, power_event_handler);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME ": Failed setting up HW\n");
		goto probe_exit;
	}
	ret = setup_platform(&power_ops, power_event_handler);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME ": Failed setting up platform\n");
		goto probe_exit_platform;
	}

	/* Check that every file operation is initialized */
	ret = semc_power_check_power_ops();
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME
		       ": Not all file operations registered\n");
		goto probe_exit_power_ops;
	}

	power_info.bat.name            = "battery";
	power_info.bat.type            = POWER_SUPPLY_TYPE_BATTERY;
	power_info.bat.properties      = semc_battery_props;
	power_info.bat.num_properties  = ARRAY_SIZE(semc_battery_props);
	power_info.bat.get_property    = power_get_battery_property;

	power_info.usb.name            = "usb";
	power_info.usb.type            = POWER_SUPPLY_TYPE_USB;
	power_info.usb.supplied_to     = power_supplied_to;
	power_info.usb.num_supplicants = ARRAY_SIZE(power_supplied_to);
	power_info.usb.properties      = power_props;
	power_info.usb.num_properties  = ARRAY_SIZE(power_props);
	power_info.usb.get_property    = power_get_power_property;

	power_info.ac.name            = "ac";
	power_info.ac.type            = POWER_SUPPLY_TYPE_MAINS;
	power_info.ac.supplied_to     = power_supplied_to;
	power_info.ac.num_supplicants = ARRAY_SIZE(power_supplied_to);
	power_info.ac.properties      = power_props;
	power_info.ac.num_properties  = ARRAY_SIZE(power_props);
	power_info.ac.get_property    = power_get_power_property;

	power_info.charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	power_info.bdata.technology = BATTERY_TECHNOLOGY_UNKNOWN;
	power_info.battery_status.temp_status = BATTERY_TEMP_STATUS_NORMAL;
	power_info.bdata.temp_celsius = 20;
	power_info.bdata.cap_percent = 50;

	ret = power_supply_register(NULL, &power_info.bat);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME ": failed to register battery\n");
		goto probe_exit;
	}

	ret = power_supply_register(NULL, &power_info.usb);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME": failed to register usb\n");
		goto probe_exit_usb;
	}

	ret = power_supply_register(NULL, &power_info.ac);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME ": failed to register usb\n");
		goto probe_exit_ac;
	}

	ret = semc_battery_create_attrs(power_info.bat.dev);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME
		       ": failed to create semc battery attrs\n");
		goto probe_exit_semc;
	}

	ret = misc_register(&power_misc);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME ": failed to register misc\n");
		goto probe_exit_misc;
	}

	wake_lock_init(&power_supply_lock, WAKE_LOCK_SUSPEND, POWER_DRV_NAME);
	atomic_set(&power_init_ok, 1);

	/* Sync the HW and platform states */
	power_ops.sync_hw();
	power_ops.sync_platform();

	/* Enable platform callbacks */
	ret = power_ops.platform_callbacks(1);
	if (ret) {
		printk(KERN_ERR POWER_DRV_NAME
		       ": failed to enable platform callbacks\n");
		goto probe_exit_platform_callbacks;
	}

	/* Check if started charging by boot and charging is ongoing */
	power_ops.get_boot_charging_info(&charging);
	power_ops.notify_platform_boot_charging_info(&charging);

	return ret;

probe_exit_platform_callbacks:
	misc_deregister(&power_misc);
probe_exit_misc:
	semc_battery_remove_attrs(power_info.bat.dev);
probe_exit_semc:
	power_supply_unregister(&power_info.ac);
probe_exit_ac:
	power_supply_unregister(&power_info.usb);
probe_exit_usb:
	power_supply_unregister(&power_info.bat);
probe_exit_power_ops:
	teardown_platform();
probe_exit_platform:
	teardown_hw();
probe_exit:
	return ret;
}

static void __exit power_exit(void)
{
	printk(KERN_INFO POWER_DRV_NAME ": Unloading...\n");

	misc_deregister(&power_misc);

	semc_battery_remove_attrs(power_info.bat.dev);

	power_supply_unregister(&power_info.ac);
	power_supply_unregister(&power_info.usb);
	power_supply_unregister(&power_info.bat);

	teardown_hw();
	teardown_platform();
}

module_init(power_init);
module_exit(power_exit);

MODULE_AUTHOR("Imre Sunyi");
MODULE_DESCRIPTION("Power driver for SEMC");

MODULE_LICENSE("GPL");
