/*
   MAX17040 fuel gauge driver
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

      This file is derived from
	htc_battery.c
	HTC Corporation
*/
/* arch/arm/mach-msm/htc_battery.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/max17040.h>
#include <mach/rpc_hsusb.h>

#ifdef CONFIG_SEMC_LOW_BATT_SHUTDOWN
#include <mach/semc_low_batt_shutdown.h>
#endif /* CONFIG_SEMC_LOW_BATT_SHUTDOWN */

#define MAX17040_ADDR		0x36

#define MAX17040_REG_VCELL	0x02
#define MAX17040_REG_SOC	0x04
#define MAX17040_REG_MODE	0x06
#define MAX17040_REG_VERSION	0x08
#define MAX17040_REG_RCOMP	0x0C
#define MAX17040_REG_OCV	0x0E
#define MAX17040_REG_COMMAND	0xFE

#define MAX17040_MODEL_LOCK	0x3E	/* 0x3E,0x3F */
#define MAX17040_MODEL1		0x40	/* 0x40-0x4F */
#define MAX17040_MODEL2		0x50	/* 0x50-0x5F */
#define MAX17040_MODEL3		0x60	/* 0x60-0x6F */
#define MAX17040_MODEL4		0x70	/* 0x70-0x7F */

#define MAX17040_RCOMP_MIN	0
#define MAX17040_RCOMP_MAX	255

#define MAX17040_TEMPERATURE_STANDARD	20

#define MAX17040_SIZE_REG	2	/* 2byte */

/* used delay for load custom model */
#define MAX17040_TIME_WAIT_WRITE_MODEL	150 /* 150ms */
#define MAX17040_TIME_WAIT_UPDATE_SOC	300 /* 300ms */
#define MAX17040_TIME_WAIT_RESTORE_SOC	150 /* 150ms */
/* used delay for resetting fuel gauge */
#define MAX17040_TIME_WAIT_RESET	300 /* 300ms */

#define MAX17040_UPDATE_INTERVAL_SEC	1
#define MAX17040_UPDATE_INTERVAL_USEC	0

#define MAX17040_NUM_RETRY	10	/* Max number of times of the retry */

#define MAX17040_CHARGER_NOT_ONLINE	0
#define MAX17040_CHARGER_ONLINE		1

#define MAX17040_BATTERY_NOT_PRESENT	0
#define MAX17040_BATTERY_PRESENT	1

#define MAX17040_DATA_NOT_UPDATE	0
#define MAX17040_DATA_UPDATE		1

#define MAX17040_MODEL_LOAD_SUCCESS	0
#define MAX17040_MODEL_LOAD_FAILURE	-1

#define MAX17040_PORWER_SUPPLY_NOT_REGISTER	0
#define MAX17040_PORWER_SUPPLY_REGISTER		1

/* Run status of recovery on retry.
 * Used retry of I2C accesss to fuel gauge.
 */
#define MAX17040_RECOVERY_NOT_DONE	0
#define MAX17040_RECOVERY_DONE		1

/* This order is the same as max17040_power_supplies[]
 * And it's also the same as max17040_update_info()
 */
enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
};

struct max17040_battery_info {
	int ac_online;		/* Connection status of AC chager */
	int usb_online;		/* Connection status of USB charger */
	int status;		/* Charging status of battery */
	int present;		/* Connection status of battery */
	int capacity;		/* Remaining capacity of battery */
	int voltage;		/* Voltage of battery */
	int temperature;	/* Temperature of battery */
	int health;		/* Health of battery */
	int low_batt_flag;	/* Flag to detect Low-Battery */
};

struct max17040_backup_data {
	u8 rcomp[MAX17040_SIZE_REG];
	u8 ocv[MAX17040_SIZE_REG];
};

struct i2c_client *max17040_i2c_client;

static struct device *max17040_dev;

static struct max17040_battery_info max17040_info;

static struct max17040_device_data *max17040_data;

static struct work_struct max17040_work;

static DEFINE_MUTEX(max17040_info_lock);

static struct hrtimer max17040_timer;

static int max17040_update_flg = MAX17040_DATA_UPDATE;

static enum power_supply_property max17040_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property max17040_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *max17040_supply_list[] = {
	"battery",
};

/* Fuel gauge dedicated attributes */
static ssize_t max17040_show_property(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static int max17040_get_power_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val);

static int max17040_get_battery_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val);

static struct power_supply max17040_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = max17040_battery_properties,
		.num_properties = ARRAY_SIZE(max17040_battery_properties),
		.get_property = max17040_get_battery_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = max17040_supply_list,
		.num_supplicants = ARRAY_SIZE(max17040_supply_list),
		.properties = max17040_power_properties,
		.num_properties = ARRAY_SIZE(max17040_power_properties),
		.get_property = max17040_get_power_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = max17040_supply_list,
		.num_supplicants = ARRAY_SIZE(max17040_supply_list),
		.properties = max17040_power_properties,
		.num_properties = ARRAY_SIZE(max17040_power_properties),
		.get_property = max17040_get_power_property,
	},
};

/* Registration result to the power supply of max17040_power_supplies[] */
static int max17040_register_flg[] = {
	MAX17040_PORWER_SUPPLY_NOT_REGISTER,	/* battery */
	MAX17040_PORWER_SUPPLY_NOT_REGISTER,	/* usb */
	MAX17040_PORWER_SUPPLY_NOT_REGISTER,	/* ac */
};

static int max17040_get_power_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int res = 0;

	mutex_lock(&max17040_info_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = max17040_info.ac_online;
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = max17040_info.usb_online;
		else
			val->intval = 0;
		break;
	default:
		res = -EINVAL;
	}

	mutex_unlock(&max17040_info_lock);

	return res;
}

static int max17040_get_battery_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	int res = 0;

	mutex_lock(&max17040_info_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max17040_info.status;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max17040_info.health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max17040_info.present;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max17040_info.capacity;
		break;

	default:
		res = -EINVAL;
	}

	mutex_unlock(&max17040_info_lock);

	return res;
}

#define MAX17040_BATTERY_ATTR(_name) \
{ \
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE }, \
	.show = max17040_show_property, \
	.store = NULL, \
}

static struct device_attribute max17040_attrs[] = {
	MAX17040_BATTERY_ATTR(batt_vol),
	MAX17040_BATTERY_ATTR(batt_temp),
	MAX17040_BATTERY_ATTR(charger_current),
	MAX17040_BATTERY_ATTR(qsd_temp),
	MAX17040_BATTERY_ATTR(charger_temp),
};

enum {
	BATT_VOL = 0,
	BATT_TEMP,
	CHARGER_CURRENT,
	QSD_TEMP,
	CHARGER_TEMP,
};

static int max17040_create_attrs(struct device *dev)
{
	unsigned int i;
	int res, retry;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(max17040_attrs); i++) {
		for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
			res = device_create_file(dev, &max17040_attrs[i]);
			/* Succeed to create attribute */
			if (res == 0)
				break;
		}

		/* Fail all retry that create attribute */
		if (res != 0) {
			dev_err(max17040_dev,
				"Failed to create attribute \"%s\" (%d)\n",
				max17040_attrs[i].attr.name, res);
			ret = res;
		}
	}

	return ret;
}

static ssize_t max17040_show_property(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t i = 0;
	const ptrdiff_t off = attr - max17040_attrs;

	mutex_lock(&max17040_info_lock);

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i,
			       PAGE_SIZE - i,
			       "%d\n",
			       max17040_info.voltage);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i,
			       PAGE_SIZE - i,
			       "%d\n",
			       max17040_info.temperature * 10);
		break;
	case CHARGER_CURRENT:
		i = sprintf(buf, "%d\n", msm_chg_charger_current());
		break;
	case QSD_TEMP:
		i = sprintf(buf, "%d\n", msm_chg_qsd_thermo()-30);
		break;
	case CHARGER_TEMP:
		i = sprintf(buf, "%d\n", msm_chg_charger_thermo()-30);
		break;
	default:
		i = -EINVAL;
	}

	mutex_unlock(&max17040_info_lock);

	return i;
}

static int max17040_msleep(long msec) /* Max of msec is 999 */
{
	struct timespec treq, trem;
	int res;

	treq.tv_sec = 0;
	treq.tv_nsec = msec * 1000000; /* Covert to nsec from msec */

	do {
		res = hrtimer_nanosleep(&treq,
					&trem,
					HRTIMER_MODE_REL,
					CLOCK_MONOTONIC);
		treq.tv_sec = trem.tv_sec;
		treq.tv_nsec = trem.tv_nsec;
	} while (res == -EINTR);

	return res;
}

static int max17040_lock_model(void)
{
	int res;

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					     MAX17040_MODEL_LOCK,
					     ARRAY_SIZE(max17040_data->lock),
					     max17040_data->lock);
	if (res < 0)
		dev_err(max17040_dev, "Failed to lock custom model\n");

	return res;
}

static int max17040_unlock_model(void)
{
	int res;

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					     MAX17040_MODEL_LOCK,
					     ARRAY_SIZE(max17040_data->unlock),
					     max17040_data->unlock);
	if (res < 0)
		dev_err(max17040_dev, "Failed to unlock custom model\n");

	return res;
}

static int max17040_write_model(void)
{
	int res;

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					MAX17040_MODEL1,
					ARRAY_SIZE(max17040_data->model[0]),
					max17040_data->model[0]);
	if (res < 0) {
		dev_err(max17040_dev,
			"Failed to write custom model (addr=0x%02x-0x%02x)\n",
			MAX17040_MODEL1, MAX17040_MODEL1 + 0x0F);
		goto error;
	}

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					MAX17040_MODEL2,
					ARRAY_SIZE(max17040_data->model[1]),
					max17040_data->model[1]);
	if (res < 0) {
		dev_err(max17040_dev,
			"Failed to write custom model (addr=0x%02x-0x%02x)\n",
			MAX17040_MODEL2, MAX17040_MODEL2 + 0x0F);
		goto error;
	}

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					MAX17040_MODEL3,
					ARRAY_SIZE(max17040_data->model[2]),
					max17040_data->model[2]);
	if (res < 0) {
		dev_err(max17040_dev,
			"Failed to write custom model (addr=0x%02x-0x%02x)\n",
			MAX17040_MODEL3, MAX17040_MODEL3 + 0x0F);
		goto error;
	}

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					MAX17040_MODEL4,
					ARRAY_SIZE(max17040_data->model[3]),
					max17040_data->model[3]);
	if (res < 0)
		dev_err(max17040_dev,
			"Failed to write custom model (addr=0x%02x-0x%02x)\n",
			MAX17040_MODEL4, MAX17040_MODEL4 + 0x0F);

error:
	return res;
}

/* Reset fuel gauge */
static void max17040_reset(void)
{
	u8 reset_command[] = {(u8)0x54, (u8)0x00};
	int res;

	dev_info(max17040_dev, "Reset fuel gauge\n");

	res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
					     MAX17040_REG_COMMAND,
					     MAX17040_SIZE_REG,
					     reset_command);
	if (res < 0) {
			dev_err(max17040_dev,
				"Failed to write COMMAND register "
				"for reset of fuel gauge\n");
			return;
	}

	/* Delay to fuel gauge to be reseted */
	res = max17040_msleep(MAX17040_TIME_WAIT_RESET);
	if (res != 0)
		dev_err(max17040_dev,
			"Failed delay for resetting fuel gauge\n");
}

static void max17040_load_model(void)
{
	struct max17040_backup_data backup_data;
	u8 verify_data[MAX17040_SIZE_REG];
	u8 max_rcomp[] = {(u8)0xFF, (u8)0x00};
	int res;
	int retry;

	dev_info(max17040_dev, "Load custom model\n");

	for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
		/* Unlock access of custom model */
		res = max17040_unlock_model();
		if (res < 0)
			goto error;

		/* Read original RCOMP register and original OCV register */
		res = i2c_smbus_read_i2c_block_data(max17040_i2c_client,
						    MAX17040_REG_RCOMP,
						    MAX17040_SIZE_REG * 2,
						    (u8 *)(&backup_data));
		if (res < 0) {
			dev_err(max17040_dev,
				"Failed to backup "
				"RCOMP register and OCV register\n");
			goto error;
		}

		/* Write greatest OCV value for custom model to OCV register */
		res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
						MAX17040_REG_OCV,
						MAX17040_SIZE_REG,
						max17040_data->greatest_ocv);
		if (res < 0) {
			dev_err(max17040_dev,
				"Failed to write greatest OCV value "
				"for custom model to OCV register "
				"before writing custom model\n");
			goto error;
		}

		/* Write max value to RCOMP register */
		res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
						     MAX17040_REG_RCOMP,
						     MAX17040_SIZE_REG,
						     max_rcomp);
		if (res < 0) {
			dev_err(max17040_dev,
				"Failed to write max value "
				"to RCOMP register\n");
			goto error;
		}

		/* Write custom model */
		res = max17040_write_model();
		if (res < 0)
			goto error;

		/*
		 * Wait to finish writing the model data
		 * because the model data is big
		 */
		res = max17040_msleep(MAX17040_TIME_WAIT_WRITE_MODEL);
		if (res != 0) {
			dev_err(max17040_dev,
				"Failed waiting that "
				"writing of model data is finished\n");
			goto error;
		}

		/* Write greatest OCV value for custom model to OCV register */
		res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
						MAX17040_REG_OCV,
						MAX17040_SIZE_REG,
						max17040_data->greatest_ocv);
		if (res < 0) {
			dev_err(max17040_dev,
				"Failed to write greatest OCV value "
				"for custom model to OCV register "
				"before reading SOC register\n");
			goto error;
		}

		/*
		 * Wait that
		 * the update of OCV register is reflected in SOC register
		 */
		res = max17040_msleep(MAX17040_TIME_WAIT_UPDATE_SOC);
		if (res != 0) {
			dev_err(max17040_dev,
				"Failed waiting that "
				"the update of OCV register "
				"is reflected in SOC register\n");
			goto error;
		}

		/* Read value for vefiring model from SOC register */
		res = i2c_smbus_read_i2c_block_data(max17040_i2c_client,
						     MAX17040_REG_SOC,
						     MAX17040_SIZE_REG,
						     verify_data);
		if (res < 0) {
			dev_err(max17040_dev,
				"Failed to read SOC register "
				"for verifying custom model\n");
			goto error;
		}

		dev_info(max17040_dev,
			"SOC register's value that verify custom model : "
			"0x%02x%02x\n",
			verify_data[0], verify_data[1]);
		/* Compare SOC value */
		if ((verify_data[0] >= max17040_data->load_result.min) &&
		    (verify_data[0] <= max17040_data->load_result.max)) {

			/* Succeed to load custom model */
			dev_info(max17040_dev,
				"Succeeded to verify custom model. "
				"SOC register's value match desired value.\n");
		} else {
			/* Fail to load custom model */
			dev_err(max17040_dev,
				"Failed to verify custom model. "
				"SOC register's value don't match "
				"desired value.\n");
			goto error;
		}

		/* Restore RCOMP register */
		res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
						     MAX17040_REG_RCOMP,
						     MAX17040_SIZE_REG * 2,
						     (u8 *)(&backup_data));
		if (res < 0) {
			dev_err(max17040_dev,
				"Failed to restore "
				"RCOMP register and OCV register\n");
			goto error;
		}

		/*
		 * Wait that
		 * the restore of OCV register is reflected in SOC register
		 */
		res = max17040_msleep(MAX17040_TIME_WAIT_RESTORE_SOC);
		if (res != 0) {
			dev_err(max17040_dev,
				"Failed waiting that the restore of OCV "
				"register is reflected in SOC register\n");
			goto error;
		}

		/* Lock access of custom model */
		res  = max17040_lock_model();
		if (res < 0)
			goto error;

		break;

error:
		max17040_reset();
	}

	/* Fail all retry that load custom model */
	if (retry > MAX17040_NUM_RETRY)
		dev_err(max17040_dev, "Failed to load custom model\n");
}

/* Initialize fuel gauge */
static void max17040_recovery(void)
{
	dev_info(max17040_dev, "Recover of fuel gauge\n");

	max17040_reset();

	/* Reload custum model */
	max17040_load_model();
}

static void max17040_update_rcomp(void)
{
	u8 new_rcomp[MAX17040_SIZE_REG];
	int tmp_rcomp, res, retry;
	int recovery = MAX17040_RECOVERY_NOT_DONE;

	if (max17040_info.temperature > MAX17040_TEMPERATURE_STANDARD)
		tmp_rcomp = max17040_data->rcomp.init
			    + (max17040_info.temperature
			       - MAX17040_TEMPERATURE_STANDARD)
			    * max17040_data->rcomp.temp_co_hot
			    / max17040_data->rcomp.temp_div;
	else if (max17040_info.temperature < MAX17040_TEMPERATURE_STANDARD)
		tmp_rcomp = max17040_data->rcomp.init
			    + (max17040_info.temperature
			       - MAX17040_TEMPERATURE_STANDARD)
			    * max17040_data->rcomp.temp_co_cold
			    / max17040_data->rcomp.temp_div;
	else
		tmp_rcomp = max17040_data->rcomp.init;

	if (tmp_rcomp > MAX17040_RCOMP_MAX)
		tmp_rcomp = MAX17040_RCOMP_MAX;

	if (tmp_rcomp < MAX17040_RCOMP_MIN)
		tmp_rcomp = MAX17040_RCOMP_MIN;

	new_rcomp[0] = (u8)tmp_rcomp;
	new_rcomp[1] = (u8)0;

	/* Set new value to RCOMP register */
	for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
		res = i2c_smbus_write_i2c_block_data(max17040_i2c_client,
						     MAX17040_REG_RCOMP,
						     MAX17040_SIZE_REG,
						     new_rcomp);
		/* Succeed to read RCOMP register */
		if (res >= 0)
			break;

		/*
		 * Count of retry exceed max,
		 * and fuel gauge don't yet do recovery
		 */
		if (retry == MAX17040_NUM_RETRY &&
		    recovery == MAX17040_RECOVERY_NOT_DONE) {

			max17040_recovery();
			recovery = MAX17040_RECOVERY_DONE;
			/* Reset count of retry for re-retry */
			retry = 0;
		}
	}

	/* Fail all retry that read RCOMP register */
	if (res < 0)
		dev_err(max17040_dev, "Failed to set RCOMP register\n");
}

static void max17040_update_status(void)
{
	int charger_status;
	int retry;
	int prev_status = max17040_info.status;

	for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
		charger_status = msm_hsusb_chg_is_charging();
		/* Succeed to get charger status */
		if (charger_status >= 0)
			break;
	}

	switch (charger_status) {
	case 0:
		max17040_info.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;

	case 1:
		if (max17040_info.capacity >= 100)
			max17040_info.status = POWER_SUPPLY_STATUS_FULL;
		else
			max17040_info.status = POWER_SUPPLY_STATUS_CHARGING;

		break;

	default:
		max17040_info.status = POWER_SUPPLY_STATUS_UNKNOWN;
		dev_err(max17040_dev,
			"Battery status gotten from USB driver "
			"is invalid value.\n");
	}

	if (max17040_info.status != prev_status)
		max17040_update_flg = MAX17040_DATA_UPDATE;
}

static void max17040_update_voltage(void)
{
	u8 vcell[MAX17040_SIZE_REG];
	int res;
	int retry;
	int prev_present = max17040_info.present;
	int prev_health = max17040_info.health;
	int recovery = MAX17040_RECOVERY_NOT_DONE;

	for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
		res = i2c_smbus_read_i2c_block_data(max17040_i2c_client,
						    MAX17040_REG_VCELL,
						    MAX17040_SIZE_REG,
						    vcell);
		/* Succeed to read VCELL register */
		if (res >= 0)
			break;

		/*
		 * Count of retry exceed max,
		 * and fuel gauge don't yet do recovery
		 */
		if (retry == MAX17040_NUM_RETRY &&
		    recovery == MAX17040_RECOVERY_NOT_DONE) {

			max17040_recovery();
			recovery = MAX17040_RECOVERY_DONE;
			/* Reset count of retry for re-retry */
			retry = 0;
		}
	}

	/* Succeed to read VCELL register */
	if (res >= 0) {
		/* Convert to mV from value of VCELL register (UNITS:1.25mV) */
		max17040_info.voltage
			= ((int)vcell[0] << 4) + ((int)vcell[1] >> 4);
		max17040_info.voltage += (max17040_info.voltage >> 2);

		if (max17040_info.voltage > max17040_data->voltage.max)
			dev_err(max17040_dev,
				"Battery voltage value is bigger than %dmV\n",
				max17040_data->voltage.max);

		if (max17040_info.voltage > 0)
			max17040_info.present = MAX17040_BATTERY_PRESENT;
		else
			max17040_info.present = MAX17040_BATTERY_NOT_PRESENT;

		/* Update health */
		if (max17040_info.voltage >=
		    max17040_data->voltage.over_voltage)
			max17040_info.health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (max17040_info.voltage <= max17040_data->voltage.dead)
			max17040_info.health = POWER_SUPPLY_HEALTH_DEAD;
		else
			max17040_info.health = POWER_SUPPLY_HEALTH_GOOD;

		if (max17040_info.present != prev_present ||
		    max17040_info.health != prev_health)
			max17040_update_flg = MAX17040_DATA_UPDATE;

	/* Fail all retry that read VCELL register */
	} else {
		dev_err(max17040_dev, "Failed to read VCELL register\n");
	}
}
static void max17040_update_temperature(void)
{
	int retry;
	int prev_temperature = max17040_info.temperature;

	for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
		max17040_info.temperature = msm_chg_battery_thermo();
		/* Succeed to get battery temperature */
		if (max17040_info.temperature >= 0)
			break;
	}

	/*
	 * When msm_chg_battery_thermo failed,
	 * return value becomes the negative value
	 * When succeded,
	 * return value is 0 or positive value (= <real temperature> + 30).
	 * 		<real temperature> = <return value> - 30
	 */
	if (max17040_info.temperature >= 0) {
		max17040_info.temperature -= 30;
	/* Fail all retry that get battery temperature */
	} else {
		max17040_info.temperature = prev_temperature;
		dev_err(max17040_dev, "Failed to get temperature\n");
	}
}

static void max17040_update_capacity(void)
{
	u8 soc[MAX17040_SIZE_REG];
	int res;
	int prev_capacity = max17040_info.capacity;
	int retry;
	int recovery = MAX17040_RECOVERY_NOT_DONE;

	if (max17040_info.low_batt_flag == 1) {
		max17040_info.capacity = 0;
		dev_dbg(max17040_dev, "battery level is low ---> Power Down\n");
		goto end;
	}

	max17040_update_rcomp();

	for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
		res = i2c_smbus_read_i2c_block_data(max17040_i2c_client,
						    MAX17040_REG_SOC,
						    MAX17040_SIZE_REG,
						    soc);
		/* Succeed to read SOC register */
		if (res >= 0)
			break;

		/*
		 * Count of retry exceed max,
		 * and fuel gauge don't yet do recovery
		 */
		if (retry == MAX17040_NUM_RETRY &&
		    recovery == MAX17040_RECOVERY_NOT_DONE) {
			max17040_recovery();
			recovery = MAX17040_RECOVERY_DONE;
			/* Reset count of retry for re-retry */
			retry = 0;
		}
	}

	/* Succeed to read SOC register */
	if (res >= 0) {
		/* Convert to battery capacity from value of SOC register */
		max17040_info.capacity
			= (((int)soc[0] << 8) + (int)soc[1]) >> 9;

		if (max17040_info.capacity > 100)
			max17040_info.capacity = 100;

end:
		if (max17040_info.capacity != prev_capacity)
			max17040_update_flg = MAX17040_DATA_UPDATE;
	/* Fail all retry that read SOC register */
	} else {
		dev_err(max17040_dev, "Failed to read SOC register\n");
	}
}

static void max17040_notify_change(void)
{
	/* Succeed registration of power supply */
	if (max17040_register_flg[CHARGER_BATTERY] ==
	    MAX17040_PORWER_SUPPLY_REGISTER)
		/* Notify a Power supply driver of a change of battery info */
		power_supply_changed(
			&max17040_power_supplies[CHARGER_BATTERY]);

	/* Succeed registration of power supply */
	if (max17040_register_flg[CHARGER_USB] ==
	    MAX17040_PORWER_SUPPLY_REGISTER)
		/* Notify a Power supply driver of a change of battery info */
		power_supply_changed(&max17040_power_supplies[CHARGER_USB]);

	/* Succeed registration of power supply */
	if (max17040_register_flg[CHARGER_AC] ==
	    MAX17040_PORWER_SUPPLY_REGISTER)
		/* Notify a Power supply driver of a change of battery info */
		power_supply_changed(&max17040_power_supplies[CHARGER_AC]);
}

static void max17040_update_info(struct work_struct *work)
{
	mutex_lock(&max17040_info_lock);

	/* Update each battery info */
	max17040_update_voltage();
	max17040_update_temperature();
	max17040_update_capacity();
	max17040_update_status();

	if (max17040_update_flg == MAX17040_DATA_UPDATE) {
		max17040_notify_change();
		max17040_update_flg = MAX17040_DATA_NOT_UPDATE;
	}

	mutex_unlock(&max17040_info_lock);
}

static enum hrtimer_restart max17040_timer_work(struct hrtimer *timer)
{
	/* Register the update process with work queue */
	schedule_work(&max17040_work);

	/* Restart periodic update */
	hrtimer_start(&max17040_timer,
		      ktime_set(MAX17040_UPDATE_INTERVAL_SEC,
				MAX17040_UPDATE_INTERVAL_USEC),
		      HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static void max17040_update_online(enum semc_charger connected,
				   uint32_t current_ma)
{
	int prev_ac_online	= max17040_info.ac_online;
	int prev_usb_online	= max17040_info.usb_online;

	mutex_lock(&max17040_info_lock);

	switch (connected) {
	case NO_CHARGER:
		max17040_info.ac_online	= MAX17040_CHARGER_NOT_ONLINE;
		max17040_info.usb_online	= MAX17040_CHARGER_NOT_ONLINE;
		break;
	case USB_CHARGER:
		max17040_info.ac_online	= MAX17040_CHARGER_NOT_ONLINE;
		max17040_info.usb_online	= MAX17040_CHARGER_ONLINE;
		break;
	case WALL_CHARGER:
		max17040_info.ac_online	= MAX17040_CHARGER_ONLINE;
		max17040_info.usb_online	= MAX17040_CHARGER_NOT_ONLINE;
		break;
	default:
		dev_err(max17040_dev,
			"Power source gotten from USB driver "
			"is invalid value.\n");
	}

	mutex_unlock(&max17040_info_lock);

	if (max17040_info.ac_online  != prev_ac_online ||
	    max17040_info.usb_online != prev_usb_online)
		max17040_notify_change();
}

static int max17040_suspend(struct i2c_client *client, pm_message_t mesg)
{
	dev_dbg(max17040_dev, "suspend\n");

	/* Stop periodic update */
	hrtimer_cancel(&max17040_timer);

	/* Cancel any scheduled work */
	cancel_work_sync(&max17040_work);

	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
	dev_dbg(max17040_dev, "resume\n");

	/* Restart periodic update */
	hrtimer_start(&max17040_timer,
		      ktime_set(0, 0),
		      HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_SEMC_LOW_BATT_SHUTDOWN
static void max17040_low_batt_callback_func(void)
{
	max17040_info.low_batt_flag = 1;

	mutex_lock(&max17040_info_lock);
	max17040_update_capacity();
	max17040_notify_change();
	mutex_unlock(&max17040_info_lock);
}
#endif /* CONFIG_SEMC_LOW_BATT_SHUTDOWN */

static int max17040_probe(struct i2c_client *client,
			  const struct i2c_device_id *device_id)
{
	unsigned int i;
	int res, retry;
	struct max17040_i2c_platform_data *pdata = client->dev.platform_data;

	max17040_i2c_client = client;
	max17040_dev = &(client->dev);
	max17040_data = pdata->data;

	dev_info(max17040_dev, "max17040 fuel gauge driver probe\n");

	/* Initialize power supplier framework */
	for (i = 0; i < ARRAY_SIZE(max17040_power_supplies); i++) {
		for (retry = 0; retry <= MAX17040_NUM_RETRY; retry++) {
			res = power_supply_register(
				max17040_dev, &max17040_power_supplies[i]);
			/* Succeed to initialize */
			if (res == 0)
				break;
		}

		/* Succeed to initialize */
		if (res == 0)
			max17040_register_flg[i]
				= MAX17040_PORWER_SUPPLY_REGISTER;
		/* Fail all retry that initialize */
		else
			dev_err(max17040_dev,
				"Failed to register power supplier "
				"\"%s\" (%d)\n",
				max17040_power_supplies[i].name, res);
	}


	/* Load custom model */
	max17040_load_model();

	/* Create peculiar attributes */
	res = max17040_create_attrs(
			max17040_power_supplies[CHARGER_BATTERY].dev);

	/* Start periodic update */
	INIT_WORK(&max17040_work, max17040_update_info);

	hrtimer_init(&max17040_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	max17040_timer.function = max17040_timer_work;

	hrtimer_start(&max17040_timer,
					ktime_set(0, 0),
					HRTIMER_MODE_REL);

#ifdef CONFIG_SEMC_LOW_BATT_SHUTDOWN
	res = lbs_register_cb_func(max17040_low_batt_callback_func);
	if (res != 0)
		printk(KERN_ERR "%s: lbs_register_cb_func failed\n", __func__);
#endif /* CONFIG_SEMC_LOW_BATT_SHUTDOWN */

	return 0;
}

static const struct i2c_device_id max17040_id[] = {
	{ "max17040_fuel_gauge", 0 },
	{ "", 0 },
};

static struct i2c_driver max17040_driver = {
	.probe		= max17040_probe,
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
	.driver	= {
		.name	= "max17040_fuel_gauge",
		.owner	= THIS_MODULE,
	},
};

static int __init max17040_init(void)
{
	printk(KERN_INFO "max17040 fuel gauge driver init\n");
	/* register for usb status changed callback */
	msm_chg_rpc_register_semc_callback(max17040_update_online);
	return i2c_add_driver(&max17040_driver);
}

static void __exit max17040_exit(void)
{
	printk(KERN_INFO "max17040 fuel gauge driver exit\n");
	msm_chg_rpc_unregister_semc_callback();
	i2c_del_driver(&max17040_driver);
}

module_init(max17040_init);
module_exit(max17040_exit);
MODULE_DESCRIPTION("MAX17040 Fuel gauge Driver");
MODULE_LICENSE("GPL");
