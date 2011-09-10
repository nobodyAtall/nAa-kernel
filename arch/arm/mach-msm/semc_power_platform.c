/*
 * Power supply driver for MSM platforms
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB
 * Author: Imre Sunyi <imre.sunyi@sonyericsson.com>
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

/****************** INCLUDE FILES SECTION *************************************/

#include <mach/semc_power_platform.h>
#include <mach/oem_rapi_client.h>
#include <mach/rpc_hsusb.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/semc/power/semc_power.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

/****************** CONSTANT AND MACRO SECTION ********************************/

#define UINT16_MAX (u16)(~((u16)0))

/****************** TYPE DEFINITION SECTION ***********************************/

/****************** LOCAL FUNCTION DECLARATION SECTION ************************/

static void msm_cutoff_level_cb_work(struct work_struct *work);
static void msm_bdata_change_cb_work(struct work_struct *work);
static int msm_suspend(struct platform_device *dev, pm_message_t state);
static int msm_resume(struct platform_device *dev);

/****************** GLOBAL VARIABLE DECLARATION SECTION ***********************/

/****************** LOCAL VARIABLE DECLARATION SECTION ************************/

static event_callback_t event_fn;
static struct msm_rpc_client *rpc_client;
static enum semc_charger usb_connected = NO_CHARGER;
static enum semc_charger usb_disconnecting = NO_CHARGER;
static u16 usb_maximum_current;
static struct battery_data bdata;
static DEFINE_MUTEX(update_lock);
static DECLARE_WORK(cutoff_level_work, msm_cutoff_level_cb_work);
static DECLARE_WORK(bdata_change_work, msm_bdata_change_cb_work);

struct platform_driver semc_power_platform_driver = {
	.suspend = msm_suspend,
	.resume  = msm_resume,
	.driver = {
		.name = SEMC_POWER_PLATFORM_NAME,
	}
};

/****************** FUNCTION DEFINITION SECTION *******************************/

static void msm_handle_connected_usb(void)
{
	switch (usb_connected) {
	case USB_CHARGER:
		event_fn(SEMC_POWER_USB_CONNECTED, &usb_maximum_current);
		usb_disconnecting = usb_connected;
		break;
	case WALL_CHARGER:
		event_fn(SEMC_POWER_AC_CONNECTED, &usb_maximum_current);
		usb_disconnecting = usb_connected;
		break;
	default:
		if (usb_disconnecting == USB_CHARGER)
			event_fn(SEMC_POWER_USB_DISCONNECTED, NULL);
		else if (usb_disconnecting == WALL_CHARGER)
			event_fn(SEMC_POWER_AC_DISCONNECTED, NULL);

		usb_disconnecting = NO_CHARGER;
		break;
	}
}

static void msm_handle_usb_max_current(void)
{
	event_fn(SEMC_POWER_USB_CURRENT_AVAILABLE, &usb_maximum_current);
}

static int msm_rapi_client_get_int(u32 event, u8 event_switch, u32 *value)
{
	int ret = 0;
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;

	memset(&client_arg, 0, sizeof client_arg);

	client_arg.event = event;
	client_arg.in_len = sizeof(event_switch);
	client_arg.input = (char *)&event_switch;
	client_arg.out_len_valid = 1;
	client_arg.output_valid = 1;
	client_arg.output_size = sizeof(*value);

	client_ret.out_len = NULL;
	client_ret.output = NULL;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret < 0)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
		       ": oem_rapi_client_str_func returned error %d\n", ret);
	else if (client_ret.out_len == NULL ||
		 client_ret.output == NULL || *client_ret.out_len == 0 ||
		 *client_ret.out_len > client_arg.output_size)
		ret = -ENOMSG;
	else {
		if (*client_ret.out_len == sizeof(u16))
			*value = *(u16 *)client_ret.output;
		else
			*value = *(u32 *)client_ret.output;
	}

	kfree(client_ret.out_len);
	kfree(client_ret.output);

	return ret;
}

static int msm_set_ovp(u8 onoff)
{
	int ret = 0;
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	u8 remote_set_fail = 0;
	u8 fet_switch_onoff = !onoff; /* OVP 'on' -> FET 'off' and vice versa */

	memset(&client_arg, 0, sizeof client_arg);

	client_arg.event = OEM_RAPI_CLIENT_EVENT_PM_BATT_FET_SWITCH_SET;
	client_arg.in_len = sizeof(fet_switch_onoff);
	client_arg.input = (char *)&fet_switch_onoff;
	client_arg.out_len_valid = 1;
	client_arg.output_valid = 1;
	client_arg.output_size = sizeof(remote_set_fail);

	client_ret.out_len = NULL;
	client_ret.output = NULL;

	printk(KERN_INFO SEMC_POWER_PLATFORM_NAME ": Setting OVP. Batt-FET %s\n",
			fet_switch_onoff ? "on" : "off");

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret < 0)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
		       ": Failed setting OVP switch. Error %d\n", ret);
	else if (client_ret.out_len == NULL ||
		 client_ret.output == NULL ||
		 *client_ret.out_len != client_arg.output_size)
		ret = -ENOMSG;
	else
		remote_set_fail = *(u8 *)client_ret.output;

	kfree(client_ret.out_len);
	kfree(client_ret.output);

	if (remote_set_fail)
		ret = -ENOEXEC;

	return ret;
}

static int msm_get_battery_voltage(u16 *voltage_mv)
{
	int ret = 0;
	u32 int_val;

	ret = msm_rapi_client_get_int(OEM_RAPI_CLIENT_EVENT_BATT_MV_GET,
				      UNIT_MV,
				      &int_val);

	if (!ret) {
		if (int_val <= UINT16_MAX)
			*voltage_mv = (u16)int_val;
		else
			ret = -ERANGE;
	}

	return ret;
}

static int msm_get_battery_current(s16 *current_ma)
{
	int ret = 0;
	u32 int_val;

	ret = msm_rapi_client_get_int(OEM_RAPI_CLIENT_EVENT_BATT_MA_GET,
				      0,
				      &int_val);

	if (!ret) {
		if (int_val <= UINT16_MAX)
			*current_ma = (s16)int_val;
		else
			ret = -ERANGE;
	}

	return ret;
}

static int msm_get_battery_id(enum battery_technology type,
			      enum unit unit,
			      u32 *id)
{
	int ret = 0;
	u32 int_val;
	u16 rapi_event;

	switch (type) {
	case BATTERY_TECHNOLOGY_TYPE1:
	    rapi_event = OEM_RAPI_CLIENT_EVENT_BATT_ID_TYPE1_GET;
	  break;
	case BATTERY_TECHNOLOGY_TYPE2:
	    rapi_event = OEM_RAPI_CLIENT_EVENT_BATT_ID_TYPE2_GET;
	  break;
	default:
	    rapi_event = OEM_RAPI_CLIENT_EVENT_BATT_ID_GET;
	  break;
	}

	ret = msm_rapi_client_get_int(rapi_event, unit, &int_val);

	if (!ret)
		*id = int_val;

	return ret;
}

static void msm_sync_platform(void)
{
	msm_chg_rpc_semc_get_usb_connected(&usb_connected,
					   &usb_maximum_current);

	msm_handle_connected_usb();
}

static void msm_notify_platform(struct semc_notify_platform *notify)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret = 0;

	memset(&client_arg, 0, sizeof client_arg);

	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_NOTIFY_PLATFORM_SET;
	client_arg.in_len = sizeof(*notify);
	client_arg.input = (char *)notify;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
		       ": Failed notify platform. Error %d\n", ret);
}
static void msm_notify_platform_boot_charging_info(
			struct semc_power_boot_charging_info *charging_info)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret = 0;

	memset(&client_arg, 0, sizeof(client_arg));
	memset(&client_ret, 0, sizeof(client_ret));

	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_NOTIFY_BOOT_CHARGING_INFO;
	client_arg.in_len = sizeof(*charging_info);
	client_arg.input = (char *)charging_info;
	ret = oem_rapi_client_streaming_function(rpc_client,
		&client_arg, &client_ret);
	if (ret)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
		": Failed notify platform boot charging info. Error %d\n", ret);

}

static void msm_set_usb_connected(enum semc_charger connected,
				  uint32_t current_ma)
{
	mutex_lock(&update_lock);
	if (usb_connected != connected) {
		usb_connected = connected;

		if (usb_maximum_current != (u16)current_ma) {
			usb_maximum_current = (u16)current_ma;
			printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
			  ": USB max allowed chg current updated to %u mA\n",
			  usb_maximum_current);
		}
		msm_handle_connected_usb();
	} else if (usb_maximum_current != (u16)current_ma) {
		usb_maximum_current = (u16)current_ma;
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
			": USB max allowed chg current updated to %u mA\n",
			usb_maximum_current);

		msm_handle_usb_max_current();
	}
	mutex_unlock(&update_lock);
}

static void msm_cutoff_level_cb_work(struct work_struct *work)
{
	printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
	       ": Battery cut-off level reached\n");
	event_fn(SEMC_POWER_VBATT_CUTOFF, NULL);
}

static void msm_bdata_change_cb_work(struct work_struct *work)
{
	struct battery_data bdata_local;

	mutex_lock(&update_lock);
	bdata_local = bdata;
	mutex_unlock(&update_lock);

	event_fn(SEMC_POWER_BDATA_UPDATE, (void *)&bdata_local);
}

static int msm_cutoff_level_cb(
		struct oem_rapi_client_streaming_func_cb_arg *arg,
		struct oem_rapi_client_streaming_func_cb_ret *ret)
{
	if (arg && arg->event == OEM_RAPI_SERVER_EVENT_CUTOFF_CB_EVENT)
		schedule_work(&cutoff_level_work);

	return 0;
}

static int msm_bdata_change_cb(
	struct oem_rapi_client_streaming_func_cb_arg *arg,
	struct oem_rapi_client_streaming_func_cb_ret *ret)
{
	if (arg && arg->event == OEM_RAPI_SERVER_EVENT_NOTIFY_BDATA_CB_EVENT) {
		if (arg->in_len == sizeof(struct battery_data)) {
			mutex_lock(&update_lock);
			bdata = *(struct battery_data *)arg->input;
			mutex_unlock(&update_lock);
			schedule_work(&bdata_change_work);
		}
	}

	return 0;
}

static int msm_subscribe_cutoff_level(void)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret = 0;
	char dummy;

	memset(&client_arg, 0, sizeof client_arg);
	memset(&client_ret, 0, sizeof client_ret);

	client_arg.event = OEM_RAPI_CLIENT_EVENT_CUTOFF_LEVEL_CB_REGISTER;
	client_arg.cb_func = msm_cutoff_level_cb;
	client_arg.input = &dummy;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
			": Failed register cutoff level. Error %d\n", ret);

	return ret;
}

static int msm_unsubscribe_cutoff_level(void)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret = 0;
	char dummy;

	memset(&client_arg, 0, sizeof client_arg);
	memset(&client_ret, 0, sizeof client_ret);

	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_CUTOFF_LEVEL_CB_UNREGISTER_SET;
	client_arg.input = &dummy;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
			": Failed unregister cutoff level. Error %d\n", ret);

	return ret;
}

static int msm_subscribe_bdata_change(void)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret = 0;
	char dummy;

	memset(&client_arg, 0, sizeof client_arg);
	memset(&client_ret, 0, sizeof client_ret);

	client_arg.event = OEM_RAPI_CLIENT_EVENT_NOTIFY_BDATA_CB_REGISTER_SET;
	client_arg.cb_func = msm_bdata_change_cb;
	client_arg.input = &dummy;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
                       ": Failed register bdata change. Error %d\n", ret);

	return ret;
}

static int msm_unsubscribe_bdata_change(void)
{
	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;
	int ret = 0;
	char dummy;

	memset(&client_arg, 0, sizeof client_arg);
	memset(&client_ret, 0, sizeof client_ret);

	client_arg.event =
		OEM_RAPI_CLIENT_EVENT_NOTIFY_BDATA_CB_UNREGISTER_SET;
	client_arg.input = &dummy;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret)
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
		       ": Failed unregister bdata change. Error %d\n", ret);

	return ret;
}

static int msm_platform_callbacks(u8 enable)
{
	int ret;

	if (enable) {
		ret = msm_subscribe_cutoff_level();
		if (!ret)
			ret = msm_subscribe_bdata_change();
	} else {
		ret = msm_unsubscribe_cutoff_level();
		if (!ret)
			ret = msm_unsubscribe_bdata_change();
	}

	return ret;
}

static int msm_suspend(struct platform_device *dev, pm_message_t state)
{
	if (mutex_is_locked(&update_lock)) {
		printk(KERN_DEBUG SEMC_POWER_PLATFORM_NAME
		       ": Not allowing suspend\n");
		return -EAGAIN;
	}

	event_fn(SEMC_POWER_CPU_OFF, NULL);

	return 0;
}

static int msm_resume(struct platform_device *dev)
{
	event_fn(SEMC_POWER_CPU_ON, NULL);

	return 0;
}

int setup_platform(struct power_ops *platform_ops, event_callback_t fn)
{
	int ret = 0;

	printk(KERN_INFO SEMC_POWER_PLATFORM_NAME ": Setting up platform\n");

	if (platform_ops == NULL) {
		ret = -EINVAL;
		goto setup_exit;
	}

	rpc_client = oem_rapi_client_init();
	if (rpc_client == NULL) {
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
				": Failed initialize oem rapi client\n");
		ret = -EPERM;
		goto setup_exit;
	}

	event_fn = fn;

	platform_ops->set_ovp                 = msm_set_ovp;
	platform_ops->get_battery_voltage     = msm_get_battery_voltage;
	platform_ops->get_battery_current     = msm_get_battery_current;
	platform_ops->get_battery_thermistor  = msm_get_battery_id;
	platform_ops->sync_platform           = msm_sync_platform;
	platform_ops->notify_platform         = msm_notify_platform;
	platform_ops->platform_callbacks      = msm_platform_callbacks;
	platform_ops->notify_platform_boot_charging_info =
		msm_notify_platform_boot_charging_info;

	ret = platform_driver_register(&semc_power_platform_driver);
	if (ret) {
		printk(KERN_INFO SEMC_POWER_PLATFORM_NAME
		       ": Failed register platform driver\n");
		goto setup_exit_close_client;
	}

	msm_chg_rpc_register_semc_callback(msm_set_usb_connected);

setup_exit:
	return ret;

setup_exit_close_client:
	oem_rapi_client_close();

	return ret;
}

int teardown_platform(void)
{
	int ret = 0;

	printk(KERN_INFO SEMC_POWER_PLATFORM_NAME ": Closing down platform\n");

	if (work_pending(&cutoff_level_work))
		cancel_work_sync(&cutoff_level_work);

	if (work_pending(&bdata_change_work))
		cancel_work_sync(&bdata_change_work);

	msm_platform_callbacks(0);
	oem_rapi_client_close();
	rpc_client = NULL;

	msm_chg_rpc_unregister_semc_callback();

	platform_driver_unregister(&semc_power_platform_driver);

	return ret;
}
