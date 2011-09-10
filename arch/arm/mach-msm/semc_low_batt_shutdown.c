/* arch/arm/mach-msm/semc_low_batt_shutdown.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB. All Rights Reserved.
 *
 * Author: Yukito Naganuma <Yukito.X.Naganuma@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 *    This file is derived from
 *      drivers/power/msm_battery.c
 *      Code Aurora Forum
*/


#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/semc_low_batt_shutdown.h>

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001


#define BATTERY_REGISTER_PROC				2
#define BATTERY_MODIFY_CLIENT_PROC			4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_ENABLE_DISABLE_FILTER_PROC		14

#define VBATT_NO_FILTER			0

#define BATTERY_CB_TYPE_PROC		1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATT_RPC_TIMEOUT	10000	/* 10 sec */

#define INVALID_BATT_HANDLE	-1

#define BATT_IS_NOT_LOW		0
#define BATT_IS_LOW		1

#define DEBUG 0

#if DEBUG
#define DBG(x...) pr_info(x)
#else
#define DBG(x...) do {} while (0)
#endif


enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};


struct msm_battery_info {
	u32 batt_api_version;
	s32 batt_handle;

	struct msm_rpc_client *batt_client;

	struct wake_lock wlock;
	int threshold_vol;
	int batt_is_low;
	void (*cb_func)(void);
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.batt_is_low = BATT_IS_NOT_LOW,
};

static DEFINE_MUTEX(lbs_mlock);

struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req = data;
	u32 *req = buf;
	int size = 0;

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = data;
	buf_ptr = buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
		u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify Vbatt client\n",
			__func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
			__func__, rep.result);
		return -EIO;
	}

	return 0;
}

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,
		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req = data;
	u32 *req = buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = data;
	buf_ptr = buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct msm_batt_vbatt_filter_req vbatt_filter_req;
	struct msm_batt_vbatt_filter_rep vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
			__func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
			__func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_info("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req = data;
	u32 *req = buf;
	int size = 0;


	*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->cb_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->more_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_error);
	size += sizeof(u32);

	return size;
}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;

	data_ptr = data;
	buf_ptr = buf;

	data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);

	return 0;
}

static int msm_batt_register(u32 desired_batt_voltage,
			u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_rep batt_reg_rep;
	int rc;

	batt_reg_req.desired_batt_voltage = desired_batt_voltage;
	batt_reg_req.voltage_direction = voltage_direction;
	batt_reg_req.batt_cb_id = batt_cb_id;
	batt_reg_req.cb_data = cb_data;
	batt_reg_req.more_data = 1;
	batt_reg_req.batt_error = 0;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, &batt_reg_req,
			msm_batt_register_ret_func, &batt_reg_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	pr_info("%s: got handle = %d\n", __func__, msm_batt_info.batt_handle);
	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req = data;
	u32 *req = buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
					void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = data;
	buf_ptr = buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
			__func__, rep.batt_error, batt_handle);
		return -EIO;
	}
	return 0;
}

static int msm_batt_cleanup(void)
{
	int rc = 0;

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
				__func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);

	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
				void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
			__func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS) {
		DBG("%s: callback received (threashold voltage: %dmV)!!\n",
			__func__, msm_batt_info.threshold_vol);

		msm_batt_info.batt_is_low = !msm_batt_info.batt_is_low;

		if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
			rc = msm_batt_modify_client(msm_batt_info.batt_handle,
					msm_batt_info.threshold_vol,
					(msm_batt_info.batt_is_low ?
					 BATTERY_VOLTAGE_ABOVE_THIS_LEVEL :
					 BATTERY_VOLTAGE_BELOW_THIS_LEVEL),
					BATTERY_CB_ID_LOW_VOL,
					msm_batt_info.threshold_vol);
			if (rc < 0) {
				printk(KERN_ERR
					"%s(): failed to modify client for"
					" registering call back when voltage"
					" goes below %u\n",
					__func__, msm_batt_info.threshold_vol);
			}
		}
	}

	return rc;
}

int lbs_register_cb_func(void (*func)(void))
{
	mutex_lock(&lbs_mlock);
	if (msm_batt_info.cb_func != NULL) {
		mutex_unlock(&lbs_mlock);
		printk(KERN_ERR "%s: callback function is already set.\n",
				__func__);
		return -EBUSY;
	}

	msm_batt_info.cb_func = func;
	mutex_unlock(&lbs_mlock);

	return 0;
}
EXPORT_SYMBOL(lbs_register_cb_func);

static int __devinit lbs_probe(struct platform_device *pdev)
{
	int rc;
	struct lbs_platform_data *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev,
			"%s: platform_data is not set", __func__);
		return -EINVAL;
	}
	msm_batt_info.threshold_vol = pdata->threshold_vol;

	rc = msm_batt_register(msm_batt_info.threshold_vol,
				BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL,
				msm_batt_info.threshold_vol);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc = msm_batt_enable_filter(VBATT_NO_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	wake_lock_init(&msm_batt_info.wlock, WAKE_LOCK_SUSPEND,
					"low-power shutdown");

	return 0;
}

static int __devexit lbs_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup failed rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

#ifdef CONFIG_PM
static int lbs_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk(KERN_INFO "%s(): going to suspend\n", __func__);

	if (msm_batt_info.batt_is_low == BATT_IS_LOW) {
		printk(KERN_INFO "%s(): battery is already low -> Shutdown.\n",
						__func__);
		mutex_lock(&lbs_mlock);
		if (msm_batt_info.cb_func != NULL)
			msm_batt_info.cb_func();
		mutex_unlock(&lbs_mlock);

		return -1;
	}

	return 0;
}

static int lbs_resume(struct platform_device *pdev)
{
	printk(KERN_INFO "%s(): going to resume\n", __func__);

	if (msm_batt_info.batt_is_low == BATT_IS_LOW) {
		printk(KERN_INFO
			"%s: battery is low in sleep -> Shutdown.\n",
			__func__);
		mutex_lock(&lbs_mlock);
		if (msm_batt_info.cb_func != NULL)
			msm_batt_info.cb_func();
		mutex_unlock(&lbs_mlock);
		wake_lock_timeout(&msm_batt_info.wlock, (HZ * 60));
	}

	return 0;
}
#endif /* CONFIG_PM */

static struct platform_driver lbs_driver = {
	.probe = lbs_probe,
	.remove = __devexit_p(lbs_remove),
	.driver = {
		   .name = "Low-Battery Shutdown",
		   .owner = THIS_MODULE,
		   },
};

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
					BATTERY_RPC_VER_2_1,
					1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
			__func__);
		return -ENODEV;
	} else if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version = BATTERY_RPC_VER_1_1;
	} else
		msm_batt_info.batt_api_version = BATTERY_RPC_VER_2_1;

	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
			__func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}

#ifdef CONFIG_PM
	lbs_driver.suspend = lbs_suspend;
	lbs_driver.resume = lbs_resume;
#endif /* CONFIG_PM */

	rc = platform_driver_register(&lbs_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
			__func__, rc);

	return rc;
}

static int __init lbs_init(void)
{
	int rc;

	pr_info("%s: enter\n", __func__);

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc. rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Battery = 0x%08x (RPC version)\n", __func__,
		msm_batt_info.batt_api_version);
	return 0;
}

static void __exit lbs_exit(void)
{
	platform_driver_unregister(&lbs_driver);
}

late_initcall(lbs_init);
module_exit(lbs_exit);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Low-Battery Shutdown Driver");

