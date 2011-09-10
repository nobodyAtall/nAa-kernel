/*
 *
 *
 * Copyright (C) 2009 SonyEricsson AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 */
/****************** INCLUDE FILES SECTION *************************************/

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <mach/msm_rpcrouter.h>
#include <mach/pmic.h>
#include <mach/misc_modem_api.h>
#include <linux/miscdevice.h>
#include <linux/semcclass.h>
#include <linux/fs.h>
#include <mach/vreg.h>
/****************** CONSTANT AND MACRO SECTION ********************************/

#define DEBUG_LEVEL 0

#define TIMEOUT_IMMEDIATELY (1 * HZ)
#define TIMEOUT_INFINITE    -1

#define PMIC_APISPROG 0x30000061
#define PMIC_APISVERS 0x00010001
#define ADC_RPC_TIMEOUT (5 * HZ)

#define ONCRPC_ADC_READ_PROC 100
#define ONCRPC_VREG_CONTROL_PROC 101
#define ONCRPC_ENABLE_CAMERA_FLASH_PROC 102
#define ONCRPC_UNSECURE_CONFIG_DIGITAL_OUT_PROC 103

#define QC_ONCRPC_ADC_READ_PROC 2
#define QC_ADC_READ_APISPROG 0x30000071
#define QC_ADC_READ_APISVERS 0x00010001

/****************** TYPE DEFINITION SECTION ***********************************/
struct misc_api_rpc_t {
	u32 prog;
	u32 vers;
	u32 proc;
	u32 timeout;
};

/****************** DATA DEFINITION SECTION ***********************************/
static const struct misc_api_rpc_t proc_adc_read = {
	QC_ADC_READ_APISPROG,
	QC_ADC_READ_APISVERS,
	QC_ONCRPC_ADC_READ_PROC,
	 TIMEOUT_IMMEDIATELY,
};

static const struct misc_api_rpc_t proc_vreg_control = {
	 PMIC_APISPROG,
	 PMIC_APISVERS,
	 ONCRPC_VREG_CONTROL_PROC,
	 TIMEOUT_IMMEDIATELY,
};

static const struct misc_api_rpc_t proc_enable_camera_flash = {
	 PMIC_APISPROG,
	 PMIC_APISVERS,
	 ONCRPC_ENABLE_CAMERA_FLASH_PROC,
	 TIMEOUT_IMMEDIATELY,
};

static int misc_rpc_call_reply(const struct misc_api_rpc_t *remote,
							void *in, int in_size,
							void *out, int out_size)
{
	struct msm_rpc_endpoint *endpoint;
	int rc = -ENODEV;

	endpoint = msm_rpc_connect_compatible(remote->prog,
											  remote->vers, 0);

	if (NULL == endpoint) {
		printk(KERN_ERR "%s: msm_rpc_connect_compatible failed\n", __func__);
		return rc;
	}
	if (IS_ERR(endpoint)) {
		printk(KERN_ERR "%s: msm_rpc_connect_compatible failed (%ld returned)\n",
			   __func__, IS_ERR(endpoint));
		goto exit_func;
	}
#if DEBUG_LEVEL
	printk(KERN_DEBUG "%s: msm_rpc_connect_compatible OK: (0x%08x:v.%08x)\n",
		   __func__, remote->prog,  remote->vers);
#endif

	rc = msm_rpc_call_reply(endpoint,
			remote->proc,
			in, in_size,
			out, out_size,
			remote->timeout);

	if (rc < 0)
		printk(KERN_ERR"%s: failed err=%d\n", __func__, rc);

#if DEBUG_LEVEL
	printk(KERN_DEBUG "%s (0x%08x:v.%08x proc=%d insize=%d outsize=%d):"
		   " rc = %d\n",
		   __func__, remote->prog, remote->vers,
		   remote->proc, in_size, out_size, rc);
#endif

exit_func:
	msm_rpc_close(endpoint);
	return rc > 0 ? 0 : rc;
}

int msm_adc_read(enum adc_logical_channel_t channel, u16 *return_value)
{
	int rc;
	struct {
		struct rpc_request_hdr hdr;
		u32 data;
	} request;
	struct {
		struct rpc_reply_hdr hdr;
		u32 data;
	} reply;

	request.data = cpu_to_be32(channel);
	rc = misc_rpc_call_reply(&proc_adc_read, &request,
							 sizeof(request), &reply, sizeof(reply));
	if (!rc)
		*return_value = (u16) be32_to_cpu(reply.data);
	return rc;
}
EXPORT_SYMBOL_GPL(msm_adc_read);

int pmic_boost_control(int enable)
{
	int rc;
	struct {
		struct rpc_request_hdr hdr;
		u32 data;
	} request;
	struct {
		struct rpc_reply_hdr hdr;
		u32 data;
	} reply;

	request.data = cpu_to_be32(enable);
	rc = misc_rpc_call_reply(&proc_vreg_control,
							 &request, sizeof(request), &reply, sizeof(reply));
	if (rc)
		printk(KERN_ERR"%s: failed (pmic_lib_err=%d)\n",
				__func__, be32_to_cpu(reply.data));
	return rc;
}
EXPORT_SYMBOL_GPL(pmic_boost_control);

int pmic_enable_camera_flash(u16 current_mamp)
{
	int rc;
	struct {
		struct rpc_request_hdr hdr;
		u32 data;
	} request;
	struct {
		struct rpc_reply_hdr hdr;
		u32 data;
	} reply;

	request.data = cpu_to_be32(current_mamp);
	rc = misc_rpc_call_reply(&proc_enable_camera_flash,
							  &request, sizeof(request), &reply, sizeof(reply));
	if (rc)
		printk(KERN_ERR"%s: failed (pmic_lib_err=%d)\n",
				__func__, be32_to_cpu(reply.data));
	return rc;
}
EXPORT_SYMBOL_GPL(pmic_enable_camera_flash);

MODULE_AUTHOR("Aleksej Makarov");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Misc modem API");

