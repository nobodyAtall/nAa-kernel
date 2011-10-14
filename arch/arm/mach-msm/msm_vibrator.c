/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

#define PM_LIBPROG      0x30000061
#define PM_LIBVERS      0x10001

#define HTC_PROCEDURE_SET_VIB_ON_OFF	22
#define PMIC_VIBRATOR_LEVEL	(2500)

static struct work_struct work_vibrator;
static struct hrtimer vibe_timer;
static int vib_enable;

static void set_pmic_vibrator(struct work_struct *work)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "init vib rpc failed!\n");
			vib_endpoint = 0;
			return;
		}
	}


	if (vib_enable)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);

	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);

	if (value == 0) {
		vib_enable=0;
		schedule_work(&work_vibrator);
	} else {
		value = (value > 15000 ? 15000 : value);

		vib_enable = 1;
		schedule_work(&work_vibrator);
		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vib_enable = 0;
	schedule_work(&work_vibrator);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __init msm_init_pmic_vibrator(void)
{
	INIT_WORK(&work_vibrator, set_pmic_vibrator);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;
	vib_enable = 0;

	return timed_output_dev_register(&pmic_vibrator);
}

arch_initcall(msm_init_pmic_vibrator);

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

