/* drivers/misc/semc/seport/seport_plug_detect.c
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

#include "seport_plug_detect.h"
#include <linux/module.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <mach/semc_seport_platform.h>

#include <linux/semc/seport/seport.h>

#define INITIAL_PLUG_VALUE_TO_FORCE_DETECTION -2
#define SEPORT_START_DONT_DISABLE_INTERRUPT -1
#define SEPORT_PLUG_DETECT_VERIFY_DELAY 300
#define SEPORT_NOT_INITIALIZED 0

static DEFINE_MUTEX(seport_plug_det_init_mutex);
static DEFINE_MUTEX(seport_plug_det_read_mutex);

static struct timer_list seport_plug_det_work_timer;

static wait_queue_head_t seport_plug_detect_wq;


/* We must initialize these in the beginning.
   initial detect information for accessory
   server depends upon this. */
static int seport_plug_status = INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;
static int seport_plug_detect_initialized = SEPORT_NOT_INITIALIZED;
static int seport_last_reported_plug_status =
	INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;

static void seport_do_plug_detect(unsigned long func_data);

static struct seport_plug_detect_data pdd = {
	.status = ATOMIC_INIT(0)
};

/* NOTE:
 * This is actually run in interrupt context. That means we can't
 * use any mutex/semaphore to syncronize access to this function!
 * Therefore, we only do one read and hope it's correct.
 * If the read is not correct, there will most likely be another
 * interrupt generated that will re-arm the timer that calls this
 * function. */
static void seport_do_plug_detect(unsigned long func_data)
{
	int phf_status;

	phf_status = seport_platform_read_detect_pin_value();

	if (seport_plug_status != phf_status) {
		printk(KERN_DEBUG "SEMC_PHF (Interrupt handler) Status changed. Headset is %s\n",
		       phf_status ? "not inserted" : "inserted");
		seport_plug_status = phf_status;
		wake_up_interruptible(&seport_plug_detect_wq);
	}
}

static irqreturn_t seport_gpio_phf_callback(int irq, void *data)
{
	seport_plug_det_work_timer.data = (unsigned long)data;

	/* According to docs, it should be possible to only use mod_timer. */
	mod_timer(&seport_plug_det_work_timer,
		  jiffies + msecs_to_jiffies(SEPORT_PLUG_DETECT_VERIFY_DELAY));

	return IRQ_HANDLED;
}

/*
 * Open and close
 */
int seport_plug_detect_open(struct inode *inode, struct file *filp)
{
	int err;

	mutex_lock(&seport_plug_det_init_mutex);
	/* If someone has already opened this device, there is no
	 * need to reinitialize everything again!
	 * Just return. */
	if (seport_plug_detect_initialized) {
		err = -ERROR_PDET_ALREADY_OPEN;
		goto error;
	}

	init_waitqueue_head(&seport_plug_detect_wq);

	seport_plug_det_work_timer.function = seport_do_plug_detect;

	init_timer(&seport_plug_det_work_timer);

	seport_gpio_phf_callback(SEPORT_START_DONT_DISABLE_INTERRUPT, &pdd);

	err = seport_platform_register_plug_detect_gpio_callback(
		seport_gpio_phf_callback, &pdd);
	if (err) {
		printk(KERN_ALERT "SEMC Systemconnector: Failed to setup GPIO callback. Goodbye!\n");
		err = -ERROR_PDET_OPEN_FAILED;
		goto error;
	}

	seport_plug_detect_initialized = 1;


error:
	mutex_unlock(&seport_plug_det_init_mutex);
	return err;
}

int seport_plug_detect_release(struct inode *inode, struct file *filp)
{
	/* Just call exisiting cleanup function. No use to duplicate! */
	wake_up_interruptible(&seport_plug_detect_wq);
	seport_plug_detect_cleanup();

	return 0;
}

unsigned int seport_plug_detect_poll(struct file *filp,
				     struct poll_table_struct *wait)
{
	poll_wait(filp, &seport_plug_detect_wq, wait);

	if (seport_last_reported_plug_status != seport_plug_status)
		return POLLIN | POLLRDNORM;

	return ERROR_NO_ERROR;
}

ssize_t seport_plug_detect_read(struct file *filp, char __user *buf,
					 size_t count, loff_t *f_pos)
{
	int size;
	struct seport_connection_event *ev =
		(struct seport_connection_event *)buf;

	mutex_lock(&seport_plug_det_read_mutex);

	if (!access_ok(VERIFY_WRITE, ev,
		       sizeof(struct seport_connection_event))) {
		printk(KERN_INFO "Invalid user data pointer supplied\n");
		size = -EINVAL;
		goto error;
	}

	ev->tm = current_kernel_time();

	seport_last_reported_plug_status = seport_plug_status;
	/* Need to invert read value to send correct report */
	ev->plug_status = !seport_last_reported_plug_status;

	/* Setting default values on unitialized parameters. */
	ev->rid_value = RID_VAL_NONE;
	ev->cco_type = CCO_UNDEFINED;

	size = sizeof(struct seport_connection_event);

error:
	mutex_unlock(&seport_plug_det_read_mutex);
	return size;
}

int seport_plug_detect_restart(void)
{
	seport_plug_status = INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;
	seport_last_reported_plug_status =
		INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;
	pdd.first_run = 1;

	if (IRQ_HANDLED == seport_gpio_phf_callback(atomic_read(&pdd.status),
						     &pdd))
		return 0;
	return -1;
}

void seport_plug_detect_cleanup(void)
{
	mutex_lock(&seport_plug_det_init_mutex);
	seport_platform_unregister_plug_detect_gpio_callback(&pdd);

	seport_plug_detect_initialized = 0;
	seport_plug_status = INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;
	seport_last_reported_plug_status =
		INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;
	del_timer_sync(&seport_plug_det_work_timer);
	mutex_unlock(&seport_plug_det_init_mutex);
}

int seport_plug_detect_get_status(void)
{
	if (seport_plug_detect_initialized)
		return seport_plug_status ? 0 : 1;
	return INITIAL_PLUG_VALUE_TO_FORCE_DETECTION;
}
