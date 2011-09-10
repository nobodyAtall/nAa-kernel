/* drivers/misc/semc/seport/seport_main.c
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <mach/semc_seport_platform.h>
#include <linux/device.h>

#include <linux/semc/seport/seport.h>
#include <linux/semc/seport/seport_dcout_iface.h>


/* Local include files. These files are placed in the driver
   code because they are not public */
#include "seport_plug_detect.h"
#include "seport_button_detect.h"
#include "seport_attrs.h"


static ulong seport_plug_detect_poll_time = 1000;
static wait_queue_head_t seport_control_wq;
static int seport_driver_error  = ERROR_NO_ERROR;


static void seport_set_poll_time(ulong time)
{
	seport_plug_detect_poll_time = time;
}

static ulong seport_get_poll_time(void)
{
	return seport_plug_detect_poll_time;
}

unsigned int seport_control_poll(struct file *filp,
				 struct poll_table_struct *wait)
{
	poll_wait(filp, &seport_control_wq, wait);

	if (seport_driver_error != ERROR_NO_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

ssize_t seport_control_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	if (!buf || count != sizeof(int)) {
		printk(KERN_ERR "%s: Null pointer recevied in read function " \
		       "or received size not compatible.\n"		\
		       "Requested size = %d, supplied size = %d\n",
		       __FILE__, count, sizeof(int));
		return -EFAULT;
	}

	*(int *)buf = seport_driver_error;
	seport_driver_error = ERROR_NO_ERROR;

	return sizeof(seport_driver_error);
}

int seport_ioctl(struct inode *inode, struct file *filp,
		 unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u16 tmp;

	/* extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != SEPORT_IOC_MAGIC)
		return -ENOTTY;
	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg,
				  _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	switch (cmd) {
	case SEPORT_IOCQRIDVAL:
		seport_platfrom_get_rid_value(&tmp);
		return tmp;

	case SEPORT_IOCQDETSTAT:
		return seport_plug_detect_get_status();

	case SEPORT_IOCGPHFVAL:
		seport_platform_get_phf_value(&tmp);
		return tmp;

	case SEPORT_IOCSPHFPOLL:
		return seport_enable_vad_button_detection((int)arg);

	case  SEPORT_IOCSBDTCT:
		return seport_enable_button_detection((int)arg);

	case  SEPORT_IOCSVIDOUT:
#if defined(CONFIG_SEPORT_VIDEO_OUT)
		printk(KERN_ALERT "Requested %s video out switch\n",
		       (int)arg == 1 ? "Enable" : "Disable");
		return seport_enable_video_out_switch((int)arg);
#else
		return 0;
#endif

	case SEPORT_IOCSDCOUT:
		return seport_set_dcout((int)arg);

		/* Enable/Disable DCOut High */
	case SEPORT_IOCSDCOHPWR:
		seport_enable_high_power((int) arg);
		break;

		/* Set poll delay */
	case SEPORT_IOCSPOLLDEL:
		seport_set_poll_time((ulong) arg);
		break;

		/* Get currently set poll delay */
	case  SEPORT_IOCQPOLLDEL:
		return seport_get_poll_time();

	case SEPORT_IOCTRIDREST:
		return seport_plug_detect_restart();

	case SEPORT_IOCRRIDMAX:
		return seport_attrs_get_rid_max_vals(
			(struct seport_trim_param_data *)arg);

	case SEPORT_IOCRRIDMIN:
		return seport_attrs_get_rid_min_vals(
			(struct seport_trim_param_data *)arg);

	case SEPORT_IOCRCCOMAX:
		return seport_attrs_get_cco_max_vals(
			(struct seport_trim_param_data *)arg);

	case SEPORT_IOCRCCOMIN:
		return seport_attrs_get_cco_min_vals(
			(struct seport_trim_param_data *)arg);

	case SEPORT_IOCSMICBIAS:
		return seport_platform_enable_mic_bias((u8)arg);

	case SEPORT_IOCSHPAMP:
		return seport_platform_enable_hp_amp((u8)arg);

	case SEPORT_IOCSMBMEAS:
		return seport_platform_enable_mic_bias_measurement((u8)arg);

	default:
		printk(KERN_ALERT "Unhandled IOCTL received. CMD = %d\n", cmd);
		return -ENOTTY;

	}
	return 0;
}

int seport_control_release(struct inode *inode, struct file *filp)
{
	/* Set driver error to indicate system going down. */
	seport_driver_error = -ERROR_SHUT_DOWN;

	/* Waking up waiting processes */
	if (waitqueue_active(&seport_control_wq))
		wake_up_interruptible(&seport_control_wq);

	/* Ignoring return value here. Either this works or
	 * it doesn't. Can't do much more! */
	(void)seport_set_dcout(0);

	return 0;
}

int seport_control_open(struct inode *inode, struct file *filp)
{
	init_waitqueue_head(&seport_control_wq);
	return 0;
}

const struct file_operations seport_plug_detect_fops = {
	.owner =    THIS_MODULE,
	.read =     seport_plug_detect_read,
	.ioctl =    seport_ioctl,
	.open =     seport_plug_detect_open,
	.release =  seport_plug_detect_release,
	.poll =     seport_plug_detect_poll,
};

const struct file_operations seport_control_fops = {
	.owner =    THIS_MODULE,
	.open =     seport_control_open,
	.read =     seport_control_read,
	.ioctl =    seport_ioctl,
	.release =  seport_control_release,
	.poll =     seport_control_poll,
};

const struct file_operations seport_button_detect_fops = {
	.owner =    THIS_MODULE,
	.read =     seport_button_detect_read,
	.ioctl =    seport_ioctl,
	.open =     seport_button_detect_open,
	.release =  seport_button_detect_release,
	.poll =     seport_button_detect_poll,
};

static struct miscdevice seport_plug_detect_misc = {
	MISC_DYNAMIC_MINOR,
	PLUG_DETECT_DEVICE_NAME,
	&seport_plug_detect_fops,
};

static struct miscdevice seport_control_misc = {
	MISC_DYNAMIC_MINOR,
	CONTROL_DEVICE_NAME,
	&seport_control_fops,
};

static struct miscdevice seport_button_detect_misc = {
	MISC_DYNAMIC_MINOR,
	BUTTON_DETECT_DEVICE_NAME,
	&seport_button_detect_fops,
};

void seport_release_device_nodes(void)
{
	misc_deregister(&seport_plug_detect_misc);
	seport_attrs_destroy_attrs(seport_control_misc.this_device);
	misc_deregister(&seport_control_misc);
	misc_deregister(&seport_button_detect_misc);
}


int seport_init_device_nodes(void)
{
	int result;

	result = misc_register(&seport_plug_detect_misc);
	if (result) {
		printk(KERN_ERR "%s - %s: Failed to register "
		       "device plug_detect0. Error code: %d\n",
		       __FILE__, __func__, result);
		return result;
	}

	seport_attrs_init();
	result = misc_register(&seport_control_misc);
	if (result) {
		printk(KERN_ERR "%s - %s: Failed to register "
		       "device control0. Error code: %d\n",
		       __FILE__, __func__, result);
		return result;
	}

	result = seport_attrs_create_attrs(seport_control_misc.this_device);
	if (result) {
		printk(KERN_ERR "%s - %s: Failed to register attributes. " \
		       "Error code: %d\n",
		       __FILE__, __func__, result);
		return result;
	}

	result = misc_register(&seport_button_detect_misc);
	if (result) {
		printk(KERN_ERR "%s - %s: Failed to register "
		       "device plug_detect0. Error code: %d\n",
		       __FILE__, __func__, result);
		return result;
	}

	return 0;
}

void report_over_current_error(u8 set)
{
	int i;
	for (i = 0; i < 10; i++)
		printk(KERN_ERR "*** %s - %s: DCout overload called!\n",
		       __FILE__, __func__);

	if (set) {
		seport_driver_error = -ERROR_DCOUT_OVERLOAD;
		wake_up_interruptible(&seport_control_wq);
	}
}

static int __init seport_init(void)
{
	int err = seport_init_device_nodes();

	if (!err) {
		printk(KERN_INFO "SEMC_Systemconnector: Successfully " \
		       "initialized.\n");
		/* TODO: Add this to a delayed worktask */
		seport_register_overcurrent_callback(
			&report_over_current_error);
	} else
		printk(KERN_ERR "SEMC_Systemconnector: Failed to create " \
		       "device nodes\n");

	return err;
}

static void __exit seport_exit(void)
{
	seport_plug_detect_cleanup();
	seport_enable_button_detection(0);
	seport_enable_vad_button_detection(0);

	seport_release_device_nodes();
}


module_init(seport_init);
module_exit(seport_exit);

MODULE_AUTHOR("Joachim Holst (joachim.holst.x@sonyericcson.com)");
MODULE_DESCRIPTION("Sony Ericcson Portable Headphone (phf) Driver\n"	\
		   "device nodes:\n/dev/systemconnector/control0\n"	\
		   "%S - Key events from remote controls\n"		\
		   "%S - Plug detect interface\n"			\
		   "%S - Control device for IOCTL");

MODULE_LICENSE("GPL");
