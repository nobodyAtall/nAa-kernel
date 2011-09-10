/*
   MSM PMIC platform light sensor driver

   Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.
   Copyright (C) 2009 Sony Ericsson Mobile Communications AB.
   Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
  
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
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <mach/pmic.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/msm-pmic-lightsensor.h>
#ifdef CONFIG_MSM_PMIC_LIGHTSENSOR_USE_LM3530
#include <linux/leds-lm3530.h>
#else
#include <mach/misc_modem_api.h>
#endif

#define DEBUG	0
#define DEV_NAME "lightsensor"
#define DRV_NAME "pmic-lightsensor"

#define PROXIMITY_OPTOSENSE_PIN 	 PM_MPP_15

enum ls_state_t {
	LS_STATE_INIT		= 0,
	LS_DEVICE_OPENED 	= 0x01,
	LS_DATA_UNDEF 		= 0x02,
	LS_DATA_READY 		= 0x04,
	LS_USE_THRESHOLD 	= 0x08,
	LS_ANY_RESULT		= 0x10,
	LS_SUSPEND			= 0x80
};

struct pmic_ls_data {
	struct lightsensor_conf_t conf;
	unsigned short resolution_mask;
	unsigned short sensor_value;
	enum ls_state_t state;	
	struct mutex update_lock;
	struct timer_list tmr;
	struct early_suspend suspend;
};

static struct pmic_ls_data ls;

static void timer_bh(struct work_struct *work);
static DECLARE_WORK(timer_work, timer_bh);
static void timer_callback(unsigned long d);
static DECLARE_WAIT_QUEUE_HEAD(waitQ);
#ifdef CONFIG_PM
static void ls_early_suspend(struct early_suspend *h);
static void ls_late_resume(struct early_suspend *h);
#endif

static u16 ls_read_adc_value(void)
{
	u16 value;
#ifdef CONFIG_MSM_PMIC_LIGHTSENSOR_USE_LM3530
	value = lm3530_als_get_value();
#else
	int rc;
	rc = msm_adc_read(APP_ADC_CHANNEL_LIGHTSENSOR, &value);
	if (rc) {
		printk(KERN_ERR "%s msm_adc_read returnd error %d\n", DEV_NAME, rc);
		value = 0;
	}
#if DEBUG
	printk(KERN_DEBUG "%s (PM_ADC) %d\n", DEV_NAME, value);
#endif
#endif
	return value;
}

static u16 ls_get_sensor_value(void)
{
	u16 value;

	value = ls_read_adc_value();
	if (ls.conf.resolution < LIGHTSENSOR_RESOLUTION_THRESHOLD)
		value &= ls.resolution_mask;
	return value;
}

static void ls_init_data(void)
{
	ls.state = LS_STATE_INIT;
	ls.conf.resolution = LIGHTSENSOR_RESOLUTION_DEFAULT;
	ls.conf.poll_period_ms = LIGHTSENSOR_DEFAULT_POLL_PERIOD;
	ls.resolution_mask = ~((1 << LIGHTSENSOR_RESOLUTION_DEFAULT) - 1);
	setup_timer(&ls.tmr, timer_callback, 0);
}

static void ls_release_data(void)
{
	del_timer_sync(&ls.tmr);
}

static int check_sensor_value(void)
{
	u16 value = ls_get_sensor_value();

	if (!(ls.state & LS_USE_THRESHOLD)) {
		value &= ls.resolution_mask;
		if (value != ls.sensor_value || ls.state & LS_DATA_UNDEF)
			goto value_updated;
	} else if ((value >= ls.conf.threshold &&
			ls.sensor_value < ls.conf.threshold) ||
			 (value < ls.conf.threshold && ls.sensor_value >= ls.conf.threshold)
			 || (ls.state & LS_DATA_UNDEF)) {
		goto value_updated;
	}
	return 0;

value_updated:

	ls.state = (ls.state & ~LS_DATA_UNDEF) | LS_DATA_READY;
	ls.sensor_value = value;
#if DEBUG
	printk("%s sensor value updated: %d\n", DEV_NAME, value);
#endif
	return 1;
}


static void timer_bh(struct work_struct *work)
{
	mutex_lock(&ls.update_lock);

	if (!(ls.state & LS_SUSPEND)) {
		if (check_sensor_value()) {
			wake_up_interruptible(&waitQ);	
			goto out;
		}

		if (ls.state & LS_DEVICE_OPENED && !(ls.state & LS_DATA_READY))
			mod_timer(&ls.tmr, jiffies +
					msecs_to_jiffies(ls.conf.poll_period_ms));
	}
out:
	mutex_unlock(&ls.update_lock);
}

static void timer_callback(unsigned long d)
{
  schedule_work(&timer_work);
}

static int ls_open(struct inode *inode, struct file *file)
{
	int rc = 0;
#if DEBUG
	printk(KERN_DEBUG "%s open request\n", DEV_NAME);
#endif
	mutex_lock(&ls.update_lock);
	if ( ls.state & LS_DEVICE_OPENED)
		rc = -EBUSY;
	else
		ls.state |= LS_DEVICE_OPENED | LS_DATA_UNDEF;
	mutex_unlock(&ls.update_lock);
	return rc;
}

static int ls_close(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_DEBUG  "%s closed.\n", DEV_NAME);
#endif
	mutex_lock(&ls.update_lock);
	ls.state = LS_STATE_INIT;
	mutex_unlock(&ls.update_lock);
	return 0;
}

static ssize_t ls_read(struct file *file, char __user *buf,
					   size_t count, loff_t *offset)
{
	int ok;

#if DEBUG
	printk("%s read request.\n", DEV_NAME);
#endif

	mutex_lock(&ls.update_lock);

	if (ls.state & LS_SUSPEND) {
		printk(KERN_ERR "%s:  is suspended!\n", DEV_NAME);
		mutex_unlock(&ls.update_lock);
		return -EBUSY;
	}

	ok = check_sensor_value();
	if (ok || ls.state & LS_ANY_RESULT)
		goto data_ready;

	if (file->f_flags & O_NONBLOCK) {
		mutex_unlock(&ls.update_lock);
		return -EAGAIN;
	}
	mod_timer(&ls.tmr, jiffies + msecs_to_jiffies(ls.conf.poll_period_ms));

	mutex_unlock(&ls.update_lock);

#if DEBUG
	printk("%s read request in a wait queue\n", DEV_NAME);
#endif
	if (-ERESTARTSYS ==
			wait_event_interruptible(waitQ, ls.state & LS_DATA_READY))
		return -ERESTARTSYS;

	mutex_lock(&ls.update_lock);

data_ready:
#if DEBUG
	printk("%s sensor value %d\n", DEV_NAME, ls.sensor_value);
#endif
	ls.state &= ~LS_DATA_READY;
	ok = count >= sizeof(ls.sensor_value) &&
			!copy_to_user(buf, &ls.sensor_value, sizeof(ls.sensor_value));
	ok = ok ? sizeof(ls.sensor_value) : -EFAULT;
	mutex_unlock(&ls.update_lock);

	return ok;
}

static unsigned int ls_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

#if DEBUG
	printk("%s poll request\n", DEV_NAME);
#endif

	mutex_lock(&ls.update_lock);

	poll_wait(filp, &waitQ, wait);

	if (!(ls.state & (LS_DATA_READY | LS_ANY_RESULT) ||
									check_sensor_value())) {
		mod_timer(&ls.tmr, jiffies + msecs_to_jiffies(ls.conf.poll_period_ms));
		goto data_not_ready;
	}

#if DEBUG
	printk("%s poll request - data ready\n", DEV_NAME);
#endif
	mask |= POLLIN | POLLRDNORM; /* readable */

data_not_ready:
	mutex_unlock(&ls.update_lock);
	return mask;
}

/* write command for PROXIMITY */
static ssize_t ls_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
#if DEBUG
	printk("%s write request.\n", DEV_NAME);
#endif

	return count;
}

/* ioctl command for PROXIMITY */
static int ls_ioctl(struct inode *inode, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	int err = 0;

	/* check cmd */
	if	(_IOC_TYPE(cmd) != LIGHTSENSOR_IOC_MAGIC)
	{
		#if DEBUG
		printk("cmd magic type error\n");
		#endif
		return -ENOTTY;
	}
	if	(_IOC_NR(cmd) > LIGHTSENSOR_IOC_MAXNR)
	{
		#if DEBUG
		printk("cmd number error\n");
		#endif
		return -ENOTTY;
	}

	if	(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if	(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if	(err)
		goto fault;

	mutex_lock(&ls.update_lock);
	switch (cmd) {

	case LIGHTSENSOR_GET_CONFIG:
		if (copy_to_user((void *)arg, &ls.conf,
				sizeof(struct lightsensor_conf_t)) != 0)
			goto fault;
		break;

	case LIGHTSENSOR_SET_CONFIG: {

		struct lightsensor_conf_t d;

		if (copy_from_user(&d, (void *)arg,
				sizeof(struct lightsensor_conf_t)) != 0)
			goto fault;

		if ((d.poll_period_ms >= LIGHTSENSOR_MIN_POLL_PERIOD))
			ls.conf.poll_period_ms  = d.poll_period_ms;

		ls.conf.threshold = d.threshold;
		ls.conf.resolution = d.resolution;

		if (d.resolution >= LIGHTSENSOR_RESOLUTION_THRESHOLD)
			ls.state |= LS_USE_THRESHOLD;
		else {
			ls.resolution_mask	= ~((1 << d.resolution) - 1);
			ls.state &= ~LS_USE_THRESHOLD;
		}
		#if DEBUG
		printk(KERN_DEBUG "%s: polling period %d ms, "
					 "threshold %d, resolution mask 0x%02X\n",
					 DEV_NAME,
					 d.poll_period_ms,
					 d.threshold,
					 ls.resolution_mask);
		#endif
	}
	break;

	case LIGHTSENSOR_SET_NO_WAIT:
		ls.state |= LS_ANY_RESULT;
		break;

	case LIGHTSENSOR_CLEAR_NO_WAIT:
		ls.state &= ~LS_ANY_RESULT;
		break;

	default:
		#if DEBUG
		printk("%s IOCTL unknown.\n", DEV_NAME);
		#endif
		break;
	}

	mutex_unlock(&ls.update_lock);
	return 0;		

fault:
	mutex_unlock(&ls.update_lock);
#if DEBUG
	printk("%s IOCTL copy to user error.\n", DEV_NAME);
#endif
	return -EFAULT;
}

static const struct file_operations ls_fops = {
	.owner = THIS_MODULE,
	.open = ls_open,
	.release = ls_close,
	.read = ls_read,
	.write = ls_write,
	.poll = ls_poll,
	.ioctl = ls_ioctl,
};

static struct miscdevice ls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_NAME,
	.fops = &ls_fops,
};

static int ls_init(struct platform_device *pdev)
{
	int res;

	mutex_init(&ls.update_lock);

	ls_init_data();

	res = misc_register(&ls_device);
	if (res) {
		printk(KERN_ERR "%s: Couldn't misc_register, error=%d\n",
			   DEV_NAME, res);
		goto err;
	}
#ifdef CONFIG_PM
	ls.suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	ls.suspend.suspend = ls_early_suspend;
	ls.suspend.resume = ls_late_resume;
	register_early_suspend(&ls.suspend);
#endif
	printk(KERN_ERR "%s: registered.\n", DEV_NAME);

	return 0;
err:
	ls_release_data();
	printk(KERN_ERR "%s:  driver installation failed\n", DEV_NAME);
	return res;
}

static void ls_exit(void)
{
	del_timer_sync(&ls.tmr);
	ls_release_data();
	misc_deregister(&ls_device);
	printk(KERN_INFO "%s driver removed\n", DEV_NAME);
}

static int ls_probe(struct platform_device *pdev)
{
	int rc;
	rc = ls_init(pdev);
	return rc;
}

static int __devexit ls_remove(struct platform_device *pdev)
{
	ls_exit();
	return 0;
}

#ifdef CONFIG_PM
static void ls_go_suspend(void)
{
	printk(KERN_DEBUG "%s: Suspend.\n", DEV_NAME);
	del_timer_sync(&ls.tmr);
	cancel_work_sync(&timer_work);
	mutex_lock(&ls.update_lock);
	ls.state |= LS_SUSPEND;
	mutex_unlock(&ls.update_lock);
}

static void ls_go_resume(void)
{
	mutex_lock(&ls.update_lock);
	ls.state &= ~LS_SUSPEND;
	if(ls.state & LS_DEVICE_OPENED)
		mod_timer(&ls.tmr, jiffies + msecs_to_jiffies(ls.conf.poll_period_ms));

	mutex_unlock(&ls.update_lock);
	printk(KERN_DEBUG "%s: Resume.\n", DEV_NAME);
}

#ifdef LS_SUPPORT_SUSPEND
static int ls_suspend(struct platform_device *dev,
		pm_message_t state)
{
	return 0;
}

static int ls_resume(struct platform_device *dev)
{
	return 0;
}
#else
	#define ls_suspend NULL
	#define ls_resume NULL
#endif

static void ls_early_suspend(struct early_suspend *h)
{
	ls_go_suspend();
}

static void ls_late_resume(struct early_suspend *h)
{
	ls_go_resume();
}

#else
	#define ls_suspend NULL
	#define ls_resume NULL
#endif

static struct platform_driver ls_driver = {
	.probe		= ls_probe,
	.remove		= __devexit_p(ls_remove),
	.suspend	= ls_suspend,
	.resume		= ls_resume,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ls_driver_init(void)
{
	return platform_driver_register(&ls_driver);
}
module_init(ls_driver_init);

static void __exit ls_driver_exit(void)
{
	platform_driver_unregister(&ls_driver);
}
module_exit(ls_driver_exit);

MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_DESCRIPTION("MSM PMIC platform light sensor driver");
MODULE_LICENSE("GPL");


