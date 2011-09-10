/* SEMC:modified */
/* 
   Proximity sensor driver

   Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.

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
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <linux/ctype.h>
#include <linux/wakelock.h>

#include <linux/es209ra_proximity.h>

#define DEBUG	0

static unsigned long Proximity_BurstDuration;	/* Burst duration(micro sec) */
struct wake_lock proximity_wakelock;

static void Proximity_initialize(void);

/* initialize function for PROXIMITY */
static void Proximity_initialize()
{
	/* initialize */
	gpio_set_value(PROXIMITY_GPIO_LEDON_PIN,PROXIMITY_GPIO_LEDON_DISABLE);
	gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,PROXIMITY_GPIO_ENBAR_DISABLE);
	gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,PROXIMITY_GPIO_POWER_OFF);
}

/* open command for PROXIMITY device file */
static int Proximity_open(struct inode *inode, struct file *file)
{
	#if DEBUG
	printk("PROXIMITY has been opened\n");
	#endif
	Proximity_initialize();
	return 0;
}

/* release command for PROXIMITY device file */
static int Proximity_close(struct inode *inode, struct file *file)
{
	#if DEBUG
	printk("PROXIMITY has been closed\n");
	#endif
	Proximity_initialize();
	return 0;
}

/* read command for PROXIMITY */
static ssize_t Proximity_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	#if DEBUG
	gpio_set_value(PROXIMITY_GPIO_POWER_PIN,PROXIMITY_GPIO_POWER_ON);
	/* wait */
	udelay(100);
	gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,PROXIMITY_GPIO_ENBAR_ENABLE);
	/* wait */
	udelay(20);
	gpio_set_value(PROXIMITY_GPIO_LEDON_PIN,PROXIMITY_GPIO_LEDON_ENABLE);

	/* wait */
	udelay(Proximity_BurstDuration);

	printk("PROXIMITY: DOUT : %d\n" , (gpio_get_value(PROXIMITY_GPIO_DOUT_PIN)==PROXIMITY_GPIO_DOUT_ON)?1:0);

	gpio_set_value(PROXIMITY_GPIO_LEDON_PIN,PROXIMITY_GPIO_LEDON_DISABLE);
	gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,PROXIMITY_GPIO_ENBAR_DISABLE);
	gpio_set_value(PROXIMITY_GPIO_POWER_PIN,PROXIMITY_GPIO_POWER_OFF);
	#endif
	return 0;
}

/* write command for PROXIMITY */
static ssize_t Proximity_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	return 0;
}

static void Proximity_setlock(int lock)
{
	if (lock)
		wake_lock(&proximity_wakelock);
	else
		wake_unlock(&proximity_wakelock);
}

/* ioctl command for PROXIMITY */
static int Proximity_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned int data = 0;
	unsigned long time = 0;

	/* check cmd */
	if(_IOC_TYPE(cmd) != PROXIMITY_IOC_MAGIC)
	{
		#if DEBUG
		printk("cmd magic type error\n");
		#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > PROXIMITY_IOC_MAXNR)
	{
		#if DEBUG
		printk("cmd number error\n");
		#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
		#if DEBUG
		printk("cmd access_ok error\n");
		#endif
		return -EFAULT;
	}

	switch(cmd)
	{
	case PROXIMITY_SET_POWER_STATE:
		if(copy_from_user((unsigned char*)&data, (unsigned char*)arg, 1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		gpio_set_value(PROXIMITY_GPIO_POWER_PIN,
				(data==PROXIMITY_GPIO_POWER_ON)?PROXIMITY_GPIO_POWER_ON:PROXIMITY_GPIO_POWER_OFF);
		return err;

	case PROXIMITY_GET_POWER_STATE:
		data = gpio_get_value(PROXIMITY_GPIO_POWER_PIN);
		if(copy_to_user((unsigned char*)arg, (unsigned char*)&data, 1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case PROXIMITY_SET_DEVICE_MODE:
		if(copy_from_user((unsigned char*)&data, (unsigned char*)arg, 1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,
				(data==PROXIMITY_GPIO_ENBAR_ENABLE)?PROXIMITY_GPIO_ENBAR_ENABLE:PROXIMITY_GPIO_ENBAR_DISABLE);
		return err;

	case PROXIMITY_GET_DEVICE_MODE:
		data = gpio_get_value(PROXIMITY_GPIO_ENBAR_PIN);
		if(copy_to_user((unsigned char*)arg, (unsigned char*)&data, 1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case PROXIMITY_SET_LED_MODE:
		if(copy_from_user((unsigned char*)&data, (unsigned char*)arg, 1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		gpio_set_value(PROXIMITY_GPIO_LEDON_PIN,
				(data==PROXIMITY_GPIO_LEDON_ENABLE)?PROXIMITY_GPIO_LEDON_ENABLE:PROXIMITY_GPIO_LEDON_DISABLE);
		return err;

	case PROXIMITY_GET_LED_MODE:
		data = gpio_get_value(PROXIMITY_GPIO_LEDON_PIN);
		if(copy_to_user((unsigned char*)arg, (unsigned char*)&data, 1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case PROXIMITY_GET_DETECTION_STATE:
		data = gpio_get_value(PROXIMITY_GPIO_DOUT_PIN);
		if(copy_to_user((unsigned char*)arg, (unsigned char*)&data, 1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case PROXIMITY_SET_BURST_ON_TIME:
		if(copy_from_user((unsigned long*)&time, (unsigned long*)arg, sizeof(arg))!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		if((time < PROXIMITY_BURST_ON_TIME_MIN) || (time > PROXIMITY_BURST_ON_TIME_MAX)) {
			#if DEBUG
			printk("Requested time is out of range\n");
			#endif
			return -EFAULT;
		}
		#if DEBUG
		Proximity_BurstDuration = time;
		printk("PROXIMITY: Burst-on-time is set to %ld(us)\n" , time);
		#endif
		return err;

	case PROXIMITY_GET_BURST_ON_TIME:
		if(copy_to_user((unsigned long*)arg, (unsigned long*)&Proximity_BurstDuration, sizeof(arg))!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case PROXIMITY_DO_SENSING:
		gpio_set_value(PROXIMITY_GPIO_POWER_PIN,PROXIMITY_GPIO_POWER_ON);
		/* wait */
		udelay(100);
		gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,PROXIMITY_GPIO_ENBAR_ENABLE);
		/* wait */
		udelay(20);
		gpio_set_value(PROXIMITY_GPIO_LEDON_PIN,PROXIMITY_GPIO_LEDON_ENABLE);

		/* wait */
		udelay(Proximity_BurstDuration);

		data = gpio_get_value(PROXIMITY_GPIO_DOUT_PIN);

		gpio_set_value(PROXIMITY_GPIO_LEDON_PIN,PROXIMITY_GPIO_LEDON_DISABLE);
		gpio_set_value(PROXIMITY_GPIO_ENBAR_PIN,PROXIMITY_GPIO_ENBAR_DISABLE);
		gpio_set_value(PROXIMITY_GPIO_POWER_PIN,PROXIMITY_GPIO_POWER_OFF);
		if(copy_to_user((unsigned char*)arg, (unsigned char*)&data, 1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case PROXIMITY_SET_LOCK:
		Proximity_setlock(true);
		return err;

	case PROXIMITY_SET_UNLOCK:
		Proximity_setlock(false);
		return err;

	default:
		return 0;
	}
}

static const struct file_operations Proximity_fops = {
	.owner = THIS_MODULE,
	.open = Proximity_open,
	.release = Proximity_close,
	.read = Proximity_read,
	.write = Proximity_write,
	.ioctl = Proximity_ioctl,
};

static struct miscdevice Proximity_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "es209ra_proximity",
	.fops = &Proximity_fops,
};

static int __init Proximity_init(void)
{
	int res;

	res = misc_register(&Proximity_device);
	if (res) {
		printk(KERN_ERR "PROXIMITY: Couldn't misc_register, error=%d\n", res);
		goto err;
	}

	/* gpio configurations */
	res = gpio_request(PROXIMITY_GPIO_POWER_PIN, "POWER");
	if (res) {
		printk(KERN_ERR
		       "PROXIMITY: Couldn't gpio_request(POWER), error=%d\n", res);
		goto err_request_power;
	}
	res = gpio_request(PROXIMITY_GPIO_ENBAR_PIN, "ENBAR");
	if (res) {
		printk(KERN_ERR
		       "PROXIMITY: Couldn't gpio_request(ENBAR), error=%d\n", res);
		goto err_request_enbar;
	}
	res = gpio_request(PROXIMITY_GPIO_LEDON_PIN, "LEDON");
	if (res) {
		printk(KERN_ERR
		       "PROXIMITY: Couldn't gpio_request(LEDON), error=%d\n", res);
		goto err_request_ledon;
	}
	res = gpio_request(PROXIMITY_GPIO_DOUT_PIN, "DOUT");
	if (res) {
		printk(KERN_ERR
		       "PROXIMITY: Couldn't gpio_request(DOUT), error=%d\n", res);
		goto err_request_dout;
	}

	gpio_direction_output(PROXIMITY_GPIO_POWER_PIN, PROXIMITY_GPIO_POWER_OFF);
	gpio_direction_output(PROXIMITY_GPIO_ENBAR_PIN, PROXIMITY_GPIO_ENBAR_DISABLE);
	gpio_direction_output(PROXIMITY_GPIO_LEDON_PIN, PROXIMITY_GPIO_LEDON_DISABLE);
	gpio_direction_input(PROXIMITY_GPIO_DOUT_PIN);

	Proximity_BurstDuration = PROXIMITY_BURST_ON_TIME_DEFAULT;

	/* init wakelock */
	wake_lock_init(&proximity_wakelock, WAKE_LOCK_SUSPEND, "proximitylock");

	printk(KERN_INFO "PROXIMITY driver installation succeeded\n");

	return 0;


err_request_dout:
	gpio_free(PROXIMITY_GPIO_LEDON_PIN);
err_request_ledon:
	gpio_free(PROXIMITY_GPIO_ENBAR_PIN);
err_request_enbar:
	gpio_free(PROXIMITY_GPIO_POWER_PIN);
err_request_power:
	misc_deregister(&Proximity_device);
err:
	printk(KERN_ERR "%s: PROXIMITY driver installation failed\n", __FILE__);
	return res;
}

static void __exit Proximity_exit(void)
{
	gpio_free(PROXIMITY_GPIO_POWER_PIN);
	gpio_free(PROXIMITY_GPIO_ENBAR_PIN);
	gpio_free(PROXIMITY_GPIO_LEDON_PIN);
	gpio_free(PROXIMITY_GPIO_DOUT_PIN);

	misc_deregister(&Proximity_device);
	wake_lock_destroy(&proximity_wakelock);

	printk(KERN_INFO "PROXIMITY driver removed\n");
}

MODULE_AUTHOR("SEMC");
MODULE_DESCRIPTION("Proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(Proximity_init);
module_exit(Proximity_exit);

