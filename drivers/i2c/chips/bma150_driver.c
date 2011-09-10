/*  $Date: 2009/03/24 17:37:35 $
 *  $Revision: 1.0 $ 
 */

/*
* Copyright (C) 2009 Bosch Sensortec GmbH
*
*	BMA150 linux driver
* 
* Usage:	BMA150 driver by i2c for linux
*
* 
* Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in 
  compliance with the License and the following stipulations. The Apache License , Version 2.0 is applicable unless 
  otherwise stated by the stipulations of the disclaimer below. 
 
* You may obtain a copy of the License at 

    http://www.apache.org/licenses/LICENSE-2.0
  
 

Disclaimer 

 * Common:
 * This Work is developed for the consumer goods industry. It may only be used 
 * within the parameters of the respective valid product data sheet.  The Work 
 * provided with the express understanding that there is no warranty of fitness for a particular purpose. 
 * It is not fit for use in life-sustaining, safety or security sensitive systems or any system or device 
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition, 
 * the Work is not fit for use in products which interact with motor vehicle systems.  
 * The resale and/or use of the Work are at the purchaser�s own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser. 
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for 
 * incidental, or consequential damages, arising from any Work or Derivative Work use not covered by the parameters of 
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch 
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased Work and Derivative Works, particularly with regard to 
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid 
 * technical specifications of the product series. They are therefore not intended or fit for resale to third 
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an 
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec 
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the 
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering 
 * samples.
 *
 * Special:
 * This Work and any related information (hereinafter called "Information") is provided free of charge 
 * for the sole purpose to support your application work. The Woek and Information is subject to the 
 * following terms and conditions: 
 *
 * The Work is specifically designed for the exclusive use for Bosch Sensortec products by 
 * personnel who have special experience and training. Do not use this Work or Derivative Works if you do not have the 
 * proper experience or training. Do not use this Work or Derivative Works fot other products than Bosch Sensortec products.  
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no 
 * responsibility for the consequences of use of such Information nor for any infringement of patents or 
 * other rights of third parties which may result from its use. No license is granted by implication or 
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are 
 * subject to change without notice.
 *
 */

/*! \file bma150_driver.c
    \brief This file contains all function implementations for the BMA150 in linux
    
    Details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/delay.h>
/* SEMC:090610:asaumi: MOD-S Add an exclusive access control */
#include <linux/semaphore.h>
#include <linux/time.h>
/* SEMC:090610:asaumi: MOD-E Add an exclusive access control */
#include <linux/i2c/bma150-ioctl.h>

#include "smb380.h"
#include "smb380calib.h"

#define BMA150_MAJOR	100
#define BMA150_MINOR	0


#define DEBUG	0

static struct i2c_client *bma150_client = NULL;

struct bma150_data{
	smb380_t smb380;
	struct class *bma_dev_class;
};

/* SEMC:090610:asaumi: MOD-S Add an exclusive access control */
static struct semaphore bma150_sem;
/* SEMC:090610:asaumi: MOD-E Add an exclusive access control */

static char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma150_i2c_delay(unsigned int msec);


/*	i2c delay routine for eeprom	*/
static inline void bma150_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma150	*/
static inline char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;
	if( bma150_client == NULL )
		return -1;
	dummy = i2c_smbus_write_byte_data(bma150_client, reg_addr, data[0]);
	return 0;
}

/*	i2c read routine for bma150	*/
static inline char bma150_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	int dummy=0;

	if( bma150_client == NULL )
		return -1;

	dummy = i2c_smbus_read_i2c_block_data(bma150_client,
					      reg_addr,
					      len, data);

	return 0;
}

/*	read command for BMA150 device file	*/
static ssize_t bma150_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{	
	smb380acc_t acc;	
	int ret;
	if( bma150_client == NULL )
	{
#if DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;
	}

	smb380_read_accel_xyz(&acc);
#if DEBUG
	printk(KERN_INFO "BMA150: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);  
#endif

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#if DEBUG
	printk(KERN_INFO "BMA150: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}

/*	write command for BMA150 device file	*/
static ssize_t bma150_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma150_client == NULL )
		return -1;
#if DEBUG
	printk("BMA150 should be accessed with ioctl command\n");
#endif
	return 0;
}

/*	open command for BMA150 device file	*/
static int bma150_open(struct inode *inode, struct file *file)
{
#if DEBUG
		printk("%s\n",__FUNCTION__); 
#endif

	if( bma150_client == NULL)
	{
#if DEBUG
		printk("I2C driver not install\n"); 
#endif
		return -1;
	}

	//#if DEBUG
	smb380_set_bandwidth( 0 );		//bandwidth 25Hz
	smb380_set_range( 0 );			//range +/-2G
	//#endif

#if DEBUG
	printk("BMA150 has been opened\n");
#endif
	return 0;
}

/*	release command for BMA150 device file	*/
static int bma150_close(struct inode *inode, struct file *file)
{
#if DEBUG
	printk("%s\n",__FUNCTION__);	
	printk("BMA150 has been closed\n");	
#endif
	return 0;
}


/* SEMC:090610:asaumi: MOD-S Add an exclusive access control */
/*	ioctl command for BMA150 device file	*/
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int temp;
	struct timespec sleeptime;

#if DEBUG
	printk("%s\n",__FUNCTION__);	
#endif

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA150_IOC_MAGIC)	
	{
#if DEBUG		
		printk("cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA150_IOC_MAXNR)
	{
#if DEBUG
		printk("cmd number error\n");
#endif
		return -ENOTTY;
	}

	err = down_interruptible(&bma150_sem);
	if(err == -EINTR) {
		return -EINTR;
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
			goto err_fault;
	}
	/* check bam150_client */
	if( bma150_client == NULL)
	{
#if DEBUG
		printk("I2C driver not install\n"); 
#endif
			goto err_fault;
	}
	
	/* cmd mapping */
	switch(cmd)
	{
	case BMA150_SOFT_RESET:
		err = smb380_soft_reset();
		break;

	case BMA150_GET_OFFSET:
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
#if DEBUG			
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_get_offset(*((unsigned short*)data),(unsigned short*)(data+2));
		if(copy_to_user((unsigned short*)arg,(unsigned short*)data,4)!=0)
		{
#if DEBUG			
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_OFFSET:
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
#if DEBUG			
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_offset(*((unsigned short*)data),*(unsigned short*)(data+2));
		break;

	case BMA150_SELFTEST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG			
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_selftest(*data);
		break;

	case BMA150_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG			
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_range(*data);
		break;

	case BMA150_GET_RANGE:
		err = smb380_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG			
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG			
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_mode(*data);
		break;

	case BMA150_GET_MODE:
		err = smb380_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG			
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_bandwidth(*data);
		break;

	case BMA150_GET_BANDWIDTH:
		err = smb380_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_WAKE_UP_PAUSE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_wake_up_pause(*data);
		break;

	case BMA150_GET_WAKE_UP_PAUSE:
		err = smb380_get_wake_up_pause(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_LOW_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG			
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_low_g_threshold(*data);
		break;

	case BMA150_GET_LOW_G_THRESHOLD:
		err = smb380_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_LOW_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_low_g_countdown(*data);
		break;

	case BMA150_GET_LOW_G_COUNTDOWN:
		err = smb380_get_low_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_HIGH_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_high_g_countdown(*data);
		break;

	case BMA150_GET_HIGH_G_COUNTDOWN:
		err = smb380_get_high_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG			
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_LOW_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_low_g_duration(*data);
		break;

	case BMA150_GET_LOW_G_DURATION:
		err = smb380_get_low_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_HIGH_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_high_g_threshold(*data);
		break;

	case BMA150_GET_HIGH_G_THRESHOLD:
		err = smb380_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_HIGH_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_high_g_duration(*data);
		break;

	case BMA150_GET_HIGH_G_DURATION:
		err = smb380_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_ANY_MOTION_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_any_motion_threshold(*data);
		break;

	case BMA150_GET_ANY_MOTION_THRESHOLD:
		err = smb380_get_any_motion_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_ANY_MOTION_COUNT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_any_motion_count(*data);
		break;

	case BMA150_GET_ANY_MOTION_COUNT:
		err = smb380_get_any_motion_count(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_INTERRUPT_MASK:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_interrupt_mask(*data);
		break;

	case BMA150_GET_INTERRUPT_MASK:
		err = smb380_get_interrupt_mask(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_RESET_INTERRUPT:
		err = smb380_reset_interrupt();
		break;

	case BMA150_READ_ACCEL_X:
		err = smb380_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_READ_ACCEL_Y:
		err = smb380_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_READ_ACCEL_Z:
		err = smb380_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_GET_INTERRUPT_STATUS:
		err = smb380_get_interrupt_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_LOW_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_low_g_int(*data);
		break;

	case BMA150_SET_HIGH_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_high_g_int(*data);
		break;

	case BMA150_SET_ANY_MOTION_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_any_motion_int(*data);
		break;

	case BMA150_SET_ALERT_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_alert_int(*data);
		break;

	case BMA150_SET_ADVANCED_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_advanced_int(*data);
		break;

	case BMA150_LATCH_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_latch_int(*data);
		break;

	case BMA150_SET_NEW_DATA_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_new_data_int(*data);
		break;

	case BMA150_GET_LOW_G_HYST:
		err = smb380_get_low_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_LOW_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_low_g_hysteresis(*data);
		break;

	case BMA150_GET_HIGH_G_HYST:
		err = smb380_get_high_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_SET_HIGH_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		err = smb380_set_high_g_hysteresis(*data);
		break;

	case BMA150_READ_ACCEL_XYZ:
		err = smb380_read_accel_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
#if DEBUG
			printk("copy_to error\n");
#endif
			goto err_fault;
		}
		break;

	case BMA150_READ_TEMPERATURE:
		err = smb380_read_temperature(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#if DEBUG
			printk("copy_to_user error\n");
#endif
			goto err_fault;
		}
		break;

	/* offset calibration routine */
	case BMA150_CALIBRATION:
		if(copy_from_user((smb380acc_t*)data,(smb380acc_t*)arg,6)!=0)
		{
#if DEBUG
			printk("copy_from_user error\n");
#endif
			goto err_fault;
		}
		/* iteration time 10 */
		temp = 10;
		err = smb380_calibrate(*(smb380acc_t*)data, &temp);
		break;

	case BMA150_READ_ACCEL_XYZ_PCTRL:
	{
		unsigned char curr_mode;
		err = smb380_get_mode(&curr_mode);
		err = smb380_set_mode(SMB380_MODE_NORMAL);

		/* wait for ready */
		sleeptime.tv_sec = 0;
		sleeptime.tv_nsec = 1000*NSEC_PER_USEC;
		err = hrtimer_nanosleep(&sleeptime, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);

		err = smb380_read_accel_xyz((smb380acc_t*)data);
		err = smb380_set_mode(curr_mode);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
#if DEBUG
			printk("copy_to error\n");
#endif
			goto err_fault;
		}
		break;
	}
		

	default:
		break;
	}

	up(&bma150_sem);
	return err;

err_fault:
	up(&bma150_sem);
	return -EFAULT;
}
/* SEMC:090610:asaumi: MOD-E Add an exclusive access control */


static const struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.read = bma150_read,
	.write = bma150_write,
	.open = bma150_open,
	.release = bma150_close,
	.ioctl = bma150_ioctl,
};


static int bma150_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma150", I2C_NAME_SIZE);

	return 0;
}

static int bma150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bma150_data *data;
	struct device *dev;
	int err = 0;
	int tempvalue;

#if DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}

	if (!(data = kmalloc(sizeof(struct bma150_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		printk(KERN_INFO "kmalloc error\n");
		goto exit;
	}
	memset(data, 0, sizeof(struct bma150_data));

	i2c_set_clientdata(client, data);

		
	/* read chip id */

	tempvalue = 0;
	i2c_master_send(client, (char*)&tempvalue, 1);
	i2c_master_recv(client, (char*)&tempvalue, 1);

	if((tempvalue&0x00FF) == 0x0002)
	{
		printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA150/SMB380 registered I2C driver!\n");
		bma150_client = client;
	}
	else
	{
		printk(KERN_INFO "Bosch Sensortec Device not found, i2c error %d \n", tempvalue);
/* Enter SEMC modification 090422 */
//		i2c_detach_client(bma150_client);
		err = -1;
/* Exit SEMC modification 090422 */
		bma150_client = NULL;
		goto exit_kfree;
	}
	
	/* register a char dev	*/
	err = register_chrdev(BMA150_MAJOR, "BMA150", &bma150_fops);
	if (err)
		goto exit_kfree;
	printk(KERN_INFO "register_chrdev\n");

	/* create BMA-dev device class */
	data->bma_dev_class = class_create(THIS_MODULE, "BMA-dev");
	if (IS_ERR(data->bma_dev_class)) {
		err = PTR_ERR(data->bma_dev_class);
		goto exit_unreg_chrdev;
	}
	printk(KERN_INFO "class_create ok\n");
	
	/* create device node for bma150 */
	dev = device_create(data->bma_dev_class, NULL, MKDEV(BMA150_MAJOR, 0), NULL,"bma150");
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto exit_unreg_class;
	}
	printk(KERN_INFO "BMA150 device create ok\n");

	data->smb380.bus_write = bma150_i2c_write;
	data->smb380.bus_read = bma150_i2c_read;
	data->smb380.delay_msec = bma150_i2c_delay;
	smb380_init(&(data->smb380));

/* SEMC:090610:asaumi: MOD-S Add an exclusive access control */
	sema_init( &bma150_sem, 1 );
/* SEMC:090610:asaumi: MOD-E Add an exclusive access control */

	return 0;

exit_unreg_class:
	class_destroy(data->bma_dev_class);
exit_unreg_chrdev:
	unregister_chrdev(BMA150_MAJOR, "BMA150");
exit_kfree:
	kfree(data);
exit:
	return err;
}


static int bma150_remove(struct i2c_client *client)
{
	struct bma150_data *data;
#if DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
	data = i2c_get_clientdata(client);
	bma150_client = NULL;
	class_destroy(data->bma_dev_class);
	unregister_chrdev(BMA150_MAJOR,"BMA150");
	kfree(data);
	return 0;
}

static int bma150_suspend(struct i2c_client *client, pm_message_t mesg)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return smb380_set_mode(SMB380_MODE_SLEEP);
}

static int bma150_resume(struct i2c_client *client)
{
#if DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	return smb380_set_mode(SMB380_MODE_NORMAL);
}

static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(bma150);

static const struct i2c_device_id bma150_id[] = {
	{ "bma150", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma150_id);

static struct i2c_driver bma150_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "bma150",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma150_id,
	.address_data	= &addr_data,
	.probe		= bma150_probe,
	.remove		= bma150_remove,
	.detect		= bma150_detect,
	.suspend	= bma150_suspend,
	.resume		= bma150_resume,
};

static int __init bma150_init(void)
{
#if DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	return i2c_add_driver(&bma150_driver);
}

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
	printk(KERN_ERR "BMA150 exit\n");
}

MODULE_DESCRIPTION("BMA150 driver");

module_init(bma150_init);
module_exit(bma150_exit);

