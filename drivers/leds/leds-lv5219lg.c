/*
 * Copyright (C) 2010 Sony Ericsson Mobile Communications.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/backlight.h>

#include <linux/leds-lv5219lg.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mutex.h>

static unsigned char reg_LEDCTL1;
static unsigned char reg_LEDCTL2;
static unsigned char reg_LEDCTL3 = 0x88;
static unsigned char reg_LEDCTL4 = 0x80;

static unsigned char reg_MLEDDACTL = 0xC0;
static unsigned char reg_S1DACTL = 0x00;
static unsigned char reg_S2DACTL = 0x00;
static unsigned char reg_R1DACTL = 0x00;
static unsigned char reg_G1DACTL = 0x00;
static unsigned char reg_B1DACTL = 0x00;
static unsigned char reg_F1DACTL = 0x1f;
static unsigned char reg_F2DACTL = 0x1f;
static unsigned char reg_F3DACTL = 0x1f;

static unsigned char reg_SFCTL;

static unsigned char reg_R1FCTL;
static unsigned char reg_G1FCTL;
static unsigned char reg_B1FCTL;
static unsigned char reg_RGB1GRCTL;
static unsigned char reg_RGB1PUCTL;
static unsigned char reg_R1AOFFCTL;
static unsigned char reg_R1AONCTL;
static unsigned char reg_G1AOFFCTL;
static unsigned char reg_G1AONCTL;
static unsigned char reg_B1AOFFCTL;
static unsigned char reg_B1AONCTL;

static struct mutex rgb_lock;

static unsigned char reg_PTCTL1 = 0x80;
static unsigned char reg_PTCTL2 = 0x00;
static unsigned char reg_PTCTL3 = 0x00;

static unsigned char reg_PTMDACTL_TABLE[17][16] = {
	{0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
	 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03},
	{0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
	 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07},
	{0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
	 0x07, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F},
	{0x07, 0x07, 0x0B, 0x0B, 0x0B, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x13, 0x13, 0x13, 0x13, 0x17, 0x17, 0x17},
	{0x0B, 0x0B, 0x0B, 0x0F, 0x0F, 0x13, 0x13, 0x13,
	 0x13, 0x17, 0x17, 0x17, 0x17, 0x1B, 0x1B, 0x1F},
	{0x0B, 0x0B, 0x0F, 0x0F, 0x0F, 0x13, 0x17, 0x17,
	 0x17, 0x1F, 0x1F, 0x23, 0x23, 0x2B, 0x2B, 0x33},
	{0x0B, 0x0F, 0x0F, 0x0F, 0x13, 0x13, 0x17, 0x17,
	 0x17, 0x1F, 0x2D, 0x2D, 0x33, 0x33, 0x43, 0x43},
	{0x0F, 0x13, 0x13, 0x13, 0x17, 0x17, 0x1F, 0x1F,
	 0x2B, 0x2B, 0x33, 0x33, 0x43, 0x43, 0x53, 0x53},
	{0x13, 0x13, 0x17, 0x17, 0x1F, 0x1F, 0x2B, 0x2B,
	 0x33, 0x33, 0x43, 0x43, 0x53, 0x53, 0x5F, 0x67},
	{0x1B, 0x1B, 0x1F, 0x1F, 0x2B, 0x2B, 0x33, 0x33,
	 0x43, 0x43, 0x4B, 0x4B, 0x5B, 0x5B, 0x63, 0x67},
	{0x27, 0x2F, 0x2F, 0x33, 0x33, 0x33, 0x37, 0x37,
	 0x47, 0x47, 0x4F, 0x4F, 0x5F, 0x5F, 0x67, 0x6B},
	{0x37, 0x3B, 0x3B, 0x43, 0x43, 0x4F, 0x53, 0x53,
	 0x53, 0x5B, 0x5B, 0x63, 0x63, 0x67, 0x67, 0x6F},
	{0x43, 0x4F, 0x4F, 0x53, 0x53, 0x57, 0x5B, 0x5B,
	 0x5B, 0x5F, 0x63, 0x67, 0x67, 0x6B, 0x6B, 0x73},
	{0x4F, 0x53, 0x53, 0x57, 0x57, 0x5B, 0x5F, 0x5F,
	 0x5F, 0x63, 0x67, 0x6B, 0x6B, 0x6F, 0x6F, 0x73},
	{0x63, 0x63, 0x67, 0x67, 0x67, 0x67, 0x6F, 0x6F,
	 0x6F, 0x6F, 0x73, 0x73, 0x73, 0x73, 0x77, 0x77},
	{0x73, 0x73, 0x73, 0x73, 0x73, 0x73, 0x73, 0x73,
	 0x73, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B},
	{0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B,
	 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B, 0x7B}
};
static struct mutex intensity_ctrl_lock;
static bool intensity_ctrl = false;

static unsigned char reg_PTMDACTL[16] = {
	0x0F, 0x0F, 0x15, 0x15, 0x1B, 0x1B, 0x23, 0x23,
	0x2D, 0x38, 0x45, 0x55, 0x67, 0x7B, 0x7B, 0x7B
};

#define DEBUG(x)

static unsigned short lv5219lg_convert_level_to_lux[] = {
  13,
  32,
  50,
  75,
  115,
  180,
  285,
  450,
  700,
  1075,
  1600,
  2450,
  4000,
  6000,
  8500,
  10000
};

static struct i2c_client *ledc_i2c_client;

struct lv5219lg_data {
	struct i2c_client *i2c_client;
};

struct lv5219lg_led {
	struct led_classdev ldev;
	int led_kind;
	unsigned long blink_on;
	unsigned long blink_off;
};

static const struct i2c_device_id lv5219lg_id[] = {
	{ "lv5219lg", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lv5219lg_id);

static void lv5219lg_mled_auto_intensity_ctrl(unsigned char sw);
static void ledc_illuminance_sensor_intensity(unsigned char *val);
static int lv5219lg_get_sensor_level(unsigned char *level);
static void lv5219lg_enable(unsigned char sw, unsigned int led_chanel);
static void lv5219lg_i2c_write(unsigned char reg_addr, unsigned char data);
static void lv5219lg_i2c_block_write(unsigned char reg_addr,
			unsigned char length, unsigned char *data);
static int lv5219lg_i2c_read(unsigned char reg_addr, unsigned char *data);
static void lv5219lg_rgb_ctrl(struct lv5219lg_led *led);

/*==============================================================================
				MLED
==============================================================================*/

static void lv5219lg_mled_normal_ctrl(unsigned char sw)
{
	if (sw == LV5219LG_CONTROL_ON) {
		lv5219lg_enable(LV5219LG_CONTROL_ON, LV5219LG_CHANNEL_MLED);

		reg_LEDCTL1 |= 0x10;
	} else {
		lv5219lg_enable(LV5219LG_CONTROL_OFF, LV5219LG_CHANNEL_MLED);

		reg_LEDCTL1 &= ~(0x10);
	}

	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);
}

static void lv5219lg_mled_normal_intensity_ctrl(unsigned char bright)
{
	if (bright == 0x00) {
		reg_MLEDDACTL |= 0x1F;
	} else {
		reg_MLEDDACTL &= ~(0x1F);
		reg_MLEDDACTL |= (0x1F & bright) - 0x01;
	}

	lv5219lg_i2c_write(LV5219LG_REG_MLEDDACTL, reg_MLEDDACTL);
}

static void lv5219lg_mled_set(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	unsigned char brightness;
	static unsigned char brightness_store = 0x00;
	int inx;

	mutex_lock(&intensity_ctrl_lock);

	brightness = value >> 3;
	inx = ((value & 0xFF) + 1) >> 4;

	ledc_illuminance_sensor_intensity(reg_PTMDACTL_TABLE[inx]);

	if (intensity_ctrl == true) {
		unsigned char level;

		if (lv5219lg_get_sensor_level(&level) != 0)
			level = 0;

		brightness = (reg_PTMDACTL[level] & 0x7C) >> 2;
	}

	if (value == LED_OFF) {
		lv5219lg_mled_normal_ctrl(LV5219LG_CONTROL_OFF);
	} else if (brightness != brightness_store) {
		if (intensity_ctrl == true)
			lv5219lg_mled_auto_intensity_ctrl(LV5219LG_CONTROL_OFF);
		lv5219lg_mled_normal_intensity_ctrl(brightness);
		lv5219lg_mled_normal_ctrl(LV5219LG_CONTROL_ON);
		if (intensity_ctrl == true)
			lv5219lg_mled_auto_intensity_ctrl(LV5219LG_CONTROL_ON);
	}

	brightness_store = brightness;
	mutex_unlock(&intensity_ctrl_lock);
}

static void lv5219lg_mled_auto_intensity_ctrl(unsigned char sw)
{
	if (sw == LV5219LG_CONTROL_ON)
		reg_PTCTL1 |= 0x20;
	else
		reg_PTCTL1 &= ~0x20;

	lv5219lg_i2c_write(LV5219LG_REG_PTCTL1, reg_PTCTL1);
}

/*==============================================================================
									 SLED
==============================================================================*/

static void lv5219lg_sled_ctrl(unsigned char led_kind, unsigned char sw)
{
	if (sw == LV5219LG_CONTROL_ON) {
		lv5219lg_enable(LV5219LG_CONTROL_ON, led_kind << 4);

		reg_LEDCTL2 |= led_kind << 2;

		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL2, reg_LEDCTL2);
	} else {
		reg_LEDCTL2 &= ~(led_kind << 2);

		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL2, reg_LEDCTL2);

		lv5219lg_enable(LV5219LG_CONTROL_OFF, LV5219LG_SLEDALL << 4);
	}
}

static void lv5219lg_sled_intensity_ctrl(unsigned char led_kind,
			unsigned char *bright)
{
	if (led_kind & LV5219LG_SLED1) {
		if (bright[0] == 0x00) {
			reg_S1DACTL |= 0x1F;
		} else {
			reg_S1DACTL &= ~(0x1F);
			reg_S1DACTL |= (0x1F & bright[0]) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_S1DACTL, reg_S1DACTL);
	}

	if (led_kind & LV5219LG_SLED2) {
		if (bright[1] == 0x00) {
			reg_S2DACTL |= 0x1F;
		} else {
			reg_S2DACTL &= ~(0x1F);
			reg_S2DACTL |= (0x1F & bright[1]) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_S2DACTL, reg_S2DACTL);
	}
}

static void lv5219lg_sled_fade_ctrl(unsigned char fade_in,
			unsigned char fade_out)
{
	reg_SFCTL &= ~(0x3F);
	reg_SFCTL |= (fade_out << 3);
	reg_SFCTL |= fade_in;

	lv5219lg_i2c_write(LV5219LG_REG_SFCTL, reg_SFCTL);
}

static void lv5219lg_sled_set(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	unsigned char intensity[2];

	mutex_lock(&intensity_ctrl_lock);

	if (value == LED_OFF) {
		lv5219lg_sled_ctrl(LV5219LG_SLED1, LV5219LG_CONTROL_OFF);
	} else {
		if (value <= LV5219LG_SLED_MIN_INTENSITY_VALUE)
			intensity[0] = 1;
		else
			intensity[0] = value >> 3;

		lv5219lg_sled_fade_ctrl(LV5219LG_SLED_FADE_IN_VALUE,
						LV5219LG_SLED_FADE_OUT_VALUE);

		lv5219lg_sled_intensity_ctrl(LV5219LG_SLED1, intensity);
		lv5219lg_sled_ctrl(LV5219LG_SLEDALL, LV5219LG_CONTROL_ON);
	}
	mutex_unlock(&intensity_ctrl_lock);
}


/*==============================================================================
				 RGB1
==============================================================================*/

static void lv5219lg_rgb1_ctrl(int led, unsigned char sw)
{
	if (sw == LV5219LG_CONTROL_ON) {
		lv5219lg_enable(LV5219LG_CONTROL_ON, led << 6);

		reg_LEDCTL3 |= led;
		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL3, reg_LEDCTL3);
	} else {
		reg_LEDCTL3 &= ~(led);
		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL3, reg_LEDCTL3);

		lv5219lg_enable(LV5219LG_CONTROL_OFF, led << 6);
	}
}

static void lv5219lg_rgb1_intensity_ctrl(unsigned char led_kind,
			unsigned char bright)
{
	if (led_kind & LV5219LG_RLED1) {
		if (bright == 0) {
			reg_R1DACTL |= 0x1F;
		} else {
			reg_R1DACTL &= ~(0x1F);
			reg_R1DACTL |= (0x1F & bright) - 0x01;
		}

		reg_R1DACTL &= ~(0xE0);
		reg_R1DACTL |= LV5219LG_RGB1_MAXIMUM_CURRENT_LIMIT;

		lv5219lg_i2c_write(LV5219LG_REG_R1DACTL, reg_R1DACTL);
	}

	if (led_kind & LV5219LG_GLED1) {
		if (bright == 0) {
			reg_G1DACTL |= 0x1F;
		} else {
			reg_G1DACTL &= ~(0x1F);
			reg_G1DACTL |= (0x1F & bright) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_G1DACTL, reg_G1DACTL);
	}

	if (led_kind & LV5219LG_BLED1) {
		if (bright == 0) {
			reg_B1DACTL |= 0x1F;
		} else {
			reg_B1DACTL &= ~(0x1F);
			reg_B1DACTL |= (0x1F & bright) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_B1DACTL, reg_B1DACTL);
	}
}

/*==============================================================================
							GLADATION / PULSE
==============================================================================*/

static void lv5219lg_fled_ctrl(int led, unsigned char sw)
{
	if (sw == LV5219LG_CONTROL_ON) {
		lv5219lg_enable(LV5219LG_CONTROL_ON, led << 12);

		reg_LEDCTL4 |= led;

		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL4, reg_LEDCTL4);
	} else {
		reg_LEDCTL4 &= ~(led);

		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL4, reg_LEDCTL4);

		lv5219lg_enable(LV5219LG_CONTROL_OFF, led << 12);
	}
}

static void lv5219lg_fled_intensity_ctrl(unsigned char led_kind,
			unsigned char *bright, unsigned char max)
{
	if (led_kind & LV5219LG_FLED1) {
		if (bright[0] == 0x00) {
			reg_F1DACTL |= 0x1F;
		} else {
			reg_F1DACTL &= ~(0x1F);
			reg_F1DACTL |= (0x1F & bright[0]) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_F1DACTL, reg_F1DACTL);
	}

	if (led_kind & LV5219LG_FLED2) {
		if (bright[1] == 0x00) {
			reg_F2DACTL |= 0x1F;
		} else {
			reg_F2DACTL &= ~(0x1F);
			reg_F2DACTL |= (0x1F & bright[1]) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_F2DACTL, reg_F2DACTL);
	}

	if (led_kind & LV5219LG_FLED3) {
		if (bright[2] == 0x00) {
			reg_F3DACTL |= 0x1F;
		} else {
			reg_F3DACTL &= ~(0x1F);
			reg_F3DACTL |= (0x1F & bright[2]) - 0x01;
		}

		lv5219lg_i2c_write(LV5219LG_REG_F3DACTL, reg_F3DACTL);
	}
}

static void lv5219lg_fled_set(struct led_classdev *led_cdev,
						enum led_brightness value)
{
	unsigned char intensity[3] = {0, 0, 0};
	unsigned char enable = LV5219LG_CONTROL_OFF;

	if (value != LED_OFF) {
		if (value <= LV5219LG_FLED_MIN_INTENSITY_VALUE) {
			intensity[0] = 1;
		} else {
			intensity[0] = value >> 3;
			intensity[1] = value >> 3;
			intensity[2] = value >> 3;
		}
		enable = LV5219LG_CONTROL_ON;
	}

	lv5219lg_fled_intensity_ctrl(LV5219LG_FLEDALL, intensity,
					LV5219LG_FLED_MAXIMUM_CURRENT_LIMIT);
	lv5219lg_fled_ctrl(LV5219LG_FLEDALL, enable);

}

/*==============================================================================
				ILLUMINANCE SENSOR
==============================================================================*/

static void lv5219lg_ls_enable(unsigned char sw)
{
	DEBUG(printk(KERN_INFO "%s - Enter\n", __func__);)
	if (sw == LV5219LG_CONTROL_ON)
		reg_PTCTL1 |= 0x40;
	else
		reg_PTCTL1 &= ~0x40;

	lv5219lg_i2c_write(LV5219LG_REG_PTCTL1, reg_PTCTL1);
}

static ssize_t lv5219lg_set_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	unsigned long enable;
	int ret = strict_strtoul(buf, 10, &enable);

	if (ret == 0) {
		mutex_lock(&intensity_ctrl_lock);
		if (enable == LV5219LG_CONTROL_ON) {
			intensity_ctrl = true;
			lv5219lg_ls_enable(LV5219LG_CONTROL_ON);
			lv5219lg_mled_auto_intensity_ctrl(LV5219LG_CONTROL_ON);
		} else {
			intensity_ctrl = false;
			lv5219lg_mled_auto_intensity_ctrl(LV5219LG_CONTROL_OFF);
			lv5219lg_ls_enable(LV5219LG_CONTROL_OFF);
		}
		mutex_unlock(&intensity_ctrl_lock);
	}

	return size;
}

static ssize_t lv5219lg_get_enable(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", intensity_ctrl);
}

static int lv5219lg_get_sensor_level(unsigned char *level)
{
	if (lv5219lg_i2c_read(LV5219LG_REG_STATUS, level))
		return -EFAULT;

	*level >>= 4;

	return 0;
}

static ssize_t ledc_illuminance_sensor_config_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	int argc;
	int inx;
	int val[16];
	int i;

	argc = sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
	 &inx, &val[0], &val[1], &val[2], &val[3], &val[4], &val[5], &val[6],
	 &val[7], &val[8], &val[9], &val[10], &val[11], &val[12], &val[13],
	 &val[14], &val[15]);

	if (argc != 17)
		goto fault;

	if ((inx < 0) || (16 < inx))
		goto fault;

	for (i = 0; i < 16; i++) {
		if ((val[i] < 0) || (255 < val[i]))
			goto fault;
	}

	mutex_lock(&intensity_ctrl_lock);
	for (i = 0; i < 16; i++)
		reg_PTMDACTL_TABLE[inx][i] = (unsigned char)val[i];
	mutex_unlock(&intensity_ctrl_lock);

	return size;

fault:
	printk(KERN_ERR "%s Parameter error\n", __func__);
	return -EINVAL;
}

static int lv5219lg_ls_open(struct inode *inode, struct file *file)
{
	DEBUG(printk(KERN_INFO "%s - Enter\n", __func__);)
	return 0;
}

static int lv5219lg_ls_release(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "%s - Enter\n", __func__);
	lv5219lg_ls_enable(LV5219LG_CONTROL_OFF);
	return 0;
}

static int lv5219lg_ls_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err = 0;

	DEBUG(printk(KERN_INFO "%s - Enter\n", __func__);)

	if (_IOC_TYPE(cmd) != LIGHTSENSOR_IOC_MAGIC) {
		printk(KERN_ERR "%s cmd magic type error\n", __func__);
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > LIGHTSENSOR_IOC_MAXNR) {
		printk(KERN_ERR "%s cmd number error\n", __func__);
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				_IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
				_IOC_SIZE(cmd));
	if (err) {
		printk(KERN_ERR "%s IOCTL error.\n", __func__);
		return -EFAULT;
	}

	switch (cmd) {
	case LIGHTSENSOR_SET_ENABLE:
		lv5219lg_ls_enable(LV5219LG_CONTROL_ON);
		break;
	case LIGHTSENSOR_SET_DISABLE:
		lv5219lg_ls_enable(LV5219LG_CONTROL_OFF);
		break;
	default:
		break;
	}

	return 0;
}

static ssize_t lv5219lg_ls_read(struct file *file,
			char __user *buf, size_t count, loff_t *offset)
{
	unsigned char level;
	unsigned short illum;

	DEBUG(printk(KERN_INFO "%s - Enter\n", __func__);)
	if (lv5219lg_i2c_read(LV5219LG_REG_STATUS, &level))
		return -EFAULT;
	level >>= 4;
	level &= 0x0f;
	if (level >= ARRAY_SIZE(lv5219lg_convert_level_to_lux))
		return -EFAULT;

	illum = lv5219lg_convert_level_to_lux[level];
	if (copy_to_user(buf, &illum, sizeof(illum)))
		return -EFAULT;

	return sizeof(illum);
}

static void ledc_illuminance_sensor_intensity(unsigned char *val)
{
	memcpy(reg_PTMDACTL, val, sizeof(reg_PTMDACTL));

	lv5219lg_i2c_block_write(LV5219LG_REG_PTMDACTL0,
		(unsigned char)ARRAY_SIZE(reg_PTMDACTL), reg_PTMDACTL);
}

static void lv5219lg_enable(unsigned char sw, unsigned int led_chanel)
{
	static unsigned int enabled = LV5219LG_CONTROL_OFF;

	if (sw == LV5219LG_CONTROL_ON)
		enabled |= led_chanel;
	else
		enabled &= ~led_chanel;

	if (enabled) {
		reg_LEDCTL1 |= 0x01;
		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);

		reg_LEDCTL1 |= 0x02;
		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);
	} else {
		reg_LEDCTL1 &= ~(0x03);
		lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);
	}
}

static void lv5219lg_i2c_write(unsigned char reg_addr, unsigned char data)
{
	if (i2c_smbus_write_byte_data(ledc_i2c_client, reg_addr, data) < 0)
		printk(KERN_ERR "%s: I2C write failed!\n", __func__);
}

static void lv5219lg_i2c_block_write(unsigned char reg_addr,
			unsigned char length, unsigned char *data)
{
	if (i2c_smbus_write_i2c_block_data(ledc_i2c_client, reg_addr,
						length, data) < 0)
		printk(KERN_ERR "%s: I2C write failed!\n", __func__);
}

static int lv5219lg_i2c_read(unsigned char reg_addr, unsigned char *data)
{
	int rc;
	DEBUG(printk(KERN_INFO "%s - Enter\n", __func__);)
	rc = i2c_smbus_read_byte_data(ledc_i2c_client, reg_addr);
	if (rc < 0) {
		printk(KERN_ERR "%s: I2C read failed!\n", __func__);
		return -EFAULT;
	}
	*data = rc;
	return 0;
}

static void lv5219lg_rgb_set(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	struct lv5219lg_led *led;

	mutex_lock(&rgb_lock);
	led = container_of(led_cdev, struct lv5219lg_led, ldev);

	lv5219lg_rgb_ctrl(led);
	mutex_unlock(&rgb_lock);
}

static void lv5219lg_rgb_pulse_enable(int led_kind, int value)
{
	static int enabled;

	if (value)
		enabled |= led_kind;
	else
		enabled &= ~led_kind;

	if (enabled) {
		reg_RGB1PUCTL |= 0x40;
		reg_LEDCTL3 &= ~(0x07);
		reg_RGB1GRCTL |= 0x08;
	} else {
		reg_RGB1PUCTL &= ~0x40;
		reg_RGB1GRCTL &= ~(0x08);
	}

	lv5219lg_i2c_write(LV5219LG_REG_RGB1PUCTL, reg_RGB1PUCTL);
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL3, reg_LEDCTL3);
	lv5219lg_i2c_write(LV5219LG_REG_RGB1GRCTL, reg_RGB1GRCTL);
}

static void lv5219lg_rgb_ctrl(struct lv5219lg_led *led)
{
	unsigned long tmp_delay_off;
	unsigned long tmp_delay_on;
	unsigned long total_time;
	unsigned char time_unit;
	unsigned char intensity;

	if (led->blink_on == 0 || led->blink_off == 0) {
		switch (led->led_kind) {
		case LV5219LG_RLED1:
			reg_R1FCTL = 0;
			reg_R1AOFFCTL = 0;
			reg_R1AONCTL = 0;

			lv5219lg_i2c_write(LV5219LG_REG_R1FCTL, reg_R1FCTL);
			lv5219lg_i2c_write(LV5219LG_REG_R1AOFFCTL,
					reg_R1AOFFCTL);
			lv5219lg_i2c_write(LV5219LG_REG_R1AONCTL, reg_R1AONCTL);
			break;
		case LV5219LG_GLED1:
			reg_G1FCTL = 0;
			reg_G1AOFFCTL = 0;
			reg_G1AONCTL = 0;

			lv5219lg_i2c_write(LV5219LG_REG_G1FCTL, reg_G1FCTL);
			lv5219lg_i2c_write(LV5219LG_REG_G1AOFFCTL,
					reg_G1AOFFCTL);
			lv5219lg_i2c_write(LV5219LG_REG_G1AONCTL, reg_G1AONCTL);
			break;
		case LV5219LG_BLED1:
			reg_B1FCTL = 0;
			reg_B1AOFFCTL = 0;
			reg_B1AONCTL = 0;

			lv5219lg_i2c_write(LV5219LG_REG_B1FCTL, reg_B1FCTL);
			lv5219lg_i2c_write(LV5219LG_REG_B1AOFFCTL,
					reg_B1AOFFCTL);
			lv5219lg_i2c_write(LV5219LG_REG_B1AONCTL, reg_B1AONCTL);
			break;
		}

		lv5219lg_rgb_pulse_enable(led->led_kind, 0);

		if (led->ldev.brightness == LED_OFF) {
			lv5219lg_rgb1_intensity_ctrl(led->led_kind, 0);
			lv5219lg_rgb1_ctrl(led->led_kind, LV5219LG_CONTROL_OFF);
		} else {
			intensity = (led->ldev.brightness >> 3) - 1;

			lv5219lg_rgb1_intensity_ctrl(led->led_kind, intensity);
			lv5219lg_rgb1_ctrl(led->led_kind, LV5219LG_CONTROL_ON);
		}
	} else {
		tmp_delay_on  = led->blink_on;
		tmp_delay_off = led->blink_off;

		total_time = tmp_delay_on + tmp_delay_off;

		if ((total_time > 0) && (total_time <= 98))
			time_unit = 0x00;
		else if ((total_time > 98) && (total_time <= 197))
			time_unit = 0x01;
		else if ((total_time > 197) && (total_time <= 393))
			time_unit = 0x02;
		else if ((total_time > 393) && (total_time <= 786))
			time_unit = 0x03;
		else if ((total_time > 786) && (total_time <= 1573))
			time_unit = 0x04;
		else if ((total_time > 1573) && (total_time <= 3146))
			time_unit = 0x05;
		else if ((total_time > 3146) && (total_time <= 6291))
			time_unit = 0x06;
		else
			time_unit = 0x07;

		tmp_delay_off = tmp_delay_on * 64 / (total_time);

		if (tmp_delay_off <= 0)
			tmp_delay_off = 1;
		else if (tmp_delay_off >= 64)
			tmp_delay_off = 63;

		reg_RGB1GRCTL &= ~7;
		reg_RGB1GRCTL |= time_unit;

		switch (led->led_kind) {
		case LV5219LG_RLED1:
			reg_R1FCTL = 3 | (3 << 3);
			reg_R1AOFFCTL = tmp_delay_off;
			reg_R1AONCTL = 0;

			lv5219lg_i2c_write(LV5219LG_REG_R1FCTL, reg_R1FCTL);
			lv5219lg_i2c_write(LV5219LG_REG_R1AOFFCTL,
					reg_R1AOFFCTL);
			lv5219lg_i2c_write(LV5219LG_REG_R1AONCTL, reg_R1AONCTL);
			break;
		case LV5219LG_GLED1:
			reg_G1FCTL = 3 | (3 << 3);
			reg_G1AOFFCTL = tmp_delay_off;
			reg_G1AONCTL = 0;

			lv5219lg_i2c_write(LV5219LG_REG_G1FCTL, reg_G1FCTL);
			lv5219lg_i2c_write(LV5219LG_REG_G1AOFFCTL,
					reg_G1AOFFCTL);
			lv5219lg_i2c_write(LV5219LG_REG_G1AONCTL, reg_G1AONCTL);
			break;
		case LV5219LG_BLED1:
			reg_B1FCTL = 3 | (3 << 3);
			reg_B1AOFFCTL = tmp_delay_off;
			reg_B1AONCTL = 0;

			lv5219lg_i2c_write(LV5219LG_REG_B1FCTL, reg_B1FCTL);
			lv5219lg_i2c_write(LV5219LG_REG_B1AOFFCTL,
					reg_B1AOFFCTL);
			lv5219lg_i2c_write(LV5219LG_REG_B1AONCTL, reg_B1AONCTL);
			break;
		}

		lv5219lg_i2c_write(LV5219LG_REG_RGB1GRCTL, reg_RGB1GRCTL);

		if (led->ldev.brightness == LED_OFF) {
			lv5219lg_rgb1_intensity_ctrl(led->led_kind, 0);
			lv5219lg_rgb1_ctrl(led->led_kind, LV5219LG_CONTROL_OFF);
		} else {
			intensity = (led->ldev.brightness >> 3) - 1;

			lv5219lg_rgb1_intensity_ctrl(led->led_kind, intensity);
			lv5219lg_rgb1_ctrl(led->led_kind, LV5219LG_CONTROL_ON);
		}
		lv5219lg_rgb_pulse_enable(led->led_kind, 1);
	}
}

static ssize_t lv5219lg_rgb_blink_on_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lv5219lg_led *led =
			container_of(led_cdev, struct lv5219lg_led, ldev);

	return sprintf(buf, "%lu\n", led->blink_on);
}

static ssize_t lv5219lg_rgb_blink_on_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lv5219lg_led *led =
			container_of(led_cdev, struct lv5219lg_led, ldev);
	unsigned long state;
	int ret = strict_strtoul(buf, 10, &state);

	if (ret == 0) {
		mutex_lock(&rgb_lock);
		if (led->blink_on != state) {
			led->blink_on = state;

			lv5219lg_rgb_ctrl(led);
		}
		mutex_unlock(&rgb_lock);
		ret = size;
	}

	return ret;
}

static ssize_t lv5219lg_rgb_blink_off_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lv5219lg_led *led =
			container_of(led_cdev, struct lv5219lg_led, ldev);

	return sprintf(buf, "%lu\n", led->blink_off);
}

static ssize_t lv5219lg_rgb_blink_off_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lv5219lg_led *led =
			container_of(led_cdev, struct lv5219lg_led, ldev);
	unsigned long state;
	int ret = strict_strtoul(buf, 10, &state);

	if (ret == 0) {
		mutex_lock(&rgb_lock);
		if (led->blink_off != state) {
			led->blink_off = state;

			lv5219lg_rgb_ctrl(led);
		}
		mutex_unlock(&rgb_lock);
		ret = size;
	}

	return ret;
}

static ssize_t lv5219lg_fled_enable_get(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", !!(led_cdev->brightness));
}

static ssize_t lv5219lg_fled_enable_set(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (!buf || !size)
		return -EINVAL;

	ret = strict_strtoul(buf, 10, &enable);

	if (ret != 0)
		return -EINVAL;

	if (!!enable)
		led_cdev->brightness = 255;
	else
		led_cdev->brightness = 0;

	lv5219lg_fled_set(led_cdev, led_cdev->brightness);

	return size;
}

static DEVICE_ATTR(blink_on, 0644,
		lv5219lg_rgb_blink_on_show, lv5219lg_rgb_blink_on_store);
static DEVICE_ATTR(blink_off, 0644,
		lv5219lg_rgb_blink_off_show, lv5219lg_rgb_blink_off_store);
static DEVICE_ATTR(spotlight_enable, 0666,
		lv5219lg_fled_enable_get, lv5219lg_fled_enable_set);
static DEVICE_ATTR(als_enable, 0600, lv5219lg_get_enable, lv5219lg_set_enable);
static DEVICE_ATTR(als_config, 0200, NULL, ledc_illuminance_sensor_config_set);

static struct lv5219lg_led lv5219lg_rgb1[] = {
	{
		.ldev = {
			.name			= "lv5219lg:rgb1:red",
			.brightness_set		= lv5219lg_rgb_set,
		},
		.led_kind		= LV5219LG_RLED1,
	},
	{
		.ldev = {
			.name			= "lv5219lg:rgb1:green",
			.brightness_set		= lv5219lg_rgb_set,
		},
		.led_kind		= LV5219LG_GLED1,
	},
	{
		.ldev = {
			.name			= "lv5219lg:rgb1:blue",
			.brightness_set		= lv5219lg_rgb_set,
		},
		.led_kind		= LV5219LG_BLED1,
	},
};

static struct led_classdev lv5219lg_fled = {
	.name			= "lv5219lg:fled",
	.brightness_set		= lv5219lg_fled_set,
};

static struct led_classdev lv5219lg_mled = {
	.name			= "lv5219lg:mled",
	.brightness_set	= lv5219lg_mled_set,
};

static struct led_classdev lv5219lg_sled = {
	.name			= "lv5219lg:sled",
	.brightness_set	= lv5219lg_sled_set,
};

static struct file_operations lv5219lg_ls_fops = {
		.owner = THIS_MODULE,
		.open = lv5219lg_ls_open,
		.release = lv5219lg_ls_release,
		.read = lv5219lg_ls_read,
		.ioctl = lv5219lg_ls_ioctl,
};

static struct miscdevice lv5219lg_ls_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = LV5219LG_LS_DEV_NAME,
		.fops = &lv5219lg_ls_fops,
};

static int lv5219lg_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int lv5219lg_resume(struct i2c_client *client)
{
	return 0;
}

static long lv5219lg_panic_blink(long time)
{
#define PANIC_BRIGHTNESS ((LED_HALF / 8) - 1)
	static bool initialized = false;

	mutex_lock(&rgb_lock);

	if (initialized)
		goto exit;

	initialized = true;

	reg_MLEDDACTL |= 0x1F;
	reg_LEDCTL1 &= ~(0x10);
	lv5219lg_i2c_write(LV5219LG_REG_MLEDDACTL, reg_MLEDDACTL);
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);

	reg_RGB1GRCTL &= ~7;
	reg_RGB1GRCTL |= 3;
	lv5219lg_i2c_write(LV5219LG_REG_RGB2GRCTL, reg_RGB1GRCTL);

	reg_R1FCTL = 3 | (3 << 3);
	reg_R1AONCTL = 0;
	reg_R1AOFFCTL = 20;
	reg_R1DACTL &= ~0x1f;
	reg_R1DACTL |= PANIC_BRIGHTNESS;
	lv5219lg_i2c_write(LV5219LG_REG_R1FCTL, reg_R1FCTL);
	lv5219lg_i2c_write(LV5219LG_REG_R1AONCTL, reg_R1AONCTL);
	lv5219lg_i2c_write(LV5219LG_REG_R1AOFFCTL, reg_R1AOFFCTL);
	lv5219lg_i2c_write(LV5219LG_REG_R1DACTL, reg_R1DACTL);

	reg_G1FCTL = 3 | (3 << 3);
	reg_G1AONCTL = 21;
	reg_G1AOFFCTL = 42;
	reg_G1DACTL &= ~0x1f;
	reg_G1DACTL |= PANIC_BRIGHTNESS;
	lv5219lg_i2c_write(LV5219LG_REG_G1FCTL, reg_G1FCTL);
	lv5219lg_i2c_write(LV5219LG_REG_G1AONCTL, reg_G1AONCTL);
	lv5219lg_i2c_write(LV5219LG_REG_G1AOFFCTL, reg_G1AOFFCTL);
	lv5219lg_i2c_write(LV5219LG_REG_G1DACTL, reg_G1DACTL);

	reg_B1FCTL = 3 | (3 << 3);
	reg_B1AONCTL = 43;
	reg_B1AOFFCTL = 63;
	reg_B1DACTL &= ~0x1f;
	reg_B1DACTL |= PANIC_BRIGHTNESS;
	lv5219lg_i2c_write(LV5219LG_REG_B1FCTL, reg_B1FCTL);
	lv5219lg_i2c_write(LV5219LG_REG_B1AONCTL, reg_B1AONCTL);
	lv5219lg_i2c_write(LV5219LG_REG_B1AOFFCTL, reg_B1AOFFCTL);
	lv5219lg_i2c_write(LV5219LG_REG_B1DACTL, reg_B1DACTL);

	reg_LEDCTL3 |= LV5219LG_RGBLED1;
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL3, reg_LEDCTL3);

	reg_LEDCTL1 |= 1;
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);

	reg_LEDCTL1 |= 2;
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);

exit: mutex_unlock(&rgb_lock);
	return 0;
}

static int lv5219lg_unconfigure(void)
{
	int i;
	struct lv5219lg_led *led;

	for (i = 0; i < ARRAY_SIZE(lv5219lg_rgb1); i++) {
		led = &lv5219lg_rgb1[i];
		device_remove_file(led->ldev.dev, &dev_attr_blink_off);
		device_remove_file(led->ldev.dev, &dev_attr_blink_on);
		led_classdev_unregister(&led->ldev);
	}

	device_remove_file(lv5219lg_fled.dev, &dev_attr_spotlight_enable);
	led_classdev_unregister(&lv5219lg_fled);
	device_remove_file(lv5219lg_mled.dev, &dev_attr_als_enable);
	device_remove_file(lv5219lg_mled.dev, &dev_attr_als_config);
	led_classdev_unregister(&lv5219lg_mled);
	led_classdev_unregister(&lv5219lg_sled);
	misc_deregister(&lv5219lg_ls_device);

	return 0;
}

static int lv5219lg_configure(struct i2c_client *client)
{
	struct lv5219lg_led *led;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(lv5219lg_rgb1); i++) {
		led = &lv5219lg_rgb1[i];

		ret = led_classdev_register(&ledc_i2c_client->dev, &led->ldev);
		if (ret < 0) {
			printk(KERN_ERR "%s: led_classdev_register %s failed!\n",
					__func__, led->ldev.name);
			goto device_failed_rgb;
		}

		ret = device_create_file(led->ldev.dev, &dev_attr_blink_on);
		if (ret) {
			printk(KERN_ERR "%s: failed to register blink_on\n",
					__func__);
			goto device_failed_blkon;
		}

		ret = device_create_file(led->ldev.dev, &dev_attr_blink_off);
		if (ret) {
			printk(KERN_ERR "%s: failed to register blink_off\n",
					__func__);
			goto device_failed_blkoff;
		}
	}

	ret = led_classdev_register(&client->dev, &lv5219lg_fled);
	if (ret < 0) {
		printk(KERN_ERR "%s: led_classdev_register fled failed!\n",
				__func__);
		goto device_failed_fled;
	}

	ret = device_create_file(lv5219lg_fled.dev, &dev_attr_spotlight_enable);
	if (ret) {
		printk(KERN_ERR "%s: fled/spotlight_enable failed!\n",
				__func__);
		goto device_failed_fled_se;
	}

	ret = led_classdev_register(&client->dev, &lv5219lg_mled);
	if (ret < 0) {
		printk(KERN_ERR "%s: led_classdev_register fled failed!\n",
				__func__);
		goto device_failed_mled;
	}

	ret = device_create_file(lv5219lg_mled.dev, &dev_attr_als_enable);
	if (ret) {
		printk(KERN_ERR "%s: mled/als_enable failed!\n", __func__);
		goto device_failed_mled_ae;
	}

	ret = device_create_file(lv5219lg_mled.dev, &dev_attr_als_config);
	if (ret) {
		printk(KERN_ERR "%s: mled/als_config failed!\n", __func__);
		goto device_failed_mled_ac;
	}

	ret = led_classdev_register(&client->dev, &lv5219lg_sled);
	if (ret < 0) {
		printk(KERN_ERR "%s: led_classdev_register sled failed!\n",
				__func__);
		goto device_failed_sled;
	}

	ret = misc_register(&lv5219lg_ls_device);
	if (ret) {
		printk(KERN_ERR "%s: lv5219lg_ls_device failed!\n", __func__);
		goto device_failed_ls;
	}
	return 0;

device_failed_ls:
	led_classdev_unregister(&lv5219lg_sled);
device_failed_sled:
	device_remove_file(lv5219lg_mled.dev, &dev_attr_als_config);
device_failed_mled_ac:
	device_remove_file(lv5219lg_mled.dev, &dev_attr_als_enable);
device_failed_mled_ae:
	led_classdev_unregister(&lv5219lg_mled);
device_failed_mled:
	device_remove_file(lv5219lg_fled.dev, &dev_attr_spotlight_enable);
device_failed_fled_se:
	led_classdev_unregister(&lv5219lg_fled);
device_failed_fled:
	for (i = 0; i < ARRAY_SIZE(lv5219lg_rgb1); i++) {
		led = &lv5219lg_rgb1[i];
		device_remove_file(led->ldev.dev, &dev_attr_blink_off);
		device_remove_file(led->ldev.dev, &dev_attr_blink_on);
		led_classdev_unregister(&led->ldev);
	}
	return -ENODEV;

device_failed_blkoff:
		device_remove_file(led->ldev.dev, &dev_attr_blink_on);
device_failed_blkon:
		led_classdev_unregister(&led->ldev);
device_failed_rgb:
	while (i--) {
		led = &lv5219lg_rgb1[i];
		device_remove_file(led->ldev.dev, &dev_attr_blink_off);
		device_remove_file(led->ldev.dev, &dev_attr_blink_on);
		led_classdev_unregister(&led->ldev);
	}

	return -ENODEV;
}

static void lv5219lg_device_init(void)
{
	lv5219lg_i2c_write(LV5219LG_REG_INTMASK, 0x3F);

	reg_LEDCTL1 = 0xe0;
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL1, reg_LEDCTL1);
	lv5219lg_mled_normal_intensity_ctrl(LED_FULL / 8);
	lv5219lg_mled_normal_ctrl(LV5219LG_CONTROL_ON);


	reg_LEDCTL2 = 0;
	lv5219lg_i2c_write(LV5219LG_REG_LEDCTL2, reg_LEDCTL2);

	reg_SFCTL = 0x3c;
	lv5219lg_i2c_write(LV5219LG_REG_SFCTL, reg_SFCTL);

	reg_R1DACTL = reg_G1DACTL= reg_B1DACTL = 0x1F;
	reg_R1DACTL |= LV5219LG_RGB1_MAXIMUM_CURRENT_LIMIT;
	lv5219lg_i2c_write(LV5219LG_REG_R1DACTL, reg_R1DACTL);
	lv5219lg_i2c_write(LV5219LG_REG_G1DACTL, reg_G1DACTL);
	lv5219lg_i2c_write(LV5219LG_REG_B1DACTL, reg_B1DACTL);

	lv5219lg_i2c_block_write(LV5219LG_REG_PTMDACTL0,
		(unsigned char)ARRAY_SIZE(reg_PTMDACTL), reg_PTMDACTL);

	reg_PTCTL1 |= 0x18;
	lv5219lg_i2c_write(LV5219LG_REG_PTCTL1, reg_PTCTL1);

	reg_PTCTL3 |= 0x34;
	lv5219lg_i2c_write(LV5219LG_REG_PTCTL3, reg_PTCTL3);

	reg_PTCTL2 |= 0x6C;
	lv5219lg_i2c_write(LV5219LG_REG_PTCTL2, reg_PTCTL2);
}

static int lv5219lg_probe(struct i2c_client *client,
			const struct i2c_device_id *device_id)
{
	ledc_i2c_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	if (lv5219lg_configure(client) < 0) {
		printk(KERN_ERR "%s: failed to create sysFS IF\n", __func__);
		return -ENODEV;
	}

	mutex_init(&intensity_ctrl_lock);

	mutex_init(&rgb_lock);

	lv5219lg_device_init();

	panic_blink = lv5219lg_panic_blink;

	printk(KERN_INFO "lv5219lg probed\n");

	return 0;
}

static int lv5219lg_remove(struct i2c_client *client)
{
	mutex_destroy(&intensity_ctrl_lock);

	mutex_destroy(&rgb_lock);

	lv5219lg_unconfigure();
	i2c_set_clientdata(client, NULL);

	panic_blink = NULL;

	return 0;
}

static struct i2c_driver lv5219lg_driver = {
	.driver = {
		.name  = "lv5219lg",
		.owner = THIS_MODULE,
	},
	.probe	= lv5219lg_probe,
	.suspend = lv5219lg_suspend,
	.resume  = lv5219lg_resume,
	.remove = __devexit_p(lv5219lg_remove),
	.id_table = lv5219lg_id,
};

static int __init lv5219lg_init(void)
{
	return i2c_add_driver(&lv5219lg_driver);
}

static void __exit lv5219lg_exit(void)
{
	i2c_del_driver(&lv5219lg_driver);
}

module_init(lv5219lg_init);
module_exit(lv5219lg_exit);

MODULE_AUTHOR("SEMC")
MODULE_DESCRIPTION("LV5219LG LED IC driver");
MODULE_LICENSE("GPL");
