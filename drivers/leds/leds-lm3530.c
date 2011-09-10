/*
 * lm3550.c -  LM3530 LMU driver
 *
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/leds-lm3530.h>

#define DBG(X) /*X*/

#define DRV_NAME "lm3530"
#define DRIVER_VERSION  "0.2"

/* General configuration register */
#define LM3530_BR_MAP_BITS        0x01
#define LM3530_BR_MAP_SHIFT       1

#define LM3530_MODE_I2C_EN        (1<<0)
#define LM3530_MODE_PWM_EN        (1<<5)
#define LM3530_MODE_SIMPLE_EN     (1<<7)
#define LM3530_MODE_BITS          \
(LM3530_MODE_I2C_EN | LM3530_MODE_PWM_EN | LM3530_MODE_SIMPLE_EN)
#define LM3530_MODE_I2C_SET        LM3530_MODE_I2C_EN
#define LM3530_MODE_I2C_PWM_SET   (LM3530_MODE_I2C_EN | LM3530_MODE_PWM_EN)
#define LM3530_MODE_PWM_SET       (LM3530_MODE_SIMPLE_EN | LM3530_MODE_PWM_EN)

#define LM3530_PWM_POLARITY_BITS   0x01
#define LM3530_PWM_POLARITY_SHIFT  6

#define LM3530_FULL_SCALE_BITS     0x07
#define LM3530_FULL_SCALE_SHIFT    2

/* ALS configuration register */
#define LM3530_ALS_AVG_TIME_BITS   0x07
#define LM3530_ALS_AVG_TIME_SHIFT  0

#define LM3530_ALS_INPUT_BITS      0x03
#define LM3530_ALS_INPUT_SHIFT     5


#define LM3530_ALS_MODE_CTL        (0x01 << 4)
#define LM3530_ALS_MODE_ENABLE     (0x01 << 3)
#define LM3530_ALS_MODE_BITS       (LM3530_ALS_MODE_CTL | LM3530_ALS_MODE_ENABLE)

/* Brightness ramp rate register */
#define LM3530_BRR_DN_BITS          0x07
#define LM3530_BRR_DN_SHIFT         0
#define LM3530_BRR_UP_BITS          0x07
#define LM3530_BRR_UP_SHIFT         3

/* ALS zone information register  R/0 */
#define LM3530_ALS_FLAG_BITS        0x01
#define LM3530_ALS_FLAG_SHIFT       3

#define LM3530_ALS_ZINFO_BITS       0x07
#define LM3530_ALS_ZINFO_SHIFT      0

/* ALS resistor select register   */
#define LM3530_ALS1_RSEL_BITS       0x0f
#define LM3530_ALS1_RSEL_SHIFT      0
#define LM3530_ALS2_RSEL_BITS       0x0f
#define LM3530_ALS2_RSEL_SHIFT      4

/* Brightnes control register   */
#define LM3530_BRIGNTNESS_BITS      0x7f
#define LM3530_BRIGNTNESS_SHIFT     0

/* Zone Boundary register  */
#define LM3530_ZB_BITS              0xff
#define LM3530_ZB_SHIFT             0

/* Zone target  register  */
#define LM3530_ZT_BITS              0x7f
#define LM3530_ZT_SHIFT             0

/* register addresses */
enum lm3530_regs {
	G_CONFIG	= 0,
	ALS_CONFIG,
	BR_RATE,
	ZONE_INF,
	ALS_RES,
	BRIGHTNESS,
	ZBOUNDARY0,
	ZBOUNDARY1,
	ZBOUNDARY2,
	ZBOUNDARY3,
	ZTARGET0,
	ZTARGET1,
	ZTARGET2,
	ZTARGET3,
	ZTARGET4,
	NUM_OF_REGS
};

const static u16 lm3530_addresses[NUM_OF_REGS] = {
	0x10, 0x20, 0x30, 0x40, 0x41, 0xa0,
	0x60, 0x61,	0x62, 0x63,
	0x70, 0x71, 0x72, 0x73, 0x74
};

#define REG_ADDR(REG) lm3530_addresses[REG]

enum lm3530_mode {
  LM3530_MODE_I2C,
  LM3530_MODE_SIMPLE_PWM,
  LM3530_MODE_I2C_PWM,
  LM3530_MODE_SIMPLE_ALS,
  LM3530_MODE_PWM_ALS,
  LM3530_MODE_I2C_PWM_ALS,
  LM3530_MODE_I2C_ALS,
  LM3530_MODE_UNKNOWN,
};

enum lm3530_pwm_polarity {
  LM3530_PWM_POL_ACTIVE_HIGH,
  LM3530_PWM_POL_ACTIVE_LOW
};

enum lm3530_br_mapping_mode {
  LM3530_BRIGHTNESS_MAP_EXP,
  LM3530_BRIGHTNESS_MAP_LINEAR
};

enum lm3530_fsc {
	LM3530_FSC_05_0,
	LM3530_FSC_08_5,
	LM3530_FSC_12_0,
	LM3530_FSC_15_5,
	LM3530_FSC_19_0,
	LM3530_FSC_22_5,
	LM3530_FSC_26_0,
	LM3530_FSC_29_5,
};

enum lm3530_als_input {
	LM3530_ALS_INPUT_ALS_AVG,
	LM3530_ALS_INPUT_ALS_1,
	LM3530_ALS_INPUT_ALS_2,
};


enum lm3530_als_avgt {
	LM3530_ALS_AVG_TIME_0032,
	LM3530_ALS_AVG_TIME_0064,
	LM3530_ALS_AVG_TIME_0128,
	LM3530_ALS_AVG_TIME_0256,
	LM3530_ALS_AVG_TIME_0512,
	LM3530_ALS_AVG_TIME_1024,
	LM3530_ALS_AVG_TIME_2048,
	LM3530_ALS_AVG_TIME_4096,
};

enum lm3530_br_rate {
	LM3530_BRR_00008,
	LM3530_BRR_01024,
	LM3530_BRR_02048,
	LM3530_BRR_04096,
	LM3530_BRR_08192,
	LM3530_BRR_16384,
	LM3530_BRR_32768,
	LM3530_BRR_65538,
};

enum lm3530_als_resist {
	LM3530_ALS_RES_HIGH_Z,
	LM3530_ALS_RES_9_36,
	LM3530_ALS_RES_5_56,
	LM3530_ALS_RES_2_32,
	LM3530_ALS_RES_1_98,
	LM3530_ALS_RES_1_85,
	LM3530_ALS_RES_1_63,
	LM3530_ALS_RES_1_16,
	LM3530_ALS_RES_1_07,
	LM3530_ALS_RES_1_03,
	LM3530_ALS_RES_0_9579,
	LM3530_ALS_RES_0_7717,
	LM3530_ALS_RES_0_7311,
	LM3530_ALS_RES_0_7123,
	LM3530_ALS_RES_0_6776,
};

enum lm3530_cfg {
	LM3530_CFG_BR_MAPPING,
	LM3530_CFG_FSC,
	LM3530_CFG_PWM_POLARITY,
	LM3530_CFG_ALS_AVG_TIME,
	LM3530_CFG_ALS_INPUT,
	LM3530_CFG_BRR_UP,
	LM3530_CFG_BRR_DOWN,
	LM3530_CFG_ALS1_RESISTOR,
	LM3530_CFG_ALS2_RESISTOR,
	LM3530_CFG_ZB_0,
	LM3530_CFG_ZB_1,
	LM3530_CFG_ZB_2,
	LM3530_CFG_ZB_3,
	LM3530_CFG_ZT_0,
	LM3530_CFG_ZT_1,
	LM3530_CFG_ZT_2,
	LM3530_CFG_ZT_3,
	LM3530_CFG_ZT_4,
};

struct lm3530_data {
	struct i2c_client*	client;
	struct work_struct als_work;
	struct early_suspend suspend;
	struct mutex update_lock;
	u8 shadow[NUM_OF_REGS];
	u8 brightness;
	enum lm3530_mode mode;
	u8 als_zone;
	u8 max_brightness;
	int zone_irq_gpio;
	int hw_enable_gpio;
	int (*power_up)(int enable);
	int suspended:1;
};

#define LOCK(p) do { \
	DBG(printk("%s: lock\n", __func__);) \
	mutex_lock(&p->update_lock); \
} while (0);

#define UNLOCK(p) do { \
	DBG(printk("%s: unlock\n", __func__);) \
	mutex_unlock(&p->update_lock); \
} while (0);

/* Helper functions to read and write to the chip */
static int lm3530_i2c_read(struct i2c_client* client, u8 reg, u8 *value)
{
	struct i2c_msg msgs[2];
	int return_value;

	/* Split into two message to get it right */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = value;

	return_value = i2c_transfer(client->adapter, msgs, 2);
	if (return_value < 0)
		printk(KERN_ERR "%s: Error reading register 0x%x\n",
				__func__, reg);
	return return_value > 0 ? 0 : -EIO;
}

/* Helper functions to read and write to the chip */
static int lm3530_i2c_write(struct i2c_client* client, u8 reg, u8 value)
{
	struct i2c_msg msg;
	u8 regs[2] = {reg, value};
	int return_value;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = regs;

	return_value = i2c_transfer(client->adapter, &msg, 1);
	if (return_value < 0)
		printk(KERN_ERR "%s: Error writing register 0x%x\n",
				__func__, reg);
	return return_value > 0 ? 0 : -EIO;
}

static int lm3530_write(struct i2c_client* client, u8 regn, u8  value)
{
	int rc = 0;
	struct lm3530_data *data = i2c_get_clientdata(client);

	DBG(printk(KERN_DEBUG "%s reg = %d (0x%02x), value 0x%02x\n",
			   __func__, regn, REG_ADDR(regn), value);)
	if (data->suspended)
		data->shadow[regn] = value;
	else if(value != data->shadow[regn]) {
		rc = lm3530_i2c_write(client, REG_ADDR(regn), value);
		if (!rc)
			data->shadow[regn] = value;
	}
	return rc;
}

static int lm3530_read(struct i2c_client* client, u8 regn, u8* value)
{
	int rc = 0;
	struct lm3530_data *data = i2c_get_clientdata(client);

	DBG(printk(KERN_DEBUG "%s reg = %d (0x%02x)\n",
			   __func__, regn, REG_ADDR(regn));)
	if (data->suspended)
		*value = data->shadow[regn];
	else {
		rc = lm3530_i2c_read(client, REG_ADDR(regn), value);
		if(!rc)
			data->shadow[regn] = *value;
	}
	return rc;
}

static int lm3530_restore_regs(struct i2c_client* client)
{
	int i;
	int rc = 0;
	struct lm3530_data *data = i2c_get_clientdata(client);

	for (i = 0; i < NUM_OF_REGS && !rc; i++) {
		if (i != BRIGHTNESS)
			rc = lm3530_i2c_write(client, REG_ADDR(i),
					data->shadow[i]);
		else {
			/*
			* We restore with 0 brightness to avoid flickering
			* while resuming from suspend
			*/
			rc = lm3530_i2c_write(client, BRIGHTNESS, 0);
			data->shadow[BRIGHTNESS] = 0;
		}
	}
	return rc;
}

static int lm3530_hw_enable(struct lm3530_data *data, int enable)
{
	int rc = 0;

	DBG(printk(KERN_DEBUG "%s: enable = %d\n", __func__, enable);)

	if (data->hw_enable_gpio > 0) {
		rc = gpio_direction_output(data->hw_enable_gpio, !!enable);
		if (rc)
			return rc;
	}
	if (data->power_up)
		rc = data->power_up(!!enable);
	return rc;
}

static int lm3530_change_config(struct i2c_client* client,
			 enum lm3530_cfg cfg, u8 value)
{
	int rc;
	struct lm3530_data *data = i2c_get_clientdata(client);

#define cfg_bits(REG,CFG,VAL) do { \
	u8 tmp = (data->shadow[REG] & \
		  (~(LM3530_##CFG##_BITS << LM3530_##CFG##_SHIFT))) | \
		(((VAL) & LM3530_##CFG##_BITS) << LM3530_##CFG##_SHIFT); \
	DBG(printk(KERN_DEBUG "%s: configurtion %d = %d" \
		   " (0x%02x -> reg 0x%02x)\n", \
		   __func__, cfg, value, tmp, REG_ADDR(REG));) \
	rc = lm3530_write(client, REG, tmp); \
} while (0);

	switch(cfg) {
	case LM3530_CFG_BR_MAPPING:
		cfg_bits(G_CONFIG, BR_MAP, value);
		break;
	case LM3530_CFG_FSC:
		cfg_bits(G_CONFIG, FULL_SCALE, value);
		break;
	case LM3530_CFG_PWM_POLARITY:
		cfg_bits(G_CONFIG, PWM_POLARITY, value);
		break;
	case LM3530_CFG_ALS_AVG_TIME:
		cfg_bits(ALS_CONFIG, ALS_AVG_TIME, value);
		break;
	case LM3530_CFG_ALS_INPUT:
		cfg_bits(ALS_CONFIG, ALS_INPUT, value);
		break;
	case LM3530_CFG_BRR_UP:
		cfg_bits(BR_RATE, BRR_UP, value);
		break;
	case LM3530_CFG_BRR_DOWN:
		cfg_bits(BR_RATE, BRR_DN, value);
		break;
	case LM3530_CFG_ALS1_RESISTOR:
		cfg_bits(ALS_RES, ALS1_RSEL, value);
		break;
	case LM3530_CFG_ALS2_RESISTOR:
		cfg_bits(ALS_RES, ALS2_RSEL, value);
		break;
	case LM3530_CFG_ZB_0:
		cfg_bits(ZBOUNDARY0, ZB, value);
		break;
	case LM3530_CFG_ZB_1:
		cfg_bits(ZBOUNDARY1, ZB, value);
		break;
	case LM3530_CFG_ZB_2:
		cfg_bits(ZBOUNDARY2, ZB, value);
		break;
	case LM3530_CFG_ZB_3:
		cfg_bits(ZBOUNDARY3, ZB, value);
		break;
	case LM3530_CFG_ZT_0:
		cfg_bits(ZTARGET0, ZT, value >> 1);
		break;
	case LM3530_CFG_ZT_1:
		cfg_bits(ZTARGET1, ZT, value >> 1);
		break;
	case LM3530_CFG_ZT_2:
		cfg_bits(ZTARGET2 ,ZT, value >> 1);
		break;
	case LM3530_CFG_ZT_3:
		cfg_bits(ZTARGET3, ZT, value >> 1);
		break;
	case LM3530_CFG_ZT_4:
		cfg_bits(ZTARGET4, ZT, value >> 1);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int lm3530_config(struct i2c_client* client,
						 enum lm3530_cfg cfg, u8 value)
{
	int rc;
	struct lm3530_data *data = i2c_get_clientdata(client);
	LOCK(data);
	rc = lm3530_change_config(client, cfg, value);
	UNLOCK(data);
	return rc;
}

static int lm3530_mode_set(struct i2c_client* client, int mode)
{
	int rc;
	struct lm3530_data *data = i2c_get_clientdata(client);
	u8 mode_bits;
	u8 als_bits;

	DBG(printk(KERN_DEBUG "%s: mode = %d\n", __func__, mode);)
	if (mode == LM3530_MODE_UNKNOWN)
		return -EINVAL;

	LOCK(data);
	mode_bits = data->shadow[G_CONFIG]   & ~LM3530_MODE_BITS;
	als_bits  = data->shadow[ALS_CONFIG] & ~LM3530_ALS_MODE_BITS;
	switch(mode)
	{
	case LM3530_MODE_I2C:
	case LM3530_MODE_I2C_ALS:
		mode_bits |= LM3530_MODE_I2C_EN;
		als_bits |= LM3530_ALS_MODE_ENABLE;
		break;
	case LM3530_MODE_SIMPLE_PWM:
		mode_bits |= LM3530_MODE_PWM_EN | LM3530_MODE_SIMPLE_EN;
		als_bits |= LM3530_ALS_MODE_ENABLE;
		break;
	case LM3530_MODE_I2C_PWM:
		mode_bits |= LM3530_MODE_PWM_EN | LM3530_MODE_I2C_EN;
		als_bits |= LM3530_ALS_MODE_ENABLE;
		break;
	case LM3530_MODE_SIMPLE_ALS:
		mode_bits |= LM3530_MODE_I2C_SET;
		als_bits |= LM3530_ALS_MODE_ENABLE | LM3530_ALS_MODE_CTL;
		break;
	case LM3530_MODE_PWM_ALS:
		mode_bits |= LM3530_MODE_PWM_EN | LM3530_MODE_I2C_EN;
		als_bits |= LM3530_ALS_MODE_ENABLE | LM3530_ALS_MODE_CTL;
		break;
	case LM3530_MODE_I2C_PWM_ALS:
		mode_bits |= LM3530_MODE_PWM_EN | LM3530_MODE_I2C_EN;
		als_bits |= LM3530_ALS_MODE_ENABLE;
		break;
	default:
		return -EINVAL;
	}
	data->mode = mode;
	rc = lm3530_write(client, G_CONFIG, mode_bits);
	if (!rc)
		rc = lm3530_write(client, ALS_CONFIG, als_bits);
	UNLOCK(data);
	return rc;
}

static int lm3530_update_brightness(struct i2c_client* client)
{
	int rc;
	u16 composit_br;
	struct lm3530_data *data = i2c_get_clientdata(client);

	LOCK(data);
	composit_br = data->brightness;
	if (data->mode == LM3530_MODE_I2C_PWM_ALS ||
					data->mode == LM3530_MODE_I2C_ALS) {
		composit_br *= (data->shadow[ZTARGET0 + data->als_zone] &
						LM3530_ZT_BITS);
		composit_br >>= 7;
		DBG(printk(KERN_DEBUG
			   "%s: using composit brightness: (%d x %d)>>8 = %d\n",
			   __func__,
			   data->brightness,
			   data->shadow[ZTARGET0 + data->als_zone],
			   composit_br);)
	}

	if (composit_br > data->max_brightness)
		composit_br = data->max_brightness;
	rc = lm3530_write(client, BRIGHTNESS, composit_br >> 1);
	UNLOCK(data);
	return rc;
}

static int lm3530_als_get(struct i2c_client *client)
{
	u8 als_info;
	lm3530_read(client, ZONE_INF, &als_info);
	return als_info & 0x07;
}

static void lm3530_go_suspend(struct lm3530_data *data)
{
	LOCK(data);
	if (data->suspended)
		goto already_in_suspend;
	DBG(printk(KERN_DEBUG "%s: suspended\n", __func__);)
	data->suspended = 1;
	lm3530_hw_enable(data, 0);
already_in_suspend:
	UNLOCK(data);
}

static void lm3530_go_resume(struct lm3530_data *data)
{
	u8 zone;
	u8 shadowreg;

	LOCK(data);
	if (!data->suspended)
		goto already_resumed;
	data->suspended = 0;
	lm3530_hw_enable(data, 1);
	lm3530_restore_regs(data->client);
	/* get current zone and clear pending request */
	/* use shadowregister for faster fade up from sleepmode */
	shadowreg = data->shadow[ALS_CONFIG];
	lm3530_change_config(data->client, LM3530_CFG_ALS_AVG_TIME,
						 LM3530_ALS_AVG_TIME_0032);
	msleep(32);
	zone = lm3530_als_get(data->client);
	lm3530_write(data->client, ALS_CONFIG, shadowreg);
	if (zone != data->als_zone) {
		data->als_zone = zone;
		sysfs_notify(&data->client->dev.kobj, NULL, "als::value");
	}
	shadowreg = data->shadow[BR_RATE];
	lm3530_change_config(data->client, LM3530_CFG_BRR_UP, LM3530_BRR_01024);
	UNLOCK(data);
	lm3530_update_brightness(data->client);
	msleep(5);
	LOCK(data);
	lm3530_write(data->client, BR_RATE, shadowreg);
	UNLOCK(data);
	DBG(printk(KERN_DEBUG "%s: resumed\n", __func__);)
	return;

already_resumed:
	UNLOCK(data);
}

static void als_zone_int_bh(struct work_struct *work)
{
	struct lm3530_data *data =
			container_of(work, struct lm3530_data, als_work);

	DBG(printk(KERN_DEBUG "%s: %s\n", DRV_NAME, __func__);)

	LOCK(data);
	data->als_zone = lm3530_als_get(data->client);
	DBG(printk(KERN_DEBUG "lm3530 interrupt: als zone changed to %d\n",
			   data->als_zone);)
	UNLOCK(data);
	sysfs_notify(&data->client->dev.kobj, NULL, "als::value");
	lm3530_update_brightness(data->client);
	enable_irq(gpio_to_irq(data->zone_irq_gpio));
}

static irqreturn_t als_zone_int_callback(int vec, void *irq_data)
{
	struct lm3530_data *data = irq_data;

	DBG(printk(KERN_DEBUG "%s (%s)\n", DRV_NAME, __func__);)
	disable_irq(gpio_to_irq(data->zone_irq_gpio));
	schedule_work(&data->als_work);
	return IRQ_HANDLED;
}

static int lm3530_setup_gpio_pins(struct lm3530_data *data)
{
	int err = -EINVAL;

	if (data->hw_enable_gpio > 0) {
		err = gpio_request(data->hw_enable_gpio, "LM3530 hw enable");
		if (err) {
			printk(KERN_ERR
			       "Could not allocate LM3530_HW_ENABLE_PIN\n");
			goto func_exit;
		}
	}
	if (data->zone_irq_gpio > 0) {
		err = gpio_request(data->zone_irq_gpio, "LM3530 interrupt");
		if (err) {
			printk(KERN_ERR
			       "Could not allocate LM3530_INTERRUPT_PIN.\n");
			goto func_exit;
		}
		err = gpio_direction_input(data->zone_irq_gpio);
		if (err) {
			printk(KERN_ERR
			       "LM3530_INTERRUPT_PIN configuration error.\n");
			goto func_exit;
		}

		err = request_irq(gpio_to_irq(data->zone_irq_gpio),
					als_zone_int_callback,
					IRQF_TRIGGER_FALLING, "LM3530 interrupt",
					data);
		if (err) {
			printk(KERN_ERR "Failed requesting %d IRQ.\n",
				   gpio_to_irq(data->zone_irq_gpio));
		}
	}
func_exit:
	 return err;
}

static void lm3530_release_gpio_pins(struct lm3530_data *data)
{
	if (data->hw_enable_gpio > 0)
		gpio_free(data->hw_enable_gpio);
	if (data->zone_irq_gpio > 0) {
		free_irq(gpio_to_irq(data->zone_irq_gpio), data);
		gpio_free(data->zone_irq_gpio);
	}
}

static void lm3530_set_backlight(void *chip_data, u8 brightness)
{
	struct lm3530_data *data = (struct lm3530_data*)chip_data;
	if (!brightness)
		lm3530_go_suspend(data);
	else {
		int need_resume;
		LOCK(data);
		data->brightness = brightness;
		need_resume = data->suspended;
		UNLOCK(data);
		if (need_resume)
			lm3530_go_resume(data);
		else
			lm3530_update_brightness(data->client);
	}
}

static int lm3530_get_backlight(void *chip_data, u8 *brightness)
{
	struct lm3530_data *data = (struct lm3530_data*)chip_data;

	if (!data || !brightness)
		return -EINVAL;

	LOCK(data);
	*brightness = data->brightness;
	UNLOCK(data);
	return 0;
}

static ssize_t attr_brightness_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	char *p;
	int ret;
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	unsigned intensity = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret && intensity < 256) {
		LOCK(data);
		data->brightness = intensity;
		UNLOCK(data);
		lm3530_update_brightness(data->client);
		return ret;
	}
	return -EINVAL;
}

struct cfg_value_map {
	const char* name;
	int value;
};

static const struct cfg_value_map cfg_brr_table[] = {
	{"8", LM3530_BRR_00008},
	{"1024", LM3530_BRR_01024},
	{"2048", LM3530_BRR_02048},
	{"4096", LM3530_BRR_04096},
	{"8192", LM3530_BRR_08192},
	{"16384", LM3530_BRR_16384},
	{"32768", LM3530_BRR_32768},
	{"65538", LM3530_BRR_65538},
	{NULL, 0},
};

static const struct cfg_value_map cfg_br_mapping_table[] = {
	{"linear", LM3530_BRIGHTNESS_MAP_LINEAR},
	{"exp", LM3530_BRIGHTNESS_MAP_EXP},
	{NULL, 0},
};

static const struct cfg_value_map cfg_fsc_table[] = {
	{"5.0", LM3530_FSC_05_0},
	{"8.5", LM3530_FSC_08_5},
	{"12.0", LM3530_FSC_12_0},
	{"15.5", LM3530_FSC_15_5},
	{"19.0", LM3530_FSC_19_0},
	{"22.5", LM3530_FSC_22_5},
	{"26.0", LM3530_FSC_26_0},
	{"29.5", LM3530_FSC_29_5},
	{NULL, 0},
};

static const struct cfg_value_map cfg_als_inut_table[] = {
	{"avg", LM3530_ALS_INPUT_ALS_AVG},
	{"als1", LM3530_ALS_INPUT_ALS_1},
	{"als2", LM3530_ALS_INPUT_ALS_2},
	{NULL, 0},
};

static const struct cfg_value_map cfg_avg_time_table[] = {
	{"32", LM3530_ALS_AVG_TIME_0032},
	{"64", LM3530_ALS_AVG_TIME_0064},
	{"128", LM3530_ALS_AVG_TIME_0128},
	{"256", LM3530_ALS_AVG_TIME_0256},
	{"512", LM3530_ALS_AVG_TIME_0512},
	{"1024", LM3530_ALS_AVG_TIME_1024},
	{"2048", LM3530_ALS_AVG_TIME_2048},
	{"4096", LM3530_ALS_AVG_TIME_4096},
	{NULL, 0},
};

static const struct cfg_value_map cfg_als_resist_table[] = {
	{"high-z", LM3530_ALS_RES_HIGH_Z},
	{"9360", LM3530_ALS_RES_9_36},
	{"5560", LM3530_ALS_RES_5_56},
	{"2320", LM3530_ALS_RES_2_32},
	{"1980", LM3530_ALS_RES_1_98},
	{"1850", LM3530_ALS_RES_1_85},
	{"1630", LM3530_ALS_RES_1_63},
	{"1160", LM3530_ALS_RES_1_16},
	{"1070", LM3530_ALS_RES_1_07},
	{"1030", LM3530_ALS_RES_1_03},
	{"957.9", LM3530_ALS_RES_0_9579},
	{"771.7", LM3530_ALS_RES_0_7717},
	{"731.1", LM3530_ALS_RES_0_7311},
	{"712.3", LM3530_ALS_RES_0_7123},
	{"677.6", LM3530_ALS_RES_0_6776},
	{NULL, 0},
};

static const struct cfg_value_map cfg_chip_mode_table[] = {
	{"i2c", LM3530_MODE_I2C},
	{"pwm", LM3530_MODE_SIMPLE_PWM},
	{"i2c_pwm", LM3530_MODE_I2C_PWM},
	{"als", LM3530_MODE_SIMPLE_ALS},
	{"pwm_als", LM3530_MODE_PWM_ALS},
	{"i2c_pwm_als", LM3530_MODE_I2C_PWM_ALS},
	{"i2c_als", LM3530_MODE_I2C_ALS},
	{NULL, 0}
};

static int lookup_table(const struct cfg_value_map *t, const char* key)
{
	int len = strlen(key);

	if (!len)
		return -EINVAL;
	if (isspace(key[len-1]))
		--len;
	while (t->name) {
		if (!strncmp(t->name, key, len)) {
			DBG(printk(KERN_DEBUG "%s: %s -> %d\n",
					   __func__, t->name, t->value);)
			return t->value;
		}
		t++;
	}
	return -EINVAL;
}

static ssize_t attr_br_mapping(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int mapping = lookup_table(cfg_br_mapping_table, buf);
	if (mapping == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_BR_MAPPING, mapping);
	return strlen(buf);
}

static ssize_t attr_br_down_rate(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int br_rate = lookup_table(cfg_brr_table, buf);
	if (br_rate == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_BRR_DOWN, br_rate);
	return strlen(buf);
}

static ssize_t attr_br_up_rate(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int br_rate = lookup_table(cfg_brr_table, buf);
	if (br_rate == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_BRR_UP, br_rate);
	return strlen(buf);
}

static ssize_t attr_br_limit(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	size_t ret;
	char *p;
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	u8 limit = simple_strtoul(buf, &p, 10);
	ret = p - buf;
	if (ret) {
		LOCK(data);
		data->max_brightness = limit;
		UNLOCK(data);
		lm3530_update_brightness(data->client);
	}
	if (*p && isspace(*p))
		ret++;
	return ret == size ? ret : -EINVAL;
}

static ssize_t attr_fsc(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int fsc = lookup_table(cfg_fsc_table, buf);
	if (fsc == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_FSC, fsc);
	return strlen(buf);
}

static ssize_t attr_suspend(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	if (strlen(buf)) {
		if (*buf == '1')
			lm3530_go_suspend(data);
		else if (*buf == '0')
			lm3530_go_resume(data);
		else
			return -EINVAL;
	}
	return strlen(buf);
}

static ssize_t attr_curve_borders(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int argc;
	int argv[4];
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	argc = sscanf(buf,"%d,%d,%d,%d", &argv[0], &argv[1], &argv[2], &argv[3]);
	if (argc != 4)
		return -EINVAL;
	while (--argc >= 0)
		lm3530_config(data->client,
				LM3530_CFG_ZB_0 + argc, argv[argc] & 0xff);
	return strlen(buf);
}

static ssize_t attr_curve_targets(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int argc;
	int argv[5];
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	argc = sscanf(buf,"%d,%d,%d,%d,%d", &argv[0], &argv[1],
		      &argv[2], &argv[3], &argv[4]);
	if (argc != 5)
		return -EINVAL;
	while (--argc >= 0)
		lm3530_config(data->client, LM3530_CFG_ZT_0 + argc,
				argv[argc] & 0xff);
	lm3530_update_brightness(data->client);
	return strlen(buf);
}

static ssize_t attr_als_r1(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int r = lookup_table(cfg_als_resist_table, buf);
	if (r == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_ALS1_RESISTOR, r);
	return strlen(buf);
}

static ssize_t attr_als_r2(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int r = lookup_table(cfg_als_resist_table, buf);
	if (r == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_ALS2_RESISTOR, r);
	return strlen(buf);
}

static ssize_t attr_avg_time(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int time = lookup_table(cfg_avg_time_table, buf);
	if (time == -EINVAL)
		return -EINVAL;
	lm3530_config(data->client, LM3530_CFG_ALS_AVG_TIME, time);
	return strlen(buf);
}

static ssize_t attr_driver_mode(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lm3530_data *data =
			(struct lm3530_data*) dev_get_drvdata(dev);
	int mode = lookup_table(cfg_chip_mode_table, buf);
	if (mode == -EINVAL)
		return -EINVAL;
	lm3530_mode_set(data->client, mode);
	lm3530_update_brightness(data->client);
	return strlen(buf);
}

static ssize_t als_zone_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	u8 als_value;
	struct lm3530_data *data = (struct lm3530_data*)dev_get_drvdata(dev);

	LOCK(data);
	if (0 == data->als_zone)
		als_value = 0;
	else
		als_value = data->shadow[ZBOUNDARY0 + data->als_zone - 1];
	UNLOCK(data);
	return sprintf(buf, "%u\n", als_value);
}

static struct device_attribute attributes[] = {
	__ATTR(br::intensity, 0222, NULL, attr_brightness_store),
	__ATTR(br::mapping, 0200, NULL, attr_br_mapping),
	__ATTR(br::rate::up, 0200, NULL, attr_br_up_rate),
	__ATTR(br::rate::down, 0200, NULL, attr_br_down_rate),
	__ATTR(br::limit, 0200, NULL, attr_br_limit),
	__ATTR(br::fsc, 0200, NULL, attr_fsc),
	__ATTR(br::suspend, 0222, NULL, attr_suspend),
	__ATTR(curve::borders, 0200, NULL, attr_curve_borders),
	__ATTR(curve::targets, 0200, NULL, attr_curve_targets),
	__ATTR(als::r1, 0200, NULL, attr_als_r1),
	__ATTR(als::r2, 0200, NULL, attr_als_r2),
	__ATTR(als::avg-t, 0200, NULL, attr_avg_time),
	__ATTR(mode, 0200, NULL, attr_driver_mode),
	__ATTR(als::value, 0444, als_zone_show, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;
error:
	for (; i >=0; i--)
		device_remove_file(dev, attributes + i);
	printk(KERN_ERR "%s: %s Unable to create interface\n",
	       DRV_NAME, __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_LEDS_LM3530_EARLY_SUSPEND)
static void lm3530_early_suspend(struct early_suspend* h)
{
	struct lm3530_data *data =
			container_of(h, struct lm3530_data, suspend);
	DBG(printk(KERN_DEBUG "lm3530 Early suspend\n");)
	lm3530_go_suspend(data);
}

static void lm3530_late_resume(struct early_suspend* h)
{
	struct lm3530_data *data =
			container_of(h, struct lm3530_data, suspend);
	DBG(printk(KERN_DEBUG "lm3530 Late resume\n");)
	lm3530_go_resume(data);
}
#endif

static void lm3530_data_init(struct lm3530_data *data,
					   struct lm3530_platform_data *pdata)
{
	mutex_init(&data->update_lock);
	data->suspended = 0;
	data->max_brightness = 255;
	data->zone_irq_gpio = pdata->zone_irq_gpio;
	data->hw_enable_gpio = pdata->hw_enable_gpio;
	data->power_up = pdata->power_up;
	if (data->zone_irq_gpio > 0)
		INIT_WORK(&data->als_work, als_zone_int_bh);
}

static int __devinit lm3530_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct lm3530_platform_data *pdata = client->dev.platform_data;
	struct lm3530_data *data;
	int result = 0;
	int i;
	u8 value;
	enum lm3530_als_input als_input;

	printk(KERN_INFO "%s\n", __func__);

	/* Make sure we have at least i2c functionality on the bus */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		result = -EIO;
		goto err_check_functionality;
	}
	if (!pdata) {
		result = -EINVAL;
		printk(KERN_ERR "%s: platform data required.\n", __func__);
		goto err_platform_data;
	}
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		result = -ENOMEM;
		goto err_alloc_data_failed;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	lm3530_data_init(data, pdata);

	result = lm3530_setup_gpio_pins(data);
	if (result)
		goto err_gpio_failed;

	result = lm3530_hw_enable(data, 1);
	if (result)
		goto err_power_on;

	for (i = 0; i < NUM_OF_REGS; i++) {
		result = lm3530_read(client, i, &value);
		if (result)
			goto err_register_read;
	}

	result = create_sysfs_interfaces(&client->dev);
	if (result)
		goto err_create_interfaces_failed;

	lm3530_config(client, LM3530_CFG_PWM_POLARITY, pdata->pwm_polarity ?
			LM3530_PWM_POL_ACTIVE_HIGH : LM3530_PWM_POL_ACTIVE_LOW);

	if (pdata->als_input == ALS_INPUT_1)
		als_input = LM3530_ALS_INPUT_ALS_1;
	else if (pdata->als_input == ALS_INPUT_2)
		als_input = LM3530_ALS_INPUT_ALS_2;
	else
		als_input = LM3530_ALS_INPUT_ALS_AVG;

	lm3530_config(client, LM3530_CFG_BR_MAPPING,
		      LM3530_BRIGHTNESS_MAP_LINEAR);
	lm3530_config(client, LM3530_CFG_ALS_INPUT, als_input);
	lm3530_mode_set(client, LM3530_MODE_I2C);
	data->als_zone = lm3530_als_get(client);
	data->brightness = 102;
	lm3530_update_brightness(client);

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_LEDS_LM3530_EARLY_SUSPEND)
	data->suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	data->suspend.suspend = lm3530_early_suspend;
	data->suspend.resume = lm3530_late_resume;
	register_early_suspend(&data->suspend);
#endif

	pdata->set_backlight = lm3530_set_backlight;
	pdata->get_backlight = lm3530_get_backlight;
	pdata->chip_data = data;

	dev_info(&client->dev, "version %s loaded\n", DRIVER_VERSION);
	return 0;

err_create_interfaces_failed:
err_register_read:
	lm3530_hw_enable(data, 0);
err_power_on:
	lm3530_release_gpio_pins(data);
err_gpio_failed:
	kfree(data);
err_alloc_data_failed:
err_platform_data:
err_check_functionality:
	printk(KERN_ERR "%s: failed with code %d.\n", __func__, result);
	return result;
}

static int __devexit lm3530_remove(struct i2c_client *client)
{
	struct lm3530_data *data = i2c_get_clientdata(client);

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_LEDS_LM3530_EARLY_SUSPEND)
	unregister_early_suspend(&data->suspend);
#endif
	remove_sysfs_interfaces(&client->dev);
	lm3530_hw_enable(data, 0);
	lm3530_release_gpio_pins(data);
	kfree(data);
	return 0;
}

static const struct i2c_device_id lm3530_id[] = {
	{ "lm3530", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3530_id);

static struct i2c_driver lm3530_driver = {
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= lm3530_probe,
	.remove	= __devexit_p(lm3530_remove),
	.id_table = lm3530_id,
};

static int __init lm3530_init(void)
{
	int err = i2c_add_driver(&lm3530_driver);
	return err;
}

static void __exit lm3530_exit(void)
{
	i2c_del_driver(&lm3530_driver);
}

module_init(lm3530_init);
module_exit(lm3530_exit);

MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LM3530 led driver");

