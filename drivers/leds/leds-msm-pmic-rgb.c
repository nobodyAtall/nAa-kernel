/*
 * MSM PMIC RGB Led driver
 *
 *
 * Copyright (C) 2009 SonyEricsson AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 *
 */

#include <linux/module.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/workqueue.h>
#include <linux/semcclass.h>
#include <linux/miscdevice.h>
#include <mach/pmic.h>

#define DBG(msg) /* msg */
#define DRV_NAME "msm_pmic_rgb_led"
#define DEV_NAME "rgb_led"
#define DRIVER_VERSION  "0.2"

#define RED_LED_MPP_PIN 	PM_MPP_2
#define GREEN_LED_MPP_PIN 	PM_MPP_21
#define BLUE_LED_MPP_PIN 	PM_MPP_22

#define BRIGHTNESS_2_SINK(B) (enum mpp_i_sink_level)((B) >> 5)
#define DEF_BLINK_ON_TIME     500
#define DEF_BLINK_OFF_TIME    500
#define DEF_BLINK_MIN_PERIOD  100
#define DEF_MAX_BRIGHTNESS    (3<<5)

enum rgb_led_t {
	RGB_LED_RED,
	RGB_LED_GREEN,
	RGB_LED_BLUE,
	RGB_LEDS_NUMBER
};

enum rgb_pwm_t {
	PWM_ALWAYS_OFF = 0,
	PWM_FIFTY_FIFTY = 127,
	PWM_ALWAYS_ON = 255
};

enum rgb_led_state_t {
	STATE_LED_INITIAL 	= 0,
	STATE_LED_BLINKING = 0x01,
	STATE_LED_BLINK_ON  = 0x02,
	STATE_LED_ON = 0x04,
	STATE_BLINK_SCHEDULLED = 0x08,
};

struct pmic_led_data_t {
	enum mpp_which	mpp;
	enum mpp_i_sink_level level;
	enum mpp_i_sink_level last_sent_level;
	enum mpp_i_sink_switch last_sent_onoff;
	u8 brightness;
	enum mpp_i_sink_level max_level;
};

struct rgb_led_data_t {
	struct pmic_led_data_t led_data[RGB_LEDS_NUMBER];
	struct semc_classdev cdev;
	struct mutex 	update_lock;
	struct timer_list  tmr;
	u16	on_ms;
	u16 off_ms;
	u8 pwm;
	enum rgb_led_state_t state;
};

static int init_led_hw(void);
static void release_led_hw(void);
static void set_led_brightness(struct pmic_led_data_t *pled, u8 brightness);
static int set_led_off(struct pmic_led_data_t *pled);
static int set_led_on(struct pmic_led_data_t *pled);
static int turn_on_all_leds(void);
static int turn_off_all_leds(void);
static void led_blink(struct work_struct *work);
static struct pmic_led_data_t* led_by_name(const char* name);
static DECLARE_WORK(timer_work, led_blink);

static struct  rgb_led_data_t rgb_leds_data = {
		.led_data[RGB_LED_RED] 	= {
			.mpp = RED_LED_MPP_PIN,
			.last_sent_level = PM_MPP__I_SINK__LEVEL_40mA,
			.last_sent_onoff = PM_MPP__I_SINK__SWITCH_ENA,
			.max_level = BRIGHTNESS_2_SINK(DEF_MAX_BRIGHTNESS),
		},

		.led_data[RGB_LED_GREEN] = {
			.mpp = GREEN_LED_MPP_PIN,
			.last_sent_level = PM_MPP__I_SINK__LEVEL_40mA,
			.last_sent_onoff = PM_MPP__I_SINK__SWITCH_ENA,
			.max_level = BRIGHTNESS_2_SINK(DEF_MAX_BRIGHTNESS),
		},

		.led_data[RGB_LED_BLUE] = {
			.mpp = BLUE_LED_MPP_PIN,
			.last_sent_level = PM_MPP__I_SINK__LEVEL_40mA,
			.last_sent_onoff = PM_MPP__I_SINK__SWITCH_ENA,
			.max_level = BRIGHTNESS_2_SINK(DEF_MAX_BRIGHTNESS),
		},

		.cdev.name = DEV_NAME,
		.state = STATE_LED_BLINKING,
		.on_ms = DEF_BLINK_ON_TIME,
		.off_ms = DEF_BLINK_OFF_TIME,
		.pwm = PWM_FIFTY_FIFTY,
};

static int init_led_hw(void)
{
	set_led_brightness(&rgb_leds_data.led_data[RGB_LED_RED], 0);
	set_led_brightness(&rgb_leds_data.led_data[RGB_LED_GREEN], 0);
	set_led_brightness(&rgb_leds_data.led_data[RGB_LED_BLUE], 0);

	return turn_off_all_leds();
}

static void release_led_hw(void)
{
	(void)init_led_hw();
}

static void set_led_brightness(struct pmic_led_data_t *pled, u8 brightness)
{
	pled->brightness = brightness;
	pled->level = BRIGHTNESS_2_SINK(brightness);
	if ((rgb_leds_data.state  & (STATE_LED_ON | STATE_LED_BLINKING)) ==
			STATE_LED_ON)
		set_led_on(pled);
}

static int set_led_off(struct pmic_led_data_t *pled)
{
	if(pled->last_sent_onoff != PM_MPP__I_SINK__SWITCH_DIS ) {
		pled->last_sent_onoff = PM_MPP__I_SINK__SWITCH_DIS;
		return pmic_secure_mpp_config_i_sink(
				pled->mpp,
				PM_MPP__I_SINK__LEVEL_5mA,
				PM_MPP__I_SINK__SWITCH_DIS);
	}
	return 0;
}

static int set_led_on(struct pmic_led_data_t *pled)
{
	enum mpp_i_sink_switch onoff;
	enum mpp_i_sink_level  level;

	level = pled->level;
	if (level > pled->max_level)
		level = pled->max_level;

	if (pled->brightness)
		onoff = PM_MPP__I_SINK__SWITCH_ENA;
	else
		onoff = PM_MPP__I_SINK__SWITCH_DIS;

	if (pled->last_sent_onoff != onoff ||
			pled->last_sent_level != level) {

		pled->last_sent_onoff = onoff;
		pled->last_sent_level = level;

		DBG(printk(KERN_DEBUG "%s: %s send level %d to mpp %d\n",
				DRV_NAME, __func__,  level, pled->mpp);)

		return pmic_secure_mpp_config_i_sink(
				pled->mpp,
				level,
				onoff);
	}
	return 0;
}

static int turn_off_all_leds(void)
{
	int rc;
	rc = set_led_off(&rgb_leds_data.led_data[RGB_LED_RED]);
	rc += set_led_off(&rgb_leds_data.led_data[RGB_LED_GREEN]);
	rc += set_led_off(&rgb_leds_data.led_data[RGB_LED_BLUE]);
	return rc;
}

static int turn_on_all_leds(void)
{
	int rc;
	rc = set_led_on(&rgb_leds_data.led_data[RGB_LED_RED]);
	rc += set_led_on(&rgb_leds_data.led_data[RGB_LED_GREEN]);
	rc += set_led_on(&rgb_leds_data.led_data[RGB_LED_BLUE]);
	return rc;
}

static void state_changed(enum rgb_led_state_t state)
{
	if (state & STATE_LED_ON) {
		if (state & STATE_LED_BLINKING) {
			if (!(state & STATE_BLINK_SCHEDULLED)) {
				rgb_leds_data.state &= ~STATE_LED_BLINK_ON;
				schedule_work(&timer_work);
			}
		} else
			turn_on_all_leds();
	} else
		turn_off_all_leds();
	rgb_leds_data.state = (rgb_leds_data.state &
						   ~(STATE_LED_ON | STATE_LED_BLINKING)) | state;
}

static void led_blink(struct work_struct *work)
{
	int (*on_off)(void);
	u16 t;

	mutex_lock(&rgb_leds_data.update_lock);


	if ((rgb_leds_data.state & (STATE_LED_ON | STATE_LED_BLINKING)) ==
			(STATE_LED_ON | STATE_LED_BLINKING)) {

		rgb_leds_data.state ^= STATE_LED_BLINK_ON;

		if (rgb_leds_data.state & STATE_LED_BLINK_ON) {
			on_off = turn_on_all_leds;
			t = rgb_leds_data.on_ms;
		} else {
			on_off = turn_off_all_leds;
			t = rgb_leds_data.off_ms;
		}

		on_off();

		mod_timer(&rgb_leds_data.tmr, jiffies + msecs_to_jiffies(t));
		rgb_leds_data.state |= STATE_BLINK_SCHEDULLED;
	} else
		rgb_leds_data.state &= ~STATE_BLINK_SCHEDULLED;

	mutex_unlock(&rgb_leds_data.update_lock);

	return;
}

static void timer_callback(unsigned long d)
{
  schedule_work(&timer_work);
}

static struct pmic_led_data_t* led_by_name(const char* name)
{
	enum rgb_led_t led;

	if (*name == 'r')
		led = RGB_LED_RED;
	else if (*name == 'g')
		led = RGB_LED_GREEN;
	else
		led = RGB_LED_BLUE;

	return &rgb_leds_data.led_data[led];
}

static ssize_t store_brightness(struct device *dev,
								struct device_attribute *attr,
								const char *buf, size_t size)
{
	u32 value;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if(-EINVAL != ret) {
		mutex_lock(&rgb_leds_data.update_lock);
		set_led_brightness(led_by_name(attr->attr.name), value & 0xff);
		mutex_unlock(&rgb_leds_data.update_lock);
		DBG(printk(KERN_DEBUG "%s: %s set to %d\n",
				DRV_NAME, attr->attr.name,  value & 0xff);)
	}
	return ret;
}

static ssize_t show_brightness(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", led_by_name(attr->attr.name)->brightness);
}

static ssize_t store_max_brightness(struct device *dev,
									struct device_attribute *attr,
									const char *buf, size_t size)
{
	u32 value;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if(-EINVAL != ret) {
		struct pmic_led_data_t *pled;

		mutex_lock(&rgb_leds_data.update_lock);

		pled = led_by_name(attr->attr.name);
		pled->max_level = BRIGHTNESS_2_SINK(value & 0xff);

		mutex_unlock(&rgb_leds_data.update_lock);
		DBG(printk(KERN_DEBUG "%s: %s set to %d\n",
				DRV_NAME, attr->attr.name,  value & 0xff);)
	}
	return ret;
}

static ssize_t show_blinking(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (rgb_leds_data.state &
								 (STATE_LED_BLINKING | STATE_LED_ON)) != 0);
}

static ssize_t store_bfp(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t size)
{
	u32 value;
	enum rgb_led_state_t state;
	ssize_t ret = semc_classdev_read_interface(buf, size, &value, 10);

	if (-EINVAL == ret)
		goto exit_failed;
	DBG(printk(KERN_DEBUG "%s: %s set value to %d\n",
				DRV_NAME, attr->attr.name,  value & 0xff);)
	mutex_lock(&rgb_leds_data.update_lock);

	state = rgb_leds_data.state  & (STATE_LED_ON | STATE_LED_BLINKING);

	if (*attr->attr.name == 'b') {	/* this is blink */
		if (value && (rgb_leds_data.pwm != PWM_ALWAYS_OFF))
			state |= STATE_LED_ON;
		else
			state &= ~STATE_LED_ON;
	} else 	{

		u16 total_ms;
		u16 on_ms;
		u16 pwm;

		if (*attr->attr.name == 'f') {
			/*
			 * This is frequency
			 * The LED appears to blink about once per second if freq is 20
			 */
			 total_ms = (1000 * 20 ) / value;
			 pwm = rgb_leds_data.pwm;

		} else if (*attr->attr.name == 'p') {
			/* this pwm */
			value &= 0xff;
			rgb_leds_data.pwm = pwm = value;

			if (value == PWM_ALWAYS_OFF) {
				state &= ~(STATE_LED_ON | STATE_LED_BLINKING);
				goto exit;
			}
			if (value == PWM_ALWAYS_ON) {
				state &= ~(STATE_LED_BLINKING);
				goto exit;
			}

			state |= STATE_LED_BLINKING;
			total_ms = rgb_leds_data.on_ms + rgb_leds_data.off_ms;
		} else
			goto exit;

		if (DEF_BLINK_MIN_PERIOD > total_ms)
			total_ms = DEF_BLINK_MIN_PERIOD;

		on_ms = (total_ms * pwm) >> 8;
		if (on_ms < DEF_BLINK_MIN_PERIOD/2)
			on_ms = DEF_BLINK_MIN_PERIOD/2;

		rgb_leds_data.off_ms = total_ms - on_ms;
		rgb_leds_data.on_ms  = on_ms;
	}

exit:
	if (state != (rgb_leds_data.state & (STATE_LED_ON | STATE_LED_BLINKING))) {
		state_changed(state);
	}

	mutex_unlock(&rgb_leds_data.update_lock);

exit_failed:
	return ret;
}

static struct device_attribute rgb_attributes[] = {
	__ATTR(red:brightness,  0644, show_brightness,  store_brightness),
	__ATTR(green:brightness,  0644, show_brightness,  store_brightness),
	__ATTR(blue:brightness,  0644, show_brightness,  store_brightness),
	__ATTR(blink,  0644, show_blinking,  store_bfp),
	__ATTR(frequency,  0200, NULL,  store_bfp),
	__ATTR(pwm,  0200, NULL,  store_bfp),
	__ATTR(red:max_brightness,  0200, NULL,  store_max_brightness),
	__ATTR(green:max_brightness,  0200, NULL,  store_max_brightness),
	__ATTR(blue:max_brightness,  0200, NULL,  store_max_brightness),
};

static long msm_pmic_rgb_panic_blink(long time)
{
	static bool initialized = false;
	mutex_lock(&rgb_leds_data.update_lock);
	if (initialized)
		goto exit;
	initialized = true;
	set_led_brightness(led_by_name("red:brightness"), 255 & 0xff);
	set_led_brightness(led_by_name("green:brightness"), 255 & 0xff);
	set_led_brightness(led_by_name("blue:brightness"), 255 & 0xff);
	turn_on_all_leds();
exit: mutex_unlock(&rgb_leds_data.update_lock);
	return 0;
}

static void init_led_data(void)
{
	mutex_init(&rgb_leds_data.update_lock);
	setup_timer(&rgb_leds_data.tmr, timer_callback, 0);
}

static void release_led_data(void)
{
	mutex_init(&rgb_leds_data.update_lock);
	del_timer_sync(&rgb_leds_data.tmr);
}

static const struct file_operations rgb_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice rgb_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_NAME,
	.fops = &rgb_fops,
};

static int __init rgb_init(void)
{
	int res=0;

	printk(KERN_INFO "%s Inserting\n", DRV_NAME);

	init_led_data();

	if(init_led_hw())
		goto exit_hw_failed;

	res = misc_register(&rgb_device);
	if (res) {
		printk(KERN_ERR "%s: Couldn't misc_register, error=%d\n",
			   DRV_NAME, res);
		goto exit_misc_err;
	}

	/* Create  interfaces  */
	rgb_leds_data.cdev.name = DEV_NAME;
	rgb_leds_data.cdev.attributes = rgb_attributes;
	rgb_leds_data.cdev.attr_number = sizeof(rgb_attributes) /
			sizeof(struct device_attribute);

	res = semc_classdev_register(rgb_device.this_device, &rgb_leds_data.cdev);

	if (res < 0) {
		printk(KERN_ERR "%s: Failed to register rgb led interface\n", DRV_NAME);
		goto exit;
	}
	panic_blink = msm_pmic_rgb_panic_blink;

	printk(KERN_INFO "%s version %s loaded\n", DRV_NAME, DRIVER_VERSION);
	return 0;

exit:
	misc_deregister(&rgb_device);
exit_misc_err:
	release_led_hw();
exit_hw_failed:
	release_led_data();
	printk(KERN_ERR "%s version %s installation failed\n",
		   DRV_NAME, DRIVER_VERSION);
	return res;
}

static void __exit rgb_exit(void)
{
	semc_classdev_unregister(&rgb_leds_data.cdev);
	misc_deregister(&rgb_device);
	release_led_hw();
	release_led_data();
	printk(KERN_INFO "%s version %s removed\n", DRV_NAME, DRIVER_VERSION);
}

module_init(rgb_init);
module_exit(rgb_exit);

MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("msm pmic rgb led driver");

