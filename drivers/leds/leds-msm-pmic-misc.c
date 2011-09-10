/*
 * MSM PMIC LED driver
 *
 * Copyright (C) 2010 SonyEricsson Mobile Communications AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 *
 */
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/leds-msm_pmic_misc.h>

#define DBG(X) /*X*/

#define DRV_NAME MSM_PMIC_MISC_DRV_NAME
#define DRIVER_VERSION  "0.3"

#define CURVE_ZONES_NUMBER 5

#define LOCK(p) do { \
	DBG(printk("%s: lock\n", __func__);) \
	mutex_lock(&p->lock); \
} while (0);

#define UNLOCK(p) do { \
	DBG(printk("%s: unlock\n", __func__);) \
	mutex_unlock(&p->lock); \
} while (0);


struct led_data_t {
	u8 user_brightness;
	u8 led_brightness;
	u8 max_brightness;
	u8 als_cut_off_value;
	u8 als_value;
	int suspended:1;
	int ctl_user:1;
	int ctl_als:1;
	struct msm_pmic_misc_led_platform_data *platform;
	struct early_suspend suspend;
	struct mutex lock;
};

static void update_led_brightness(struct led_data_t* led)
{
	u8 br;

	if (led->suspended)
		return;
	DBG(printk(KERN_DEBUG
			   "%s::%s (%s)"
			   " user %d, max %d, led %d, als %d, als-cut %d,"
			   " user_ctl %d, als_ctl %d\n",
				DRV_NAME, led->platform->name, __func__,
				led->user_brightness,
				led->max_brightness,
				led->led_brightness,
				led->als_value,
				led->als_cut_off_value,
				!!led->ctl_user,
				!!led->ctl_als);)

	br = (led->ctl_user) ? led->user_brightness : led->max_brightness;
	if (led->ctl_als && (led->als_value > led->als_cut_off_value))
		br = 0;
	else if (br > led->max_brightness)
		br = led->max_brightness;

	if (br != led->led_brightness) {
		led->led_brightness = br;
		led->platform->set_brightness(led->platform, br);
	}
}

static void set_user_brightness(struct led_data_t* led, u8 brightness)
{
	DBG(printk(KERN_DEBUG "%s::%s (%s) brightness %d\n",
		   DRV_NAME, led->platform->name, __func__, brightness);)
	LOCK(led);
	if (brightness == led->user_brightness)
		goto func_exit;
	led->user_brightness = brightness;
	update_led_brightness(led);
func_exit:
	UNLOCK(led);
}

static void misc_leds_early_suspend(struct early_suspend *h)
{
	struct led_data_t* led = container_of(h, struct led_data_t, suspend);

	DBG(printk(KERN_DEBUG "%s::%s (%s)\n",
				   DRV_NAME, led->platform->name, __func__);)
	LOCK(led);
	if (led->suspended)
		goto func_exit;
	led->suspended = 1;
	led->led_brightness = 0;
	led->platform->set_brightness(led->platform, 0);
func_exit:
	UNLOCK(led);
}

static void misc_leds_late_reusme(struct early_suspend *h)
{
	struct led_data_t* led = container_of(h, struct led_data_t, suspend);

	DBG(printk(KERN_DEBUG "%s::%s (%s)\n",
				   DRV_NAME, led->platform->name, __func__);)
	LOCK(led);
	if (!led->suspended)
		goto func_exit;
	led->suspended = 0;
	update_led_brightness(led);
func_exit:
	UNLOCK(led);
}

static ssize_t attr_brightness(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	size_t ret;
	char *p;
	struct led_data_t *led = dev_get_drvdata(dev);
	u8 br = simple_strtoul(buf, &p, 10);
	ret = p - buf;
	if (ret)
		set_user_brightness(led, br);
	if (*p && isspace(*p))
		ret++;
	return ret == size ? ret : -EINVAL;
}

static ssize_t attr_brightness_limit(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	size_t ret;
	char *p;
	struct led_data_t *led = dev_get_drvdata(dev);
	u8 limit = simple_strtoul(buf, &p, 10);
	ret = p - buf;
	if (ret) {
		LOCK(led);
		led->max_brightness = limit;
		update_led_brightness(led);
		UNLOCK(led);
	}
	if (*p && isspace(*p))
		ret++;
	return ret == size ? ret : -EINVAL;
}

static ssize_t attr_current_limit(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	size_t ret;
	char *p;
	struct led_data_t *led = dev_get_drvdata(dev);
	int current_ma = simple_strtoul(buf, &p, 10);
	ret = p - buf;
	if (ret) {
		LOCK(led);
		led->max_brightness =
			led->platform->currnet_to_brightness(current_ma);
		update_led_brightness(led);
		UNLOCK(led);
	}
	if (*p && isspace(*p))
		ret++;
	return ret == size ? ret : -EINVAL;
}

static ssize_t attr_als_cut_off(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	size_t ret;
	char *p;
	struct led_data_t *led = dev_get_drvdata(dev);
	int als_cut_off = simple_strtoul(buf, &p, 10);
	ret = p - buf;
	if (ret) {
		LOCK(led);
		led->als_cut_off_value = als_cut_off;
		update_led_brightness(led);
		UNLOCK(led);
	}
	if (*p && isspace(*p))
		ret++;
	return ret == size ? ret : -EINVAL;
}

static ssize_t attr_als_changed(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	size_t ret;
	char *p;
	struct led_data_t *led = dev_get_drvdata(dev);
	u8 als = simple_strtoul(buf, &p, 10);
	ret = p - buf;
	if (ret) {
		LOCK(led);
		led->als_value = als;
		update_led_brightness(led);
		UNLOCK(led);
	}
	if (*p && isspace(*p))
		ret++;
	return ret == size ? ret : -EINVAL;
}

static ssize_t attr_control_mode(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	int ctl_user = 0;
	int ctl_als = 0;
	struct led_data_t *led = dev_get_drvdata(dev);

	if (!strncmp(buf, "user_als", sizeof("user_als") - 1))
		ctl_user = ctl_als = 1;
	else if (!strncmp(buf, "als", sizeof("als") - 1))
		ctl_als = 1;
	else if (!strncmp(buf, "user", sizeof("user") - 1))
		ctl_user = 1;
	else
		return -EINVAL;
	LOCK(led);
	led->ctl_user = ctl_user;
	led->ctl_als = ctl_als;
	update_led_brightness(led);
	UNLOCK(led);
	return strlen(buf);
}

static struct device_attribute attributes[] = {
	__ATTR(brightness, 0222, NULL,  attr_brightness),
	__ATTR(max::brightness,  0200, NULL,  attr_brightness_limit),
	__ATTR(max::current_ma,  0200, NULL,  attr_current_limit),
	__ATTR(als::cut-off,  0200, NULL,  attr_als_cut_off),
	__ATTR(control::mode,  0200, NULL,  attr_control_mode),
	__ATTR(als::value,  0222, NULL,  attr_als_changed),
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

static int msm_pmic_misc_led_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_pmic_misc_led_platform_data *pdata = pdev->dev.platform_data;
	struct led_data_t *led;

	if (!pdata) {
		printk(KERN_ERR "%s: platform data required\n", DRV_NAME);
		return -ENODEV;
	}

	printk(KERN_INFO "%s::%s probing\n", DRV_NAME, pdata->name);

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		printk(KERN_ERR "%s::%s not enough memory\n",
		       DRV_NAME, pdata->name);
		return  -ENOMEM;
	}

	memset(led, 0, sizeof(*led));
	led->platform = pdata;
	led->max_brightness = 255;
	led->ctl_user = 1;
	mutex_init(&led->lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
	led->suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	led->suspend.suspend = misc_leds_early_suspend;
	led->suspend.resume = misc_leds_late_reusme;
	register_early_suspend(&led->suspend);
#endif

	platform_set_drvdata(pdev, led);
	rc = create_sysfs_interfaces(&pdev->dev);
	if (rc)
		goto err_create_interfaces_failed;

	return rc;

err_create_interfaces_failed:
	kfree(led);
	return rc;
}

static int __devexit msm_pmic_misc_led_remove(struct platform_device *pdev)
{
	struct led_data_t *led = platform_get_drvdata(pdev);
	remove_sysfs_interfaces(&pdev->dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&led->suspend);
#endif
	kfree(led);
	return 0;
}

static struct platform_driver msm_pmic_misc_led_driver = {
	.probe		= msm_pmic_misc_led_probe,
	.remove		= __devexit_p(msm_pmic_misc_led_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_misc_led_init(void)
{
	return platform_driver_register(&msm_pmic_misc_led_driver);
}
module_init(msm_pmic_misc_led_init);

static void __exit msm_pmic_misc_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_misc_led_driver);
}
module_exit(msm_pmic_misc_led_exit);


MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("platform: msm pmic misc leds driver");

