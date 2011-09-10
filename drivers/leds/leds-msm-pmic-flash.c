/*
 * MSM PMIC Flash Led driver
 *
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 *
 */
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <mach/pmic.h>
#include <linux/platform_device.h>
#include <linux/leds-msm_pmic_flashled.h>

#define DRV_NAME "msm_pmic_flash_led"
#define DRIVER_VERSION  "0.1"

#define LOCK() mutex_lock(&flashled.update_lock)
#define UNLOCK() mutex_unlock(&flashled.update_lock)

struct pmic_flash_led_data {
	struct mutex update_lock;
	struct timer_list  tmr;
	u16  spotlight_current;
	u16  flashled_current;
	u16  flashled_vboost_mv;
	u16  spotlight_vboost_mv;
	struct platform_device *pdev;
	struct msm_pmic_flashled_platform_data *pltf;
	int spotlight_enabled:1;
	int camflash_enabled:1;
};

static struct pmic_flash_led_data flashled;

static int init_led_hw(void)
{
	int rc = 0;
	char const *err_str;

	if (flashled.pltf->sync_enable) {
		rc = pmic_secure_mpp_config_digital_input(
					flashled.pltf->sync_mpp,
					flashled.pltf->sync_level,
					flashled.pltf->sync_in_dbus);
		if(rc) {
			err_str = "pmic_secure_mpp_config_digital_input";
			goto error;
		}

		rc =  pmic_flash_led_set_polarity(flashled.pltf->sync_polarity);
		if(rc) {
			err_str = "pmic_flash_led_set_polarity";
			goto error;
		}
	}

	rc = pmic_flash_led_set_current(0);
	if(rc) {
		err_str = "ic_flash_led_set_current";
		goto error;
	}

	rc = pmic_flash_led_set_mode(FLASH_LED_MODE__MANUAL);
	if(rc) {
		err_str = "pmic_flash_led_set_mode";
		goto error;
	}

error:
	if(rc)
		printk(KERN_ERR "%s: HW initialization failed (%s returned %d).\n",
			DRV_NAME, err_str,  rc);
	return rc;
}

static int set_spotlight(int enable)
{
	if (flashled.camflash_enabled) {
		del_timer_sync(&flashled.tmr);
		pmic_flash_led_set_mode(FLASH_LED_MODE__MANUAL);
	}

	LOCK();
	flashled.camflash_enabled = 0;
	flashled.spotlight_enabled = !!enable;
	UNLOCK();

	if (enable) {
		flashled.pltf->set_boost_voltage(flashled.pdev,
					flashled.spotlight_vboost_mv);
		flashled.pltf->enable_boost_voltage(flashled.pdev, 1);
		return pmic_flash_led_set_current(flashled.spotlight_current);
	}

	flashled.pltf->enable_boost_voltage(flashled.pdev, 0);
	return  pmic_flash_led_set_current(0);
}

static int connect_flashled_to_bus(u16 milliseconds)
{
	int rc;

	flashled.pltf->set_boost_voltage(flashled.pdev,
				flashled.flashled_vboost_mv);
	if (flashled.pltf->sync_enable) {
		rc = pmic_flash_led_set_mode(flashled.pltf->flash_ctl_dbus);
		if (rc)
			return rc;
	}
	LOCK();
	flashled.spotlight_enabled = 0;
	flashled.camflash_enabled = 1;
	UNLOCK();
	rc = pmic_flash_led_set_current(flashled.flashled_current);
	if (rc)
		return rc;
	flashled.pltf->enable_boost_voltage(flashled.pdev, 1);
	mod_timer(&flashled.tmr, jiffies + msecs_to_jiffies(milliseconds));
	return 0;
}

static void timer_bh(struct work_struct *work)
{
	pmic_flash_led_set_current(0);
	flashled.pltf->enable_boost_voltage(flashled.pdev, 0);
	flashled.camflash_enabled = 0;
	pmic_flash_led_set_mode(FLASH_LED_MODE__MANUAL);
}
static DECLARE_WORK(timer_work, timer_bh);

static void timer_callback(unsigned long d)
{
	schedule_work(&timer_work);
}

static ssize_t spotlight_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 enable = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret)
		set_spotlight(!!enable);
	if (*p && isspace(*p))
		ret++;
	return ret;
}

static ssize_t spotlight_get_enable(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;

	LOCK();
	ret = sprintf(buf, "%u\n", flashled.spotlight_enabled);
	UNLOCK();
	return ret;
}

static ssize_t spotlight_set_current(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 spotlight_current = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret) {
		LOCK();
		flashled.spotlight_current = spotlight_current;
		UNLOCK();
	}
	if (*p && isspace(*p))
		ret++;
	return ret;
}

static ssize_t flashled_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 milliseconds = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret) {
		if (milliseconds > flashled.pltf->enable_time_limit)
			milliseconds = flashled.pltf->enable_time_limit;
		connect_flashled_to_bus(milliseconds);
	}
	if (*p && isspace(*p))
		ret++;
	return ret;
}

static ssize_t flashled_set_current(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 flashled_current = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret) {
		LOCK();
		flashled.flashled_current = flashled_current;
		UNLOCK();
	}
	if (*p && isspace(*p))
		ret++;
	return ret;
}

static ssize_t flashled_set_boost(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 flashled_vboost_mv = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret) {
		LOCK();
		flashled.flashled_vboost_mv = flashled_vboost_mv;
		UNLOCK();
	}
	if (*p && isspace(*p))
		ret++;
	return ret;
}


static ssize_t spotlight_set_boost(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 spotlight_vboost_mv = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret) {
		LOCK();
		flashled.spotlight_vboost_mv = spotlight_vboost_mv;
		UNLOCK();
	}
	if (*p && isspace(*p))
		ret++;
	return ret;
}

static ssize_t dbg_boost_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	size_t ret;
	char *p;
	u32 enable = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (ret) {
		if (enable)
			flashled.pltf->set_boost_voltage
				(flashled.pdev, flashled.flashled_vboost_mv);
		flashled.pltf->enable_boost_voltage(flashled.pdev, !!enable);
	}
	if (*p && isspace(*p))
		ret++;
	return ret;
}

static struct device_attribute attributes[] = {
	__ATTR(cmaflash::enable, 0222, NULL, flashled_set_enable),
	__ATTR(cmaflash::boost_mv, 0200, NULL, flashled_set_boost),
	__ATTR(cmaflash::current_ma, 0200, NULL, flashled_set_current),
	__ATTR(spotlight::enable, 0666, spotlight_get_enable, spotlight_set_enable),
	__ATTR(spotlight::boost_mv, 0200, NULL, spotlight_set_boost),
	__ATTR(spotlight::current_ma, 0200, NULL, spotlight_set_current),
	__ATTR(debug::enable_boost, 0200, NULL, dbg_boost_enable),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < sizeof(attributes) / sizeof(struct device_attribute); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;
error:
	for (; i >=0; i--)
		device_remove_file(dev, attributes + i);
	printk(KERN_ERR "%s: %s Unable to create interface\n", DRV_NAME, __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < sizeof(attributes) / sizeof(struct device_attribute); i++)
		device_remove_file(dev, attributes + i);
}

static void init_led_data(struct platform_device *pdev)
{
	struct msm_pmic_flashled_platform_data *pltf = pdev->dev.platform_data;

	mutex_init(&flashled.update_lock);
	setup_timer(&flashled.tmr, timer_callback, 0);
	flashled.pdev = pdev;
	flashled.pltf = pltf;
	flashled.spotlight_current = pltf->def_spotlight_curtent;
	flashled.flashled_current = pltf->def_camflash_current;
	flashled.flashled_vboost_mv = pltf->def_camflash_boost_v;
	flashled.spotlight_vboost_mv = pltf->def_spotlight_boost_v;
	flashled.spotlight_enabled = 0;
	flashled.camflash_enabled = 0;
}

static int msm_pmic_flash_probe(struct platform_device *pdev)
{
	int rc;

	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s: platform data required\n", DRV_NAME);
		return -ENODEV;
	}

	printk(KERN_INFO "%s: probing\n", DRV_NAME);
	init_led_data(pdev);
	rc = init_led_hw();
	if (rc) {
		printk(KERN_ERR "%s: HW init error\n", DRV_NAME);
		return -ENODEV;
	}
	rc = create_sysfs_interfaces(&pdev->dev);
	if (rc) {
		printk(KERN_ERR "%s: Unable to create sysfs attributes\n", DRV_NAME);
		return -ENODEV;
	}

	return rc;
}

static int __devexit msm_pmic_flash_remove(struct platform_device *pdev)
{
	remove_sysfs_interfaces(&pdev->dev);
	return 0;
}

static struct platform_driver msm_pmic_flashled_driver = {
	.probe		= msm_pmic_flash_probe,
	.remove		= __devexit_p(msm_pmic_flash_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_flashled_init(void)
{
	return platform_driver_register(&msm_pmic_flashled_driver);
}
module_init(msm_pmic_flashled_init);

static void __exit msm_pmic_flashled_exit(void)
{
	platform_driver_unregister(&msm_pmic_flashled_driver);
}
module_exit(msm_pmic_flashled_exit)

MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("msm pmic flash led driver");
