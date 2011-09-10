/* drivers/gpio/semc_gpio_extr.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Alexandar Rodzevski <alexandar.rodzevski@sonyericsson.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <asm/io.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/semc/semc_gpio_extr.h>

#define DRV_NAME		SEMC_GPIO_EXTR_DRV_NAME
#define MAX_GPIO_NBR		300
#define MAX_CHARS_PER_GPIO	5
#define STRING_NBR_BASE		10
#define DBG(x)

static DEFINE_MUTEX(update_lock);

static ssize_t get_gpio_list(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, j;
	struct semc_gpio_extr_platform_data *pdata = dev->platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	for (i = j = 0; i < pdata->list_size; i++)
		j += snprintf(&buf[j], MAX_CHARS_PER_GPIO,
			"%u\n", pdata->gpio_list[i]);
	return (ssize_t)j;
}

static ssize_t get_select_gpio(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct semc_gpio_extr_platform_data *pdata = dev->platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	return snprintf(buf, MAX_CHARS_PER_GPIO, "%d\n", pdata->gpio_select);
}

static ssize_t set_select_gpio(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 gpio;
	char *p;
	int i;
	struct semc_gpio_extr_platform_data *pdata = dev->platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	gpio = simple_strtoul(buf, &p, STRING_NBR_BASE);
	if(!(p - buf) || (gpio > MAX_GPIO_NBR))
		return -EINVAL;

	DBG(printk(KERN_INFO "%s: %s: gpio:%d\n", DRV_NAME, __func__, gpio);)
	for (i = 0; i < pdata->list_size; i++)
		if ((u8)gpio == pdata->gpio_list[i])
			goto found;
	printk(KERN_ERR "%s: %s: GPIO:%d not extracted, check board file\n",
		DRV_NAME, __func__, gpio);
	return -1;

found:	mutex_lock(&update_lock);
	pdata->gpio_select = gpio;
	mutex_unlock(&update_lock);
	return (p - buf);
}

static ssize_t get_gpio(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct semc_gpio_extr_platform_data *pdata = dev->platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	return snprintf(buf, MAX_CHARS_PER_GPIO, "%i\n",
		pdata->get_gpio(pdata->gpio_select));
}

static ssize_t set_gpio(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 gpio_val;
	char *p;
	struct semc_gpio_extr_platform_data *pdata = dev->platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	if (pdata->gpio_select == SEMC_GPIO_EXTR_NO_SELECT) {
		printk(KERN_ERR "%s: %s: No GPIO is selected\n",
			DRV_NAME, __func__);
		return -1;
	}

	gpio_val = simple_strtoul(buf, &p, STRING_NBR_BASE);
	if(!(p - buf) || (gpio_val > 1))
		return -EINVAL;

	if(pdata->set_gpio(pdata->gpio_select, gpio_val))
		return -EINVAL;
	DBG(printk(KERN_INFO "%s: %s: val:%d\n",DRV_NAME, __func__, gpio_val);)
	return (p - buf);
}

static ssize_t init_gpios(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct semc_gpio_extr_platform_data *pdata = dev->platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	if (pdata->init_gpios()) {
		strcpy(buf, "GPIO(s) not re-initialized\n");
		return strlen("GPIO(s) not re-initialized\n");
	}
	pdata->gpio_select = SEMC_GPIO_EXTR_NO_SELECT;
	strcpy(buf, "GPIO(s) re-initialized\n");
	return strlen("GPIO(s) re-initialized\n");
}

static struct device_attribute semc_gpio_extr_attributes[] = {
	__ATTR(gpio_list, 0600, get_gpio_list,  NULL),
	__ATTR(gpio_select, 0600, get_select_gpio,  set_select_gpio),
	__ATTR(gpio_get, 0600, get_gpio,  NULL),
	__ATTR(gpio_set, 0600, NULL,  set_gpio),
	__ATTR(gpios_init, 0600, init_gpios,  NULL),
};

static int create_sysfs_interfaces(struct platform_device *pdev)
{
	int i = 0;
	int itterations = sizeof(semc_gpio_extr_attributes) /
				sizeof(struct device_attribute);

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	for (; i < itterations; i++)
		if (device_create_file(&pdev->dev,
			semc_gpio_extr_attributes + i))
			goto error;
	return 0;

error:	for (; i >= 0; i--)
		device_remove_file(&pdev->dev, semc_gpio_extr_attributes + i);
	printk(KERN_ERR "%s: %s Unable to create interface\n",
		DRV_NAME, __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct platform_device *pdev)
{
	int i = 0;
	int itterations = sizeof(semc_gpio_extr_attributes) /
				sizeof(struct device_attribute);

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	for (; i < itterations; i++)
		device_remove_file(&pdev->dev, semc_gpio_extr_attributes + i);
}

static int probe(struct platform_device *pdev)
{
	struct semc_gpio_extr_platform_data *pdata =
		(struct semc_gpio_extr_platform_data *)pdev->dev.platform_data;

	DBG(printk(KERN_INFO "%s: %s\n", DRV_NAME, __func__);)
	if (!pdata->gpio_list || !pdata->list_size) {
		printk(KERN_ERR "%s: %s: Bad GPIO-list\n", DRV_NAME, __func__);
		return -ENODEV;
	}
	if (!pdata->init_gpios || pdata->init_gpios()) {
		printk(KERN_ERR "%s: %s: Bad init\n", DRV_NAME, __func__);
		return -ENODEV;
	}
	if (!pdata->get_gpio || !pdata->set_gpio) {
		printk(KERN_ERR "%s: %s: Bad CB(s)\n", DRV_NAME, __func__);
		return -ENODEV;
	}
	if(create_sysfs_interfaces(pdev)) {
		printk(KERN_ERR "%s: %s: Bad SysFS IF\n", DRV_NAME, __func__);
		return -ENODEV;
	}
	pdata->gpio_select = SEMC_GPIO_EXTR_NO_SELECT;
	return 0;
}

static int __devexit semc_gpio_extr_remove(struct platform_device *pdev)
{
	remove_sysfs_interfaces(pdev);
	printk(KERN_INFO "%s: %s:\n", DRV_NAME, __func__);
	return 0;
}


static struct platform_driver semc_gpio_extr_driver = {
	.probe = probe,
	.remove = __devexit_p(semc_gpio_extr_remove),
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init semc_gpio_extr_init(void)
{
	int rc = platform_driver_register(&semc_gpio_extr_driver);
	DBG(printk(KERN_INFO "%s: %s rc=%d\n",
		DRV_NAME, __func__, rc);)
	return rc;
}

static void __exit semc_gpio_extr_exit(void)
{
	platform_driver_unregister(&semc_gpio_extr_driver);
}

module_init(semc_gpio_extr_init);
module_exit(semc_gpio_extr_exit);

MODULE_AUTHOR("Alexandar Rodzevski (alexandar.rodzevski@sonyericsson.com)");
MODULE_DESCRIPTION("semc gpio extractor");
MODULE_LICENSE("GPL");

