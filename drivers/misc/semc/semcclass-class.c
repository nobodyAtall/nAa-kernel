/*
 * SEMC Class Core
 *
 * Copyright (C) 2009 SonyEricsson AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/semcclass.h>
#include "semcclass.h"

static struct class *semc_class;

/**
 * led_classdev_register - register a new object of led_classdev class.
 * @dev: The device to register.
 * @semc_cdev: the led_classdev structure for this device.
 */
int semc_classdev_register(struct device *parent,
						   struct semc_classdev *semc_cdev)
{
	int rc;
	int i;

	semc_cdev->dev = device_create(semc_class, parent, 0, semc_cdev,
				      "%s", semc_cdev->name);
	if (IS_ERR(semc_cdev->dev))
		return PTR_ERR(semc_cdev->dev);

	/* register the attributes */
	for (i = 0, rc = 0; i < semc_cdev->attr_number && !rc; i++) {
		rc = device_create_file(semc_cdev->dev, &semc_cdev->attributes[i]);
	}
	if (rc) {
    /* remove created attributes */
		if (i) {
			do {
				device_remove_file(semc_cdev->dev, &semc_cdev->attributes[--i]);
			} while (i);
		}
		goto err_out;
	}

	/* add to the list of class objects */
	down_write(&semcclass_list_lock);
	list_add_tail(&semc_cdev->node, &semcclass_list);
	up_write(&semcclass_list_lock);

	printk(KERN_INFO "Registered semc device: %s\n",
			semc_cdev->name);

	return 0;

err_out:
	device_unregister(semc_cdev->dev);
	return rc;
}
EXPORT_SYMBOL_GPL(semc_classdev_register);

/**
 * led_classdev_unregister - unregisters a object of led_properties class.
 * @semc_cdev: the led device to unregister
 *
 * Unregisters a previously registered via led_classdev_register object.
 */
void semc_classdev_unregister(struct semc_classdev *semc_cdev)
{
	int i;

	for (i = 0; i < semc_cdev->attr_number; i++)
		device_remove_file(semc_cdev->dev, &semc_cdev->attributes[i]);

	device_unregister(semc_cdev->dev);
	down_write(&semcclass_list_lock);
	list_del(&semc_cdev->node);
	up_write(&semcclass_list_lock);
}
EXPORT_SYMBOL_GPL(semc_classdev_unregister);

int semc_classdev_read_interface(const char *buf, int size, u32 *value, u8 base)
{
	ssize_t ret = -EINVAL;
	char *after;
	int count;

	*value = simple_strtoul(buf, &after, base);
	count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		ret = count;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(semc_classdev_read_interface);

static int __init semcclass_init(void)
{
	semc_class = class_create(THIS_MODULE, "semc");
	if (IS_ERR(semc_class))
		return PTR_ERR(semc_class);
	return 0;
}

static void __exit semcclass_exit(void)
{
	class_destroy(semc_class);
}

subsys_initcall(semcclass_init);
module_exit(semcclass_exit);

MODULE_AUTHOR("Aleksej Makarov");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEMC Class Interface");
