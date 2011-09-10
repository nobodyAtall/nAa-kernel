/*
   Sim Card sensor driver.

   Copyright (C) 2009 Sony Ericsson Mobile Communications AB.
   Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>

*/

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/semc/sim_card_detect.h>

#define DBG(X)  /* X */

#define DEV_NAME "simcard-sensor"
#define DEBOUNCE_TIMEOUT_MS 300

struct simc_data_t {
	struct delayed_work simc_status_check;
	struct platform_device *pdev;
	int gpio;
};

static void simc_update_status(struct work_struct *work)
{
	struct delayed_work *dwork =
			container_of(work, struct delayed_work, work);
	struct simc_data_t *simc =
			container_of(dwork, struct simc_data_t, simc_status_check);

	DBG(printk(KERN_DEBUG "%s (%s)\n",	DEV_NAME, __func__);)

	sysfs_notify(&simc->pdev->dev.kobj, NULL, "simdetect");
	enable_irq(gpio_to_irq(simc->gpio));

}

static void simc_data_init(struct simc_data_t *simc)
{
	INIT_DELAYED_WORK(&simc->simc_status_check, simc_update_status);
}

static irqreturn_t simc_int_callback(int vec, void *data)
{
	struct simc_data_t *simc = (struct simc_data_t *)data;

	DBG(printk(KERN_DEBUG "%s (%s): vec = 0x%x\n",
				DEV_NAME, __func__, vec);)

	disable_irq(gpio_to_irq(simc->gpio));
	schedule_delayed_work(&simc->simc_status_check,
						  msecs_to_jiffies(DEBOUNCE_TIMEOUT_MS));

	return IRQ_HANDLED;
}


static int register_gpio_callback(irq_handler_t func, void *data)
{
	int err;
	struct simc_data_t *simc = (struct simc_data_t *)data;
	int gpio_n = simc->gpio;

	err = gpio_request(gpio_n , "SIM card detect interrupt");
	if (err) {
		printk(KERN_ERR "%s (%s): Could not allocate GPIO %d.\n",
			   DEV_NAME, __func__, gpio_n);
		goto force_return;
	}

	err = request_irq(
					gpio_to_irq(gpio_n),
					func,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					"SIM interrupt",
					data);
	if (err) {
		printk(KERN_ERR "%s (%s): Failed requesting %d IRQ.\n",
				DEV_NAME, __func__, gpio_to_irq(gpio_n));
	}
force_return:
	DBG(printk(KERN_DEBUG "%s (%s): returned status %d\n",
				DEV_NAME, __func__, err);)
	return err;
}

static void unregister_gpio_callback(void * data)
{
	struct simc_data_t *simc = (struct simc_data_t *)data;

	free_irq(gpio_to_irq(simc->gpio), &data);
}

static ssize_t get_simc_inserted(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct simc_data_t *simc = platform_get_drvdata(pdev);

	return sprintf(buf, "%u\n", gpio_get_value(simc->gpio));
}

static  struct device_attribute sysfs_interface =
	__ATTR(simdetect, 0444, get_simc_inserted, NULL);

static int create_sysfs_interfaces(struct platform_device *pdev)
{
	if (!device_create_file(&pdev->dev, &sysfs_interface))
		return 0;

	printk(KERN_ERR "%s: %s Unable to create interface\n", DEV_NAME, __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &sysfs_interface);
}

static int __devinit simc_probe(struct platform_device *pdev)
{
	int result;
	struct simc_data_t *simc;

	simc = kzalloc(sizeof(struct simc_data_t), GFP_KERNEL);
	if (!simc) {
		result = -ENOMEM;
		goto fail_alloc_mem;
	}
	simc_data_init(simc);

	if(create_sysfs_interfaces(pdev)) {
		result = -ENODEV;
		goto fail_alloc_mem;
	}

	platform_set_drvdata(pdev, simc);
	simc->pdev = pdev;
	simc->gpio =
			((struct sim_detect_platform_data *)pdev->dev.platform_data)->gpio;
	result = register_gpio_callback(simc_int_callback, simc);
	if (result)
		goto fail_irq_req;

	printk(KERN_DEBUG "%s: registered\n", DEV_NAME);
	return 0;

fail_irq_req:
	remove_sysfs_interfaces(pdev);
fail_alloc_mem:
	kfree(simc);
	printk(KERN_DEBUG "%s: failed\n", DEV_NAME);
	return result;
}

static int __devexit simc_remove(struct platform_device *pdev)
{
	struct simc_data_t *simc = platform_get_drvdata(pdev);
	unregister_gpio_callback(simc);
	remove_sysfs_interfaces(pdev);
	platform_set_drvdata(pdev, NULL);
	kfree(simc);
	return 0;
}

static struct platform_driver simc_driver = {
	.probe		= simc_probe,
	.remove		= __devexit_p(simc_remove),
	.driver		= {
		.name = DEV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init simc_init(void)
{
	return platform_driver_register(&simc_driver);
}
module_init(simc_init);

static void __exit simc_exit(void)
{
	platform_driver_unregister(&simc_driver);
}
module_exit(simc_exit);


MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_DESCRIPTION("sim card sensor driver");
MODULE_LICENSE("GPL");

