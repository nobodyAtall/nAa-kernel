/*
 * MSM PMIC vibrator driver
 *
 *
 * Copyright (C) 2010 SonyEricsson Mobile Communications AB
 *
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
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <mach/pmic.h>
#include <linux/semc/msm_pmic_vibrator.h>

#define DRV_NAME "msm_pmic_vibrator"
#define DRIVER_VERSION  "0.2"

#define DBG(X) /*X*/

enum vibrator_state_t {
	STATE_VIB_INIT = 0x00,
	STATE_VIB_ON = 0x01
};

struct pmic_vibrator_data_t {
	struct mutex 	update_lock;
	struct timer_list  tmr;
	u16  on_voltage;
	enum vibrator_state_t state;
	struct task_struct *vib_task;
	struct mutex vib_task_lock;
	struct msm_pmic_vibrator_platform_data pdata;
};

static struct pmic_vibrator_data_t vib;

static int set_vib_voltage(u16 volt)
{
	int rc;
	rc = vib.pdata.platform_set_vib_voltage(volt);
	if(rc)
		printk(KERN_ERR "%s: Failed to set motor voltage %d\n", DRV_NAME, volt);
	return rc;
}

static int init_vib_hw(void)
{
	return vib.pdata.platform_init_vib_hw();
}

static void release_vib_hw(void)
{
	set_vib_voltage(vib.pdata.off_voltage);
}

static int vib_on(u16 milliseconds)
{
	int rc;

	if(milliseconds < vib.pdata.mimimal_on_time)
		milliseconds = vib.pdata.mimimal_on_time;
	rc = set_vib_voltage(vib.on_voltage);
	if(rc)
		goto err;
	DBG(printk(KERN_DEBUG "%s: Turn motor on for %d ms\n",
			   DRV_NAME, milliseconds);)
	vib.state |= STATE_VIB_ON;
	mod_timer(&vib.tmr, jiffies + msecs_to_jiffies(milliseconds));

err:
	return rc;
}

static int vib_off(void)
{
	int rc;
	rc = set_vib_voltage(vib.pdata.off_voltage);
	DBG(printk(KERN_DEBUG "%s: Turn motor off.\n", DRV_NAME);)
	if(!rc)
		vib.state &= ~STATE_VIB_ON;
	return rc;
}

static int vib_task_function(void *data)
{
	struct sched_param param;

	param.sched_priority = MAX_RT_PRIO - 1;
	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		mutex_lock(&vib.vib_task_lock);
		vib_off();
	}
	return 0;
}

static void timer_callback(unsigned long d)
{
	mutex_unlock(&vib.vib_task_lock);
}

static ssize_t vib_set_enable(struct device *dev,
							  struct device_attribute *attr,
							  const char *buf, size_t size)
{
	u32 value;
	char *p;
	int ret;

	value = simple_strtoul(buf, &p, 10);
	ret = (p - buf);
	if(!ret)
		return -EINVAL;

	mutex_lock(&vib.update_lock);
	if (value)
		vib_on(value);
	else
		vib_off();
	mutex_unlock(&vib.update_lock);

	return ret;
}

static ssize_t vib_set_voltage(struct device *dev,
							   struct device_attribute *attr,
							   const char *buf, size_t size)
{
	u32 value;
	char *p;
	int ret;

	value = simple_strtoul(buf, &p, 10);
	ret = (p - buf);
	if(!ret)
		return -EINVAL;
	mutex_lock(&vib.update_lock);
	if (value > vib.pdata.max_voltage)
		vib.on_voltage = vib.pdata.max_voltage;
	else if (value < vib.pdata.min_voltage)
		vib.on_voltage = vib.pdata.min_voltage;
	else
		vib.on_voltage = value;
	mutex_unlock(&vib.update_lock);

	return ret;
}

static ssize_t vib_get_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", !!(vib.state & STATE_VIB_ON));
}

static ssize_t vib_get_voltage(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", vib.on_voltage);
}

static struct device_attribute vib_attributes[] = {
	__ATTR(enable,  0666, vib_get_enable,  vib_set_enable),
	__ATTR(voltage_mv,  0644, vib_get_voltage,  vib_set_voltage),
}; 

static int create_sysfs_interfaces(struct platform_device *pdev)
{
	int i;

	for (i = 0;
		 i < sizeof(vib_attributes) / sizeof(struct device_attribute); i++) {
		if (device_create_file(&pdev->dev, vib_attributes + i))
			goto error;
	}
	return 0;
error:
	for (; i >=0; i--)
		device_remove_file(&pdev->dev, vib_attributes + i);
	printk(KERN_ERR "%s: %s Unable to create interface\n", DRV_NAME, __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct platform_device *pdev)
{
	int i;

	for (i = 0;
		 i < sizeof(vib_attributes) / sizeof(struct device_attribute); i++)
		device_remove_file(&pdev->dev, vib_attributes + i);
}

static int init_vib_data(struct msm_pmic_vibrator_platform_data *pdata)
{
	if (!pdata) {
		printk(KERN_INFO "%s (%s) platform data required.\n",
			   DRV_NAME, __func__);
		return -1;
	}

	memcpy(&vib.pdata, pdata, sizeof(vib.pdata));
	mutex_init(&vib.update_lock);
	mutex_init(&vib.vib_task_lock);
	mutex_lock(&vib.vib_task_lock);
	setup_timer(&vib.tmr, timer_callback, 0);
	vib.vib_task = kthread_run(vib_task_function, vib.vib_task, DRV_NAME);
	vib.on_voltage = vib.pdata.default_voltage;
	vib.state = STATE_VIB_INIT;
	return 0;
}

static void release_vib_data(void)
{
	del_timer_sync(&vib.tmr);
}

static int vib_probe(struct platform_device *pdev)
{
	int res;
	struct msm_pmic_vibrator_platform_data *pdata =
			(struct msm_pmic_vibrator_platform_data *)pdev->dev.platform_data;

	if (init_vib_data(pdata))
		return -ENODEV;

	res = init_vib_hw();
	if (res)
		goto exit_hw_failed;

	if(create_sysfs_interfaces(pdev)) {
		res = -ENODEV;
		goto exit_classdev_err;
	}
	printk(KERN_INFO "%s version %s loaded\n", DRV_NAME, DRIVER_VERSION);

	return 0;

exit_classdev_err:
	release_vib_hw();
exit_hw_failed:
	printk(KERN_ERR "%s version %s installation failed\n",
		   DRV_NAME, DRIVER_VERSION);
	return res;
}

static int __devexit vib_remove(struct platform_device *pdev)
{
	release_vib_hw();
	release_vib_data();
	remove_sysfs_interfaces(pdev);
	printk(KERN_INFO "%s version %s removed\n", DRV_NAME, DRIVER_VERSION);
	return 0;
}


static struct platform_driver vibrator_driver = {
	.probe = vib_probe,
	.remove = __devexit_p(vib_remove),
	.driver = {
	.name = DRV_NAME,
	.owner = THIS_MODULE,
	},
};

static int __init vib_init(void)
{
	int rc = platform_driver_register(&vibrator_driver);
	DBG(printk(KERN_DEBUG "%s: %s rc=%d\n", DRV_NAME, __func__, rc);)
	return rc;
}

static void __exit vib_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}

module_init(vib_init);
module_exit(vib_exit);

MODULE_AUTHOR("Aleksej Makarov (aleksej.makarov@sonyericsson.com)");
MODULE_DESCRIPTION("msm pmic vibrator driver");
MODULE_LICENSE("GPL");

