/* SEMC:modified */
/* 
   Audio Jack Driver

   Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License, version 2, as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/atomic.h>
#include <mach/gpio.h>
#include <linux/wakelock.h>

#include "proc_comm.h"
#include "es209ra_headset.h"

#ifdef	CONFIG_ARM
#include <asm/mach-types.h>
#endif

#define HEADSET_BUTTON_PRESS    0x84
#define HEADSET_BUTTON_RELEASE  0x85

#define DEBUG
//#define DEBUG_VERBOSE

struct audio_jack_driver {
	struct switch_dev swDev;
	struct input_dev *inDev;
	struct wake_lock audiojack_wakelock;
	
	unsigned int interrupt;
	unsigned int ischeckirq;
	unsigned int need_no_wait;
	unsigned int need_evaluate_timer;
	unsigned int onemorecheck;
	
	unsigned int gpio_det_in;
	unsigned int gpio_det_out;
	unsigned int headset_plugged_wait_time;

	struct hrtimer timer;
	ktime_t debounceTime;
	struct hrtimer retryTimer;
	ktime_t retryDebounceTime;
	struct hrtimer evaluateTimer;
	ktime_t evaluateDebounceTime;
};

static struct audio_jack_driver *jack;

enum {
	NO_DEVICE			= 0,
	ES209RA_PHF			= 1,
	ES209RA_HEADSET		= 2,
	DEVICE_UNKNOWN		= 0xFF,
};

/* work queue */
static struct workqueue_struct *audiojackworkqueue;
static struct work_struct audiojackwork;

/* Threshold information for detection */
static unsigned int headset_threshold_voltage_low  = 200;
static unsigned int headset_threshold_voltage_high  = 1900;
/* Threshold information for button */
static unsigned int button_pressed_threshold = 200;
static unsigned int button_released_threshold = 200;
/* sampling time information*/
static unsigned int headset_sampling_time = 100;
static unsigned int headset_button_check_time = 100;
/* internal use flag to check current connected accessory */
static unsigned int es209ra_current_accessory_state = NO_DEVICE;


static ssize_t es209ra_audio_jack_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&jack->swDev))
	{
		case NO_DEVICE:
		{
			printk(KERN_INFO "ES209RA Audio Jack Driver : No Device\n");
			return sprintf(buf, "No Device\n");
		}
		case ES209RA_PHF:
		{
			printk(KERN_INFO "ES209RA Audio Jack Driver : PHF\n");
			return sprintf(buf, "PHF\n");
		}
		case ES209RA_HEADSET:
		{
			printk(KERN_INFO "ES209RA Audio Jack Driver : Headset\n");
			return sprintf(buf, "Headset\n");
		}
	}
	return -EINVAL;
}


static int es209ra_audio_jack_set_sample_time(unsigned int *sample_time_ms, unsigned int *button_time_ms)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_SET_HEADSET_ADC_OPT, sample_time_ms, button_time_ms);
	
	return ret;
}


static int es209ra_audio_jack_set_threshold(unsigned int *threshold_min, unsigned int *threshold_mid)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_SET_HEADSET_ADC_THRESHOLD_OPT, threshold_min, threshold_mid);
	
	return ret;
}


static int es209ra_audio_jack_get_adc_value(unsigned int *valid, unsigned int *adc_value)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_GET_HEADSET_ADC_VALUE, valid, adc_value);
	
	return ret;
}


static int es209ra_audio_jack_notify_plug_detect_to_amss(void)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_HEADSET_PLUGGED, 0, 0);
	
	return ret;
}


static int es209ra_audio_jack_notify_plug_removed_to_amss(void)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_HEADSET_UNPLUGGED, 0, 0);
	
	return ret;
}


static int es209ra_audio_jack_notify_suspend_to_amss(void)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_HEADSET_SUSPEND, 0, 0);
	
	return ret;
}


static int es209ra_audio_jack_notify_resume_to_amss(void)
{
	int ret;
	
	ret = msm_proc_comm(PCOM_HEADSET_RESUME, 0, 0);
	
	return ret;
}


static int es209ra_audio_jack_calc_average(int totalnum, int* values)
{
	int ave, i, sumvalue;
	
	/* init auto variables */
	ave = 0;
	sumvalue = 0;
	
	for (i = 0; i < totalnum; i++)
	{
		sumvalue += values[i];
	}
	
	if (totalnum)
	{
		ave = sumvalue / totalnum;
	}
	
	return ave;
}


static void es209ra_audio_jack_remove_headset(void)
{
	switch_set_state(&jack->swDev, NO_DEVICE);
}


static void es209ra_audio_jack_insert_headset(unsigned int type)
{
	switch_set_state(&jack->swDev, type);
}


static void es209ra_audio_jack_button_pressed(void)
{
	/* wake lock start */
	wake_lock(&jack->audiojack_wakelock);
	msleep(50);

	if (ES209RA_PHF == es209ra_current_accessory_state)
	{
		printk(KERN_INFO "ES209RA Audio Jack Driver : Button Pressed!!!\n");
		input_report_key(jack->inDev, KEY_MEDIA, 1);
		input_sync(jack->inDev);
		msleep(100);
	}
	else
	{
		/* No opeartion */
#ifdef DEBUG
	    printk(KERN_INFO "Ignore button pressed event\n");
#endif // #ifdef DEBUG
	}
	/* unlock the wakelock */
	wake_unlock(&jack->audiojack_wakelock);
}


static void es209ra_audio_jack_button_released(void)
{
	unsigned int valid, adc_value, i, get_average_value;
	int count = 0;
	int retry_limit = 3;
	int getadcvalue[retry_limit];
	
	/* wake lock start */
	wake_lock(&jack->audiojack_wakelock);
	
	if (ES209RA_PHF == es209ra_current_accessory_state)
	{
		printk(KERN_INFO "ES209RA Audio Jack Driver : Button Released!!!\n");
		input_report_key(jack->inDev, KEY_MEDIA, 0);
		input_sync(jack->inDev);
		msleep(100);
	}
	else
	{
		msleep(headset_button_check_time*2);
		/* check earphone or PHF */
		for (i = 0; i < retry_limit; i++)
		{
			es209ra_audio_jack_get_adc_value(&valid, &adc_value);
			if(valid)
			{
				getadcvalue[i] = adc_value;
				count++;
			}
		}
		get_average_value = es209ra_audio_jack_calc_average(count, getadcvalue);
#ifdef DEBUG
	    printk(KERN_INFO "ES209RA Audio Jack Driver : get_average_value=%d in es209ra_audio_jack_button_released\n", get_average_value);
#endif // #ifdef DEBUG
		if ((headset_threshold_voltage_low < get_average_value) &&
				(headset_threshold_voltage_high > get_average_value))
		{
			/* detect PHF */
			es209ra_current_accessory_state = ES209RA_PHF;
		}
		else if (headset_threshold_voltage_low > get_average_value)
		{
			/* detect earphone */
			es209ra_current_accessory_state = ES209RA_HEADSET;
		}
		
		if (ES209RA_PHF == es209ra_current_accessory_state)
		{
			es209ra_audio_jack_remove_headset();
			msleep(1250);
			es209ra_audio_jack_insert_headset(es209ra_current_accessory_state);
		}
	}
	/* unlock the wakelock */
	wake_unlock(&jack->audiojack_wakelock);
}


void es209ra_audio_jack_button_handler(int key)
{
	switch(key)
	{
		case HEADSET_BUTTON_PRESS:
		{
			es209ra_audio_jack_button_pressed();
		}
		break;
		
		case HEADSET_BUTTON_RELEASE:
		{
			es209ra_audio_jack_button_released();
		}
		break;
	}
}


static enum hrtimer_restart es209ra_audio_jack_timer_detect_event(struct hrtimer *data)
{
	jack->need_no_wait = 0;
	queue_work(audiojackworkqueue, &audiojackwork);
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart es209ra_audio_jack_timer_retry_event(struct hrtimer *data)
{
	jack->need_no_wait = 1;
	if (!jack->need_evaluate_timer || jack->onemorecheck)
	{
		if (jack->onemorecheck)
			jack->onemorecheck = 0;
		queue_work(audiojackworkqueue, &audiojackwork);
	}
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart es209ra_audio_jack_timer_evaluate_finished_event(struct hrtimer *data)
{
	jack->need_evaluate_timer = 1;
	hrtimer_cancel(&jack->retryTimer);
	return HRTIMER_NORESTART;
}


static void es209ra_audio_jack_detection_work(struct work_struct *work)
{
	int getgpiovalue, i, count;
	int retry_limit = 3;
	int getadcvalue[retry_limit];
	unsigned int adc_value, valid, get_average_value;
	unsigned int es209ra_previous_accessory_state;
	
	/* init auto variable */
	count = 0;
	get_average_value = 0;
	es209ra_previous_accessory_state = es209ra_current_accessory_state;
	
	/* wake lock start */
	wake_lock(&jack->audiojack_wakelock);
	
	/* sleep 800ms */
	if (!jack->need_no_wait)
	{
		msleep(jack->headset_plugged_wait_time);
	}
	
	/* Something plugged in or out, lets make sure what plugged in or out */
	getgpiovalue = gpio_get_value(jack->gpio_det_in);
	
	if (!getgpiovalue)
	{
		/* detect earphone or PHF */
		/* notify plug detect to AMSS */
		es209ra_audio_jack_notify_plug_detect_to_amss();
		msleep(headset_button_check_time*2);
		
		/* check earphone or PHF */
		for (i = 0; i < retry_limit; i++)
		{
			es209ra_audio_jack_get_adc_value(&valid, &adc_value);
			if(valid)
			{
				getadcvalue[i] = adc_value;
				count++;
			}
		}
		get_average_value = es209ra_audio_jack_calc_average(count, getadcvalue);
		
#ifdef DEBUG
		printk(KERN_INFO "ES209RA Audio Jack Driver : get_average_value = %d\n", get_average_value);
#endif // #ifdef DEBUG
		if ((headset_threshold_voltage_low < get_average_value) &&
				(headset_threshold_voltage_high > get_average_value))
		{
			/* detect PHF */
			es209ra_current_accessory_state = ES209RA_PHF;
		}
		else if (headset_threshold_voltage_low > get_average_value)
		{
			/* detect earphone */
			es209ra_current_accessory_state = ES209RA_HEADSET;
		}
		else
		{
			/* unknown device inserted */
			es209ra_current_accessory_state = DEVICE_UNKNOWN;
		}
	}
	else
	{
		/* set current status to NO_DEVICE */
		es209ra_current_accessory_state = NO_DEVICE;
	}
	
	/* Change check flag */
	jack->ischeckirq = 1;
	
#ifdef DEBUG
	printk(KERN_INFO "ES209RA Audio Jack Driver : previous_state = %d\n", es209ra_previous_accessory_state);
	printk(KERN_INFO "ES209RA Audio Jack Driver : current_state = %d\n", es209ra_current_accessory_state);
#endif // #ifdef DEBUG
	
	if (DEVICE_UNKNOWN == es209ra_current_accessory_state)
	{
		es209ra_audio_jack_notify_plug_removed_to_amss();
		es209ra_audio_jack_remove_headset();
		jack->onemorecheck = 0;
		/* set timer to check ADC value again and return immediately */
		hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
		/* set timer for evaluating */
		if (jack->need_evaluate_timer)
		{
			hrtimer_start(&jack->evaluateTimer, jack->evaluateDebounceTime, HRTIMER_MODE_REL);
			jack->need_evaluate_timer = 0;
		}
		/* unlock the wakelock to prepare the next wakelock */
		wake_unlock(&jack->audiojack_wakelock);
		return;
	}
	else
	{
		/* cancel retryTimer and evaluateTimer here since lo longer need to retry */
		/* starting rertyTimer and evaluateTimer equal previous state is DEVICE_UNKNOWN */
		if (DEVICE_UNKNOWN == es209ra_previous_accessory_state)
		{
			hrtimer_cancel(&jack->retryTimer);
			hrtimer_cancel(&jack->evaluateTimer);
			jack->need_evaluate_timer = 1;
		}
		/* Check previous state and current state and take action */
		/* previous state is NO_DEVICE */
		if (NO_DEVICE == es209ra_previous_accessory_state)
		{
			if (ES209RA_HEADSET == es209ra_current_accessory_state ||
					ES209RA_PHF == es209ra_current_accessory_state)
			{
				es209ra_audio_jack_insert_headset(es209ra_current_accessory_state);
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
		}
		/* previous state is ES209RA_PHF */
		else if (ES209RA_PHF == es209ra_previous_accessory_state)
		{
			if (NO_DEVICE == es209ra_current_accessory_state)
			{
				/* notify plug removed to AMSS */
				es209ra_audio_jack_notify_plug_removed_to_amss();
				es209ra_audio_jack_remove_headset();
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
			else if (ES209RA_HEADSET == es209ra_current_accessory_state)
			{
				es209ra_audio_jack_remove_headset();
				msleep(1250);
				es209ra_audio_jack_insert_headset(es209ra_current_accessory_state);
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
		}
		/* previous state is ES209RA_HEADSET */
		else if (ES209RA_HEADSET == es209ra_previous_accessory_state)
		{
			if (NO_DEVICE == es209ra_current_accessory_state)
			{
				/* notify plug removed to AMSS */
				es209ra_audio_jack_notify_plug_removed_to_amss();
				es209ra_audio_jack_remove_headset();
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
			else if (ES209RA_PHF == es209ra_current_accessory_state)
			{
				es209ra_audio_jack_remove_headset();
				msleep(1250);
				es209ra_audio_jack_insert_headset(es209ra_current_accessory_state);
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
		}
		/* previous state is DEVICE_UNKNOWN */
		else
		{
			if (NO_DEVICE == es209ra_current_accessory_state)
			{
				/* notify plug removed to AMSS */
				es209ra_audio_jack_notify_plug_removed_to_amss();
				es209ra_audio_jack_remove_headset();
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
			else if (ES209RA_PHF == es209ra_current_accessory_state)
			{
				/* notify dummy plug insert signal to AMSS to enable MIC bias for button detection */
				es209ra_audio_jack_notify_plug_removed_to_amss();
				msleep(10);
				es209ra_audio_jack_notify_plug_detect_to_amss();
				es209ra_audio_jack_insert_headset(es209ra_current_accessory_state);
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
			else
			{
				es209ra_audio_jack_notify_plug_removed_to_amss();
				es209ra_audio_jack_insert_headset(es209ra_current_accessory_state);
				/* just one more check the current status to secure notification to upper layer */
				if (jack->onemorecheck)
					hrtimer_start(&jack->retryTimer, jack->retryDebounceTime, HRTIMER_MODE_REL);
			}
		}
	}
	/* unlock the wakelock */
	wake_unlock(&jack->audiojack_wakelock);
}


static irqreturn_t es209ra_audio_jack_detect_irq_handler(int irq, void *dev_id)
{
	int value;
	
#ifdef DEBUG_VERBOSE
	printk(KERN_INFO "ES209RA Audio Jack Driver : irq\n");
#endif // #ifdef DEBUG
	
	if (jack->ischeckirq)
	{
		/* Change check flag */
		jack->ischeckirq = 0;
		/* Change wait flag */
		jack->need_no_wait = 0;
		/* Change one more check flag */
		jack->onemorecheck = 1;
		/* check current gpio value */
		value = gpio_get_value(jack->gpio_det_in);
		set_irq_type(jack->interrupt, value ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		
#ifdef DEBUG
		printk(KERN_INFO "ES209RA Audio Jack Driver : irq locked. value = %d\n", value);
#endif // #ifdef DEBUG
		
		/* start detection_work to determine what kind of applications has been inserted */
		hrtimer_start(&jack->timer, jack->debounceTime, HRTIMER_MODE_REL);
	}
	
	return IRQ_HANDLED;
}


static int es209ra_audio_jack_probe(struct platform_device *pdev)
{
	int ret;
	struct es209ra_headset_platform_data	*pdata = pdev->dev.platform_data;
	
	printk(KERN_INFO "H2W: Registering H2W (headset) driver\n");
	jack = kzalloc(sizeof(struct audio_jack_driver), GFP_KERNEL);
	if (!jack)
	{
		return -ENOMEM;
	}
	
	if (!pdata) {
		printk(KERN_ERR "no platform data?\n");
		return -ENODEV;
	}
	
	/* initialize */
	jack->gpio_det_in = pdata->gpio_detin;
	jack->gpio_det_out = pdata->gpio_detout;
	jack->headset_plugged_wait_time = pdata->wait_time;
	
	/* Set gpio value to HIGH */
	gpio_set_value(jack->gpio_det_out, 1);
	/* set detection debounce time to 0ms (immediately) */
	jack->debounceTime = ktime_set(0, 0);
	/* Set button debounce time to 500ms */
	jack->retryDebounceTime = ktime_set(0, 500000000);
	/* Set evaluate time to 120s */
	jack->evaluateDebounceTime = ktime_set(120, 0);
	/* device name */
	jack->swDev.name = "h2w";
	/* print function name */
	jack->swDev.print_name = es209ra_audio_jack_print_name;
	/* Set check irq flag */
	jack->ischeckirq = 1;
	/* Set wait flag */
	jack->need_no_wait = 0;
	/* Set evaluate need flag */
	jack->need_evaluate_timer = 1;
	
	ret = switch_dev_register(&jack->swDev);
	if (ret < 0)
	{
		printk(KERN_ERR "ES209RA Audio Jack Driver : switch_dev_register failed\n");
		goto err_switch_dev_register;
	}
	
	/* Initialize work queue for ES209RA Audio Jack Driver */
	audiojackworkqueue = create_workqueue("es209ra_headset_wq");
	INIT_WORK(&audiojackwork, es209ra_audio_jack_detection_work);
	
	ret = gpio_request(jack->gpio_det_in, "es209ra_detect");
	if (ret < 0)
	{
		printk(KERN_ERR "ES209RA Audio Jack Driver : gpio_request failed\n");
		goto err_request_detect_gpio;
	}
	
	ret = gpio_direction_input(jack->gpio_det_in);
	if (ret < 0)
	{
		printk(KERN_ERR "ES209RA Audio Jack Driver : gpio_direction_input failed\n");	
		goto err_set_detect_gpio;
	}
	
	jack->interrupt = ret = gpio_to_irq(jack->gpio_det_in);
	if (ret < 0) {
		printk(KERN_ERR "ES209RA Audio Jack Driver : gpio_to_irq failed\n");
		goto err_get_h2w_detect_irq_num_failed;
	}
	/* set timer function for detection */
	hrtimer_init(&jack->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	jack->timer.function = es209ra_audio_jack_timer_detect_event;
	/* set timer function for retry */
	hrtimer_init(&jack->retryTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	jack->retryTimer.function = es209ra_audio_jack_timer_retry_event;
	/* set timer function for evaluation */
	hrtimer_init(&jack->evaluateTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	jack->evaluateTimer.function = es209ra_audio_jack_timer_evaluate_finished_event;
	
	jack->inDev = input_allocate_device();
	if (!jack->inDev) {
		ret = -ENOMEM;
		printk(KERN_ERR "ES209RA Audio Jack Driver : Failed to allocate input device\n");
		goto err_request_input_dev;
	}
	
	jack->inDev->name = pdata->keypad_name;
	jack->inDev->evbit[0] = BIT_MASK(EV_KEY);
	jack->inDev->keybit[BIT_WORD(KEY_MEDIA)] = BIT_MASK(KEY_MEDIA);
	
	ret = input_register_device(jack->inDev);
	if (ret < 0)
	{
		printk(KERN_ERR "ES209RA Audio Jack Driver : input_register_device failed\n");
		goto err_register_input_dev;
	}
	
	/* set sampling time to AMSS */
	es209ra_audio_jack_set_sample_time(&headset_sampling_time, &headset_button_check_time);
	
	/* set threshold voltage */
	es209ra_audio_jack_set_threshold(&button_released_threshold, &button_pressed_threshold);
	
	/* init wakelock */
	wake_lock_init(&jack->audiojack_wakelock, WAKE_LOCK_SUSPEND, "audiojacklock");
	
	/* Disable button until plugged in */
	ret = set_irq_wake(jack->interrupt, 1);
	if (ret < 0)
		goto err_request_input_dev;
	
	/* IRQ request */
	ret = request_irq(jack->interrupt, es209ra_audio_jack_detect_irq_handler, IRQF_TRIGGER_FALLING, "audiojackirq", NULL);
	if (ret < 0)
	{
		printk(KERN_ERR "ES209RA Audio Jack Driver : request_irq failed\n");
		goto err_request_detect_irq;
	}
	
	printk(KERN_INFO "ES209RA Audio Jack Driver : probe finished\n");
	
	return 0;
	
err_register_input_dev:
	input_free_device(jack->inDev);
err_request_input_dev:
err_request_detect_irq:
err_get_h2w_detect_irq_num_failed:
err_set_detect_gpio:
	gpio_free(jack->gpio_det_in);
err_request_detect_gpio:
	switch_dev_unregister(&jack->swDev);
err_switch_dev_register:
		printk(KERN_ERR "ES209RA Audio Jack Driver : Failed to register driver\n");
	
	printk(KERN_ERR "ES209RA Audio Jack Driver : Failed to register driver in probe\n");
	return ret;
}


static int es209ra_audio_jack_remove(struct platform_device *pdev)
{
	if (switch_get_state(&jack->swDev))
	{
		es209ra_audio_jack_remove_headset();
	}
	input_unregister_device(jack->inDev);
	gpio_free(jack->gpio_det_out);
	gpio_free(jack->gpio_det_in);
	free_irq(jack->interrupt, 0);
	switch_dev_unregister(&jack->swDev);
	wake_lock_destroy(&jack->audiojack_wakelock);
	
	return 0;
}


static int es209ra_audio_jack_suspend(struct platform_device *pdev, pm_message_t message)
{
#ifdef DEBUG
	printk(KERN_INFO "ES209RA Audio Jack Driver : es209ra_audio_jack_suspend START\n");
#endif // #ifdef DEBUG
	
	/* notify suspend signal to AMSS to disable MIC bias enabling*/
	es209ra_audio_jack_notify_suspend_to_amss();
	
#ifdef DEBUG
	printk(KERN_INFO "ES209RA Audio Jack Driver : es209ra_audio_jack_suspend END\n");
#endif // #ifdef DEBUG
	
	return 0;
}


static int es209ra_audio_jack_resume(struct platform_device *pdev)
{
#ifdef DEBUG
	printk(KERN_INFO "ES209RA Audio Jack Driver : es209ra_audio_jack_resume START\n");
#endif // #ifdef DEBUG
	
	/* notify suspend signal to AMSS to disable MIC bias enabling */
	es209ra_audio_jack_notify_resume_to_amss();
	
#ifdef DEBUG
	printk(KERN_INFO "ES209RA Audio Jack Driver : es209ra_audio_jack_resume END\n");
#endif // #ifdef DEBUG
	
	return 0;
}


static struct platform_driver es209ra_audio_jack = {
	.probe		= es209ra_audio_jack_probe,
	.remove		= es209ra_audio_jack_remove,
	.suspend	= es209ra_audio_jack_suspend,
	.resume		= es209ra_audio_jack_resume,
	.driver		= {
		.name		= "es209ra_audio_jack",
		.owner	= THIS_MODULE,
	},
};


static int __init es209ra_audio_jack_init(void)
{
	int ret;
	ret = platform_driver_register(&es209ra_audio_jack);
	return ret;
}


static void __exit es209ra_audio_jack_exit(void)
{
	platform_driver_unregister(&es209ra_audio_jack);
}

module_init(es209ra_audio_jack_init);
module_exit(es209ra_audio_jack_exit);

MODULE_AUTHOR("SEMC");
MODULE_DESCRIPTION("3.5mm audio jack driver");
MODULE_LICENSE("GPL");
