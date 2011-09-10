/* drivers/misc/semc/seport/seport_button_detect.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

/*
 * This file handles button detection from accessories.
 * Since we do not know how fast userspace can handle the
 * buttons, and this driver may be much faster, we add all
 * button events to a linked list in order. This is to make
 * sure that no button event whatsoever will be forgotten.
 */

#include <linux/slab.h>

#include "seport_button_detect.h"
#include "seport_attrs.h"

#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <mach/semc_seport_platform.h>

#include <linux/semc/seport/seport.h>
#include <linux/semc/seport/seport_dcout_iface.h>

/*
 * Wakelock is not and will most likely never be part of the official kernel.
 * I really hate to use it, but I currently have no other choice.
 * Linus Torvalds, please forgive me!
 */
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#define NUM_BUTTON_READS 10
#define MIN_VALID_BUTTON_EVENTS 7
#define NUM_VALID_BUTTON_IDS  20

#define SEPORT_BUTTON_INITIAL_VALUES 0
static wait_queue_head_t seport_btn_wq;

static u8 seport_buttons_online = SEPORT_BUTTON_INITIAL_VALUES;
static u8 seport_vad_online = SEPORT_BUTTON_INITIAL_VALUES;
static u8 seport_buttons_shutdown = SEPORT_BUTTON_INITIAL_VALUES;
static u8 seport_last_btn_id = SEPORT_BUTTON_INITIAL_VALUES;

LIST_HEAD(seport_button_event_list);

/* This struct contains the required information. No use creating a new one
 * although the name may be confusing */
struct seport_plug_detect_data vad_det_data = {
	.status = ATOMIC_INIT(0)
};

/*
 * Used for linked list containing button events.
 */
struct seport_event_node {
	struct seport_button_event *event;
	struct list_head list;
};

/* Mutex used when adding/removing events from linked list. */
static DEFINE_MUTEX(seport_button_event_lock);
/* Mutex used when client is retrieving an event via read.
 * and accessing local variables */
static DEFINE_MUTEX(seport_button_generic_lock);

#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock seport_btn_wake_lock;
#endif


/** Interrupt handling function defenitions */
static void seport_gpio_buttondetect_work(struct work_struct *work);
static int seport_gpio_buttondetect_callback(int irq, void *data);

static struct workqueue_struct *seport_vadsense_wq;
static struct workqueue_struct *seport_btn_det_work;

static struct work_struct seport_ain_work;

static int seport_vad_buttondetect_callback(int irq, void *data);

/** Private functions **/
static u8 button_events_available(void)
{
	u8 retval = 0;

	mutex_lock(&seport_button_event_lock);
	retval = (u8)!list_empty(&seport_button_event_list);
	mutex_unlock(&seport_button_event_lock);

	return retval;
}

static struct seport_button_event *seport_get_next_button_event(void)
{
	struct seport_event_node *node;
	struct seport_button_event *ev = NULL;

	mutex_lock(&seport_button_event_lock);
	if (!list_empty(&seport_button_event_list)) {
		node = list_first_entry(&seport_button_event_list,
					struct seport_event_node, list);
		ev = node->event;
		list_del(&node->list);
		kfree(node);
	}

	mutex_unlock(&seport_button_event_lock);

	return ev;
}

static int seport_add_button_event(struct seport_button_event *ev)
{

	struct seport_event_node *evn =
		kmalloc(sizeof(struct seport_event_node),
						GFP_KERNEL);
	if (NULL == evn) {
		printk(KERN_ERR "%s : Failed to allocate memory " \
		       "for button event node!\n"
		       , __func__);
		return -1;
	}

	evn->event = ev;

	mutex_lock(&seport_button_event_lock);
	list_add_tail(&evn->list, &seport_button_event_list);
	mutex_unlock(&seport_button_event_lock);

	wake_up_interruptible(&seport_btn_wq);

	return 0;
}

static void seport_clear_button_list(void)
{
	struct list_head *ptr, *n;
	struct seport_event_node *evn;

	mutex_lock(&seport_button_event_lock);
	if (!list_empty(&seport_button_event_list)) {
		list_for_each_safe(ptr, n, &seport_button_event_list) {
			evn = list_entry(ptr, struct seport_event_node, list);
			list_del(ptr);
			kfree(evn->event);
			kfree(evn);
		}
	}
	mutex_unlock(&seport_button_event_lock);
}

static void seport_button_detect_cleanup(void)
{
	/* We can't lock the _generic_ mutex here since other functions
	 * are already holding it. If we try that, we get major deadlocks!
	 */
	seport_buttons_online = seport_vad_online = 0;

	if (waitqueue_active(&seport_btn_wq)) {
		wake_up_interruptible(&seport_btn_wq);
		seport_buttons_shutdown = 1;
	}

	seport_clear_button_list();

}


/*
 * @TODO: Disable button handling if kalloc fails, and report error.
 */
static void seport_gpio_buttondetect_work(struct work_struct *work)
{
	int err;
	u16 btn;
	u8 i, count;
	u8 button_filter[NUM_VALID_BUTTON_IDS];
	struct seport_button_event *ev;

	int hp_mode = seport_get_high_power_mode();
	memset(button_filter, 0, ARRAY_SIZE(button_filter));

	/* To avoid false interrupts during shutdown. */
	if (!seport_buttons_online)
		goto release_wake_lock;

	/* Delaying 4ms, according to spec, to make sure that we have a
	 * correct measurement. */
	msleep(4);

	for (count = 0; count < NUM_BUTTON_READS; count++) {
		err = seport_platform_get_button_id(&btn);
		seport_attrs_parse_button_value(&btn, hp_mode);
		if (btn < ARRAY_SIZE(button_filter))
			button_filter[btn]++;
	}

	/* Determining if any button is detected at least
	 * MIN_VALID_BUTTON_EVENT times. */
	for (i = 0; i < ARRAY_SIZE(button_filter); i++) {
		if (button_filter[i] >= MIN_VALID_BUTTON_EVENTS) {
			btn = (u16) i;
			break;
		}
	}

	if (i < ARRAY_SIZE(button_filter)) {
		ev = kmalloc(sizeof(struct seport_button_event), GFP_KERNEL);
		if (NULL == ev) {
			printk(KERN_ERR "%s: Failed to allocate memory. Shutting down button detection\n",
			       __func__);
			goto release_wake_lock;
		}

		ev->pressed = 1;
		seport_last_btn_id = ev->button = btn;
		ev->tm = current_kernel_time();
		ev->high_power = hp_mode;

		if (-1 == seport_add_button_event(ev)) {
			/* @todo: Disable button handling if kalloc  fails. */
			printk(KERN_ERR "%s: Failed to add event to list. Aborting",
			       __func__);
			kfree(ev);
			goto release_wake_lock;
		}

		/* Button release needs to be handled in a little loop.
		   Loop is exited if plug or button is released.
		   At least for the moment...
		*/
		do {
			seport_platform_get_button_id(&btn);
			msleep(10);
		} while (btn < 250 && seport_buttons_online);

		/*
		 * If we got stuck in the while loop, we will exit when
		 * headset is removed. In that case, we still must send a
		 * release event in order for AccessoryServer to get
		 * correctly reset.
		 */
		if (!seport_buttons_online)
			printk(KERN_INFO "Button release aborted du to external reason. Sending notification\n");
		else
			printk(KERN_INFO "%s: Button released\n",
			       __func__);

		ev = kmalloc(sizeof(struct seport_button_event), GFP_KERNEL);
		if (NULL == ev) {
			printk(KERN_ERR "%s: Failed to allocate memory. " \
			       "Shutting down button detection\n",
			       __func__);
			goto release_wake_lock;
		}
		ev->pressed = 0;
		ev->button = seport_last_btn_id;
		ev->tm = current_kernel_time();
		ev->high_power = seport_get_high_power_mode();

		if (-1 == seport_add_button_event(ev)) {
			/* @todo: Disable button handling if kalloc  fails. */
			printk(KERN_ERR "%s: Failed to add event to list. Stopping button detection\n",
			       __func__);
			kfree(ev);
			return;
		}

	} else
		printk(KERN_INFO "%s: No change in default button state or valid button detected\n",
		       __func__);

	seport_platform_enable_button_detect_interrupt();

release_wake_lock:
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&seport_btn_wake_lock);
#endif
}

static int seport_gpio_buttondetect_callback(int irq, void *data)
{
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&seport_btn_wake_lock);
#endif
	seport_platform_disable_button_detect_interrupt();
	queue_work(seport_btn_det_work, &seport_ain_work);

	return IRQ_HANDLED;
}

static void seport_vad_buttondetect_work(struct work_struct *work)
{
	struct seport_button_event *ev;
	struct seport_plug_detect_data *vdat;

	vdat = container_of(work, struct seport_plug_detect_data, local_work);
	if (!vdat) {
		printk(KERN_INFO "*** %s - Failed to retrieve data structure." \
		       " Aborting\n", __func__);
		return;
	}

	ev = kmalloc(sizeof(struct seport_button_event), GFP_KERNEL);
	if (NULL == ev) {
		printk(KERN_ERR "%s: Failed to allocate memory. " \
		       " Shutting down button detection\n",
		       __func__);
		return;
	}

	ev->pressed = !atomic_read(&vdat->status);
	ev->button = BUTTON_VAD;
	ev->tm = current_kernel_time();
	ev->high_power = seport_get_high_power_mode();

	if (-1 == seport_add_button_event(ev)) {
		/* @todo: Disable button handling if kalloc  fails. */
		printk(KERN_ERR "%s: Failed to add event to list.",
		       __func__);
		kfree(ev);
	}

}

static int seport_vad_buttondetect_callback(int irq, void *data)
{
	int i;
	struct seport_plug_detect_data *dt = data;
	i = queue_work(seport_vadsense_wq, &dt->local_work);

	return IRQ_HANDLED;
}

/* Public functions */
int seport_enable_button_detection(int enable)
{
	int retval = -1;

	mutex_lock(&seport_button_generic_lock);

	if (enable && !seport_buttons_online) {
		seport_buttons_shutdown = 0;
		retval = seport_platform_register_button_gpio_callback(
			seport_gpio_buttondetect_callback,
			NULL);
		if (0 == retval) {
			seport_buttons_online = 1;
		} else {
			retval = ERROR_NO_AIN_ACCESSORY;
			seport_buttons_online = 0;
		}

	} else if (enable && seport_buttons_online) {
		retval = 0;
	} else if (seport_buttons_online) {
		seport_buttons_online = 0;
		seport_platform_disable_button_detect_interrupt();
		seport_platform_unregister_button_gpio_callback();
		retval = 0;
	}

	if (!seport_vad_online && !seport_buttons_online)
		seport_button_detect_cleanup();

	mutex_unlock(&seport_button_generic_lock);

	return retval;
}

int seport_enable_vad_button_detection(int enable)
{
	int retval = -1;

	mutex_lock(&seport_button_generic_lock);

	if (enable && !seport_vad_online) {
		seport_buttons_shutdown = 0;
		retval = seport_platform_register_vad_button_callback(
			seport_vad_buttondetect_callback,
			&vad_det_data);
		seport_vad_online = 1;
	} else if (enable && seport_vad_online) {
		retval = 0;
	} else if (seport_vad_online) {
		seport_platform_unregister_vad_button_callback();
		seport_vad_online = 0;
		retval = 0;
	}

	/* If nothing wants any information, release the sysytem */
	if (!seport_vad_online && !seport_buttons_online)
		seport_button_detect_cleanup();

	mutex_unlock(&seport_button_generic_lock);

	return retval;
}

int seport_button_detect_release(struct inode *inode, struct file *filp)
{
	seport_enable_vad_button_detection(0);
	seport_enable_button_detection(0);
	return 0;
}


unsigned int seport_button_detect_poll(struct file *filp,
				       struct poll_table_struct *wait)
{
	int err = 0;

	poll_wait(filp, &seport_btn_wq, wait);

	mutex_lock(&seport_button_generic_lock);

	if (seport_buttons_shutdown) {
		seport_buttons_shutdown = 0;
		err =  -1; /* TODO: Change return value to kernel standard */
	}

	if (!err) {
		if (button_events_available())
			err = (POLLIN | POLLRDNORM);
	}

	mutex_unlock(&seport_button_generic_lock);

	return err;
}

/*
 * @TODO:
 * Update this to handle more than one client. Later problem.
 * We only support one client to start with! */
ssize_t seport_button_detect_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *f_pos)
{
	int retval = -EFAULT;
	struct seport_button_event *bevent = NULL;
	struct seport_button_event *ev = (struct seport_button_event *)buf;

	if (NULL == ev || count != sizeof(struct seport_button_event)) {
		printk(KERN_ALERT "button_detect_read: User supplied pointer " \
		       "not valid.\n");
		goto read_error;
	}

	/* Making sure that we only have one reader reading here! */
	mutex_lock(&seport_button_generic_lock);
	bevent = seport_get_next_button_event();

	if (NULL == bevent) {
		printk(KERN_ALERT "button_detect_read: Failed to retrieve "
		       "valid button event.\n");
		goto read_error;
	}

	ev->tm.tv_sec = bevent->tm.tv_sec;
	ev->tm.tv_nsec = bevent->tm.tv_nsec;
	ev->pressed = bevent->pressed;
	ev->button = bevent->button;
	ev->high_power = bevent->high_power;

	retval = sizeof(struct seport_button_event);

read_error:
	if (NULL != bevent)
		kfree(bevent);
	mutex_unlock(&seport_button_generic_lock);
	return retval;

}

/* @TODO:
 * Update this to handle several clients. Later problem. We will only have one
 * in the first stage.*/
int seport_button_detect_open(struct inode *inode, struct file *filp)
{
	struct seport_plug_detect_data *d = &vad_det_data;

	init_waitqueue_head(&seport_btn_wq);

	seport_vadsense_wq = create_singlethread_workqueue("seport_vad_det_wq");
	if(!seport_vadsense_wq) {
		printk(KERN_ERR "%s - Failed to allocate VAD button workqueue"
		       ". Out of memory\n", __func__);
		return -ENOMEM;
	}

	seport_btn_det_work = create_singlethread_workqueue("seport_btn_det_wq");
	if(!seport_btn_det_work) {
		printk(KERN_ERR "%s - Failed to create AIN workqueue. Out of"
		       "memory\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&d->local_work, seport_vad_buttondetect_work);
	INIT_WORK(&seport_ain_work, seport_gpio_buttondetect_work);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&seport_btn_wake_lock, WAKE_LOCK_SUSPEND,
		       "seport_btn_det_lock");
#endif
	return 0;
}
