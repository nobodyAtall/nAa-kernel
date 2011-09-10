/* drivers/misc/semc/seport/seport_plug_detect.h
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

#ifndef _SEPORT_PLUG_DETECT_H_
#define _SEPORT_PLUG_DETECT_H_

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/interrupt.h>

int seport_plug_detect_open(struct inode *inode, struct file *filp);

unsigned int seport_plug_detect_fspoll(struct file *filp,
				       poll_table *wait);

ssize_t seport_plug_detect_read(struct file *filp, char __user *buf,
				size_t count, loff_t *f_pos);

int seport_plug_detect_restart(void);

void seport_plug_detect_cleanup(void);

int seport_plug_detect_get_status(void);

int seport_plug_detect_release(struct inode *inode, struct file *filp);

unsigned int seport_plug_detect_poll(struct file *filp,
				     struct poll_table_struct *wait);

#endif /* _SEPORT_PLUG_DETECT_H_ */
