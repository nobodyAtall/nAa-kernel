/* drivers/misc/semc/seport/seport_button_detect.h
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

#ifndef _SEPORT_BUTTON_DETECT_H_
#define _SEPORT_BUTTON_DETECT_H_

#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

ssize_t      seport_button_detect_read(struct file *filp, char __user *buf,
				       size_t count, loff_t *f_pos);

unsigned int seport_button_detect_fspoll(struct file *filp, poll_table *wait);

int          seport_enable_button_detection(int enable);
int          seport_enable_vad_button_detection(int enable);
int          seport_button_detect_release(struct inode *inode,
					  struct file *filp);
int          seport_button_detect_open(struct inode *inode, struct file *filp);
unsigned int seport_button_detect_poll(struct file *filp,
				       struct poll_table_struct *wait);

#endif /* _SEPORT_BUTTON_DETECT_H_ */
