/*
 * SEMC Class Core
 *
 * Copyright (C) 2009 SonyEricsson AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 */
#ifndef __SEMCCLASS_H_INCLUDED
#define __SEMCCLASS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/semcclass.h>

extern struct rw_semaphore semcclass_list_lock;
extern struct list_head    semcclass_list;

#endif	/* __SEMCCLASS_H_INCLUDED */
