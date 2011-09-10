/*
 * SEMC Class Core
 *
 * Copyright (C) 2009 SonyEricsson AB
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/leds.h>
#include "semcclass.h"

DECLARE_RWSEM(semcclass_list_lock);
EXPORT_SYMBOL_GPL(semcclass_list_lock);

LIST_HEAD(semcclass_list);
EXPORT_SYMBOL_GPL(semcclass_list);
