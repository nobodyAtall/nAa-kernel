/*
 * Driver model for leds and led triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_SEMCCLASS_H_INCLUDED
#define __LINUX_SEMCCLASS_H_INCLUDED

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>

struct device;
/*
 * SEMC class Core
 */

/*
*   How to set attributes:
*   
*		static struct device_attribute my_attributes[] = {
*			__ATTR(attr_name1, 0644, attr1_show, attr1_store)   	
*			__ATTR(attr_name2, 0644, attr2_show, attr2_store)   	
*			...
*			__ATTR(attr_namen, 0644, attrn_show, attrn_store)   	
* 	};  
*   
*   struct semc_classdev my_classdev;
*   my_classdev.device_attribute = my_attributes;
*   my_classdev.attr_number = sizeof(my_attributes)/sizoef(struct device_attribute);
*   
*   ssize_t attr1_store (struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
*   ssize_t attr1_show  (struct device *dev, struct device_attribute *attr, char *buf);
*/


struct semc_classdev {
	const char		*name;

	struct device		   *dev;
	struct list_head	 node;			/* LED Device list */

  struct device_attribute* attributes;  /* attributes array */
  int attr_number;                        /* number of attributes*/
};

extern int  semc_classdev_register(struct device *parent, struct semc_classdev *semc_cdev);
extern void semc_classdev_unregister(struct semc_classdev *semc_cdev);
int semc_classdev_read_interface(const char *buf, int size, u32 *value, u8 base);


#endif		/* __LINUX_SEMCCLASS_H_INCLUDED */

/*
struct attribute {
        const char              *name;
        struct module           *owner;
        mode_t                  mode;
};
*/


