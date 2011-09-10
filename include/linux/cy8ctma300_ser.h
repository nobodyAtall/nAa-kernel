/* include/linux/cy8ctma300_ser.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Alexandar Rodzevski <alexandar.rodzevski@sonyericsson.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#ifndef __CY8CTMA300_SER_H__
#define __CY8CTMA300_SER_H__

/* Device name */
#define CY8CTMA300_SER_DEV "cy8ctma300_ser"

/* Register addresses */
/* Only registers used will defined at this time */
#define REG_STATUS (0x02)
#define REG_POS_X1 (0x03)
#define REG_POS_Y1 (0x05)
#define REG_POS_Z1 (0x07)

/* Various defines (to be moved into cy8ctma300 header) */
#define CY8CTMA300_TEMPLATE 1
#define CONFIG_TOUCHSCREEN_CY8CTMA300_I2C 1
#define CONFIG_TOUCHSCREEN_CY8CTMA300_SPI 0
#define CONFIG_TOUCHSCREEN_CY8CTMA300_UART 0
#define	MAX_12BIT			((1<<12)-1)
#define	TOUCHSCREEN_TIMEOUT		(msecs_to_jiffies(50))
/* Various flags needed */
#define CY8F_XY_AXIS_FLIPPED (0x01)

#define GET_NUM_FINGERS(X) ((X) &0x0F)
#define IS_LARGE_AREA(X) (((X) & 0x10) >> 4)
#define FLIP_DATA(X) ((X) && 0x01)



/* CY8CTMA300 private data
 *
 * Still to be added:
 * Registers for Power management.
 */

struct cy8ctma300_ser_platform_data {
  int (*xres)(void);
  u32 maxx;
  u32 maxy;
  u32 flags;
};

#endif
