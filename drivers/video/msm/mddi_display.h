/* drivers/video/msm/mddi_display.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Joakim Wesslén <joakim.wesslen@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#if !defined(__MDDI_DISPLAY_H__)
#define __MDDI_DISPLAY_H__

/* Debug */
#define DBG(klevel, level, string, args...) \
do { \
 if (dbg_lvl >= level) \
 printk(klevel string, ##args); \
} while(0);

/* MDDI Write register functions */
/* Byte order,  word0=P4P3P2P1, little endian */
#define write_reg(__X,__Y) \
do { \
  mddi_queue_register_write(__X,__Y,TRUE,0); \
} while(0);

#define write_reg_16(__X, __Y0, __Y1, __Y2, __Y3, __NBR) \
do { \
 mddi_host_register_write16(__X, __Y0, __Y1, __Y2, __Y3, __NBR, \
	TRUE, NULL, MDDI_HOST_PRIM); \
} while(0);

#define write_reg_xl(__X, __Y, __NBR) \
do { \
 mddi_host_register_write_xl(__X, __Y, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM); \
} while(0);

/* Debug levels */
#define LEVEL_QUIET 0
#define LEVEL_DEBUG 1
#define LEVEL_TRACE 2
#define LEVEL_PARAM 3

/* Dynamic backlight control */
#define DBC_OFF 0
#define DBC_ON  1
#define DBC_MODE_UI    1
#define DBC_MODE_IMAGE 2
#define DBC_MODE_VIDEO 3

/* Display power */
#define POWER_OFF 0
#define POWER_ON  1

/* Enums */
enum mddi_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

struct panel_ids_t {
	u32 driver_ic_id;
	u32 cell_id;
	u32 module_id;
	u32 revision_id;
};


#endif /* __MDDI_DISPLAY_H__ */

