/* drivers/video/msm/mddi_hitachi_qvga.c
 *
 * MDDI client driver for the Hitachi QVGA display
 * with driver IC Samsung S6D04D1X11.
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <mach/gpio.h>
#include <mach/vreg.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/swab.h>
#include <mach/gpio.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <mach/board.h>
#include <linux/mutex.h>
#include <linux/autoconf.h>
#include "mddi_display.h"

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0017

/* Display CELL ID value */
#define MDDI_HITACHI_QVGA_CELL_ID 0xFD

/* Debug setup */
#define DBG_STR "MDDI: Hitachi QVGA: "

/* Frame time, used for delays */
#define MDDI_FRAME_TIME 5

/* ESD recovery setup */
//#define ESD_RECOVERY_SUPPORT
#define ESD_POLL_TIME_MS 2000
#define ESD_FAILURE_CHECK_AGAIN_TIME_MS 100
#define ESD_FAILURE_NUMBER_MAX 3

enum lcd_registers_t {
	LCD_REG_COLUMN_ADDRESS = 0x2A,
	LCD_REG_PAGE_ADDRESS = 0x2B,
	LCD_REG_CELL_ID = 0xDA,
	LCD_REG_MODULE_ID = 0xDB,
	LCD_REG_REVISION_ID = 0xDC
};

/* Function Configuration */
static int dbc_ctrl = DBC_ON;
static int dbc_mode = DBC_MODE_VIDEO;
static int power_ctrl = POWER_OFF;
static int dbg_lvl = LEVEL_QUIET;

/* Variable declarations */
static enum mddi_lcd_state lcd_state = LCD_STATE_OFF;
static DEFINE_MUTEX(mddi_mutex);
static DEFINE_MUTEX(hitachi_panel_ids_lock);

static struct lcd_data_t {
#ifdef ESD_RECOVERY_SUPPORT
	struct delayed_work esd_check;
	struct platform_device *pdev;
	int failure_counter;
#endif /* ESD_RECOVERY_SUPPORT */
	struct {
		u16 x1;
		u16 x2;
		u16 y1;
		u16 y2;
	} last_window;
} lcd_data;

static struct panel_ids_t panel_ids;

/* Function prototypes */
#ifdef ESD_RECOVERY_SUPPORT
static void esd_recovery_resume(void);
#endif

/* Kernel Module setup */
module_param(dbc_ctrl, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dbc_ctrl, "Dynamic Backlight Control DBC_OFF = 0, DBC_ON = 1");

module_param(dbg_lvl, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dbg_lvl,
		"Debug level QUIET = 0, DEBUG = 1, TRACE = 2, PARAM = 3");

/* Sysfs */
static ssize_t show_driver_info(struct device *pdev,
			struct device_attribute *attr,
			char *buf);
static ssize_t show_dbc_ctrl(struct device *pdev,
			struct device_attribute *attr,
			char *buf);
static ssize_t store_dbc_ctrl(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count);
static ssize_t show_dbc_mode(struct device *pdev,
			struct device_attribute *attr,
			char *buf);
static ssize_t store_dbc_mode(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count);
static ssize_t show_power_ctrl(struct device *pdev,
			struct device_attribute *attr,
			char *buf);
static ssize_t store_power_ctrl(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count);
static ssize_t show_dbg_lvl(struct device *pdev,
			struct device_attribute *attr,
			char *buf);
static ssize_t store_dbg_lvl(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count);

static DEVICE_ATTR(display_driver_info, 0444, show_driver_info, NULL);
static DEVICE_ATTR(dbc_ctrl, 0644, show_dbc_ctrl, store_dbc_ctrl);
static DEVICE_ATTR(dbc_mode, 0644, show_dbc_mode, store_dbc_mode);
static DEVICE_ATTR(power_ctrl, 0644, show_power_ctrl, store_power_ctrl);
static DEVICE_ATTR(dbg_lvl, 0644, show_dbg_lvl, store_dbg_lvl);


/* ----- Driver functions ----- */

static void hitachi_qvga_write_dbc_mode(int mode)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	write_reg_16(0x55, mode, 0, 0, 0, 1)
}

static void hitachi_lcd_dbc_on(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (dbc_ctrl) {
		DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"dbc_ctrl = %d\n", dbc_ctrl);

		/* Manual brightness */
		write_reg_16(0x51, 0x000000FF, 0, 0, 0, 1);
		/* Mobile Image Enhancement Mode */
		hitachi_qvga_write_dbc_mode(dbc_mode);
		/* Minimum Brightness */
		write_reg_16(0x5E, 0x00000000, 0, 0, 0, 1);
		/* Mobile Image Enhancement Control 1 */
		write_reg_16(0xCA, 0x00104040, 0, 0, 0, 1);
		/* BL Control Mode */
		write_reg_16(0xCB, 0x00000002, 0, 0, 0, 1);
		/* Mobile Image Enhancement Control 2 */
		write_reg_16(0xCC, 0x003F0120, 0x000000EF, 0, 0, 2);
		/* BL Control */
		write_reg_16(0xCD, 0x00001645, 0, 0, 0, 1);
		/* BL Control */
		write_reg_16(0x53, 0x00000024, 0, 0, 0, 1);
	}
}

static void hitachi_lcd_dbc_off(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (dbc_ctrl) {
		DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"dbc_ctrl = %d\n", dbc_ctrl);
		/* BL Control */
		write_reg_16(0x53, 0x00000000, 0, 0, 0, 1);
	}
}

static void hitachi_lcd_window_address_set(enum lcd_registers_t reg,
						u16 start, u16 stop)
{
	uint32 para;

	para = start;
	para = (para << 16) | (start + stop);
	para = swab32 (para);
	write_reg_16 (reg, para, 0, 0, 0, 1);
	if (reg == LCD_REG_COLUMN_ADDRESS) {
		lcd_data.last_window.x1 = start;
		lcd_data.last_window.x2 = stop;
	} else {
		lcd_data.last_window.y1 = start;
		lcd_data.last_window.y2 = stop;
	}
}

static void hitachi_lcd_driver_init(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;
	u32 color_depth;
	static u32 display_init_flag = 0;

	if (pdev && pdev->dev.platform_data) {
		panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

		DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n",
			__func__, lcd_state);

		/* VCOM */
		write_reg_16(0xF4, 0x48485959, 0x00000011, 0, 0, 2);
		/* Src Output Ctrl */
		write_reg_16(0xF5, 0xF00B0012, 0x00001A00, 0, 0, 2);
		mddi_wait(11); /* spec: > 10 ms */

		/* Power Control */
		write_reg_16(0xF3, 0x072A2AFF, 0x20666622, 0, 0, 2);
		/* Display Control */
		write_reg_16(0xF2,
				0x08001313, 0x00080808, 0x001A1A04, 0, 3);

		/* Red positive gamma */
		write_reg_16(0xF7, 0x01000505, 0x3D332B17,
				0x13160303, 0x00222207, 4);
		/* Red negative gamma */
		write_reg_16(0xF8, 0x01000505, 0x3D332B17,
				0x13160303, 0x00222207, 4);

		/* Gamma settings for:
		   Robyn Hitachi Module ID 0x02, Revision ID 0x20
		   Mimmi Hitachi Module ID 0x03, Revision ID 0x10 */
		if ((((panel_ids.module_id & 0xFF) == 0x02) &&
		     ((panel_ids.revision_id & 0xFF) == 0x20)) ||
			(((panel_ids.module_id & 0xFF) == 0x03) &&
		     ((panel_ids.revision_id & 0xFF) == 0x10))) {
			/* Green positive gamma */
			write_reg_16(0xF9, 0x03000505, 0x3B312814,
						0x15170505, 0x00222107, 4);
			/* Green negative gamma */
			write_reg_16(0xFA, 0x03000505, 0x3B312814,
						0x15170505, 0x00222107, 4);
			/* Blue Positive gamma */
			write_reg_16(0xFB, 0x14000505, 0x3F38321D,
						0x13140000, 0x00228406, 4);
			/* Blue Negative gamma */
			write_reg_16(0xFC, 0x14000505, 0x3F38321D,
						0x13140000, 0x00228406, 4);
		} else {
			/* Green positive gamma */
			write_reg_16(0xF9, 0x01000505, 0x3D332B17,
					     0x13160303, 0x00222207, 4);
			/* Green negative gamma */
			write_reg_16(0xFA, 0x01000505, 0x3D332B17,
					     0x13160303, 0x00222207, 4);
			/* Blue Positive gamma */
			write_reg_16(0xFB, 0x01000505, 0x3D332B17,
					     0x13160303, 0x00222207, 4);
			/* Blue Negative gamma */
			write_reg_16(0xFC, 0x01000505, 0x3D332B17,
					     0x13160303, 0x00222207, 4);
		}

		/* Gate Control */
		write_reg_16(0xFD, 0x00000211, 0, 0, 0, 1);
		/* Memory data Access Control */
		write_reg_16(0x36, 0x00000048, 0, 0, 0, 1);
		/* Tearing effect line on */
		write_reg_16(0x35, 0x00000000, 0, 0, 0, 1);
		/* Interface Pixel Format*/
		if (panel->panel_info.bpp == 16) {
			color_depth = 0x00000055;
		} else if (panel->panel_info.bpp == 18) {
			color_depth = 0x00000066;
		} else {
			/* assuming 24 bpp */
			color_depth = 0x00000077;
		}
		write_reg_16(0x3A, color_depth, 0, 0, 0, 1);
		/* Page Address Set */
		hitachi_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS,
					       0, panel->panel_info.xres - 1);
		hitachi_lcd_window_address_set(LCD_REG_PAGE_ADDRESS,
					       0, panel->panel_info.yres - 1);
		if(!display_init_flag){
			/* Replace display internal random data with black pixels */
			mddi_video_stream_black_display(0, 0, panel->panel_info.xres,
					panel->panel_info.yres, MDDI_HOST_PRIM);
			/*Hide garbage on screen when the phone power on in the first*/
			display_init_flag = 1;
		}
	}
}

static void hitachi_lcd_window_adjust(uint16 x1, uint16 x2,
				      uint16 y1, uint16 y2)
{
	DBG(KERN_INFO, LEVEL_PARAM,
		DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (lcd_data.last_window.x1 != x1 || lcd_data.last_window.x2 != x2) {
		DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (column) [%d, %d]\n",
				   __func__, x1, x2);
		hitachi_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
	}

	if (lcd_data.last_window.y1 != y1 || lcd_data.last_window.y2 != y2) {
		DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (page) [%d, %d]\n",
				   __func__, y1, y2);
		hitachi_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);
	}

	mutex_unlock(&mddi_mutex);
}

#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
static void hitachi_lcd_enter_sleep(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	/* Display off */
	write_reg_16(0x28, 0, 0, 0, 0, 1);
	mddi_wait(2*MDDI_FRAME_TIME); /* > 2 frames */
	/* Sleep in */
	write_reg_16(0x10, 0, 0, 0, 0, 1);
	mddi_wait(121); /* spec: > 120 ms */
}
#endif

static void hitachi_lcd_exit_sleep(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	/* TestKey */
	write_reg_16(0xF1, 0x0000005A, 0, 0, 0, 1);
	/* Logic TestReg 2 */
	write_reg_16(0xFF, 0x40000000, 0, 0, 0, 1);
	/* Sleep out */
	write_reg_16(0x11, 0x00000000, 0, 0, 0, 1);
	mddi_wait(121); /* spec: > 120 ms */
	/* TestKey */
	write_reg_16(0xF1, 0x00000000, 0, 0, 0, 1);
	/* Display on */
	write_reg_16(0x29, 0x00000000, 0, 0, 0, 1);
}

#ifndef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
static void hitachi_lcd_enter_deepstandby(void)
{
	DBG(KERN_INFO, LEVEL_TRACE,
		"%s [%d]\n", __func__, lcd_state);
	/* Display off */
	write_reg_16(0x28, 0x00000000, 0, 0, 0, 1);
	mddi_wait(2*MDDI_FRAME_TIME); /* > 2 frames */
	/* MDDI Control */
	write_reg_16(0xB0, 0x00000001, 0, 0, 0, 1);
}

static void hitachi_toggle_reset(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

		if (panel && panel->panel_ext->exit_deep_standby)
			panel->panel_ext->exit_deep_standby();
	}
}

static void hitachi_lcd_exit_deepstandby(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE,
		"%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		/* Reset toggle */
		hitachi_toggle_reset(pdev);
		/* Re-Initialize */
		hitachi_lcd_driver_init(pdev);
	}
}
#endif

static void hitachi_power_on(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;
		if (panel && panel->panel_ext->power_on)
			panel->panel_ext->power_on();
	}
}

static void hitachi_power_off(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (pdev) {
		panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;
		if (panel && panel->panel_ext->power_off)
			panel->panel_ext->power_off();
	}
}

static int mddi_hitachi_lcd_on(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	if (power_ctrl) {
		switch (lcd_state) {
		case LCD_STATE_OFF:
			hitachi_power_on(pdev);
			lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			hitachi_lcd_driver_init(pdev);
			hitachi_lcd_exit_sleep();
			hitachi_lcd_dbc_on();
			lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_SLEEP:
#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
			hitachi_power_on(pdev);
			hitachi_lcd_driver_init(pdev);
#else
			hitachi_lcd_exit_deepstandby(pdev);
#endif
			hitachi_lcd_exit_sleep();
			hitachi_lcd_dbc_on();
			lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_ON:
			break;

		default:
			break;
		}
	}
#ifdef ESD_RECOVERY_SUPPORT
	if (lcd_state == LCD_STATE_ON)
		esd_recovery_resume();
#endif
	mutex_unlock(&mddi_mutex);
	return 0;
}


static int mddi_hitachi_lcd_off(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	if (power_ctrl) {
		switch (lcd_state) {
		case LCD_STATE_POWER_ON:
			hitachi_power_off(pdev);
			lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			hitachi_lcd_dbc_off();
#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
			hitachi_lcd_enter_sleep();
			hitachi_power_off(pdev);
#else
			hitachi_lcd_enter_deepstandby();
#endif
			lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			hitachi_power_off(pdev);
			lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_OFF:
			break;

		default:
			break;
		}
	}
	mutex_unlock(&mddi_mutex);
#ifdef ESD_RECOVERY_SUPPORT
	cancel_delayed_work(&lcd_data.esd_check);
#endif
	return 0;
}

static int check_panel_ids(void)
{
	int ret;

	mutex_lock(&hitachi_panel_ids_lock);

	ret = mddi_host_register_read(LCD_REG_CELL_ID, &panel_ids.cell_id,
				      1, MDDI_HOST_PRIM);
	if (((panel_ids.cell_id & 0xFF) != MDDI_HITACHI_QVGA_CELL_ID) ||
								(ret < 0)) {
		mutex_unlock(&hitachi_panel_ids_lock);
		return -1;
	}

	/* LCD_REG_DRIVER_IC_ID not used in Hitachi */
	panel_ids.driver_ic_id = 0xFF;

	ret = mddi_host_register_read(LCD_REG_MODULE_ID, &panel_ids.module_id,
							1, MDDI_HOST_PRIM);
	if (ret < 0) {
		printk(KERN_INFO DBG_STR"Failed to read LCD_REG_MODULE_ID\n");
		panel_ids.module_id = 0xFF;
	}
	ret = mddi_host_register_read(LCD_REG_REVISION_ID,
						&panel_ids.revision_id,
						1, MDDI_HOST_PRIM);
	if (ret < 0) {
		printk(KERN_INFO DBG_STR"Failed to read LCD_REG_REVISION_ID\n");
		panel_ids.revision_id = 0xFF;
	}

	mutex_unlock(&hitachi_panel_ids_lock);
	return 0;
}

#ifdef ESD_RECOVERY_SUPPORT
static int esd_failure_check(void)
{
	u32 id = 0;

	if (mddi_host_register_read(LCD_REG_CELL_ID, &id, 1, MDDI_HOST_PRIM)) {
		printk(KERN_INFO DBG_STR"MDDI read timeout/error\n");
		return 0;
	}
	id &= 0xff;
	/* During high MDDI bus activity, id can be 0 */
	if (id && id != MDDI_HITACHI_QVGA_CELL_ID) {
		printk(KERN_INFO DBG_STR"esd display ID  0x%02x wrong.\n", id);
		return -1;
	}
	return 0;
}

static void esd_recovery_func(struct work_struct *work)
{
	int timeout = msecs_to_jiffies(ESD_POLL_TIME_MS);

	mutex_lock(&mddi_mutex);
	if (lcd_state == LCD_STATE_ON) {
		if (esd_failure_check()) {
			if (++lcd_data.failure_counter >
					ESD_FAILURE_NUMBER_MAX) {
				printk(KERN_INFO DBG_STR
					"%s (ver:0x%x) ESD recovery started.\n",
					__func__, MDDI_DRIVER_VERSION);
				/*
				*  Recovery process: TBD
				*/
				hitachi_lcd_driver_init(lcd_data.pdev);
				hitachi_lcd_dbc_on();

				printk(KERN_INFO DBG_STR
					"%s (ver:0x%x) ESD recovery finished\n",
					__func__, MDDI_DRIVER_VERSION);
				lcd_data.failure_counter = 0;
			} else {
				timeout = msecs_to_jiffies(
					ESD_FAILURE_CHECK_AGAIN_TIME_MS);
			}
		} else {
			lcd_data.failure_counter = 0;
		}
		schedule_delayed_work(&lcd_data.esd_check, timeout);
	}
	mutex_unlock(&mddi_mutex);
}

static void esd_recovery_init(struct platform_device *pdev)
{
	lcd_data.pdev = pdev;
	lcd_data.failure_counter = 0;
	INIT_DELAYED_WORK(&lcd_data.esd_check, esd_recovery_func);
}

static void esd_recovery_resume(void)
{
	lcd_data.failure_counter = 0;
	schedule_delayed_work(&lcd_data.esd_check, ESD_POLL_TIME_MS);
}
#endif /* ESD_RECOVERY_SUPPORT */


/* --- Sysfs --- */

static ssize_t show_driver_info(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	return snprintf(buf, PAGE_SIZE, "%s cell ID = 0x%x, "
				"module ID = 0x%x, revision ID = 0x%x, "
				"driver IC ID = 0x%x, driver ID = 0x%x\n",
				DBG_STR,
				panel_ids.cell_id & 0xFF,
				panel_ids.module_id & 0xFF,
				panel_ids.revision_id & 0xFF,
				panel_ids.driver_ic_id & 0xFF,
				MDDI_DRIVER_VERSION);
}

static ssize_t show_dbc_ctrl(struct device *pdev,
			struct device_attribute *attr,
			char *buf)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	return snprintf(buf, PAGE_SIZE, "%i\n", dbc_ctrl);
}

static ssize_t store_dbc_ctrl(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		printk(KERN_ALERT DBG_STR"%s Invalid flag for dbc ctrl\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	if (ret)
		dbc_ctrl = DBC_ON;
	else
		dbc_ctrl = DBC_OFF;

	DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"%s dbc_ctrl set to %d\n",
			__func__, dbc_ctrl);
	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static ssize_t show_dbc_mode(struct device *pdev,
			struct device_attribute *attr,
			char *buf)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	return snprintf(buf, PAGE_SIZE, "%i\n", dbc_mode);
}

static ssize_t store_dbc_mode(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (lcd_state != LCD_STATE_ON) {
		printk(KERN_ALERT DBG_STR"%s: LCD in sleep. "
			"Do not perform any register commands!\n", __func__);
		ret = -EINVAL;
		goto unlock;
	}

	if (sscanf(buf, "%i", &ret) != 1) {
		printk(KERN_ALERT DBG_STR"%s Invalid flag for dbc mode\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	switch (ret) {
	case DBC_MODE_UI:
	case DBC_MODE_IMAGE:
	case DBC_MODE_VIDEO:
		dbc_mode = ret;
		break;
	default:
		printk(KERN_ALERT DBG_STR"%s Invalid value for dbc mode\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	hitachi_qvga_write_dbc_mode(dbc_mode);

	DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"%s dbc_mode set to %d\n",
			__func__, dbc_mode);

	ret = strnlen(buf, count);
unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static ssize_t show_power_ctrl(struct device *pdev,
			struct device_attribute *attr,
			char *buf)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	return snprintf(buf, PAGE_SIZE, "%i\n", power_ctrl);
}

static ssize_t store_power_ctrl(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;
	struct platform_device *pf_dev = NULL;

	pf_dev = container_of(pdev, struct platform_device, dev);
	if (!pf_dev) {
		printk(KERN_ERR DBG_STR"%s pf_dev == NULL\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		printk(KERN_ALERT DBG_STR"%sInvalid flag for power_ctrl\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	if (ret) {
		power_ctrl = POWER_ON;
		hitachi_power_on(pf_dev);
		hitachi_lcd_driver_init(pf_dev);
#ifndef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
		hitachi_lcd_exit_deepstandby(pf_dev);
#endif
		hitachi_lcd_exit_sleep();
		hitachi_lcd_dbc_on();
		lcd_state = LCD_STATE_ON;
#ifdef ESD_RECOVERY_SUPPORT
		esd_recovery_resume();
#endif
	} else {
		power_ctrl = POWER_OFF;
		hitachi_lcd_dbc_off();
#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
		hitachi_lcd_enter_sleep();
#else
		hitachi_lcd_enter_deepstandby();
#endif
		hitachi_power_off(pf_dev);
		lcd_state = LCD_STATE_SLEEP;
#ifdef ESD_RECOVERY_SUPPORT
		cancel_delayed_work(&lcd_data.esd_check);
#endif
	}

	DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"%s power_ctrl set to %d\n",
			__func__, power_ctrl);
	ret = strnlen(buf, count);
unlock:
	mutex_unlock(&mddi_mutex);
error:
	return ret;
}

static ssize_t show_dbg_lvl(struct device *pdev,
			struct device_attribute *attr,
			char *buf)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	return snprintf(buf, PAGE_SIZE, "%i\n", dbg_lvl);
}

static ssize_t store_dbg_lvl(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		printk(KERN_ALERT "Invalid flag for debug\n");
		ret = -EINVAL;
		goto unlock;
	}

	switch (ret) {
	case LEVEL_QUIET:
	case LEVEL_DEBUG:
	case LEVEL_TRACE:
	case LEVEL_PARAM:
		dbg_lvl = ret;
		break;
	default:
		printk(KERN_ALERT DBG_STR"%sInvalid value for dbg_lvl\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"%s dbg_lvl set to %d\n",
			__func__, dbg_lvl);

	ret = strnlen(buf, count);
unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static void sysfs_attribute_register(struct platform_device *pdev)
{
	int ret;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	ret = device_create_file(&pdev->dev, &dev_attr_display_driver_info);
	if (ret != 0)
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" display_driver_version attributes (%d)\n",
			__func__, ret);

	ret = device_create_file(&pdev->dev, &dev_attr_dbc_ctrl);
	if (ret != 0)
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" dbc attributes (%d)\n", __func__, ret);

	ret = device_create_file(&pdev->dev, &dev_attr_dbc_mode);
	if (ret != 0)
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" dbc mode attributes (%d)\n", __func__, ret);

	ret = device_create_file(&pdev->dev, &dev_attr_power_ctrl);
	if (ret != 0)
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" power attributes (%d)\n", __func__, ret);

	ret = device_create_file(&pdev->dev, &dev_attr_dbg_lvl);
	if (ret != 0)
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" debug attributes (%d)\n", __func__, ret);
}

static int mddi_hitachi_qvga_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct msm_fb_panel_data *panel_data;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	if (!pdev) {
		printk(KERN_ERR DBG_STR"%s Display failed in probe\n",
			__func__);
		ret = -ENODEV;
		goto exit_point;
	}
	if (!pdev->dev.platform_data) {
		printk(KERN_ERR DBG_STR"%s Display failed in probe,"
			" no platform data\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}
	panel_data = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	if (!check_panel_ids()) {
		printk(KERN_INFO "%s Found display with cell ID = 0x%x, "
				"module ID = 0x%x, revision ID = 0x%x, "
				"driver IC ID = 0x%x, driver ID = 0x%x\n",
				DBG_STR, panel_ids.cell_id & 0xFF,
				panel_ids.module_id & 0xFF,
				panel_ids.revision_id & 0xFF,
				panel_ids.driver_ic_id & 0xFF,
				MDDI_DRIVER_VERSION);

		lcd_state = LCD_STATE_POWER_ON;
		power_ctrl = POWER_ON;

#ifdef ESD_RECOVERY_SUPPORT
		esd_recovery_init(pdev);
#endif
		panel_data->on  = mddi_hitachi_lcd_on;
		panel_data->off = mddi_hitachi_lcd_off;
		panel_data->panel_ext->window_adjust =
						hitachi_lcd_window_adjust;

		/* Add mfd on driver_data */
		msm_fb_add_device(pdev);

		sysfs_attribute_register(pdev);
		ret = 0;
	}
exit_point:
	return ret;
}

static int __devexit mddi_hitachi_qvga_lcd_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_display_driver_info);
	device_remove_file(&pdev->dev, &dev_attr_dbc_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbc_mode);
	device_remove_file(&pdev->dev, &dev_attr_power_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbg_lvl);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_hitachi_qvga_lcd_probe,
	.remove = __devexit_p(mddi_hitachi_qvga_lcd_remove),
	.driver = {
		.name   = "mddi_hitachi_qvga",
	},
};

static int __init mddi_hitachi_qvga_lcd_init(void)
{
	int ret;

	DBG(KERN_INFO, LEVEL_TRACE,  "%s (ver:0x%x) [%d]\n",
			__func__, MDDI_DRIVER_VERSION, lcd_state);

	ret = platform_driver_register(&this_driver);

	return ret;
}

static void __exit mddi_hitachi_qvga_lcd_exit(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("joakim.wesslen@sonyericsson.com");
MODULE_DESCRIPTION("MDDI implementation of the Hitachi QVGA display");

module_init(mddi_hitachi_qvga_lcd_init);
module_exit(mddi_hitachi_qvga_lcd_exit);
