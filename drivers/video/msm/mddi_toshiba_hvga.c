/* drivers/video/msm/mddi_toshiba_hvga.c
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
#define MDDI_DRIVER_VERSION 0x0011

/* Display CELL ID value */
#define MDDI_TOSHIBA_HVGA_CELL_ID 0x5A

/* Debug setup */
#define DBG_STR "MDDI: Toshiba HVGA: "

/* ESD recovery setup */
#define ESD_RECOVERY_SUPPORT
#define ESD_CHECKSUM_VAL 0x01
#define ESD_POLL_TIME_MS 2000
#define ESD_FAILURE_CHECK_AGAIN_TIME_MS 100
#define ESD_FAILURE_NUMBER_MAX 3

enum lcd_registers {
	LCD_REG_CELL_ID = 0xA100,
	LCD_REG_MODULE_ID = 0xA101,
	LCD_REG_REVISION_ID = 0xA102
};

/* Function Configuration */
static int dbc_ctrl = DBC_ON;
static int dbc_mode = DBC_MODE_VIDEO;
static int power_ctrl = POWER_OFF;
static int dbg_lvl = LEVEL_QUIET;

/* Variable declarations */
static enum mddi_lcd_state lcd_state = LCD_STATE_OFF;
static DEFINE_MUTEX(mddi_mutex);
static DEFINE_MUTEX(toshiba_panel_ids_lock);

#ifdef ESD_RECOVERY_SUPPORT
static struct lcd_data_t {
	struct delayed_work esd_check;
	struct platform_device *pdev;
	int failure_counter;
} lcd_data;
#endif

static struct panel_ids_t panel_ids;

/* Function prototypes */
#ifdef ESD_RECOVERY_SUPPORT
static void esd_recovery_resume(void);
static void esd_recovery_init(struct platform_device *pdev);
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

static void gamma_r_ctrl(void) {
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);

	write_reg(0x2880, 0x0012);
	write_reg(0x2980, 0x001B);
	write_reg(0x2A80, 0x0030);
	write_reg(0x2B80, 0x0048);
	write_reg(0x2C80, 0x000F);
	write_reg(0x2D80, 0x0032);
	write_reg(0x2E80, 0x0060);
	write_reg(0x2F80, 0x004A);
	write_reg(0x3080, 0x0022);
	write_reg(0x3180, 0x0027);
	write_reg(0x3280, 0x0090);
	write_reg(0x3380, 0x001F);
	write_reg(0x3480, 0x0048);
	write_reg(0x3580, 0x006C);
	write_reg(0x3680, 0x0051);
	write_reg(0x3780, 0x008C);
	write_reg(0x3880, 0x0030);
	write_reg(0x3980, 0x0056);
	write_reg(0x3A80, 0x0029);
	write_reg(0x3B80, 0x004F);
	write_reg(0x3C80, 0x0073);
	write_reg(0x3D80, 0x00AE);
	write_reg(0x3E80, 0x0013);
	write_reg(0x3F80, 0x0037);
	write_reg(0x4080, 0x0060);
	write_reg(0x4180, 0x006F);
	write_reg(0x4280, 0x0018);
	write_reg(0x4380, 0x001D);
	write_reg(0x4480, 0x00B5);
	write_reg(0x4580, 0x001F);
	write_reg(0x4680, 0x004D);
	write_reg(0x4780, 0x0070);
	write_reg(0x4880, 0x00B7);
	write_reg(0x4980, 0x00CF);
	write_reg(0x4A80, 0x0064);
	write_reg(0x4B80, 0x006D);
	write_reg(0x4C80, 0x0012);
	write_reg(0x4D80, 0x001B);
	write_reg(0x4E80, 0x0030);
	write_reg(0x4F80, 0x0048);
	write_reg(0x5080, 0x0012);
	write_reg(0x5180, 0x0035);
	write_reg(0x5280, 0x0063);
	write_reg(0x5380, 0x0056);
	write_reg(0x5480, 0x0022);
	write_reg(0x5580, 0x0027);
	write_reg(0x5680, 0x0094);
	write_reg(0x5780, 0x001D);
	write_reg(0x5880, 0x0042);
	write_reg(0x5980, 0x006A);
	write_reg(0x5A80, 0x005A);
	write_reg(0x5B80, 0x0071);
	write_reg(0x5C80, 0x0030);
	write_reg(0x5D80, 0x0056);
	write_reg(0x5E80, 0x0029);
	write_reg(0x5F80, 0x004F);
	write_reg(0x6080, 0x008E);
	write_reg(0x6180, 0x00A5);
	write_reg(0x6280, 0x0015);
	write_reg(0x6380, 0x003D);
	write_reg(0x6480, 0x0062);
	write_reg(0x6580, 0x006B);
	write_reg(0x6680, 0x0018);
	write_reg(0x6780, 0x001D);
	write_reg(0x6880, 0x00A9);
	write_reg(0x6980, 0x001C);
	write_reg(0x6A80, 0x004A);
	write_reg(0x6B80, 0x006D);
	write_reg(0x6C80, 0x00B7);
	write_reg(0x6D80, 0x00CF);
	write_reg(0x6E80, 0x0064);
	write_reg(0x6F80, 0x006D);
	write_reg(0x7080, 0x0012);
	write_reg(0x7180, 0x0039);
	write_reg(0x7280, 0x0068);
	write_reg(0x7380, 0x007E);
	write_reg(0x7480, 0x0018);
	write_reg(0x7580, 0x003D);
	write_reg(0x7680, 0x0067);
	write_reg(0x7780, 0x006B);
	write_reg(0x7880, 0x0022);
	write_reg(0x7980, 0x0027);
	write_reg(0x7A80, 0x009B);
	write_reg(0x7B80, 0x000F);
	write_reg(0x7C80, 0x0024);
	write_reg(0x7D80, 0x0048);
	write_reg(0x7E80, 0x0099);
	write_reg(0x7F80, 0x00BE);
	write_reg(0x8080, 0x0064);
	write_reg(0x8180, 0x007A);
	write_reg(0x8280, 0x0005);
	write_reg(0x8380, 0x001B);
	write_reg(0x8480, 0x0041);
	write_reg(0x8580, 0x0066);
	write_reg(0x8680, 0x0037);
	write_reg(0x8780, 0x005B);
	write_reg(0x8880, 0x0070);
	write_reg(0x8980, 0x0064);
	write_reg(0x8A80, 0x0018);
	write_reg(0x8B80, 0x001D);
	write_reg(0x8C80, 0x0094);
	write_reg(0x8D80, 0x0018);
	write_reg(0x8E80, 0x0042);
	write_reg(0x8F80, 0x0067);
	write_reg(0x9080, 0x0081);
	write_reg(0x9180, 0x0097);
	write_reg(0x9280, 0x0046);
	write_reg(0x9380, 0x006D);
}

static void toshiba_lcd_driver_init(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;
	u32 color_depth;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);

	panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	/* memory data access control i.e which direction to refresh & write */
	write_reg(0x3600, 0);

	/* pixel format, 0x0077 = 24 bpp, 0x0066 = 18bpp, 0x0055 = 16bpp */
	if (panel->panel_info.bpp == 16) {
		color_depth = 0x0055;
	} else if (panel->panel_info.bpp == 18) {
		color_depth = 0x0066;
	} else {
		/* assuming 24 bpp */
		color_depth = 0x0077;
	}
	write_reg(0x3A00, color_depth);

	/* MDDI current output selection 1.0 */
	write_reg(0xAE00, 0);

	/* CMD2 unlock */
	write_reg(0xF300, 0x00AA);

	/* LCD settings */
	write_reg(0x0380, 0x0044);
	write_reg(0xA280, 0x0064);

	/* Vsync */
	write_reg(0x3500, 0x0010);
	write_reg(0x4400, 0x0000);
	write_reg(0x4401, 0x0000);

	/* Gamma settings */
	if ((panel_ids.module_id == 0x02) && (panel_ids.revision_id < 0x10))
		gamma_r_ctrl();

	/* CMD2 lock */
	write_reg(0x1580, 0x00AA);

	/* Backlight control settings */
	write_reg(0x5100, 0x00FF); /* Display Brightness =255 (max)*/
	write_reg(0x5300, 0x002C);
	write_reg(0xF400, 0x0055);
	write_reg(0x83C0, 0x0018);
	write_reg(0x81C1, 0x0004);
	write_reg(0x82C0, 0x00B0);
	write_reg(0x83C1, 0x0040);
	write_reg(0x83C2, 0x0000);
	write_reg(0x90C0, 0x0032);
	write_reg(0x90C1, 0x0044);
	write_reg(0x90C2, 0x0065);
	write_reg(0x90C3, 0x0076);
	write_reg(0x90C4, 0x0088);
	write_reg(0xF5C0, 0x0055);

	/* CMD2 Unlock */
	write_reg(0xF300, 0x00AA);
	/* CMD2-page 1 Unlock */
	write_reg(0x0080, 0x0001);

	/* Backlight Control Setting */
	write_reg(0x1BC0, 0x00F3);
	write_reg(0x1CC0, 0x00EC);
	write_reg(0x1DC0, 0x00E7);
	write_reg(0x1EC0, 0x00E6);
	write_reg(0x20C0, 0x00F4);
	write_reg(0x21C0, 0x00F3);
	write_reg(0x22C0, 0x00E6);
	write_reg(0x23C0, 0x00E6);
	write_reg(0x24C0, 0x00D9);
	write_reg(0x25C0, 0x00D9);
	write_reg(0x26C0, 0x00CC);
	write_reg(0x27C0, 0x00BF);
	write_reg(0x28C0, 0x00BF);
	write_reg(0x29C0, 0x00B3);
	write_reg(0x2AC0, 0x00E6);
	write_reg(0x2BC0, 0x00D9);
	write_reg(0x2CC0, 0x00CC);
	write_reg(0x2DC0, 0x00BF);
	write_reg(0x2EC0, 0x00B3);
	write_reg(0x2FC0, 0x00A6);
	write_reg(0x30C0, 0x0099);
	write_reg(0x31C0, 0x0099);
	write_reg(0x32C0, 0x008C);
	write_reg(0x33C0, 0x008C);
	write_reg(0x34C0, 0x001F);
	write_reg(0x35C0, 0x000F);

	/* CMD2 page 1 Lock */
	write_reg(0x15C0, 0x00AA);
}

void mddi_toshiba_window_adjust(uint16 x1, uint16 x2, uint16 y1, uint16 y2)
{
	/*
	 * For the novatek NT35351 the x1, x2, y1 ,y2 coordinates
	 * are given in the video stream packets themselves.
	 */
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (column) [%d, %d]\n",
			   __func__, x1, x2);
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (page) [%d, %d]\n",
			   __func__, y1, y2);
	;
}
static void enable_backlight_control(int mode)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	/* Set the mode for CABC */
	if (dbc_ctrl == DBC_ON) {
		write_reg(0x5500, mode);
	}
}

static void disable_backlight_control(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	/* Disable CABC */
	if (dbc_ctrl == DBC_ON) {
		write_reg(0x5500, 0);
	}
}

static void toshiba_panel_on(void)
{
	/* Turn display ON */
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	write_reg(0x2900, 0);
}

static void toshiba_panel_off(void)
{
	/* Turn display OFF */
	write_reg(0x2800, 0);
}



static void toshiba_lcd_power_on(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel =
		(struct msm_fb_panel_data *)pdev->dev.platform_data;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);

	if (panel) {
		if (panel->panel_ext->power_on)
			panel->panel_ext->power_on();
	}
}

static void toshiba_lcd_power_off(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel =
		(struct msm_fb_panel_data *)pdev->dev.platform_data;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);

	if (panel) {
		if (panel->panel_ext->power_off) {
			panel->panel_ext->power_off();
		}
	}
}

static void toshiba_lcd_enter_sleep(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	/* Sleep In */
	write_reg(0x1000, 0);
	mddi_wait(80);
}

static void toshiba_lcd_exit_sleep(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	/* Sleep Out */
	write_reg(0x1100, 0);
	mddi_wait(100);
}

static void toshiba_lcd_enter_deep_standby(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	write_reg(0x4F00, 0x0001);
	mddi_wait(2);
	DBG(KERN_INFO, LEVEL_TRACE, "%s %s exit. \n", DBG_STR, __func__);
}

static void toshiba_exit_deep_standby(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel =
		(struct msm_fb_panel_data *)pdev->dev.platform_data;
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);

	if (panel) {
		if (panel->panel_ext->exit_deep_standby)
			panel->panel_ext->exit_deep_standby();
	}
}

static int mddi_toshiba_lcd_on(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE, "%s %s enter. lcd_state: %d\n",
			DBG_STR, __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	switch (lcd_state) {
	case LCD_STATE_OFF:
		toshiba_lcd_power_on(pdev);
		lcd_state = LCD_STATE_POWER_ON;
		break;

	case LCD_STATE_POWER_ON:
		toshiba_lcd_exit_sleep();
		toshiba_lcd_driver_init(pdev);
		toshiba_panel_on();
		enable_backlight_control(dbc_mode);
		lcd_state = LCD_STATE_ON;
		break;

	case LCD_STATE_SLEEP:
		toshiba_exit_deep_standby(pdev);
		toshiba_lcd_exit_sleep();
		toshiba_lcd_driver_init(pdev);
		toshiba_panel_on();
		enable_backlight_control(dbc_mode);
		lcd_state = LCD_STATE_ON;
		break;

	case LCD_STATE_ON:
		break;

	default:
		break;
	}
#ifdef ESD_RECOVERY_SUPPORT
	if (lcd_state == LCD_STATE_ON)
		esd_recovery_resume();
#endif
	mutex_unlock(&mddi_mutex);
	DBG(KERN_INFO, LEVEL_TRACE, "%s %s exit. lcd_state: %d\n",
			DBG_STR, __func__, lcd_state);
	return 0;
}

static int mddi_toshiba_lcd_off(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE, "%s %s enter. lcd_state: %d\n",
			DBG_STR, __func__, lcd_state);

	mutex_lock(&mddi_mutex);

	switch (lcd_state) {
	case LCD_STATE_POWER_ON:
		toshiba_lcd_power_off(pdev);
		lcd_state = LCD_STATE_OFF;
		break;

	case LCD_STATE_ON:
		disable_backlight_control();
		toshiba_panel_off();
		toshiba_lcd_enter_sleep();
		toshiba_lcd_enter_deep_standby();
		lcd_state = LCD_STATE_SLEEP;
		break;

	case LCD_STATE_SLEEP:
		toshiba_lcd_power_off(pdev);
		lcd_state = LCD_STATE_OFF;
		break;

	case LCD_STATE_OFF:
		break;

	default:
		break;
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

	mutex_lock(&toshiba_panel_ids_lock);

	ret = mddi_host_register_read(LCD_REG_CELL_ID, &panel_ids.cell_id, 1,
								MDDI_HOST_PRIM);
	if (((panel_ids.cell_id & 0xFF) != MDDI_TOSHIBA_HVGA_CELL_ID) ||
								(ret < 0)) {
		mutex_unlock(&toshiba_panel_ids_lock);
		return -1;
	}

	/* LCD_REG_DRIVER_IC_ID not used*/
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

	mutex_unlock(&toshiba_panel_ids_lock);
	return 0;
}

#ifdef ESD_RECOVERY_SUPPORT
static int esd_failure_check(void)
{
	u32 checksum = 0;

	if (mddi_host_register_read(0x0F00, &checksum, 1, MDDI_HOST_PRIM)) {
		printk(KERN_INFO DBG_STR"MDDI read timeout/error\n");
		return 0;
	}
	checksum &= 0xff;
	/* During high MDDI bus-activity it can be 0, thus 0 can be valid */
	if (checksum && (checksum == ESD_CHECKSUM_VAL)) {
		printk(KERN_INFO DBG_STR"esd checksum 0x%x wrong\n", checksum);
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
				toshiba_exit_deep_standby(lcd_data.pdev);
				toshiba_lcd_exit_sleep();
				toshiba_lcd_driver_init(lcd_data.pdev);
				toshiba_panel_on();
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

	enable_backlight_control(dbc_mode);

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
	struct msm_fb_data_type *mfd;

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
		toshiba_lcd_power_on(pf_dev);
		mfd = (struct msm_fb_data_type *)
			pf_dev->dev.platform_data;
		if (mfd == NULL) {
			DBG(KERN_INFO, LEVEL_DEBUG, "%s %s mfd == null\n",
				DBG_STR, __func__);
		}
		if ((mfd->mddi_early_suspend.resume) == NULL) {
			DBG(KERN_INFO, LEVEL_DEBUG,
			"%s %s mfd->mddi_early_suspend.resume ==  null\n",
			DBG_STR, __func__);
		} else {
			DBG(KERN_INFO, LEVEL_DEBUG,
			"%s %s mfd->mddi_early_suspend.resume-> !=  null\n",
			DBG_STR, __func__);
			mfd->mddi_early_suspend.resume(
						&(mfd->mddi_early_suspend));
		}
		/* Perform power-on sequence */
		lcd_state = LCD_STATE_POWER_ON;
		toshiba_lcd_exit_sleep();
		toshiba_lcd_driver_init(pf_dev);
		toshiba_panel_on();
		enable_backlight_control(dbc_mode);
		lcd_state = LCD_STATE_ON;
		power_ctrl = POWER_ON;
	} else {
		disable_backlight_control();
		toshiba_panel_off();
		toshiba_lcd_enter_sleep();
		toshiba_lcd_enter_deep_standby();
		lcd_state = LCD_STATE_SLEEP;
		power_ctrl = POWER_OFF;
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

static int mddi_toshiba_hvga_lcd_probe(struct platform_device *pdev)
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
		panel_data->on  = mddi_toshiba_lcd_on;
		panel_data->off = mddi_toshiba_lcd_off;
		panel_data->panel_ext->window_adjust =
				mddi_toshiba_window_adjust;

		/* Add mfd on driver_data */
		msm_fb_add_device(pdev);

		sysfs_attribute_register(pdev);
		ret = 0;
	}
exit_point:
	return ret;
}
static int __devexit mddi_toshiba_hvga_lcd_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_display_driver_info);
	device_remove_file(&pdev->dev, &dev_attr_dbc_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbc_mode);
	device_remove_file(&pdev->dev, &dev_attr_power_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbg_lvl);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_toshiba_hvga_lcd_probe,
	.remove = __devexit_p(mddi_toshiba_hvga_lcd_remove),
	.driver = {
		.name = "mddi_toshiba_hvga",
	},
};

static int __init mddi_toshiba_hvga_lcd_init(void)
{
	int ret;

	DBG(KERN_INFO, LEVEL_TRACE,  "%s (ver:0x%x) [%d]\n",
			__func__, MDDI_DRIVER_VERSION, lcd_state);
	ret = platform_driver_register(&this_driver);
	return ret;
}

static void __exit mddi_toshiba_hvga_lcd_exit(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("joakim.wesslen@sonyericsson.com");
MODULE_DESCRIPTION("MDDI implementation of the Toshiba HVGA display");

module_init(mddi_toshiba_hvga_lcd_init);
module_exit(mddi_toshiba_hvga_lcd_exit);
