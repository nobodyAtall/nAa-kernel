/* drivers/video/msm/mddi_samsung_qvga.c
 *
 * MDDI client driver for the Samsung QVGA display
 * with driver IC Himax HX8356-A01.
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
#define MDDI_DRIVER_VERSION 0x0019

/* Display CELL ID value */
#define MDDI_SAMSUNG_QVGA_CELL_ID 0x80

/* Debug setup */
#define DBG_STR "MDDI: Samsung QVGA: "

/* Numbet retries of Digital Gamma Control if it fails */
#define NBR_DGC_WRITE_RETRIES 10

/* ESD recovery setup */
#define ESD_RECOVERY_SUPPORT
#define ESD_POLL_TIME_MS 2000
#define ESD_FAILURE_CHECK_AGAIN_TIME_MS 100
#define ESD_FAILURE_NUMBER_MAX 3

enum lcd_registers_t {
	LCD_REG_COLUMN_ADDRESS = 0x02,
	LCD_REG_ROW_ADDRESS = 0x06,
	LCD_REG_CELL_ID = 0x65,
	LCD_REG_MODULE_ID = 0x64,
	LCD_REG_REVISION_ID = 0x66
};

/* Function Configuration */
static int dbc_ctrl = DBC_ON;
static int dbc_mode = DBC_MODE_VIDEO;
static int power_ctrl = POWER_OFF;
static int dbg_lvl = LEVEL_QUIET;

/* Variable declarations */
static enum mddi_lcd_state lcd_state = LCD_STATE_OFF;
static DEFINE_MUTEX(mddi_mutex);
static DEFINE_MUTEX(samsung_panel_ids_lock);

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
#endif /* ESD_RECOVERY_SUPPORT */

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
#ifdef ENABLE_SYSFS_WRITE_DGC
static ssize_t store_write_dgc(struct device *pdev,
			struct device_attribute *attr,
			const char *buf,
			size_t count);
#endif /* ENABLE_SYSFS_WRITE_DGC */
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
#ifdef ENABLE_SYSFS_WRITE_DGC
static DEVICE_ATTR(write_dgc, 0644, NULL, store_write_dgc);
#endif
static DEVICE_ATTR(dbg_lvl, 0644, show_dbg_lvl, store_dbg_lvl);

#define DIGITAL_GAMMA_ARRAY_SIZE 126
static uint32 reg_acc_data_digital_gamma[DIGITAL_GAMMA_ARRAY_SIZE] = {
 0x00000000, 0x00000006, 0x0000000D, 0x00000013,
 0x00000019, 0x0000001D, 0x00000024, 0x0000002D,
 0x00000037, 0x00000040, 0x0000004A, 0x00000053,
 0x0000005B, 0x00000064, 0x0000006D, 0x00000078,
 0x00000080, 0x00000089, 0x00000093, 0x0000009B,
 0x000000A3, 0x000000AA, 0x000000B2, 0x000000BB,
 0x000000C3, 0x000000CC, 0x000000D4, 0x000000DC,
 0x000000E4, 0x000000EB, 0x000000F2, 0x000000FA,
 0x000000FF, 0x00000000, 0x00000000, 0x00000000,
 0x00000000, 0x00000000, 0x00000000, 0x00000000,
 0x00000000, 0x00000000, 0x00000000, 0x00000006,
 0x0000000B, 0x00000011, 0x00000017, 0x0000001A,
 0x00000020, 0x00000029, 0x00000032, 0x0000003C,
 0x00000044, 0x0000004E, 0x00000056, 0x0000005E,
 0x00000068, 0x00000071, 0x0000007A, 0x00000082,
 0x00000088, 0x00000092, 0x0000009B, 0x000000A3,
 0x000000AB, 0x000000B3, 0x000000BC, 0x000000C3,
 0x000000CC, 0x000000D6, 0x000000DE, 0x000000E6,
 0x000000EF, 0x000000F9, 0x000000FF, 0x00000000,
 0x00000000, 0x00000000, 0x00000000, 0x00000000,
 0x00000000, 0x00000000, 0x00000000, 0x00000000,
 0x00000005, 0x00000007, 0x0000000C, 0x00000012,
 0x00000015, 0x00000019, 0x0000001D, 0x00000025,
 0x0000002C, 0x00000035, 0x0000003C, 0x00000045,
 0x0000004C, 0x00000054, 0x0000005C, 0x00000063,
 0x0000006B, 0x00000075, 0x0000007F, 0x00000086,
 0x00000090, 0x00000097, 0x0000009F, 0x000000A6,
 0x000000B0, 0x000000B8, 0x000000C1, 0x000000CD,
 0x000000D5, 0x000000DE, 0x000000E7, 0x000000F3,
 0x000000FF, 0x00000000, 0x00000000, 0x00000000,
 0x00000000, 0x00000000, 0x00000000, 0x00000000,
 0x00000000, 0x00000000
};


/* ----- Driver functions ----- */

static void samsung_lcd_dbc_on(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (dbc_ctrl) {
		/* CABC Control */
		write_reg(0x94, 0xFF);	/* DBV */
		/* BCTRL=1, DD=0, BL=1 */
		write_reg(0x95, 0x24);	/* 0x24: On, 0x04: Off */
		/* Adaptive brightness control on */
		write_reg(0x96, dbc_mode);
		mddi_wait(6); /* spec: > 5 ms */
	}
}

static void samsung_lcd_dbc_off(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (dbc_ctrl) {
		/* Adaptive brightness control off */
		write_reg(0x96, 0x00);
		mddi_wait(6); /* spec: > 5 ms */
	}
}

static void samsung_lcd_digital_gamma_ctrl(boolean on)
{
	if (on) {
		write_reg(0x5F, 0x01);
	} else {
		write_reg(0x5F, 0x00);
	}
}

static void samsung_lcd_digital_gamma_write(void)
{
	uint32 pre_err_cnt;
	uint32 post_err_cnt;
	u8 nbr_retries = 0;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s \n", __func__);
	do {
		pre_err_cnt = mddi_host_get_error_count();
		write_reg_xl(0x5E, reg_acc_data_digital_gamma,
						DIGITAL_GAMMA_ARRAY_SIZE);
		post_err_cnt = mddi_host_get_error_count();
		nbr_retries++;
		if (pre_err_cnt != post_err_cnt) {
			printk(KERN_INFO "%s %s DGC write error."
				" nbr_retries = %d.\n", DBG_STR,
				__func__, nbr_retries);
		}
	} while ( (pre_err_cnt != post_err_cnt) &&
			(nbr_retries < NBR_DGC_WRITE_RETRIES) );
	samsung_lcd_digital_gamma_ctrl(true);
}

static void samsung_lcd_driver_init(struct platform_device *pdev)
{
	static bool display_init_flag;
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	/* Start Init Sequence */
	write_reg(0x3E, 0x40);	/* SON */
	write_reg(0x16, 0x08);	/* BGR=1  */
	write_reg(0x71, 0x68);	/* SS01, GS=1, TEON=1 */

	/* Power On Sequence */
	write_reg(0x60, 0x01);	/* OSC_EN */
	mddi_wait(11); /* spec: > 10 ms */
	write_reg(0x21, 0x00);	/* FS=00 */
	write_reg(0x27, 0x06);	/* BP=6 */
	write_reg(0x28, 0x06);	/* FP=6 */
	write_reg(0x1C, 0x04);	/* AP=100, const curr = 1 */
	mddi_wait(11); /* spec: > 10 ms */
	write_reg(0x1B, 0x14);	/* PON=1, XDK=1 */
	mddi_wait(41); /* spec: > 40 ms */
	write_reg(0x1D, 0x04);	/* VGBP x 4.32 */
	write_reg(0x1E, 0x04);	/* 1.76 x VCI */
	write_reg(0x1F, 0x09);	/* VGBP x 3.9 */
	write_reg(0x20, 0x10);	/* BT */
	write_reg(0x43, 0x80);	/* VCOMG=1 */
	mddi_wait(11); /* spec: > 10 ms */

	/* Only use VCOMH register for:
	   Robyn Samsung Module ID 0x00, Revision ID 0x00
	   Mimmi Samsung Module ID 0x00, Revision ID 0x00 */
	if (((panel_ids.module_id & 0xFF) == 0x00) &&
				((panel_ids.revision_id & 0xFF) == 0x00)) {
		write_reg(0x44, 0x48);   /* VCOMH */
		mddi_wait(11); /* spec: > 10 ms */
	}

	write_reg(0x45, 0x0E);	/* VCOMA */
	mddi_wait(11); /* spec: > 10 ms */
	write_reg(0x01, 0x00);	/* Rev_panel off */

	/* Gamma Setting */
	write_reg(0xD0, 0x00);
	write_reg(0xD1, 0x03);
	write_reg(0xD2, 0x08);
	write_reg(0xD3, 0x0B);
	write_reg(0xD4, 0x0D);
	write_reg(0xD5, 0x3F);
	write_reg(0xD6, 0x0F);
	write_reg(0xD7, 0x17);
	write_reg(0xD8, 0x03);
	write_reg(0xD9, 0x0C);
	write_reg(0xDA, 0x0F);
	write_reg(0xDB, 0x18);
	write_reg(0xDC, 0x17);
	write_reg(0xDD, 0x0B);
	write_reg(0xDE, 0x14);
	write_reg(0xDF, 0x06);
	write_reg(0xE0, 0x11);
	write_reg(0xE1, 0x00);
	write_reg(0xE2, 0x32);
	write_reg(0xE3, 0x34);
	write_reg(0xE4, 0x3E);
	write_reg(0xE5, 0x3B);
	write_reg(0xE6, 0x3F);
	write_reg(0xE7, 0x41);
	write_reg(0xE8, 0x49);
	write_reg(0xE9, 0x0E);
	write_reg(0xEA, 0x0F);
	write_reg(0xEB, 0x05);
	write_reg(0xEC, 0x00);
	write_reg(0xED, 0x06);
	write_reg(0xEE, 0x08);
	write_reg(0xEF, 0x0D);
	write_reg(0xF0, 0x13);
	write_reg(0xF1, 0x1B);
	write_reg(0xF2, 0xDE);
	/* CABC Control. Not in spec */
	/* This shall be here according to Optics HW */
	write_reg(0x98, 0x0F); /* pwm out =40.8 kHz, active high */
	mddi_wait(10);
	if (pdev && pdev->dev.platform_data) {
		struct msm_fb_panel_data *panel =
			(struct msm_fb_panel_data *)pdev->dev.platform_data;
		if (!display_init_flag) {
			/* Replace display internal random data
			with black pixels */
			mddi_video_stream_black_display(0, 0,
					panel->panel_info.xres,
					panel->panel_info.yres, MDDI_HOST_PRIM);
			display_init_flag = 1;
		}
	}
}

static void samsung_lcd_window_address_set(enum lcd_registers_t reg,
						u16 start , u16 stop)
{
	uint32 para;

	mutex_lock(&mddi_mutex);
	para = (uint32) (start >> 8);
	write_reg (reg, para);
	para = (uint32) (start & 0xff);
	write_reg (reg + 1, para);
	para = (uint32) ((start + stop) >> 8);
	write_reg (reg + 2, para);
	para = (uint32) ((start + stop) & 0xff);
	write_reg (reg + 3, para);

	if (reg == LCD_REG_COLUMN_ADDRESS) {
		lcd_data.last_window.x1 = start;
		lcd_data.last_window.x2 = stop;
	} else {
		lcd_data.last_window.y1 = start;
		lcd_data.last_window.y2 = stop;
	}

	mutex_unlock(&mddi_mutex);
}

/* x1 = starting x (column)
   x2 = (width-1)
   y1 = starting y (row)
   y2 = (height-1) */
static void samsung_lcd_window_adjust(uint16 x1, uint16 x2,
				      uint16 y1, uint16 y2)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	/* Column address start register */
	if (x1 != lcd_data.last_window.x1 || x2 != lcd_data.last_window.x2) {
		samsung_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
		DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (column) [%d, %d]\n",
		   __func__, x1, x2);
	}
	if (y1 != lcd_data.last_window.y1 || y2 != lcd_data.last_window.y2) {
		samsung_lcd_window_address_set(LCD_REG_ROW_ADDRESS, y1, y2);
		DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (row) [%d, %d]\n",
		   __func__, y1, y2);
	}
}

static void samsung_lcd_power_off_regs(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	write_reg(0x1C, 0x00);	/* */
	write_reg(0x1B, 0x0C);	/* */
	write_reg(0x43, 0x00);	/* */
}


//Enter Standby
static void samsung_lcd_enter_sleep(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	write_reg(0x26, 0x38);	/* */
	mddi_wait(41); /* spec: > 40 ms */
	write_reg(0x26, 0x20);	/* */
	write_reg(0x1B, 0x15);	/* */
	mddi_wait(41); /* spec: > 40 ms */
	write_reg(0x60, 0x00);	/* */
	mddi_wait(11); /* spec: > 10 ms */
}

/* Release Standby */
static void samsung_lcd_exit_sleep(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	write_reg(0x60, 0x01);	/* OSC_EN */
	mddi_wait(11); /* spec: > 10 ms */
	write_reg(0x1B, 0x14);	/* PON=1, XDK=0 */
	mddi_wait(41); /* spec: > 40 ms */
	write_reg(0x26, 0x24);	/* VGL, VSSD */
	write_reg(0x26, 0x38);	/* VGL/VGH, PT */
	mddi_wait(41); /* spec: > 40 ms */
	write_reg(0x26, 0x3C);	/* VGL/VGH, DISPLAY */
}

static void samsung_power_on(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;
	panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (panel && panel->panel_ext->power_on)
		panel->panel_ext->power_on();
}

static void samsung_power_off(struct platform_device *pdev)
{
	struct msm_fb_panel_data *panel;
	panel = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	if (panel && panel->panel_ext->power_off)
		panel->panel_ext->power_off();
}

static int mddi_samsung_lcd_on(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	switch (lcd_state) {
	case LCD_STATE_OFF:
		samsung_power_on(pdev);
		lcd_state = LCD_STATE_POWER_ON;
		break;

	case LCD_STATE_POWER_ON:
		samsung_lcd_driver_init(pdev);
		samsung_lcd_exit_sleep();
		samsung_lcd_dbc_on();
		samsung_lcd_digital_gamma_write();
		lcd_state = LCD_STATE_ON;
		break;

	case LCD_STATE_SLEEP:
#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
		samsung_power_on(pdev);
		samsung_lcd_driver_init(pdev);
#endif
		samsung_lcd_exit_sleep();
		samsung_lcd_dbc_on();
#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
		samsung_lcd_digital_gamma_write();
#endif
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
	return 0;
}


static int mddi_samsung_lcd_off(struct platform_device *pdev)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	switch (lcd_state) {
	case LCD_STATE_POWER_ON:
		samsung_lcd_power_off_regs();
		samsung_power_off(pdev);
		lcd_state = LCD_STATE_OFF;
		break;

	case LCD_STATE_ON:
		samsung_lcd_dbc_off();
		samsung_lcd_enter_sleep();
#ifdef CONFIG_FB_MSM_MDDI_SEMC_LCD_POWER_OFF_SLEEP_MODE
		samsung_lcd_power_off_regs();
		samsung_power_off(pdev);
#endif
		lcd_state = LCD_STATE_SLEEP;
		break;

	case LCD_STATE_SLEEP:
		samsung_lcd_power_off_regs();
		samsung_power_off(pdev);
		lcd_state = LCD_STATE_OFF;
		break;

	case LCD_STATE_OFF:
		break;

	default:
		break;
	}
	mutex_unlock(&mddi_mutex);
#ifdef ESD_RECOVERY_SUPPORT
	cancel_delayed_work_sync(&lcd_data.esd_check);
#endif
	return 0;
}

static int check_panel_ids(void)
{
	int ret;

	mutex_lock(&samsung_panel_ids_lock);

	ret = mddi_host_register_read(LCD_REG_CELL_ID, &panel_ids.cell_id,
				      1, MDDI_HOST_PRIM);
	if (((panel_ids.cell_id & 0xFF) != MDDI_SAMSUNG_QVGA_CELL_ID) ||
								(ret < 0)) {
		mutex_unlock(&samsung_panel_ids_lock);
		return -1;
	}

	/* LCD_REG_DRIVER_IC_ID not used in Samsung */
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

	mutex_unlock(&samsung_panel_ids_lock);
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
	if (id && id != MDDI_SAMSUNG_QVGA_CELL_ID) {
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
				samsung_lcd_driver_init(lcd_data.pdev);
				samsung_lcd_dbc_on();

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
		printk(KERN_ALERT DBG_STR"%sInvalid flag for dbc ctrl\n",
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

	samsung_lcd_dbc_on();

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
		samsung_power_on(pf_dev);
		samsung_lcd_driver_init(pf_dev);
		samsung_lcd_exit_sleep();
		samsung_lcd_dbc_on();
		samsung_lcd_digital_gamma_write();
		lcd_state = LCD_STATE_ON;
	} else {
		power_ctrl = POWER_OFF;
		samsung_lcd_dbc_off();
		samsung_lcd_enter_sleep();
		samsung_lcd_power_off_regs();
		samsung_power_off(pf_dev);
		lcd_state = LCD_STATE_SLEEP;
	}

	DBG(KERN_INFO, LEVEL_PARAM, DBG_STR"%s power_ctrl set to %d\n",
			__func__, power_ctrl);
	ret = strnlen(buf, count);
unlock:
	mutex_unlock(&mddi_mutex);
error:
	return ret;
}

#ifdef ENABLE_SYSFS_WRITE_DGC
static ssize_t store_write_dgc(struct device *dev_p,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);

	mutex_lock(&mddi_mutex);
	samsung_lcd_digital_gamma_write();
	mutex_unlock(&mddi_mutex);
error:
	return 0;
}
#endif /* ENABLE_SYSFS_WRITE_DGC */


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

#ifdef ENABLE_SYSFS_WRITE_DGC
	ret = device_create_file(&pdev->dev, &dev_attr_write_dgc);
	if (ret != 0) {
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" write dgc (%d)\n", __func__, ret);
	}
#endif /* ENABLE_SYSFS_WRITE_DGC */

	ret = device_create_file(&pdev->dev, &dev_attr_dbg_lvl);
	if (ret != 0)
		printk(KERN_ERR DBG_STR"%s Failed to register"
			" debug attributes (%d)\n", __func__, ret);
}

static int mddi_samsung_qvga_lcd_probe(struct platform_device *pdev)
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
#endif /* ESD_RECOVERY_SUPPORT */

		panel_data->on  = mddi_samsung_lcd_on;
		panel_data->off = mddi_samsung_lcd_off;
		panel_data->panel_ext->window_adjust =
					samsung_lcd_window_adjust;

		/* Add mfd on driver_data */
		msm_fb_add_device(pdev);

		sysfs_attribute_register(pdev);
		ret = 0;
	}
exit_point:
	return ret;
}

static int __devexit mddi_samsung_qvga_lcd_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_display_driver_info);
	device_remove_file(&pdev->dev, &dev_attr_dbc_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbc_mode);
	device_remove_file(&pdev->dev, &dev_attr_power_ctrl);
#ifdef ENABLE_SYSFS_WRITE_DGC
	device_remove_file(&pdev->dev, &dev_attr_write_dgc);
#endif
	device_remove_file(&pdev->dev, &dev_attr_dbg_lvl);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_samsung_qvga_lcd_probe,
	.remove = __devexit_p(mddi_samsung_qvga_lcd_remove),
	.driver = {
		.name = "mddi_samsung_qvga",
	},
};

static int __init mddi_samsung_qvga_lcd_init(void)
{
	int ret;

	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s (ver:0x%x) [%d]\n",
			__func__, MDDI_DRIVER_VERSION, lcd_state);
	ret = platform_driver_register(&this_driver);
	return ret;
}

static void __exit mddi_samsung_qvga_lcd_exit(void)
{
	DBG(KERN_INFO, LEVEL_TRACE, DBG_STR"%s [%d]\n", __func__, lcd_state);
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("joakim.wesslen@sonyericsson.com");
MODULE_DESCRIPTION("MDDI implementation of the Samsung QVGA display");

module_init(mddi_samsung_qvga_lcd_init);
module_exit(mddi_samsung_qvga_lcd_exit);
