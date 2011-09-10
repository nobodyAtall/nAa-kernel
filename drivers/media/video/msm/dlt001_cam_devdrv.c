/*
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/board.h>

#include <mach/vreg.h>

#include "dlt001_cam_devdrv.h"
#include "dlt001_cam_devdrv_table.h"

#ifdef CDBG
#undef CDBG
#endif

#define CDBG(fmt, args...) printk(KERN_DEBUG "msm_camdrv: " fmt, ##args)

#ifdef DDBG
#undef DDBG
#endif

#define DDBG(fmt, args...)

/* ******** Local functions ************* */
static int32_t dlt001_gpio_access(int gpio_pin, int dir);
static int32_t dlt001_resource_enable(struct msm_camera_sensor_pwr *resource);
static int32_t dlt001_resource_disable(struct msm_camera_sensor_pwr *resource);
static int32_t dlt001_sensor_on(void);
static int32_t dlt001_sensor_init(void);
static void dlt001_sensor_off(void);
static int dlt001_sensor_open(const struct msm_camera_sensor_info *data);
static int dlt001_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit dlt001_i2c_remove(struct i2c_client *client);
static int32_t dlt001_i2c_write(uint16_t address, uint8_t data_length, const uint8_t *data);
static int32_t dlt001_i2c_read(uint16_t address, uint8_t length, uint8_t *data);
static int32_t dlt001_set_sensor_mode(uint8_t mode);
static int32_t dlt001_monitor_config(void);
static int32_t dlt001_raw_snapshot_config(void);
static int32_t dlt001_raw_snapshot_start(void);
static int32_t dlt001_snapshot_config(void);
static int32_t dlt001_half_release_config(void);
static int32_t dlt001_set_test_pattern(enum set_test_pattern mode);
static int32_t dlt001_set_focus_mode(enum camera_focus_mode focus_mode);
static int32_t dlt001_update_focus_mode(enum camera_focus_mode focus_mode);
static int32_t dlt001_set_scene(enum camera_scene scene);
static int32_t dlt001_update_scene(enum camera_scene start_scene);
static int32_t dlt001_set_dimensions(struct camera_dimension dimension);
static int32_t dlt001_get_af_status(enum camera_af_status *status);
static int32_t dlt001_get_exif(struct cam_ctrl_exif_params *exif);
static int32_t dlt001_read_vendor_data(void);
static void dlt001_add_bytes_in_switched_endian(uint8_t *p_package_buf, uint8_t pkg_position,
						uint8_t size_of_data, uint8_t *Data);
static int32_t dlt001_send_reg_table(const struct reg_entry *table, uint32_t tables_to_send);
static int32_t dlt001_check_msts(uint8_t value, uint32_t timeout_msec);
static int32_t dlt001_check_bsts(uint8_t value, uint32_t timeout_msec);
static int32_t dlt001_check_afsts(uint8_t value, uint32_t timeout_msec);
static int32_t dlt001_cam_is_assist_light_needed(int *result);
static int32_t dlt001_set_framerate(uint16_t fps);
static int32_t dlt001_write_calibration_data(struct dlt001_calibration_data);
static int init_thread(void *data);
static int autoflash_enable(int onoff);
static int autoflash_strobe(int onoff);
static int autoflash_adjust(void);

/* ********** Local variables/structs ************ */

static struct dlt001_ctrl *dlt001_ctrl = NULL;
static DECLARE_WAIT_QUEUE_HEAD(dlt001_wait_queue);
DECLARE_MUTEX(dlt001_sem);

/**
 * I2C Device ID Structure Body.
 *
 */
static const struct i2c_device_id dlt001_id[] = {
	{"dlt001_camera", 0},
	{}
};

/**
 * I2C Device Structure Body.
 *
 */
static struct i2c_driver dlt001_driver = {
	.id_table = dlt001_id,
	.probe = dlt001_i2c_probe,
	.remove = __exit_p(dlt001_i2c_remove),
	.driver = {
		   .name = "dlt001_camera",
		   },
};

/**
 * Precess IOCTL messages.
 *
 */
int dlt001_sensor_config(void __user *argp)
{
	int32_t ret;
	struct sensor_cfg_data cfg_data;

	DDBG("dlt001_sensor_config [S]\n");

	ret = copy_from_user(&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data));
	if (ret) {
		CDBG("dlt001_sensor_config copy_from_user failed\n");
		return -EFAULT;
	}

	if (cfg_data.cfgtype == CFG_SET_MODE &&
	    cfg_data.mode == SENSOR_PREVIEW_MODE &&
	    dlt001_ctrl->dev_mode == CAMERA_MODE_MONITOR && dlt001_ctrl->init_complete) {
		DDBG("Already running in monitor");
		return 0;
	}

	down(&dlt001_sem);

	DDBG("dlt001_sensor_config cfgtype = %d\n", cfg_data.cfgtype);
	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		ret = dlt001_set_sensor_mode(cfg_data.mode);
		break;

	case CFG_SET_TEST_PATTERN:
		ret = dlt001_set_test_pattern(cfg_data.cfg.set_test_pattern);
		break;

	case CFG_GET_AF_STATUS:
		ret = dlt001_get_af_status(&cfg_data.cfg.af_status);
		break;

	case CFG_GET_EXIF_INFO:
		ret = dlt001_get_exif(&cfg_data.cfg.exif);
		break;

	case CFG_SET_SCENE:
		ret = dlt001_set_scene(cfg_data.cfg.scene);
		break;

	case CFG_SET_FOCUS_MODE:
		ret = dlt001_set_focus_mode(cfg_data.cfg.focus_mode);
		break;

	case CFG_SET_DIMENSIONS:
		ret = dlt001_set_dimensions(cfg_data.cfg.dimension);
		break;

	case CFG_GET_AF_ASSIST_LIGHT:
		ret = dlt001_cam_is_assist_light_needed((int *)&cfg_data.rs);
		break;

	case CFG_SET_FPS:
		ret = dlt001_set_framerate(cfg_data.cfg.fps.f_mult);
		break;

	default:
		CDBG("dlt001_sensor_config cfgtype failed\n");
		ret = -EFAULT;
		break;
	}

	up(&dlt001_sem);

	ret = copy_to_user((void *)argp, &cfg_data, sizeof(struct sensor_cfg_data));
	if (ret) {
		CDBG("dlt001_sensor_config copy_to_user failed\n");
		return -EFAULT;
	}

	DDBG("dlt001_sensor_config [E]\n");

	return ret;
}

/**
 * Release
 *
 */
int dlt001_sensor_release(void)
{

	DDBG("dlt001_sensor_release [S]\n");

	down(&dlt001_sem);

	if (dlt001_ctrl->opened) {
		dlt001_sensor_off();
		dlt001_ctrl->opened = 0;
	}

	dlt001_ctrl->dev_mode = CAMERA_MODE_STANDBY;

	up(&dlt001_sem);

	DDBG("dlt001_sensor_release [E]\n");

	return 0;
}

/**
 * Exit
 *
 */
void dlt001_exit(void)
{
	DDBG("dlt001_exit [S]\n");

	i2c_del_driver(&dlt001_driver);

	DDBG("dlt001_exit [E]\n");
}

/**
 * Probe
 *
 */
static int dlt001_camera_probe(const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *s)
{
	int rc = 0;

	DDBG("dlt001_camera_probe [S]\n");

	dlt001_ctrl = kzalloc(sizeof(struct dlt001_ctrl), GFP_KERNEL);
	if (NULL == dlt001_ctrl) {
		CDBG("dlt001_cam_probe memory allocation failed\n");
		goto probe_done;
	}

	if (info != NULL) {
		dlt001_ctrl->sensordata = (struct msm_camera_sensor_info *)info;
		rc = i2c_add_driver(&dlt001_driver);
		if (IS_ERR_VALUE(rc)) {
			kfree(dlt001_ctrl);
			goto probe_done;
		}
	}

	dlt001_ctrl->dev_mode = CAMERA_MODE_STANDBY;

	rc = dlt001_sensor_on();
	if (rc < 0) {
		CDBG("dlt001_camera_probe sensor_on failed\n");
		goto probe_done;
	}

	msm_camio_clk_enable(CAMIO_VFE_CLK);

	/* Output CAM_MCLK(19.2MHz) */
	msm_camio_clk_rate_set(DLT001_DEFAULT_CLOCK_RATE);

	msleep(40);

	/* CAM_RESET_N release(GPIO0 = High) */
	rc = dlt001_gpio_access(dlt001_ctrl->sensordata->sensor_reset, TRUE);
	if (rc) {
		CDBG("dlt001_probe CAM_RESET_N release failed\n");
		goto probe_done;
	}

	msleep(20);

	/* STANDBY (GPIO0 = High) */
	rc = dlt001_resource_enable(&dlt001_ctrl->sensordata->standby);
	if (rc) {
		CDBG("dlt001_probe STANDBY failed\n");
		goto probe_done;
	}

	mdelay(10);

	rc = dlt001_read_vendor_data();
	if (rc != 0) {
		CDBG("dlt001_read_vendr_data failed\n");
		goto probe_done;
	}

	CDBG("Camera vendor [%d]\n", dlt001_ctrl->vendor_id);
	CDBG("Camera revision[%d]\n", dlt001_ctrl->camera_revision);

	s->s_init = dlt001_sensor_open;
	s->s_release = dlt001_sensor_release;
	s->s_config = dlt001_sensor_config;

 probe_done:
	if (dlt001_ctrl) {
		dlt001_sensor_off();
	}

	DDBG("dlt001_camera_probe [E]\n");
	return rc;
}

static int32_t dlt001_read_vendor_data(void)
{
	int32_t rc = 0;
	uint32_t data = 0;
	uint8_t v_flag_otp0 = 0;
	uint8_t v_flag_otp1 = 0;
	enum dlt001_otp otp_id = OTP_NO_DATA_WRITTEN;
	uint32_t c_nr_1 = 0;
	uint32_t c_nr_2 = 0;
	uint8_t c_nr_res = 0;
	uint32_t c_nb = 0;
	uint32_t c_pr = 0;
	uint32_t c_pb = 0;
	uint16_t inormr_k = 0x10FB;
	uint16_t inormb_k = 0x10AB;
	uint16_t iawbprer_k = 0x013C;
	uint16_t iawbpreb_k = 0x0241;
	uint16_t inormr_s = 0x10BD;
	uint16_t inormb_s = 0x0EE0;
	uint16_t iawbprer_s = 0x0146;
	uint16_t iawbpreb_s = 0x0239;

	rc = dlt001_i2c_read(DLT001_USERCTRL_0000, BYTE_2, (uint8_t *) &data);
	dlt001_ctrl->camera_revision = data & LOW_BIT_32;

	rc += dlt001_i2c_read(DLT001_USERCTRL_0250, BYTE_4, (uint8_t *) &data);
	if ((data & DLT001_V_FLAG_OTP_MASK) > 0)
		v_flag_otp0 = 1;

	rc += dlt001_i2c_read(DLT001_USERCTRL_025C, BYTE_4, (uint8_t *) &data);
	if ((data & DLT001_V_FLAG_OTP_MASK) > 0)
		v_flag_otp1 = 1;

	if (v_flag_otp0 == 0) {
		if (v_flag_otp1 == 0)
			otp_id = OTP_NO_DATA_WRITTEN;
		else
			otp_id = OTP_1;
	} else {
		if (v_flag_otp1 == 0)
			otp_id = OTP_0;
		else
			otp_id = OTP_1;
	}

	switch (otp_id) {
	case OTP_NO_DATA_WRITTEN:
		CDBG("dlt001_read_vendor_data failed! No valid OTP\n");
		return -1;
	case OTP_0:
		rc += dlt001_i2c_read(DLT001_USERCTRL_0258, BYTE_4, (uint8_t *) &data);
		break;
	case OTP_1:
		rc += dlt001_i2c_read(DLT001_USERCTRL_0264, BYTE_4, (uint8_t *) &data);
		break;
	default:
		return -1;
	}

	if ((data & DLT001_VENDOR_ID_OTP_MASK) == 0) {
		dlt001_ctrl->vendor_id = VENDOR_ID_0;
	} else {
		dlt001_ctrl->vendor_id = VENDOR_ID_1;
	}

	if (dlt001_ctrl->camera_revision != DLT001_CAM_REV_ES1) {
		switch (otp_id) {
		case OTP_0:
			rc += dlt001_i2c_read(DLT001_USERCTRL_0258, BYTE_4, (uint8_t *) &data);
			dlt001_ctrl->calibration_data.shd_index = ((data >> 20) & DLT001_SHD_MASK);
			rc += dlt001_i2c_read(DLT001_USERCTRL_0250, BYTE_4, (uint8_t *) &data);
			c_nr_1 = (data >> 26);
			rc += dlt001_i2c_read(DLT001_USERCTRL_0254, BYTE_4, (uint8_t *) &data);
			c_nr_2 = ((data & DLT001_WB_DATA_OTPB0_2_MASK) << 6);
			break;
		case OTP_1:
			rc += dlt001_i2c_read(DLT001_USERCTRL_0264, BYTE_4, (uint8_t *) &data);
			dlt001_ctrl->calibration_data.shd_index = ((data >> 20) & DLT001_SHD_MASK);
			rc += dlt001_i2c_read(DLT001_USERCTRL_025C, BYTE_4, (uint8_t *) &data);
			c_nr_1 = (data >> 26);
			rc += dlt001_i2c_read(DLT001_USERCTRL_0260, BYTE_4, (uint8_t *) &data);
			c_nr_2 = ((data & DLT001_WB_DATA_OTPB0_2_MASK) << 6);
			break;
		default:
			return -1;
			break;
		}

		c_nr_res = (c_nr_1 + c_nr_2);
		c_nb = ((data >> 2) & LOW_LOW_BIT_32);
		c_pr = ((data >> 10) & LOW_LOW_BIT_32);
		c_pb = ((data >> 18) & LOW_LOW_BIT_32);
		if (dlt001_ctrl->vendor_id == VENDOR_ID_0) {
			dlt001_ctrl->calibration_data.normr = (inormr_k * (128 + c_nr_res)) / 256;
			dlt001_ctrl->calibration_data.normb = (inormb_k * (128 + c_nb)) / 256;
			dlt001_ctrl->calibration_data.awbprer = (iawbprer_k * (128 + c_pr)) / 256;
			dlt001_ctrl->calibration_data.awbpreb = (iawbpreb_k * (128 + c_pb)) / 256;
		} else if (dlt001_ctrl->vendor_id == VENDOR_ID_1) {
			dlt001_ctrl->calibration_data.normr = (inormr_s * (128 + c_nr_res)) / 256;
			dlt001_ctrl->calibration_data.normb = (inormb_s * (128 + c_nb)) / 256;
			dlt001_ctrl->calibration_data.awbprer = (iawbprer_s * (128 + c_pr)) / 256;
			dlt001_ctrl->calibration_data.awbpreb = (iawbpreb_s * (128 + c_pb)) / 256;
		}

		switch (otp_id) {
		case OTP_0:
			rc += dlt001_i2c_read(DLT001_USERCTRL_0250, BYTE_4, (uint8_t *) &data);
			dlt001_ctrl->calibration_data.otp_inf = ((data >> 5) & DLT001_AF_MASK);
			dlt001_ctrl->calibration_data.otp_macro = ((data >> 16) & DLT001_AF_MASK);
			break;
		case OTP_1:
			rc += dlt001_i2c_read(DLT001_USERCTRL_025C, BYTE_4, (uint8_t *) &data);
			dlt001_ctrl->calibration_data.otp_inf = ((data >> 5) & DLT001_AF_MASK);
			dlt001_ctrl->calibration_data.otp_macro = ((data >> 16) & DLT001_AF_MASK);
			break;
		default:
			return -1;
		}
		dlt001_ctrl->calibration_data.af_c =
		    (((((8 * dlt001_ctrl->calibration_data.otp_macro) -
			(8 * dlt001_ctrl->calibration_data.otp_inf)) / 6) + 6) / 8);
		dlt001_ctrl->calibration_data.af_d = dlt001_ctrl->calibration_data.af_c / 4;
		dlt001_ctrl->calibration_data.af_e =
		    dlt001_ctrl->calibration_data.otp_inf +
		    (dlt001_ctrl->calibration_data.af_c * 8);
		dlt001_ctrl->calibration_data.af_i =
		    dlt001_ctrl->calibration_data.otp_inf +
		    (dlt001_ctrl->calibration_data.af_c * 5);
		dlt001_ctrl->calibration_data.af_j = dlt001_ctrl->calibration_data.af_c * 2;
		dlt001_ctrl->calibration_data.af_k = dlt001_ctrl->calibration_data.af_d / 2;
		dlt001_ctrl->calibration_data.af_l = dlt001_ctrl->calibration_data.af_d * 4;
		dlt001_ctrl->calibration_data.af_m =
		    dlt001_ctrl->calibration_data.otp_inf +
		    (dlt001_ctrl->calibration_data.af_c * 6);
		dlt001_ctrl->calibration_data.af_g_k = 1023;
		dlt001_ctrl->calibration_data.af_g_s = 255;
	}

	return rc;
}

/**
 * Set calibration data
 *
 */

static int32_t dlt001_write_calibration_data(struct dlt001_calibration_data calibration_data)
{
	int32_t ret = 0;

	dlt001_i2c_write(DLT001_ADJ_4A04, BYTE_2, (uint8_t *) &calibration_data.normr);
	dlt001_i2c_write(DLT001_ADJ_4A06, BYTE_2, (uint8_t *) &calibration_data.normb);
	dlt001_i2c_write(DLT001_ADJ_4A08, BYTE_2, (uint8_t *) &calibration_data.awbprer);
	dlt001_i2c_write(DLT001_ADJ_4A0A, BYTE_2, (uint8_t *) &calibration_data.awbpreb);

	dlt001_i2c_write(DLT001_AF_4876, BYTE_2, (uint8_t *) &calibration_data.otp_inf);
	dlt001_i2c_write(DLT001_AF_487A, BYTE_2, (uint8_t *) &calibration_data.otp_inf);
	dlt001_i2c_write(DLT001_AF_486C, BYTE_2, (uint8_t *) &calibration_data.af_c);
	dlt001_i2c_write(DLT001_AF_4870, BYTE_2, (uint8_t *) &calibration_data.af_c);
	dlt001_i2c_write(DLT001_AF_486E, BYTE_2, (uint8_t *) &calibration_data.af_d);
	dlt001_i2c_write(DLT001_AF_4872, BYTE_2, (uint8_t *) &calibration_data.af_d);

	if (dlt001_ctrl->vendor_id == VENDOR_ID_0) {
		if (calibration_data.af_e > calibration_data.af_g_k) {
			calibration_data.af_e = calibration_data.af_g_k;
		}
		dlt001_i2c_write(DLT001_AF_4878, BYTE_2, (uint8_t *) &calibration_data.af_e);
		dlt001_i2c_write(DLT001_AF_4880, BYTE_2, (uint8_t *) &calibration_data.af_m);
		dlt001_i2c_write(DLT001_AF_495E, BYTE_2, (uint8_t *) &calibration_data.af_e);
	}
	if (dlt001_ctrl->vendor_id == VENDOR_ID_1) {
		if (calibration_data.af_e > calibration_data.af_g_s) {
			calibration_data.af_e = calibration_data.af_g_s;
		}
		dlt001_i2c_write(DLT001_AF_4878, BYTE_2, (uint8_t *) &calibration_data.af_e);
		dlt001_i2c_write(DLT001_AF_4880, BYTE_2, (uint8_t *) &calibration_data.af_m);
		dlt001_i2c_write(DLT001_AF_495E, BYTE_2, (uint8_t *) &calibration_data.af_e);
	}

	dlt001_i2c_write(DLT001_AF_487E, BYTE_2, (uint8_t *) &calibration_data.otp_inf);
	dlt001_i2c_write(DLT001_AF_487C, BYTE_2, (uint8_t *) &calibration_data.af_i);
	dlt001_i2c_write(DLT001_AF_4844, BYTE_2, (uint8_t *) &calibration_data.af_j);
	dlt001_i2c_write(DLT001_AF_486A, BYTE_2, (uint8_t *) &calibration_data.otp_inf);
	dlt001_i2c_write(DLT001_AF_4960, BYTE_2, (uint8_t *) &calibration_data.otp_inf);
	dlt001_i2c_write(DLT001_AF_4822, BYTE_2, (uint8_t *) &calibration_data.af_k);
	dlt001_i2c_write(DLT001_AF_4824, BYTE_2, (uint8_t *) &calibration_data.af_d);
	dlt001_i2c_write(DLT001_AF_4838, BYTE_2, (uint8_t *) &calibration_data.af_l);

	return ret;
}

/**
 * Set sensor mode
 *
 */
static int32_t dlt001_set_sensor_mode(uint8_t mode)
{
	int32_t ret = 0;

	DDBG("dlt001_set_sensor_mode [S]\n");

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		ret = dlt001_monitor_config();
		break;
	case SENSOR_SNAPSHOT_MODE:
		ret = dlt001_snapshot_config();
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		ret = dlt001_raw_snapshot_config();
		break;
	case SENSOR_RAW_SNAPSHOT_START:
		ret = dlt001_raw_snapshot_start();
		break;
	case SENSOR_HALF_RELEASE_MODE:
		ret = dlt001_half_release_config();
		break;
	default:
		CDBG("dlt001_set_sensor_mode mode failed\n");
		ret = -EINVAL;
		break;
	}

	DDBG("dlt001_set_sensor_mode [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Set monitor mode
 *
 */
static int32_t dlt001_monitor_config()
{
	int32_t ret = 0;
	uint8_t afclr = 0x01;
	char data;

	DDBG("dlt001_monitor_config [S]\n");

	if (dlt001_ctrl->dev_mode == CAMERA_MODE_MONITOR) {
		DDBG("dlt001_monitor_config Already in monitor mode. Do nothing...\n");
		goto monitor_done;
	}

	ret = dlt001_i2c_read(DLT001_USERCTRL_0010, BYTE_1, &data);

	if (data == DLT001_MSTS_HR_VAL) {
		if (dlt001_ctrl->scene == SENSOR_SCENE_TWILIGHT) {
			ret = dlt001_set_regs(dlt001_hr_reset);
			if (ret) {
				CDBG("dlt001_hr_reset failed\n");
				return ret;
			}
		}
		if (dlt001_ctrl->scene == SENSOR_SCENE_AUTO) {
			if (dlt001_ctrl->autoflash_assist_light_on) {
				ret = dlt001_set_regs(dlt001_hr_LED_reset);
				if (ret) {
					CDBG("dlt001_hr_LED_reset failed\n");
					return ret;
				}
			} else {
				ret = dlt001_set_regs(dlt001_hr_auto_reset);
				if (ret) {
					CDBG("dlt001_hr__auto_reset failed\n");
					return ret;
				}
			}
		}
		dlt001_i2c_write(0x4885, BYTE_1, &afclr);
		ret = dlt001_check_afsts(DLT001_AFSTS_VAL, DLT001_POLLING_TIMEOUT_MSEC);
		if (ret) {
			CDBG("dlt001_monitor_config dlt001_check_afsts failed\n");
		}
	}
	ret = dlt001_set_regs(dlt001_mode_monitor);
	if (ret) {
		CDBG("dlt001_monitor_config failed\n");
		return ret;
	}

	ret = dlt001_check_msts(DLT001_MSTS_MON_VAL, DLT001_POLLING_TIMEOUT_MSEC);
	if (ret) {
		CDBG("dlt001_monitor_config dlt001_check_msts failed\n");
	}

	dlt001_ctrl->dev_mode = CAMERA_MODE_MONITOR;
	dlt001_ctrl->autoflash_used = FALSE;
	dlt001_ctrl->autoflash_assist_light_on = FALSE;
	dlt001_ctrl->autoflash_poll_reg_x2AA = FALSE;

 monitor_done:

	if (!dlt001_ctrl->init_complete) {
		kthread_run(init_thread, NULL, "sensor_init");
	}

	DDBG("dlt001_monitor_config [E] ret[%d]\n", ret);

	return ret;
}

/**
 * RAW snapshot config
 *
 */
static int32_t dlt001_raw_snapshot_config()
{

	int ret = 0;
	DDBG("dlt001_raw_snapshot_config [S]\n");

	ret = dlt001_set_regs(dlt001_prepare_mode_capture);
	if (ret) {
		CDBG("dlt001_raw_snapshot_config failed\n");
		return ret;
	}
	if (dlt001_ctrl->scene == SENSOR_SCENE_TWILIGHT) {
		ret = dlt001_check_bsts(DLT001_BSTS_VAL, DLT001_POLLING_TIMEOUT_MSEC);
		if (ret) {
			CDBG("dlt001_monitor_config dlt001_check_bsts failed\n");
		}
	}

	DDBG("dlt001_raw_snapshot_config [E]\n");
	return TRUE;
}

/**
 * RAW snapshot start
 *
 */
static int32_t dlt001_raw_snapshot_start()
{
	int32_t i, ret;

	DDBG("dlt001_raw_snapshot_start [S]\n");

	/* if flash used, and autoflash_adjust() calculation was done */
	if (dlt001_ctrl->autoflash_assist_light_on && !dlt001_ctrl->autoflash_poll_reg_x2AA) {
		/** USE calculated values computed in autoflash_adjust() */
		DDBG(".. FLASH is on: sending AE/AWB offset values to register"
			" 0x282=%d, 0x445c=%d, 0x445e=%d\n",
			dlt001_ctrl->autoflash_cmds[0][1],
			dlt001_ctrl->autoflash_cmds[1][1], dlt001_ctrl->autoflash_cmds[2][1]);

		i = 0;
		for (i = 0; i < 3; i++) {
			ret = dlt001_i2c_write(dlt001_ctrl->autoflash_cmds[i][0],
						BYTE_2,
						(uint8_t *) &(dlt001_ctrl->autoflash_cmds[i][1]));

			if (ret) {
				CDBG("Failed writing I2C register 0x%4.4x: %d",
					dlt001_ctrl->autoflash_cmds[i][0], ret);

				return ret;
			}
		}
	}

	ret = dlt001_set_regs(dlt001_mode_capture);
	if (ret) {
		CDBG("dlt001_raw_snapshot_start set_regs failed\n");
		return ret;
	}

	DDBG("dlt001_raw_snapshot_start [E] ret[%d]\n", ret);
	dlt001_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	return ret;
}

/**
 * Snapshot config
 *
 */
static int32_t dlt001_snapshot_config()
{
	int32_t ret;

	DDBG("dlt001_snapshot_config [S]\n");

	/* Change to capture mode */
	ret = dlt001_set_regs(dlt001_mode_capture_YUV);
	if (ret) {
		CDBG("snapshot_config: set_regs failed\n");
		return ret;
	}

	DDBG("dlt001_snapshot_config [E] ret[%d]\n", ret);
	dlt001_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	return ret;
}

/**
 * Half release config
 *
 */
static int32_t dlt001_half_release_config()
{
	int32_t ret;

	DDBG("dlt001_half_release_config [S]\n");

	/**
	* Only Flash LED related code, runs only if CFG_GET_AF_ASSIST_LIGHT was invoked
	*/
	if (dlt001_ctrl->autoflash_used) {
		/* only to make code readable */
		int onoff = dlt001_ctrl->autoflash_assist_light_on;

		ret = autoflash_enable(onoff);
		if (ret) {
			CDBG("autoflash_enable(%d) failed, ret: %d", onoff, ret);
		}

		ret = autoflash_strobe(onoff);
		if (ret) {
			CDBG("autoflash_strobe(%d) failed, ret: %d", onoff, ret);
		}
		/* if flash needed, then AF polling will continue equence, set flag to TRUE */
		if (onoff) {
			DDBG("autoflash_assist_light_on, set autoflash_poll_reg_x2AA to TRUE!\n");

			dlt001_ctrl->autoflash_poll_reg_x2AA = TRUE;
			dlt001_ctrl->aeawb_timeout = jiffies + 2 * HZ;
		}

	} else {
		dlt001_ctrl->autoflash_poll_reg_x2AA = FALSE;

		ret = autoflash_enable(FALSE);
		if (ret) {
			CDBG("autoflash_enable(OFF) failed, ret: %d", ret);
		}
	}

	if (dlt001_ctrl->scene == SENSOR_SCENE_TWILIGHT) {
		ret = dlt001_set_regs(dlt001_hr_twilight);
		if (ret) {
			CDBG("dlt001_hr_twilight failed\n");
			return ret;
		}
	} else if (dlt001_ctrl->scene == SENSOR_SCENE_AUTO) {
		if (dlt001_ctrl->autoflash_assist_light_on) {
			ret = dlt001_set_regs(dlt001_hr_LED);
			if (ret) {
				CDBG("dlt001_hr_LED failed\n");
				return ret;
			}
		} else {
			ret = dlt001_set_regs(dlt001_hr_auto_start);
			if (ret) {
				CDBG("dlt001_hr_auto_start failed\n");
				return ret;
			}
		}
	} else {
		ret = dlt001_set_regs(dlt001_mode_half_release);
		if (ret) {
			CDBG("dlt001_half_release_config\n");
			return ret;
		}
	}

	ret = dlt001_check_msts(DLT001_MSTS_HR_VAL, DLT001_POLLING_TIMEOUT_MSEC);
	if (ret) {
		CDBG("dlt001_half_release_config dlt001_check_msts failed\n");
		return ret;
	}

	dlt001_ctrl->dev_mode = CAMERA_MODE_HALF_RELEASE;

	DDBG("dlt001_half_relase_config [E]\n");

	return ret;
}

/**
 * Get AF status
 *
 */
static int32_t dlt001_get_af_status(enum camera_af_status *status)
{
	int8_t data = -1;
	int32_t ret;

	DDBG("dlt001_get_af_status [S]\n");

	*status = SENSOR_AF_IN_PROGRESS;

	if (dlt001_ctrl->autoflash_poll_reg_x2AA) {

		if (time_before(jiffies, dlt001_ctrl->aeawb_timeout)) {

			ret = autoflash_adjust();

			return ret;

		} else {
			CDBG(".. dlt001_get_af_status: TIMED OUT waiting for AWB/AE, do AF status poll only!");
		}
	}

	ret = dlt001_i2c_read(0x6D77, 1, &data);
	if (ret) {
		CDBG("dlt001_get_af_status failed\n");
	} else {
		if (data != 0x02) {
			if (dlt001_ctrl->scene == SENSOR_SCENE_TWILIGHT) {
				ret = dlt001_set_regs(dlt001_hr_reset);
				if (ret) {
					CDBG("dlt001_hr_reset failed\n");
					return ret;
				}
			}
			if (dlt001_ctrl->scene == SENSOR_SCENE_AUTO) {
				if (dlt001_ctrl->autoflash_assist_light_on) {
					ret = dlt001_set_regs(dlt001_hr_LED_reset);
					if (ret) {
						CDBG("dlt001_hr_LED_reset failed\n");
						return ret;
					}
				} else {
					ret = dlt001_set_regs(dlt001_hr_auto_reset);
					if (ret) {
						CDBG("dlt001_hr__auto_reset failed\n");
						return ret;
					}
				}
			}
		}
		switch (data) {
		case 0x00:
		default:
			*status = SENSOR_AF_FAILED;
			break;
		case 0x01:
			*status = SENSOR_AF_SUCCESS;
			break;
		case 0x02:
			*status = SENSOR_AF_IN_PROGRESS;
			break;
		}
	}

	DDBG("dlt001_get_af_status [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Get EXIF data
 *
 */
static int32_t dlt001_get_exif(struct cam_ctrl_exif_params *exif)
{
	int32_t ret = 0;
	int8_t data_8 = -1;
	uint16_t data_16_low = -1;
	uint16_t data_16_high = -1;

	DDBG("dlt001_get_exif [S]\n");

	ret = dlt001_i2c_read(0x00F0, BYTE_1, &data_8);
	if (ret) {
		CDBG("dlt001_get_exif failed\n");
		return ret;
	}

	exif->iso_speed_index = data_8;

	ret = dlt001_i2c_read(0x00F2, BYTE_2, (int8_t *) &data_16_low);
	if (ret) {
		CDBG("dlt001_get_exif failed\n");
		return ret;
	}

	ret = dlt001_i2c_read(0x00F4, BYTE_2, (int8_t *) &data_16_high);
	if (ret) {
		CDBG("dlt001_get_exif failed\n");
		return ret;
	}

	DDBG("data_16_low %d\n", data_16_low);
	DDBG("data_16_high %d\n", data_16_high);

	exif->shutter_speed = (data_16_high << SHIFT_16) | data_16_low;
	DDBG("Shutter speed %d us\n", exif->shutter_speed);

	exif->camera_revision = dlt001_ctrl->camera_revision;
	DDBG("Camera revision %d\n", exif->camera_revision);

	exif->flash_fired = dlt001_ctrl->autoflash_assist_light_on;
	DDBG("Flash fired %d\n", exif->flash_fired);

	DDBG("dlt001_get_exif [X]\n");
	return ret;
}

/**
 * Set test pattern on/off
 *
 */
static int32_t dlt001_set_test_pattern(enum set_test_pattern mode)
{
	int32_t ret;

	DDBG("dlt001_set_test_pattern [S]\n");

	if (mode == TEST_PATTERN_ON) {
		ret = dlt001_set_regs(dlt001_test_pattern_on);
	} else {
		ret = dlt001_set_regs(dlt001_test_pattern_off);
	}

	if (ret) {
		CDBG("dlt001_set_test_pattern send_reg_table failed\n");
		return ret;
	}

	DDBG("dlt001_set_test_pattern [E]\n");

	return ret;
}

static int32_t dlt001_update_scan_range(enum camera_focus_mode focus_mode, enum camera_scene scene)
{

	int32_t ret = 0;
	int update_register = 0;

	DDBG("dlt001_update_scan_range [S]\n");

	switch (scene) {
	case SENSOR_SCENE_AUTO:
		if (dlt001_ctrl->scan_range_reg != DLT001_SCAN_RANGE_REG_AUTO) {
			dlt001_ctrl->scan_range_reg = DLT001_SCAN_RANGE_REG_AUTO;
			update_register = 1;
		}
	break;
	case SENSOR_SCENE_TWILIGHT:
		if (dlt001_ctrl->scan_range_reg != DLT001_SCAN_RANGE_REG_TWILIGHT) {
			dlt001_ctrl->scan_range_reg = DLT001_SCAN_RANGE_REG_TWILIGHT;
			update_register = 1;
		}
	break;
	case SENSOR_SCENE_SPORTS:
		if (dlt001_ctrl->scan_range_reg != DLT001_SCAN_RANGE_REG_SPORTS) {
			dlt001_ctrl->scan_range_reg = DLT001_SCAN_RANGE_REG_SPORTS;
			update_register = 1;
		}
	break;
	default:
		CDBG("dlt001_update_scan_range scene %d not supported.\n", scene);
	break;
	}

	if (focus_mode == SENSOR_FOCUS_MODE_MACRO) {
		if (dlt001_ctrl->scan_range_val != DLT001_SCAN_RANGE_MACRO) {
			dlt001_ctrl->scan_range_val = DLT001_SCAN_RANGE_MACRO;
			update_register = 1;
		}
	} else {
		if (dlt001_ctrl->scan_range_val != DLT001_SCAN_RANGE_AUTO) {
			dlt001_ctrl->scan_range_val = DLT001_SCAN_RANGE_AUTO;
			update_register = 1;
		}
	}

	if (update_register == 1) {
		DDBG("dlt001_update_scan_range setting reg %d to value %d\n", dlt001_ctrl->scan_range_reg, dlt001_ctrl->scan_range_val);
		ret = dlt001_i2c_write(dlt001_ctrl->scan_range_reg, BYTE_2, (uint8_t *) &dlt001_ctrl->scan_range_val);
	}

	DDBG("dlt001_update_scan_range [E]\n");

	return ret;
}


/**
 * Set focus mode
 *
 */
static int32_t dlt001_set_focus_mode(enum camera_focus_mode focus_mode)
{
	int32_t ret = 0;

	DDBG("dlt001_set_focus_mode [S]\n");

	if (dlt001_ctrl->focus_mode != focus_mode) {
		dlt001_ctrl->focus_mode = focus_mode;
		if (dlt001_ctrl->init_complete) {
			ret = dlt001_update_focus_mode(focus_mode);
		}
	}
	DDBG("dlt001_set_focus_mode [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Update focus_mode
 *
 */
static int32_t dlt001_update_focus_mode(enum camera_focus_mode focus_mode)
{
	int32_t ret = 0;

	DDBG("dlt001_update_focus_mode [S]\n");

	switch (focus_mode) {
	case SENSOR_FOCUS_MODE_AUTO:
	case SENSOR_FOCUS_MODE_MACRO:
	default:
		DDBG("dlt001_update_focus_mode setting focus mode auto\n");
		ret = dlt001_set_regs(dlt001_focus_mode_auto);
		if (!ret) {
			ret = dlt001_set_regs(dlt001_primary_focus_window_auto);
			DDBG("dlt001_update_focus_mode setting focus window auto\n");
		}
		if (dlt001_ctrl->af_4838_val != dlt001_ctrl->calibration_data.af_d * 4) {
			dlt001_ctrl->af_4838_val = dlt001_ctrl->calibration_data.af_d * 4;
			dlt001_i2c_write(DLT001_AF_4838, BYTE_2, (uint8_t *) &dlt001_ctrl->af_4838_val);
		}
	break;

	case SENSOR_FOCUS_MODE_CONTINUOUS:
		DDBG("dlt001_update_focus_mode setting focus mode continuous\n");
		ret = dlt001_set_regs(dlt001_focus_mode_continuous);
		if (!ret) {
			ret = dlt001_set_regs(dlt001_primary_focus_window_continuous);
			DDBG("dlt001_update_focus_mode setting focus window continuous\n");
		}
		if (dlt001_ctrl->af_4838_val != dlt001_ctrl->calibration_data.af_d * 2) {
			dlt001_ctrl->af_4838_val = dlt001_ctrl->calibration_data.af_d * 2;
			dlt001_i2c_write(DLT001_AF_4838, BYTE_2, (uint8_t *) &dlt001_ctrl->af_4838_val);
		}
	break;

	case SENSOR_FOCUS_MODE_FIXED:
		CDBG("dlt001_update_focus_mode focus mode not supported by camera\n");
	break;
	}

	dlt001_update_scan_range(focus_mode, dlt001_ctrl->scene);

	DDBG("dlt001_update_focus_mode [E]\n");

	return ret;
}

/**
 * Set scene
 *
 */
static int32_t dlt001_set_scene(enum camera_scene scene)
{
	int32_t ret = 0;

	DDBG("dlt001_set_scene [S]\n");

	if (dlt001_ctrl->scene != scene) {
		dlt001_ctrl->scene = scene;
		if (dlt001_ctrl->init_complete) {
			ret = dlt001_update_scene(scene);
		}
	}

	DDBG("dlt001_set_scene [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Update scene
 *
 */
static int32_t dlt001_update_scene(enum camera_scene scene)
{
	int32_t ret = 0;

	DDBG("dlt001_update_scene [S]\n");
	switch (scene) {
	case SENSOR_SCENE_AUTO:
	default:
		DDBG("dlt001 Setting normal scene mode\n");
		ret = dlt001_set_regs(dlt001_scene_normal);
		break;
	case SENSOR_SCENE_TWILIGHT:
		DDBG("dlt001 Setting twilight scene mode\n");
		ret = dlt001_set_regs(dlt001_scene_twilight);
		break;
	case SENSOR_SCENE_SPORTS:
		DDBG("dlt001 Setting sports scene mode\n");
		ret = dlt001_set_regs(dlt001_scene_sports);
		break;
	}

	dlt001_update_scan_range(dlt001_ctrl->focus_mode, scene);

	DDBG("dlt001_update_scene [E] ret[%d]\n", ret);

	return ret;
}

static int dlt001_cam_is_assist_light_needed(int *result)
{
	int32_t ret = 0;
	uint16_t reg_x288, reg_x26A, reg_x26C;
	int16_t reg_x284;
	int A;

	DDBG("dlt001_cam_is_assist_light_needed [S]\n");

	ret = dlt001_i2c_read(0x288, BYTE_2, (int8_t *) &reg_x288);
	if (!ret)
		ret = dlt001_i2c_read(0x284, BYTE_2, (int8_t *) &reg_x284);
	if (!ret)
		ret = dlt001_i2c_read(0x26A, BYTE_2, (int8_t *) &reg_x26A);
	if (!ret)
		ret = dlt001_i2c_read(0x26C, BYTE_2, (int8_t *) &reg_x26C);

	DDBG("dlt001_cam_is_assist_light_needed, reg 0x288: %d, 0x284: %d, "
		"0x26A: %d, 0x26C: %d\n", reg_x288, reg_x284, reg_x26A, reg_x26C);

	if (ret) {
		CDBG("dlt001_cam_is_assist_light_needed failed, result=%d\n", ret);
		return ret;
	}

	A = reg_x288 + reg_x284;
	DDBG("dlt001_cam_is_assist_light_needed: A = (reg x288 + reg x284) = %d\n", A);

	dlt001_ctrl->autoflash_used = TRUE;
	dlt001_ctrl->autoflash_assist_light_on = (A < DLT001_FLASH_NEEDED_AE_MIN);
	dlt001_ctrl->autoflash_reg_x288 = reg_x288;
	dlt001_ctrl->autoflash_reg_x284 = reg_x284;
	dlt001_ctrl->autoflash_reg_x26A = reg_x26A;
	dlt001_ctrl->autoflash_reg_x26C = reg_x26C;

	*result = dlt001_ctrl->autoflash_assist_light_on;

	DDBG("dlt001_cam_is_assist_light_needed: %s, [E] ret[%d]\n", (*result ? "TRUE" : "FALSE"),
		ret);

	return ret;
}

static int32_t dlt001_set_dimensions(struct camera_dimension dimension)
{
	int32_t ret = 0;

	DDBG("dlt001_set_dimensions [S]\n");

	if (dimension.picture_width == 640 && dimension.picture_height == 480) {
		DDBG("Setting VGA snapshot size\n");
		ret = dlt001_set_regs(dlt001_snapshot_resolution_640x480);
	} else if (dimension.picture_width == 1280 && dimension.picture_height == 960) {
		DDBG("Setting 1MP snapshot size\n");
		ret = dlt001_set_regs(dlt001_snapshot_resolution_1280x960);
	} else {
		DDBG("Setting 5MP snapshot size, input %d x %d\n", dimension.picture_width,
		     dimension.picture_height);
		ret = dlt001_set_regs(dlt001_snapshot_resolution_2592x1944);
	}

	if (dimension.thumbnail_width == 640 && dimension.thumbnail_height == 480) {
		DDBG("Setting VGA thumbnail size\n");
		ret = dlt001_set_regs(dlt001_thumbnail_size_VGA);
	} else {
		DDBG("Setting QVGA thumbnail size, input %d x %d\n", dimension.thumbnail_width,
		     dimension.thumbnail_height);
		ret = dlt001_set_regs(dlt001_thumbnail_size_QVGA);
	}

	DDBG("dlt001_set_dimensions [E] ret[%d]\n", ret);

	return ret;
}

static int32_t dlt001_set_framerate(uint16_t fps)
{
	int32_t ret = 0;
	DDBG("dlt001_set_framerate [S]\n");

	if (fps == 0) {
		ret = dlt001_set_regs(dlt001_framerate_variable);
	} else if (fps == 30) {
		ret = dlt001_set_regs(dlt001_framerate_30);
	} else {
		CDBG("dlt001_set_framerate error, %d fps not supported by camera\n", fps);
		ret = -EFAULT;
	}

	DDBG("dlt001_set_framerate [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Access GPIO
*/
static int32_t dlt001_gpio_access(int gpio_pin, int dir)
{
	int rc = 0;

	DDBG("dlt001_gpio_access [S]\n");

	rc = gpio_request(gpio_pin, "dlt001_camera");
	if (!rc) {
		gpio_direction_output(gpio_pin, dir);
	}
	gpio_free(gpio_pin);

	DDBG("dlt001_gpio_access [E] rc[%d]\n", rc);

	return rc;
}

/**
 * Enable a resource (GPIO or VREG)
*/
static int32_t dlt001_resource_enable(struct msm_camera_sensor_pwr *resource)
{
	int32_t ret;

	if (!resource) {
		CDBG("dlt001_resource_enable argument is NULL.\n");
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		DDBG("dlt001_resource_enable GPIO[%d]\n", resource->resource.number);
		ret = dlt001_gpio_access(resource->resource.number, TRUE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		DDBG("dlt001_resource_enable VREG[%s]\n", resource->resource.name);
		ret = vreg_enable(vreg_get(0, resource->resource.name));
		break;
	default:
		CDBG("dlt001_resource_enable invalid resource type[%d]\n", resource->type);
		ret = 1;
		break;
	}

	return ret;
}

/**
 * Disable a resource (GPIO or VREG)
*/
static int32_t dlt001_resource_disable(struct msm_camera_sensor_pwr *resource)
{
	int32_t ret;

	if (!resource) {
		CDBG("dlt001_resource_disable argument is NULL.\n");
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		DDBG("dlt001_resource_disable GPIO[%d]\n", resource->resource.number);
		ret = dlt001_gpio_access(resource->resource.number, FALSE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		DDBG("dlt001_resource_disable VREG[%s]\n", resource->resource.name);
		ret = vreg_disable(vreg_get(0, resource->resource.name));
		break;
	default:
		CDBG("dlt001_resource_disable invalid resource type[%d]\n", resource->type);
		ret = 1;
		break;
	}

	return ret;
}

/**
 * Power on sensor
 *
 */
static int32_t dlt001_sensor_on(void)
{
	int32_t ret = 0;

	DDBG("dlt001_sensor_on [S]\n");

	/* Power on VCAM_SD12(GPI117 = High) 1,2V Core */
	ret = dlt001_resource_enable(&dlt001_ctrl->sensordata->vcam_sd12);
	if (ret) {
		CDBG("dlt001_sensor_on Power on VCAM_SD12 failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on VCAM_IO(PM7540/REG_GP4) 2,6V */
	ret = dlt001_resource_enable(&dlt001_ctrl->sensordata->vcam_io);
	if (ret) {
		CDBG("dlt001_sensor_on Power on VCAM_IO failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on VCAM_SA28(PM7540/RFRX2) */
	ret = dlt001_resource_enable(&dlt001_ctrl->sensordata->vcam_sa28);
	if (ret) {
		CDBG("dlt001_sensor_on Power on VCAM_SA28 failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on AF(PM7540/RFTX) */
	ret = dlt001_resource_enable(&dlt001_ctrl->sensordata->vcam_af30);
	if (ret) {
		CDBG("dlt001_sensor_on Power on VCAM_AF failed\n");
		return ret;
	}

	mdelay(5);

	DDBG("dlt001_sensor_on [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Initialize sensor
 *
 */
static int32_t dlt001_sensor_init(void)
{
	int32_t ret = 0;

	DDBG("dlt001_sensor_init [S]\n");

	if (dlt001_ctrl->camera_revision == DLT001_CAM_REV_ES1) {
		ret = dlt001_set_regs(dlt001_GEN_period_1_ES1);
		if (ret) {
			CDBG("dlt001_set_regs dlt001_GEN_Period_1_ES1 failed\n");
		}
	} else {
        ret = dlt001_set_regs(dlt001_GEN_period_1_ES2);
		mdelay(5);
		if (ret) {
			CDBG("dlt001_set_regs dlt001_GEN_Period_1_ES2 failed\n");
		}
		ret = dlt001_write_calibration_data(dlt001_ctrl->calibration_data);
		if (ret) {
			CDBG("dlt001_write_calibration_data failed\n");
		}
	}

	dlt001_ctrl->dev_mode = CAMERA_MODE_MONITOR;
	dlt001_ctrl->scene = SENSOR_SCENE_AUTO;
	dlt001_ctrl->focus_mode = SENSOR_FOCUS_MODE_AUTO;
	dlt001_ctrl->init_complete = 0;
	dlt001_ctrl->af_4838_val = dlt001_ctrl->calibration_data.af_l;
	dlt001_ctrl->scan_range_reg = DLT001_SCAN_RANGE_REG_AUTO;
	dlt001_ctrl->scan_range_val = DLT001_SCAN_RANGE_AUTO;

	DDBG("dlt001_sensor_init [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Power off sensor
 *
 */
static void dlt001_sensor_off(void)
{
	int32_t ret = 0;

	DDBG("dlt001_sensor_off [S]\n");

	/* Power off STANDBY (GPIO2 = LOW) */
	ret = dlt001_resource_disable(&dlt001_ctrl->sensordata->standby);
	if (ret) {
		CDBG("dlt001_sensor_off Power off STANDBY failed\n");
	}

	mdelay(5);

	/* CAM_RESET_N release(GPIO0 = LOW) */
	ret = dlt001_gpio_access(dlt001_ctrl->sensordata->sensor_reset, FALSE);
	if (ret) {
		CDBG("dlt001_sensor_off CAM_RESET_N release failed\n");
	}

	msleep(20);

	/* Output CAM_MCLK(0MHz) */
	msm_camio_clk_rate_set(0);

	msm_camio_clk_disable(CAMIO_VFE_CLK);

	mdelay(5);

	/* Power off VCAM_AF */
	ret = dlt001_resource_disable(&dlt001_ctrl->sensordata->vcam_af30);
	if (ret) {
		CDBG("dlt001_sensor_off Power off VCAM_AF failed\n");
	}
	mdelay(5);
	/* Power off VCAM_L2(GPIO43 = LOW) */
	ret = dlt001_resource_disable(&dlt001_ctrl->sensordata->vcam_sa28);
	if (ret) {
		CDBG("dlt001_sensor_off Power off VCAM_L2 failed\n");
	}
	msleep(250);
	/* Power off VCAM_IO(PM7540/REG_RFRX2) */
	ret = dlt001_resource_disable(&dlt001_ctrl->sensordata->vcam_io);
	if (ret) {
		CDBG("dlt001_sensor_off Power off VCAM_IO failed\n");
	}

	msleep(150);

	/* Power off VCAM_SD(GPIO142 = LOW) */
	ret = dlt001_resource_disable(&dlt001_ctrl->sensordata->vcam_sd12);
	if (ret) {
		CDBG("dlt001_sensor_off Power off VCAM_SD failed\n");
	}

	DDBG("dlt001_sensor_off [E]\n");
}

/**
 * Open Processing.
 *
 */
static int dlt001_sensor_open(const struct msm_camera_sensor_info *data)
{
	int32_t ret = 0;

	down(&dlt001_sem);

	DDBG("dlt001_open [S]\n");

	if (dlt001_ctrl->opened) {
		CDBG("dlt001_open already opened\n");
		ret = 0;
		goto open_done;
	}

	ret = dlt001_sensor_on();
	if (ret) {
		CDBG("dlt001_open sensor_on failed\n");
		goto open_done;
	}

	msm_camio_clk_enable(CAMIO_VFE_CLK);

	/* Output CAM_MCLK(19.2MHz) */
	msm_camio_clk_rate_set(DLT001_DEFAULT_CLOCK_RATE);

	msm_camio_camif_pad_reg_reset();

	msleep(40);

	/* CAM_RESET_N release(GPI89 = High) */
	ret = dlt001_gpio_access(dlt001_ctrl->sensordata->sensor_reset, TRUE);
	if (ret) {
		CDBG("dlt001_sensor_open CAM_RESET_N release failed\n");
		goto open_done;
	}

	msleep(20);

	ret = dlt001_sensor_init();
	if (ret) {
		CDBG("dlt001_open sensor_init failed\n");
		goto open_done;
	}

	mdelay(5);

	/* STANDBY (GPIO0 = High) */
	ret = dlt001_resource_enable(&dlt001_ctrl->sensordata->standby);
	mdelay(10);
	if (ret) {
		CDBG("dlt001_sensor_open STANDBY failed\n");
		goto open_done;
	}

 open_done:

	if (ret) {
		dlt001_sensor_off();
		ret = -EFAULT;
	} else {
		dlt001_ctrl->opened = 1;
	}

	up(&dlt001_sem);

	DDBG("dlt001_open [E]\n");

	return ret;
}

/**
 * Probe Processing.
 *
 */
static int dlt001_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	DDBG("dlt001_probe [S]\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("dlt001_probe i2c_check_functionality failed\n");
		kfree(dlt001_ctrl->sensorw);
		dlt001_ctrl->sensorw = NULL;

		return -ENOTSUPP;
	}

	dlt001_ctrl->sensorw = kzalloc(sizeof(struct dlt001_work), GFP_KERNEL);

	if (NULL == dlt001_ctrl->sensorw) {
		CDBG("dlt001_probe sensorw failed\n");
		kfree(dlt001_ctrl->sensorw);
		dlt001_ctrl->sensorw = NULL;

		return -ENOMEM;
	}

	i2c_set_clientdata(client, dlt001_ctrl->sensorw);

	dlt001_ctrl->client = client;

	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&dlt001_wait_queue);

	DDBG("dlt001_probe [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Remove Processing.
 *
 */
static int __exit dlt001_i2c_remove(struct i2c_client *client)
{
	struct dlt001_work_t *sensorw = i2c_get_clientdata(client);

	DDBG("dlt001_remove [S]\n");

	free_irq(client->irq, sensorw);
	dlt001_ctrl->client = NULL;
	kfree(sensorw);

	DDBG("dlt001_remove [E]\n");

	return 0;
}

/**
 * Single or Sequential Write to Random Location.
 *
 */
static int32_t dlt001_i2c_write(uint16_t address, uint8_t data_length, const uint8_t *data)
{
	uint8_t i2c_package[DLT001_I2C_MAX_BYTES + DLT001_I2C_WRITE_FOOTPRINT];
	uint8_t package_length = 0;

	if (data_length > DLT001_I2C_MAX_BYTES) {
		CDBG("dlt001_i2c_write length[%d] failed\n", data_length);
		return -EFAULT;
	}
	/* Add 2 byte Register Address in switched endian */
	dlt001_add_bytes_in_switched_endian(i2c_package, 0, DLT001_I2C_WRITE_FOOTPRINT,
					    (uint8_t *) &address);
	if (data != NULL) {
		memcpy(i2c_package + DLT001_I2C_WRITE_FOOTPRINT, data, data_length);	/* Add Data */
	}

	package_length = DLT001_I2C_WRITE_FOOTPRINT + data_length;

	if (i2c_master_send(dlt001_ctrl->client, i2c_package, package_length) != package_length) {
		CDBG("dlt001_i2c_write i2c_master_send failed\n");
		return -EIO;
	}

	return 0;
}

/**
 * Single or Sequential Read to Random Location.
 *
 */

static int32_t dlt001_i2c_read(uint16_t address, uint8_t length, uint8_t *data)
{
	int32_t ret = 0;

	DDBG("dlt001_i2c_read [S]\n");

	if (!data) {
		CDBG("dlt001_i2c_read *data failed\n");
		return -EFAULT;
	}

	ret = dlt001_i2c_write(address, 0, NULL);
	if (ret < 0) {
		CDBG("dlt001_i2c_read i2c_write failed\n");
		return ret;
	}

	if (i2c_master_recv(dlt001_ctrl->client, data, length) < 0) {
		CDBG("dlt001_i2c_read i2c_master_recv failed\n");
		return -EIO;
	}

	DDBG("dlt001_i2c_read [E]\n");

	return 0;
}

/*
 *  Switches places of Most Significant Bit (MSB) and Least Significant  *
 *  Bit (LSB) before adding the data to the package buffer               *
*/
static void dlt001_add_bytes_in_switched_endian(uint8_t *p_package_buf, uint8_t pkg_position,
						uint8_t size_of_data, uint8_t *Data)
{
	int MSB_byte_number, byte_number;
	for (MSB_byte_number = size_of_data - 1, byte_number = 0;
	     MSB_byte_number >= 0; --MSB_byte_number, ++byte_number) {
		memcpy(p_package_buf + pkg_position + byte_number, Data + MSB_byte_number,
		       sizeof(uint8_t));
	}
}

static int32_t dlt001_send_reg_table(const struct reg_entry *table, uint32_t tables_to_send)
{
	int32_t ret = 0;
	uint32_t table_index = 0;
	uint32_t data_index = 0;
	uint32_t j;
	uint8_t data_buffer[DLT001_I2C_MAX_BYTES] = { 0 };
	uint32_t grouped_data = 0;	/* The amount of continued data to group with one i2c write */
	uint8_t grouped_tables = 0;

	DDBG("dlt001_send_reg_table [tables: %d] [S]\n", tables_to_send);

	if (!table || tables_to_send == 0) {
		CDBG("dlt001_send_reg_table *table tables_to_send[%d] failed\n", tables_to_send);
		return -EFAULT;
	}
	/* Will loop thru entire table and send all data, if possible data will be grouped */
	while (table_index < tables_to_send) {
		grouped_data = 0;
		grouped_tables = 0;

		switch (table[table_index].reg_bits) {
		case REG_BITS_8:
			{
				/* Gather the amount of data we can send in one i2c stream, if next address is one incremented
				   the i2c write will continue to write at next address automaticly when sending more data then
				   one register can hold. Only group 8 bits registers since we can't be sure that when writing
				   32 bits register that is is really 32 bits or 4 grouped 8 bits. */
				while (((table_index + grouped_tables + 1) < tables_to_send) &&	/* Only if there are more tables to be found */
				       (grouped_data < (DLT001_I2C_MAX_BYTES - 1)) &&	/* Make sure we only send MAX allowed bytes */
				       ((table[table_index + grouped_tables + 1].address) ==	/* Only if next tables address is */
					(table[table_index + grouped_tables].address + 1)) &&	/* one incremented address as current */
				       (table[table_index + grouped_tables + 1].reg_bits == REG_BITS_8)	/* Only if next table is the same amount of bit holder */
				    ) {
					grouped_data++;
					grouped_tables++;
				}

				/* Load all tables, default to one */
				for (j = 0; j < grouped_tables + 1; j++)
					data_buffer[j] =
					    table[table_index + j].data & LOW_LOW_BIT_32;
			}
			break;
		case REG_BITS_16:
			while (((table_index + grouped_tables + 1) < tables_to_send) &&	/* Only if there are more tables to be found */
			       (grouped_data < (DLT001_I2C_MAX_BYTES - 2)) &&	/* Make sure we only send MAX allowed bytes */
			       ((table[table_index + grouped_tables + 1].address) ==	/* Only if next tables address is */
				(table[table_index + grouped_tables].address + 2)) &&	/* one incremented address as current */
			       (table[table_index + grouped_tables + 1].reg_bits == REG_BITS_16)	/* Only if next table is the same amount of bit holder */
			    ) {
				grouped_data += 2;
				grouped_tables++;
			}

			/* Load all tables, default to one */
			for (j = 0, data_index = 0; j < grouped_tables + 1; j++, data_index += 2) {
				data_buffer[data_index] =
				    (table[table_index + j].data & LOW_HIGH_BIT_32) >> SHIFT_8;
				data_buffer[data_index + 1] =
				    (table[table_index + j].data & LOW_LOW_BIT_32);
			}

			grouped_data++;	/* To hold the "extra" byte compared to REG_BITS_8 */
			break;
		case REG_BITS_32:
			while (((table_index + grouped_tables + 1) < tables_to_send) &&	/* Only if there are more tables to be found */
			       (grouped_data < (DLT001_I2C_MAX_BYTES - 4)) &&	/* Make sure we only send MAX allowed bytes */
			       ((table[table_index + grouped_tables + 1].address) ==	/* Only if next tables address is */
				(table[table_index + grouped_tables].address + 4)) &&	/* one incremented address as current */
			       (table[table_index + grouped_tables + 1].reg_bits == REG_BITS_32)	/* Only if next table is the same amount of bit holder */
			    ) {
				grouped_data += 4;
				grouped_tables++;
			}

			/* Load all tables, default to one */
			for (j = 0, data_index = 0; j < grouped_tables + 1; j++, data_index += 4) {
				data_buffer[data_index] =
				    (table[table_index + j].data & HIGH_HIGH_BIT_32) >> SHIFT_24;
				data_buffer[data_index + 1] =
				    (table[table_index + j].data & HIGH_LOW_BIT_32) >> SHIFT_16;
				data_buffer[data_index + 2] =
				    (table[table_index + j].data & LOW_HIGH_BIT_32) >> SHIFT_8;
				data_buffer[data_index + 3] =
				    (table[table_index + j].data & LOW_LOW_BIT_32);
			}

			grouped_data += 3;	/* To hold the "three extra" bytes compared to REG_BITS_8 */
			break;
		default:
			CDBG("dlt001_send_reg_table wrong reg_bits\n");
			break;
		}

		ret = dlt001_i2c_write(table[table_index].address,
				       grouped_data + 1, &data_buffer[0]);
		if (ret) {
			CDBG("dlt001_send_reg_table i2c_write failed\n");
			break;
		}
		mdelay(4);
		table_index += grouped_tables + 1;
	}

	DDBG("dlt001_send_reg_table [E] ret[%d]\n", ret);

	return ret;
}

static int32_t dlt001_check_msts(uint8_t value, uint32_t timeout_msec)
{
	char data;
	int32_t ret = 0;
	unsigned long timeout_jiffies;

	timeout_jiffies = jiffies + msecs_to_jiffies(timeout_msec);

	while (time_before(jiffies, timeout_jiffies)) {
		ret = dlt001_i2c_read(DLT001_USERCTRL_0010, BYTE_1, &data);
		if (ret) {
			CDBG("dlt001_check_msts: i2c_read failed\n");
			return ret;
		}

		DDBG("dlt001_check_msts: %X\n", data);
		if (data == value) {
			DDBG("dlt001_check_msts match %X\n", value);
			return 0;
		}
		msleep(DLT001_POLLING_SLEEP_MSEC);
	}

	CDBG("dlt001_check_msts: timeout \n");
	return 1;
}

static int32_t dlt001_check_bsts(uint8_t value, uint32_t timeout_msec)
{
	char data;
	int32_t ret = 0;
	unsigned long timeout_jiffies;

	timeout_jiffies = jiffies + msecs_to_jiffies(timeout_msec);

	while (time_before(jiffies, timeout_jiffies)) {
		ret = dlt001_i2c_read(DLT001_USERCTRL_0004, BYTE_1, &data);
		if (ret) {
			CDBG("dlt001_check_bsts: i2c_read failed\n");
			return ret;
		}

		DDBG("dlt001_check_bsts: %X\n", data);
		if (data == value) {
			DDBG("dlt001_check_bsts match %X\n", value);
			return 0;
		}
		msleep(DLT001_POLLING_SLEEP_MSEC);
	}

	CDBG("dlt001_check_bsts: timeout \n");
	return 1;
}

static int32_t dlt001_check_afsts(uint8_t value, uint32_t timeout_msec)
{
	char data;
	int32_t ret = 0;
	unsigned long timeout_jiffies;

	timeout_jiffies = jiffies + msecs_to_jiffies(timeout_msec);

	while (time_before(jiffies, timeout_jiffies)) {

		ret = dlt001_i2c_read(DLT001_SOUT_6D76, BYTE_1, &data);
		if (ret) {
			CDBG("dlt001_check_afsts: i2c_read failed\n");
			return ret;
		}

		DDBG("dlt001_check_afsts: %X\n", data);
		if (data == value) {
			DDBG("dlt001_check_afsts match %X\n", value);
			return 0;
		}

		msleep(DLT001_POLLING_SLEEP_MSEC);
	}

	CDBG("dlt001_check_afsts: timeout \n");
	return 1;
}

static int autoflash_enable(int onoff)
{
	const uint16_t cmds[3][2] = { {0x027D, 0x05}, {0x028C, 0x01}, {0x0097, 0x02} };
	uint8_t zero;
	int nret, i;

	nret = -1;
	zero = 0;

	for (i = 0; i < 3; i++) {

		if (onoff) {
			/* enable */
			nret = dlt001_i2c_write(cmds[i][0], BYTE_1, (uint8_t *) &(cmds[i][1]));
		} else {
			/* disable - all are 0 */
			nret = dlt001_i2c_write(cmds[i][0], BYTE_1, &zero);
		}

		/* break if ioctl fails */
		if (nret) {
			break;
		}
	}

	return nret;
}

static int autoflash_adjust()
{
	static const uint16_t CMDS[3][2] = { {0x0282, 0xffff}, {0x445c, 0xffff}, {0x445e, 0xffff} };
	uint8_t reg_x2AA;
	uint16_t reg_x28A;
	int16_t reg_x286;
	uint16_t reg_x26E;
	uint16_t reg_x270;
	uint16_t reg_6C26;
	uint16_t reg_6C28;
	int nret, AEO, SR, SB;

	nret = dlt001_i2c_read(0x02AA, BYTE_1, &reg_x2AA);

	if (nret) {
		CDBG("autoflash_adjust: Failed reading I2C reg 0x02AA: %d", nret);
		return nret;
	}

	if (reg_x2AA != 0) {
		DDBG("autoflash_adjust: Camera is still busy, reg. 0x02AA=%d\n", reg_x2AA);
		return 0;
	}

	reg_x28A = 0;
	reg_x286 = 0;
	reg_x26E = 0;
	reg_x270 = 0;
	reg_6C26 = 0;
	reg_6C28 = 0;
	AEO = 0;
	SR = 0;
	SB = 0;

	DDBG("autoflash_adjust: Camera finished AE adjust, reg. 0x02AA=%d\n", reg_x2AA);

	if ((nret = dlt001_i2c_read(0x28A, BYTE_2, (uint8_t *) &reg_x28A)) ||
		(nret = dlt001_i2c_read(0x286, BYTE_2, (uint8_t *) &reg_x286)) ||
		(nret = dlt001_i2c_read(0x26E, BYTE_2, (uint8_t *) &reg_x26E)) ||
		(nret = dlt001_i2c_read(0x270, BYTE_2, (uint8_t *) &reg_x270)) ||
		(nret = dlt001_i2c_read(0x6C26, BYTE_2, (uint8_t *) &reg_6C26)) ||
		(nret = dlt001_i2c_read(0x6C28, BYTE_2, (uint8_t *) &reg_6C28))) {
		CDBG("autoflash_adjust: Failed reading camera I2C register: %d", nret);

		return nret;
	}

	DDBG(".. set autoflash_poll_reg_x2AA to FALSE!\n");
	dlt001_ctrl->autoflash_poll_reg_x2AA = FALSE;
	memcpy(&(dlt001_ctrl->autoflash_cmds[0][0]), CMDS, sizeof(CMDS));

	{
		int32_t A, B, C, D, E, F, RF, RF2, K, RM;
		int32_t R1, R2, KL;

		A = dlt001_ctrl->autoflash_reg_x288 + dlt001_ctrl->autoflash_reg_x284;
		C = dlt001_ctrl->autoflash_reg_x26A;
		D = dlt001_ctrl->autoflash_reg_x26C;
		B = reg_x28A + reg_x286;
		E = reg_x26E;
		F = reg_x270;

		R1 = 4000;
		R2 = 10000;
		KL = 500;
		RF2 = 0;
		RM = 0;

		if ((B - A) >= 5000) {
			AEO = -2317 - reg_x286;
		} else {
			AEO = -AEO_table[(B - A) / 10] - reg_x286;
		}
		RF = (1000 * (B - A)) / A;
		if (RF > 50) {
			RM = ((373 * RF * RF) - (4137 * RF)) / 1000 + 224;
			if (RM < R1) {
				K = 1000;
			} else {
				if (RM < R2) {
					K = 1000 -  ((RM - R1) * (1000 - KL)) / (R2 - R1);
				} else {
					K = 500;
				}
			}
			SR = K * (1172 - C * 334 / 1000) / 1000 - 200;
			SB = (1788 - D * 345 / 1000) / 1000;
		} else {
			RF2 = (18313 * (B - A)) / A;
			if (RF2 > 100) {
				K = 1000;
			} else {
				if (RF2 > 50) {
					K = 20 * RF2 - 1000;
				} else {
					K = 0;
				}
			}

			SR = K * (972 - C * 334 / 1000) / 1000;
			SB = K * (1788 - D * 345 / 1000) / 1000;
		}

		DDBG("autoflash_adjust: Rf %d\n", RF);
		DDBG("autoflash_adjust: Rf2 %d\n", RF2);
		DDBG("autoflash_adjust: REG A %d B %d C %d D %d E %d F %d\n", A, B, C, D, E, F);
		DDBG("autoflash_adjust: SR %d\n", SR);
		DDBG("autoflash_adjust: SB %d\n", SB);
	}

	/* these are stored in device header structure, and reused in .._raw_snapshot_config */
	dlt001_ctrl->autoflash_cmds[0][1] = AEO;
	dlt001_ctrl->autoflash_cmds[1][1] = SR;
	dlt001_ctrl->autoflash_cmds[2][1] = SB;

	return 0;
}


static int autoflash_strobe(int onoff)
{
	int nret;
	uint8_t tmp;

	tmp = (onoff ? 0x09 : 0x08);

	nret = dlt001_i2c_write(0x069, BYTE_1, &tmp);
	return nret;
}

static int __dlt001_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, dlt001_camera_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __dlt001_probe,
	.driver = {
			.name = "msm_camera_dlt001",
			.owner = THIS_MODULE,
			},
};

static int __init dlt001_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(dlt001_init);

static int init_thread(void *data)
{
	int32_t ret = 0;

	CDBG("Camera init thread started\n");

	down(&dlt001_sem);

	if (dlt001_ctrl->vendor_id == VENDOR_ID_0) {
		if (dlt001_ctrl->camera_revision == DLT001_CAM_REV_ES1) {
			ret = dlt001_set_regs(dlt001_vendor_0_period_2_ES1);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_KM0_period_2_ES1 failed\n");
			}

			ret = dlt001_set_regs(dlt001_vendor_0_period_3_ES1);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_0_period_3_ES1 failed\n");
			}
		} else {
            if (dlt001_ctrl->calibration_data.shd_index == 1) {
				ret = dlt001_set_regs(dlt001_vendor_0_SHD_1_ES2);
			}
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_0_SHD_1_ES2 failed\n");
			}
			if (dlt001_ctrl->calibration_data.shd_index == 2) {
				ret = dlt001_set_regs(dlt001_vendor_0_SHD_2_ES2);
			}
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_0_SHD_2_ES2 failed\n");
			}
			if (dlt001_ctrl->calibration_data.shd_index == 3) {
				ret = dlt001_set_regs(dlt001_vendor_0_SHD_3_ES2);
			}
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_0_SHD_3_ES2 failed\n");
			}

			ret = dlt001_set_regs(dlt001_vendor_0_period_2_ES2);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_0_period_2_ES2 failed\n");
			}

			ret = dlt001_set_regs(dlt001_vendor_0_period_3_ES2);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_0_period_3_ES2 failed\n");
			}
		}

	} else if (dlt001_ctrl->vendor_id == VENDOR_ID_1) {
		if (dlt001_ctrl->camera_revision == DLT001_CAM_REV_ES1) {
			ret = dlt001_set_regs(dlt001_vendor_1_period_2_ES1);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_period_2_ES1 failed\n");
			}

			ret = dlt001_set_regs(dlt001_vendor_1_period_3_ES1);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_period_3_ES1 failed\n");
			}
		} else {
			if (dlt001_ctrl->calibration_data.shd_index == 1) {
				ret = dlt001_set_regs(dlt001_vendor_1_SHD_1_ES2);
			}
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_SHD_1_ES2 failed\n");
			}
			if (dlt001_ctrl->calibration_data.shd_index == 2) {
				ret = dlt001_set_regs(dlt001_vendor_1_SHD_2_ES2);
			}
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_SHD_2_ES2 failed\n");
			}
			if (dlt001_ctrl->calibration_data.shd_index == 3) {
				ret = dlt001_set_regs(dlt001_vendor_1_SHD_3_ES2);
			}
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_SHD_3_ES2 failed\n");
			}
			ret = dlt001_set_regs(dlt001_vendor_1_period_2_ES2);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_period_2_ES2 failed\n");
			}

			ret = dlt001_set_regs(dlt001_vendor_1_period_3_ES2);
			if (ret) {
				CDBG("dlt001_set_regs dlt001_vendor_1_period_3_ES2 failed\n");
			}
		}
	}
	ret = dlt001_update_scene(dlt001_ctrl->scene);
	ret = dlt001_update_focus_mode(dlt001_ctrl->focus_mode);
	dlt001_ctrl->init_complete = 1;

	up(&dlt001_sem);

	CDBG("Camera init thread end");
	return ret;

}
