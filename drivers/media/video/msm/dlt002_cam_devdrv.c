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

#include "dlt002_cam_devdrv.h"
#include "dlt002_cam_devdrv_table.h"

#ifdef CDBG
#undef CDBG
#endif

#define CDBG(fmt, args...) printk(KERN_INFO "msm_camdrv: " fmt, ##args)

#ifdef DDBG
#undef DDBG
#endif

#define DDBG(fmt, args...)

/* ******** Local functions ************* */
static int32_t dlt002_gpio_access(int gpio_pin, int dir);
static int32_t dlt002_resource_enable(struct msm_camera_sensor_pwr *resource);
static int32_t dlt002_resource_disable(struct msm_camera_sensor_pwr *resource);
static int32_t dlt002_sensor_on(void);
static int32_t dlt002_sensor_init(void);
static void dlt002_sensor_off(void);
static int dlt002_sensor_open(const struct msm_camera_sensor_info *data);
static int dlt002_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit dlt002_i2c_remove(struct i2c_client *client);
static int32_t dlt002_i2c_write(uint16_t address, uint8_t data_length, const uint8_t *data);
static int32_t dlt002_i2c_read(uint16_t address, uint8_t length, uint8_t *data);
static int32_t dlt002_set_sensor_mode(uint8_t mode);
static int32_t dlt002_monitor_config(void);
static int32_t dlt002_raw_snapshot_config(void);
static int32_t dlt002_raw_snapshot_start(void);
static int32_t dlt002_snapshot_config(void);
static int32_t dlt002_set_test_pattern(enum set_test_pattern mode);
static int32_t dlt002_set_scene(enum camera_scene scene);
static int32_t dlt002_update_scene(enum camera_scene scene);
static int32_t dlt002_set_dimensions(struct camera_dimension dimension);
static int32_t dlt002_get_af_status(enum camera_af_status *status);
static int32_t dlt002_check_bsts(uint8_t value, uint32_t timeout_msec);
static int32_t dlt002_get_exif(struct cam_ctrl_exif_params *exif);
static int32_t dlt002_read_vendor_data(void);
static void dlt002_add_bytes_in_switched_endian(uint8_t *p_package_buf, uint8_t pkg_position,
						uint8_t size_of_data, uint8_t *Data);
static int32_t dlt002_send_reg_table(const struct reg_entry *table, uint32_t tables_to_send);
static int32_t dlt002_check_msts(uint8_t value, uint32_t timeout_msec);
static int32_t dlt002_set_framerate(uint16_t fps);
static int init_thread(void *data);

/* ********** Local variables/structs ************ */

static struct dlt002_ctrl *dlt002_ctrl = NULL;
static DECLARE_WAIT_QUEUE_HEAD(dlt002_wait_queue);
DECLARE_MUTEX(dlt002_sem);

/**
 * I2C Device ID Structure Body.
 *
 */
static const struct i2c_device_id dlt002_id[] = {
	{"dlt002_camera", 0},
	{}
};

/**
 * I2C Device Structure Body.
 *
 */
static struct i2c_driver dlt002_driver = {
	.id_table = dlt002_id,
	.probe = dlt002_i2c_probe,
	.remove = __exit_p(dlt002_i2c_remove),
	.driver = {
		   .name = "dlt002_camera",
		   },
};

/**
 * Precess IOCTL messages.
 *
 */
int dlt002_sensor_config(void __user *argp)
{
	int32_t ret;
	struct sensor_cfg_data cfg_data;

	DDBG("dlt002_sensor_config  [S]\n");

	ret = copy_from_user(&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data));
	if (ret) {
		CDBG("dlt002_sensor_config copy_from_user failed\n");
		return -EFAULT;
	}

	if (cfg_data.cfgtype == CFG_SET_MODE &&
		cfg_data.mode == SENSOR_PREVIEW_MODE &&
		dlt002_ctrl->dev_mode == CAMERA_MODE_MONITOR && dlt002_ctrl->init_complete) {
		DDBG("Already running in monitor");
		return 0;
	}

	down(&dlt002_sem);

	DDBG("dlt002_sensor_config cfgtype = %d\n", cfg_data.cfgtype);
	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		ret = dlt002_set_sensor_mode(cfg_data.mode);
		break;

	case CFG_SET_TEST_PATTERN:
		ret = dlt002_set_test_pattern(cfg_data.cfg.set_test_pattern);
		break;

	case CFG_GET_AF_STATUS:
		ret = dlt002_get_af_status(&cfg_data.cfg.af_status);
		break;

	case CFG_GET_EXIF_INFO:
		ret = dlt002_get_exif(&cfg_data.cfg.exif);
		break;

	case CFG_SET_SCENE:
		ret = dlt002_set_scene(cfg_data.cfg.scene);
		break;

	case CFG_SET_FOCUS_MODE:
		/* Do nothing */
		break;

	case CFG_SET_DIMENSIONS:
		ret = dlt002_set_dimensions(cfg_data.cfg.dimension);
		break;

	case CFG_SET_FPS:
		ret = dlt002_set_framerate(cfg_data.cfg.fps.f_mult);
		break;

	default:
		CDBG("dlt002_sensor_config cfgtype failed\n");
		ret = -EFAULT;
		break;
	}

	up(&dlt002_sem);

	ret = copy_to_user((void *)argp, &cfg_data, sizeof(struct sensor_cfg_data));
	if (ret) {
		CDBG("dlt002_sensor_config copy_to_user failed\n");
		return -EFAULT;
	}

	DDBG("dlt002_sensor_config [E]\n");

	return ret;
}

/**
 * Release
 *
 */
int dlt002_sensor_release(void)
{

	DDBG("dlt002_sensor_release [S]\n");

	down(&dlt002_sem);

	if (dlt002_ctrl->opened) {
		dlt002_sensor_off();
		dlt002_ctrl->opened = 0;
	}

	dlt002_ctrl->dev_mode = CAMERA_MODE_STANDBY;

	up(&dlt002_sem);

	DDBG("dlt002_sensor_release [E]\n");

	return 0;
}

/**
 * Exit
 *
 */
void dlt002_exit(void)
{
	DDBG("dlt002_exit [S]\n");

	i2c_del_driver(&dlt002_driver);

	DDBG("dlt002_exit [E]\n");
}

/**
 * Probe
 *
 */
static int dlt002_camera_probe(const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *s)
{
	int rc = 0;

	DDBG("dlt002_camera_probe [S]\n");

	dlt002_ctrl = kzalloc(sizeof(struct dlt002_ctrl), GFP_KERNEL);
	if (NULL == dlt002_ctrl) {
		CDBG("dlt002_cam_probe memory allocation failed\n");
		goto probe_done;
	}

	if (info != NULL) {
		dlt002_ctrl->sensordata = (struct msm_camera_sensor_info *)info;
		rc = i2c_add_driver(&dlt002_driver);
		if (IS_ERR_VALUE(rc)) {
			kfree(dlt002_ctrl);
			goto probe_done;
		}
	}

	dlt002_ctrl->dev_mode = CAMERA_MODE_STANDBY;

	rc = dlt002_sensor_on();
	if (rc < 0) {
		CDBG("dlt002_camera_probe sensor_on failed\n");
		goto probe_done;
	}

	msm_camio_clk_enable(CAMIO_VFE_CLK);

	/* Output CAM_MCLK(19.2MHz) */
	msm_camio_clk_rate_set(DLT002_DEFAULT_CLOCK_RATE);

	msleep(40);

	/* CAM_RESET_N release(GPIO0 = High) */
	rc = dlt002_gpio_access(dlt002_ctrl->sensordata->sensor_reset, TRUE);
	if (rc) {
		CDBG("dlt002_probe CAM_RESET_N release failed\n");
		goto probe_done;
	}

	msleep(20);

	/* STANDBY (GPIO0 = High) */
	rc = dlt002_resource_enable(&dlt002_ctrl->sensordata->standby);
	if (rc) {
		CDBG("dlt002_probe STANDBY failed\n");
		goto probe_done;
	}

	mdelay(10);

	rc = dlt002_read_vendor_data();
	if (rc != 0) {
		CDBG("dlt002_read_vendor_data failed\n");
		goto probe_done;
	}

	CDBG("Camera vendor [%d]\n", dlt002_ctrl->vendor_id);
	CDBG("Camera revision[%d]\n", dlt002_ctrl->camera_revision);

	s->s_init = dlt002_sensor_open;
	s->s_release = dlt002_sensor_release;
	s->s_config = dlt002_sensor_config;

 probe_done:
	if (dlt002_ctrl) {
		dlt002_sensor_off();
	}

	DDBG("dlt002_camera_probe [E]\n");
	return rc;
}

static int32_t dlt002_read_vendor_data(void)
{
	int32_t rc = 0;
	uint32_t data = 0;
	uint8_t v_flag_otp0 = 0;
	uint8_t v_flag_otp1 = 0;
	enum dlt002_otp otp_id = OTP_NO_DATA_WRITTEN;

	rc = dlt002_i2c_read(DLT002_USERCTRL_0000, BYTE_2, (uint8_t *) &data);
	dlt002_ctrl->camera_revision = data & LOW_BIT_32;

	rc += dlt002_i2c_read(DLT002_USERCTRL_022C, BYTE_4, (uint8_t *) &data);
	if ((data & DLT002_V_FLAG_OTP_MASK) > 0)
		v_flag_otp0 = 1;

	rc += dlt002_i2c_read(DLT002_USERCTRL_0238, BYTE_4, (uint8_t *) &data);
	if ((data & DLT002_V_FLAG_OTP_MASK) > 0)
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
		CDBG("dlt002_read_vendor_data failed! No valid OTP\n");
		//return -1;
		//read OTP0 anyway - if properly written vendor selection will work - if = 0 vendor 1 will be set.
		rc += dlt002_i2c_read(DLT002_USERCTRL_0230, BYTE_4, (uint8_t *) &data);
		break;
	case OTP_0:
		rc += dlt002_i2c_read(DLT002_USERCTRL_0230, BYTE_4, (uint8_t *) &data);
		DDBG("dlt002 OTP0 Reg 0x0230 holds:[0x%X]\n", data);
		break;
	case OTP_1:
		rc += dlt002_i2c_read(DLT002_USERCTRL_023C, BYTE_4, (uint8_t *) &data);
		break;
	default:
		return -1;
	}

	if ((data & DLT002_VENDOR_ID_OTP_MASK) > 0) {
		dlt002_ctrl->vendor_id = VENDOR_ID_0;
	} else {
		dlt002_ctrl->vendor_id = VENDOR_ID_1;
	}

	return rc;
}

/**
 * Set sensor mode
 *
 */
static int32_t dlt002_set_sensor_mode(uint8_t mode)
{
	int32_t ret = 0;

	DDBG("dlt002_set_sensor_mode [S]\n");

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		ret = dlt002_monitor_config();
		break;
	case SENSOR_SNAPSHOT_MODE:
		ret = dlt002_snapshot_config();
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		ret = dlt002_raw_snapshot_config();
		break;
	case SENSOR_RAW_SNAPSHOT_START:
		ret = dlt002_raw_snapshot_start();
		break;
	default:
		CDBG("dlt002_set_sensor_mode mode failed\n");
		ret = -EINVAL;
		break;
	}

	DDBG("dlt002_set_sensor_mode [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Set monitor mode
 *
 */
static int32_t dlt002_monitor_config()
{
	int32_t ret = 0;

	DDBG("dlt002_monitor_config [S]\n");

	if (dlt002_ctrl->dev_mode == CAMERA_MODE_MONITOR) {
		DDBG("dlt002_monitor_config Already in monitor mode. Do nothing...\n");
		goto monitor_done;
	}

	ret = dlt002_set_regs(dlt002_mode_monitor);
	if (ret) {
		CDBG("dlt002_monitor_config failed\n");
		return ret;
	}

	ret = dlt002_check_msts(DLT002_MSTS_MON_VAL, DLT002_POLLING_TIMEOUT_MSEC);
	if (ret) {
		CDBG("dlt002_monitor_config dlt002_check_msts failed\n");
	}

	dlt002_ctrl->dev_mode = CAMERA_MODE_MONITOR;

 monitor_done:

	if (!dlt002_ctrl->init_complete) {
		kthread_run(init_thread, NULL, "sensor_init");
	}

	DDBG("dlt002_monitor_config [E] ret[%d]\n", ret);

	return ret;
}

/**
 * RAW snapshot config
 *
 */
static int32_t dlt002_raw_snapshot_config()
{

	int ret = 0;
	DDBG("dlt002_raw_snapshot_config [S]\n");

	ret = dlt002_set_regs(dlt002_prepare_mode_capture);
	if (ret) {
		CDBG("dlt002_raw_snapshot_config failed\n");
		return ret;
	}
	if (dlt002_ctrl->scene == SENSOR_SCENE_TWILIGHT) {
		ret = dlt002_check_bsts(DLT002_BSTS_VAL, DLT002_POLLING_TIMEOUT_MSEC);
		if (ret) {
			CDBG("dlt002_monitor_config dlt002_check_bsts failed\n");
		}
	}

	DDBG("dlt002_raw_snapshot_config [E]\n");
	return TRUE;
}

/**
 * RAW snapshot start
 *
 */
static int32_t dlt002_raw_snapshot_start()
{
	int32_t ret;

	DDBG("dlt002_raw_snapshot_start [S]\n");

	/* Change to capture mode */
	ret = dlt002_set_regs(dlt002_mode_capture);
	if (ret) {
		CDBG("dlt002_raw_snapshot_start send_reg_table failed\n");
		return ret;
	}

	DDBG("dlt002_raw_snapshot_start [E] ret[%d]\n", ret);
	dlt002_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	return ret;
}

/**
 * Snapshot config
 *
 */
static int32_t dlt002_snapshot_config()
{
	int32_t ret;

	DDBG("dlt002_snapshot_config [S]\n");

	/* Change to capture mode */
	ret = dlt002_set_regs(dlt002_mode_capture_YUV);
	if (ret) {
		CDBG("send_reg_table failed\n");
		return ret;
	}

	DDBG("dlt002_snapshot_config [E] ret[%d]\n", ret);
	dlt002_ctrl->dev_mode = CAMERA_MODE_CAPTURE;
	return ret;
}

/**
 * Get AF status
 *
 */
static int32_t dlt002_get_af_status(enum camera_af_status *status)
{
	DDBG("dlt002_get_af_status [S]\n");

	*status = SENSOR_AF_SUCCESS;

	DDBG("dlt002_get_af_status [E]\n");
	return 0;
}

/**
 * Get EXIF data
 *
 */
static int32_t dlt002_get_exif(struct cam_ctrl_exif_params *exif)
{
	int32_t ret = 0;
	int8_t data_8 = -1;
	uint16_t data_16_low = -1;
	uint16_t data_16_high = -1;

	DDBG("dlt002_get_exif [S]\n");

	ret = dlt002_i2c_read(0x00F0, BYTE_1, &data_8);
	if (ret) {
		CDBG("dlt002_get_exif failed\n");
		return ret;
	}

	exif->iso_speed_index = data_8;

	ret = dlt002_i2c_read(0x00F2, BYTE_2, (int8_t *) &data_16_low);
	if (ret) {
		CDBG("dlt002_get_exif failed\n");
		return ret;
	}

	ret = dlt002_i2c_read(0x00F4, BYTE_2, (int8_t *) &data_16_high);
	if (ret) {
		CDBG("dlt002_get_exif failed\n");
		return ret;
	}

	DDBG("data_16_low %d\n", data_16_low);
	DDBG("data_16_high %d\n", data_16_high);

	exif->shutter_speed = (data_16_high << SHIFT_16) | data_16_low;
	DDBG("Shutter speed %d us\n", exif->shutter_speed);

	exif->camera_revision = dlt002_ctrl->camera_revision;
	DDBG("Camera revision %d\n", exif->camera_revision);

	exif->flash_fired = FALSE;

	DDBG("dlt002_get_exif [X]\n");
	return ret;
}

/**
 * Set test pattern on/off
 *
 */
static int32_t dlt002_set_test_pattern(enum set_test_pattern mode)
{
	int32_t ret;

	DDBG("dlt002_set_test_pattern [S]\n");

	if (mode == TEST_PATTERN_ON) {
		ret = dlt002_set_regs(dlt002_test_pattern_on);
	} else {
		ret = dlt002_set_regs(dlt002_test_pattern_off);
	}

	if (ret) {
		CDBG("dlt002_set_test_pattern send_reg_table failed\n");
		return ret;
	}

	DDBG("dlt002_set_test_pattern [E]\n");

	return ret;
}

/**
 * Set scene
 *
 */
static int32_t dlt002_set_scene(enum camera_scene scene)
{
	int32_t ret = 0;

	DDBG("dlt002_set_scene [S]\n");

	if (dlt002_ctrl->scene != scene) {
		dlt002_ctrl->scene = scene;
		if (dlt002_ctrl->init_complete) {
			ret = dlt002_update_scene(scene);
		}
	}

	DDBG("dlt002_set_scene [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Set scene
 *
 */
static int32_t dlt002_update_scene(enum camera_scene scene)
{
	int32_t ret = 0;

	DDBG("dlt002_update_scene [S]\n");

	switch (scene) {
	case SENSOR_SCENE_AUTO:
	default:
		DDBG("dlt002 Setting normal scene mode\n");
		ret = dlt002_set_regs(dlt002_scene_normal);
		break;
	case SENSOR_SCENE_BEACH:
	case SENSOR_SCENE_SNOW:
		DDBG("dlt002 Setting beach&snow scene mode\n");
		ret = dlt002_set_regs(dlt002_scene_beach_snow);
		break;
	case SENSOR_SCENE_TWILIGHT:
		DDBG("dlt002 Setting twilight scene mode\n");
		ret = dlt002_set_regs(dlt002_scene_twilight);
		break;
	case SENSOR_SCENE_SPORTS:
		DDBG("dlt002 Setting sports scene mode\n");
		ret = dlt002_set_regs(dlt002_scene_sports);
		break;
	}

	DDBG("dlt002_update_scene [E] ret[%d]\n", ret);

	return ret;
}

static int32_t dlt002_set_dimensions(struct camera_dimension dimension)
{
	int32_t ret = 0;

	DDBG("dlt002_set_dimensions [S]\n");

	if (dimension.picture_width == 640 && dimension.picture_height == 480) {
		DDBG("Setting VGA snapshot size\n");
		ret = dlt002_set_regs(dlt002_snapshot_resolution_640x480);
	} else if (dimension.picture_width == 1280 && dimension.picture_height == 960) {
		DDBG("Setting 1MP snapshot size\n");
		ret = dlt002_set_regs(dlt002_snapshot_resolution_1280x960);
	} else if (dimension.picture_width == 1632 && dimension.picture_height == 1224) {
		DDBG("Setting 2MP snapshot size\n");
		ret = dlt002_set_regs(dlt002_snapshot_resolution_1632x1224);
	} else {
		DDBG("Setting 3MP snapshot size, input %d x %d\n",
		     dimension.picture_width, dimension.picture_height);
		ret = dlt002_set_regs(dlt002_snapshot_resolution_2048x1536);
	}

	if (dimension.thumbnail_width == 640 && dimension.thumbnail_height == 480) {
		DDBG("Setting VGA thumbnail size.\n");
		ret = dlt002_set_regs(dlt002_thumbnail_size_VGA);
	} else {
		DDBG("Setting QVGA thumbnail size. Input %d x %d\n",
		     dimension.thumbnail_width, dimension.thumbnail_height);
		ret = dlt002_set_regs(dlt002_thumbnail_size_QVGA);
	}

	DDBG("dlt002_set_dimensions [E] ret[%d]\n", ret);

	return ret;
}

static int32_t dlt002_set_framerate(uint16_t fps)
{
	int32_t ret = 0;
	DDBG("dlt002_set_framerate [S]\n");

	if (fps == 0) {
		ret = dlt002_set_regs(dlt002_framerate_variable);
	} else if (fps == 30) {
		ret = dlt002_set_regs(dlt002_framerate_30);
	} else {
		CDBG("dlt002_set_framerate error, %d fps not supported by camera\n", fps);
		ret = -EFAULT;
	}

	DDBG("dlt002_set_framerate [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Access GPIO
*/
static int32_t dlt002_gpio_access(int gpio_pin, int dir)
{
	int rc = 0;

	DDBG("dlt002_gpio_access [S]\n");

	rc = gpio_request(gpio_pin, "dlt002_camera");
	if (!rc) {
		gpio_direction_output(gpio_pin, dir);
	}
	gpio_free(gpio_pin);

	DDBG("dlt002_gpio_access [E] rc[%d]\n", rc);

	return rc;
}

/**
 * Enable a resource (GPIO or VREG)
*/
static int32_t dlt002_resource_enable(struct msm_camera_sensor_pwr
				      *resource)
{
	int32_t ret;

	if (!resource) {
		CDBG("dlt002_resource_enable argument is NULL.\n");
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		DDBG("dlt002_resource_enable GPIO[%d]\n", resource->resource.number);
		ret = dlt002_gpio_access(resource->resource.number, TRUE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		DDBG("dlt002_resource_enable VREG[%s]\n", resource->resource.name);
		ret = vreg_enable(vreg_get(0, resource->resource.name));
		break;
	default:
		CDBG("dlt002_resource_enable invalid resource type[%d]\n", resource->type);
		ret = 1;
		break;
	}

	return ret;
}

/**
 * Disable a resource (GPIO or VREG)
*/
static int32_t dlt002_resource_disable(struct msm_camera_sensor_pwr
				       *resource)
{
	int32_t ret;

	if (!resource) {
		CDBG("dlt002_resource_disable argument is NULL.\n");
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		DDBG("dlt002_resource_disable GPIO[%d]\n", resource->resource.number);
		ret = dlt002_gpio_access(resource->resource.number, FALSE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		DDBG("dlt002_resource_disable VREG[%s]\n", resource->resource.name);
		ret = vreg_disable(vreg_get(0, resource->resource.name));
		break;
	default:
		DDBG("dlt002_resource_disable invalid resource type[%d]\n", resource->type);
		ret = 1;
		break;
	}

	return ret;
}

/**
 * Power on sensor
 *
 */
static int32_t dlt002_sensor_on(void)
{
	int32_t ret = 0;

	DDBG("dlt002_sensor_on [S]\n");

	/* Power on VCAM_SD12(GPI117 = High) 1,2V Core */
	ret = dlt002_resource_enable(&dlt002_ctrl->sensordata->vcam_sd12);
	if (ret) {
		CDBG("dlt002_sensor_on Power on VCAM_SD12 failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on VCAM_IO(PM7540/REG_GP4) 2,6V */
	ret = dlt002_resource_enable(&dlt002_ctrl->sensordata->vcam_io);
	if (ret) {
		CDBG("dlt002_sensor_on Power on VCAM_IO failed\n");
		return ret;
	}

	mdelay(5);

	/* Power on VCAM_SA28(PM7540/RFRX2) */
	ret = dlt002_resource_enable(&dlt002_ctrl->sensordata->vcam_sa28);
	if (ret) {
		CDBG("dlt002_sensor_on Power on VCAM_SA28 failed\n");
		return ret;
	}

	mdelay(5);

	DDBG("dlt002_sensor_on [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Initialize sensor
 *
 */
static int32_t dlt002_sensor_init(void)
{
	int32_t ret = 0;

	DDBG("dlt002_sensor_init [S]\n");

	ret = dlt002_set_regs(dlt002_GEN_period_1_ES1);
	if (ret) {
		CDBG("dlt002_send_reg_table dlt002_GEN_period_1_ES1 failed\n");
	}

	dlt002_ctrl->dev_mode = CAMERA_MODE_MONITOR;
	dlt002_ctrl->scene = SENSOR_SCENE_AUTO;
	dlt002_ctrl->init_complete = 0;

	DDBG("dlt002_sensor_init [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Power off sensor
 *
 */
static void dlt002_sensor_off(void)
{
	int32_t ret = 0;

	DDBG("dlt002_sensor_off [S]\n");

	/* Power off STANDBY (GPIO2 = LOW) */
	ret = dlt002_resource_disable(&dlt002_ctrl->sensordata->standby);
	if (ret) {
		CDBG("dlt002_sensor_off Power off STANDBY failed\n");
	}

	mdelay(5);

	/* CAM_RESET_N release(GPIO0 = LOW) */
	ret = dlt002_gpio_access(dlt002_ctrl->sensordata->sensor_reset, FALSE);
	if (ret) {
		CDBG("dlt002_sensor_off CAM_RESET_N release failed\n");
	}

	msleep(20);

	/* Output CAM_MCLK(0MHz) */
	msm_camio_clk_rate_set(0);
	msm_camio_clk_disable(CAMIO_VFE_CLK);

	mdelay(5);

	/* Power off VCAM_L2(GPIO43 = LOW) */
	ret = dlt002_resource_disable(&dlt002_ctrl->sensordata->vcam_sa28);
	if (ret) {
		CDBG("dlt002_sensor_off Power off VCAM_L2 failed\n");
	}

	msleep(30);

	/* Power off VCAM_IO(PM7540/REG_RFRX2) */
	ret = dlt002_resource_disable(&dlt002_ctrl->sensordata->vcam_io);
	if (ret) {
		CDBG("dlt002_sensor_off Power off VCAM_IO failed\n");
	}

	msleep(5);

	/* Power off VCAM_SD(GPIO142 = LOW) */
	ret = dlt002_resource_disable(&dlt002_ctrl->sensordata->vcam_sd12);
	if (ret) {
		CDBG("dlt002_sensor_off Power off VCAM_SD failed\n");
	}

	DDBG("dlt002_sensor_off [E]\n");
}

/**
 * Open Processing.
 *
 */
static int dlt002_sensor_open(const struct msm_camera_sensor_info
			      *data)
{
	int32_t ret = 0;

	down(&dlt002_sem);

	DDBG("dlt002_open [S]\n");

	if (dlt002_ctrl->opened) {
		CDBG("dlt002_open already opened\n");
		ret = 0;
		goto open_done;
	}

	ret = dlt002_sensor_on();
	if (ret) {
		CDBG("dlt002_open sensor_on failed\n");
		goto open_done;
	}

	msm_camio_clk_enable(CAMIO_VFE_CLK);

	/* Output CAM_MCLK(19.2MHz) */
	msm_camio_clk_rate_set(DLT002_DEFAULT_CLOCK_RATE);

	msm_camio_camif_pad_reg_reset();	/* Moved from vfe_7x_init(...)*/

	msleep(40);

	/* CAM_RESET_N release(GPI89 = High) */
	ret = dlt002_gpio_access(dlt002_ctrl->sensordata->sensor_reset, TRUE);
	if (ret) {
		CDBG("dlt002_sensor_open CAM_RESET_N release failed\n");
		goto open_done;
	}

	msleep(20);

	ret = dlt002_sensor_init();
	if (ret) {
		CDBG("dlt002_open sensor_init failed\n");
		goto open_done;
	}

	mdelay(5);

	/* STANDBY (GPIO0 = High) */
	ret = dlt002_resource_enable(&dlt002_ctrl->sensordata->standby);
	mdelay(10);
	if (ret) {
		CDBG("dlt002_sensor_open STANDBY failed\n");
		goto open_done;
	}

 open_done:

	if (ret) {
		dlt002_sensor_off();
		ret = -EFAULT;
	} else {
		dlt002_ctrl->opened = 1;
	}

	up(&dlt002_sem);

	DDBG("dlt002_open [E]\n");

	return ret;
}

/**
 * Probe Processing.
 *
 */
static int dlt002_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	DDBG("dlt002_probe [S]\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("dlt002_probe i2c_check_functionality failed\n");
		kfree(dlt002_ctrl->sensorw);
		dlt002_ctrl->sensorw = NULL;

		return -ENOTSUPP;
	}

	dlt002_ctrl->sensorw = kzalloc(sizeof(struct dlt002_work), GFP_KERNEL);

	if (NULL == dlt002_ctrl->sensorw) {
		CDBG("dlt002_probe sensorw failed\n");
		kfree(dlt002_ctrl->sensorw);
		dlt002_ctrl->sensorw = NULL;

		return -ENOMEM;
	}

	i2c_set_clientdata(client, dlt002_ctrl->sensorw);

	dlt002_ctrl->client = client;

	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&dlt002_wait_queue);

	DDBG("dlt002_probe [E] ret[%d]\n", ret);

	return ret;
}

/**
 * Remove Processing.
 *
 */
static int __exit dlt002_i2c_remove(struct i2c_client *client)
{
	struct dlt002_work *sensorw = i2c_get_clientdata(client);

	DDBG("dlt002_remove [S]\n");

	free_irq(client->irq, sensorw);
	dlt002_ctrl->client = NULL;
	kfree(sensorw);

	DDBG("dlt002_remove [E]\n");

	return 0;
}

/**
 * Single or Sequential Write to Random Location.
 *
 */
static int32_t dlt002_i2c_write(uint16_t address, uint8_t data_length, const uint8_t *data)
{
	uint8_t i2c_package[DLT002_I2C_MAX_BYTES + DLT002_I2C_WRITE_FOOTPRINT];
	uint8_t package_length = 0;

	if (data_length > DLT002_I2C_MAX_BYTES) {
		CDBG("dlt002_i2c_write length[%d] failed\n", data_length);
		return -EFAULT;
	}
	/* Add 2 byte Register Address in switched endian */
	dlt002_add_bytes_in_switched_endian(i2c_package, 0,
					    DLT002_I2C_WRITE_FOOTPRINT, (uint8_t *) &address);
	if (data != NULL) {
		memcpy(i2c_package + DLT002_I2C_WRITE_FOOTPRINT, data, data_length);
	}

	package_length = DLT002_I2C_WRITE_FOOTPRINT + data_length;

	if (i2c_master_send(dlt002_ctrl->client, i2c_package, package_length) != package_length) {
		CDBG("dlt002_i2c_write i2c_master_send failed\n");
		return -EIO;
	}

	return 0;
}

/**
 * Single or Sequential Read to Random Location.
 *
 */

static int32_t dlt002_i2c_read(uint16_t address, uint8_t length, uint8_t *data)
{
	int32_t ret = 0;

	DDBG("dlt002_i2c_read [S]\n");

	if (!data) {
		CDBG("dlt002_i2c_read *data failed\n");
		return -EFAULT;
	}

	ret = dlt002_i2c_write(address, 0, NULL);
	if (ret < 0) {
		CDBG("dlt002_i2c_read i2c_write failed\n");
		return ret;
	}

	if (i2c_master_recv(dlt002_ctrl->client, data, length) < 0) {
		CDBG("dlt002_i2c_read i2c_master_recv failed\n");
		return -EIO;
	}

	DDBG("dlt002_i2c_read [E]\n");

	return 0;
}

/*
 *  Switches places of Most Significant Bit (MSB) and Least Significant  *
 *  Bit (LSB) before adding the data to the package buffer               *
*/
static void dlt002_add_bytes_in_switched_endian(uint8_t *
						p_package_buf,
						uint8_t pkg_position,
						uint8_t size_of_data, uint8_t *Data)
{
	int MSB_byte_number, byte_number;
	for (MSB_byte_number = size_of_data - 1, byte_number = 0;
	     MSB_byte_number >= 0; --MSB_byte_number, ++byte_number) {
		memcpy(p_package_buf + pkg_position + byte_number,
		       Data + MSB_byte_number, sizeof(uint8_t));
	}
}

static int32_t dlt002_send_reg_table(const struct reg_entry *table, uint32_t tables_to_send)
{
	int32_t ret = 0;
	uint32_t table_index = 0;
	uint32_t data_index = 0;
	uint32_t j;
	uint8_t data_buffer[DLT002_I2C_MAX_BYTES] = { 0 };
	uint32_t grouped_data = 0; /* The amount of continued data to group with one i2c write */
	uint8_t grouped_tables = 0;

	DDBG("dlt002_send_reg_table [tables: %d] [S]\n", tables_to_send);

	if (!table || tables_to_send == 0) {
		CDBG("dlt002_send_reg_table *table tables_to_send[%d] failed\n", tables_to_send);
		return -EFAULT;
	}
	/* Will loop thru entire table and send all data, if possible data will be grouped */
	while (table_index < tables_to_send) {
		grouped_data = 0;
		grouped_tables = 0;

		switch (table[table_index].reg_bits) {
		case REG_BITS_8:
			{
				/* Gather the amount of data we can send in one i2c stream, if next address is one incremented *
				 the i2c write will continue to write at next address automaticly when sending more data then  *
				 one register can hold. Only group 8 bits registers since we can't be sure that when writing   *
				 32 bits register that is is really 32 bits or 4 grouped 8 bits. */
				while (((table_index + grouped_tables + 1) < tables_to_send) &&	/* Only if there are more tables to be found */
				       (grouped_data < (DLT002_I2C_MAX_BYTES - 1)) &&	/* Make sure we only send MAX allowed bytes */
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
			       (grouped_data < (DLT002_I2C_MAX_BYTES - 2)) &&	/* Make sure we only send MAX allowed bytes */
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
			       (grouped_data < (DLT002_I2C_MAX_BYTES - 4)) &&	/* Make sure we only send MAX allowed bytes */
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
			CDBG("dlt002_send_reg_table wrong reg_bits\n");
			break;
		}

		ret = dlt002_i2c_write(table[table_index].address,
				       grouped_data + 1, &data_buffer[0]);
		if (ret) {
			CDBG("dlt002_send_reg_table i2c_write failed\n");
			break;
		}

		mdelay(4);
		table_index += grouped_tables + 1;
	}

	DDBG("dlt002_send_reg_table [E] ret[%d]\n", ret);

	return ret;
}

static int32_t dlt002_check_msts(uint8_t value, uint32_t timeout_msec)
{
	char data;
	int32_t ret = 0;
	unsigned long timeout_jiffies;

	timeout_jiffies = jiffies + msecs_to_jiffies(timeout_msec);

	while (time_before(jiffies, timeout_jiffies)) {
		ret = dlt002_i2c_read(DLT002_USERCTRL_0010, BYTE_1, &data);
		if (ret) {
			CDBG("dlt002_check_msts: i2c_read failed\n");
			return ret;
		}

		DDBG("dlt002_check_msts: %X\n", data);
		if (data == value) {
			DDBG("dlt002_check_msts match %X\n", value);
			return 0;
		}
		msleep(DLT002_POLLING_SLEEP_MSEC);
	}

	CDBG("dlt002_check_msts: timeout \n");
	return 1;
}

static int32_t dlt002_check_bsts(uint8_t value, uint32_t timeout_msec)
{
	char data;
	int32_t ret = 0;
	unsigned long timeout_jiffies;

	timeout_jiffies = jiffies + msecs_to_jiffies(timeout_msec);

	while (time_before(jiffies, timeout_jiffies)) {
		ret = dlt002_i2c_read(DLT002_USERCTRL_0004, BYTE_1, &data);
		if (ret) {
			CDBG("dlt002_check_bsts: i2c_read failed\n");
			return ret;
		}

		DDBG("dlt002_check_bsts: %X\n", data);
		if (data == value) {
			DDBG("dlt002_check_bsts match %X\n", value);
			return 0;
		}
		msleep(DLT002_POLLING_SLEEP_MSEC);
	}

	CDBG("dlt002_check_bsts: timeout \n");
	return 1;
}

static int __dlt002_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, dlt002_camera_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __dlt002_probe,
	.driver = {
		   .name = "msm_camera_dlt002",
		   .owner = THIS_MODULE,
		   },
};

static int __init dlt002_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(dlt002_init);

static int init_thread(void *data)
{
	int32_t ret = 0;

	DDBG("Camera init thread started\n");

	down(&dlt002_sem);

	if (dlt002_ctrl->vendor_id == VENDOR_ID_0) {
		ret = dlt002_set_regs(dlt002_vendor_0_period_2_ES1);
		DDBG("dlt002_send_reg_table dlt002_vendor_0_period_2_ES1 sent\n");
		if (ret) {
			CDBG("dlt002_send_reg_table dlt002_vendor_0_period_2_ES1 failed\n");
		}

		ret = dlt002_set_regs(dlt002_vendor_0_period_3_ES1);
		DDBG("dlt002_send_reg_table dlt002_vendor_0_period_3_ES1 sent\n");
		if (ret) {
			CDBG("dlt002_send_reg_table dlt002_vendor_0_period_3_ES1 failed\n");
		}
	}

	else if (dlt002_ctrl->vendor_id == VENDOR_ID_1) {
		ret = dlt002_set_regs(dlt002_vendor_1_period_2_ES1);
		DDBG("dlt002_send_reg_table dlt002_vendor_1_period_2_ES1 sent\n");
		if (ret) {
			CDBG("dlt002_send_reg_table dlt002_vendor_1_period_2_ES1 failed\n");
		}

		ret = dlt002_set_regs(dlt002_vendor_1_period_3_ES1);
		DDBG("dlt002_send_reg_table dlt002_vendor_1_period_3_ES1 sent\n");
		if (ret) {
			CDBG("dlt002_send_reg_table dlt002_vendor_1_period_3_ES1 failed\n");
		}
	}

	ret = dlt002_update_scene(dlt002_ctrl->scene);
	dlt002_ctrl->init_complete = 1;

	up(&dlt002_sem);

	DDBG("Camera init thread end\n");
	return ret;

}
