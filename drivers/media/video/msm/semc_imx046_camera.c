/*
 * semc_imx046_camera.c
 * Copyright (C) 2010 Sony Ericsson Mobile Communications Japan, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This file is derived from
 *      mt9t013.c
 *      Code Aurora Forum.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>

#define USE_RESOURCE_OF_POWER
#ifdef USE_RESOURCE_OF_POWER
#include <mach/vreg.h>
#endif /*USE_RESOURCE_OF_POWER*/

#include "semc_imx046_camera.h"

/*#define POWERON_NOT_USE_INTERRUPT*/
/*#define MONITOR_NOT_USE_INTERRUPT*/
/*#define SNAPSHOT_NOT_USE_INTERRUPT*/
/*#define PATH_CHECK*/
#define MOVE_MONITOR_MODE
#define LOG_OUTPUT_I2C_TRANSFER

#define TRUE 1
#define FALSE 0
#define boolean uint8_t

#define SEMC_IMX046_MSG_ERR(fmt, args...) printk(KERN_ERR \
		"SEMC_IMX046_ERR :" fmt, ##args)

/* #define _HIGH_ENA_ */
#ifdef _HIGH_ENA_
#define SEMC_IMX046_MSG_HIGH(fmt, args...) printk(KERN_DEBUG \
		"SEMC_IMX046_HIGH:" fmt, ##args)
#else
#define SEMC_IMX046_MSG_HIGH(fmt, args...)
#endif /*_HIGH_ENA_*/

/* #define _DBG_ENA_ */
#ifdef _DBG_ENA_
#define SEMC_IMX046_MSG_DBG(fmt, args...) printk(KERN_DEBUG \
		"SEMC_IMX046_DBG :" fmt, ##args)
#else
#define SEMC_IMX046_MSG_DBG(fmt, args...)
#endif /*_DBG_ENA_*/

struct semc_imx046_camera_work_t {
	struct work_struct work;
};

struct semc_imx046_camera_int_t {
	enum sensor_int_sync_type sync;
	int32_t count;
	wait_queue_head_t int_wait;
	spinlock_t isr_lock;
};

struct semc_imx046_camera_ctrl_t {
	int8_t opened;
	const struct msm_camera_sensor_info *sensordata;
	struct semc_imx046_camera_work_t *sensorw;
	struct i2c_client *client;
	struct msm_sensor_resp *resp;
	struct semc_imx046_camera_int_t vsync_int;
	struct semc_imx046_camera_int_t camera_int;
	struct msm_sync *syncdata;
};

#define I2C_PARAM_RW_BASIC_LENGTH_MAX	32

#define UINT8_2_UINT32(pdata) (((uint32_t)((uint8_t *)pdata)[0] << 24) | \
		 ((uint32_t)((uint8_t *)pdata)[1] << 16) | \
		 ((uint32_t)((uint8_t *)pdata)[2] << 8) | \
		 ((uint32_t)((uint8_t *)pdata)[3]))
#define UINT8_2_UINT16(pdata) (((uint16_t)((uint8_t *)pdata)[0] << 8) | \
		((uint16_t)((uint8_t *)pdata)[1]))
#define ARRAY_ELEMS(array_symbol) (sizeof(array_symbol) / \
		sizeof(array_symbol[0]))

static struct semc_imx046_camera_ctrl_t *semc_imx046_camera_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(semc_imx046_camera_wait_queue);
static DECLARE_MUTEX(semc_imx046_camera_sem);

static irqreturn_t camera_isr(int irq, void *data)
{
	enum sensor_int_sync_type sync_type;
	unsigned long flags;

	spin_lock_irqsave(&semc_imx046_camera_ctrl->camera_int.isr_lock, flags);

	sync_type = semc_imx046_camera_ctrl->camera_int.sync;

	if (SENSOR_INT_DISABLE != sync_type)
		disable_irq(semc_imx046_camera_ctrl->sensordata->sensor_int);

	semc_imx046_camera_ctrl->camera_int.sync = SENSOR_INT_DISABLE;

	SEMC_IMX046_MSG_HIGH("%s [S] sync[%d]\n", __func__, sync_type);

	switch (sync_type) {
	case SENSOR_INT_ENABLE_SYNC:
		wake_up(&semc_imx046_camera_ctrl->camera_int.int_wait);
		break;

	case SENSOR_INT_ENABLE_ASYNC:
		{
			struct msm_sensor_resp_t resp;
			struct msm_sensor_resp_int_t resp_int;

			resp_int.int_type = SENSOR_INT_TYPE_CAMERA;
			resp_int.ext_data.camera.dummy = 0x89ABCDEF;

			resp.type = SENSOR_RESP_MSG_INT_EVENT;
			resp.extdata = &resp_int;
			resp.extlen = sizeof(struct msm_sensor_resp_int_t);

			if (semc_imx046_camera_ctrl->resp)
				semc_imx046_camera_ctrl->resp->sensor_resp(
					&resp,
					MSM_CAM_Q_SENSOR_MSG,
					semc_imx046_camera_ctrl->syncdata);
		}
		break;

	case SENSOR_INT_DISABLE:
	case SENSOR_INT_ENABLE_NOT_USE:
	default:
		break;
	}

	spin_unlock_irqrestore(&semc_imx046_camera_ctrl->camera_int.isr_lock,
			 flags);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t vsync_isr(int irq, void *data)
{
	struct semc_imx046_camera_int_t vsync_int_params;
	unsigned long flags;

	spin_lock_irqsave(&semc_imx046_camera_ctrl->vsync_int.isr_lock, flags);

	vsync_int_params = semc_imx046_camera_ctrl->vsync_int;

	if (SENSOR_INT_DISABLE != vsync_int_params.sync)
		disable_irq(semc_imx046_camera_ctrl->sensordata->sensor_vsync);

	semc_imx046_camera_ctrl->vsync_int.sync = SENSOR_INT_DISABLE;

	SEMC_IMX046_MSG_HIGH("%s [S] vsync_int_async[%d] vsync_int_cnt[%d]\n", \
		__func__, vsync_int_params.sync, vsync_int_params.count);

	switch (vsync_int_params.sync) {
	case SENSOR_INT_ENABLE_SYNC:
		if (vsync_int_params.count) {
			semc_imx046_camera_ctrl->vsync_int.count--;
			semc_imx046_camera_ctrl->vsync_int.sync =
				SENSOR_INT_ENABLE_SYNC;
			enable_irq(semc_imx046_camera_ctrl->
				sensordata->sensor_vsync);
		} else {
			wake_up(&semc_imx046_camera_ctrl->vsync_int.int_wait);
		}
		break;
	case SENSOR_INT_ENABLE_ASYNC:
		{
			struct msm_sensor_resp_t resp;
			struct msm_sensor_resp_int_t resp_int;

			resp_int.int_type = SENSOR_INT_TYPE_VSYNC;
			resp_int.ext_data.vsync.dummy = 0x1234;

			resp.type = SENSOR_RESP_MSG_INT_EVENT;
			resp.extdata = &resp_int;
			resp.extlen = sizeof(struct msm_sensor_resp_int_t);

			if (semc_imx046_camera_ctrl->resp)
				semc_imx046_camera_ctrl->resp->sensor_resp(
					&resp,
					MSM_CAM_Q_SENSOR_MSG,
					semc_imx046_camera_ctrl->syncdata);
		}
		break;

	case SENSOR_INT_DISABLE:
	case SENSOR_INT_ENABLE_NOT_USE:
	default:
		break;
	}

	spin_unlock_irqrestore(&semc_imx046_camera_ctrl->vsync_int.isr_lock,
			flags);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return IRQ_HANDLED;
}

static int32_t semc_imx046_camera_i2c_param_read(
		struct sensor_param_io_t *io_data)
{
#define PARAM_READ_FOOTPRINT		1
	uint8_t sbuf[5];
	uint8_t rbuf[I2C_PARAM_RW_BASIC_LENGTH_MAX];
	int32_t i;
	uint8_t readlength;
#ifdef LOG_OUTPUT_I2C_TRANSFER
	uint8_t dbgbuf[65];
#endif /*LOG_OUTPUT_I2C_TRANSFER */
	int32_t ret;

	struct i2c_msg msgs[] = {
		{
		 .addr = semc_imx046_camera_ctrl->client->addr,
		 .flags = 0,
		 .len = sizeof(sbuf),
		 .buf = &sbuf[0],
		 },
		{
		 .addr = semc_imx046_camera_ctrl->client->addr,
		 .flags = I2C_M_RD,
		 .len = io_data->length + PARAM_READ_FOOTPRINT,
		 .buf = &rbuf[0],
		 },
	};

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (!io_data->data) {
		SEMC_IMX046_MSG_ERR("%s *data failed\n", __func__);
		return -EFAULT;
	}

	if (io_data->length >
		(I2C_PARAM_RW_BASIC_LENGTH_MAX - PARAM_READ_FOOTPRINT)) {
		SEMC_IMX046_MSG_ERR("%s length[%d] failed\n", \
			__func__, io_data->length);
		return -EFAULT;
	}

	memset(rbuf, 0, sizeof(rbuf));

	sbuf[0] = sizeof(sbuf);
	sbuf[1] = 0x01;
	sbuf[2] = (uint8_t) ((io_data->address & 0xFF00) >> 8);
	sbuf[3] = (uint8_t) ((io_data->address & 0x00FF));
	sbuf[4] = io_data->length;

#ifdef LOG_OUTPUT_I2C_TRANSFER
	SEMC_IMX046_MSG_DBG("%s	write(addr[0x%02X], \
		data[0x%02X%02X%02X%02X%02X])\n", __func__, \
		msgs[0].addr, msgs[0].buf[0], msgs[0].buf[1], msgs[0].buf[2], \
		msgs[0].buf[3], msgs[0].buf[4]);
#endif /*LOG_OUTPUT_I2C_TRANSFER */
	if (i2c_transfer(
		semc_imx046_camera_ctrl->client->adapter, msgs, 2) < 0) {
		SEMC_IMX046_MSG_ERR("%s i2c_transfer failed\n", __func__);
#ifndef PATH_CHECK
		return -EIO;
#endif /*PATH_CHECK */
	}
#ifdef LOG_OUTPUT_I2C_TRANSFER
	for (i = 0; i < msgs[1].len; i++)
		sprintf(&dbgbuf[i * 2], "%02X", msgs[1].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_IMX046_MSG_DBG("%s	read(readlen[%d], data[0x%s])\n", __func__, \
		msgs[1].len, &dbgbuf[0]);
#endif /*LOG_OUTPUT_I2C_TRANSFER */

	readlength = rbuf[0];

	if (readlength != io_data->length + 1) {
		SEMC_IMX046_MSG_ERR("%s length[%d] readlength[%d] failed\n", \
			__func__, io_data->length, readlength);
#ifndef PATH_CHECK
		return -EFAULT;
#endif /*PATH_CHECK */
	}

	ret = copy_to_user((void *) io_data->data, &rbuf[PARAM_READ_FOOTPRINT],
		 io_data->length);
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s copy_to_user failed\n", __func__);
#ifndef PATH_CHECK
		return -EFAULT;
#endif /*PATH_CHECK */
	}

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_i2c_param_write(
		struct sensor_param_io_t *io_data)
{
#define PARAM_WRITE_FOOTPRINT	 4
	uint8_t sbuf[I2C_PARAM_RW_BASIC_LENGTH_MAX];
	int32_t i;
#ifdef LOG_OUTPUT_I2C_TRANSFER
	uint8_t dbgbuf[65];
#endif /*LOG_OUTPUT_I2C_TRANSFER */
	int32_t ret;

	struct i2c_msg msg[] = {
		{
		 .addr = semc_imx046_camera_ctrl->client->addr,
		 .flags = 0,
		 .len = io_data->length + PARAM_WRITE_FOOTPRINT,
		 .buf = &sbuf[0],
		 },
	};

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (!io_data->data) {
		SEMC_IMX046_MSG_ERR("%s *data failed\n", __func__);
		return -EFAULT;
	}

	if (io_data->length >
		(I2C_PARAM_RW_BASIC_LENGTH_MAX - PARAM_WRITE_FOOTPRINT)) {
		SEMC_IMX046_MSG_ERR("%s length[%d] failed\n", \
			__func__, io_data->length);
		return -EFAULT;
	}

	sbuf[0] = io_data->length + PARAM_WRITE_FOOTPRINT;
	sbuf[1] = 0x02;
	sbuf[2] = (uint8_t) ((io_data->address & 0xFF00) >> 8);
	sbuf[3] = (uint8_t) ((io_data->address & 0x00FF));

	ret = copy_from_user((void *) &sbuf[PARAM_WRITE_FOOTPRINT],
		(void *) io_data->data, io_data->length);
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s copy_from_user failed\n", __func__);
		return -EFAULT;
	}
#ifdef LOG_OUTPUT_I2C_TRANSFER
	for (i = 0; i < msg[0].len; i++)
		sprintf(&dbgbuf[i * 2], "%02X", msg[0].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_IMX046_MSG_DBG("%s	write(addr[0x%2X], data[0x%s])\n", __func__, \
		 msg[0].addr, &dbgbuf[0]);
#endif /*LOG_OUTPUT_I2C_TRANSFER */

	if (i2c_transfer(
		semc_imx046_camera_ctrl->client->adapter, msg, 1) < 0) {
		SEMC_IMX046_MSG_ERR("%s i2c_transfer failed\n", __func__);
#ifndef PATH_CHECK
		return -EIO;
#endif /*PATH_CHECK */
	}

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_i2c_memory_read(
		struct sensor_memory_io_t *io_data)
{
#define MEM_READ_FOOTPRINT	3
	uint8_t sbuf[8];
	uint8_t *rbuf;
	int32_t i;
	uint32_t buflength;
	uint16_t readlength;
#ifdef LOG_OUTPUT_I2C_TRANSFER
	uint8_t dbgbuf[65];
#endif /*LOG_OUTPUT_I2C_TRANSFER */
	int32_t ret;

	struct i2c_msg msgs[] = {
		{
		 .addr = semc_imx046_camera_ctrl->client->addr,
		 .flags = 0,
		 .len = sizeof(sbuf),
		 .buf = &sbuf[0],
		 },
		{
		 .addr = semc_imx046_camera_ctrl->client->addr,
		 .flags = I2C_M_RD,
		 .len = 0,
		 .buf = NULL,
		 },
	};

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (!io_data->data) {
		SEMC_IMX046_MSG_ERR("%s *data failed\n", __func__);
		return -EFAULT;
	}

	if (io_data->length)
		buflength = io_data->length + MEM_READ_FOOTPRINT;
	else
		buflength = 0x10000 + MEM_READ_FOOTPRINT;

	rbuf = kmalloc(buflength, GFP_KERNEL);

	if (!rbuf) {
		SEMC_IMX046_MSG_ERR("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}

	msgs[1].len = buflength;
	msgs[1].buf = rbuf;

	memset(rbuf, 0, sizeof(rbuf));

	sbuf[0] = 0x00;
	sbuf[1] = 0x03;
	sbuf[2] = (uint8_t) ((io_data->address & 0xFF000000) >> 24);
	sbuf[3] = (uint8_t) ((io_data->address & 0x00FF0000) >> 16);
	sbuf[4] = (uint8_t) ((io_data->address & 0x0000FF00) >> 8);
	sbuf[5] = (uint8_t) ((io_data->address & 0x000000FF));
	sbuf[6] = (uint8_t) ((io_data->length & 0xFF00) >> 8);
	sbuf[7] = (uint8_t) ((io_data->length & 0x00FF));

#ifdef LOG_OUTPUT_I2C_TRANSFER
	for (i = 0; i < (32 < msgs[0].len ? 32 : msgs[0].len); i++)
		sprintf(&dbgbuf[i * 2], "%02X", msgs[0].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_IMX046_MSG_DBG(" \
		%s	write(length[0x%X], addr[0x%2X], data[0x%s])\n", \
		__func__, msgs[0].len, msgs[0].addr, &dbgbuf[0]);
#endif /*LOG_OUTPUT_I2C_TRANSFER */

	if (i2c_transfer(
		semc_imx046_camera_ctrl->client->adapter, msgs, 2) < 0) {
		SEMC_IMX046_MSG_ERR("%s i2c_transfer failed\n", __func__);
		kfree(rbuf);
		return -EIO;
	}

	readlength = ((uint16_t) (rbuf[1]) << 8) | ((uint16_t) rbuf[2]);

	if (readlength != io_data->length) {
		SEMC_IMX046_MSG_ERR("%s length[%d] readlength[%d] failed\n", \
			__func__, io_data->length, readlength);
		kfree(rbuf);
		return -EFAULT;
	}

	ret = copy_to_user((void *) io_data->data, &rbuf[MEM_READ_FOOTPRINT],
		io_data->length);
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s copy_to_user failed\n", __func__);
#ifndef PATH_CHECK
		return -EFAULT;
#endif /*PATH_CHECK */
	}

	kfree(rbuf);

#ifdef LOG_OUTPUT_I2C_TRANSFER
	for (i = 0; i < (32 < msgs[1].len ? 32 : msgs[1].len); i++)
		sprintf(&dbgbuf[i * 2], "%02X", msgs[1].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_IMX046_MSG_DBG(" \
		%s	read(length[0x%X], addr[0x%2X], data[0x%s])\n", \
		__func__, msgs[1].len, msgs[1].addr, &dbgbuf[0]);
#endif /*LOG_OUTPUT_I2C_TRANSFER */

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_i2c_memory_write(
		struct sensor_memory_io_t *io_data)
{
#define MEM_WRITE_FOOTPRINT 8
	uint8_t *sbuf;
	int32_t i;
	uint32_t buflength;
#ifdef LOG_OUTPUT_I2C_TRANSFER
	uint8_t dbgbuf[65];
#endif /*LOG_OUTPUT_I2C_TRANSFER */
	int32_t ret;

	struct i2c_msg msg[] = {
		{
		 .addr = semc_imx046_camera_ctrl->client->addr,
		 .flags = 0,
		 .len = 0,
		 .buf = NULL,
		 },
	};

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (!io_data->data) {
		SEMC_IMX046_MSG_ERR("%s *data failed\n", __func__);
		return -EFAULT;
	}

	if (io_data->length)
		buflength = io_data->length + MEM_WRITE_FOOTPRINT;
	else
		buflength = 0x10000 + MEM_WRITE_FOOTPRINT;

	sbuf = kmalloc(buflength, GFP_KERNEL);

	if (!sbuf) {
		SEMC_IMX046_MSG_ERR("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}

	msg[0].len = buflength;
	msg[0].buf = sbuf;

	memset(sbuf, 0, sizeof(sbuf));

	sbuf[0] = 0x00;
	sbuf[1] = 0x04;
	sbuf[2] = (uint8_t) ((io_data->address & 0xFF000000) >> 24);
	sbuf[3] = (uint8_t) ((io_data->address & 0x00FF0000) >> 16);
	sbuf[4] = (uint8_t) ((io_data->address & 0x0000FF00) >> 8);
	sbuf[5] = (uint8_t) ((io_data->address & 0x000000FF));
	sbuf[6] = (uint8_t) ((io_data->length & 0xFF00) >> 8);
	sbuf[7] = (uint8_t) ((io_data->length & 0x00FF));

	ret = copy_from_user((void *) &sbuf[MEM_WRITE_FOOTPRINT],
		(void *) io_data->data, io_data->length);
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s copy_from_user failed\n", __func__);
		return -EFAULT;
	}
#ifdef LOG_OUTPUT_I2C_TRANSFER
	for (i = 0; i < (32 < msg[0].len ? 32 : msg[0].len); i++)
		sprintf(&dbgbuf[i * 2], "%02X", msg[0].buf[i]);
	dbgbuf[i * 2] = '\0';
	SEMC_IMX046_MSG_DBG(" \
		%s	write(length[0x%X], addr[0x%2X], data[0x%s])\n", \
		__func__, msg[0].len, msg[0].addr, &dbgbuf[0]);
#endif /*LOG_OUTPUT_I2C_TRANSFER */

	if (i2c_transfer(
		semc_imx046_camera_ctrl->client->adapter, msg, 1) < 0) {
		SEMC_IMX046_MSG_ERR("%s i2c_transfer faild\n", __func__);
		kfree(sbuf);
		return -EIO;
	}

	kfree(sbuf);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_reg_int(
		struct semc_imx046_camera_int_t *int_data,
		int int_no, void *isr, int edge, char *name)
{
	int32_t ret;
	unsigned long flags;

	spin_lock_irqsave(&int_data->isr_lock, flags);
	int_data->sync = SENSOR_INT_ENABLE_NOT_USE;
	spin_unlock_irqrestore(&int_data->isr_lock, flags);

	ret = request_irq(int_no, isr, edge, name, 0);
	if (ret < 0) {
		SEMC_IMX046_MSG_ERR("%s request_irq failed\n", __func__);
		ret = -EFAULT;
	}

	spin_lock_irqsave(&int_data->isr_lock, flags);
	if (SENSOR_INT_DISABLE != int_data->sync)
		disable_irq(int_no);
	int_data->sync = SENSOR_INT_DISABLE;
	spin_unlock_irqrestore(&int_data->isr_lock, flags);

	return 0;
}

static int32_t semc_imx046_camera_reg_int_api(
		struct sensor_int_enable_t *int_enable)
{
	struct semc_imx046_camera_int_t *int_data;
	int32_t ret;
	int int_no;
	void *isr;
	int edge;
	char *name;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	switch (int_enable->type) {
	case SENSOR_INT_TYPE_CAMERA:
		int_data = &semc_imx046_camera_ctrl->camera_int;
		int_no = semc_imx046_camera_ctrl->sensordata->sensor_int;
		isr = camera_isr;
		edge = IRQF_TRIGGER_HIGH;
		name = "isp";
		break;
	case SENSOR_INT_TYPE_VSYNC:
		int_data = &semc_imx046_camera_ctrl->vsync_int;
		int_no = semc_imx046_camera_ctrl->sensordata->sensor_vsync;
		isr = vsync_isr;
		edge = IRQF_TRIGGER_RISING;
		name = "vsync";
		break;
	case SENSOR_INT_TYPE_INVALID:
	default:
		int_data = NULL;
		int_no = 0;
		isr = NULL;
		edge = 0;
		name = NULL;
		return -EFAULT;
	}

	ret = semc_imx046_camera_reg_int(int_data, int_no, isr, edge, name);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s semc_imx046_camera_reg_int failed\n", \
			__func__);
	return ret;
}

static int32_t semc_imx046_camera_wait_int(
		struct semc_imx046_camera_int_t *int_data,
		int timeout_ms)
{
	int32_t ret;

	SEMC_IMX046_MSG_HIGH("%s [S] sync[%d]\n", __func__, int_data->sync);

#if defined(POWERON_NOT_USE_INTERRUPT) || \
	defined(MONITOR_NOT_USE_INTERRUPT) || \
	defined(SNAPSHOT_NOT_USE_INTERRUPT)
	if (SENSOR_INT_ENABLE_NOT_USE >= int_data->sync) {
		mdelay(100);
		ret = timeout_ms - 100;
	} else
#endif /*!defined(POWERON_NOT_USE_INTERRUPT) &&
	!defined(MONITOR_NOT_USE_INTERRUPT)
	!defined(SNAPSHOT_NOT_USE_INTERRUPT) */
	{
		ret = wait_event_timeout(int_data->int_wait,
			(SENSOR_INT_ENABLE_NOT_USE >= int_data->sync),
			msecs_to_jiffies(timeout_ms));
	}

	SEMC_IMX046_MSG_HIGH("%s [E] time left[%d]\n", __func__, ret);

#ifdef PATH_CHECK
	if (!ret)
		ret = 1;
#endif /*PATH_CHECK */

	return ret;
}

static int32_t semc_imx046_camera_disable_int(
		struct semc_imx046_camera_int_t *int_data,
		int int_no)
{
	enum sensor_int_sync_type pre;
	unsigned long flags;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	spin_lock_irqsave(&int_data->isr_lock, flags);

	pre = int_data->sync;
	if (SENSOR_INT_DISABLE != pre)
		disable_irq(int_no);
	else
		SEMC_IMX046_MSG_DBG("%s already disabled, int_status[%d]\n", \
			__func__, pre);
	int_data->sync = SENSOR_INT_DISABLE;

	spin_unlock_irqrestore(&int_data->isr_lock, flags);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return (int32_t) pre;
}

static int32_t semc_imx046_camera_disable_int_api(
		struct sensor_int_enable_t *int_enable)
{
	struct semc_imx046_camera_int_t *int_data;
	int int_no;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	switch (int_enable->type) {
	case SENSOR_INT_TYPE_CAMERA:
		int_data = &semc_imx046_camera_ctrl->camera_int;
		int_no = semc_imx046_camera_ctrl->sensordata->sensor_int;
		break;
	case SENSOR_INT_TYPE_VSYNC:
		int_data = &semc_imx046_camera_ctrl->vsync_int;
		int_no = semc_imx046_camera_ctrl->sensordata->sensor_vsync;
		break;
	case SENSOR_INT_TYPE_INVALID:
	default:
		int_data = NULL;
		int_no = 0;
		return -EFAULT;
	}

	semc_imx046_camera_disable_int(int_data, int_no);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_enable_int(
		struct semc_imx046_camera_int_t *int_data,
		int int_no, enum sensor_int_sync_type sync,
		int32_t count)
{
	enum sensor_int_sync_type pre;
	unsigned long flags;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	spin_lock_irqsave(&int_data->isr_lock, flags);
	pre = int_data->sync;
	int_data->sync = sync;
	int_data->count = count;

	if (SENSOR_INT_DISABLE == pre)
		enable_irq(int_no);
	else
		SEMC_IMX046_MSG_ERR("%s int_status[%d]\n", __func__, pre);
	spin_unlock_irqrestore(&int_data->isr_lock, flags);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return (int32_t) pre;
}

static int32_t semc_imx046_camera_enable_int_api(
		struct sensor_int_enable_t *int_enable)
{
	struct semc_imx046_camera_int_t *int_data;
	int int_no;
	enum sensor_int_sync_type sync;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	switch (int_enable->type) {
	case SENSOR_INT_TYPE_CAMERA:
		int_data = &semc_imx046_camera_ctrl->camera_int;
		int_no = semc_imx046_camera_ctrl->sensordata->sensor_int;
		break;
	case SENSOR_INT_TYPE_VSYNC:
		int_data = &semc_imx046_camera_ctrl->vsync_int;
		int_no = semc_imx046_camera_ctrl->sensordata->sensor_vsync;
		break;
	case SENSOR_INT_TYPE_INVALID:
	default:
		int_data = NULL;
		int_no = 0;
		return -EFAULT;
	}

	sync = int_enable->sync;

	semc_imx046_camera_enable_int(int_data, int_no, int_enable->sync,
			int_enable->count);

	if (SENSOR_INT_ENABLE_SYNC == sync) {
		int32_t ret;
		int32_t timeout_ms;

		ret = copy_from_user((void *) &timeout_ms,
			 (void *) int_enable->timeout_ms, sizeof(int32_t));
		if (ret) {
			SEMC_IMX046_MSG_ERR( \
				"%s copy_from_user failed\n", __func__);
			return -EFAULT;
		}

		ret = semc_imx046_camera_wait_int(int_data, timeout_ms);
		if (!ret) {
			semc_imx046_camera_disable_int(int_data, int_no);
			SEMC_IMX046_MSG_ERR("%s wait_int timeout\n", __func__);
			return -EFAULT;
		}
		timeout_ms = ret;

		ret = copy_to_user((void *) int_enable->timeout_ms, &timeout_ms,
			sizeof(int32_t));
		if (ret) {
			semc_imx046_camera_disable_int(int_data, int_no);
			SEMC_IMX046_MSG_ERR( \
				"%s copy_to_user failed\n", __func__);
			return -EFAULT;
		}
	}

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_gpio_access(int gpio_pin, int dir)
{
	int rc = 0;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	rc = gpio_request(gpio_pin, "semc_imx046_camera");
	if (!rc)
		gpio_direction_output(gpio_pin, dir);
	gpio_free(gpio_pin);

	SEMC_IMX046_MSG_HIGH("%s [E] rc[%d]\n", __func__, rc);

	return rc;
}

static int32_t semc_imx046_camera_gpio_ctrl(
		struct sensor_gpio_ctrl_t *gpio_ctrl)
{
	int rc;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	rc = semc_imx046_camera_gpio_access(gpio_ctrl->gpio, gpio_ctrl->value);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return rc;
}

#ifdef USE_RESOURCE_OF_POWER
static int32_t semc_imx046_camera_resource_enable(
		const struct msm_camera_sensor_pwr *resource)
{
	int32_t ret;

	if (!resource) {
		SEMC_IMX046_MSG_ERR("%s argument is NULL.\n", __func__);
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		SEMC_IMX046_MSG_DBG("%s GPIO[%d]\n", __func__, \
			resource->resource.number);
		ret = semc_imx046_camera_gpio_access(resource->resource.number,
			TRUE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		SEMC_IMX046_MSG_DBG("%s VREG[%s]\n", __func__, \
			resource->resource.name);
		ret = vreg_enable(vreg_get(0, resource->resource.name));
		break;
	default:
		SEMC_IMX046_MSG_ERR("%s invalid resource type[%d]\n", \
			__func__, resource->type);
		ret = 1;
		break;
	}

	return ret;
}

static int32_t semc_imx046_camera_resource_disable(
		const struct msm_camera_sensor_pwr *resource)
{
	int32_t ret;

	if (!resource) {
		SEMC_IMX046_MSG_ERR("%s argument is NULL.\n", __func__);
		return 1;
	}

	switch (resource->type) {
	case MSM_CAMERA_SENSOR_PWR_GPIO:
		SEMC_IMX046_MSG_DBG("%s GPIO[%d]\n", __func__, \
			resource->resource.number);
		ret = semc_imx046_camera_gpio_access(resource->resource.number,
			FALSE);
		break;
	case MSM_CAMERA_SENSOR_PWR_VREG:
		SEMC_IMX046_MSG_DBG("%s VREG[%s]\n", __func__, \
			resource->resource.name);
		ret = vreg_disable(vreg_get(0, resource->resource.name));
		break;
	default:
		SEMC_IMX046_MSG_ERR("%s invalid resource type[%d]\n", \
			__func__, resource->type);
		ret = 1;
		break;
	}

	return ret;
}
#endif /*USE_RESOURCE_OF_POWER */

static int semc_imx046_camera_init_client(struct i2c_client *client)
{
	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (NULL == client) {
		SEMC_IMX046_MSG_ERR("%s *client failed\n", __func__);
		return -EFAULT;
	}

	init_waitqueue_head(&semc_imx046_camera_wait_queue);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static int32_t semc_imx046_camera_param_init(void)
{
	if (!semc_imx046_camera_ctrl)
		return 1;

	semc_imx046_camera_ctrl->camera_int.sync = SENSOR_INT_DISABLE;
	semc_imx046_camera_ctrl->camera_int.count = 0;
	semc_imx046_camera_ctrl->vsync_int.sync = SENSOR_INT_DISABLE;
	semc_imx046_camera_ctrl->vsync_int.count = 0;

	return 0;
}

static int32_t semc_imx046_camera_sensor_on(void)
{
	int32_t ret = 0;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	do {
#ifdef USE_RESOURCE_OF_POWER
#if defined(CONFIG_SONY_5MP_CAMERA_HEAD)
		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_l1);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_L1_L2 failed\n", \
				__func__);
			break;
		}

		mdelay(1);

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_af);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_AF failed\n", \
				__func__);
			break;
		}

		mdelay(1);

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_sd);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_SD failed\n", \
				__func__);
			break;
		}

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_sdap);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_SD_AP failed\n", \
				__func__);
			break;
		}

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_io);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_IO failed\n", \
				__func__);
			break;
		}

		mdelay(1);

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_sa);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_SA failed\n", \
				__func__);
			break;
		}

		mdelay(1);
#else
		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_l1);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_L1 failed\n", \
				__func__);
			break;
		}

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_l2);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_L2 failed\n", \
				__func__);
			break;
		}

		mdelay(1);

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_sd);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_SD failed\n", \
				__func__);
			break;
		}

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_io);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_IO failed\n", \
				__func__);
			break;
		}

		mdelay(1);

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_af);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_AF failed\n", \
				__func__);
			break;
		}

		ret = semc_imx046_camera_resource_enable(
			&semc_imx046_camera_ctrl->sensordata->vcam_sa);
		if (ret) {
			SEMC_IMX046_MSG_ERR("%s Power on VCAM_SA failed\n", \
				__func__);
			break;
		}

		mdelay(1);
#endif /*CONFIG_SONY_5MP_CAMERA_HEAD */
#endif /*USE_RESOURCE_OF_POWER */
	} while (0);

	SEMC_IMX046_MSG_HIGH("%s [E] ret[%d]\n", __func__, ret);

	return ret;
}

static void semc_imx046_camera_sensor_exit(void)
{
	int32_t ret = 0;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	semc_imx046_camera_disable_int(&semc_imx046_camera_ctrl->camera_int,
				 semc_imx046_camera_ctrl->
				 sensordata->sensor_int);
	semc_imx046_camera_disable_int(&semc_imx046_camera_ctrl->vsync_int,
				 semc_imx046_camera_ctrl->
				 sensordata->sensor_vsync);
	free_irq(semc_imx046_camera_ctrl->sensordata->sensor_int, 0);
	free_irq(semc_imx046_camera_ctrl->sensordata->sensor_vsync, 0);

	ret = semc_imx046_camera_gpio_access(semc_imx046_camera_ctrl->
		 sensordata->sensor_reset, FALSE);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s CAM_RESET_N release failed\n", \
			__func__);

	mdelay(1);

	ret = gpio_tlmm_config(GPIO_CFG
		(semc_imx046_camera_ctrl->sensordata->mclk, 0,
		GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Stop CAM_MCLK failed\n", __func__);

	msm_camio_clk_rate_set(0);

	mdelay(1);

#ifdef USE_RESOURCE_OF_POWER
#if defined(CONFIG_SONY_5MP_CAMERA_HEAD)
	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_sa);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_SA failed\n", __func__);

	mdelay(10);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_sd);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_SD failed\n", __func__);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_sdap);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_SD_AP failed\n", \
			__func__);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_io);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_IO failed\n", __func__);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_af);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_AF failed\n", __func__);

	mdelay(10);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_l1);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_L1_L2 failed\n",
			__func__);
#else
	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_sa);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_SA failed\n", __func__);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_af);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_AF failed\n", __func__);

	mdelay(10);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_sd);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_SD failed\n", __func__);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_io);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_IO failed\n", __func__);

	mdelay(10);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_l2);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_L2 failed\n", __func__);

	ret = semc_imx046_camera_resource_disable(
		&semc_imx046_camera_ctrl->sensordata->vcam_l1);
	if (ret)
		SEMC_IMX046_MSG_ERR("%s Power off VCAM_L1 failed\n", __func__);
#endif /*CONFIG_SONY_5MP_CAMERA_HEAD */
#endif /*USE_RESOURCE_OF_POWER */
	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);
}

static int semc_imx046_camera_sensor_check(void *sync, void *sensor_resp)
{
	int32_t ret = 0;

	down(&semc_imx046_camera_sem);

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (semc_imx046_camera_ctrl->opened) {
		SEMC_IMX046_MSG_ERR("semc_imx046_camera_open already opened\n");
		ret = -EBUSY;
		goto check_done;
	}

	semc_imx046_camera_ctrl->syncdata = (struct msm_sync *) sync;
	semc_imx046_camera_ctrl->resp = (struct msm_sensor_resp *) sensor_resp;

check_done:
	up(&semc_imx046_camera_sem);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return ret;
}

static int semc_imx046_camera_sensor_open_init(
		const struct msm_camera_sensor_info *data)
{
	int32_t ret = 0;

	down(&semc_imx046_camera_sem);

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (semc_imx046_camera_ctrl->opened) {
		SEMC_IMX046_MSG_ERR("semc_imx046_camera_open already opened\n");
		ret = 0;
		goto open_done;
	}

	ret = semc_imx046_camera_param_init();
	if (ret)
		SEMC_IMX046_MSG_ERR("%s param_init return failed\n", __func__);

	ret = semc_imx046_camera_reg_int(&semc_imx046_camera_ctrl->vsync_int,
			 semc_imx046_camera_ctrl->
			 sensordata->sensor_vsync, vsync_isr,
			 IRQF_TRIGGER_RISING, "vsync");
	if (ret)
		goto open_done;

	ret = semc_imx046_camera_sensor_on();
	if (ret < 0) {
		SEMC_IMX046_MSG_ERR("%s sensor_on failed\n", __func__);
		goto open_done;
	}

	ret = gpio_tlmm_config(GPIO_CFG
		(semc_imx046_camera_ctrl->sensordata->mclk, 1,
		GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s Output CAM_MCLK failed\n", __func__);
		goto open_done;
	}

	mdelay(1);

	ret = semc_imx046_camera_gpio_access(semc_imx046_camera_ctrl->
					 sensordata->sensor_reset, TRUE);
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s CAM_RESET_N release failed\n", \
			__func__);
		goto open_done;
	}

	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	mdelay(20);

open_done:
	if (ret) {
		semc_imx046_camera_sensor_exit();
		ret = -EFAULT;
	} else {
		semc_imx046_camera_ctrl->opened = 1;
	}

	up(&semc_imx046_camera_sem);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return ret;
}

int semc_imx046_camera_sensor_config(void __user *argp)
{
	int32_t ret;
	struct sensor_cfg_data cfg_data;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	ret = copy_from_user(&cfg_data,
		(void *) argp, sizeof(struct sensor_cfg_data));
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s copy_from_user failed\n", __func__);
		return -EFAULT;
	}

	down(&semc_imx046_camera_sem);

	SEMC_IMX046_MSG_HIGH("%s cfgtype = %d\n", __func__, cfg_data.cfgtype);
	switch (cfg_data.cfgtype) {
	case CFG_PARAM_WRITE:
		ret = semc_imx046_camera_i2c_param_write(
			&cfg_data.cfg.param_io);
		break;

	case CFG_PARAM_READ:
		ret = semc_imx046_camera_i2c_param_read(
			&cfg_data.cfg.param_io);
		break;

	case CFG_ENABLE_INT:
		ret = semc_imx046_camera_enable_int_api(
			&cfg_data.cfg.int_enable);
		break;

	case CFG_DISABLE_INT:
		ret = semc_imx046_camera_disable_int_api(
			&cfg_data.cfg.int_enable);
		break;

	case CFG_MEMORY_WRITE:
		ret = semc_imx046_camera_i2c_memory_write(
			&cfg_data.cfg.memory_io);
		break;

	case CFG_MEMORY_READ:
		ret = semc_imx046_camera_i2c_memory_read(
			&cfg_data.cfg.memory_io);
		break;

	case CFG_REGISTER_INT:
		ret = semc_imx046_camera_reg_int_api(
			&cfg_data.cfg.int_enable);
		break;

	case CFG_GPIO_CTRL:
		ret = semc_imx046_camera_gpio_ctrl(
			&cfg_data.cfg.gpio_ctrl);
		break;

	default:
		SEMC_IMX046_MSG_ERR("%s cfgtype failed\n", __func__);
		ret = -EFAULT;
		break;
	}

	up(&semc_imx046_camera_sem);

	if (ret) {
		SEMC_IMX046_MSG_ERR("%s failed ret = %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = copy_to_user((void *) argp, &cfg_data,
		sizeof(struct sensor_cfg_data));
	if (ret) {
		SEMC_IMX046_MSG_ERR("%s copy_to_user failed\n", __func__);
		return -EFAULT;
	}

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return ret;
}

int semc_imx046_camera_sensor_release(void)
{
	int ret = -EBADF;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	down(&semc_imx046_camera_sem);

	if (semc_imx046_camera_ctrl->opened) {
		semc_imx046_camera_sensor_exit();
		ret = semc_imx046_camera_ctrl->opened = 0;
	} else {
		ret = 0;
	}

	up(&semc_imx046_camera_sem);

	SEMC_IMX046_MSG_HIGH("%s [E] ret[%d]\n", __func__, ret);

	return ret;
}

static int semc_imx046_camera_probe(struct i2c_client *client,
		 const struct i2c_device_id *id)
{
	int ret = 0;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SEMC_IMX046_MSG_ERR("%s i2c_check_functionality failed\n", \
			__func__);
		kfree(semc_imx046_camera_ctrl->sensorw);
		semc_imx046_camera_ctrl->sensorw = NULL;

		return -ENOTSUPP;
	}

	semc_imx046_camera_ctrl->sensorw =
		kzalloc(sizeof(struct semc_imx046_camera_work_t), GFP_KERNEL);

	if (NULL == semc_imx046_camera_ctrl->sensorw) {
		SEMC_IMX046_MSG_ERR("%s sensorw failed\n", __func__);
		kfree(semc_imx046_camera_ctrl->sensorw);
		semc_imx046_camera_ctrl->sensorw = NULL;

		return -ENOMEM;
	}

	i2c_set_clientdata(client, semc_imx046_camera_ctrl->sensorw);

	semc_imx046_camera_init_client(client);
	semc_imx046_camera_ctrl->client = client;

	SEMC_IMX046_MSG_HIGH("%s [E] ret[%d]\n", __func__, ret);

	return ret;
}

static int __exit semc_imx046_camera_remove(struct i2c_client *client)
{
	struct semc_imx046_camera_work_t *sensorw = i2c_get_clientdata(client);

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	free_irq(client->irq, sensorw);
	semc_imx046_camera_ctrl->client = NULL;
	kfree(sensorw);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return 0;
}

static const struct i2c_device_id semc_imx046_camera_id[] = {
	{"semc_imx046_camera", 0},
	{}
};

static struct i2c_driver semc_imx046_camera_driver = {
	.id_table = semc_imx046_camera_id,
	.probe = semc_imx046_camera_probe,
	.remove = __exit_p(semc_imx046_camera_remove),
	.driver = {
			.name = "semc_imx046_camera",
	},
};

static int32_t semc_imx046_camera_init(
		const struct msm_camera_sensor_info *data)
{
	int32_t ret = 0;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	semc_imx046_camera_ctrl =
		kzalloc(sizeof(struct semc_imx046_camera_ctrl_t), GFP_KERNEL);
	if (NULL == semc_imx046_camera_ctrl) {
		SEMC_IMX046_MSG_ERR("%s *semc_imx046_camera_ctrl failed\n", \
			__func__);
		return -ENOMEM;
	}

	init_waitqueue_head(&semc_imx046_camera_ctrl->camera_int.int_wait);
	spin_lock_init(&semc_imx046_camera_ctrl->camera_int.isr_lock);

	init_waitqueue_head(&semc_imx046_camera_ctrl->vsync_int.int_wait);
	spin_lock_init(&semc_imx046_camera_ctrl->vsync_int.isr_lock);

	if (NULL != data) {
		semc_imx046_camera_ctrl->sensordata = data;
		ret = i2c_add_driver(&semc_imx046_camera_driver);
		if (IS_ERR_VALUE(ret))
			kfree(semc_imx046_camera_ctrl);
	}

	SEMC_IMX046_MSG_HIGH("%s [E] ret[%d]\n", __func__, ret);

	return ret;
}

void semc_imx046_camera_exit(void)
{
	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	i2c_del_driver(&semc_imx046_camera_driver);

	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);
}

static int32_t semc_imx046_camera_probe_init(
		const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;

	SEMC_IMX046_MSG_HIGH("%s [S]\n", __func__);

	rc = semc_imx046_camera_init(info);
	if (rc < 0) {
		SEMC_IMX046_MSG_ERR("%s driver initialization, failed\n", \
			__func__);
		goto probe_done;
	}

	s->s_init = semc_imx046_camera_sensor_open_init;
	s->s_release = semc_imx046_camera_sensor_release;
	s->s_config = semc_imx046_camera_sensor_config;
	s->s_check = semc_imx046_camera_sensor_check;

probe_done:
	SEMC_IMX046_MSG_HIGH("%s [E]\n", __func__);

	return rc;
}

static int __semc_imx046_camera_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, semc_imx046_camera_probe_init);
}

static struct platform_driver msm_camera_driver = {
	.probe = __semc_imx046_camera_probe,
	.driver = {
		.name = "msm_camera_semc_imx046_camera",
		.owner = THIS_MODULE,
	},
};

static int __init semc_imx046_camera_driver_init(void)
{
	semc_imx046_camera_ctrl = NULL;
	return platform_driver_register(&msm_camera_driver);
}

module_init(semc_imx046_camera_driver_init);
