/* drivers/input/touchscreen/synaptics_i2c_rmi4.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Alexandar Rodzevski <alexandar.rodzevski@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/synaptics_i2c_rmi.h>

#include <asm/uaccess.h>

// dx : defines
#define DX_MODULE_VER			"v006"
#define DX_MODULE_NAME			"x8gesture"
#define MODE_PINCH_ZOOM			1
#define MODE_GAME			2
#define W_THRESHOLD			150
#define W_THRESHOLD_ZOOM_IN		350
#define W_THRESHOLD_LARGE_ZOOM_OUT	150
#define W_NO_ZOOM			20
#define D_STABLE_THRESHOLD		80000
#define D_TWO_FINGER_THRESHOLD		150000
#define D_THRESHOLD_ZOOM_IN		160*120*4
#define D_RECOVER_THRESHOLD		40000
#define NUM_EVENT_STABLE		20
#define AXIS_THRESHOLD			40

#define SYNA_DBG(x)			

#define synpatics_swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

/* Product firmware specific defines, hence the MACROs and register-load function */
#define F01_RMI_QUERY00_MANUFACT_ID(x)		(unsigned char)(x.f01_RMI.query + 0)
#define F01_RMI_QUERY01_PROD_PROPS(x) 		(unsigned char)(x.f01_RMI.query + 1)
#define F01_RMI_QUERY02_CUST_FAMILY(x) 		(unsigned char)(x.f01_RMI.query + 2)
#define F01_RMI_QUERY03_FW_REW(x) 		(unsigned char)(x.f01_RMI.query + 3)
#define F01_RMI_CMD00_DEVICE_COMMAND(x) 	(unsigned char)(x.f01_RMI.cmd + 0)
#define F01_RMI_CTRL00_DEVICE_CONTROL(x)	(unsigned char)(x.f01_RMI.ctrl + 0)
#define F01_RMI_CTRL01_00_INTERRUPT_ENABLE(x)	(unsigned char)(x.f01_RMI.ctrl + 1)
#define F01_RMI_DATA00_DEVICE_STATUS(x) 	(unsigned char)(x.f01_RMI.data + 0)
#define F01_RMI_DATA01_00_INTERRUPT_STATUS(x) 	(unsigned char)(x.f01_RMI.data + 1)

#define F11_2D_QUERY00_PER_DEVICE(x)		(unsigned char)(x.f11_2D.query + 0)
#define F11_2D_QUERY01_REPORTING_MODE(x)	(unsigned char)(x.f11_2D.query + 1)
#define F11_2D_CMD00_2D_COMMAND(x)		(unsigned char)(x.f11_2D.cmd + 0)
#define F11_2D_CTRL00_REPORT_MODE(x)		(unsigned char)(x.f11_2D.ctrl + 0)
#define F11_2D_CTRL02_DELTA_X_THRES(x)		(unsigned char)(x.f11_2D.ctrl + 2)
#define F11_2D_CTRL03_DELTA_Y_THRES(x)		(unsigned char)(x.f11_2D.ctrl + 3)
#define F11_2D_CTRL06_2D_MAX_X_7_0(x)		(unsigned char)(x.f11_2D.ctrl + 6)
#define F11_2D_CTRL07_2D_MAX_X_11_8(x)		(unsigned char)(x.f11_2D.ctrl + 7)
#define F11_2D_CTRL08_2D_MAX_Y_7_0(x)		(unsigned char)(x.f11_2D.ctrl + 8)
#define F11_2D_CTRL09_2D_MAX_Y_11_8(x)		(unsigned char)(x.f11_2D.ctrl + 9)
#define F11_2D_DATA00_2D_FINGER_STATE(x)	(unsigned char)(x.f11_2D.data + 0)

#define F08_BIST_CMD00_BIST_COMMAND(x) 		(unsigned char)(x.f08_BIST.cmd + 0)
#define F08_BIST_DATA00_TEST_NBR_CTRL(x)	(unsigned char)(x.f08_BIST.data + 0)
#define F08_BIST_DATA01_OVERALL_RES(x) 		(unsigned char)(x.f08_BIST.data + 1)
#define F08_BIST_DATA02_TEST_SPECIFIC_RES(x) 	(unsigned char)(x.f08_BIST.data + 2)

/* 2D data registers byte format: | X11:4 | Y11:4 | Y3:0 | X3:0 | WyWx | Z | */
#define NBR_OF_2D_DATA_REGS_PER_FINGER	5
/* Maximum two fingers supported by this device driver. */
#define MAX_NBR_OF_FINGERS_SUPPORTED	2

/* TODO: To be activated when needed two-finger detection */
/* #define TWO_FINGER_DETECTION */


/* The 2D data packet order is applicable for each finger detected. */
enum rmi4_2D_data_pkt_index {
	PKT_INDX_2D_X_POS_11__4,
	PKT_INDX_2D_Y_POS_11__4,
	PKT_INDX_2D_YX_POS_3__0,
	PKT_INDX_2D_WIDTH_XY,
	PKT_INDX_2D_Z
};

enum drv_state {
	DRV_STATE_POWEROFF,
	DRV_STATE_INACTIVE_APP,
	DRV_STATE_ACTIVE_BL,
	DRV_STATE_ACTIVE_APP,
	DRV_STATE_UNKNOWN
};

struct rmi4_panel_info {
	uint8_t nbr_of_sensor_panels;
	uint8_t nbr_of_fingers;
	uint8_t nbr_of_data_bytes;
	uint8_t manufact_id;
	uint8_t prod_properties;
	uint8_t cust_family;
	uint8_t firmware_rev;
	uint16_t max_x;
	uint16_t max_y;
};

struct rmi4_func_reg_types {
	unsigned char query;
	unsigned char cmd;
	unsigned char ctrl;
	unsigned char data;
	unsigned char nbr_of_ints;
	unsigned char func_exists;
};

struct rmi4_func_base_addr {
	struct rmi4_func_reg_types f11_2D;
	struct rmi4_func_reg_types f08_BIST;
	struct rmi4_func_reg_types f01_RMI;
};

struct rmi4_2D_packet {
	uint8_t pkt_size;
	uint8_t *pkt;
};

struct rmi4_2D_finger_data {
	uint16_t x;
	uint16_t y;
	uint8_t w;
	uint8_t z;
};

static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	enum drv_state state;
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	int snap_state[2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
	struct cdev device_cdev;
	int device_major;
	struct class *device_class;
	struct rmi4_func_base_addr rmi4_func;
	struct rmi4_panel_info info_2D;
	struct rmi4_2D_packet pkt_2D;
};


struct x8g_data_type {
	int prev_x;
	int prev_y;
	int finger;
	int center_x;
	int center_y;
	int fakex;
	int fakey;
	int first_x;
	int first_y;
	int send_event;
	int num_stable_event;
	int state;
	int mode;
};

static struct x8g_data_type x8gdata; 

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static int synaptics_load_rmi4_func_regs
	(struct i2c_client *client, struct rmi4_func_base_addr *rmi4_func)
{
	int ret;
	unsigned char pkt_size = sizeof(struct rmi4_func_reg_types);
	struct rmi4_func_reg_types *func;

	func = &rmi4_func->f11_2D;
	ret = i2c_smbus_read_i2c_block_data(client,
		SYNA_PDT_P00_F11_2D_QUERY_BASE, pkt_size,
		(unsigned char *)func);
	if (ret < 0)
		goto read_block_data_fail;
	SYNA_DBG(printk(KERN_INFO "%s: Func:f11_2D Query_Base:0x%02x Cmd_Base:0x%02x "
		"Ctrl_Base:0x%02x Data_Base:0x%02x\n", __FUNCTION__,
		func->query, func->cmd, func->ctrl, func->data);)

	func = &rmi4_func->f08_BIST;
	ret = i2c_smbus_read_i2c_block_data(client,
		SYNA_PDT_P00_F08_BIST_QUERY_BASE, pkt_size,
		(unsigned char *)func);
	if (ret < 0)
		goto read_block_data_fail;
	SYNA_DBG(printk(KERN_INFO "%s: Func:f08_BIST Query_Base:0x%02x Cmd_Base:0x%02x "
		"Ctrl_Base:0x%02x Data_Base:0x%02x\n", __FUNCTION__,
		func->query, func->cmd, func->ctrl, func->data);)

	func = &rmi4_func->f01_RMI;
	ret = i2c_smbus_read_i2c_block_data(client,
		SYNA_PDT_P00_F01_RMI_QUERY_BASE, pkt_size,
		(unsigned char *)func);
	if (ret < 0)
		goto read_block_data_fail;
	SYNA_DBG(printk(KERN_INFO "%s: Func:f01_RMI Query_Base:0x%02x Cmd_Base:0x%02x "
		"Ctrl_Base:0x%02x Data_Base:0x%02x\n", __FUNCTION__,
		func->query, func->cmd, func->ctrl, func->data);)

	return 0;

read_block_data_fail:
	printk(KERN_ERR "%s: Failed reading block data, err:%d\n",
		__FUNCTION__, ret);
	return ret;
}

static int synaptics_load_panel_info(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);
	struct rmi4_panel_info *info = &ts->info_2D;

	ret = i2c_smbus_read_byte_data(client,
		F11_2D_QUERY00_PER_DEVICE(ts->rmi4_func));
	if (ret < 0) {
		printk(KERN_ERR "%s: Error, "
		"failed reading nbr_of_sensor_panels\n", __FUNCTION__);
		return ret;
	}
	info->nbr_of_sensor_panels = ret + 1;

	ret = i2c_smbus_read_byte_data(client,
		F11_2D_QUERY01_REPORTING_MODE(ts->rmi4_func));
	if (ret < 0) {
		printk(KERN_ERR "%s: Error "
		"failed reading nbr_of_fingers\n", __FUNCTION__);
		return ret;
	} else {
		uint8_t buf[4];

		info->nbr_of_fingers = (ret & 0x07) + 1;
		if (info->nbr_of_fingers > MAX_NBR_OF_FINGERS_SUPPORTED) {
			printk(KERN_ERR "%s: Error, "
			"nbr_of_fingers:%d exceeds nbr supported by driver\n",
			__FUNCTION__, info->nbr_of_fingers);
			return -1;
		}
		info->nbr_of_data_bytes =
		(info->nbr_of_fingers * NBR_OF_2D_DATA_REGS_PER_FINGER);

		ret = i2c_smbus_read_i2c_block_data(ts->client,
			F01_RMI_QUERY00_MANUFACT_ID(ts->rmi4_func), 4, buf);
		if (ret < 0) {
			printk(KERN_ERR "%s: Error, "
			"failed reading revision data\n", __FUNCTION__);
			return ret;
		}
		info->manufact_id = buf[0];
		info->prod_properties = buf[1];
		info->cust_family = buf[2];
		info->firmware_rev = buf[3];

		ret = i2c_smbus_read_i2c_block_data(ts->client,
			F11_2D_CTRL06_2D_MAX_X_7_0(ts->rmi4_func), 4, buf);
		if (ret < 0) {
			printk(KERN_ERR "%s: Error "
			"Failed reading max_x/y\n", __FUNCTION__);
			return ret;
		}
		info->max_x = 0x00ff & ((uint16_t)buf[0]);
		info->max_x |= (uint16_t)(buf[1] << 8);
		info->max_y = 0x00ff & ((uint16_t)buf[2]);
		info->max_y |= (uint16_t)(buf[3] << 8);
		return 0;
	}
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;

	ret = i2c_smbus_write_byte_data(ts->client,
		F01_RMI_CMD00_DEVICE_COMMAND(ts->rmi4_func), 0x01);
	if (ret < 0) {
		printk(KERN_ERR "%s: Error, "
			"failed to Reset the firmware\n",
			__FUNCTION__);
		return ret;
	}
	msleep(100);

	ret = i2c_smbus_write_byte_data(ts->client,
		F01_RMI_CTRL00_DEVICE_CONTROL(ts->rmi4_func), 0x80);
	if (ret < 0){
		printk(KERN_ERR "%s: Error, "
			"failed to set the FW as Configured\n",
			__FUNCTION__);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(ts->client,
		F01_RMI_DATA00_DEVICE_STATUS(ts->rmi4_func));
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to read out the Device status\n",
			__FUNCTION__);
		return ret;
	}
	if ((ret & 0xce) != 0) {
		printk(KERN_ERR "%s: Invalid Device Status: 0x%x\n",
			__FUNCTION__, ret);
		return ret;
	}

	// report mode : most info
	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00_REPORT_MODE(ts->rmi4_func), 0x00);
	printk(KERN_ERR DX_MODULE_NAME ": switched to continuous reporting mode\n");

	return ret;
}

static int synaptics_snap_to_inactive_edge
	(struct synaptics_ts_data *ts, unsigned int coord, unsigned int i)
{
	if (ts->snap_state[i]) {
		if (coord <= ts->snap_down_off[i])
			coord = ts->snap_down[i];
		else if (coord >= ts->snap_up_off[i])
			coord = ts->snap_up[i];
		else
			ts->snap_state[i] = 0;
	} else {
		if (coord <= ts->snap_down_on[i]) {
			coord = ts->snap_down[i];
			ts->snap_state[i] = 1;
		} else if (coord >= ts->snap_up_on[i]) {
			coord = ts->snap_up[i];
			ts->snap_state[i] = 1;
		}
	}
	return coord;
}

static void synaptics_2D_data_handler(struct synaptics_ts_data *ts)
{
	int ret;
	int i;
	int w, wx, wy;
	int d, dx, dy;
	int d2, dx2, dy2;
	int d3, dx3, dy3;
	
	uint8_t finger_state;
	uint8_t finger_state_mask;
	uint8_t pkt_offset;
	uint8_t pkt_size = ts->pkt_2D.pkt_size;
	uint8_t *pkt = ts->pkt_2D.pkt;
	struct rmi4_2D_finger_data f_data[MAX_NBR_OF_FINGERS_SUPPORTED];

	ret = i2c_smbus_read_i2c_block_data(ts->client,
		F11_2D_DATA00_2D_FINGER_STATE(ts->rmi4_func), pkt_size, pkt);
	if (ret < 0) {
		printk(KERN_ERR "%s: Error, Failed reading packet\n",
			__FUNCTION__);
		return;
	}

	finger_state = pkt[0];
	finger_state_mask = 0x03;

	pkt_offset = 1;

	if ((finger_state & finger_state_mask) > 0x01) {
		printk(KERN_ERR "%s: Error, bad finger_state:0x%02X", __FUNCTION__, finger_state);
		return;
	}

	f_data[0].x = (pkt[pkt_offset + PKT_INDX_2D_YX_POS_3__0] & 0x000F);
	f_data[0].x |= ((pkt[pkt_offset + PKT_INDX_2D_X_POS_11__4] << 4) & 0x0FF0);
	f_data[0].y = ((pkt[pkt_offset + PKT_INDX_2D_YX_POS_3__0] >> 4) & 0x000F);
	f_data[0].y |= ((pkt[pkt_offset + PKT_INDX_2D_Y_POS_11__4] << 4) & 0x0FF0);

	if (ts->flags & SYNAPTICS_FLIP_X) f_data[0].x = (ts->info_2D.max_x - f_data[0].x);
	if (ts->flags & SYNAPTICS_FLIP_Y) f_data[0].y = (ts->info_2D.max_y - f_data[0].y);
	if (ts->flags & SYNAPTICS_SNAP_TO_INACTIVE_EDGE) {
		f_data[0].x = synaptics_snap_to_inactive_edge(ts, f_data[0].x, 0);
		f_data[0].y = synaptics_snap_to_inactive_edge(ts, f_data[0].y, 1);
	}
	if (ts->flags & SYNAPTICS_SWAP_XY) synpatics_swap(f_data[0].x, f_data[0].y);
	
	f_data[0].z = pkt[pkt_offset + PKT_INDX_2D_Z];
	/* Setting width to the highest axis-value */
	f_data[0].w =
		((pkt[pkt_offset + PKT_INDX_2D_WIDTH_XY] & 0x0F) >=
		(pkt[pkt_offset + PKT_INDX_2D_WIDTH_XY] & 0xF0) >> 4) ?
		(pkt[pkt_offset + PKT_INDX_2D_WIDTH_XY] & 0x0F) :
		(pkt[pkt_offset + PKT_INDX_2D_WIDTH_XY] & 0xF0) >> 4;



	if (f_data[0].z == 0 && f_data[0].w == 0) {		// untouched?
		SYNA_DBG(printk(KERN_INFO "%s: untouched\n", __FUNCTION__);)
		x8gdata.num_stable_event = 0;
		x8gdata.send_event = 1;
		x8gdata.finger = 0;			// we are waiting for first finger
		x8gdata.state = 0;
	}
	else {
		if (x8gdata.finger == 0) {
			SYNA_DBG(printk(KERN_INFO "%s: start touching\n", __FUNCTION__);)
			x8gdata.finger = 1;		// touched, waiting for finger 2

			// save position at first touch
			x8gdata.prev_x = f_data[0].x;
			x8gdata.prev_y = f_data[0].y;

			x8gdata.first_x = f_data[0].x;
			x8gdata.first_y = f_data[0].y;
		}
	}

	wx = (pkt[pkt_offset + PKT_INDX_2D_WIDTH_XY] & 0x0F);
	wy = (pkt[pkt_offset + PKT_INDX_2D_WIDTH_XY] & 0xF0) >> 4;
	dx = f_data[0].x - x8gdata.prev_x;
	if (dx < 0) dx = -dx;
	dy = f_data[0].y - x8gdata.prev_y;
	if (dy < 0) dy = -dy;
	d = dx*dx + dy*dy;
	w = wx*wx + wy*wy;
	d2 = dx2 = dy2 = 0;
	
	SYNA_DBG(printk(KERN_INFO "%s: x=%4d, y=%4d w=%3d d=%d\n", __FUNCTION__, f_data[0].x, f_data[0].y, w, d);)

	// check mode
	if (d < D_TWO_FINGER_THRESHOLD) {
		// first: down left, now: up right -> mode pinch
		if (x8gdata.first_x <= ts->info_2D.max_x / 8 && 7 * x8gdata.first_y >= ts->info_2D.max_y / 8 &&
			f_data[0].x >= 7 * ts->info_2D.max_x / 8 && f_data[0].y <= ts->info_2D.max_y / 8) {
			x8gdata.mode = MODE_PINCH_ZOOM;
			SYNA_DBG(printk(KERN_INFO "%s: switching to pinch zoom mode\n", __FUNCTION__);)
		}
		// first: down right, now: up left -> mode game
		if (x8gdata.first_x >= 7 * ts->info_2D.max_x / 8 && 7 * x8gdata.first_y >= ts->info_2D.max_y / 8 &&
			f_data[0].x <= ts->info_2D.max_x / 8 && f_data[0].y <= ts->info_2D.max_y / 8) {
			x8gdata.mode = MODE_GAME;
			SYNA_DBG(printk(KERN_INFO "%s: switching to game mode\n", __FUNCTION__);)
		}
	}
	
		
	
	// pinch zoom mode
	switch (x8gdata.mode) {
	case MODE_PINCH_ZOOM:
		if (x8gdata.finger <= 1) {
			// very very big touch?
			if (w > W_THRESHOLD_LARGE_ZOOM_OUT && 
			    (x8gdata.prev_x <= ts->info_2D.max_x / 5 || x8gdata.prev_x >= 4 * ts->info_2D.max_x / 5 ||	// are we around the the borders?
			     x8gdata.prev_y <= ts->info_2D.max_y / 5 || x8gdata.prev_y >= 4 * ts->info_2D.max_y / 5)) {
				x8gdata.num_stable_event++;
				if (x8gdata.num_stable_event >= 5) {
					// zoom out at the center
					x8gdata.center_x = ts->info_2D.max_x / 2;
					x8gdata.center_y = ts->info_2D.max_y / 2;
					SYNA_DBG(printk(KERN_INFO "%s: BIG zoom out detected w=%d wx=%d wy=%d c(%d:%d)\n", __FUNCTION__, w, wx, wy, x8gdata.center_x, x8gdata.center_y);)
					x8gdata.finger = 3;	// we have 2 fingers already
					x8gdata.send_event = 2;
					x8gdata.state = 3;	// BIG zoom out at the center
				}
				else {
					// disable sending data to wait for stable
					x8gdata.send_event = 0;
				}
			}
			else 	// no, at the center
				if (w > W_THRESHOLD) {
					x8gdata.num_stable_event++;
					if (x8gdata.num_stable_event > 5) {
						// big touch?
						if (w > W_THRESHOLD_ZOOM_IN) {				// too big touch, it's a zoom out!
							x8gdata.center_x = ts->info_2D.max_x / 2;	// zoom out at the center
							x8gdata.center_y = ts->info_2D.max_y / 2;
							SYNA_DBG(printk(KERN_INFO "%s: zoom out detected w=%d wx=%d wy=%d c(%d:%d)\n", __FUNCTION__, w, wx, wy, x8gdata.center_x, x8gdata.center_y);)
							x8gdata.state = 2;	// zoom out at the center
							x8gdata.finger = 3;	// we have 2 fingers already
							x8gdata.send_event = 2;	// allow sending data for 2 fingers
						}
						else {
							dx3 = f_data[0].x - x8gdata.first_x;
							dy3 = f_data[0].y - x8gdata.first_y;
							d3 = dx3 * dx3 + dy3 * dy3;
							if (d3 > D_THRESHOLD_ZOOM_IN) {		// we have enough distance from the first touch?
								x8gdata.center_x = x8gdata.first_x;
								x8gdata.center_y = x8gdata.first_y;
								SYNA_DBG(printk(KERN_INFO "%s: zoom in detected w=%d wx=%d wy=%d c(%d:%d)\n", __FUNCTION__, w, wx, wy, x8gdata.center_x, x8gdata.center_y);)
								x8gdata.state = 1;	// zoom in
								x8gdata.finger = 3;	// we have 2 fingers already
								x8gdata.send_event = 2;	// allow sending data for 2 fingers
							}
							else {
								// continue to wait
								x8gdata.send_event = 0;
							}
						}
					}
					else {
						// skip sending data to wait for stable
						x8gdata.send_event = 0;
					}
				}
		}

		if (x8gdata.finger >= 3) {
			// check for no more zoom
			if (w < W_NO_ZOOM) {
				// we have one finger only
				x8gdata.finger = 1;
				x8gdata.send_event = 1;
				x8gdata.state = 0;
			}
			else {
				dx2 = f_data[0].x - x8gdata.fakex;
				dy2 = f_data[0].y - x8gdata.fakey;
				d2 = dx2*dx2 + dy2*dy2;
			}
		}
	
		//SYNA_DBG(printk(KERN_INFO "%s: w=%3d, d=%8d, d2=%d, dx2=f=%d\n", __FUNCTION__, w, d, d2, x8gdata.finger);)


		// we are not expecting LARGE distances during zooming
		if (x8gdata.finger == 3 && d > D_STABLE_THRESHOLD && d2 > D_STABLE_THRESHOLD) {
			// try to recover from this...
			// look for what corner we WERE in
		
			if (x8gdata.prev_x <= x8gdata.center_x && x8gdata.prev_y <= x8gdata.center_y) i = 0;	// up left
			if (x8gdata.prev_x >= x8gdata.center_x && x8gdata.prev_y <= x8gdata.center_y) i = 1;	// up right
			if (x8gdata.prev_x >= x8gdata.center_x && x8gdata.prev_y >= x8gdata.center_y) i = 2;	// down right
			if (x8gdata.prev_x <= x8gdata.center_x && x8gdata.prev_y >= x8gdata.center_y) i = 3;	// down left
		
			// get current distance to the center
			dx = f_data[0].x - x8gdata.center_x;
			if (dx < 0) dx = -dx;
			dy = f_data[0].y - x8gdata.center_y;
			if (dy < 0) dy = -dy;

			switch (i) {
				case 0 :		// up left
					dx2 = x8gdata.center_x - dx;
					dy2 = x8gdata.center_y - dy;
					break;
				case 1 :		// up right
					dx2 = x8gdata.center_x + dx;
					dy2 = x8gdata.center_y - dy;
					break;
				case 2 :		// down right
					dx2 = x8gdata.center_x + dx;
					dy2 = x8gdata.center_y + dy;
					break;
				case 3 :		// down left
					dx2 = x8gdata.center_x - dx;
					dy2 = x8gdata.center_y + dy;
					break;
			}
		
			SYNA_DBG(printk(KERN_INFO "%s: recovering i=%d dx=%d dy=%d, new(%d:%d) old(%d:%d)\n", __FUNCTION__, i, dx, dy, dx2, dy2, x8gdata.prev_x, x8gdata.prev_y);)


			// recheck for this position (dx2:dy2)
			dx = dx2 - x8gdata.prev_x;
			dy = dy2 - x8gdata.prev_y;
			d = dx*dx + dy*dy;
			// and the fake position
			dx3 = dx2 - x8gdata.fakex;
			dy3 = dy2 - x8gdata.fakey;
			d3 = dx3*dx3 + dy3*dy3;
			SYNA_DBG(printk(KERN_INFO "%s: recovering d'=%d d2'=%d, thres=%d\n", __FUNCTION__, d, d3, D_RECOVER_THRESHOLD);)
		
			// acceptable?
			if (d < D_RECOVER_THRESHOLD || d2 < D_RECOVER_THRESHOLD) {
				// ok, consider this one
				f_data[0].x = dx2;
				f_data[0].y = dy2;
				SYNA_DBG(printk(KERN_INFO "%s: recovering accepted\n", __FUNCTION__);)
			}
			else {	// rejects
				SYNA_DBG(printk(KERN_INFO "%s: recovering rejected\n", __FUNCTION__);)
				x8gdata.send_event = 0;
			}
		}
		if (x8gdata.send_event != 0 || x8gdata.finger==1) {	// we are allowed to send events, save this place
			x8gdata.prev_x = f_data[0].x;
			x8gdata.prev_y = f_data[0].y;
		}
	
	
		if (x8gdata.state == 1) {
			// are we zooming in? decrease speed a little bit
			f_data[0].x = x8gdata.center_x - 3*(f_data[0].x-x8gdata.center_x)/4;
			f_data[0].y = x8gdata.center_y - 3*(f_data[0].y-x8gdata.center_y)/4;
		}
		
		if (x8gdata.send_event >= 2) {		// are we allowed to send some events for second finger?
			// calculate position for first saved finger
			x8gdata.fakex = x8gdata.center_x * 2 - f_data[0].x;
			x8gdata.fakey = x8gdata.center_y * 2 - f_data[0].y;
		
			// recheck fakex and fakey values
			if (x8gdata.fakex < 0) x8gdata.fakex = 0;
			if (x8gdata.fakey < 0) x8gdata.fakey = 0;
			if (x8gdata.fakex > ts->info_2D.max_x) x8gdata.fakex = ts->info_2D.max_x;
			if (x8gdata.fakey > ts->info_2D.max_y) x8gdata.fakey = ts->info_2D.max_y;
		}
		break;
	case MODE_GAME:
		if (x8gdata.finger <= 1) {
			if (w > W_THRESHOLD) {
				// big touch, wait for stabilization
				x8gdata.num_stable_event++;
				if (x8gdata.num_stable_event > 3) {
					// generate an event at the old position also
					x8gdata.fakex = x8gdata.prev_x;
					x8gdata.fakey = x8gdata.prev_y;
					x8gdata.finger = 3;	// we have 2 fingers already
					x8gdata.send_event = 2;
				}
				else {
					// be hold, wait for stablilization
					x8gdata.send_event = 0;
				}
			}
			else {
				x8gdata.num_stable_event = 0;
				if (x8gdata.send_event != 0 || x8gdata.finger==1) {
					// we are allowed to send events, save this place
					x8gdata.prev_x = f_data[0].x;
					x8gdata.prev_y = f_data[0].y;
				}
			}
		}
		
		if (x8gdata.finger >= 3) {
			// check for no more zoom
			if (w < W_NO_ZOOM) {
				// we have one finger only
				x8gdata.finger = 1;
				x8gdata.send_event = 1;
				x8gdata.state = 0;
			}
			else {
				dx2 = f_data[0].x - x8gdata.fakex;
				dy2 = f_data[0].y - x8gdata.fakey;
				d2 = dx2*dx2 + dy2*dy2;
				// mistaken (come back to the starting point)?
				if (d2 < D_STABLE_THRESHOLD) {
					// dont send these
					x8gdata.send_event = 0;
				}
			}
		}
		break;
	}
	

	if (x8gdata.send_event >= 1) {		// are we allowed to send some events for first finger?
		/* press- or release to be masked out on finger_0. */
		finger_state_mask = 0x01;
		if (finger_state & finger_state_mask) {
			input_report_abs(ts->input_dev, ABS_X, f_data[0].x);
			input_report_abs(ts->input_dev, ABS_Y, f_data[0].y);

			// dx : mt messages also
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, f_data[0].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, f_data[0].y);
		}
		input_report_abs(ts->input_dev, ABS_PRESSURE, f_data[0].z);
		input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, f_data[0].w);

		// dx : mt messages also
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, f_data[0].z);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, f_data[0].w);

		input_report_key(ts->input_dev, BTN_TOUCH, (finger_state & finger_state_mask));
		// dx : mt messages also
		input_mt_sync(ts->input_dev);
	}

	if (x8gdata.send_event >= 2) {		// are we allowed to send some events for second finger?
		// generate events for 2nd finger
		input_report_abs(ts->input_dev, ABS_HAT0X, x8gdata.fakex);
		input_report_abs(ts->input_dev, ABS_HAT0Y, x8gdata.fakey);

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x8gdata.fakex);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, x8gdata.fakey);

		input_report_abs(ts->input_dev, ABS_PRESSURE, f_data[0].z);
		input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, f_data[0].w);

		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, f_data[0].z);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, f_data[0].w);

		input_report_key(ts->input_dev, BTN_2, (finger_state & finger_state_mask));

		input_mt_sync(ts->input_dev);
	}

	if (x8gdata.send_event >= 1) {		// send sync message
		input_sync(ts->input_dev);
	}
	
	// re-enable send events in case we have just disabled it due to LARGE distances
	if (x8gdata.send_event == 0 && x8gdata.finger == 3) x8gdata.send_event = 2;

	// re-enable send events in case we have just disabled it due to not LARGE enough distance
	if (x8gdata.send_event == 0 && x8gdata.finger == 1) x8gdata.send_event = 1;
}

#ifdef __ETS_SUPPORT__
static int synaptics_ts_check_state
	(struct synaptics_ts_data *ts, int exp_state)
{
	int i;

	/* value 25 = 500(power on time)/20(mdelay) */
	for (i = 0; i < 25; i++) {
		if (ts->state >= exp_state) {
			return 0;
		} else {
			msleep(20);
		}
	}

	return -1;
}

static int synaptics_ts_do_calibration(struct i2c_client *client,
	struct synaptics_i2c_rmi_ioctl_clbr *data, int cmd)
{
	return 0;
}

static int synaptics_ts_do_bist
	(struct i2c_client *client, struct synaptics_i2c_rmi_ioctl_bist *bist)
{
	int ret = 0;
	int retry = 10;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);

	SNAPD(SNAP_LVL_INFO, SNAP_ETS, "SNAP BIST is running...\n");

	/* F08_BIST_Data0 = 0x0, do complete BIST */
	ret = i2c_smbus_write_byte_data(client,
		F08_BIST_DATA00_TEST_NBR_CTRL(ts->rmi4_func), 0x0);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		goto err_bist_failed;
	}

	/* RunBIST F08_BIST_Cmd0 (bit0) = 0x01 */
	ret = i2c_smbus_write_byte_data(client,
		F08_BIST_CMD00_BIST_COMMAND(ts->rmi4_func), 0x1);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		goto err_bist_failed;
	}

	while (retry-- > 0) {
		ret = i2c_smbus_read_byte_data(client,
		F08_BIST_CMD00_BIST_COMMAND(ts->rmi4_func));
		if ((ret >= 0) && ((ret & 0x1) == 0))
			break;
		msleep(100);
	}

	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte failed\n");
		goto err_bist_failed;
	}

	/* F08_BIST_Data1 0:Pass, 1:Fail */
	ret = i2c_smbus_read_byte_data(client,
		F08_BIST_DATA01_OVERALL_RES(ts->rmi4_func));
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte failed\n");
		goto err_bist_failed;
	}

	if (ret == 1) {	/* Will indicate the failing electrode number */
		/* F08_BIST_Data2 */
		ret = i2c_smbus_read_byte_data(client,
		F08_BIST_DATA02_TEST_SPECIFIC_RES(ts->rmi4_func));
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_read_byte failed\n");
			goto err_bist_failed;
		}

		bist->failure = bist->sensor_no =
			ret & 0x3f;	/* Failing Senor Number */
		bist->limit = ret >> 6;	/* Limit Failure Code */
	}
	SNAPD(SNAP_LVL_INFO, SNAP_ETS, "SNAP BIST Overall: %d.\n", ret);
	ret = 0;

err_bist_failed:
	return ret;
}

static int synaptics_ts_open(struct inode *inode, struct file *file)
{
	struct synaptics_ts_data *ts =
	container_of(inode->i_cdev, struct synaptics_ts_data, device_cdev);

	file->private_data = ts;
	return 0;
}

static int synaptics_ts_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int synaptics_ts_write(struct file *file, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t synaptics_ts_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client =
		((struct synaptics_ts_data *)file->private_data)->client;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);
	int err = 0;
	struct synaptics_i2c_rmi_ioctl_clbr data;
	struct synaptics_i2c_rmi_ioctl_bist bist;

	err = synaptics_ts_check_state(ts, DRV_STATE_ACTIVE_BL);
	if (err == -1) {
		dev_err(&ts->client->dev,
		"synaptics_ts_data, ioctl, invalid state\n");
		goto done;
	}

	switch (cmd) {
	case IOCTL_VALSET:
		if (!access_ok(VERIFY_READ, (void __user *) arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&ts->client->dev,
			"synaptics_ts_data, ioctl, invalid access\n");
			goto done;
		}

		if (copy_from_user(&data, (void __user *) arg, sizeof(data))) {
			err = -EFAULT;
			dev_err(&ts->client->dev,
			"synaptics_ts_data, ioctl, copy_from_user error\n");
			goto done;
		}
		/* IOCTL_VALSET */
		err = synaptics_ts_do_calibration(client, &data, 0x00);

		break;
	case IOCTL_VALGET:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&ts->client->dev,
			"synaptics_ts_data, ioctl, invalid access\n");
			goto done;
		}
		/* IOCTL_VALGET */
		err = synaptics_ts_do_calibration(client, &data, 0x01);

		if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
			err = -EFAULT;
			dev_err(&ts->client->dev,
			"synaptics_ts_data, ioctl, copy_to_user error\n");
			goto done;
		}
		break;
	case IOCTL_BIST:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(&ts->client->dev,
			"synaptics_ts_data, ioctl, invalid access\n");
			goto done;
		}

		memset(&bist, 0, sizeof(bist));
		/* IOCTL_BIST */
		err = synaptics_ts_do_bist(client, &bist);

		if (copy_to_user((void __user *) arg, &bist, sizeof(bist))) {
			err = -EFAULT;
			dev_err(&ts->client->dev,
			"synaptics_ts_data, ioctl, copy_to_user error\n");
			goto done;
		}
		break;
	default:
		err = -EINVAL;
		dev_err(&ts->client->dev,
		"synaptics_ts_data, ioctl, cmd error\n");
		break;
	}
done:
	return (err);
}

static struct file_operations synaptics_ts_fops = {
	.owner   = THIS_MODULE,
	.write   = synaptics_ts_write,
	.open	= synaptics_ts_open,
	.ioctl   = synaptics_ts_ioctl,
	.release = synaptics_ts_release,
};
#endif // #ifdef __ETS_SUPPORT__

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	uint8_t buf[2];
	uint8_t device_status;
	uint8_t interrupt_status;
	struct synaptics_ts_data *ts =
		container_of(work, struct synaptics_ts_data, work);

	ret = i2c_smbus_read_i2c_block_data(ts->client,
		F01_RMI_DATA00_DEVICE_STATUS(ts->rmi4_func), sizeof(buf), buf);
	if (ret < 0) {
		printk(KERN_ERR "%s: Error, Failed reading packet\n",
			__FUNCTION__);
		goto exit_work_function;
	}
	device_status = buf[0];
	interrupt_status = buf[1];
/*	SYNA_DBG(printk(KERN_INFO "%s: dev_status:0x%02X int_status:0x%02X\n",*/
/*			__FUNCTION__, device_status, interrupt_status);)*/

	if (ts->use_irq) {
		if (interrupt_status & SYNA_F01_RMI_INT_SOURCE_MASK_STATUS) {
			SYNA_DBG(printk(KERN_INFO "%s: STATUS IRQ, "
				"device status:0x%x\n",
				__FUNCTION__, device_status);)
			if ((device_status & 0x8f) != 0) {
				printk(KERN_ERR "%s: IRQ Error, "
					" bad device_status:0x%02X",
					__FUNCTION__,device_status);
				synaptics_init_panel(ts);
				goto exit_work_function;
			}
		}
		if (interrupt_status & SYNA_F11_2D_INT_SOURCE_MASK_ABS0) {
/*			SYNA_DBG(printk(KERN_INFO "%s: ABS0 IRQ\n",*/
/*				__FUNCTION__);)*/
			synaptics_2D_data_handler(ts);
		}
		if (interrupt_status & SYNA_F08_BIST_INT_SOURCE_MASK_BIST) {
			SYNA_DBG(printk(KERN_INFO "%s: BIST IRQ\n",
				__FUNCTION__);)
			/* TODO: implement BIST irq handling */
			goto exit_work_function;
		}
		if (interrupt_status & SYNA_F34_FLASH_INT_SOURCE_MASK_FLASH) {
			SYNA_DBG(printk(KERN_INFO "%s: Flash IRQ\n",
				__FUNCTION__);)
			/* TODO: implement Flash irq handling */
			goto exit_work_function;
		}
	} else {
		if ((device_status & 0x8f) != 0) {
			printk(KERN_ERR "%s: Poll Error, "
				"bad device_status:0x%02X",
				__FUNCTION__, device_status);
			synaptics_init_panel(ts);
			goto exit_work_function;
		}
		synaptics_2D_data_handler(ts);
	}

exit_work_function:
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts =
		container_of(timer, struct synaptics_ts_data, timer);

	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	disable_irq(ts->client->irq);
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct synaptics_i2c_rmi_platform_data *pdata;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int snap_left_on;
	int snap_left_off;
	int snap_right_on;
	int snap_right_off;
	int snap_top_on;
	int snap_top_off;
	int snap_bottom_on;
	int snap_bottom_off;
	uint32_t panel_version;
#ifdef __ETS_SUPPORT__
	dev_t device_t = MKDEV(0,0);
	struct device *class_dev_t = NULL;
#endif // #ifdef __ETS_SUPPORT__

	SNAPD(SNAP_LVL_INFO, SNAP_LOAD, "SNAP probe here\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata) {
		if (pdata->power) {
			ts->power = pdata->power;
			ret = ts->power(1);
			if (ret < 0) {
				printk(KERN_ERR "%s: Error, power on failed\n",
					__FUNCTION__);
				goto err_power_failed;
			}
		}
		ts->flags = pdata->flags;
	}

	ret = synaptics_load_rmi4_func_regs(ts->client, &ts->rmi4_func);
	if (ret < 0) {
		printk(KERN_ERR "%s: load rmi4_func base addresses failed\n",
			__FUNCTION__);
		goto err_panel_failure;
	}

	ret = synaptics_init_panel(ts);
	if (ret < 0) {
		printk(KERN_ERR "%s: synaptics_init_panel failed\n",
			__FUNCTION__);
		goto err_panel_failure;
	}

	ret = synaptics_load_panel_info(ts->client);
	if (ret < 0) {
		printk(KERN_ERR "%s: synaptics_load_panel_info failed\n",
			__FUNCTION__);
		goto err_panel_failure;
	}

	SYNA_DBG(printk(KERN_INFO "%s: nbr_panels:%d nbr_fingers:%d data_size:%d\n",
		__FUNCTION__, ts->info_2D.nbr_of_sensor_panels,
		ts->info_2D.nbr_of_fingers, ts->info_2D.nbr_of_data_bytes);
	printk(KERN_INFO "%s: fw_rev:%d max_x:%d max_y:%d\n",
		__FUNCTION__, ts->info_2D.firmware_rev,
		ts->info_2D.max_x, ts->info_2D.max_y);)

	ts->pkt_2D.pkt_size =
	((ts->info_2D.nbr_of_fingers * NBR_OF_2D_DATA_REGS_PER_FINGER) + 1);
	ts->pkt_2D.pkt = kzalloc(ts->pkt_2D.pkt_size, GFP_KERNEL);
	if (ts->pkt_2D.pkt == NULL) {
		printk(KERN_ERR "Failed allocating 2D pkt data\n");
		goto err_panel_failure;
	}

	max_x = ts->info_2D.max_x;
	max_y = ts->info_2D.max_y;
	if (ts->flags & SYNAPTICS_SWAP_XY)
		synpatics_swap(max_x, max_y);
	/* TODO: check how this is applicable to the RMI4.0 configuration */
	panel_version = ts->info_2D.manufact_id << 8;
	panel_version |= ts->info_2D.firmware_rev;

	if (pdata) {
		while (pdata->version > panel_version)
			pdata++;
		inactive_area_left = pdata->inactive_left;
		inactive_area_right = pdata->inactive_right;
		inactive_area_top = pdata->inactive_top;
		inactive_area_bottom = pdata->inactive_bottom;
		snap_left_on = pdata->snap_left_on;
		snap_left_off = pdata->snap_left_off;
		snap_right_on = pdata->snap_right_on;
		snap_right_off = pdata->snap_right_off;
		snap_top_on = pdata->snap_top_on;
		snap_top_off = pdata->snap_top_off;
		snap_bottom_on = pdata->snap_bottom_on;
		snap_bottom_off = pdata->snap_bottom_off;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		inactive_area_left = 0;
		inactive_area_right = 0;
		inactive_area_top = 0;
		inactive_area_bottom = 0;
		snap_left_on = 0;
		snap_left_off = 0;
		snap_right_on = 0;
		snap_right_off = 0;
		snap_top_on = 0;
		snap_top_off = 0;
		snap_bottom_on = 0;
		snap_bottom_off = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input device\n",
			__FUNCTION__);
		goto err_input_dev_alloc_failed;
	}
	/* TODO: Change to "synaptics-rmi4.0-touchscreen" */
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);	// dx : we have mt support
	set_bit(EV_ABS, ts->input_dev->evbit);

	inactive_area_left = inactive_area_left * max_x / 0x10000;
	inactive_area_right = inactive_area_right * max_x / 0x10000;
	inactive_area_top = inactive_area_top * max_y / 0x10000;
	inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;

	snap_left_on = snap_left_on * max_x / 0x10000;
	snap_left_off = snap_left_off * max_x / 0x10000;
	snap_right_on = snap_right_on * max_x / 0x10000;
	snap_right_off = snap_right_off * max_x / 0x10000;
	snap_top_on = snap_top_on * max_y / 0x10000;
	snap_top_off = snap_top_off * max_y / 0x10000;
	snap_bottom_on = snap_bottom_on * max_y / 0x10000;
	snap_bottom_off = snap_bottom_off * max_y / 0x10000;

	fuzz_x = fuzz_x * max_x / 0x10000;
	fuzz_y = fuzz_y * max_y / 0x10000;

	ts->snap_down[!!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_left;
	ts->snap_up[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x + inactive_area_right;
	ts->snap_down[!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_top;
	ts->snap_up[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y + inactive_area_bottom;
	ts->snap_down_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_on;
	ts->snap_down_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_off;
	ts->snap_up_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_on;
	ts->snap_up_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_off;
	ts->snap_down_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_on;
	ts->snap_down_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_off;
	ts->snap_up_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_on;
	ts->snap_up_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_off;
	SYNA_DBG(printk(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	printk(KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       inactive_area_left, inactive_area_right,
	       inactive_area_top, inactive_area_bottom);
	printk(KERN_INFO "synaptics_ts_probe: snap_x %d-%d %d-%d, snap_y %d-%d %d-%d\n",
	       snap_left_on, snap_left_off, snap_right_on, snap_right_off,
	       snap_top_on, snap_top_off, snap_bottom_on, snap_bottom_off);)

	input_set_abs_params(ts->input_dev, ABS_X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);

	// dx : we have mt support
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, fuzz_w, 0);
	// default pinch zoom mode
	x8gdata.mode = MODE_PINCH_ZOOM;
	
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#ifdef __ETS_SUPPORT__
 	ts->state = DRV_STATE_ACTIVE_APP;
	/* Character Device: set up */
	ret = alloc_chrdev_region(&device_t, 0, 1, "synaptics_ts");	//0:base num
	if (ret)
		goto err_cleanup_input;

	ts->device_major = MAJOR(device_t);	//deviceA_minor def_value

	cdev_init(&(ts->device_cdev), &synaptics_ts_fops);	//deviceA_fops:method func systemcall
	ts->device_cdev.owner = THIS_MODULE;
	ts->device_cdev.ops = &synaptics_ts_fops;
	ret = cdev_add(&(ts->device_cdev), MKDEV(ts->device_major,0), 1);
	if (ret)
		goto err_cleanup_chrdev;

	ts->device_class = class_create(THIS_MODULE, "synaptics_ts");
	if (IS_ERR(ts->device_class)) {
		ret = -1;
		goto err_cleanup_cdev;
	}

	class_dev_t = device_create(ts->device_class, NULL,
			MKDEV(ts->device_major,0), NULL, "synaptics_ts");
	if (IS_ERR(class_dev_t)) {
		ret = -1;
		goto err_cleanup_class;
	}
#endif // #ifdef __ETS_SUPPORT__

	if (client->irq) {
		ret = i2c_smbus_write_byte_data(ts->client,
			F01_RMI_CTRL01_00_INTERRUPT_ENABLE(ts->rmi4_func), 0);
		if (ret < 0)
			printk(KERN_ERR "%s: failed to send disable interrupts"
					".. continue anyway.\n", __FUNCTION__);
		ret = request_irq(client->irq, synaptics_ts_irq_handler,
				0, client->name, ts);
		if (ret < 0) {
			dev_err(&client->dev, "request_irq failed\n");
			goto err_irq_failure;
		}
		ret = i2c_smbus_write_byte_data(ts->client,
			F01_RMI_CTRL01_00_INTERRUPT_ENABLE(ts->rmi4_func),
				(SYNA_F01_RMI_INT_SOURCE_MASK_STATUS |
				SYNA_F11_2D_INT_SOURCE_MASK_ABS0));
		if (ret < 0) {
			free_irq(client->irq, ts);
			dev_err(&client->dev, "enable interrupts failed\n");
			goto err_irq_failure;
		}
		ts->use_irq = 1;
	} else {
		ts->use_irq = 0;
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	SYNA_DBG(printk(KERN_INFO "%s: Start touchscreen %s in %s mode\n", __FUNCTION__,
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");)
	printk(KERN_ERR DX_MODULE_NAME ": pinch zoom gesture added\n");

	return 0;

err_irq_failure:
#ifdef __ETS_SUPPORT__
err_cleanup_class:
	class_destroy(ts->device_class);
err_cleanup_cdev:
	cdev_del(&(ts->device_cdev));
err_cleanup_chrdev:
	unregister_chrdev_region(device_t, 1);
err_cleanup_input:
	input_unregister_device(ts->input_dev);
#endif // #ifdef __ETS_SUPPORT__

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_panel_failure:
err_power_failed:
	if (ts->power)
		ts->power(0);
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	if (ts->power)
		ts->power(0);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	/* disable interrupt */
	ret = i2c_smbus_write_byte_data(ts->client,
		F01_RMI_CTRL01_00_INTERRUPT_ENABLE(ts->rmi4_func), 0);
	if (ret < 0)
		printk(KERN_ERR "%s: i2c_smbus_write_byte_data failed\n",
			__FUNCTION__);
	/* deep sleep */
	ret = i2c_smbus_write_byte_data(client,
		F01_RMI_CTRL00_DEVICE_CONTROL(ts->rmi4_func), 0x81);
	if (ret < 0)
		printk(KERN_ERR "%s: i2c_smbus_write_byte_data failed\n",
			__FUNCTION__);
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "%s: power off failed\n",
				__FUNCTION__);
	}
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "%s: power on failed\n", __FUNCTION__);
	}

	synaptics_init_panel(ts);

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
		/* enable abs int */
		i2c_smbus_write_byte_data(ts->client,
		F01_RMI_CTRL01_00_INTERRUPT_ENABLE(ts->rmi4_func), 0x08);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_I2C_RMI_NAME,
	},
};

static int __devinit synaptics_ts_init(void)
{
	struct i2c_driver * otherDriver;
	struct device_driver * other;
	
	printk(KERN_INFO DX_MODULE_NAME ": module " DX_MODULE_VER " loaded\n");
	
	other = driver_find(SYNAPTICS_I2C_RMI_NAME, &i2c_bus_type);

	if (other)
	{
		put_driver(other);
		otherDriver = to_i2c_driver(other);
		printk(KERN_ERR DX_MODULE_NAME ": Stock driver found: %s, addr 0x%x, owner %x\n", other->name, (int)otherDriver, (int)other->owner);
		i2c_del_driver(otherDriver);
		printk(KERN_ERR DX_MODULE_NAME ": Stock driver removed\n");
	}

	printk(KERN_ERR DX_MODULE_NAME ": build configuration w=%d, ds=%d, dtf=%d, es=%d\n",
			W_THRESHOLD, D_STABLE_THRESHOLD, D_TWO_FINGER_THRESHOLD, NUM_EVENT_STABLE);

	synaptics_wq = create_singlethread_workqueue("synaptics_wq_mod");
	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver RMI 4.x");
MODULE_LICENSE("GPL");
