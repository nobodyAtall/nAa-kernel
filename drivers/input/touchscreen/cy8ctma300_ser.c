/* drivers/input/touchscreen/cy8ctma300_ser.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *   based on Penmount (drivers/input/touchscreen/penmount.c)
 *
 * Author: Oskar Anderö <oskar.andero@sonyericsson.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/cy8ctma300_ser.h>

#define DRIVER_DESC "Cypress cy8ctma300 touch driver"

MODULE_AUTHOR("Oskar Andero <oskar.andero@sonyericsson.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* cy8 register index. */
#define CY8_REG_HST_MODE			0x00
#define CY8_REG_BL_MODE				0x00
/* cy8 register bitfields. */
#define REG_HST_MODE_BIT_SOFT_RESET		0x01
#define REG_HST_MODE_BIT_DEEP_SLEEP		0x02
#define REG_HST_MODE_BIT_LOW_POWER		0x04
#define REG_HST_MODE_BIT_NOT_USED		0x08
#define REG_HST_MODE_BIT_TEST_MODE0		0x10
#define REG_HST_MODE_BIT_TEST_MODE1		0x20
#define REG_HST_MODE_BIT_TEST_MODE2		0x40
#define REG_HST_MODE_BIT_DATA_READ_TOGGLE	0x80
/* cy8 register defines. */
#define REG_HST_MODE_OP_MASK_ALL		0xff
#define REG_HST_MODE_OP_NORMAL			0x00
#define REG_HST_MODE_OP_DEEP_SLEEP		0x02
#define REG_HST_MODE_OP_SYSTEM_INFO		0x10
#define REG_BL_MODE_OP_EXIT			0x20
#define REG_HST_MODE_OP_BOOTLOADER		0x20

#define CY8_MAX_DATA_LENGTH			0x80
#define CY8_NORMAL_MODE_DATA_LENGTH	0x0e
#define CY8_SYS_INFO_MODE_DATA_LENGTH	0x1f
#define CY8_BL_MODE_DATA_LENGTH		0x10
#define CY8_START_CONDITION_HI 		0xa5
#define CY8_START_CONDITION_LO 		0x5a
#define CY8_MAX_NBR_WAIT_FULL_PKT	       10
#define RPT_TIMEOUT msecs_to_jiffies(100)

#define DBG(x)

enum cy8_cmd {
	CMD_CY8_NO_CMD,
	CMD_CY8_EXIT_BL_MODE,
	CMD_CY8_GET_ANY_FULL_PKT,
	CMD_CY8_SET_SYS_INFO_MODE,
	CMD_CY8_SET_NORMAL_MODE,
	CMD_CY8_SET_SLEEP_MODE,
};

enum cy8_mode {
	CY8_MODE_UNDEF,
	CY8_MODE_BOOTLOADER,
	CY8_MODE_SYS_INFO,
	CY8_MODE_NORMAL,
	CY8_MODE_SLEEP,
};

struct cy8_sysinfo {
	unsigned int max_x;
	unsigned int max_y;
	unsigned int bl_vers;
	unsigned int cust_id;
	unsigned int proj_id;
	unsigned int fw_vers;
};

struct cy8_bl_map {
	u8 status;
	u8 error;
	u8 app_id_h;
	u8 app_id_l;
	u8 app_ver_h;
	u8 app_ver_l;
};

struct cy8_pkt {
	unsigned char reg;
	unsigned char chksum;
	unsigned char pkt_idx;
	unsigned char data_len;
	unsigned char data_idx;
	unsigned char data[CY8_MAX_DATA_LENGTH];
	unsigned char err;
};

struct cy8_sync_cmd {
	enum cy8_cmd cmd;
	struct completion cmplt;
};

struct cy8_ser {
	struct input_dev *dev;
	struct serio *serio;
	char phys[32];

	/*
	 * This should be removed as soon as the bug is corrected
	 * in firmware!
	 */
	unsigned int last_x;
	unsigned int last_y;

	enum cy8_mode		mode;
	struct cy8_sync_cmd	sync_cmd;
	struct cy8_sysinfo	sysinfo;
	struct mutex		mutex;
	struct cy8_pkt		rx_pkt;
	struct cy8_pkt		tx_pkt;
	struct early_suspend	early_suspend;
	unsigned fw_update:1;
};

static int (*cy8ctma300_ser_xres)(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cy8ctma300_early_suspend(struct early_suspend *h);
static void cy8ctma300_late_resume(struct early_suspend *h);
#endif

static int cy8ctma300_ser_xres_dummy(void)
{
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)
	return 0;
}

static int cy8ctma300_ser_checksum(const struct cy8_pkt const *pkt)
{
	int chksum, i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	for (chksum = i = 0; i < pkt->data_len; i++)
		chksum += pkt->data[i];
	chksum += pkt->data_len;
	chksum += pkt->reg;
	DBG(printk(KERN_INFO "%s: chksum:%d\n", __func__, chksum & 0xff);)
	return chksum & 0xff;
}

static void cy8ctma300_log_err_packet(const struct cy8_pkt const *pkt)
{
	int i;

	printk(KERN_ERR "%s: reg %d, chksum %d, data_len %d, data:[ ", __func__,
		   pkt->reg, pkt->chksum, pkt->data_len);
	for (i=0; (i < pkt->data_len) && (i < CY8_MAX_DATA_LENGTH); i++)
		printk("%d ", pkt->data[i]);
	printk("] err %d, data_idx %d\n", pkt->err, pkt->data_idx);
}

static int cy8ctma300_ser_send_echo(struct serio *serio)
{
	int err;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	if ((err = serio_write(serio, CY8_START_CONDITION_HI)))
		goto out;
	if ((err = serio_write(serio, CY8_START_CONDITION_LO)))
		goto out;
	if ((err = serio_write(serio, 0)))
		goto out;
	if ((err = serio_write(serio, 0)))
		goto out;
	if ((err = serio_write(serio, 0)))
		goto out;
out:	return err;
}

static int cy8ctma300_ser_send_packet
	(struct serio *serio, const struct cy8_pkt const *cy8_pkt)
{
	int err, i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	if ((err = serio_write(serio, CY8_START_CONDITION_HI)))
		goto out;
	if ((err = serio_write(serio, CY8_START_CONDITION_LO)))
		goto out;
	if ((err = serio_write(serio, cy8_pkt->chksum)))
		goto out;
	if ((err = serio_write(serio, cy8_pkt->data_len)))
		goto out;
	if ((err = serio_write(serio, cy8_pkt->reg)))
		goto out;

	DBG(printk(KERN_INFO "%s: packet %02x %02x %02x %02x  %02x", __func__,
		CY8_START_CONDITION_HI, CY8_START_CONDITION_LO,
		cy8_pkt->chksum, cy8_pkt->data_len, cy8_pkt->reg);)

	for (i = 0; i < cy8_pkt->data_len; i++) {
		if ((err = serio_write(serio, cy8_pkt->data[i])))
			goto out;
		DBG(printk(KERN_INFO "\t%02x",cy8_pkt->data[i]);)
	}
	DBG(printk(KERN_INFO "\n");)
out:	return err;
}

static int cy8ctma300_ser_wait_for_any_compl_packet(struct cy8_ser* cy8_ser)
{
	struct cy8_sync_cmd *sync = &cy8_ser->sync_cmd;
	struct serio *serio = cy8_ser->serio;
	int err = -1, i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	sync->cmd = CMD_CY8_GET_ANY_FULL_PKT;
	for (i = 0; i < CY8_MAX_NBR_WAIT_FULL_PKT; i++) {
		if(wait_for_completion_timeout(&sync->cmplt, RPT_TIMEOUT)) {
			err = 0;
			break;
		}
		if (i == (CY8_MAX_NBR_WAIT_FULL_PKT -1))
			break;
		DBG(printk(KERN_INFO "%s: Timed out, ECHO(%d)\n", __func__, i);)
		(void)cy8ctma300_ser_send_echo(serio);
	}
	sync->cmd = CMD_CY8_NO_CMD;
	if (err)
		printk(KERN_ERR "%s: Error, no compl. pkt occured\n", __func__);
	return err;
}

static int cy8ctma300_ser_exit_bootloader(struct cy8_ser *cy8_ser)
{
	struct cy8_sync_cmd *sync = &cy8_ser->sync_cmd;
	struct cy8_pkt *cy8_pkt = &cy8_ser->tx_pkt;
	struct serio *serio = cy8_ser->serio;
	int err = -1, i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	sync->cmd = CMD_CY8_EXIT_BL_MODE;
	cy8_pkt->reg = CY8_REG_HST_MODE;
	cy8_pkt->data_len = 1;
	cy8_pkt->data[0] = REG_BL_MODE_OP_EXIT;
	cy8_pkt->chksum = cy8ctma300_ser_checksum(cy8_pkt);
	for (i = 0; i < CY8_MAX_NBR_WAIT_FULL_PKT; i++) {
		if (i % 2) /* send echo on every 2:nd attempt*/
			(void)cy8ctma300_ser_send_echo(serio);
		else
			(void)cy8ctma300_ser_send_packet(serio, cy8_pkt);
		if(wait_for_completion_timeout(&sync->cmplt, RPT_TIMEOUT)) {
			cy8_ser->mode = CY8_MODE_UNDEF;
			err = 0;
			break;
		}
		DBG(printk(KERN_INFO "%s: Timed out, try:%d\n", __func__, i);)
	}
	sync->cmd = CMD_CY8_NO_CMD;
	if (err)
		printk(KERN_ERR "%s: Unable to exit BL mode\n", __func__);
	return err;
}

static int cy8ctma300_ser_set_sysinfo_mode(struct cy8_ser *cy8_ser)
{
	struct cy8_sync_cmd *sync = &cy8_ser->sync_cmd;
	struct cy8_pkt *cy8_pkt = &cy8_ser->tx_pkt;
	struct serio *serio = cy8_ser->serio;
	int err = -1, i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	sync->cmd = CMD_CY8_SET_SYS_INFO_MODE;
	cy8_pkt->reg = CY8_REG_HST_MODE;
	cy8_pkt->data_len = 1;
	cy8_pkt->data[0] = REG_HST_MODE_OP_SYSTEM_INFO;
	cy8_pkt->chksum = cy8ctma300_ser_checksum(cy8_pkt);
	for (i = 0; i < CY8_MAX_NBR_WAIT_FULL_PKT; i++) {
		if (i % 2) /* send echo on every 2:nd attempt*/
			(void)cy8ctma300_ser_send_echo(serio);
		else
			(void)cy8ctma300_ser_send_packet(serio, cy8_pkt);
		if(wait_for_completion_timeout(&sync->cmplt, RPT_TIMEOUT)) {
			cy8_ser->mode = CY8_MODE_SYS_INFO;
			err = 0;
			break;
		}
		DBG(printk(KERN_INFO "%s: Timed out, try:%d\n", __func__, i);)
	}
	sync->cmd = CMD_CY8_NO_CMD;
	if (err)
		printk(KERN_ERR "%s: Unable to set Sys Info mode\n", __func__);
	return err;
}

static int cy8ctma300_ser_set_normal_mode(struct cy8_ser *cy8_ser)
{
	struct cy8_sync_cmd *sync = &cy8_ser->sync_cmd;
	struct cy8_pkt *cy8_pkt = &cy8_ser->tx_pkt;
	struct serio *serio = cy8_ser->serio;
	int err = -1, i;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	sync->cmd = CMD_CY8_SET_NORMAL_MODE;
	cy8_pkt->reg = CY8_REG_HST_MODE;
	cy8_pkt->data_len = 1;
	cy8_pkt->data[0] = REG_HST_MODE_OP_NORMAL;
	cy8_pkt->chksum = cy8ctma300_ser_checksum(cy8_pkt);
	for (i = 0; i < CY8_MAX_NBR_WAIT_FULL_PKT; i++) {
		if (i % 2) /* send echo on every 2:nd attempt*/
			(void)cy8ctma300_ser_send_echo(serio);
		else
			(void)cy8ctma300_ser_send_packet(serio, cy8_pkt);
		if(wait_for_completion_timeout(&sync->cmplt, RPT_TIMEOUT)) {
			cy8_ser->mode = CY8_MODE_NORMAL;
			err = 0;
			break;
		}
		DBG(printk(KERN_INFO "%s: Timed out, try:%d\n", __func__, i);)
	}
	sync->cmd = CMD_CY8_NO_CMD;
	if (err)
		printk(KERN_ERR "%s: Unable to set Normal mode\n", __func__);
	return err;
}

static int cy8ctma300_ser_set_sleep_mode(struct cy8_ser *cy8_ser)
{
	struct cy8_sync_cmd *sync = &cy8_ser->sync_cmd;
	struct cy8_pkt *cy8_pkt = &cy8_ser->tx_pkt;
	struct serio *serio = cy8_ser->serio;
	int err = 0;
	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	sync->cmd = CMD_CY8_SET_SLEEP_MODE;
	cy8_pkt->reg = CY8_REG_HST_MODE;
	cy8_pkt->data_len = 1;
	cy8_pkt->data[0] = REG_HST_MODE_OP_DEEP_SLEEP;
	cy8_pkt->chksum = cy8ctma300_ser_checksum(cy8_pkt);
	(void)cy8ctma300_ser_send_packet(serio, cy8_pkt);
	if (!wait_for_completion_timeout(&sync->cmplt, RPT_TIMEOUT)) {
		printk(KERN_ERR "%s: Timed out\n", __func__);
		cy8ctma300_log_err_packet(cy8_pkt);
		err = -1;
	}
	cy8_ser->mode = CY8_MODE_SLEEP;
	sync->cmd = CMD_CY8_NO_CMD;
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cy8ctma300_early_suspend(struct early_suspend *h)
{
	struct cy8_ser *cy8_ser =
		container_of(h, struct cy8_ser, early_suspend);
	struct mutex *mutex = &cy8_ser->mutex;
	DBG(printk(KERN_INFO "%s \n", __func__));
	mutex_lock(mutex);
	if (!cy8_ser->fw_update)
		cy8ctma300_ser_set_sleep_mode(cy8_ser);
	mutex_unlock(mutex);
}

static void cy8ctma300_late_resume(struct early_suspend *h)
{
	struct cy8_ser *cy8_ser =
		container_of(h, struct cy8_ser, early_suspend);
	struct mutex *mutex = &cy8_ser->mutex;
	DBG(printk(KERN_INFO "%s \n", __func__));
	mutex_lock(mutex);
	if (!cy8_ser->fw_update) {
		/* Send an echo just for wake-up */
		(void)cy8ctma300_ser_send_echo(cy8_ser->serio);
		(void)cy8ctma300_ser_set_normal_mode(cy8_ser);
	}
	mutex_unlock(mutex);
}
#endif

static irqreturn_t cy8ctma300_ser_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct cy8_ser *cy8_ser = serio_get_drvdata(serio);
	struct input_dev *dev = cy8_ser->dev;
	struct cy8_sync_cmd *sync = &cy8_ser->sync_cmd;
	struct cy8_pkt *cy8_pkt = &cy8_ser->rx_pkt;

	DBG(printk(KERN_INFO"%s: enter\n", __func__));

	if (cy8_pkt->err && (data != CY8_START_CONDITION_HI)) {
		DBG(printk(KERN_INFO"%s: dropping packet data %x\n",
			__func__, data));
		goto exit;
	}

	switch (cy8_pkt->pkt_idx) {
	case 0:
		DBG(printk(KERN_INFO"%s: start condition high = %.2x\n",
			__func__, data));
		if (data != CY8_START_CONDITION_HI) {
			goto pkt_err;
		} else {
			cy8_pkt->err = 0;
			goto pkt_ok;
		}
	case 1:
		DBG(printk(KERN_INFO"%s: start condition low = %.2x\n",
			__func__, data));
		if (data != CY8_START_CONDITION_LO)
			goto pkt_err;
		else
			goto pkt_ok;
	case 2:
		DBG(printk(KERN_INFO"%s: checksum = %x\n", __func__, data));
		cy8_pkt->chksum = data;
		goto pkt_ok;
	case 3:
		DBG(printk(KERN_INFO"%s: data length = %u\n", __func__, data));
		cy8_pkt->data_len = data;
		goto pkt_ok;
	case 4:
		DBG(printk(KERN_INFO"%s: index = %u\n", __func__, data));
		cy8_pkt->reg = data;
		goto pkt_ok;
	default:
		DBG(printk(KERN_INFO"%s: data[%u] = 0x%x\n",
			__func__, cy8_pkt->data_idx, data));
		cy8_pkt->data[cy8_pkt->data_idx++] = data;

	}

	if (cy8_pkt->data_idx == cy8_pkt->data_len) {
		cy8_pkt->pkt_idx = 0;
		cy8_pkt->data_idx = 0;
		DBG(printk(KERN_INFO"%s: received a full packet!\n", __func__));
		if (cy8ctma300_ser_checksum(cy8_pkt) != cy8_pkt->chksum) {
			printk(KERN_ERR "%s: checksum error!\n", __func__);
			goto exit;
		} else if (sync->cmd == CMD_CY8_GET_ANY_FULL_PKT) {
			DBG(printk(KERN_INFO"%s: ANY_FULL_PKT/EXIT_BL_MODE\n",
				 __func__);)
			goto sync_cmplt;
		}

		switch (cy8_pkt->data[0] & REG_HST_MODE_OP_MASK_ALL) {
		case REG_HST_MODE_OP_SYSTEM_INFO:
		{
			if (cy8_pkt->data_len !=
				CY8_SYS_INFO_MODE_DATA_LENGTH) {
				DBG(printk(KERN_ERR "%s: err sys_info len:%d\n",
					 __func__, cy8_pkt->data_len);)
				goto pkt_err;
			}
			cy8_ser->sysinfo.max_x =
				(cy8_pkt->data[7] << 8) |
				cy8_pkt->data[8];
			cy8_ser->sysinfo.max_y =
				(cy8_pkt->data[9] << 8) |
				cy8_pkt->data[10];
			cy8_ser->sysinfo.bl_vers =
				(cy8_pkt->data[11] << 8) |
				cy8_pkt->data[12];
			cy8_ser->sysinfo.cust_id =
				cy8_pkt->data[15];
			cy8_ser->sysinfo.proj_id =
				cy8_pkt->data[16];
			cy8_ser->sysinfo.fw_vers =
				(cy8_pkt->data[17] << 8) |
				cy8_pkt->data[18];
			DBG(printk(KERN_INFO "%s:max_x=%d max_y=%d bl_vers=0x%x"
			" cust_id=0x%x proj_id=0x%x fw_vers=0x%x\n", __func__,
			cy8_ser->sysinfo.max_x, cy8_ser->sysinfo.max_y,
			cy8_ser->sysinfo.bl_vers, cy8_ser->sysinfo.cust_id,
			cy8_ser->sysinfo.proj_id, cy8_ser->sysinfo.fw_vers);)

			if (sync->cmd == CMD_CY8_SET_SYS_INFO_MODE) {
				DBG(printk(KERN_INFO"%s: SET_SYS_INFO_MODE\n",
					__func__);)
				goto sync_cmplt;
			}
			goto exit;
		}

		case REG_HST_MODE_OP_DEEP_SLEEP:
			if (sync->cmd == CMD_CY8_SET_SLEEP_MODE) {
				DBG(printk(KERN_INFO "%s: SET_SLEEP_MODE\n",
					__func__);)
				goto sync_cmplt;
			}
			goto exit;

		case REG_HST_MODE_OP_NORMAL:
		{
			int x = (cy8_pkt->data[3] << 8) | cy8_pkt->data[4];
			int y = (cy8_pkt->data[5] << 8) | cy8_pkt->data[6];
			int pressure = cy8_pkt->data[7];
			int finger = cy8_pkt->data[2];

			if (cy8_pkt->data_len != CY8_NORMAL_MODE_DATA_LENGTH) {
				DBG(printk(KERN_ERR "%s: err normal len:%d\n",
					 __func__, cy8_pkt->data_len));
				goto pkt_err;
			}
			if (sync->cmd == CMD_CY8_SET_NORMAL_MODE ||
					sync->cmd == CMD_CY8_EXIT_BL_MODE) {
			/*
			* As per definition the fw application always starts
			* in Normal Mode, hence exit bl mode sync here
			*/
				DBG(printk(KERN_INFO"%s: SET_NORMAL_MODE\n",
					__func__);)
				goto sync_cmplt;
			}
			/*
			 * This should be removed as soon as the bug is corrected
			 * in firmware!
			 */
			if (finger) {
				cy8_ser->last_x = x;
				cy8_ser->last_y = y;
			} else {
				x = cy8_ser->last_x;
				y = cy8_ser->last_y;
			}

			DBG(printk(KERN_INFO"x = %u [0x%x]\n", x, x));
			DBG(printk(KERN_INFO"y = %u [0x%x]\n", y, y));
			DBG(printk(KERN_INFO"pressure = %u\n", pressure));
			DBG(printk(KERN_INFO"finger = %u\n", finger));

			if (!dev)
				goto exit;
			input_report_abs(dev, ABS_X, x);
			input_report_abs(dev, ABS_Y, y);
			input_report_abs(dev, ABS_PRESSURE, pressure);
			input_report_abs(dev, ABS_TOOL_WIDTH, 10);
			input_report_key(dev, BTN_TOUCH, finger);
			input_sync(dev);
			goto exit;
		}
		case REG_HST_MODE_OP_BOOTLOADER:
			goto exit;
		default:
			printk(KERN_INFO"%s: Unknown OP-Mode: 0x%x\n",
				__func__, cy8_pkt->data[0]);
			goto pkt_err;
		}
	}

	if (cy8_pkt->data_idx == CY8_MAX_DATA_LENGTH) {
		printk(KERN_ERR "%s: buffer overflow!\n", __func__);
		cy8_pkt->pkt_idx = 0;
		cy8_pkt->data_idx = 0;
		goto pkt_err;
	}
pkt_ok:
	cy8_pkt->pkt_idx++;
	goto exit;
sync_cmplt:
	if (!completion_done(&sync->cmplt))
		complete(&sync->cmplt);
	goto exit;
pkt_err:
	printk(KERN_ERR "%s: packet error!\n", __func__);
	cy8ctma300_log_err_packet(cy8_pkt);
	cy8_pkt->pkt_idx = 0;
	cy8_pkt->data_idx = 0;
	cy8_pkt->err = 1;
exit:	return IRQ_HANDLED;
}

static int cy8ctma300_setup_input_device(struct cy8_ser *cy8_ser)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = CY8CTMA300_SER_DEV;
	input_dev->phys = cy8_ser->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_CYPRESS;
	input_dev->id.product = 0;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &cy8_ser->serio->dev;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, 0, cy8_ser->sysinfo.max_x - 1,
			     0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, cy8_ser->sysinfo.max_y - 1,
			     0, 0);
	input_set_abs_params(input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&cy8_ser->serio->dev, "%s: unable to register input\n",
			__func__);
		input_free_device(input_dev);
	}
	cy8_ser->dev = input_dev;
	snprintf(cy8_ser->phys, sizeof(cy8_ser->phys),
			"%s/input0", cy8_ser->serio->phys);
	dev_info(&cy8_ser->serio->dev, "Registered input device %s at %s\n",
		input_dev->name, cy8_ser->phys);
	return err;
}

static ssize_t cy8_fw_bin_write(struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cy8_ser *cy8_ser = dev_get_drvdata(dev);
	if(!size || (size - 1) > CY8_MAX_DATA_LENGTH) {
		printk(KERN_ERR "%s: Error, size:%d (max data len:%d)\n",
			__func__, size, CY8_MAX_DATA_LENGTH);
		return -1;
	}
	mutex_lock(&cy8_ser->mutex);
	cy8_ser->tx_pkt.reg = buf[0];
	cy8_ser->tx_pkt.data_len = (size - 1);
	memcpy(&cy8_ser->tx_pkt.data[0], &buf[1], (size - 1));
	cy8_ser->tx_pkt.chksum = cy8ctma300_ser_checksum(&cy8_ser->tx_pkt);
	(void)cy8ctma300_ser_send_packet(cy8_ser->serio, &cy8_ser->tx_pkt);
	if (cy8ctma300_ser_wait_for_any_compl_packet(cy8_ser))
		size = -EIO;
	mutex_unlock(&cy8_ser->mutex);
	return size;
}

static ssize_t cy8_fw_bin_read(struct kobject *kobj,
	struct bin_attribute *ba,
	char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cy8_ser *cy8_ser = dev_get_drvdata(dev);
	int count = 0;
	DBG(printk(KERN_INFO"%s: enter\n", __func__));

	mutex_lock(&cy8_ser->mutex);
	if (!cy8_ser->fw_update)
		goto exit;
	if (!size) {
		printk(KERN_ERR "%s: Error, size:%d (pkt data len:%d)\n",
			__func__, size, cy8_ser->rx_pkt.data_len);
		goto exit;
	}
	(void)cy8ctma300_ser_send_echo(cy8_ser->serio);
	if (cy8ctma300_ser_wait_for_any_compl_packet(cy8_ser))
		goto exit;
	count = sizeof(cy8_ser->rx_pkt.data);
	if (count > size)
		count = size;
	memcpy(buf, &cy8_ser->rx_pkt.data[0], count);
exit:	mutex_unlock(&cy8_ser->mutex);
	return count;
}

static struct bin_attribute cy8_fw_bin_rw = {
	.attr = {
		.name = "cy8_fw_bin_rw",
		.mode = 0644,
	},
	.size = 128,
	.read = cy8_fw_bin_read,
	.write = cy8_fw_bin_write,
};

static ssize_t cy8_fwload_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cy8_ser *cy8_ser = dev_get_drvdata(dev);
	return sprintf(buf, "max_x:0x%02X max_y:0x%02X "
			"bl_vers::0x%02X cust_id::0x%02X "
			"proj_id:0x%02X fw_vers:0x%02X\n",
			cy8_ser->sysinfo.max_x, cy8_ser->sysinfo.max_y,
			cy8_ser->sysinfo.bl_vers, cy8_ser->sysinfo.cust_id,
			cy8_ser->sysinfo.proj_id, cy8_ser->sysinfo.fw_vers);
}

static ssize_t cy8_dtid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cy8_ser *cy8_ser = dev_get_drvdata(dev);
	return sprintf(buf, "%02X%02X", cy8_ser->sysinfo.cust_id,
		       cy8_ser->sysinfo.proj_id);
}

static ssize_t cy8_fwload_ctrl(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	char *p;
	int ret;
	struct cy8_ser *cy8_ser = dev_get_drvdata(dev);
	unsigned val = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (*p && isspace(*p))
		ret++;
	printk(KERN_DEBUG "%s: %u\n", __func__, val);

	mutex_lock(&cy8_ser->mutex);
	if (val && !cy8_ser->fw_update) {
		cy8_ser->fw_update = 1;
		cy8ctma300_ser_xres();
		if (cy8ctma300_ser_wait_for_any_compl_packet(cy8_ser))
			goto exit;
		if (sysfs_create_bin_file(&dev->kobj, &cy8_fw_bin_rw))
			printk(KERN_ERR "%s: err creating bin-file\n",__func__);
		printk(KERN_INFO "%s: FW loader started.\n", __func__);
	} else if (!val && cy8_ser->fw_update) {
		sysfs_remove_bin_file(&dev->kobj, &cy8_fw_bin_rw);
		cy8ctma300_ser_xres();
		cy8_ser->fw_update = 0;
		if (cy8ctma300_ser_wait_for_any_compl_packet(cy8_ser))
			goto exit;
		if(cy8ctma300_ser_exit_bootloader(cy8_ser))
			goto exit;
		if(cy8ctma300_ser_set_sysinfo_mode(cy8_ser))
			goto exit;
		if (!cy8_ser->dev && cy8ctma300_setup_input_device(cy8_ser))
			goto exit;
		if (cy8ctma300_ser_set_normal_mode(cy8_ser))
			goto exit;
		printk(KERN_INFO "%s: FW loader finished.\n", __func__);
	}
exit:	mutex_unlock(&cy8_ser->mutex);
	return  ret == size ? ret : -EINVAL;
}

static struct device_attribute fwloader =
	__ATTR(cy8_fwloader, 0644, cy8_fwload_show, cy8_fwload_ctrl);
static struct device_attribute cy_dtid =
	__ATTR(dtid, 0444, cy8_dtid_show, NULL);

static void cy8ctma300_ser_disconnect(struct serio *serio)
{
	struct cy8_ser *cy8_ser = serio_get_drvdata(serio);
	DBG(printk(KERN_INFO"%s: enter\n", __func__));

	if (cy8_ser->dev) {
		input_get_device(cy8_ser->dev);
		input_unregister_device(cy8_ser->dev);
		input_put_device(cy8_ser->dev);
	}
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cy8_ser->early_suspend);
#endif
	kfree(cy8_ser);
}

static int cy8ctma300_ser_reconnect(struct serio *serio)
{
	DBG(printk(KERN_INFO "%s\n", __func__));
	return 0;
}

static int cy8ctma300_ser_connect(struct serio *serio, struct serio_driver *drv)
{
	struct cy8_ser *cy8_ser;
	int err;
	DBG(printk(KERN_INFO"%s: enter\n", __func__));

	cy8_ser = kzalloc(sizeof(struct cy8_ser), GFP_KERNEL);
	if (!cy8_ser)
		return -ENOMEM;

	cy8_ser->serio = serio;
	cy8_ser->sync_cmd.cmd = CMD_CY8_NO_CMD;
	init_completion(&cy8_ser->sync_cmd.cmplt);
	mutex_init(&cy8_ser->mutex);

	serio_set_drvdata(serio, cy8_ser);
	err = serio_open(serio, drv);
	if (err)
		goto error_connect1;

	err = -EINVAL;
	cy8_ser->mode = CY8_MODE_BOOTLOADER;
	if (cy8ctma300_ser_xres())
		goto error_connect2;

	/* Wait for any complete packet after reset, i.e. chip-alive ack. */
	if (cy8ctma300_ser_wait_for_any_compl_packet(cy8_ser))
		goto error_connect2;
	if (cy8ctma300_ser_exit_bootloader(cy8_ser)) {
		struct cy8_bl_map *bl =
			(struct cy8_bl_map *)cy8_ser->rx_pkt.data;
		cy8_ser->sysinfo.cust_id = bl->app_id_h;
		cy8_ser->sysinfo.proj_id = bl->app_id_l;
		dev_info(&serio->dev, "%s: registered in bootloader mode,"
			 "proj ID=%02x%02x, FW vers=%02x%02x\n",
			 __func__, bl->app_id_h, bl->app_id_l,
			 bl->app_ver_h, bl->app_ver_l);
		goto bypass;
	}
	if (cy8ctma300_ser_set_sysinfo_mode(cy8_ser))
		goto error_connect2;
	if (cy8ctma300_setup_input_device(cy8_ser))
		goto error_connect2;
	if (cy8ctma300_ser_set_normal_mode(cy8_ser))
		goto error_connect3;
bypass:

#ifdef CONFIG_HAS_EARLYSUSPEND
	cy8_ser->early_suspend.level = 1;
	cy8_ser->early_suspend.suspend = cy8ctma300_early_suspend;
	cy8_ser->early_suspend.resume = cy8ctma300_late_resume;
	register_early_suspend(&cy8_ser->early_suspend);
#endif
	err = device_create_file(&serio->dev, &fwloader);
	if (err) {
		printk(KERN_ERR "%s: Error, could not create attribute\n",
			__func__);
		goto error_fs_dev;
	}
	err = device_create_file(&serio->dev, &cy_dtid);
	if (err) {
		printk(KERN_ERR "%s: Error, could not create attribute\n",
			__func__);
		goto error_fs_dev_dtid;
	}
	return 0;

error_fs_dev_dtid:
	device_remove_file(&serio->dev, &fwloader);
error_fs_dev:
	unregister_early_suspend(&cy8_ser->early_suspend);
error_connect3:
	input_unregister_device(cy8_ser->dev);
error_connect2:
	serio_set_drvdata(serio, NULL);
	serio_close(serio);
error_connect1:
	kfree(cy8_ser);
	printk(KERN_ERR "%s: error!\n", __func__);
	return err;
}

static int cy8ctma300_pf_probe(struct platform_device *pdev)
{
	struct cy8ctma300_ser_platform_data *pdata =
		(struct cy8ctma300_ser_platform_data *)pdev->dev.platform_data;

	DBG(printk(KERN_INFO"%s: enter\n", __func__));
	cy8ctma300_ser_xres = cy8ctma300_ser_xres_dummy;
	if (pdata->xres)
		cy8ctma300_ser_xres = pdata->xres;
	else
		printk(KERN_INFO "%s: No registered xres function\n", __func__);

	return 0;
}

static struct serio_device_id cy8ctma300_ser_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_CYPRESS,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, cy8ctma300_ser_serio_ids);

static struct serio_driver cy8ctma300_ser_drv = {
	.driver		= {
		.name	= CY8CTMA300_SER_DEV,
	},
	.description	= DRIVER_DESC,
	.id_table	= cy8ctma300_ser_serio_ids,
	.interrupt	= cy8ctma300_ser_interrupt,
	.connect	= cy8ctma300_ser_connect,
	.disconnect	= cy8ctma300_ser_disconnect,
	.reconnect	= cy8ctma300_ser_reconnect,
};

static struct platform_driver cy8ctma300_pf_drv = {
	.probe = cy8ctma300_pf_probe,
	.driver = {
		.name = CY8CTMA300_SER_DEV,
		.owner = THIS_MODULE,
	},
};

static int __init cy8ctma300_ser_init(void)
{
	int rc;
	DBG(printk(KERN_INFO "%s: enter\n", __func__));
	rc = platform_driver_register(&cy8ctma300_pf_drv);
	if (rc) {
		printk(KERN_ERR "%s: Unsuccessfull pf drv reg\n", __func__);
		return rc;
	}
	rc = serio_register_driver(&cy8ctma300_ser_drv);
	if (rc)
		printk(KERN_ERR "%s: Unsuccessfull ser drv reg\n", __func__);
	return rc;

}

static void __exit cy8ctma300_ser_exit(void)
{
	DBG(printk(KERN_INFO "%s: enter\n", __func__));
	serio_unregister_driver(&cy8ctma300_ser_drv);
	platform_driver_unregister(&cy8ctma300_pf_drv);
}

module_init(cy8ctma300_ser_init);
module_exit(cy8ctma300_ser_exit);
