/* linux/drivers/input/touchscreen/cy8ctma300_spi.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * Author: Kenji Tokutake <Kenji.Tokutake@SonyEricsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


/*#define DEBUG*/
/*#define TOUCH_FWUPDATE_IGNORE*/
/*#define DEBUG_DUMMY_FW_CHECK*/
#define MODULE_VER	"1.00"

#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/cypress_touch.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/irq.h>

#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_ARM
#include <asm/mach-types.h>
#endif

#include "cy8ctma300_spi_fw.h"

/*--------------------------------------------------------------------------*/
/* Operating Mode Register Address */
/* touch data (R) */
#define TP_REG_NINT_TMO_FLAG		0x00
#define TP_REG_TOUCH_CTR		0x00
#define TP_REG_FNGR_CNT			0x00
#define TP_REG_FNGR_STAT		0x01
#define TP_REG_FNGR_ID			0x03
#define TP_REG_FNGR_XH			0x04
#define TP_REG_FNGR_XL			0x05
#define TP_REG_FNGR_YH			0x06
#define TP_REG_FNGR_YL			0x07
#define TP_REG_FNGR_Z			0x08

/* FW info (R) */
#define TP_REG_FW_VER			0x2F

/* control register (RW) */
#define TP_REG_SYS_CTRL			0x20
#define TP_REG_TMA_CTRL0		0x21	/* prescaler */
#define TP_REG_TMA_CTRL1		0x22	/* subconversion */
#define TP_REG_TMA_CTRL2		0x23	/* shift */
#define TP_REG_TMA_CTRL3		0x24	/* noisethreshold */
#define TP_REG_TMA_CTRL4		0x25	/* fingerthreshold */
#define TP_REG_TMA_CTRL5		0x26
#define TP_REG_NINT_TMO			0x27
#define TP_REG_INITMOVE_STEP		0x28
#define TP_REG_MOVE_STEP		0x29
#define TP_REG_AP_SLEEP_TIME		0x30

/* Operating Mode Register Values */
/* errors */
#define TP_VAL_FNGR_CNT_ERR	0x0F

/* finger status */
#define TP_VAL_FNGR_MOVE	0x08
#define TP_VAL_FNGR_DOWN	0x04

/* finger status mask */
#define TP_MASK_FNGR_MOVE	0x08
#define TP_MASK_FNGR_DOWN	0x04

/* Operating Mode Macros */
#define TP_GET_TOUCH_CTR(byte)	((byte &  0xF0) >> 4)
#define TP_GET_FNGR_CNT(byte)	(byte & 0x0F)
#define TP_CHK_NINT_TMO(byte)	((byte & 0x80) >> 7)
#define TP_CHK_MOVE(byte, offset) (((byte >> offset) & TP_MASK_FNGR_MOVE) >> 3)
#define TP_CHK_DOWN(byte, offset) (((byte >> offset) & TP_MASK_FNGR_DOWN) >> 2)
#define TP_SHIFT_FNGR_STAT_MS	4
#define TP_SHIFT_FNGR_STAT_LS	0

#define TP_TOUCH_MAJOR_MAX	255
#define TP_TOUCH_CNT_MAX	4

#define TP_TRACK_ACTIVE		1
#define TP_TRACK_DELETE		2
#define TP_TRACK_INACTIVE	3

#define TP_FNGR_NOTRACK		0
#define TP_FNGR_TRACK		1

#define SPI_DUMMY_DATA		3			/* AA,BB,CC */
#define TOUCH_DATA_BYTES	(27 + SPI_DUMMY_DATA)	/* 27 + 3 dummy data */
#define TOUCH_DATA_FCNT		3	/* offset to finger count byte */
#define TOUCH_DATA_FSTAT1	4	/* offset to finger status 1 byte */
#define TOUCH_DATA_FSTAT2	5	/* offset to finger status 1 byte */

#define TP_MAX_SPI_MSG		64	/* at least 50 registers + 3 dummy */

/* Bootloader Mode */
/* BL register address */
#define TP_REG_BL		0xFF

/* BL command */
#define TP_BL_CMD_WRITE_BLOCK	0x39

/* BL write block command packet */
#define TP_BL_OFFSET_BL_OFFSET	2
#define TP_BL_OFFSET_KEY	3
#define TP_BL_OFFSET_CMD	11
#define TP_BL_OFFSET_BLOCK_NUM	13
#define TP_BL_OFFSET_DATA	14
#define TP_BL_OFFSET_DATA_CHK	142
#define TP_BL_OFFSET_CMD_CHK	143
#define TP_BL_OFFSET_EXEC_CMD	144

/* firmware download related stuff */
#define TP_FIRMWARE_BLOCK_SIZE		128
#define TP_FIRMWARE_WFB_SIZE		146
#define TP_FIRMWARE_DATA_SIZE		128
#define TP_FIRMWARE_SUPPORTED		0x1A
#define TP_FIRMWARE_SIZE		32769
#define TP_FIRMWARE_BLOCK_MAX		229
#define TP_FIRMWARE_BLOCK_APP_CHKSUM	255
#define TP_FIRMWARE_BLOCK_DATA		1
#define TP_FIRMWARE_UPDATE_MAX		1

#ifdef DEBUG
#define DBGLOG(format, args...)				\
	do {						\
		printk(KERN_DEBUG format, ## args);	\
	} while (0);

#define ERRLOG(format, args...)				\
	do {						\
		printk(KERN_ERR format, ## args);	\
	} while (0);
#else
#define DBGLOG(format, args...)
#define ERRLOG(format, args...)
#endif

/*--------------------------------------------------------------------------*/
struct cy8ctma300_spi_touch {
	u8 id;
	u16 __attribute__ ((packed)) x;
	u16 __attribute__ ((packed)) y;
	u8 z;
};

struct cy8ctma300_spi_data {
	u8 touch_status;
	u8 finger_status[2];
	struct cy8ctma300_spi_touch finger_data[TP_TOUCH_CNT_MAX];
};

struct cy8ctma300_touch {
	struct input_dev *input;
	spinlock_t lock;
	struct spi_device *spi;
	struct work_struct setup_work;
	struct work_struct mt_work;
	struct spi_message async_spi_message;
	struct spi_transfer async_spi_transfer;
	u8 async_read_buf[TOUCH_DATA_BYTES];
	struct cdev device_cdev;
	int device_major;
	int first_irq;
	int fwupd_err_count;
	u8 fw_version;
	struct cy8ctma300_spi_touch mt_pos[TP_TOUCH_CNT_MAX];
	u8 active_track_cnt;
	u8 track_state[TP_TOUCH_CNT_MAX];
	u8 track_detect[TP_TOUCH_CNT_MAX];
#ifdef CONFIG_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int init_complete;
	struct task_struct *kmain_task;
};

/* send packet to get touch data from device */
static const u8 cy8ctma300_send_echo[TOUCH_DATA_BYTES] =
	{ 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*--------------------------------------------------------------------------*/
#ifdef	DEBUG
static void dump_buf(u8 *buf, unsigned len)
{
	int i;

	if (!len || !buf)
		return;

	DBGLOG("%s: dump_buf (%03d): ", __func__, len);
	for (i = 0; i < len; i++)
		printk("0x%02X ", buf[i]);
	printk("\n");
};

static void dump_buf_parse(u8 *buf, unsigned len)
{
	int i = 0;
	int ct = 0;
	u8 fdetect = 0;
	u16 x, y, z;
	u8 fstat = 0;
	u8 offset = 0;

	DBGLOG("%s: (%03d): \n", __func__, len);

	if (!len || !buf)
		return;

	fdetect = TP_GET_FNGR_CNT(buf[TP_REG_FNGR_CNT]);

	DBGLOG("%s: nINT Timeout(0x00/b7)=%d\n", __func__,
		TP_CHK_NINT_TMO(buf[TP_REG_NINT_TMO_FLAG]));

	DBGLOG("%s: FingerNum(0x00/b3-b0)=%d\n", __func__, fdetect);

	for (i = 0; i < TP_TOUCH_CNT_MAX; i++) {
		ct = 6 * i;
		x = buf[TP_REG_FNGR_XL + ct] | (buf[TP_REG_FNGR_XH + ct] << 8);
		y = buf[TP_REG_FNGR_YL + ct] | (buf[TP_REG_FNGR_YH + ct] << 8);
		z = buf[TP_REG_FNGR_Z + ct];

		fstat = buf[((i + 1) / 2) + ((i + 1) % 2)];

		offset = (i + 1) % 2 ? TP_SHIFT_FNGR_STAT_MS :
						TP_SHIFT_FNGR_STAT_LS;

		DBGLOG("%s: Finger(%d) STAT(mov=%d dwn=%d) ID=%d, X=%d, Y=%d, "
			"Z=%d \n", __func__, i + 1, TP_CHK_MOVE(fstat,
			offset), TP_CHK_DOWN(fstat, offset),
			buf[TP_REG_FNGR_ID + ct], x, y, z);
	}
};
#endif

static int reg_write(struct spi_device *spi, u8 addr, u8 * buf, size_t len)
{
	int err = 0;
	u8 write_buf[TP_MAX_SPI_MSG];

	DBGLOG("%s: reg_write(0x%02X, %d)\n", __func__, addr, len);

	if (!buf || !len || (len > (ARRAY_SIZE(write_buf) - SPI_DUMMY_DATA)))
		return -EINVAL;

#ifdef DEBUG
	dump_buf(buf, len);
#endif

	write_buf[0] = addr;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;

	/* copy the buffer data to write_buf */
	memcpy(&write_buf[3], buf, len);

	err = spi_write(spi, write_buf, len + SPI_DUMMY_DATA);

	if (err)
		ERRLOG("%s: reg_write error %d\n", __func__, err);

	return err;
};

static inline int reg_write_byte(struct spi_device *spi, u8 addr, u8 data)
{
	return reg_write(spi, addr, &data, 1);
};

static int reg_read(struct spi_device *spi, u8 addr, u8 * buf, size_t len)
{
	int err = 0;
	u8 read_buf[TP_MAX_SPI_MSG];

	struct spi_message sync_spi_message;
	struct spi_transfer sync_spi_transfer;

	DBGLOG("%s: reg_read (0x%02X, %d)\n", __func__, addr, len);

	if (!buf || !len || (len > (ARRAY_SIZE(read_buf) - SPI_DUMMY_DATA)))
		return -EINVAL;

	memset(&sync_spi_transfer, 0, sizeof(sync_spi_transfer));

	read_buf[0] = 0x80 | addr;

	sync_spi_transfer.tx_buf = read_buf;
	sync_spi_transfer.rx_buf = read_buf;
	sync_spi_transfer.len = len + SPI_DUMMY_DATA;

	spi_message_init(&sync_spi_message);
	spi_message_add_tail(&sync_spi_transfer, &sync_spi_message);

	DBGLOG("%s: reg_read spi_sync start\n", __func__);

	err = spi_sync(spi, &sync_spi_message);

	if (err) {
		printk(KERN_ERR "%s: reg_read error %d\n", __func__, err);
	} else {
#ifdef DEBUG
		dump_buf(read_buf, len + SPI_DUMMY_DATA);
#endif
		memcpy(buf, read_buf + SPI_DUMMY_DATA, len);
	}

	return err;
};

static int reg_read_bl(struct spi_device *spi, u8 * buf, size_t len)
{
	int err = 0;
	u8 read_buf[TP_MAX_SPI_MSG];
	u8 write_buf[2];

	struct spi_message sync_spi_message;
	struct spi_transfer sync_spi_transfer;

	if (!buf || !len || (len > (ARRAY_SIZE(read_buf) - SPI_DUMMY_DATA)))
		return -EINVAL;

	memset(&sync_spi_transfer, 0, sizeof(sync_spi_transfer));

	write_buf[0] = 0x80;
	write_buf[1] = 0x00;

	sync_spi_transfer.tx_buf = write_buf;
	sync_spi_transfer.rx_buf = read_buf;
	sync_spi_transfer.len = len;

	spi_message_init(&sync_spi_message);
	spi_message_add_tail(&sync_spi_transfer, &sync_spi_message);

	err = spi_sync(spi, &sync_spi_message);

	if (err)
		printk(KERN_ERR "%s: reg_read error %d\n", __func__, err);
	else
		memcpy(buf, read_buf, len);

	return err;
};


static int reg_write_then_read_bl(struct spi_device *spi, u8 *wbuf,
				size_t wlen, u8 *rbuf, size_t rlen, u8 dump)
{
	int err = 0;

	if (!wbuf || !wlen)
		return -EINVAL;

#ifndef DEBUG_DUMMY_FW_CHECK
	err = spi_write(spi, wbuf, wlen);
#endif

	if (err) {
		printk(KERN_ERR "%s: write error %d\n", __func__, err);
		return err;
	}
#ifdef DEBUG
	if (dump)
		dump_buf(wbuf, wlen);
#endif

	mdelay(100);

	if (rbuf && rlen) {
#ifndef DEBUG_DUMMY_FW_CHECK
		err = reg_read_bl(spi, rbuf, rlen);
#endif
#ifdef DEBUG
	if (!err && dump)
		dump_buf(rbuf, rlen);
#endif
	}

	return err;
};

#if defined(TOUCH_CALIB_INIT) && defined(DEBUG)
static void dump_calib_register(struct spi_device *spi)
{
	u8 read_buf = 0xFF;	/* 0xff is set for update*/

	/* Prescaler */
	if (!reg_read(spi, TP_REG_TMA_CTRL0, &read_buf, 1))
		DBGLOG("%s: Prescaler=%x \n", __func__, read_buf);
	/* Subconversion */
	if (!reg_read(spi, TP_REG_TMA_CTRL1, &read_buf, 1))
		DBGLOG("%s: subconversion=%x \n", __func__, read_buf);
	/* Shift Data */
	if (!reg_read(spi, TP_REG_TMA_CTRL2, &read_buf, 1))
		DBGLOG("%s: shift=%x \n", __func__, read_buf);
	/* Noise Threshold */
	if (!reg_read(spi, TP_REG_TMA_CTRL3, &read_buf, 1))
		DBGLOG("%s: noisethreshold=%x \n", __func__, read_buf);
	if (!reg_read(spi, TP_REG_TMA_CTRL4, &read_buf, 1))
		DBGLOG("%s: fingerthreshold=%x \n", __func__, read_buf);
	/* nINT assertion Timeout */
	if (!reg_read(spi, 0x27, &read_buf, 1))
		DBGLOG("%s: nINT assertion Timeout=%x \n", __func__, read_buf);

	if (!reg_read(spi, 0x28, &read_buf, 1))
		DBGLOG("%s: Initial Step move=%x \n", __func__, read_buf);

	if (!reg_read(spi, 0x29, &read_buf, 1))
		DBGLOG("%s: Move step=%x \n", __func__, read_buf);
}
#endif

static int reg_read_intcause_async(struct spi_device *spi, void *complete)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);

	DBGLOG("%s: start\n", __func__);

	tp->async_spi_transfer.tx_buf = cy8ctma300_send_echo;
	tp->async_spi_transfer.rx_buf = tp->async_read_buf;

	tp->async_spi_transfer.len = TOUCH_DATA_BYTES;

	spi_message_init(&tp->async_spi_message);
	spi_message_add_tail(&tp->async_spi_transfer, &tp->async_spi_message);
	tp->async_spi_message.complete = complete;
	tp->async_spi_message.context = (void *)tp;

	return spi_async(spi, &tp->async_spi_message);
};

static int reset_device(struct cy8ctma300_touch *tp)
{
	struct spi_device *spi = tp->spi;
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	int reset_time = 250;

	/* send reset signal to device (Active Low) */
	gpio_set_value(pdata->gpio_reset_pin, 0);
	udelay(reset_time);
	gpio_set_value(pdata->gpio_reset_pin, 1);

	return 0;
};

static int cy8ctma300_update_firmware(void *tp_arg)
{
	struct cy8ctma300_touch *tp = tp_arg;
	struct spi_device *spi = tp->spi;
	u8 BootloaderKeys[11] = {0x00, 0x00, 0xFF, 0x00, 0x01, 0x02, 0x03,
				 0x04, 0x05, 0x06, 0x07};
	u8 BootloaderExit[12] = {0x00, 0x00, 0xFF, 0x00, 0x01, 0x02, 0x03,
				 0x04, 0x05, 0x06, 0x07, 0x3B};
	int err = 0;
	char *fw_buf;
	u8 read_buf[5] = {0, 0, 0, 0, 0};
	u8 WriteFlashBlock[TP_FIRMWARE_WFB_SIZE];	/* key + block*/
	u8 cmd_checksum_init = 0;
	u16 app_checksum_all = 0;
	int block_count = 0;
	int i;
	int j;
	u8 dump = 0;

	DBGLOG("%s: start \n", __func__);

	/* ENTER boot loader mode end */
	fw_buf = (char *)firmware;

	/* byte00-byte10 (write + direction + bootloaderkey) */
	memcpy(WriteFlashBlock, BootloaderKeys, ARRAY_SIZE(BootloaderKeys));

	/* byte 11 (write block command) */
	WriteFlashBlock[TP_BL_OFFSET_CMD] = TP_BL_CMD_WRITE_BLOCK;

	/* initialize byte12-byte13 (block number) */
	WriteFlashBlock[TP_BL_OFFSET_BLOCK_NUM - 1] = 0x00;
	WriteFlashBlock[TP_BL_OFFSET_BLOCK_NUM] = 0x00;

	/* byte144-byte145 (execute command) */
	WriteFlashBlock[TP_BL_OFFSET_EXEC_CMD] = 0xCA;
	WriteFlashBlock[TP_BL_OFFSET_EXEC_CMD + 1] = 0x53;

	/* count the checksum */
	for (i = 2; i <= 11; i++)
		cmd_checksum_init += WriteFlashBlock[i];

	/* add 1 for app checksum block (255) */
	block_count = TP_FIRMWARE_BLOCK_MAX + 1;

	for (i = TP_FIRMWARE_BLOCK_DATA; i <= block_count; i++) {

		/* initialize checksums */
		/* data checksum */
		WriteFlashBlock[TP_BL_OFFSET_DATA_CHK] = 0;
		/* cmd checksum */
		WriteFlashBlock[TP_BL_OFFSET_CMD_CHK] = cmd_checksum_init;

		/* set block number */
		if (i == block_count) {
			WriteFlashBlock[TP_BL_OFFSET_BLOCK_NUM] = 0xFF;
			memcpy(&WriteFlashBlock[TP_BL_OFFSET_DATA],
				&fw_buf[TP_FIRMWARE_BLOCK_SIZE *
				TP_FIRMWARE_BLOCK_APP_CHKSUM],
				TP_FIRMWARE_BLOCK_SIZE);
			/*
			 * update the appchecksum value (last 2 data bytes)
			 * assuming target is big endian
			 */
			WriteFlashBlock[TP_BL_OFFSET_DATA + 126] =
						(u8)(app_checksum_all>>8);
			WriteFlashBlock[TP_BL_OFFSET_DATA + 127] =
						(u8)(app_checksum_all&0x00FF);

			DBGLOG("%s: chksumblk1-blk229 0x%X 0x%X \n", __func__,
				WriteFlashBlock[TP_BL_OFFSET_DATA + 126],
				WriteFlashBlock[TP_BL_OFFSET_DATA + 127]);
		} else {
			WriteFlashBlock[TP_BL_OFFSET_BLOCK_NUM] = i;
			memcpy(&WriteFlashBlock[TP_BL_OFFSET_DATA],
				&fw_buf[TP_FIRMWARE_BLOCK_SIZE * i],
				TP_FIRMWARE_BLOCK_SIZE);
		}

		/* calculate data checksum */
		for (j = 14; j < TP_FIRMWARE_BLOCK_SIZE + 14; j++) {
			WriteFlashBlock[TP_BL_OFFSET_DATA_CHK] +=
						WriteFlashBlock[j];

			/* for getting the app chksum */
			app_checksum_all += (u16)WriteFlashBlock[j];
		}

		/* calculate cmd checksum */
		WriteFlashBlock[TP_BL_OFFSET_CMD_CHK] +=
				(WriteFlashBlock[TP_BL_OFFSET_DATA_CHK] * 2 +
				WriteFlashBlock[TP_BL_OFFSET_BLOCK_NUM]);

		err = reg_write_then_read_bl(spi, WriteFlashBlock,
			ARRAY_SIZE(WriteFlashBlock), read_buf, 5, dump);

		if (err) {
			ERRLOG("%s: write/read firmware error\n", __func__);
			goto err_r_fs_irq;
		}

#ifdef DEBUG_DUMMY_FW_CHECK
		read_buf[0] = 0x03;
		read_buf[1] = 0x3F;
		if (i == 1)
			read_buf[1] = 0x07;
#endif

		mdelay(20);
	}

	/* EXIT bootloader mode start */
	err = reg_write_then_read_bl(spi, BootloaderExit,
			ARRAY_SIZE(BootloaderExit), NULL, 0, 1);

	if (err) {
		ERRLOG("%s: write/read firmware error\n", __func__);
		goto err_r_fs_irq;
	}
	DBGLOG("%s: BootloaderExit\n", __func__);
	/* EXIT bootloader mode end */

err_r_fs_irq:
	DBGLOG("%s: Return=%d\n", __func__, err);
	reset_device(tp);

	/* workaround for FW checksum calculation */
	msleep(500);

	/* add - workaround for sequence schedule again */
	schedule_work(&tp->setup_work);

	return err;
};

static void cy8ctma300_setup_work(struct work_struct *work)
{
	struct cy8ctma300_touch *tp =
		container_of(work, struct cy8ctma300_touch, setup_work);
	struct spi_device *spi = tp->spi;
	int err = 0;
	u8 read_buffer = 0xFF;	/* 0xff is set for update*/
	u8 bl_write_buf[4] = {'S', 'E', 'M', 'C'};
	u8 read_buf[5] = {0, 0, 0, 0, 0};
	u8 BootloaderEnter[12] = {0x00, 0x00, 0xFF, 0x00, 0x01, 0x02, 0x03,
						0x04, 0x05, 0x06, 0x07, 0x38};

	/* Skip FW update */
	if (!tp->first_irq)
		goto start_second_irq;

	/* workaround to start again the timing */
	reset_device(tp);
	msleep(300);

	/* Get FW version */
	if (reg_read(spi, TP_REG_FW_VER, &read_buffer, 1)) {
		DBGLOG("%s: error reading TP_REG_FW_VER \n", __func__);
		goto err_enable_irq;
	} else if (read_buffer == 0xFF) {
		/*
		 * temporary workaround if firmware update
		 * is not yet ready
		 */
		DBGLOG("%s: 500ms delay \n", __func__);
		msleep(500);	/* additional wait time*/
		if (reg_read(spi, TP_REG_FW_VER, &read_buffer, 1)) {
			DBGLOG("%s: error reading TP_REG_FW_VER \n", __func__);
			goto err_enable_irq;
		}
	}
	tp->fw_version = read_buffer;
	printk(KERN_INFO "%s: FW version 0x%02X \n", __func__, tp->fw_version);

	tp->first_irq = 0;

#ifdef TOUCH_FWUPDATE_IGNORE
	/* skip the FW update */
	read_buffer = firmware[TP_FIRMWARE_SIZE-1];
#endif

	/* get the value of firmware supported from the header file */
	if (read_buffer < firmware[TP_FIRMWARE_SIZE-1] &&
		tp->fwupd_err_count < TP_FIRMWARE_UPDATE_MAX) {

		DBGLOG("%s: FW update \n", __func__);
		err = reg_write(spi, 0x2A, bl_write_buf, 4);
		if (err) {
			ERRLOG("%s: write BL register error\n", __func__);
			goto err_enable_irq;
		}
		msleep(300);

		/* ENTER boot loader mode start */
		err = reg_write_then_read_bl(tp->spi, BootloaderEnter,
			ARRAY_SIZE(BootloaderEnter), read_buf, 5, 1);
		if (err) {
			ERRLOG("%s: write/read error\n", __func__);
			goto err_enable_irq;
		}

#ifdef DEBUG_DUMMY_FW_CHECK
		read_buf[0] = 0x01;
		read_buf[1] = 0x00;
#endif
		/*
		 * change the check value
		 * (to be checked based on BL specs)
		 */
		if ((read_buf[0] != 0x01) || (read_buf[1] != 0x00)) {
			ERRLOG("%s: write enter state error read_buf=%x,%x\n",
				__func__, read_buf[0], read_buf[1]);
			err = -1;
		}
		/* ENTER boot loader mode end */

		/* flag for FW update try */
		tp->fwupd_err_count++;

		if (err) {
			ERRLOG("%s: BootloaderEnter error read_buf=%x,%x\n",
				__func__, read_buf[0], read_buf[1]);

			/* To return to normal mode */
			reset_device(tp);

			/* workaround for FW checksum calculation */
			msleep(300);

			goto start_allpoint_mode;
		} else {
			DBGLOG("%s: BootloaderEnter OK read_buf=%x,%x\n",
				__func__, read_buf[0], read_buf[1]);

			/* Start FW update operation */
			wake_up_process(tp->kmain_task);
			goto out;
		}
	}

start_second_irq:
	if (tp->fwupd_err_count) {
		/*
		 * limitation - additional 500ms delay
		 * on first boot after flash
		 */
		DBGLOG("%s: delay after fwupdate 500ms\n", __func__);
		msleep(500);
		tp->fwupd_err_count = 0;
		tp->fw_version = firmware[TP_FIRMWARE_SIZE-1];
		printk(KERN_INFO "%s: updated FW version 0x%02X \n",
				__func__, tp->fw_version);
	}

#ifdef TOUCH_CALIB_INIT
	/* check calibration */
#ifdef DEBUG
	dump_calib_register(spi);
#endif

	/* update the calibration registers */
	/* Prescaler */
	reg_write_byte(spi, TP_REG_TMA_CTRL0, 0x46);
	/* Subconversion */
	reg_write_byte(spi, TP_REG_TMA_CTRL1, 0x08);
	/* Shift Data */
	reg_write_byte(spi, TP_REG_TMA_CTRL2, 0x04);
	/* Noise Threshold */
	reg_write_byte(spi, TP_REG_TMA_CTRL3, 0x07);
	/* Finger Threshold */
	reg_write_byte(spi, TP_REG_TMA_CTRL4, 0x01);
	/* nINT assertion Timeout */
	reg_write_byte(spi, TP_REG_NINT_TMO, 0x64);
	/* Initial Step Value for Move */
	reg_write_byte(spi, TP_REG_INITMOVE_STEP, 0x08);
	/* Move step size after first move */
	reg_write_byte(spi, TP_REG_MOVE_STEP, 0x08);

	/* Calibration start command */
	reg_write_byte(spi, TP_REG_SYS_CTRL, 0x40);

#ifdef DEBUG
	dump_calib_register(spi);
#endif

	msleep(100);	/* to be compatible between v1.07 and v1.10 for now */
#endif

	/* Adjust to 60Hz */
	reg_write_byte(spi, TP_REG_AP_SLEEP_TIME, 0x01);

#ifdef DEBUG
	read_buffer = 0xFF;
	if (!reg_read(spi, TP_REG_AP_SLEEP_TIME, &read_buffer, 1))
		DBGLOG("%s: AP_SLEEP_TIME=%x \n", __func__, read_buffer);
#endif

start_allpoint_mode:
	if (tp->fw_version >= 0x19) {
		/* start all-point mode */
		reg_write_byte(spi, TP_REG_SYS_CTRL, 0x20);
	}

err_enable_irq:
	enable_irq(tp->spi->irq);

out:
	return;
}


static void cy8ctma300_update_track(struct cy8ctma300_touch *tp, int track,
			struct cy8ctma300_spi_data *cur_touch, u8 fdetect)
{
	int i;
	int found = 0;
	u8 fstat = 0;
	u8 offset = 0;

	/* find finger and update */
	for (i = 0; i < fdetect; i++) {
		fstat = cur_touch->finger_status[(i / 2)];
		offset = (i + 1) %  2 ? TP_SHIFT_FNGR_STAT_MS :
						TP_SHIFT_FNGR_STAT_LS;

		if ((TP_CHK_MOVE(fstat, offset)) && (tp->mt_pos[track].id ==
					cur_touch->finger_data[i].id)) {
			/* correct x,y MSB and LSB */
			cur_touch->finger_data[i].x =
				((cur_touch->finger_data[i].x & 0x00FF) << 8) |
				((cur_touch->finger_data[i].x & 0xFF00) >> 8);
			cur_touch->finger_data[i].y =
				((cur_touch->finger_data[i].y & 0x00FF) << 8) |
				((cur_touch->finger_data[i].y & 0xFF00) >> 8);

			/* swap x and y */
			tp->mt_pos[track].x = cur_touch->finger_data[i].y;
			tp->mt_pos[track].y = cur_touch->finger_data[i].x;
			tp->mt_pos[track].z = cur_touch->finger_data[i].z;

			DBGLOG("%s: MT track updated \n", __func__);
			found = 1;
			tp->track_detect[i] = TP_FNGR_TRACK;
			break;
		}

		/* not move finger found */
		if (tp->mt_pos[track].id == cur_touch->finger_data[i].id) {
			found = 1;
			tp->track_detect[i] = TP_FNGR_TRACK;
			break;
		}
	}

	/* delete track if not detected */
	if (!found) {
		tp->track_state[track] = TP_TRACK_DELETE;

		if (fdetect > 0)
			tp->active_track_cnt--;

		DBGLOG("%s: MT track deleted \n", __func__);
	}

};


static void cy8ctma300_new_track(struct cy8ctma300_touch *tp, int track,
			struct cy8ctma300_spi_data *cur_touch, u8 fdetect)
{
	int i;
	u8 fstat = 0;
	u8 offset = 0;

	/* find down detect and add to track */
	for (i = 0; i < fdetect; i++) {

		if (tp->track_detect[i] == TP_FNGR_TRACK)
			continue;

		fstat = cur_touch->finger_status[(i / 2)];
		offset = (i + 1) %  2 ? TP_SHIFT_FNGR_STAT_MS :
						TP_SHIFT_FNGR_STAT_LS;

		if (TP_CHK_DOWN(fstat, offset)) {
			/* correct x,y MSB and LSB */
			cur_touch->finger_data[i].x =
				((cur_touch->finger_data[i].x & 0x00FF) << 8) |
				((cur_touch->finger_data[i].x & 0xFF00) >> 8);
			cur_touch->finger_data[i].y =
				((cur_touch->finger_data[i].y & 0x00FF) << 8) |
				((cur_touch->finger_data[i].y & 0xFF00) >> 8);

			tp->mt_pos[track].id = cur_touch->finger_data[i].id;
			/* swap x and y */
			tp->mt_pos[track].x = cur_touch->finger_data[i].y;
			tp->mt_pos[track].y = cur_touch->finger_data[i].x;
			tp->mt_pos[track].z = cur_touch->finger_data[i].z;
			tp->track_state[track] = TP_TRACK_ACTIVE;
			tp->active_track_cnt++;

			tp->track_detect[i] = TP_FNGR_TRACK;
			DBGLOG("%s: MT track added \n", __func__);
			break;
		}
	}
};


static void cy8ctma300_mt_work(struct work_struct *work)
{
	struct cy8ctma300_touch *tp =
		container_of(work, struct cy8ctma300_touch, mt_work);
	struct cy8ctma300_spi_data cur_touch;
	int i = 0;
	u8 fdetect = 0;
	u8 report = 0;

	memcpy(&cur_touch, tp->async_read_buf + SPI_DUMMY_DATA,
		 TOUCH_DATA_BYTES - SPI_DUMMY_DATA);

	fdetect = TP_GET_FNGR_CNT(cur_touch.touch_status);

	if (tp->active_track_cnt == 0)
		memset(&tp->track_state[0], TP_TRACK_INACTIVE,
						ARRAY_SIZE(tp->track_state));

	memset(&tp->track_detect[0], TP_FNGR_NOTRACK,
						ARRAY_SIZE(tp->track_detect));

	for (i = 0; i < TP_TOUCH_CNT_MAX; i++) {
		if (tp->track_state[i] == TP_TRACK_ACTIVE)
			cy8ctma300_update_track(tp, i, &cur_touch, fdetect);
		else if (tp->track_state[i] == TP_TRACK_INACTIVE)
			cy8ctma300_new_track(tp, i, &cur_touch, fdetect);

		if (tp->track_state[i] == TP_TRACK_DELETE) {
			input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 0);
			tp->track_state[i] = TP_TRACK_INACTIVE;

			report = 1;
			DBGLOG("%s: MT report removed finger \n", __func__);
		} else if (tp->track_state[i] == TP_TRACK_ACTIVE) {
			input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR,
						tp->mt_pos[i].z);

			report = 1;
			DBGLOG("%s: MT report active finger \n", __func__);
		} else {
			report = 0;
		}

		if (report) {
			input_report_abs(tp->input, ABS_MT_TRACKING_ID,
						tp->mt_pos[i].id);
			input_report_abs(tp->input, ABS_MT_POSITION_X,
						tp->mt_pos[i].x);
			input_report_abs(tp->input, ABS_MT_POSITION_Y,
						tp->mt_pos[i].y);
			input_mt_sync(tp->input);
		}
	}
	input_sync(tp->input);

	if (fdetect == 0)
		tp->active_track_cnt = 0;

	enable_irq(tp->spi->irq);
}


static void cy8ctma300_touch_isr(void *context)
{
	struct cy8ctma300_touch *tp = (struct cy8ctma300_touch *)context;
	u8 fdetect = 0;

	DBGLOG("%s: start\n", __func__);

#ifdef DEBUG
	dump_buf(tp->async_read_buf, TOUCH_DATA_BYTES);
	dump_buf_parse(tp->async_read_buf + SPI_DUMMY_DATA,
			 TOUCH_DATA_BYTES - SPI_DUMMY_DATA);
#endif

	fdetect = TP_GET_FNGR_CNT(tp->async_read_buf[TOUCH_DATA_FCNT]);

	if (fdetect > TP_TOUCH_CNT_MAX) {
		ERRLOG("%s: Error Invalid detected=%d \n", __func__, fdetect);
		enable_irq(tp->spi->irq);
		goto out;
	}

	if (tp->async_read_buf[TOUCH_DATA_FSTAT1] == 0 &&
				tp->async_read_buf[TOUCH_DATA_FSTAT2] == 0) {
		DBGLOG("%s: schedule_work()\n", __func__);
		schedule_work(&tp->setup_work);
		goto out;
	}

	/* schedule finger info processing */
	schedule_work(&tp->mt_work);

out:
	return;
};

static irqreturn_t cy8ctma300_touch_irq(int irq, void *handle)
{
	struct cy8ctma300_touch *tp = handle;
	unsigned long flags;

	DBGLOG("%s: start \n", __func__);

	spin_lock_irqsave(&tp->lock, flags);

	disable_irq(tp->spi->irq);

	reg_read_intcause_async(tp->spi, cy8ctma300_touch_isr);
	spin_unlock_irqrestore(&tp->lock, flags);

	return IRQ_HANDLED;
};

/*--------------------------------------------------------------------------*/

static int cy8ctma300_touch_suspend(struct spi_device *spi,
				pm_message_t message)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	disable_irq(tp->spi->irq);
	return 0;
}

static int cy8ctma300_touch_resume(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	enable_irq(tp->spi->irq);
	return 0;
};

#ifdef CONFIG_EARLYSUSPEND
static void cy8ctma300_touch_early_suspend(struct early_suspend *es)
{
	struct cy8ctma300_touch *tp;
	tp = container_of(es, struct cy8ctma300_touch, early_suspend);

	DBGLOG("%s: early suspend\n", __func__);

	cy8ctma300_touch_suspend(tp->spi, PMSG_SUSPEND);
};

static void cy8ctma300_touch_late_resume(struct early_suspend *es)
{
	struct cy8ctma300_touch *tp;
	tp = container_of(es, struct cy8ctma300_touch, early_suspend);

	DBGLOG("%s: late resume\n", __func__);

	cy8ctma300_touch_resume(tp->spi);
};
#endif

static int cy8ctma300_touch_open(struct inode *inode, struct file *file)
{
	struct cy8ctma300_touch *tp =
	container_of(inode->i_cdev, struct cy8ctma300_touch, device_cdev);

	DBGLOG("%s: open start %d\n", __func__, tp->init_complete);

	file->private_data = tp;
	return 0;
};

static int cy8ctma300_touch_release(struct inode *inode, struct file *file)
{
	return 0;
};


static int do_calibration_valset(struct spi_device *spi,
			struct cy8ctma300_touch_ioctl_clbr *data)
{
	int err = -1;
	u8 read_buffer;

	disable_irq(spi->irq);

	DBGLOG("%s: prescaler=%d\n", __func__, data->prescaler);
	DBGLOG("%s: subconversion=%d\n", __func__, data->subconversion);
	DBGLOG("%s: shift=%d\n", __func__, data->shift);
	DBGLOG("%s: noisethreshold=%d\n", __func__, data->noisethreshold);
	DBGLOG("%s: fingerthreshold=%d\n", __func__, data->fingerthreshold);
	switch (data->clbr_num) {
	case 0:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL0, data->prescaler);
		break;
	case 1:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL1,
						data->subconversion);
		break;
	case 2:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL2, data->shift);
		break;
	case 3:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL3,
						data->noisethreshold);
		break;
	case 4:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL4,
						data->fingerthreshold);
		break;
	/* put additional control register */
	case 5:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL0, data->prescaler);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL1,
						data->subconversion);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL2, data->shift);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL3,
						data->noisethreshold);
		if (err)
			break;
		err = reg_write_byte(spi, TP_REG_TMA_CTRL4,
						data->fingerthreshold);
		break;
	case 6:
		err = reg_write_byte(spi, TP_REG_INITMOVE_STEP,
					data->initmovestep);
		break;
	case 7:
		err = reg_write_byte(spi, TP_REG_MOVE_STEP,
						data->movestep);
		break;
	/* put additional control register */
	case 8:
		err = reg_write_byte(spi, TP_REG_TMA_CTRL0,
						data->prescaler);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL1,
						data->subconversion);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL2,
						data->shift);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL3,
						data->noisethreshold);
		err = reg_write_byte(spi, TP_REG_TMA_CTRL4,
						data->fingerthreshold);
		err = reg_write_byte(spi, TP_REG_NINT_TMO,
						data->ninttimeout);
		err = reg_write_byte(spi, TP_REG_INITMOVE_STEP,
						data->initmovestep);
		err = reg_write_byte(spi, TP_REG_MOVE_STEP,
						data->movestep);
		break;
	default:
		err = -EINVAL;
		ERRLOG("%s: clbr_num error %d\n", __func__, data->clbr_num);
		enable_irq(spi->irq);
		goto done;
		break;
	}

	if (err) {
		enable_irq(spi->irq);
		ERRLOG("%s: error reading register \n", __func__);
		err = -EIO;
		goto done;
	}

	/* update system control to start calibration */
	err = reg_write_byte(spi, TP_REG_SYS_CTRL, 0x40);
	if (err) {
		enable_irq(spi->irq);
		ERRLOG("%s: error writing TP_REG_SYS_CTRL \n", __func__);
		err = -EIO;
		goto done;
	}

	/* adjust or remove this delay */
	msleep(200);

	/* Check if register was set then reset */
	err = reg_read(spi, TP_REG_SYS_CTRL, &read_buffer, 1);
	if (err) {
		ERRLOG("%s: error reading TP_REG_SYS_CTRL \n", __func__);
		err = -EIO;
		enable_irq(spi->irq);
		goto done;
	} else {
		if (read_buffer == 0x00) {
			/* start all-point mode */
			err = reg_write_byte(spi, TP_REG_SYS_CTRL, 0x20);
			if (err) {
				ERRLOG("%s: err ap mode \n", __func__);
				err = -EIO;
				enable_irq(spi->irq);
				goto done;
			}
		} else {
			ERRLOG("%s: err TP_REG_SYS_CTRL \n", __func__);
			err = -EIO;
			enable_irq(spi->irq);
			goto done;
		}
	}
	enable_irq(spi->irq);

done:
	return err;
}


static int do_calibration_valget(struct spi_device *spi,
			struct cy8ctma300_touch_ioctl_clbr *data)
{
	int err = -1;

	err = reg_read(spi, TP_REG_TMA_CTRL0, &data->prescaler, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL1, &data->subconversion, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL2, &data->shift, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL3, &data->noisethreshold, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_TMA_CTRL4, &data->fingerthreshold, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_NINT_TMO, &data->ninttimeout, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_INITMOVE_STEP, &data->initmovestep, 1);
	if (err)
		goto read_err;

	err = reg_read(spi, TP_REG_MOVE_STEP, &data->movestep, 1);
	if (err)
		goto read_err;

	DBGLOG("%s: prescaler=%d\n", __func__, data->prescaler);
	DBGLOG("%s: subconversion=%d\n", __func__,  data->subconversion);
	DBGLOG("%s: shift=%d\n", __func__, data->shift);
	DBGLOG("%s: noisethreshold=%d\n", __func__, data->noisethreshold);
	DBGLOG("%s: fingerthreshold=%d\n", __func__, data->fingerthreshold);
	DBGLOG("%s: nINT timeout=%d\n", __func__, data->ninttimeout);
	DBGLOG("%s: initmovestep=%d\n", __func__, data->initmovestep);
	DBGLOG("%s: movestep=%d\n", __func__, data->movestep);

	goto done;

read_err:
	err = -EIO;
done:
	return err;
}

static ssize_t cy8ctma300_touch_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct cy8ctma300_touch *tp =
	(struct cy8ctma300_touch *)file->private_data;
	struct spi_device *spi = tp->spi;
	struct cy8ctma300_touch_ioctl_clbr data;

	int err = 0;

	struct cy8ctma300_touch_reg_read_req reg_read_req;
	struct cy8ctma300_touch_reg_write_req reg_write_req;

	DBGLOG("%s: ioctl start init_complete=%d\n", __func__,
						tp->init_complete);

	switch (cmd) {
	case IOCTL_VALSET:
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, copy_from_user error\n", __func__);
			goto done;
		}

		err = do_calibration_valset(spi, &data);

		break;

	case IOCTL_VALGET:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, invalid access\n", __func__);
			goto done;
		}

		err = do_calibration_valget(spi, &data);

		if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, copy_to_user error\n", __func__);
			goto done;
		}
		break;

	case IOCTL_TOUCH_REG_READ:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&reg_read_req, (void __user *)arg,
			sizeof(struct cy8ctma300_touch_reg_read_req))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, copy_from_user error\n", __func__);
			goto done;
		}

		if (reg_read(spi, reg_read_req.reg, reg_read_req.buf,
						reg_read_req.len) != 0) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, reg_read error\n", __func__);
			goto done;
		}

		if (copy_to_user((void __user *)arg, (void *)&reg_read_req,
			sizeof(struct cy8ctma300_touch_reg_read_req))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, copy_to_user error\n", __func__);
			goto done;
		}
		break;

	case IOCTL_TOUCH_REG_WRITE:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, invalid access\n", __func__);
			goto done;
		}
		/*disable_irq(spi->irq);*/
		if (copy_from_user(&reg_write_req, (void __user *)arg,
			sizeof(struct cy8ctma300_touch_reg_write_req))) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, copy_from_user error\n", __func__);
			goto done;
		}

		/* use the register write function here */
		if (reg_write(spi, reg_write_req.reg, reg_write_req.buf,
						reg_write_req.len) != 0) {
			err = -EFAULT;
			ERRLOG("%s: ioctl, reg_write error\n", __func__);
			goto done;
		}
		/*enable_irq(spi->irq);*/
		break;

	default:
		ERRLOG("%s: ioctl, cmd error\n", __func__);
		return -EINVAL;
		break;
	}

done:
	return err;

};

static const struct file_operations cy8ctma300_touch_fops = {
	.owner = THIS_MODULE,
	.open = cy8ctma300_touch_open,
	.ioctl = cy8ctma300_touch_ioctl,
	.release = cy8ctma300_touch_release,
};

static int cy8ctma300_touch_probe(struct spi_device *spi)
{
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	struct cy8ctma300_touch *tp;
	int err = 0;
	struct input_dev *dev;
	dev_t device_t = MKDEV(0, 0);

	DBGLOG("%s: start \n", __func__);

	if (!pdata) {
		dev_err(&spi->dev, "%s: no platform data?\n", __func__);
		return -ENODEV;
	};

	if (!spi->irq) {
		dev_err(&spi->dev, "%s: no IRQ?\n", __func__);
		return -ENODEV;
	}

	/* Set up SPI */
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_1;

	DBGLOG("%s: SPI setup (%uHz) OK\n", __func__, spi->max_speed_hz);

	err = spi_setup(spi);
	if (err < 0)
		return err;

	/* GPIO: set up */
	DBGLOG("%s: Requesting GPIO Reset ownership\n", __func__);

	err = gpio_request(pdata->gpio_reset_pin, "cy8ctma300_touch_reset");
	if (err)
		goto err_gpio_setup;

	DBGLOG("%s: Requesting GPIO IRQ ownership\n", __func__);

	err = gpio_request(pdata->gpio_irq_pin, "cy8ctma300_touch_irq");
	if (err)
		goto err_gpio_setup;

	DBGLOG("%s: Configuring GPIO Reset direction\n", __func__);

	err = gpio_direction_output(pdata->gpio_reset_pin, 1);
	if (err)
		goto err_gpio_setup;

	DBGLOG("%s: Configuring GPIO IRQ direction\n", __func__);

	err = gpio_direction_input(pdata->gpio_irq_pin);
	if (err)
		goto err_gpio_setup;

	tp = kzalloc(sizeof(struct cy8ctma300_touch), GFP_KERNEL);
	if (!tp) {
		err = -ENOMEM;
		goto err_gpio_setup;
	}

	DBGLOG("%s: Allocated private data\n", __func__);

	tp->spi = spi;
	spin_lock_init(&tp->lock);
	dev_set_drvdata(&spi->dev, tp);

	DBGLOG("%s: Driver data set\n", __func__);

	/* initialize the work queue */
	INIT_WORK(&tp->setup_work, cy8ctma300_setup_work);
	INIT_WORK(&tp->mt_work, cy8ctma300_mt_work);

	/* workaround for firmware update causing strong vibration */
	tp->kmain_task = kthread_create(cy8ctma300_update_firmware,
				(void *)tp, "cy8ctma300_touch_update");
	if (IS_ERR(tp->kmain_task)) {
		err = -1;
		goto err_mem_free;
	}

	dev = input_allocate_device();
	if (!dev) {
		err = -ENOMEM;
		goto err_cleanup_mem;
	}
	DBGLOG("%s: Allocated input device\n", __func__);
		tp->input = dev;

	dev->name = "cy8ctma300_touch";
	dev->phys = "cy8ctma300_touch/input0";
	dev->dev.parent = &spi->dev;

	dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(dev, ABS_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(dev, ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(dev, ABS_PRESSURE, 0, 0, 0, 0);

	/* cypressTMA300E multitouch support */
	input_set_abs_params(dev, ABS_MT_POSITION_X, pdata->x_min,
				pdata->x_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, pdata->y_min,
				pdata->y_max, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, TP_TOUCH_MAJOR_MAX,
				 0, 0);

	err = input_register_device(dev);
	if (err)
		goto err_cleanup_mem;

	DBGLOG("%s: Registered input device\n", __func__);

	err = alloc_chrdev_region(&device_t, 0, 1, "cy8ctma300_touch");
	if (err)
		goto err_cleanup_input;

	DBGLOG("%s: Allocated character device\n", __func__);

	tp->device_major = MAJOR(device_t);

	cdev_init(&(tp->device_cdev), &cy8ctma300_touch_fops);
	tp->device_cdev.owner = THIS_MODULE;
	tp->device_cdev.ops = &cy8ctma300_touch_fops;

	err = cdev_add(&(tp->device_cdev), MKDEV(tp->device_major, 0), 1);
	if (err)
		goto err_cleanup_chrdev;

	DBGLOG("%s: Character device added\n", __func__);

	/* set flag as for first IRQ */
	tp->first_irq = 1;

#ifdef CONFIG_EARLYSUSPEND
	/* register early suspend */
	tp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tp->early_suspend.suspend = cy8ctma300_touch_early_suspend;
	tp->early_suspend.resume = cy8ctma300_touch_late_resume;
	register_early_suspend(&tp->early_suspend);
#endif

	/* workaround for irq-on effect to v1.10 and v1.07 */
	err = request_irq(tp->spi->irq, cy8ctma300_touch_irq,
			IRQF_TRIGGER_FALLING, tp->spi->dev.driver->name, tp);

	if (err) {
		ERRLOG("irq %d busy?\n", tp->spi->irq);
		goto err_cleanup_device;
	}

	DBGLOG("%s: Registered IRQ\n", __func__);

	tp->init_complete++;

	if (err)
		goto err_mem_free;
	else
		DBGLOG("%s: Device registered OK\n", __func__);

	return 0;

err_cleanup_device:
#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif
err_cleanup_chrdev:
	cdev_del(&(tp->device_cdev));
	unregister_chrdev_region(device_t, 1);
err_cleanup_input:
	input_unregister_device(dev);
err_cleanup_mem:
	if (dev)
		input_free_device(dev);
err_mem_free:
	kfree(tp);
err_gpio_setup:
	gpio_free(pdata->gpio_reset_pin);
	gpio_free(pdata->gpio_irq_pin);
	ERRLOG("%s: probe() fail: %d\n", __func__, err);

	return err;
};

static int __devexit cy8ctma300_touch_remove(struct spi_device *spi)
{
	struct cy8ctma300_touch *tp = dev_get_drvdata(&spi->dev);
	struct cypress_touch_platform_data *pdata = spi->dev.platform_data;
	dev_t device_t = MKDEV(tp->device_major, 0);

	DBGLOG("%s: unregistering touchscreen\n", __func__);

	if (!tp)
		return -ENODEV;

	if (!tp->init_complete) {
		ERRLOG("%s: can't unregister driver, FW\n", __func__);
		return -EBUSY;
	}

	gpio_free(pdata->gpio_reset_pin);
	gpio_free(pdata->gpio_irq_pin);

	cy8ctma300_touch_suspend(spi, PMSG_SUSPEND);

#ifdef CONFIG_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif

	if (tp->input)
		input_unregister_device(tp->input);
	if (&tp->device_cdev) {
		cdev_del(&(tp->device_cdev));
		unregister_chrdev_region(device_t, 1);
	};
	free_irq(tp->spi->irq, tp);
	kfree(tp);

	DBGLOG("%s: unregistered touchscreen\n", __func__);

	return 0;
};

static struct spi_driver cy8ctma300_touch_driver = {
	.driver = {
		.name = "cypress_touchscreen",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		},
	.probe = cy8ctma300_touch_probe,
	.remove = __devexit_p(cy8ctma300_touch_remove),
#ifndef CONFIG_EARLYSUSPEND
	.suspend = cy8ctma300_touch_suspend,
	.resume = cy8ctma300_touch_resume,
#endif
};

static int __init cy8ctma300_touch_init(void)
{
	int err;

	printk(KERN_DEBUG "%s: V%s built %s %s\n", __func__, MODULE_VER,
						__DATE__, __TIME__);
	err = spi_register_driver(&cy8ctma300_touch_driver);
	DBGLOG("%s: module init, result=%d\n", __func__, err);

	return err;
};

static void __exit cy8ctma300_touch_exit(void)
{
	spi_unregister_driver(&cy8ctma300_touch_driver);
	printk(KERN_DEBUG "%s: module exit\n", __func__);
};

module_init(cy8ctma300_touch_init);
module_exit(cy8ctma300_touch_exit);

MODULE_DESCRIPTION("Touchscreen driver for Cypress CY8CTMA300 hardware");
MODULE_LICENSE("GPL");
