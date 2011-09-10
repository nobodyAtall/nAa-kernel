/* linux/drivers/input/touchscreen/es209ra_touch_mt.c
 *
 * Copyright (C) 2009 Sony Ericsson Mobile Communications, INC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*#define DEBUG*/
/*#define CONFIG_FWUPDATE_IGNORE*/
#define CONFIG_FW_HEADER
#ifdef CONFIG_EARLYSUSPEND
#define CONFIG_ES209RA_TOUCH_EARLYSUSPEND
#endif

#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/es209ra_touch_mt.h>
#include <linux/earlysuspend.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <mach/gpio.h>
#include <asm/irq.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>

#ifdef CONFIG_FW_HEADER
#include "es209ra_touch_mt_fw.h"
#endif

#ifdef CONFIG_ARM
#include <asm/mach-types.h>
#endif

/*--------------------------------------------------------------------------*/

/* control & status register address*/
#define TP_REG_FWVER		0x00
#define TP_REG_CTRL		0x01
#define TP_REG_GCTRL		0x02
#define TP_REG_SLP1TO		0x03
#define TP_REG_SLP2TO		0x04
#define TP_REG_SLP1SR		0x05
#define TP_REG_SLP2SR		0x06
#define TP_REG_ACTSENS		0x07
#define TP_REG_SLP1SENS		0x08
#define TP_REG_SLP2SENS		0x09

/* gesture register address*/
#define TP_REG_CLICKT		0x0A
#define TP_REG_MCLICKT		0x0B
#define TP_REG_MCLICKS		0x0C
#define TP_REG_LTOUCHT		0x0D
#define TP_REG_ROTATES		0x0E
#define TP_REG_DRAGS		0x0F
#define TP_REG_FLICKS		0x10
#define TP_REG_ZOOMS		0x11

/* calibration register address*/
#define TP_REG_CRES		0x12
#define TP_REG_CSP		0x13
#define TP_REG_CPC		0x14
#define TP_REG_CIDACC		0x15
#define TP_REG_CIDACR		0x16
#define TP_REG_CNTHRS		0x17
#define TP_REG_CFTHRS		0x18
#define TP_REG_CNNTHR		0x19
#define TP_REG_CLBR		0x1A
#define TP_REG_CBLUTHR		0x1B
#define TP_REG_CUD		0x1C

/* touch data register address*/
#define TP_REG_XLSB		0x20
#define TP_REG_XYMSB		0x21
#define TP_REG_YLSB		0x22
#define TP_REG_Z		0x23
#define TP_REG_XLSB2		0x24
#define TP_REG_XYMSB2		0x25
#define TP_REG_YLSB2		0x26
#define TP_REG_Z2		0x27
#define TP_REG_GC		0x28
#define TP_REG_GPLSB		0x29
#define TP_REG_GPMSB		0x2A
#define TP_REG_MAXIVX		0x2B
#define TP_REG_MINIVX		0x2C
#define TP_REG_MAXIVY		0x2D
#define TP_REG_MINIVY		0x2E
#define TP_REG_TSTAT		0x2F

#define TSTAT_TCH12(S)  	(!!((S) & (1 << 0)))
#define TSTAT_AXISY(S)  	(!!((S) & (1 << 1)))
#define TSTAT_AXISX(S)  	(!!((S) & (1 << 2)))
#define TSTAT_1STTCHX(S)	(!!((S) & (1 << 3)))
#define TSTAT_1STTCHY(S)	(!!((S) & (1 << 4)))
#define TSTAT_1STTCHL(S)	(!!((S) & (1 << 5)))

/* calibration register address*/
#define TP_REG_BL			0xFF

#define FLASH_BLOCK_SIZE		(128)
#define FB_NO(x)			((x)-2)
#define FIRMWARE_PATH   		"/data/local/tmp/fw.hex"
#define FW_LENGTH			(32768)
#define PARAM_LENGTH    		(24)
#define MAX_FIRMWAREBLOCK       	(239)
#define CHECKBLOCK      		(255)
#define UPDATE_RETRY_NUMBER     	(1)
#define UPDATE_PERMISSION_VERSION	(0x35)
#define RESET_TIME			(3*HZ)

#define MAX_TP_SUPPORT		2
#define FIRST_TOUCH_POINT	0
#define SECOND_TOUCH_POINT	1
#define JUMP_PIXEL		40

/*Gesture Codes*/
#define GC_TOUCHUP		0x00
#define GC_CLICK		0x01
#define GC_FTOUCH1		0x02
#define GC_DRAG			0x04
#define GC_FTOUCH2		0x82
#define GC_DRAG2		0x84
#define GC_ZOOM2		0x87
#define GC_ERROR		0xFF

/*touch state*/
#define NO_TOUCH		0
#define ONE_TOUCH		1
#define TWO_TOUCH		2

#ifdef DEBUG
#define DBGLOG(format, args...) printk(KERN_DEBUG format, ## args)
#define ERRLOG(format, args...) printk(KERN_ERR format, ## args)
#else
#define DBGLOG(format, args...)
#define ERRLOG(format, args...)
#endif

/*--------------------------------------------------------------------------*/

enum drv_state{
	DRV_STATE_POWEROFF = 0,
	DRV_STATE_BEGIN_RESET,
	DRV_STATE_INACTIVE_APP,
	DRV_STATE_ACTIVE_BL,
	DRV_STATE_ACTIVE_APP,
	DRV_STATE_UNKNOWN,
};

struct touch_dataspace {
	u8 xlsb1;
	u8 xymsb1;
	u8 ylsb1;
	u8 z1;
	u8 xlsb2;
	u8 xymsb2;
	u8 ylsb2;
	u8 z2;
	u8 gc;
	u8 gplsb;
	u8 gpmsb;
	u8 maxivx;
	u8 minivx;
	u8 maxivy;
	u8 minivy;
	u8 tstat;
};

struct finger_data {
	u16 x;
	u16 y;
	u16 old_x;
	u16 old_y;
};

/* data structure used by this driver statically */
struct es209ra_touch {
	enum drv_state		state;
	struct input_dev	*input;
	struct spi_device	*spi;
	struct work_struct	isr_work;
	struct work_struct	tmowork;
#ifdef CONFIG_ES209RA_TOUCH_EARLYSUSPEND
	struct early_suspend	early_suspend;
#endif
	struct finger_data	finger_data[MAX_TP_SUPPORT];
	struct touch_dataspace	touch_data;
	struct cdev		device_cdev;
	int			device_major;
	struct class		*device_class;
	u8			fw_ver;
	u16			old_x;
	u16			old_y;
	u8			g_state_err;
	u8			g_state;
	u8			update_counter;
	struct timer_list	tickfn;
	struct mutex		touch_lock;
};
static void es209ra_setup_work(struct es209ra_touch *tp);
static void touch_data_handler(struct es209ra_touch *tp);
/*--------------------------------------------------------------------------*/

int es209ra_touch_reg_write(struct spi_device *spi, u8 addr,
					u8 *buf, size_t len)
{
	int err;
	u8 *write_buf;
	u8 dummy_read_req[] = {0x00, 0x00};
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);

	if (buf == NULL)
		return -EINVAL;

	/* Dummy SPI write due to issue on the device */
	err = spi_write(spi, dummy_read_req, 2);
	if (err)
		return err;

	udelay(180);

	write_buf = kzalloc(sizeof(u8)*(len+2), GFP_KERNEL);
	if (write_buf == NULL)
		return -ENOMEM;

	write_buf[0] = 0x01;
	write_buf[1] = addr;

	memcpy(&write_buf[2], buf, len);

	err = spi_write(spi, write_buf, len+2);

	kfree(write_buf);

	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		mdelay(2);

	return 0;
}

static int reg_write_byte(struct spi_device *spi, u8 addr, u8 data)
{
	int err;

	err = es209ra_touch_reg_write(spi, addr, &data, 1);
	if (err)
		return err;

	return 0;
}

int es209ra_touch_reg_read(struct spi_device *spi, u8 addr, u8 *buf, size_t len)
{
	int err;
	u8 read_req[2];
	u8 dummy_read_req[] = {0x00, 0x00};
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);

	if (buf == NULL || len == 0)
		return -EINVAL;

	read_req[0] = 0x00;
	read_req[1] = addr;

	/* Dummy SPI write due to issue on the device */
	err = spi_write(spi, dummy_read_req, 2);
	if (err)
		return err;

	udelay(180);

	err = spi_write(spi, read_req, 2);
	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		mdelay(1);

	if (len == 1) {
		u8 read_buf[2];

		/* To keep always more than 2 byte read access
		 * The device does't allow 1 byte access */
		err = spi_read(spi, read_buf, len+1);
		*buf = read_buf[0];
	} else {
		err = spi_read(spi, buf, len);
	}

	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		mdelay(2);

	return 0;
}

static int no_dummy_reg_write(struct spi_device *spi, u8 addr,
					u8 *buf, size_t len)
{
	int err;
	u8 *write_buf;
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);

	if (buf == NULL)
		return -EINVAL;

	write_buf = kzalloc(sizeof(u8)*(len+2), GFP_KERNEL);
	if (write_buf == NULL)
		return -ENOMEM;

	write_buf[0] = 0x01;
	write_buf[1] = addr;

	memcpy(&write_buf[2], buf, len);

	err = spi_write(spi, write_buf, len+2);

	kfree(write_buf);

	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		mdelay(2);

	return 0;
}

static int no_dummy_reg_read(struct spi_device *spi, u8 addr, u8* buf)
{
	int err;
	u8 read_req[2];
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	u8 read_buf[2]  = {0x00, 0xFF};
	u8 write_buf[2] = {0x00, 0xFF};

	if (buf == NULL)
		return -EINVAL;

	read_req[0] = 0x00;
	read_req[1] = addr;

	err = spi_write(spi, read_req, 2);
	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		mdelay(1);

	udelay(180);

	{
		/* To keep always more than 2 byte read access
		 * The device does't allow 1 byte access */
		struct spi_transfer	t = {
			.rx_buf		= read_buf,
			.tx_buf		= write_buf,
			.len		= 2,
		};
		struct spi_message	m;

		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		err = spi_sync(spi, &m);
		*buf = read_buf[0];
	}
	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		mdelay(2);

	return 0;
}

static int spi_sync_read(struct spi_device *spi, u8 addr, u8 *buf, size_t len)
{
	int err;
	u8 read_req[2];
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);

	read_req[0] = 0x00;
	read_req[1] = addr;

	err = spi_write(spi, read_req, 2);
	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		msleep(1);

	udelay(180);

	err = spi_read(spi, buf, len);
	if (err)
		return err;

	if (tp->fw_ver < 0x10)
		msleep(2);

	return 0;
}


static void reset_device(struct es209ra_touch_platform_data *pdata)
{
	gpio_set_value(pdata->gpio_reset_pin, 1);

	udelay(1000);

	gpio_set_value(pdata->gpio_reset_pin, 0);

	udelay(1000);
}
static void dump_calibration_parameters(struct es209ra_touch_ioctl_clbr *data)
{
	DBGLOG("%s: resolution=%d\n", __func__, data->resolution);
	DBGLOG("%s: scanspeed=%d\n", __func__, data->scanspeed);
	DBGLOG("%s: prescaler=%d\n", __func__, data->prescaler);
	DBGLOG("%s: columnidac=%d\n", __func__, data->columnidac);
	DBGLOG("%s: rowidac=%d\n", __func__, data->rowidac);
	DBGLOG("%s: noisethreshold=%d\n", __func__, data->noisethreshold);
	DBGLOG("%s: fingerthreshold=%d\n", __func__, data->fingerthreshold);
	DBGLOG("%s: negnoisethreshold=%d\n", __func__, data->negnoisethreshold);
	DBGLOG("%s: lowbaselinereset=%d\n", __func__, data->lowbaselinereset);
	DBGLOG("%s: blupdatethreshold=%d\n", __func__, data->blupdatethreshold);
}

static int do_calibration_ioctl_valset(struct spi_device *spi,
			  struct es209ra_touch_ioctl_clbr *data)
{
	struct es209ra_touch_platform_data *pdata = spi->dev.platform_data;
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	int err = 0;
	u8 read_buffer;

	mutex_lock(&tp->touch_lock);
	if (tp->fw_ver >= 0x7)
		disable_irq(spi->irq);

	dump_calibration_parameters(data);

	switch (data->clbr_num) {
	case 0:
		reg_write_byte(spi, TP_REG_CRES, data->resolution);
		break;
	case 1:
		reg_write_byte(spi, TP_REG_CSP, data->scanspeed);
		break;
	case 2:
		reg_write_byte(spi, TP_REG_CPC, data->prescaler);
		break;
	case 3:
		reg_write_byte(spi, TP_REG_CNTHRS, data->noisethreshold);
		break;
	case 4:
		reg_write_byte(spi, TP_REG_CFTHRS, data->fingerthreshold);
		break;
	case 5:
		reg_write_byte(spi, TP_REG_CNNTHR, data->negnoisethreshold);
		break;
	case 6:
		reg_write_byte(spi, TP_REG_CLBR, data->lowbaselinereset);
		break;
	case 7:
		reg_write_byte(spi, TP_REG_CBLUTHR, data->blupdatethreshold);
		break;
	case 8:
		reg_write_byte(spi, TP_REG_CRES, data->resolution);
		reg_write_byte(spi, TP_REG_CSP, data->scanspeed);
		reg_write_byte(spi, TP_REG_CPC, data->prescaler);
		reg_write_byte(spi, TP_REG_CNTHRS, data->noisethreshold);
		reg_write_byte(spi, TP_REG_CFTHRS, data->fingerthreshold);
		reg_write_byte(spi, TP_REG_CNNTHR, data->negnoisethreshold);
		reg_write_byte(spi, TP_REG_CLBR, data->lowbaselinereset);
		reg_write_byte(spi, TP_REG_CBLUTHR, data->blupdatethreshold);
		break;
	default:
		ERRLOG("%s: reg num error = %d\n", __func__, data->clbr_num);
		if (tp->fw_ver >= 0x7)
			enable_irq(spi->irq);

		mutex_unlock(&tp->touch_lock);
		return -EINVAL;
	}

	reg_write_byte(spi, TP_REG_CUD, 0x01);

	if (tp->fw_ver >= 0x7) {
		msleep(200);

		err = es209ra_touch_reg_read(spi, TP_REG_CUD, &read_buffer, 1);
		if (err) {
			err = -EIO;
		} else {
			/* reset release device */
			DBGLOG("%s: state=DRV_STATE_BEGIN_RESET\n", __func__);
			tp->state = DRV_STATE_BEGIN_RESET;
			DBGLOG("%s: touch_reset_device\n", __func__);
			reset_device(pdata);
		}
		enable_irq(spi->irq);
	} else {
		msleep(250);
	}

	mutex_unlock(&tp->touch_lock);
	return err;
}

static int do_calibration_ioctl_valget(struct spi_device *spi,
			  struct es209ra_touch_ioctl_clbr *data)
{
	es209ra_touch_reg_read(spi, TP_REG_CRES, &data->resolution, 1);
	es209ra_touch_reg_read(spi, TP_REG_CSP, &data->scanspeed, 1);
	es209ra_touch_reg_read(spi, TP_REG_CPC, &data->prescaler, 1);
	es209ra_touch_reg_read(spi, TP_REG_CIDACC, &data->columnidac, 1);
	es209ra_touch_reg_read(spi, TP_REG_CIDACR, &data->rowidac, 1);
	es209ra_touch_reg_read(spi, TP_REG_CNTHRS, &data->noisethreshold, 1);
	es209ra_touch_reg_read(spi, TP_REG_CFTHRS, &data->fingerthreshold, 1);
	es209ra_touch_reg_read(spi, TP_REG_CNNTHR,
						&data->negnoisethreshold, 1);
	es209ra_touch_reg_read(spi, TP_REG_CLBR, &data->lowbaselinereset, 1);
	es209ra_touch_reg_read(spi, TP_REG_CBLUTHR,
						&data->blupdatethreshold, 1);

	dump_calibration_parameters(data);

	return 0;
}

static int do_calibration(struct spi_device *spi,
			  struct es209ra_touch_ioctl_clbr *data, int cmd)
{
	int err;

	switch (cmd) {
	case 0x00:
		err = do_calibration_ioctl_valset(spi, data);
		break;
	case 0x01:
		err = do_calibration_ioctl_valget(spi, data);
		break;
	default:
		err = -EINVAL;
		ERRLOG("%s: cmd error\n", __func__);
		break;
	}

	return err;
}

#if !defined(CONFIG_FW_HEADER)
static ssize_t get_fw_data(struct es209ra_touch *tp, char *fw_buf,
					size_t count, int islast)
{
	ssize_t length = -1;
	struct file *filp;
	struct inode *inode = NULL;
	loff_t pos = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(FIRMWARE_PATH, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		ERRLOG("%s: File %p filp_open error\n", __func__, filp);
		goto err_r_fs;
	}
	if (!filp->f_op) {
		ERRLOG("%s: File Operation Method Error\n", __func__);
		goto err_close_file;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) {
		ERRLOG("%s: Get inode from filp failed\n", __func__);
		goto err_close_file;
	}

	length = filp->f_path.dentry->d_inode->i_size;
	if (length < count) {
		ERRLOG("%s: Invalid firmware length %d\n", __func__, length);
		goto err_close_file;
	}

	DBGLOG("%s: file length=%d\n", __func__, length);

	if (islast == 1)
		pos = length - 1;

	length = vfs_read(filp, fw_buf, count, &pos);

err_close_file:
	filp_close(filp, NULL);
err_r_fs:
	set_fs(old_fs);

	return length;
}
#endif

static int check_state(struct es209ra_touch *tp, int exp_state)
{
	int i;

	/* value 25 = 500(power on time)/20(mdelay) */
	for (i = 0; i < 25; i++) {
		if (tp->state >= exp_state)
			return 0;
		else
			msleep(20);
	}

	return -1;
}

static u8 get_latest_firmware_ver(struct es209ra_touch *tp)
{
	ssize_t length;
	char fw_buf;

	DBGLOG("%s: start\n", __func__);

#if !defined(CONFIG_FW_HEADER)
	length = get_fw_data(tp, &fw_buf, 1, 1);
	if (length != 1) {
		ERRLOG("%s: Failed to read %d\n", __func__, length);
		fw_buf = 0;
	}
#else
	length = 0;
	fw_buf = firmware[FW_LENGTH];
#endif

	DBGLOG("%s: fw version = %d.%d\n", __func__, fw_buf>>4, fw_buf&0x0F);
	return fw_buf;
}

static int update_firmware(struct es209ra_touch *tp)
{
	struct spi_device *spi = tp->spi;
	struct es209ra_touch_platform_data *pdata = spi->dev.platform_data;
	const u8 BootloaderKeys[8] = {0x0A, 0x06, 0x0D, 0x06,
					0x00, 0x01, 0x0D, 0x03};
	u8 BootloaderEnter[9] = {0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x38};
	u8 BootloaderExit[11] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x3B, 0xCA, 0x53};
	u8 WriteFlashBlock[143];
	u8 WriteFlashBlock_FBNO142_Init, read_buf;
	ssize_t length;
	int err = 0;
	int i;
	int j;
	char *fw_buf;

	DBGLOG("%s: start\n", __func__);

	memcpy(BootloaderEnter, BootloaderKeys, ARRAY_SIZE(BootloaderKeys));
	err = es209ra_touch_reg_write(tp->spi, TP_REG_BL, BootloaderEnter,
						ARRAY_SIZE(BootloaderEnter));
	if (err) {
		ERRLOG("%s: write enter error\n", __func__);
		goto err_r_fs_irq;
	}

	msleep(200);

	err = no_dummy_reg_read(tp->spi, TP_REG_BL, &read_buf);
	if (err) {
		ERRLOG("%s: write enter read state error\n", __func__);
		goto err_r_fs_irq;
	}
	if (read_buf != 0x3) {
		ERRLOG("%s: write enter state error %d\n", __func__, read_buf);
		err = -1;
		goto err_r_fs_irq;
	}
	DBGLOG("%s: BootloaderEnter OK readbuf=%x\n", __func__, read_buf);

#if !defined(CONFIG_FW_HEADER)
	fw_buf = kmalloc(FW_LENGTH, GFP_KERNEL);
	if (fw_buf == NULL) {
		ERRLOG("%s: kernel memory alloc error\n", __func__);
		err = -1;
		goto err_r_fs_irq;
	}
	length = get_fw_data(tp, fw_buf, FW_LENGTH, 0);
	if ((length != FW_LENGTH) {
		ERRLOG("%s: Failed to read %d\n", __func__, length);
		err = -1;
		goto err_free_data;
	}
#else
	length = 0;
	fw_buf = (char *)firmware;
#endif

	memcpy(WriteFlashBlock, BootloaderKeys, ARRAY_SIZE(BootloaderKeys));

	WriteFlashBlock[FB_NO(10)] = 0x39;
	WriteFlashBlock[FB_NO(11)] = 0x00;
	WriteFlashBlock[FB_NO(143)] = 0xCA;
	WriteFlashBlock[FB_NO(144)] = 0x53;

	WriteFlashBlock_FBNO142_Init = TP_REG_BL;

	for (i = 2; i < 12; i++)
		WriteFlashBlock_FBNO142_Init += WriteFlashBlock[FB_NO(i)];

	i = 2;
	/* Data Initialize */
	while (1) {
		/* Data Check Sum */
		WriteFlashBlock[FB_NO(141)] = 0;
		/* Command Check Sum FB_NO(1-11) */
		WriteFlashBlock[FB_NO(142)] = WriteFlashBlock_FBNO142_Init;

		WriteFlashBlock[FB_NO(12)] = i;
		memcpy(&WriteFlashBlock[FB_NO(13)],
			&fw_buf[FLASH_BLOCK_SIZE*i], (int)FLASH_BLOCK_SIZE);

		 /* Data Check Sum FB_NO(13-140) */
		for (j = 13; j < 141; j++)
			WriteFlashBlock[FB_NO(141)] +=
			  WriteFlashBlock[FB_NO(j)];

		/* Command Check Sum add FB_NO(12-141) */
		WriteFlashBlock[FB_NO(142)] += (WriteFlashBlock[FB_NO(141)]*2 +
						WriteFlashBlock[FB_NO(12)]);
		/* Download New Application Firmware */
		err = no_dummy_reg_write(tp->spi, TP_REG_BL, WriteFlashBlock,
						ARRAY_SIZE(WriteFlashBlock));
		if (err) {
			ERRLOG("%s: write firmware error\n", __func__);
			goto err_free_data;
		}

		msleep(100);

		err = no_dummy_reg_read(tp->spi, TP_REG_BL, &read_buf);
		if (err) {
			ERRLOG("%s: read state error %d\n", __func__, i);
			goto err_free_data;
		}

		if (read_buf != 0x3) {
			ERRLOG("%s: write firmware state error %d,%d\n",
						__func__, read_buf, i);
			err = -1;
			goto err_free_data;
		}

		i++; /* next flash block */

		if (i > CHECKBLOCK)
			break;

		if (i > MAX_FIRMWAREBLOCK)
			i = CHECKBLOCK;
	}

	memcpy(BootloaderExit, BootloaderKeys, ARRAY_SIZE(BootloaderKeys));
	/* Device will be automatically reset after this command */
	err = no_dummy_reg_write(tp->spi, TP_REG_BL, BootloaderExit,
						ARRAY_SIZE(BootloaderExit));
	if (err) {
		ERRLOG("%s: write exit error\n", __func__);
		goto err_free_data;
	}
	DBGLOG("%s: BootloaderExit\n", __func__);

	DBGLOG("%s: state = DRV_STATE_POWEROFF\n", __func__);
	tp->state = DRV_STATE_POWEROFF;

err_free_data:
#if !defined(CONFIG_FW_HEADER)
	kfree(fw_buf);
#endif
err_r_fs_irq:
	if (tp->state != DRV_STATE_POWEROFF) {
		/* reset release device */
		DBGLOG("%s: state = DRV_STATE_BEGIN_RESET\n", __func__);
		tp->state = DRV_STATE_BEGIN_RESET;
		DBGLOG("%s: touch_reset_device\n", __func__);
		reset_device(pdata);
	}

	DBGLOG("%s: Return=%d\n", __func__, err);
	return err;
}

static void es209ra_touch_tmowork(struct work_struct *work)
{
	struct es209ra_touch *tp =
		container_of(work, struct es209ra_touch, tmowork);
	struct spi_device *spi = tp->spi;
	struct es209ra_touch_platform_data *pdata = spi->dev.platform_data;

	mutex_lock(&tp->touch_lock);

	if (tp->g_state != NO_TOUCH) {
		tp->touch_data.gc = GC_TOUCHUP;
		touch_data_handler(tp);
	}

	DBGLOG("%s: state = DRV_STATE_BEGIN_RESET\n", __func__);
	tp->state = DRV_STATE_BEGIN_RESET;
	DBGLOG("%s: touch_reset_device\n", __func__);
	reset_device(pdata);

	mutex_unlock(&tp->touch_lock);
}

static void touch_timeout(unsigned long arg)
{
	struct es209ra_touch *tp = (struct es209ra_touch *)arg;

	DBGLOG("%s: start\n", __func__);
	schedule_work(&tp->tmowork);
}

static void touch_data_handler(struct es209ra_touch *tp)
{
	struct finger_data *tp1 = &tp->finger_data[FIRST_TOUCH_POINT];
	struct finger_data *tp2 = &tp->finger_data[SECOND_TOUCH_POINT];
	struct finger_data *down;
	struct finger_data *temp;

	tp1->x = tp->touch_data.xlsb1 | ((tp->touch_data.xymsb1 & 0x0F) << 8);
	tp1->y = tp->touch_data.ylsb1 | ((tp->touch_data.xymsb1 & 0xF0) << 4);
	tp2->x = tp->touch_data.xlsb2 | ((tp->touch_data.xymsb2 & 0x0F) << 8);
	tp2->y = tp->touch_data.ylsb2 | ((tp->touch_data.xymsb2 & 0xF0) << 4);

	down = (tp1->x != 0xFFF) ? tp1 : tp2;

	DBGLOG("%s: [%02x] xy1=(%d %d)"
		"xy2=(%d %d) TCH12=%d AXISxy=(%d %d)"
		"1stTchXY=(%d %d) 1stTchL=%d %s\n",
		__func__, tp->touch_data.gc,
		tp1->x, tp1->y, tp2->x, tp2->y,
		TSTAT_TCH12(tp->touch_data.tstat),
		TSTAT_AXISX(tp->touch_data.tstat),
		TSTAT_AXISY(tp->touch_data.tstat),
		TSTAT_1STTCHX(tp->touch_data.tstat),
		TSTAT_1STTCHY(tp->touch_data.tstat),
		TSTAT_1STTCHL(tp->touch_data.tstat),
		((tp->touch_data.tstat & 0x7) != 0x7) ? "*** error" : "");

	switch (tp->touch_data.gc) {
	case GC_TOUCHUP:
	case GC_CLICK:
		if (timer_pending(&tp->tickfn)) {
			del_timer(&tp->tickfn);
			tp->g_state_err = 0;
		}
		if (tp->g_state == ONE_TOUCH) {
			input_report_abs(tp->input, ABS_MT_POSITION_X,
							down->old_x);
			input_report_abs(tp->input, ABS_MT_POSITION_Y,
							down->old_y);
			input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(tp->input);
			input_sync(tp->input);
			DBGLOG("%s: ST -> touch up: (%d, %d)",
				__func__, down->old_x, down->old_y);
		} else if (tp->g_state == TWO_TOUCH) {
			input_report_abs(tp->input, ABS_MT_POSITION_X,
							tp1->old_x);
			input_report_abs(tp->input, ABS_MT_POSITION_Y,
							tp1->old_y);
			input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(tp->input);

			input_report_abs(tp->input, ABS_MT_POSITION_X,
							tp2->old_x);
			input_report_abs(tp->input, ABS_MT_POSITION_Y,
							tp2->old_y);
			input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 0);
			input_mt_sync(tp->input);
			input_sync(tp->input);
			DBGLOG("%s: MT -> tp1 touch up: (%d, %d)"
				" tp2 touch up: (%d, %d) \n",
				__func__,
				tp1->old_x, tp1->old_y,
				tp2->old_x, tp2->old_y);
			tp1->old_x = 0xFFF;
			tp1->old_y = 0xFFF;
			tp2->old_x = 0xFFF;
			tp2->old_y = 0xFFF;
		}
		tp->g_state = NO_TOUCH;
		break;
	case GC_FTOUCH1:
	case GC_DRAG:
		if (timer_pending(&tp->tickfn)) {
			del_timer(&tp->tickfn);
			tp->g_state_err = 0;
		}
		/*send ST report*/
		input_report_abs(tp->input, ABS_MT_POSITION_X, down->x);
		input_report_abs(tp->input, ABS_MT_POSITION_Y, down->y);
		input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 10);
		input_mt_sync(tp->input);
		input_sync(tp->input);
		DBGLOG("%s: ST -> tp1 touch down: %d, %d\n",
				__func__, down->x, down->y);
		down->old_x = down->x;
		down->old_y = down->y;
		tp->g_state = ONE_TOUCH;
		break;
	case GC_FTOUCH2:
		if (timer_pending(&tp->tickfn)) {
			del_timer(&tp->tickfn);
			tp->g_state_err = 0;
		}
		/* swap order of sending fingers when 1st finger
		   goes down->up->down */
		if ((tp->g_state == ONE_TOUCH) &&
		    (tp1->x != down->old_x && tp1->y != down->old_y)) {
			temp = tp1;
			tp1 = tp2;
			tp2 = temp;
		}
		/*report 2 successive finger*/
		input_report_abs(tp->input, ABS_MT_POSITION_X, tp1->x);
		input_report_abs(tp->input, ABS_MT_POSITION_Y, tp1->y);
		input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 10);
		input_mt_sync(tp->input);
		tp1->old_x = tp1->x;
		tp1->old_y = tp1->y;

		input_report_abs(tp->input, ABS_MT_POSITION_X, tp2->x);
		input_report_abs(tp->input, ABS_MT_POSITION_Y, tp2->y);
		input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 10);
		input_mt_sync(tp->input);
		input_sync(tp->input);
		DBGLOG("%s: MT -> tp1 touch down: (%d, %d),"
			" tp2 touch down: (%d, %d) \n",
				__func__,
				tp1->x, tp1->y,
				tp2->x, tp2->y);
		tp2->old_x = tp2->x;
		tp2->old_y = tp2->y;
		tp->g_state = TWO_TOUCH;
		break;
	case GC_DRAG2:
        case GC_ZOOM2:
		if (timer_pending(&tp->tickfn)) {
			del_timer(&tp->tickfn);
			tp->g_state_err = 0;
		}
		/*reduce jump*/
		if (abs(tp1->x - tp1->old_x) > JUMP_PIXEL) {
			tp1->x = tp1->old_x + ((tp1->x - tp1->old_x) * 1/10);
			DBGLOG("%s: tp1->x = %d jumped!\n", __func__, tp1->x);
		}
		if (abs(tp1->y - tp1->old_y) > JUMP_PIXEL) {
			tp1->y = tp1->old_y + ((tp1->y - tp1->old_y) * 1/10);
			DBGLOG("%s: tp1->y = %d jumped!\n", __func__, tp1->y);
		}
		if (abs(tp2->x - tp2->old_x) > JUMP_PIXEL) {
			tp2->x = tp2->old_x + ((tp2->x - tp2->old_x) * 1/10);
			DBGLOG("%s: tp2->x = %d jumped!\n", __func__, tp2->x);
		}
		if (abs(tp2->y - tp2->old_y) > JUMP_PIXEL) {
			tp2->y = tp2->old_y + ((tp2->y - tp2->old_y) * 1/10);
			DBGLOG("%s: tp2->y = %d jumped!\n", __func__, tp2->y);
		}
		/*send MT report*/
		input_report_abs(tp->input, ABS_MT_POSITION_X, tp1->x);
		input_report_abs(tp->input, ABS_MT_POSITION_Y, tp1->y);
		input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 10);
		input_mt_sync(tp->input);
		tp1->old_x = tp1->x;
		tp1->old_y = tp1->y;

		input_report_abs(tp->input, ABS_MT_POSITION_X, tp2->x);
		input_report_abs(tp->input, ABS_MT_POSITION_Y, tp2->y);
		input_report_abs(tp->input, ABS_MT_TOUCH_MAJOR, 10);
		input_mt_sync(tp->input);
		input_sync(tp->input);
		DBGLOG("%s: MT -> tp1 touch down: (%d, %d),"
			" tp2 touch down: (%d, %d) \n",
			__func__, tp1->x, tp1->y, tp2->x, tp2->y);
		tp2->old_x = tp2->x;
		tp2->old_y = tp2->y;
		break;
	case GC_ERROR:
		ERRLOG("%s: touch err\n", __func__);
		if (timer_pending(&tp->tickfn) == 0) {
			DBGLOG("%s: timeout set!\n", __func__);
			tp->tickfn.expires = jiffies + RESET_TIME;
			add_timer(&tp->tickfn);
		}
		tp->g_state_err = 1;
		break;
	default:
		DBGLOG("%s: -> Unknown gesture reg code!\n", __func__);
		break;
	}

}
static void es209ra_touch_isr(struct work_struct *work)
{
	struct es209ra_touch *tp =
		container_of(work, struct es209ra_touch, isr_work);
	struct spi_device *spi = tp->spi;
	int err;

	mutex_lock(&tp->touch_lock);

	err = spi_sync_read(spi, TP_REG_XLSB, (u8 *)&tp->touch_data,
					sizeof(tp->touch_data));

	if (err)
		ERRLOG("%s: spi_sync_read failed\n", __func__);

	if (tp->state == DRV_STATE_ACTIVE_APP) {
		touch_data_handler(tp);
	} else if ((tp->state == DRV_STATE_POWEROFF) ||
		(tp->state == DRV_STATE_BEGIN_RESET)) {
		DBGLOG("%s: setup_work()\n", __func__);
		es209ra_setup_work(tp);
	} else {
		ERRLOG("%s: invalid state\n", __func__);
	}

	mutex_unlock(&tp->touch_lock);
}

static irqreturn_t es209ra_touch_irq(int irq, void *handle)
{
	struct es209ra_touch *tp = handle;

	schedule_work(&tp->isr_work);

	return IRQ_HANDLED;
}

static void es209ra_setup_work(struct es209ra_touch *tp)
{
	u8 read_buf = 0xFF;

	DBGLOG("%s: update_counter=%d\n", __func__, tp->update_counter);
	if (tp->g_state_err == 1) {
		/* Reset Error */
		DBGLOG("%s: timeout setup!\n", __func__);
		tp->g_state_err = 0;
		DBGLOG("%s: state = DRV_STATE_ACTIVE_APP\n", __func__);
		tp->state = DRV_STATE_ACTIVE_APP;

		reg_write_byte(tp->spi, TP_REG_CTRL, 0x01);
		reg_write_byte(tp->spi, TP_REG_GCTRL, 0x64);
		reg_write_byte(tp->spi, TP_REG_SLP1TO, 0x96);
		reg_write_byte(tp->spi, TP_REG_SLP2TO, 0x96);
		reg_write_byte(tp->spi, TP_REG_SLP1SR, 0x0C);
		reg_write_byte(tp->spi, TP_REG_SLP2SR, 0x18);
		reg_write_byte(tp->spi, TP_REG_ACTSENS, 0x11);
		reg_write_byte(tp->spi, TP_REG_SLP1SENS, 0x22);
		reg_write_byte(tp->spi, TP_REG_SLP2SENS, 0x33);
		reg_write_byte(tp->spi, TP_REG_DRAGS, 0x03);
		return;
	}

	if ((tp->state != DRV_STATE_POWEROFF))
		es209ra_touch_reg_read(tp->spi, TP_REG_BL, &read_buf, 1);
	DBGLOG("%s: es209ra_touch_setup_%x\n", __func__, read_buf);

	/* check whether application firmware or bootloader is running */
	if (read_buf == 0x0) {
		/* app mode */
		es209ra_touch_reg_read(tp->spi, TP_REG_FWVER,
						&tp->fw_ver, 1);
		DBGLOG("%s: fw version = %d.%d\n", __func__,
			tp->fw_ver>>4, tp->fw_ver&0x0F);

		if ((tp->fw_ver < 0x06) || (tp->fw_ver > 0x99)) {
			ERRLOG("%s: unknown ver = %d\n", __func__, tp->fw_ver);
			return;
		}

		DBGLOG("%s: state = DRV_STATE_ACTIVE_APP\n", __func__);
		tp->state = DRV_STATE_ACTIVE_APP;

#if !defined(CONFIG_FWUPDATE_IGNORE)
		if ((tp->update_counter < UPDATE_RETRY_NUMBER)
		   && (tp->fw_ver >= UPDATE_PERMISSION_VERSION)
		   && (tp->fw_ver != get_latest_firmware_ver(tp))) {
			DBGLOG("%s: firmware update is required\n", __func__);
			tp->update_counter++;
			update_firmware(tp);
		} else {
#endif
			/* initialze the device */
			reg_write_byte(tp->spi, TP_REG_CTRL, 0x01);
			reg_write_byte(tp->spi, TP_REG_GCTRL, 0x64);
			reg_write_byte(tp->spi, TP_REG_SLP1TO, 0x96);
			reg_write_byte(tp->spi, TP_REG_SLP2TO, 0x96);
			reg_write_byte(tp->spi, TP_REG_SLP1SR, 0x0C);
			reg_write_byte(tp->spi, TP_REG_SLP2SR, 0x18);
			reg_write_byte(tp->spi, TP_REG_ACTSENS, 0x11);
			reg_write_byte(tp->spi, TP_REG_SLP1SENS, 0x22);
			reg_write_byte(tp->spi, TP_REG_SLP2SENS, 0x33);
			reg_write_byte(tp->spi, TP_REG_DRAGS, 0x03);
#if !defined(CONFIG_FWUPDATE_IGNORE)
		}
#endif
	} else if (read_buf == 0x1) {
		/* boot mode */
		DBGLOG("%s: state = DRV_STATE_ACTIVE_BL\n", __func__);
		tp->state = DRV_STATE_ACTIVE_BL;
		if ((tp->update_counter >= UPDATE_RETRY_NUMBER)) {
			tp->update_counter = 0;
			ERRLOG("%s: firmware update failed and finished\n",
								__func__);
#if !defined(CONFIG_FWUPDATE_IGNORE)
		} else {
			DBGLOG("%s: FW update failed and required. Count=%d\n",
						__func__, tp->update_counter);
			tp->update_counter++;
			update_firmware(tp);
		}
#endif
	} else {
		DBGLOG("%s: state = DRV_STATE_BEGIN_RESET\n", __func__);
		tp->state = DRV_STATE_BEGIN_RESET;
		ERRLOG("%s: mode error = %d\n", __func__, read_buf);
	}
}

/*--------------------------------------------------------------------------*/

static int es209ra_touch_suspend(struct spi_device *spi, pm_message_t message)
{
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	int err;
	u8 data;

	disable_irq(tp->spi->irq);

	/* clear worker and timer */
	if (work_pending(&tp->isr_work))
		flush_work(&tp->isr_work);

	if (work_pending(&tp->tmowork))
		flush_work(&tp->tmowork);

	if (timer_pending(&tp->tickfn)) {
		del_timer(&tp->tickfn);
		tp->g_state_err = 0;
	}

	err = es209ra_touch_reg_read(spi, TP_REG_CTRL, &data, 1);
	if (err)
		return err;

	err = reg_write_byte(spi, TP_REG_CTRL, data&0xFE);
	if (err)
		return err;

	return 0;
}

static int es209ra_touch_resume(struct spi_device *spi)
{
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	struct es209ra_touch_platform_data *pdata = spi->dev.platform_data;

	enable_irq(tp->spi->irq);

	DBGLOG("%s: state = DRV_STATE_BEGIN_RESET\n", __func__);
	tp->state = DRV_STATE_BEGIN_RESET;
	DBGLOG("%s: state = touch_reset_device\n", __func__);
	reset_device(pdata);

	return 0;
}

#ifdef CONFIG_ES209RA_TOUCH_EARLYSUSPEND
static void es209ra_touch_early_suspend(struct early_suspend *es)
{
	struct es209ra_touch *tp;
	tp = container_of(es, struct es209ra_touch, early_suspend);

	DBGLOG("%s: start\n", __func__);

	es209ra_touch_suspend(tp->spi, PMSG_SUSPEND);
}

static void es209ra_touch_late_resume(struct early_suspend *es)
{
	struct es209ra_touch *tp;
	tp = container_of(es, struct es209ra_touch, early_suspend);

	DBGLOG("%s: start\n", __func__);

	es209ra_touch_resume(tp->spi);
}
#endif

static int es209ra_touch_open(struct inode *inode, struct file *file)
{
	struct es209ra_touch *tp =
		container_of(inode->i_cdev, struct es209ra_touch, device_cdev);

	file->private_data = tp;
	return 0;
}

static int es209ra_touch_release(struct inode *inode, struct file *file)
{
	return 0;
}


static int parse_and_calibrate(struct es209ra_touch *tp,
	struct spi_device *spi, const char __user *buf, size_t count)
{
	int err = -1;
	int i;
	int j;
	u8 read_buffer_in[PARAM_LENGTH/2] = {0};
	u8 tmp_read_buffer[PARAM_LENGTH] = {0};

	err = check_state(tp, DRV_STATE_ACTIVE_BL);
	if (err == -1) {
		ERRLOG("%s: invalid state\n", __func__);
		return err;
	}
	if (!access_ok(VERIFY_READ, (void __user *)buf, PARAM_LENGTH)) {
		err = -EFAULT;
		ERRLOG("%s: invalid access\n", __func__);
		goto done;
	}

	if (copy_from_user(tmp_read_buffer, buf, (sizeof(u8)*PARAM_LENGTH))) {
		err = -1;
		ERRLOG("%s: copy_from_user error\n", __func__);
		goto done;
	}


	/* parse & conversion */
	for (i = 0; i < ARRAY_SIZE(read_buffer_in); i++) {
		for (j = 0; j < 2; j++) {
			u8 c = toupper(tmp_read_buffer[i * 2 + j]);
			if (isxdigit(c))
				read_buffer_in[i] = (read_buffer_in[i] << 4) |
					(isdigit(c) ? c - '0' : c - 'A' + 10);
			else {
				err = -1;
				ERRLOG("%s: case default error, %d\n",
					__func__, tmp_read_buffer[j]);
				goto done;
			}
		}
	}

	err = do_calibration(spi,
		(struct es209ra_touch_ioctl_clbr *)read_buffer_in,
		(int)read_buffer_in[(PARAM_LENGTH/2)-1]);

done:
	if (err == 0)
		err = count;
	else
		ERRLOG("%s: error return %d\n", __func__, err);

	return err;
}


static int es209ra_touch_write(struct file *file, const char __user *buf,
						size_t count, loff_t *f_pos)
{
	struct spi_device *spi =
		((struct es209ra_touch *)file->private_data)->spi;
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	int err = -1;
	u8 fwver = 0;

	if (count >= 4)
		/*calibrate*/
		err = parse_and_calibrate(tp, spi, buf, count);
	else if (count == 3) {
		/* get mode */
		es209ra_touch_reg_read(tp->spi, TP_REG_BL, &fwver, 1);
		DBGLOG("%s: mode%d\n", __func__, fwver);
	} else {
		/* update firmware*/
		if (tp->state == DRV_STATE_ACTIVE_APP) {
			es209ra_touch_reg_read(tp->spi,
				TP_REG_FWVER, &fwver, 1);
			DBGLOG("%s: firmware version = %d.%d\n",
				__func__, fwver>>4, fwver&0x0F);
		}

#ifdef DEBUG
		DBGLOG("%s: fwver debug\n", __func__);
		if (count != 1)
			fwver = 0x07;
#endif
		DBGLOG("%s: firmware version = %d.%d\n",
				__func__, fwver>>4, fwver&0x0F);

		if (!fwver || (fwver != get_latest_firmware_ver(tp))) {
			DBGLOG("%s: firmware updated is required\n", __func__);
			mutex_lock(&tp->touch_lock);
			err = update_firmware(tp);
			mutex_unlock(&tp->touch_lock);
		}
	}
	return err;
}

static ssize_t es209ra_touch_ioctl(struct inode *inode, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct spi_device *spi =
		((struct es209ra_touch *)file->private_data)->spi;
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	int err = 0;
	struct es209ra_touch_ioctl_clbr data;

	err = check_state(tp, DRV_STATE_ACTIVE_BL);
	if (err == -1) {
		ERRLOG("%s: invalid state\n", __func__);
		goto done;
	}

	switch (cmd) {
	case IOCTL_VALSET:
		if (!access_ok(VERIFY_READ, (void __user *) arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			ERRLOG("%s: invalid access\n", __func__);
			goto done;
		}

		if (copy_from_user(&data, (void __user *) arg, sizeof(data))) {
			err = -EFAULT;
			ERRLOG("%s: copy_from_user error\n", __func__);
			goto done;
		}

		err = do_calibration(spi, &data, 0x00);

		break;
	case IOCTL_VALGET:
		if (!access_ok(VERIFY_WRITE, (void __user *) arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			ERRLOG("%s: invalid access\n", __func__);
			goto done;
		}

		err = do_calibration(spi, &data, 0x01);

		if (copy_to_user((void __user *) arg, &data, sizeof(data))) {
			err = -EFAULT;
			ERRLOG("%s: copy_to_user error\n", __func__);
			goto done;
		}
		break;
	default:
		err = -EINVAL;
		ERRLOG("%s: cmd error\n", __func__);
		goto done;
		break;
	}
done:
	return err;
}

static const struct file_operations es209ra_touch_fops = {
	.owner   = THIS_MODULE,
	.write   = es209ra_touch_write,
	.open	= es209ra_touch_open,
	.ioctl   = es209ra_touch_ioctl,
	.release = es209ra_touch_release,
};

static int __devinit es209ra_touch_probe(struct spi_device *spi)
{
	struct es209ra_touch *tp;
	struct input_dev *input_dev;
	struct es209ra_touch_platform_data *pdata = spi->dev.platform_data;
	int err;
	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;

	/* Check the parameters */
	if (!spi->irq) {
		ERRLOG("%s: no IRQ?\n", __func__);
		return -ENODEV;
	}

	if (!pdata) {
		ERRLOG("%s: no platform data?\n", __func__);
		return -ENODEV;
	}

	/* Set up SPI*/
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	/* Input Device: allocate and set up */
	tp = kzalloc(sizeof(struct es209ra_touch), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!tp || !input_dev) {
		err = -ENOMEM;
		goto err_cleanup_mem;
	}

	/* Device Data: Setup*/
	tp->spi = spi;
	tp->input = input_dev;
	tp->state = DRV_STATE_BEGIN_RESET;
	tp->g_state = NO_TOUCH;
	tp->g_state_err = 0;
	tp->update_counter = 0;

	dev_set_drvdata(&spi->dev, tp);

	/* Input Device: set up */
	input_dev->name = "es209ra_touch";
	input_dev->phys = "es209ra_touch/input0";
	input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	DBGLOG("%s: max x = %d, max y = %d\n", __func__,
				pdata->x_max, pdata->y_max);

	input_set_abs_params(input_dev, ABS_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
						pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
						pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 10, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	err = input_register_device(input_dev);
	if (err)
		goto err_cleanup_mem;

	mutex_init(&tp->touch_lock);

	/* Character Device: set up */
	err = alloc_chrdev_region(&device_t, 0, 1, "es209ra_touch");
	if (err)
		goto err_cleanup_input;

	tp->device_major = MAJOR(device_t);

	cdev_init(&(tp->device_cdev), &es209ra_touch_fops);
	tp->device_cdev.owner = THIS_MODULE;
	tp->device_cdev.ops = &es209ra_touch_fops;
	err = cdev_add(&(tp->device_cdev), MKDEV(tp->device_major, 0), 1);
	if (err)
		goto err_cleanup_chrdev;

	tp->device_class = class_create(THIS_MODULE, "es209ra_touch");
	if (IS_ERR(tp->device_class)) {
		err = -1;
		goto err_cleanup_cdev;
	}

	class_dev_t = device_create(tp->device_class, NULL,
			MKDEV(tp->device_major, 0), NULL, "es209ra_touch");
	if (IS_ERR(class_dev_t)) {
		err = -1;
		goto err_cleanup_class;
	}

	/* register firmware validation and initialization to workqueue*/
	INIT_WORK(&tp->isr_work, es209ra_touch_isr);
	INIT_WORK(&tp->tmowork, es209ra_touch_tmowork);

	/* GPIO: set up*/
	err = gpio_request(pdata->gpio_reset_pin, "esh209ra_touch_gpio_reset");
	if (err)
		goto err_cleanup_device;

	err = gpio_direction_output(pdata->gpio_reset_pin, 1);
	if (err)
		goto err_cleanup_device;

	err = gpio_request(pdata->gpio_irq_pin, "esh209ra_touch_gpio_irq");
	if (err)
		goto err_cleanup_device;

	err = gpio_direction_input(pdata->gpio_irq_pin);
	if (err)
		goto err_cleanup_device;

	/* IRQ: request */
	err = set_irq_type(spi->irq, IRQ_TYPE_LEVEL_LOW);
	if (err)
		goto err_cleanup_device;
	if (request_irq(spi->irq, es209ra_touch_irq, IRQF_TRIGGER_NONE,
		spi->dev.driver->name, tp)) {
		ERRLOG("%s: probe, irq %d busy\n", __func__, spi->irq);
		err = -EBUSY;
		goto err_cleanup_device;
	}
	err = set_irq_type(spi->irq, IRQ_TYPE_EDGE_FALLING);
	if (err)
		goto err_cleanup_irq;

#ifdef CONFIG_ES209RA_TOUCH_EARLYSUSPEND
	/* register early suspend*/
	tp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tp->early_suspend.suspend = es209ra_touch_early_suspend;
	tp->early_suspend.resume = es209ra_touch_late_resume;
	register_early_suspend(&tp->early_suspend);
#endif

	/* reset release device */
	reset_device(pdata);

	init_timer(&tp->tickfn);
	tp->tickfn.function = touch_timeout;
	tp->tickfn.data = (unsigned long)tp;

	return 0;

err_cleanup_irq:
	free_irq(spi->irq, tp);
err_cleanup_device:
	device_destroy(tp->device_class, MKDEV(tp->device_major, 0));
err_cleanup_class:
	class_destroy(tp->device_class);
err_cleanup_cdev:
	cdev_del(&(tp->device_cdev));
err_cleanup_chrdev:
	unregister_chrdev_region(device_t, 1);
err_cleanup_input:

	input_unregister_device(input_dev);
	mutex_destroy(&tp->touch_lock);
err_cleanup_mem:
	input_free_device(input_dev);
	kfree(tp);
	return err;
}

static int __devexit es209ra_touch_remove(struct spi_device *spi)
{
	struct es209ra_touch *tp = dev_get_drvdata(&spi->dev);
	struct es209ra_touch_platform_data *pdata = spi->dev.platform_data;
	int err;
	dev_t device_t = MKDEV(tp->device_major, 0);

	err = gpio_direction_output(pdata->gpio_reset_pin, 1);
	if (err)
		return err;

	es209ra_touch_suspend(spi, PMSG_SUSPEND);

#ifdef CONFIG_ES209RA_TOUCH_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif

	input_unregister_device(tp->input);
	device_destroy(tp->device_class, MKDEV(tp->device_major, 0));
	class_destroy(tp->device_class);
	cdev_del(&(tp->device_cdev));
	unregister_chrdev_region(device_t, 1);
	input_free_device(tp->input);
	free_irq(tp->spi->irq, tp);
	mutex_destroy(&tp->touch_lock);
	kfree(tp);

	DBGLOG("%s: unregistered touchscreen\n", __func__);

	return 0;
}

static struct spi_driver es209ra_touch_driver = {
	.driver = {
		.name	= "es209ra_touch",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= es209ra_touch_probe,
	.remove		= __devexit_p(es209ra_touch_remove),
#ifndef CONFIG_ES209RA_TOUCH_EARLYSUSPEND
	.suspend	= es209ra_touch_suspend,
	.resume		= es209ra_touch_resume,
#endif
};

static int __init es209ra_touch_init(void)
{
	int err;

	err = spi_register_driver(&es209ra_touch_driver);

	printk(KERN_INFO "\nes209ra_touch, module init, result = %d \n", err);

	return err;
}
module_init(es209ra_touch_init);

static void __exit es209ra_touch_exit(void)
{
	spi_unregister_driver(&es209ra_touch_driver);
	printk(KERN_INFO "\nes209ra_touch, module exit\n");
}
module_exit(es209ra_touch_exit);

MODULE_DESCRIPTION("es209ra TouchScreen Driver");
MODULE_LICENSE("GPL");
