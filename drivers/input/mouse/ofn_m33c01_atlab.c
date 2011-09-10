/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/ofn_atlab.h>
#include <mach/io.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("Atlab Optical finger navigation driver");
MODULE_ALIAS("platform:ofn-atlab");

#define OPTNAV_SLAVE_REG		0x0
#define OPTNAV_POLLING_STATUS_REG	0x1
#define OPTNAV_KEYS_STATUS_REG		0x2
#define OPTNAV_MOTION_X_REG		0x3
#define OPTNAV_MOTION_Y_REG		0x4
#define OPTNAV_BLK_INT_CONFIG_REG	0x5
#define OPTNAV_FUNCTION_1_REG		0x6
#define OPTNAV_FUNCTION_2_REG		0x7
#define OPTNAV_COLD_RESET_REG		0xFE

#define OPTNAV_XY_BUTTON_MASK		0xC0
#define OPTNAV_XY_MOTION_MASK		0x80
#define OPTNAV_KEY_1_2_3_MASK		0x07
#define OPTNAV_KEY1_PUSHED_MASK		0x01
#define OPTNAV_KEY2_PUSHED_MASK		0x02
#define OPTNAV_KEY3_PUSHED_MASK		0x04

#define OPTNAV_CLK_FREQ_MASK		0x1F
#define OPTNAV_CLK_FREQ			0x5
#define OPTNAV_FAST_CPI			BIT(4)
#define OPTNAV_TSEN_EN			BIT(3)
#define OPTNAV_NO_MOTION		BIT(1)
#define OPTNAV_OFN_EN			BIT(0)

#define OPTNAV_INVERT_Y			BIT(7)
#define OPTNAV_INVERT_X			BIT(5)
#define OPTNAV_SWAP_XY			BIT(4)
#define OPTNAV_HOLD_A_B_EN		BIT(2)
#define OPTNAV_MOTION_FILTER_EN		BIT(1)

#define OPTNAV_SLEEP_PERIOD		(msecs_to_jiffies(10))
#define OPTNAV_STABILIZE_DELAY_MS       10

#define QUALCOMM_VENDORID               0x5413

#define DRIVER_NAME                    "ofn_atlab_m33"
#define DEVICE_NAME                    "m33c01"
#define OPTNAV_NAME                    "Atlab OFN"
#define OPTNAV_DEVICE                  "/dev/ofn"

struct optnav_data {
	struct input_dev  *optnav_dev;
	struct i2c_client *clientp;
	int              (*optnav_on) (void);
	void             (*optnav_off)(void);
	struct ofn_function1 function1;
	struct ofn_function2 function2;
	struct delayed_work optnav_work;
	struct workqueue_struct *work_q;
};


int optnav_check_xy_motion(struct optnav_data *dd, int poll_status)
{
	int rc1 = 0;
	s8 x_val, y_val;

	if (poll_status & OPTNAV_XY_MOTION_MASK) {
		rc1 = i2c_smbus_read_word_data(dd->clientp,
			       OPTNAV_MOTION_X_REG);
		if (rc1 < 0) {
			dev_err(&dd->clientp->dev, "%s:i2c read fail\n",
							 __func__);
			return rc1;
		}
		x_val = rc1 & 0xFF;
		y_val = rc1 >> 8 & 0xFF;

		input_report_rel(dd->optnav_dev, REL_X, x_val);
		input_report_rel(dd->optnav_dev, REL_Y, y_val);
	}
	return 0;
}

/*
 * REVISIT -- Enable the power saving mode
 * The device operates in power saving mode when PD (power down) pin is
 * enabled. This is of interest if we have an interrupt based mechanism.
 */
static void optnav_work_f(struct work_struct *work)
{
	struct optnav_data     *dd =
		container_of(work, struct optnav_data, optnav_work.work);
	int                     rc, rc1, rc2, delay = 0, button = 0;

	/*
	 * Poll the device. While we keep getting new data, poll with
	 * no additional delay. When we receive no new data, gradually
	 * reduce the time between polls. On reaching a 100ms delay
	 * reschedule the work to restart the polling again.
	 */
	do {
		if (delay)
			msleep(delay);

		rc = i2c_smbus_read_byte_data(dd->clientp,
				OPTNAV_POLLING_STATUS_REG);
		rc2 = i2c_smbus_read_byte_data(dd->clientp,
				OPTNAV_KEYS_STATUS_REG);
		/*
		 * Read required for key status register
		 * for further correct reads
		 */
		if (rc < 0 || rc2 < 0) {
			dev_err(&dd->clientp->dev,
				"%s: i2c read failed\n", __func__);
			goto on_workf_exit;
		}
		if (rc & OPTNAV_XY_BUTTON_MASK)	{
			/* Either X,Y motion or button press */
			button = 0;
			delay = 0;

			rc1 = optnav_check_xy_motion(dd, rc);
			if (rc1 < 0)
				goto on_workf_exit;

			while ((rc2 & OPTNAV_KEY_1_2_3_MASK) != 0) {
				/*
				 * Check if any of the buttons are
				 * pressed.
				 */
				if (rc2 & OPTNAV_KEY1_PUSHED_MASK) {
					input_report_key(dd->optnav_dev,
							BTN_RIGHT, 1);
					button = 1;
				} else {
					input_report_key(dd->optnav_dev,
							BTN_LEFT, 1);
					button = 2;
				}
				rc2 = i2c_smbus_read_byte_data(dd->clientp,
						OPTNAV_KEYS_STATUS_REG);
				if (rc2 < 0) {
					dev_err(&dd->clientp->dev,
						"%s:i2c read failed\n",
							__func__);
					goto on_workf_exit;
				}
			}
			if (button == 1)
				input_report_key(dd->optnav_dev, BTN_RIGHT, 0);
			else if (button == 2)
				input_report_key(dd->optnav_dev, BTN_LEFT, 0);

			input_sync(dd->optnav_dev);
		} else
			delay += 10;

	} while (delay <= 40);

on_workf_exit:
	queue_delayed_work(dd->work_q, &dd->optnav_work, OPTNAV_SLEEP_PERIOD);
}

static int optnav_enable(struct optnav_data *dd)
{
	int rc = 0, rc1 = 0, rc2 = 0;
	int clk_freq = 0;
	u8 function1 = 0, function2 = 0, count = 0;

	if (dd->optnav_on != NULL) {
		rc = dd->optnav_on();
		if (rc)
			goto on_enable_exit;
	}
	mdelay(OPTNAV_STABILIZE_DELAY_MS);

	/* Check if the I2C slave address matches */
	rc = i2c_smbus_read_byte_data(dd->clientp, OPTNAV_SLAVE_REG);
	if (rc < 0) {
		dev_err(&dd->clientp->dev,
			"optnav_enable: I2C read failed \n");
		goto on_i2c_write_fail;
	}
	if (dd->clientp->addr != ((rc & 0xFF) >> 1)) {
		pr_err("%s: I2C slave addresses do not match", __func__);
		rc = -ENODEV;
		goto on_i2c_write_fail;
	}

	/* Reset all blocks (including register block) of the device */
	rc = i2c_smbus_write_byte_data(dd->clientp, OPTNAV_COLD_RESET_REG,
								0xFF);
	if (rc < 0) {
		dev_err(&dd->clientp->dev,
			"optnav_enable: I2C write failed \n");
		goto on_i2c_write_fail;
	}

	/* Get Function1 register default configuration */
	function1 = i2c_smbus_read_byte_data(dd->clientp,
					OPTNAV_FUNCTION_1_REG);

	/* Set Function1 register configuration */
	if (dd->function1.no_motion1_en == true)
		function1 |= OPTNAV_NO_MOTION;
	else
		function1 &= ~OPTNAV_NO_MOTION;

	if (dd->function1.touch_sensor_en == false)
		function1 |= OPTNAV_TSEN_EN;
	else
		function1 &= ~OPTNAV_TSEN_EN;

	if (dd->function1.ofn_en == true)
		function1 |= OPTNAV_OFN_EN;
	else
		function1 &= ~OPTNAV_OFN_EN;

	clk_freq = dd->function1.clock_select_khz;
	if (clk_freq % 750) {
		pr_err("%s: Wrong input param in function1 \n", __func__);
		rc = -EINVAL;
		goto on_i2c_write_fail;
	}
	clk_freq /= 750;
	if ((clk_freq >= 1) && (clk_freq <= 8) && !(clk_freq & (clk_freq-1))) {
		/*
		 * Clock frequency can only be .75, 1.5, 3, 6MHz
		 * the bit values for these freqs are 100,010,011,001
		 */
		do {
			count++;
			clk_freq >>= 1;
		} while (clk_freq != 0);
		count = 5 - count;
		function1 &= OPTNAV_CLK_FREQ_MASK;
		function1 |= (count << OPTNAV_CLK_FREQ);
	} else {
		pr_err("%s: Wrong input param in function1 \n", __func__);
		rc = -EINVAL;
		goto on_i2c_write_fail;
	}

	if (dd->function1.cpi_selection == 600)
		function1 |= OPTNAV_FAST_CPI;
	else if (dd->function1.cpi_selection == 1200)
		function1 &= ~OPTNAV_FAST_CPI;
	else {
		pr_err("%s: Wrong input param in function1 \n", __func__);
		rc = -EINVAL;
		goto on_i2c_write_fail;
	}

	/* Get Function2 register default configuration */
	function2 = i2c_smbus_read_byte_data(dd->clientp,
					OPTNAV_FUNCTION_2_REG);

	/* Set Function2 register configuration */
	if (dd->function2.invert_y == true)
		function2 |= OPTNAV_INVERT_Y;
	else
		function2 &= ~OPTNAV_INVERT_Y;

	if (dd->function2.invert_x == true)
		function2 |= OPTNAV_INVERT_X;
	else
		function2 &= ~OPTNAV_INVERT_X;

	if (dd->function2.swap_x_y == true)
		function2 |= OPTNAV_SWAP_XY;
	else
		function2 &= ~OPTNAV_SWAP_XY;

	if (dd->function2.hold_a_b_en == true)
		function2 |= OPTNAV_HOLD_A_B_EN;
	else
		function2 &= ~OPTNAV_HOLD_A_B_EN;

	if (dd->function2.motion_filter_en == true)
		function2 |= OPTNAV_MOTION_FILTER_EN;
	else
		function2 &= ~OPTNAV_MOTION_FILTER_EN;

	/*
	 * Set the interrupt config. register to disable the
	 * interrupts from motion and key presses as we are
	 * operating in polling mode.
	 */
	rc = i2c_smbus_write_byte_data(dd->clientp, OPTNAV_BLK_INT_CONFIG_REG,
									0x1);

	rc1 = i2c_smbus_write_byte_data(dd->clientp, OPTNAV_FUNCTION_1_REG,
								function1);

	rc2 = i2c_smbus_write_byte_data(dd->clientp, OPTNAV_FUNCTION_2_REG,
								function2);

	if (rc < 0 || rc1 < 0 || rc2 < 0) {
		dev_err(&dd->clientp->dev,
			"optnav_enable: I2C write failed\n");
		goto on_i2c_write_fail;
	}
	/*
	 * Schedule the work only if the device has been
	 * initialized correctly.
	 */
	queue_delayed_work(dd->work_q, &dd->optnav_work, OPTNAV_SLEEP_PERIOD);

	return rc;

on_i2c_write_fail:
	if (dd->optnav_off != NULL)
		dd->optnav_off();
on_enable_exit:
	return rc;
}

static int optnav_dev_open(struct input_dev *dev)
{
	int rc;
	struct optnav_data *dd = input_get_drvdata(dev);
	if (!dd->clientp) {
		/* Check if a valid i2c client is present */
		pr_err("%s: no i2c adapter present\n", __func__);
		return  -ENODEV;
	}

	rc = optnav_enable(dd);
	if (rc) {
		pr_err("%s: optnav_enable failed \n", __func__);
		return  -ENODEV;
	}

	return rc;
}

static void optnav_dev_close(struct input_dev *dev)
{
	struct optnav_data *dd = input_get_drvdata(dev);
	cancel_delayed_work_sync(&dd->optnav_work);
	/* Disable the device */
	if (dd->optnav_off != NULL)
		dd->optnav_off();
}

static int __devexit optnav_remove(struct i2c_client *client)
{
	struct optnav_data              *dd;
	struct ofn_atlab_platform_data  *pd;

	dd = i2c_get_clientdata(client);
	pd = client->dev.platform_data;
	destroy_workqueue(dd->work_q);
	input_unregister_device(dd->optnav_dev);
	if (pd->gpio_release != NULL)
		pd->gpio_release();
	kfree(dd);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
/* TODO: Power optimization in suspend and resume */
static int optnav_suspend(struct i2c_client *client,
			    pm_message_t state)
{
	struct optnav_data *dd;

	dd = i2c_get_clientdata(client);
	cancel_delayed_work_sync(&dd->optnav_work);

	return 0;
}

static int optnav_resume(struct i2c_client *client)
{
	struct optnav_data *dd;

	dd = i2c_get_clientdata(client);
	queue_delayed_work(dd->work_q, &dd->optnav_work, OPTNAV_SLEEP_PERIOD);

	return 0;
}
#endif

static const struct i2c_device_id optnav_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, optnav_id);

static int __devinit optnav_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int                              rc = -1;
	struct ofn_atlab_platform_data  *pd;
	struct optnav_data              *dd;

	dd = kzalloc(sizeof *dd, GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	i2c_set_clientdata(client, dd);
	dd->clientp = client;
	pd = client->dev.platform_data;
	if (!pd) {
		dev_err(&client->dev, "optnav_probe: platform data not set\n");
		rc = -EFAULT;
		goto probe_free_exit;
	}

	dd->optnav_on = pd->optnav_on;
	dd->optnav_off = pd->optnav_off;
	dd->function1 = pd->function1;
	dd->function2 = pd->function2;

	if (pd->gpio_setup == NULL) {
		rc = -EINVAL;
		goto probe_free_exit;
	}
	rc = pd->gpio_setup();
	if (rc) {
		rc = -EINVAL;
		goto probe_free_exit;
	}

	dd->optnav_dev = input_allocate_device();
	if (!dd->optnav_dev) {
		rc = -ENOMEM;
		goto probe_fail_in_alloc;
	}

	input_set_drvdata(dd->optnav_dev, dd);

	dd->optnav_dev->open       = optnav_dev_open;
	dd->optnav_dev->close      = optnav_dev_close;
	dd->optnav_dev->name       = OPTNAV_NAME;
	dd->optnav_dev->phys       = OPTNAV_DEVICE;
	dd->optnav_dev->id.bustype = BUS_I2C;
	dd->optnav_dev->id.vendor  = QUALCOMM_VENDORID;
	dd->optnav_dev->id.product = 1;
	dd->optnav_dev->id.version = 1;
	input_set_capability(dd->optnav_dev, EV_REL, REL_X);
	input_set_capability(dd->optnav_dev, EV_REL, REL_Y);
	input_set_capability(dd->optnav_dev, EV_KEY, BTN_LEFT);
	input_set_capability(dd->optnav_dev, EV_KEY, BTN_RIGHT);

	rc = input_register_device(dd->optnav_dev);
	if (rc) {
		dev_err(&client->dev,
			"optnav_probe: input_register_device rc=%d\n", rc);
		rc = -EINVAL;
		goto probe_fail_in_alloc;
	}

	dd->work_q = create_singlethread_workqueue("kofn_workq");
	if (dd->work_q == NULL) {
		pr_err("%s: Unable to create a work queue\n", __func__);
		rc = -ENOMEM;
		goto probe_fail_work_q;
	}
	INIT_DELAYED_WORK(&dd->optnav_work, optnav_work_f);

	return 0;

probe_fail_work_q:
	input_unregister_device(dd->optnav_dev);
	dd->optnav_dev = NULL;
probe_fail_in_alloc:
	if (pd->gpio_release != NULL)
		pd->gpio_release();
probe_free_exit:
	kfree(dd);
	i2c_set_clientdata(client, NULL);
probe_exit:
	return rc;
}

static struct i2c_driver optnav_driver = {
	.driver = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
	.probe   = optnav_probe,
	.remove  =  __devexit_p(optnav_remove),
#ifdef CONFIG_PM
	.suspend = optnav_suspend,
	.resume  = optnav_resume,
#endif
	.id_table = optnav_id,
};

static int __init optnav_init(void)
{
	int rc;

	rc = i2c_add_driver(&optnav_driver);
	if (rc)
		printk(KERN_ERR "optnav_init FAILED: i2c_add_driver rc=%d\n",
								       rc);
	return rc;
}

static void __exit optnav_exit(void)
{
	i2c_del_driver(&optnav_driver);
}

module_init(optnav_init);
module_exit(optnav_exit);
