/* arch/arm/mach-msm/semc_seport_platform.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/misc_modem_api.h>
#include <mach/semc_seport_platform.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/semc/seport/seport_dcout_iface.h>
#include <mach/qdsp5/snd_adie.h>
#include <mach/rpc_server_handset.h>
#include <mach/oem_rapi_client.h>
#include <mach/semc_rpc_server_handset.h>
#include <linux/errno.h>
#include <asm/atomic.h>

/** Defines **/
#define SEPORT_MIN_VALID_VAD_LEVEL 116
#define SEPORT_PLATFORM_INITIAL_STATE 0
#define SEPORT_PLATFORM_INITIAL_CLID -1

/** Local variables **/
struct seport_config *seport_conf;
static u8 seport_plug_detect_interrupt_enabled =
	SEPORT_PLATFORM_INITIAL_STATE;
static u8 seport_button_detect_interrupt_enabled =
	SEPORT_PLATFORM_INITIAL_STATE;
static int seport_pf_client_id =
	SEPORT_PLATFORM_INITIAL_CLID;
static struct msm_rpc_client *rpc_client;

static int (*seport_vad_det_cb_func)(int, void*);
static struct seport_plug_detect_data *seport_vad_data;

static atomic_t vad_detection_enabled = ATOMIC_INIT(0);


/** Forward declarations **/
static int seport_platform_read_mean_rid_val(u8 samples);
static int seport_platform_register_mic_bias_client(u8 register_client);
static int register_rapi_client(void);

/** Public functions **/
int seport_platform_get_phf_value(u16 *phf_val)
{

	int err = msm_adc_read(APP_ADC_CHANNEL_SYSCON_PHF, phf_val);
	if (err)
		printk(KERN_CRIT "Failed to read a correct RID value.\n");

	return err;
}
EXPORT_SYMBOL_GPL(seport_platform_get_phf_value);

int seport_platfrom_get_rid_value(u16 *ridVal)
{
	int err = gpio_tlmm_config(GPIO_CFG(seport_conf->button_detect_pin,
					    0, GPIO_INPUT, GPIO_NO_PULL,
					    GPIO_2MA),
				   GPIO_ENABLE);
	if (err) {
		printk(KERN_CRIT "Failed to set button interrupt GPIO pin to" \
		       " correct state\n");
		return err;
	}
	msleep(10);

	gpio_set_value(seport_conf->rid_ping_enable_pin, 0);
	if (err) {
		printk(KERN_CRIT "Failed to enable RID ping pin\n");
		return err;
	}
	/* Need this in order to get thre required RID poll time */
	msleep(20);

	err = msm_adc_read(APP_ADC_CHANNEL_SYSCON_AID, ridVal);
	if (err) {
		printk(KERN_CRIT "Failed to read a correct RID value.\n");
		return err;
	}

	gpio_set_value(seport_conf->rid_ping_enable_pin, 1);

	/* Ignoring return values here. We can not do much about them. */
	(void)gpio_tlmm_config(GPIO_CFG(seport_conf->button_detect_pin,
					    0, GPIO_OUTPUT, GPIO_PULL_DOWN,
					GPIO_2MA),
				   GPIO_ENABLE);

	msleep(40);

	return err;
}
EXPORT_SYMBOL_GPL(seport_platfrom_get_rid_value);

int seport_platform_register_button_gpio_callback(int (*func)(int, void *arg),
						  void *data)
{
	int retval, mean_rid, mean_ain, irq;

	/* TODO: Make sure that we handle errors in a later state! */
	gpio_set_value(seport_conf->rid_ping_enable_pin, 0);
	msleep(10);
	retval = gpio_tlmm_config(GPIO_CFG(seport_conf->button_detect_pin,
					   0, GPIO_INPUT, GPIO_NO_PULL,
					   GPIO_2MA),
				  GPIO_ENABLE);
	if (retval) {
		printk(KERN_CRIT "Failed to configure GPIO pin. Aborting\n");
		gpio_set_value(seport_conf->rid_ping_enable_pin, 0);
		return retval;
	}
	/* Delay a bit to make sure GPIOs are in the correct mode. */
	msleep(10);

	mean_rid = seport_platform_read_mean_rid_val(3);

	/* Enabling DCout to see if we have different AIN value than RID.
	 * This is used to filter out inserted accessories with pressed
	 * buttons. */
	(void)seport_set_dcout(1);
	msleep(10);

	mean_ain = seport_platform_read_mean_rid_val(3);

	/*
	 * Here we try to detect if we have a valid AIN accessory. This will
	 * not handle if RID == AIN which can be the case in very special
	 * circumstances even though the accessory is a valid AIN event. This
	 * can happen if a button is pressed when the accessory is inserted and
	 * we have no way to filter that specific case.
	 * The +-4 steps should be a good range to handle different RID values.
	 */
	if (abs(mean_ain - mean_rid) > 4) {
		irq = gpio_to_irq(seport_conf->button_detect_pin);
		if (0 <= irq) {
			retval = request_irq(irq,
					     func, IRQF_TRIGGER_FALLING,
					     "seport_buttondetect",
					     data);

			if (retval) {
				printk(KERN_CRIT "Failed to register" \
				       " interrupt handler for button" \
				       " detection\n");
				return retval;
			}
		} else {
			printk(KERN_CRIT "%s - Irq for requested GPIO (%d) " \
			       "does not exist. Button detection will not " \
			       "work\n", __func__,
			       seport_conf->button_detect_pin);
			return -ENXIO;
		}

		/*
		 * Setting interrupt enabled here, will in worst case generate
		 * a "unmatched irq_wake" print in the kernel log when
		 * shutting down the system, but at least some detection will
		 * work.
		 */
		seport_button_detect_interrupt_enabled = 1;

		retval = enable_irq_wake(irq);
		if (retval)
			printk(KERN_CRIT "Failed to enable wakup on IRQ " \
			       "request\n");

		return retval;
	} else {
		printk(KERN_INFO "Accessory was not detected as an AIN " \
		       "accessory or a special button was pressed while " \
		       "inserting accessory\n");

		/* Ignoring return values here. Can't do much about
		   them if they fail. */
		gpio_set_value(seport_conf->rid_ping_enable_pin, 1);

		(void)gpio_tlmm_config(GPIO_CFG(seport_conf->button_detect_pin,
						0, GPIO_OUTPUT, GPIO_PULL_DOWN,
						GPIO_2MA),
				       GPIO_ENABLE);
	}
	return -3; /* ERROR_NO_AIN_ACCESSORY; */
}
EXPORT_SYMBOL_GPL(seport_platform_register_button_gpio_callback);

int seport_platform_get_button_id(u16 *button_id)
{
	int err = msm_adc_read(APP_ADC_CHANNEL_SYSCON_AID, button_id);;

	if (err)
		printk(KERN_INFO "Failed to read a valid Button ID value\n");

	return err;
}
EXPORT_SYMBOL_GPL(seport_platform_get_button_id);

void seport_platform_unregister_button_gpio_callback(void)
{
	/* Ignoring return values here. We can not do
	   much about any problems here.*/
	int irq;
	(void)gpio_direction_output(seport_conf->button_detect_pin, 0);
	gpio_set_value(seport_conf->rid_ping_enable_pin, 1);
	if (seport_button_detect_interrupt_enabled) {
		irq = gpio_to_irq(seport_conf->button_detect_pin);
		if (0 <= irq) {
			(void)disable_irq_wake(irq);
			free_irq(irq, NULL);
			seport_button_detect_interrupt_enabled = 0;
		} else {
			printk(KERN_CRIT "%s - Failed to unregister button " \
			       "detect interrupt. GPIO (%d) does not exist",
			       __func__, seport_conf->button_detect_pin);
		}
	}
}
EXPORT_SYMBOL_GPL(seport_platform_unregister_button_gpio_callback);

int seport_platform_register_plug_detect_gpio_callback(
	int (*func)(int, void *arg), void * data)
{
	int err;
	int irq;

	(void)gpio_direction_output(seport_conf->headset_detect_enable_pin, 1);

	irq = gpio_to_irq(seport_conf->plug_detect_read_pin);
	if (0 <= irq) {
		err = request_irq(irq,
				  func, IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
				  "seport_plug_detect",
				  data);
		if (err) {
			printk(KERN_CRIT "%s - Failed to subscribe to plug " \
			       "detect interrupt\n", __func__);
			return err;
		}
	} else {
		printk(KERN_CRIT "%s - Failed to register interrupt for GPIO " \
		       "(%d). GPIO Does not exist\n", __func__,
		       seport_conf->plug_detect_read_pin);
		return -ENXIO;
	}

	/*
	 * Setting interrupt enabled here, will in worst case generate
	 * a "unmatched irq_wake" print in the kernel log when
	 * shutting down the system, but at least some detection will work.
	 */
	seport_plug_detect_interrupt_enabled = 1;

	err = enable_irq_wake(irq);
	if (err)
		printk(KERN_CRIT "%s - Failed to enable wakeup on interrupt\n",
		       __func__);

	return err;
}
EXPORT_SYMBOL_GPL(seport_platform_register_plug_detect_gpio_callback);

void seport_platform_unregister_plug_detect_gpio_callback(void *data)
{
	int irq;

	(void)gpio_direction_output(seport_conf->headset_detect_enable_pin, 0);

	if (seport_plug_detect_interrupt_enabled) {
		irq = gpio_to_irq(seport_conf->plug_detect_read_pin);
		if (0 <= irq) {
			disable_irq_wake(irq);
			free_irq(irq, data);
			seport_plug_detect_interrupt_enabled = 0;
		} else {
			printk(KERN_CRIT "%s - Failed to disable plug detect" \
			       "interrupt. GPIO (%d) does not exist\n",
			       __func__, seport_conf->plug_detect_read_pin);
		}
	}
}
EXPORT_SYMBOL_GPL(seport_platform_unregister_plug_detect_gpio_callback);

#if defined(CONFIG_SEPORT_VIDEO_OUT)
int seport_platform_enable_video_out_switch(int enable)
{
	return gpio_direction_output(seport_conf->video_out_switch, level);
}
EXPORT_SYMBOL_GPL(seport_platform_enable_video_out_switch);
#endif /* CONFIG_SEPORT_VIDEO_OUT */

int seport_platform_read_detect_pin_value(void)
{
	return gpio_get_value(seport_conf->plug_detect_read_pin);
}
EXPORT_SYMBOL_GPL(seport_platform_read_detect_pin_value);

void seport_platform_disable_button_detect_interrupt(void)
{
	int irq = gpio_to_irq(seport_conf->button_detect_pin);
	if (0 <= irq)
		disable_irq(irq);
	else
		printk(KERN_WARNING "%s - Failed to disable button detect " \
		       "interrupt. IRQ for GPIO (%d).GPIO does not exist.\n",
		       __func__, seport_conf->button_detect_pin);
}
EXPORT_SYMBOL_GPL(seport_platform_disable_button_detect_interrupt);

void seport_platform_enable_button_detect_interrupt(void)
{
	int irq = gpio_to_irq(seport_conf->button_detect_pin);
	if (0 <= irq)
		enable_irq(irq);
	else
		printk(KERN_WARNING "%s - Failed to enable button detect " \
		       "interrupt. IRQ for GPIO (%d).GPIO does not exist.\n",
		       __func__, seport_conf->button_detect_pin);
}
EXPORT_SYMBOL_GPL(seport_platform_enable_button_detect_interrupt);

void seport_platform_disable_detect_interrupt(void)
{
	int irq = gpio_to_irq(seport_conf->plug_detect_read_pin);
	if (0 <= irq)
		disable_irq(irq);
	else
		printk(KERN_WARNING "%s - Failed to disable plug detect " \
		       "interrupt. IRQ for GPIO (%d).GPIO does not exist.\n",
		       __func__, seport_conf->button_detect_pin);
}
EXPORT_SYMBOL_GPL(seport_platform_disable_detect_interrupt);

void seport_platform_enable_detect_interrupt(void)
{
	int irq = gpio_to_irq(seport_conf->plug_detect_read_pin);
	if (0 <= irq)
		enable_irq(irq);
	else
		printk(KERN_WARNING "%s - Failed to enable plug detect " \
		       " interrupt. IRQ for GPIO (%d).GPIO does not exist.\n",
		       __func__, seport_conf->button_detect_pin);

}
EXPORT_SYMBOL_GPL(seport_platform_enable_detect_interrupt);

int seport_platform_register_vad_button_callback(int (*func)(int, void *arg),
						 void *data)
{
	u16 hsd_val;
	u8 i;
	u16 last_hsd_val = 0xFFFF;

	if (!atomic_read(&vad_detection_enabled)) {
		seport_platform_enable_mic_bias(1);
		msleep(5);

		for (i = 0; i < 10; i++) {
			(void)seport_platform_get_phf_value(&hsd_val);
			last_hsd_val = min(last_hsd_val, hsd_val);
			msleep(5);
		}

		if(last_hsd_val <= SEPORT_MIN_VALID_VAD_LEVEL) {
			seport_platform_enable_mic_bias(0);
			printk(KERN_WARNING
			       "%s - Too low MIC Bias level. VAD will not work." \
			       "Aborting!\n", __func__);
			return -EFAULT;
		}

		report_headset_status(1);
		seport_vad_det_cb_func = func;
		seport_vad_data = data;
		atomic_set(&vad_detection_enabled, 1);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(seport_platform_register_vad_button_callback);

void seport_platform_unregister_vad_button_callback(void)
{
	if (atomic_read(&vad_detection_enabled)) {
		seport_platform_enable_mic_bias(0);
		report_headset_status(0);
		report_headset_status(0);
		msleep(50);
		seport_platform_enable_mic_bias(0);
		seport_vad_det_cb_func = NULL;
		seport_vad_data = NULL;
		atomic_set(&vad_detection_enabled, 0);
	}
}
EXPORT_SYMBOL_GPL(seport_platform_unregister_vad_button_callback);


int seport_platform_enable_hp_amp(u8 enable)
{
	return 0;
}
EXPORT_SYMBOL_GPL(seport_platform_enable_hp_amp);

int seport_platform_enable_mic_bias(u8 enable)
{
	int err = -1;

	if (seport_pf_client_id == -1) {
		printk(KERN_ERR "%s - No registered client when trying to " \
		       "start MIC Bias. Aborting!", __func__);
		goto out;
	}

	if (enable) {
		err = adie_svc_config_adie_block(seport_pf_client_id, HSSD, 1);

		if (ADIE_SVC_STATUS_SUCCESS != err) {
			if (ADIE_SVC_STATUS_INUSE == err)
				printk(KERN_INFO "%s - Ooops! Client is in " \
				       "use\n", __func__);
			else
				printk(KERN_INFO "%s - Enabling of HSSD " \
				       "failed!\n", __func__);
			goto out;
		}

		err = adie_svc_config_adie_block(seport_pf_client_id,
						 MIC_BIAS, 1);

		if (ADIE_SVC_STATUS_SUCCESS != err) {
			if (ADIE_SVC_STATUS_INUSE == err)
				printk(KERN_INFO "%s - Ooops! Client is in " \
				       "use\n", __func__);
			else
				printk(KERN_INFO "%s - Enabling of MIC Bias " \
				       "failed!\n", __func__);
			goto out;
		}
	} else {
		err = adie_svc_config_adie_block(seport_pf_client_id,
						 MIC_BIAS, 0);

		if (ADIE_SVC_STATUS_SUCCESS != err) {
			if (ADIE_SVC_STATUS_INUSE == err)
				printk(KERN_INFO "%s - Ooops! Client is in " \
				       "use\n", __func__);
			else
				printk(KERN_INFO "%s - Enabling of MIC Bias " \
				       "failed!\n", __func__);
			goto out;
		}


		err = adie_svc_config_adie_block(seport_pf_client_id, HSSD, 0);

		if (ADIE_SVC_STATUS_SUCCESS != err) {
			if (ADIE_SVC_STATUS_INUSE == err)
				printk(KERN_INFO "%s - Ooops! Client is in " \
				       "use\n", __func__);
			else
				printk(KERN_INFO "%s - Enabling of HSSD " \
				       "failed!\n", __func__);
			goto out;
		}
	}
out:
	return err;
}
EXPORT_SYMBOL_GPL(seport_platform_enable_mic_bias);

int seport_platform_get_hssd_threshold(u8 max_val, int *value)
{
	int ret = 0;

	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;

	if (!rpc_client) {
		printk(KERN_CRIT "%s - No communication channel available. " \
		       "Aborting!", __func__);
		return -EPERM;
	}

	memset(&client_arg, 0, sizeof client_arg);

	client_arg.event = OEM_RAPI_CLIENT_EVENT_SEPORT_HSSD_VAL_GET;
	client_arg.in_len = sizeof(max_val);
	client_arg.input = (char *)&max_val;
	client_arg.out_len_valid = 1;
	client_arg.output_valid = 1;
	client_arg.output_size = sizeof(*value);

	client_ret.out_len = NULL;
	client_ret.output = NULL;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret < 0)
		printk(KERN_INFO
		       "%s : oem_rapi_client_str_func returned error %d\n",
		       __func__, ret);
	else if (client_ret.out_len == NULL ||
		 client_ret.output == NULL ||
		 *client_ret.out_len != client_arg.output_size)
		ret = -ENOMSG;
	else
		*value = *(int *)client_ret.output;

	kfree(client_ret.out_len);
	kfree(client_ret.output);

	return ret;
}
EXPORT_SYMBOL_GPL(seport_platform_get_hssd_threshold);

int seport_platform_set_hssd_threshold(u8 max, int value)
{
	int ret = 0;
	u8 remote_set_fail = 0;
	uint32_t val;

	struct oem_rapi_client_streaming_func_arg client_arg;
	struct oem_rapi_client_streaming_func_ret client_ret;

	if (0 > value) {
		printk(KERN_ERR "%s - This system doesn't support " \
		       "negative values. Aborting!\n", __func__);
		return -EINVAL;
	}

	val = (uint)value;

	memset(&client_arg, 0, sizeof client_arg);

	if (max)
		client_arg.event = OEM_RAPI_CLIENT_EVENT_SEPORT_HSSD_MAX_SET;
	else
		client_arg.event = OEM_RAPI_CLIENT_EVENT_SEPORT_HSSD_MIN_SET;

	client_arg.in_len = sizeof(val);
	client_arg.input = (char *)&val;
	client_arg.out_len_valid = 1;
	client_arg.output_valid = 1;
	client_arg.output_size = sizeof(remote_set_fail);

	client_ret.out_len = NULL;
	client_ret.output = NULL;

	ret = oem_rapi_client_streaming_function(rpc_client,
						 &client_arg,
						 &client_ret);
	if (ret < 0)
		printk(KERN_WARNING
		       "%s : Failed setting HSSD detection value. Error %d\n",
		       __func__, ret);
	else if (NULL == client_ret.out_len ||
		 NULL == client_ret.output ||
		 *client_ret.out_len != client_arg.output_size)
		ret = -ENOMSG;
	else if (*(u8 *)client_ret.output)
		ret = -ENOEXEC;

	kfree(client_ret.out_len);
	kfree(client_ret.output);

	return ret;
}
EXPORT_SYMBOL_GPL(seport_platform_set_hssd_threshold);

void seport_platform_vad_callback(uint32_t key, uint32_t event)
{
	if (!atomic_read(&vad_detection_enabled))
		return;

	if (key == HS_HEADSET_SWITCH_K) {
		atomic_set(&seport_vad_data->status, event == HS_REL_K);
		seport_vad_det_cb_func(
			atomic_read(&seport_vad_data->status),
			seport_vad_data);
	}
}
EXPORT_SYMBOL_GPL(seport_platform_vad_callback);

int seport_platform_enable_mic_bias_measurement(u8 enable)
{
	int err = adie_svc_config_adie_block(seport_pf_client_id,
					     HPH_PA, enable);

	if (ADIE_SVC_STATUS_SUCCESS != err && ADIE_SVC_STATUS_INUSE != err) {
			printk(KERN_CRIT
			       "%s - %s of ADIE failed!\n",
			       __func__, enable ? "Enabling" : "Disabling");
			return err;
	}

	err = seport_platform_enable_mic_bias(enable);
	if (err)
		printk(KERN_INFO "%s - Failed to %s MIC Bias\n",
		       __func__, enable ? "enable" : "disable");

	return err;
}
EXPORT_SYMBOL_GPL(seport_platform_enable_mic_bias_measurement);


/** Private functions */
static int seport_platform_register_mic_bias_client(u8 register_client)
{

	int err = 0;

	if (register_client) {
		if (seport_pf_client_id < 0) {
			seport_pf_client_id = adie_svc_get();
			if(seport_pf_client_id < 0) {
				err = -EFAULT;
				goto out;
			}
		} else {
			printk(KERN_WARNING "%s - Client already registered as: %d\n",
			       __func__, seport_pf_client_id);
			err = -EEXIST;
			goto out;
		}

		if (ADIE_SVC_STATUS_FAILURE == seport_pf_client_id ||
		    ADIE_SVC_STATUS_INUSE == seport_pf_client_id) {
			printk(KERN_CRIT "%s - Failed to register client or client " \
			       "already in use\n", __func__);
			err = -ENOMSG;
		}
	} else {
		if (seport_pf_client_id >= 0) {
			(void)adie_svc_put(seport_pf_client_id);
			seport_pf_client_id = -1;
		} else {
			printk(KERN_INFO "%s - Client already deregistered\n",
			       __func__);
			err = -EEXIST;
		}
	}

out:
	return err;
}

static int seport_platform_read_mean_rid_val(u8 samples)
{
	int i;
	u16 rid;
	int mean_rid = 0;

	/* First value is junk. Throw away. */
	seport_platform_get_button_id(&rid);

	for (i = 0; i < samples; i++) {
		rid = 0;
		seport_platform_get_button_id(&rid);
		mean_rid += (int)rid;
	}

	return mean_rid / (int)samples;
}

static int register_rapi_client(void)
{
	int ret = 0;

	rpc_client = oem_rapi_client_init();
	if (rpc_client == NULL) {
		printk(KERN_INFO
		       "%s : Failed initialize oem rapi client\n",
			__func__);
		ret = -EPERM;
	}
	return ret;
}

static int __devinit seport_platform_probe(struct platform_device *pdev)
{
	if (!pdev)
		goto out;

	seport_conf = pdev->dev.platform_data;

	if (!seport_conf)
		goto out;

	if (seport_conf->initialize(seport_conf))
		goto out;

	if(seport_platform_register_mic_bias_client(1))
		printk(KERN_CRIT "%s - Failed to register ADIE client\n", __func__);

	if(register_rapi_client())
		printk(KERN_WARNING "%s - Failed to register OEMRAPI client. " \
		       "HSSD detection are not available\n", __func__);
	return 0;

out:
	printk(KERN_CRIT "*** %s - Ooops! Major problem occurred. SEport "\
	       "platform exiting.\n", __func__);
	return -1;
}

static int __devexit seport_platform_remove(struct platform_device *pdev)
{
	struct seport_config *cfg;

	if (!pdev)
		return -1;

	cfg = pdev->dev.platform_data;

	if (seport_plug_detect_interrupt_enabled)
		seport_platform_unregister_plug_detect_gpio_callback(NULL);
	if (seport_button_detect_interrupt_enabled)
		seport_platform_unregister_button_gpio_callback();

	gpio_free(cfg->headset_detect_enable_pin);
	gpio_free(cfg->plug_detect_read_pin);
	gpio_free(cfg->button_detect_pin);
	gpio_free(cfg->rid_ping_enable_pin);

#if defined(CONFIG_SEPORT_VIDEO_OUT)
	gpio_free(cfg->video_out_switch);
#endif

	oem_rapi_client_close();

	/* Deregistering ADIE services. */
	seport_platform_register_mic_bias_client(0);

	return 0;
}

struct platform_driver seport_platform_device_driver = {
	.probe   = seport_platform_probe,
	.remove  = __devexit_p(seport_platform_remove),
	.driver  = {
		.name = SEPORT_DRIVER_NAME,
	}
};

static int __init seport_platform_init(void)
{
	return platform_driver_register(&seport_platform_device_driver);
}
module_init(seport_platform_init);

static void __exit seport_platform_exit(void)
{
}
module_exit(seport_platform_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst.x@sonyericsson.com>");
MODULE_DESCRIPTION("Architecture dependant driver for SEMC SEport");
MODULE_LICENSE("GPL");
