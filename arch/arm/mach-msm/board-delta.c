/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Brian Swetland <swetland@google.com>
 * Author: Lars Svensson <lars1.svensson@sonyericsson.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>

#include "devices.h"
#include "socinfo.h"
#include "board-delta.h"
#include "msm-keypad-devices.h"
#include "board-delta-keypad.h"
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif
#ifdef CONFIG_SEMC_POWER_BQ24180
#include <linux/semc/power/semc_power.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_4X
#include <linux/synaptics_i2c_rmi.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_CORE
#include <linux/cyttsp.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SER
#include <linux/cy8ctma300_ser.h>
#endif
#ifdef CONFIG_MSM_SIM_CARD_DETECT
#include <linux/semc/sim_card_detect.h>
#endif
#ifdef CONFIG_SENSORS_AK8973
#include <linux/akm8973.h>
#endif
#include "pm.h"
#include <linux/msm_kgsl.h>

/* LMU driver */
#include <linux/leds-lm3530.h>

#ifdef CONFIG_SEMC_RPC_SERVER_HANDSET
#include <mach/semc_rpc_server_handset.h>
#endif /* CONFIG_SEMC_RPC_SERVER_HANDSET */

/* MDDI includes */
#include "../../../drivers/video/msm/msm_fb_panel.h"
#include "../../../drivers/video/msm/mddihost.h"
#ifdef CONFIG_SEMC_SEPORT_PLATFORM
#include <mach/semc_seport_platform.h>
#endif
#ifdef CONFIG_SEMC_POWER
#include <mach/semc_power_platform.h>
#endif
#ifdef CONFIG_LEDS_MSM_PMIC_FLASHLED
#include <linux/leds-msm_pmic_flashled.h>
#endif
#ifdef CONFIG_SEMC_MSM_PMIC_VIBRATOR
#include <mach/pmic.h>
#include  <linux/semc/msm_pmic_vibrator.h>
#endif
#ifdef CONFIG_LEDS_MSM_PMIC_MISC
#include <mach/pmic.h>
#include <linux/leds-msm_pmic_misc.h>
#endif
#ifdef CONFIG_SEMC_GPIO_EXTR
#include  <linux/semc/semc_gpio_extr.h>
#endif
#define MSM_PMEM_MDP_SIZE	0xC74000
#define MSM_PMEM_ADSP_SIZE	0x8EC000
#ifdef CONFIG_CAPTURE_KERNEL
#include "smd_private.h"
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define MSM_RAM_CONSOLE_START   (0x0DA00000 - MSM_RAM_CONSOLE_SIZE)
#define MSM_RAM_CONSOLE_SIZE    (128 * SZ_1K)
#endif
#define MSM_PMEM_AUDIO_SIZE	0x5B000

/*
 * Framebuffer size calculation. The 128kb extra is an old temp patch in
 * msm_fb.c that needs to be investigated and removed.
 *
 * (pix_w+align)*pix_h*bitdepth*double_buffer + 128kb
 *
 * QVGA: (240+16)*320*2*2 + 128kb = 0x70000
 * HVGA: 320*480*2*2 + 128kb = 0xB6000
 */
#if	defined(CONFIG_MACH_MSM7X27_ROBYN) || \
	defined(CONFIG_MACH_MSM7X27_MIMMI)
#define MSM_FB_SIZE		0x70000
#elif	defined(CONFIG_MACH_MSM7X27_SHAKIRA)
#define MSM_FB_SIZE		0xB6000
#else
#warning "Warning, fb size not configured in board config. Assuming HVGA."
#define MSM_FB_SIZE		0xB6000
#endif

#define MSM_GPU_PHYS_SIZE	SZ_2M
#define PMEM_KERNEL_EBI1_SIZE	0x1C000
#ifdef CONFIG_CAPTURE_KERNEL
#define AMSSCORE_RAM_START	0x0DA00000
#define AMSSCORE_RAM_END	0x0FFFFFFF
#define SMEMCORE_RAM_START	0x00100000
#define SMEMCORE_RAM_END   	0x001FFFFF
#define ADSPCORE_RAMA_START	0xAC000000
#define ADSPCORE_RAMA_END	0xAC03FFFF
#define ADSPCORE_RAMB_START	0xAC200000
#define ADSPCORE_RAMB_END	0xAC23FFFF
#define ADSPCORE_RAMC_START	0xAC400000
#define ADSPCORE_RAMC_END	0xAC43FFFF
#define ADSPCORE_RAMI_START	0xAC800000
#define ADSPCORE_RAMI_END	0xAC83FFFF
#endif

/** Initial configuration for all used SEport GPIOS. Only set them here! */
#ifdef CONFIG_SEMC_SEPORT_PLATFORM
#define PLUG_DET_ENA_PIN 91
#define PLUG_DET_READ_PIN 92
#define BUTTON_DET_READ_PIN 114
#if defined (CONFIG_SEPORT_VIDEO_OUT)
#define VIDEO_OUT_SWITCH -1
#endif
#define RID_PING_ENABLE_PIN 30
#endif

static u32 startup_reason;

#ifdef CONFIG_SEMC_POWER_BQ24180
#define SEMC_POWER_BQ24180_GPIO_INTERRUPT_PIN 124
#endif /* CONFIG_SEMC_POWER_BQ24180 */

#ifdef CONFIG_SENSORS_AK8973
#define AKM8973_GPIO_RESET_PIN	109
#define AKM8973_GPIO_IRQ_PIN	107
#endif /* CONFIG_SENSORS_AK8973 */

#ifdef CONFIG_GPIO_ETS
static struct resource semc_gpios_resources = {
	.start = 0,
	.end   = 164 - 1,
	.flags = IORESOURCE_IRQ,
};

static struct platform_device semc_gpios_device = {
	.name = "semc-atp-gpio",
	.id = -1,
	.num_resources = 1,
	.resource = &semc_gpios_resources,
};
#endif

#ifdef CONFIG_SEMC_SEPORT_PLATFORM
static unsigned seport_config_power_on[] = {
	GPIO_CFG(PLUG_DET_ENA_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(PLUG_DET_READ_PIN, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(BUTTON_DET_READ_PIN, 0, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(RID_PING_ENABLE_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
#if defined (CONFIG_SEPORT_VIDEO_OUT)
	GPIO_CFG(VIDEO_OUT_SWITCH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
#endif
};

int seport_initialize_gpio(struct seport_config *cfg)
{
	int err, pin;

	if(!cfg || -1 == cfg->headset_detect_enable_pin) {
		printk(KERN_ERR "*** %s - Ooops! Invalid inparameter. Aborting!\n",
			__func__);
		return -EIO;
	}

	/* Configuring GPIOS. Must be done before requesting them */
	for (pin = 0; pin < ARRAY_SIZE(seport_config_power_on); pin++) {
		err = gpio_tlmm_config(seport_config_power_on[pin],
				       GPIO_ENABLE);
		if (err) {
			printk(KERN_ERR
			       "%s: gpio_tlmm_config(%#x)=%d\n",
			       __func__, seport_config_power_on[pin], err);
			return -EIO;
		}
	}


	err = gpio_request(cfg->headset_detect_enable_pin,
			   "Seport_plug_detect_enable");
	if (err) {
		printk(KERN_CRIT "Could not allocate headset sensing GPIO "
		       "pin\n");
		goto out;
	}

	err = gpio_request(cfg->plug_detect_read_pin,
			   "Seport_plug_detect_read");
	if (err) {
		printk(KERN_CRIT "Could not allocate headset sensing GPIO "
		       "pin\n");
		goto out;
	}

	err = gpio_request(cfg->rid_ping_enable_pin,
			   "Seport_RID_ping_enable");
	if (err) {
		printk(KERN_CRIT "Could not allocate RID ping GPIO "
		       "pin. Now me nasty ;->\n");
		goto out;
	}

	err = gpio_request(cfg->button_detect_pin,
			   "Seport_button_detect_read");
	if (err) {
		printk(KERN_CRIT "Could not allocate button detect interrupt GPIO "
		       "pin\n");
		goto out;
	}

#if defined(CONFIG_SEPORT_VIDEO_OUT)
	err = gpio_request(cfg->video_out_switch, "Seport_video_out_switch");
	if (err) {
		printk(KERN_CRIT "Could not allocate button detect interrupt GPIO "
		       "pin\n");
	}

#endif
	/* Initial state for RID. Output high disables rid.
	 * When this is done, interrupt pin must be set asw
	 * output, low in order to avoid problems. */
	gpio_set_value(cfg->rid_ping_enable_pin, 1);
	gpio_set_value(cfg->button_detect_pin,  0);

out:
	return err;
}

static struct seport_config seport_cfg = {
	.headset_detect_enable_pin = PLUG_DET_ENA_PIN,
	.plug_detect_read_pin = PLUG_DET_READ_PIN,
	.button_detect_pin = BUTTON_DET_READ_PIN,
	.rid_ping_enable_pin = RID_PING_ENABLE_PIN,
#if defined(CONFIG_SEPORT_VIDEO_OUT)
	.video_out_switch = VIDEOU_OUT_SWITCH,
#endif

	.initialize = &seport_initialize_gpio,
};


static struct platform_device seport_platform_device = {
	.name = SEPORT_DRIVER_NAME,
	.id = -1,
	.dev = {
		.platform_data = &seport_cfg,
	},
};
#endif /* CONFIG_SEMC_SEPORT_PLATFORM */

#ifdef CONFIG_SEMC_RPC_SERVER_HANDSET
/*
 * Add callbacks here. Every defined callback will receive
 * all events. The types are defined in the file
 * semc_rpc_server_handset.h
 */
static handset_cb_array_t semc_handset_callbacks = {
	&seport_platform_vad_callback,
	&keypad_rpc_key_callback,
};

static struct semc_handset_data semc_hs_data = {
	.callbacks = semc_handset_callbacks,
	.num_callbacks = ARRAY_SIZE(semc_handset_callbacks),
};

static struct platform_device semc_rpc_handset_device = {
	.name = SEMC_HANDSET_DRIVER_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_hs_data,
	},
};
#endif /* CONFIG_SEMC_RPC_SERVER_HANDSET */

#ifdef CONFIG_SEMC_POWER
static struct platform_device semc_power_device = {
	.name = SEMC_POWER_PLATFORM_NAME,
};
#endif /* CONFIG_SEMC_POWER */

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_lun_config mass_storage_lun_config[] = {
	{	/*lun#0*/
		.is_cdrom = false,
		.shift_size = 9,
		.can_stall = true,
	},
	{       /*lun#1*/
		.is_cdrom = true,
		.shift_size = 11,
		.can_stall = false,
	},
};

static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = ARRAY_SIZE(mass_storage_lun_config),
	.buf_size       = 16384,
	.vendor         = "SEMC",
	.product        = "Mass storage",
	.release        = 0xffff,
	.lun_conf	= mass_storage_lun_config,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data = &usb_mass_storage_pdata,
	},
};
#endif
#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
#if defined(CONFIG_MACH_MSM7X27_ROBYN)
	{
		/* MSC( + CDROM) */
		.product_id	= 0x3137,
		.functions	= 0xD,
		/* ADB + MSC( + CDROM) */
		.adb_product_id	= 0x2137,
		.adb_functions	= 0xD1,
		/* DIAG + ADB + MODEM + NMEA + MSC( + CDROM) */
		.eng_product_id	= 0x2146,
		.eng_functions	= 0xD7614,
	},
	{	/*  (MSC) */
		.product_id	= 0xE137,
		.functions	= 0x2,
		/* MSC + ADB */
		.adb_product_id	= 0x6137,
		.adb_functions	= 0x12,
		/* MSC + ADB + MODEM + NMEA + DIAG */
		.eng_product_id	= 0x6146,
		.eng_functions	= 0x47612,
	},
#endif
#if defined(CONFIG_MACH_MSM7X27_MIMMI)
	{
		/* MSC( + CDROM) */
		.product_id	= 0x3138,
		.functions	= 0xD,
		/* ADB + MSC( + CDROM) */
		.adb_product_id	= 0x2138,
		.adb_functions	= 0xD1,
		/* DIAG + ADB + MODEM + NMEA + MSC( + CDROM) */
		.eng_product_id	= 0x2146,
		.eng_functions	= 0xD7614,
	},
	{	/*  (MSC) */
		.product_id	= 0xE138,
		.functions	= 0x2,
		/* MSC + ADB */
		.adb_product_id	= 0x6138,
		.adb_functions	= 0x12,
		/* MSC + ADB + MODEM + NMEA + DIAG */
		.eng_product_id	= 0x6146,
		.eng_functions	= 0x47612,
	},
#endif
#if defined(CONFIG_MACH_MSM7X27_SHAKIRA)
	{
		/* MSC( + CDROM) */
		.product_id	= 0x3149,
		.functions	= 0xD,
		/* ADB + MSC( + CDROM) */
		.adb_product_id	= 0x2149,
		.adb_functions	= 0xD1,
		/* DIAG + ADB + MODEM + NMEA + MSC( + CDROM) */
		.eng_product_id	= 0x2146,
		.eng_functions	= 0xD7614,
	},
	{	/*  (MSC) */
		.product_id	= 0xE149,
		.functions	= 0x2,
		/* MSC + ADB */
		.adb_product_id	= 0x6149,
		.adb_functions	= 0x12,
		/* MSC + ADB + MODEM + NMEA + DIAG */
		.eng_product_id	= 0x6146,
		.eng_functions	= 0x47612,
	},
#endif
	{
		/* ADB+MSC+ECM */
		.product_id	= 0x3146,
		.functions	= 0x821,
		.adb_product_id	= 0x3146,
		.adb_functions	= 0x821,
	},
};
static struct usb_mass_storage_lun_config msc_lun_config = {
	.is_cdrom	= false,
	.shift_size	= 9,
	.can_stall	= true,
	.vendor		= "SEMC",
	.product	= "Mass Storage",
	.release	= 0x0001,
};
static struct usb_mass_storage_lun_config cdrom_lun_config = {
	.is_cdrom	= true,
	.shift_size	= 11,
	.can_stall	= false,
	.vendor		= "SEMC",
	.product	= "CD-ROM",
	.release	= 0x0001,
};
static struct usb_mass_storage_lun_config msc_cdrom_lun_config[] = {
	{
		.is_cdrom	= false,
		.shift_size	= 9,
		.can_stall	= true,
		.vendor		= "SEMC",
		.product	= "Mass Storage",
		.release	= 0x0001,
	},
	{
		.is_cdrom	= true,
		.shift_size	= 11,
		.can_stall	= false,
		.vendor		= "SEMC",
		.product	= "CD-ROM",
		.release	= 0x0001,
	},
};
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0FCE,
	.version		= 0x0100,
	.serial_number		= "1234567890ABCDEF",
	.compositions		= usb_func_composition,
	.num_compositions	= ARRAY_SIZE(usb_func_composition),
	.product_name		= "SEMC HSUSB Device",
	.manufacturer_name	= "SEMC",
	.nluns			= 1,
	.cdrom_lun_conf		= &cdrom_lun_config,
	.msc_lun_conf		= &msc_lun_config,
	.msc_cdrom_lun_conf	= msc_cdrom_lun_config,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
	.platform_data = &android_usb_pdata,
	},
};
#endif

#if defined(CONFIG_LEDS_MSM_PMIC_FLASHLED)
static 	struct vreg *pvreg_boost;

static void __init msm_pmic_vreg_boost_init(void)
{
	printk(KERN_INFO "%s:\n", __func__);
	pvreg_boost = vreg_get(NULL, "boost");
	if (!pvreg_boost)
		printk(KERN_ERR "%s: vreg_get failed (%d).\n",
			__func__, -ENODEV);
}

static int msm_pmic_set_boost_voltage(struct platform_device *pdev, u16 mv)
{
	int rc;

	if (!pvreg_boost) {
		printk(KERN_ERR "%s: no vreg_boost ref.\n", __func__);
		return -ENODEV;
	}
	rc = vreg_set_level(pvreg_boost, mv);
	if (rc)
		printk(KERN_ERR "%s: %s vreg_set_level failed (%d).\n",
			__func__, pdev->name, rc);
	return rc;
}

static int msm_pmic_enable_boost_voltage(struct platform_device *pdev,
					int enable)
{
	int rc;

	if (!pvreg_boost) {
		printk(KERN_ERR "%s: no vreg_boost ref.\n", __func__);
		return -ENODEV;
	}
	rc = enable ? vreg_enable(pvreg_boost) : vreg_disable(pvreg_boost);
	if (rc)
		printk(KERN_ERR "%s: %s failed (%d).\n",
			__func__, pdev->name, rc);
	return rc;
}

static struct msm_pmic_flashled_platform_data flashled_platform_data = {
	.set_boost_voltage = msm_pmic_set_boost_voltage,
	.enable_boost_voltage = msm_pmic_enable_boost_voltage,
	.def_camflash_boost_v = 5000,
	.def_spotlight_boost_v = 4700,
	.def_camflash_current = 480,
	.def_spotlight_curtent = 80,
	.sync_enable = 1,
	.enable_time_limit = 1000,
	.sync_mpp = PM_MPP_18,
	.sync_level = PM_MPP__DLOGIC__LVL_MSME,
	.sync_polarity = FLASH_LED_POL__ACTIVE_HIGH,
	.flash_ctl_dbus = FLASH_LED_MODE__DBUS1,
	.sync_in_dbus = PM_MPP__DLOGIC_IN__DBUS1,
};
static struct platform_device flash_led_device = {
	.name = "msm_pmic_flash_led",
	.id = -1,
	.dev = {
		.platform_data = &flashled_platform_data,
	},
};
#endif

#if defined(CONFIG_LEDS_MSM_PMIC_MISC_KEYPAD)
static int pmic_led_set_level(struct msm_pmic_misc_led_platform_data *pdata,
							  u8 brightness)
{
	if (brightness) {
		brightness = brightness / 16 + 1;
		if (brightness > 15)
			brightness = 15;
	}
	return  pmic_set_led_intensity(pdata->id.led_id, brightness);
}

static int pmic_led_current_2_brightness(int current_ma)
{
	if (current_ma < 10)
		return 0;
	return  (current_ma / 10 ) * 16 - 1;
}
#endif

#ifdef CONFIG_LEDS_MSM_PMIC_MISC_KEYPAD
static struct msm_pmic_misc_led_platform_data msm_pmic_kbd_led_pdata = {
	.currnet_to_brightness = pmic_led_current_2_brightness,
	.set_brightness = pmic_led_set_level,
	.id = { .led_id = LED_KEYPAD, },
	.name = "keyboard_led",
};
static struct platform_device keypad_led_device = {
	.name = MSM_PMIC_MISC_DRV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &msm_pmic_kbd_led_pdata,
	},
};
#endif

#if defined(CONFIG_MSM_PMIC_LIGHTSENSOR)
static struct platform_device lightsensor_device = {
	.name = "pmic-lightsensor",
	.id = -1,
};
#endif
#if defined(CONFIG_DELTA_PROXIMITY_SENSOR)
static struct platform_device delta_proximity_device = {
	.name = "proximity-sensor",
	.id = -1,
};
#endif
#ifdef CONFIG_MSM_SIM_CARD_DETECT
static struct sim_detect_platform_data simc_platform_data = {
	.gpio = 18
};
static struct platform_device simcard_detect_device = {
	.name = "simcard-sensor",
	.id = -1,
	.dev = {
		.platform_data = &simc_platform_data,
	},
};
#endif

#ifdef CONFIG_SEMC_MSM_PMIC_VIBRATOR
static int msm7227_platform_set_vib_voltage(u16 volt_mv)
{
	int rc = pmic_vib_mot_set_volt(volt_mv);

	if (rc)
		printk(KERN_ERR "%s: Failed to set motor voltage\n", __func__);
	return rc;
}

static int msm7227_platform_init_vib_hw(void)
{
	int rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);

	if (rc) {
		printk(KERN_ERR "%s: Failed to set pin mode\n", __func__);
		return rc;
	}
	return pmic_vib_mot_set_volt(0);
}

static struct msm_pmic_vibrator_platform_data vibrator_platform_data = {
	.min_voltage = 1200,
	.max_voltage = 3100,
	.off_voltage = 0,
	.default_voltage = 3100,
	.mimimal_on_time = 10,
	.platform_set_vib_voltage = msm7227_platform_set_vib_voltage,
	.platform_init_vib_hw = msm7227_platform_init_vib_hw,
};
static struct platform_device vibrator_device = {
	.name = "msm_pmic_vibrator",
	.id = -1,
	.dev = {
		.platform_data = &vibrator_platform_data,
	},
};
#endif

#ifdef CONFIG_SEMC_GPIO_EXTR
static u8 semc_gpio_extr_list[] = {82, 129};

static int semc_gpio_extr_set(u8 gpio, u8 val)
{
	if (gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_OUTPUT,
		GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE))
		return -1;
	gpio_set_value(gpio, val);
	return 0;
}

static int semc_gpio_extr_get(u8 gpio)
{
	if (gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_INPUT,
		GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE))
		return -1;
	return gpio_get_value(gpio);
}

static int semc_gpio_extr_init(void)
{
	int i = 0;
	for (; i < sizeof(semc_gpio_extr_list); i++)
		gpio_tlmm_config(GPIO_CFG(semc_gpio_extr_list[i], 0,
			GPIO_OUTPUT,GPIO_NO_PULL, GPIO_2MA), GPIO_DISABLE);
	return 0;
}

static struct semc_gpio_extr_platform_data semc_gpio_extr_pf_data = {
	.gpio_list = semc_gpio_extr_list,
	.list_size = ARRAY_SIZE(semc_gpio_extr_list),
	.init_gpios = semc_gpio_extr_init,
	.get_gpio = semc_gpio_extr_get,
	.set_gpio = semc_gpio_extr_set
};

static struct platform_device semc_gpio_extr_device = {
	.name = SEMC_GPIO_EXTR_DRV_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_gpio_extr_pf_data,
	},
};
#endif

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
#ifdef CONFIG_USB_FUNCTION_GG
	{"gg", 6},
#endif
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
#if defined(CONFIG_MACH_MSM7X27_ROBYN)
	{	/*  (ms) */
		.product_id         = 0xE137,
		.functions	    = 0x10, /* 10000 */
	},

	{	/* (ms+adb) */
		.product_id         = 0xD137,
		.functions	    = 0x12, /* 10010 */
	},
#endif
#if defined(CONFIG_MACH_MSM7X27_MIMMI)
	{	/*  (ms) */
		.product_id         = 0xE138,
		.functions	    = 0x10, /* 10000 */
	},

	{	/* (ms+adb) */
		.product_id         = 0xD138,
		.functions	    = 0x12, /* 10010 */
	},
#endif
#if defined(CONFIG_MACH_MSM7X27_SHAKIRA)
        {       /*  (ms) */
                .product_id         = 0xE149,
                .functions          = 0x10, /* 10000 */
        },

        {       /* (ms+adb) */
                .product_id         = 0xD149,
                .functions          = 0x12, /* 10010 */
        },
#endif
	{	/* (ms+nmea+modem+diag) */
		.product_id         = 0x0146,
		.functions	    = 0x1D, /* 11101 */
	},

	{	/* (ms+nmea+modem+adb+diag) */
		.product_id         = 0x2146,
		.functions	    = 0x1F, /* 11111 */
	},

	{	/* (eth+ms+adb) */
		.product_id         = 0x3146,
		.functions	    = 0x32, /* 110010 */
	},

	{	/* (eth+ms+nmea+modem+diag) */
		.product_id         = 0xD146,
		.functions	    = 0x3D, /* 111101 */
	},

	{	/* (eth+ms+nmea+modem+adb+diag) */
		.product_id         = 0xE146,
		.functions	    = 0x3F, /* 111111 */
	},
#ifdef CONFIG_USB_FUNCTION_GG
	{
		.product_id         = 0xADDE,
		.functions	    = 0x40, /* 1000010 */
	},
#endif
};
#endif

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.vendor_id          = 0x0FCE,
	.product_name       = "Sony Ericsson USB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Sony Ericsson",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,
#endif
};

#if 0
extern void usb_function_enable(const char*, int);

static int __init delta_init_usb(void)
{
	if (startup_reason & STARTUP_REASON_TYPE_APPROVAL) {
		printk(KERN_INFO "GTA mode: enabling cdc-ether\n");
		usb_function_enable("ethernet", 1);
	}

	return 0;
}
late_initcall(delta_init_usb);
#endif

static int hsusb_rpc_connect(int connect)
{
#ifdef CONFIG_CAPTURE_KERNEL
	return 0;
#else
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
#endif
}

static int msm_hsusb_rpc_phy_reset(void __iomem *addr)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect             = hsusb_rpc_connect,
	.phy_reset               = msm_hsusb_rpc_phy_reset,
	.pmic_notif_init         = msm_pm_app_rpc_init,
	.pmic_notif_deinit       = msm_pm_app_rpc_deinit,
	.pmic_register_vbus_sn   = msm_pm_app_register_vbus_sn,
	.pmic_unregister_vbus_sn = msm_pm_app_unregister_vbus_sn,
	.pmic_enable_ldo         = msm_pm_app_enable_usb_ldo,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET_OP, 26),
	SND(HANDSET_CL, 27),
	SND(FARFIELD_OP, 28),
	SND(FARFIELD_CL, 29),
	SND(FARFIELD_OP_LINEIN, 30),
	SND(FARFIELD_CL_LINEIN , 31),
	SND(FARFIELD_OP_FM, 32),
	SND(FARFIELD_CL_FM, 33),
	SND(FARFIELD_OP_FM_LINEIN, 34),
	SND(FARFIELD_CL_FM_LINEIN, 35),
        SND(FARFIELD_HEADPHONE, 36),
	SND(FARFIELD_HEADSET, 37),
	SND(HEADSET, 38),
	SND(HEADSET_FM, 39),
	SND(POW_HEADSET, 40),
	SND(POW_HEADSET_FM, 41),
	SND(HEADPHONE , 42),
	SND(HEADPHONE_FM, 43),
	SND(LINEOUT_FIXED, 44),
	SND(LINEOUT_FIXED_FM, 45),
        SND(DHVH, 46),
	SND(DHVH_FM, 47),
	SND(BT, 48),
	SND(BT_NREC, 49),
	SND(BT_LEGACY , 50),
	SND(BT_HBH_6XX, 51),
	SND(TTY, 52),
	SND(HCO, 53),
	SND(VCO, 54),
	SND(HAC, 55),
	SND(FARFIELD_LINEOUT, 56),
	SND(HANDSET_CL_SKT, 57),
	SND(HEADSET_LOW_ST, 58),
	SND(POW_HEADSET_LOW_ST, 59),
	SND(HEADPHONE_LOW_ST, 60),
	SND(CURRENT, 62),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 4),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 4),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 4),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#ifdef CONFIG_CAPTURE_KERNEL
static struct resource kdump_amsscoredump_resources[] = {
	{
		.name   = "amsscore0",
		.start  = AMSSCORE_RAM_START,
		.end    = AMSSCORE_RAM_END,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "smemcore0",
		.start  = SMEMCORE_RAM_START,
		.end    = SMEMCORE_RAM_END,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "adspcore0",
		.start  = ADSPCORE_RAMA_START,
		.end    = ADSPCORE_RAMA_END,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "adspcore1",
		.start  = ADSPCORE_RAMB_START,
		.end    = ADSPCORE_RAMB_END,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "adspcore2",
		.start  = ADSPCORE_RAMC_START,
		.end    = ADSPCORE_RAMC_END,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "adspcore3",
		.start  = ADSPCORE_RAMI_START,
		.end    = ADSPCORE_RAMI_END,
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device kdump_amsscoredump_device = {
	.name		= "amsscoredump",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(kdump_amsscoredump_resources),
	.resource	= kdump_amsscoredump_resources,
};
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_MSM_RPCSERVER_HANDSET
static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = "7k_handset",
	},
};
#endif /* CONFIG_MSM_RPCSERVER_HANDSET */

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x27_ffa() || machine_is_msm7x27_ffa()) {
		if (!strcmp(name, "lcdc_gordon_vga"))
			ret = 0;
		else
			ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_EN,
};

static unsigned bt_config_power_on[] = {
	GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* CTS */
	GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* Rx */
	GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* Tx */
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(90, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* BT Enable */
};
static unsigned bt_config_power_off[] = {
	GPIO_CFG(43, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* CTS */
	GPIO_CFG(45, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* Rx */
	GPIO_CFG(46, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* Tx */
	GPIO_CFG(68, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(90, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* BT Enable */
};

static int bluetooth_power(int on)
{
   int pin, rc;
   /* BT power controlled by gpio 90 in Robyn */
	if (on) {
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}

      /* Enable BT */
      gpio_set_value(90, 1);
	} else {
      /* Disable BT */
      gpio_set_value(90, 0);

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}
	}
	return 0;
}

static void __init bt_power_init(void)
{
	if (gpio_request(90, "bt_en"))
		printk(KERN_ERR "failed to request gpio bt_en\n");

	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif
#ifdef CONFIG_ARCH_MSM7X27
static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "kgsl_phys_memory",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_platform_data kgsl_pdata;

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};
#endif

#ifdef CONFIG_SEMC_POWER_BQ24180
static int bq24180_gpio_init(void)
{
	int ret = 0;

	/* Request GPIO */
	ret = gpio_request(SEMC_POWER_BQ24180_GPIO_INTERRUPT_PIN,
			   "charge state");
	if (ret) {
		printk(KERN_ERR "bq24180: GPIO request error\n");
		return ret;
	}

	/* Set GPIO to input */
	ret = gpio_direction_input(SEMC_POWER_BQ24180_GPIO_INTERRUPT_PIN);
	if (ret) {
		printk(KERN_ERR "bq24180: GPIO direction error\n");
		return ret;
	}

	return 0;
}

static struct semc_power_platform_data bq24180_platform_data = {
	.gpio_init = bq24180_gpio_init
};
#endif /* CONFIG_SEMC_POWER_BQ24180 */

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_4X
static int synaptics_power(int on)
{
	if (on == 1) {
		if (gpio_request(112, "touch_int")) {
			printk(KERN_ERR "%s: Failed to request gpio touch_int\n",
			__func__);
			return -EIO;
		}
		if (gpio_tlmm_config(
			GPIO_CFG(112, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),
			GPIO_ENABLE)) {
			printk(KERN_ERR "%s: gpio_tlmm_config failed\n", __func__);
			return -EIO;
		}
		if (gpio_configure(112, IRQF_TRIGGER_LOW))	{
			printk(KERN_ERR "%s: failed to configure gpio\n", __func__);
			return -EIO;
		}
	}
	else if (on == 0) {
		gpio_free(112);
	}
	return 0;
}

static struct synaptics_i2c_rmi_platform_data synaptics_platform_data = {
	.flags = SYNAPTICS_FLIP_Y,
	.power = synaptics_power,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SER
#define CY8_XRES_GPIO	126

static int cy8ctma300_ser_xres(void)
{
	int polarity;

	if (gpio_request(CY8_XRES_GPIO, "cy8ctma300_ser_xres")) {
		printk(KERN_ERR "%s: Failed to request XRES gpio\n",
		__func__);
		return -EIO;
	}

	polarity = (gpio_get_value(CY8_XRES_GPIO) & 0x01);
	printk(KERN_INFO "%s: %d\n",__func__, polarity);

	if (gpio_tlmm_config(
		GPIO_CFG(CY8_XRES_GPIO, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		GPIO_ENABLE)) {
		printk(KERN_ERR "%s: gpio_tlmm_config failed\n", __func__);
		return -EIO;
	}

	/* Toggle the pin according to its default polarity */
	gpio_set_value(CY8_XRES_GPIO, (polarity^1));
	msleep(20);
	gpio_set_value(CY8_XRES_GPIO, polarity);
	gpio_free(CY8_XRES_GPIO);
	msleep(40);
	return 0;
}

static struct cy8ctma300_ser_platform_data cy8ctma300_ser_pf_data = {
	.xres = cy8ctma300_ser_xres,
};

static struct platform_device cy8ctma300_ser_device = {
	.name = CY8CTMA300_SER_DEV,
	.id = -1,
	.dev = {
		.platform_data = &cy8ctma300_ser_pf_data,
	},
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
#define CYTTSP_IRQ_GPIO 112
static int cyttsp_wakeup(void)
{
	int ret;
	ret = gpio_direction_output(CYTTSP_IRQ_GPIO, 0);
	if (ret) {
		printk(KERN_ERR "%s: Failed to request gpio_direction_output\n",
		__func__);
                return ret;
	}
	gpio_set_value(CYTTSP_IRQ_GPIO, 1);
	ret = gpio_direction_input(CYTTSP_IRQ_GPIO);
	if (ret) {
		printk(KERN_ERR "%s: Failed to request gpio_direction_input\n",
		__func__);
		return ret;
	}
	msleep(3);
	return 0;
}

static int cyttsp_init(int on)
{
	int ret;

	if (on) {
		ret = gpio_request(CYTTSP_IRQ_GPIO, "CYTTSP IRQ GPIO");
		if (ret) {
			printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			       __func__, CYTTSP_IRQ_GPIO);
			return ret;
		}
		gpio_direction_input(CYTTSP_IRQ_GPIO);
	} else {
		gpio_free(CYTTSP_IRQ_GPIO);
	}
	return 0;
}

static struct cyttsp_platform_data cypress_i2c_touch_data = {
	.wakeup = cyttsp_wakeup,
	.init = cyttsp_init,
	.mt_sync = input_mt_sync,
	/* TODO: maxx and maxy values should be retrieved from the firmware */
	.maxx = 319,
	.maxy = 479,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = 1,
	.use_mt = 0,
	.use_trk_id = 0,
	.use_hndshk = 0,
	.use_timer = 0,
	.use_sleep = 1,
	.use_gestures = 0,
	.use_load_file = 0,
	.use_force_fw_update = 0,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP_NONE | CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_SPI_NAME,
	.irq_gpio = CYTTSP_IRQ_GPIO,
};

#endif /* CONFIG_TOUCHSCREEN_CYTTSP_CORE */

#ifdef CONFIG_SENSORS_AK8973
static int ak8973_gpio_config(int enable)
{
	if (enable) {
		if (gpio_request(AKM8973_GPIO_RESET_PIN, "akm8973_xres")) {
			printk(KERN_ERR "%s: gpio_req xres"
				" - Fail!", __func__);
			return -EIO;
		}
		if (gpio_tlmm_config(GPIO_CFG(AKM8973_GPIO_RESET_PIN, 0,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE)) {
			printk(KERN_ERR "%s: gpio_tlmm_conf xres"
				" - Fail!", __func__);
			goto ak8973_gpio_fail_0;
		}
		/* Reset is active low, so just a precaution setting. */
		gpio_set_value(AKM8973_GPIO_RESET_PIN, 1);

		if (gpio_request(AKM8973_GPIO_IRQ_PIN, "akm8973_irq")) {
			printk(KERN_ERR "%s: gpio_req irq\n"
				" - Fail!", __func__);
			goto ak8973_gpio_fail_0;
		}
		if (gpio_tlmm_config(GPIO_CFG(AKM8973_GPIO_IRQ_PIN, 0,
			GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE)) {
			printk(KERN_ERR "%s: gpio_tlmm_conf irq\n"
				" - Fail!", __func__);
			goto ak8973_gpio_fail_1;
		}

	} else {
		gpio_free(AKM8973_GPIO_RESET_PIN);
		gpio_free(AKM8973_GPIO_IRQ_PIN);
	}
	return 0;

ak8973_gpio_fail_1:
	gpio_free(AKM8973_GPIO_IRQ_PIN);
ak8973_gpio_fail_0:
	gpio_free(AKM8973_GPIO_RESET_PIN);
	return -EIO;
}

static int ak8973_xres(void)
{
	gpio_set_value(AKM8973_GPIO_RESET_PIN, 0);
	msleep(10);
	gpio_set_value(AKM8973_GPIO_RESET_PIN, 1);
	msleep(20);
	return 0;
}

static struct akm8973_i2c_platform_data akm8973_platform_data = {
	.gpio_config = ak8973_gpio_config,
	.xres = ak8973_xres
};
#endif /* CONFIG_SENSORS_AK8973 */

#ifdef CONFIG_LEDS_LM3530
int lm3530_power_on(int enable)
{
	int err;
	struct vreg *pvreg;

	pvreg = vreg_get(NULL, "gp3");
	if (!pvreg)
		goto err_exit;
	if (enable) {
		vreg_set_level(pvreg, 3000);
		err = vreg_enable(pvreg);
	} else
		err = vreg_disable(pvreg);
	if (err)
		printk(KERN_ERR "%s Unable to set vreg.\n", __func__);
	return err;
err_exit:
	printk(KERN_ERR "%s Unable to allocate vreg.\n", __func__);
	return -ENODEV;
}

static struct lm3530_platform_data lm3530_data = {
	.power_up = lm3530_power_on,
	.zone_irq_gpio = 121,
	.hw_enable_gpio = 125,
	.pwm_polarity = 1,  /* high PWM level is activwe */
	.als_input = ALS_INPUT_2,
};
#endif

static struct i2c_board_info i2c_devices[] = {
#ifdef CONFIG_DLT002_CAMERA
	{
		I2C_BOARD_INFO("dlt002_camera", 0x1A),
	},
#endif
#ifdef CONFIG_DLT001_CAMERA
	{
		I2C_BOARD_INFO("dlt001_camera", 0x1A),
	},
#endif
#ifdef CONFIG_LEDS_LM3530
	{
		I2C_BOARD_INFO("lm3530", 0x36),
		.platform_data = &lm3530_data
	},
#endif
#ifdef CONFIG_SEMC_POWER_BQ24180
	{
		I2C_BOARD_INFO("bq24180", 0x6B),
		.platform_data = &bq24180_platform_data,
		.irq = MSM_GPIO_TO_INT(SEMC_POWER_BQ24180_GPIO_INTERRUPT_PIN)
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_4X
	{
		I2C_BOARD_INFO("synaptics-rmi-ts", 0x20),
		.platform_data = &synaptics_platform_data,
		.irq = MSM_GPIO_TO_INT(112)
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_CYTTSP_I2C
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cypress_i2c_touch_data,
	},
#endif
#ifdef CONFIG_SENSORS_BMA150
	{
		I2C_BOARD_INFO("bma150", 0x38),
	},
#endif
#ifdef CONFIG_SENSORS_AK8973
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.platform_data = &akm8973_platform_data,
	},
#endif
#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = MSM_GPIO_TO_INT(107)
	},
#endif
};

#if defined CONFIG_DLT001_CAMERA || defined CONFIG_DLT002_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_STB */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_TRIG */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* MCLK */
	GPIO_CFG(117, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* VCAMSD12_EN */
};

static uint32_t camera_on_gpio_table[] = {
   /* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA), /* CAM_STB */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* CAM_TRIG */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
	GPIO_CFG(117, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* VCAMSD12_EN */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};
#endif

#if defined(CONFIG_DLT001_CAMERA) || defined(CONFIG_DLT002_CAMERA)
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.low_current  = 30,
	._fsrc.pmic_src.high_current = 100,
};
#endif

#ifdef CONFIG_DLT001_CAMERA
static struct msm_camera_sensor_flash_data flash_dlt001 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_dlt001_data = {
	.sensor_name  = "dlt001_camera",
	.sensor_reset   = 89,
	.sensor_pwd	  = 0,
	.vcm_pwd      = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_dlt001,
#ifdef CONFIG_MSM_CAMERA
	.standby    = {.type = MSM_CAMERA_SENSOR_PWR_GPIO,
		.resource.number = 0,},
	.vcam_sd12  = {.type = MSM_CAMERA_SENSOR_PWR_GPIO,
		.resource.number = 117,},		/* 1,2V Core */
	.vcam_sa28  = {.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "rfrx2",},		/* 2,8V Sensor Analog */
	.vcam_io    = {.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name   = "gp4",},		/* 2,6V I/O */
	.vcam_af30  = {.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "rftx",},		/* 3,0V AF */
#endif
};

static struct platform_device msm_camera_sensor_dlt001 = {
	.name      = "msm_camera_dlt001",
	.dev       = {
		.platform_data = &msm_camera_sensor_dlt001_data,
	},
};
#endif

#ifdef CONFIG_DLT002_CAMERA
static struct msm_camera_sensor_flash_data flash_dlt002 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_dlt002_data = {
	.sensor_name  = "dlt002_camera",
	.sensor_reset   = 89,
	.sensor_pwd	  = 0,
	.vcm_pwd      = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_dlt002,
#ifdef CONFIG_MSM_CAMERA
	.standby    = {.type = MSM_CAMERA_SENSOR_PWR_GPIO,
		.resource.number = 0,},
	.vcam_sd12  = {.type = MSM_CAMERA_SENSOR_PWR_GPIO,
		.resource.number = 117,},		/* 1,2V Core */
	.vcam_sa28  = {.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "rfrx2",},		/* 2,8V Sensor Analog */
	.vcam_io    = {.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name   = "gp4",},		/* 2,6V I/O */
	.vcam_af30  = {.type = MSM_CAMERA_SENSOR_PWR_VREG,
		.resource.name = "rftx",},		/* 3,0V AF */
#endif
};

static struct platform_device msm_camera_sensor_dlt002 = {
	.name      = "msm_camera_dlt002",
	.dev       = {
		.platform_data = &msm_camera_sensor_dlt002_data,
	},
};
#endif

#ifdef CONFIG_FB_MSM_MDDI
/* Platform dependent display driver functions */

/* GPIO */
#define GPIO_MSM_MDDI_XRES	88

/* Voltage Regulators */
static struct vreg *vreg_vlcd;
static struct vreg *vreg_vlcd_io;

/* Generic LCD Regulators On function for SEMC Delta displays */
static void semc_delta_lcd_regulators_on(void)
{
	int rc = 0;

	/* VLCD_IO on
	VLCD_IO <-> Vreg_gp2 on PM7540 (QC PowerManagement ic) */
	vreg_vlcd_io = vreg_get(NULL, "gp2");
	if (IS_ERR(vreg_vlcd_io)) {
		printk(KERN_DEBUG
		"%s vreg_get(gp2) err.\n", __func__);
		return;
	}

	rc = vreg_enable(vreg_vlcd_io);	/* PM7540 GP2 ON */
	if (rc) {
		printk(KERN_DEBUG
		"%s vreg_enable(gp2) err. rc=%d\n", __func__, rc);
		return;
	}

	/* VLCD on
	VLCD <-> Vreg_mmc on PM7540 (QC PowerManagement ic) */
	vreg_vlcd = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_vlcd)) {
		printk(KERN_DEBUG
		"%s vreg_get(mmc) err.\n", __func__);
		return;
	}

	rc = vreg_enable(vreg_vlcd);	/* PM7540 MMC ON */
	if (rc) {
		printk(KERN_DEBUG
		"%s vreg_enable(mmc) err. rc=%d\n", __func__, rc);
		return;
	}
}

/* Generic Power On function for SEMC Delta displays */
static void semc_delta_lcd_power_on(u8 delay1, u8 delay2, u8 delay3)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_MSM_MDDI_XRES,
				  0,
				  GPIO_OUTPUT,
				  GPIO_NO_PULL,
				  GPIO_2MA),
				  GPIO_ENABLE);
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0); /* Hitachi qvga spec */
	semc_delta_lcd_regulators_on();
	mdelay(delay1);
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0); /* Samsung qvga spec */
	mdelay(delay2);
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(delay3);
}

#ifdef CONFIG_FB_MSM_SEMC_LCD_BACKLIGHT_CONTROL
static void semc_delta_lcd_backlight_ctrl(bool resume)
{
	if (lm3530_data.set_backlight && lm3530_data.chip_data &&
						lm3530_data.get_backlight) {
		if (resume) {
			int res;
			u8 brightness;

			res = lm3530_data.get_backlight(lm3530_data.chip_data,
								&brightness);
			if (!res)
				lm3530_data.set_backlight(lm3530_data.chip_data,
								brightness);
		} else {
			lm3530_data.set_backlight(lm3530_data.chip_data, 0);
		}
	}
}
#endif

#ifdef CONFIG_FB_MSM_MDDI_HITACHI_QVGA_LCD

/* Display resolution */
#define HITACHI_QVGA_PANEL_XRES	240
#define HITACHI_QVGA_PANEL_YRES	320

static void hitachi_qvga_lcd_power_on(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	semc_delta_lcd_regulators_on();
	mdelay(1); /* spec: > 10 us */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(6); /* spec: > 5 ms */
}

static void hitachi_qvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	vreg_disable(vreg_vlcd);
	vreg_disable(vreg_vlcd_io);
}

static void hitachi_qvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(1); /* spec: > 10 us */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(6); /* spec: > 5 ms */
}

/* SEMC Delta display specific struct */
static struct panel_data_ext hitachi_qvga_panel_ext = {
	.power_on = hitachi_qvga_lcd_power_on,
	.power_off = hitachi_qvga_lcd_power_off,
	.exit_deep_standby = hitachi_qvga_lcd_exit_deep_standby,
#ifdef CONFIG_FB_MSM_SEMC_LCD_BACKLIGHT_CONTROL
	.backlight_ctrl = semc_delta_lcd_backlight_ctrl,
#endif
};

static struct msm_fb_panel_data hitachi_qvga_panel_data;

static struct platform_device mddi_hitachi_qvga_display_device = {
	.name = "mddi_hitachi_qvga",
	.id = -1,
};

static void __init msm_mddi_hitachi_qvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &hitachi_qvga_panel_data;

	printk(KERN_DEBUG "%s \n", __func__);

	panel_data->panel_info.xres = HITACHI_QVGA_PANEL_XRES;
	panel_data->panel_info.yres = HITACHI_QVGA_PANEL_YRES;
	panel_data->panel_info.type = MDDI_PANEL;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 16;
	panel_data->panel_info.clk_rate = 43000000;
	panel_data->panel_info.clk_min = 40000000;
	panel_data->panel_info.clk_max = 45000000;
	panel_data->panel_info.fb_num = 2;

	panel_data->panel_info.mddi.vdopkt = 0x0023;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	panel_data->panel_info.lcd.refx100 = 8500;
	panel_data->panel_info.lcd.v_back_porch = 2;
	panel_data->panel_info.lcd.v_front_porch = 14;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;

	panel_data->panel_ext = &hitachi_qvga_panel_ext;

	mddi_hitachi_qvga_display_device.dev.platform_data =
						&hitachi_qvga_panel_data;

	platform_device_register(&mddi_hitachi_qvga_display_device);
};
#endif /* CONFIG_FB_MSM_MDDI_HITACHI_QVGA_LCD */

#ifdef CONFIG_FB_MSM_MDDI_SAMSUNG_QVGA_LCD

/* Display resolution */
#define SAMSUNG_QVGA_PANEL_XRES	240
#define SAMSUNG_QVGA_PANEL_YRES	320

static void samsung_qvga_lcd_power_on(void)
{
	semc_delta_lcd_regulators_on();
	mdelay(11); /* spec: > 10 ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(11); /* spec: > 10 ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(11); /* spec: > 10 ms */
};

static void samsung_qvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	vreg_disable(vreg_vlcd);
	vreg_disable(vreg_vlcd_io);
};

static struct panel_data_ext samsung_qvga_panel_ext = {
	.power_on = samsung_qvga_lcd_power_on,
	.power_off = samsung_qvga_lcd_power_off,
#ifdef CONFIG_FB_MSM_SEMC_LCD_BACKLIGHT_CONTROL
	.backlight_ctrl = semc_delta_lcd_backlight_ctrl,
#endif
};

static struct msm_fb_panel_data samsung_qvga_panel_data;

static struct platform_device mddi_samsung_qvga_display_device = {
	.name = "mddi_samsung_qvga",
	.id = -1,
};

static void __init msm_mddi_samsung_qvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &samsung_qvga_panel_data;

	printk(KERN_DEBUG "%s \n", __func__);

	panel_data->panel_info.xres = SAMSUNG_QVGA_PANEL_XRES;
	panel_data->panel_info.yres = SAMSUNG_QVGA_PANEL_YRES;
	panel_data->panel_info.type = MDDI_PANEL;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 16;
	panel_data->panel_info.clk_rate = 43000000;
	panel_data->panel_info.clk_min = 40000000;
	panel_data->panel_info.clk_max = 45000000;
	panel_data->panel_info.fb_num = 2;

	panel_data->panel_info.mddi.vdopkt = 0x0023;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	panel_data->panel_info.lcd.refx100 = 7300;
	panel_data->panel_info.lcd.v_back_porch = 2;
	panel_data->panel_info.lcd.v_front_porch = 14;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;

	panel_data->panel_ext = &samsung_qvga_panel_ext;

	mddi_samsung_qvga_display_device.dev.platform_data =
						&samsung_qvga_panel_data;

	platform_device_register(&mddi_samsung_qvga_display_device);
};
#endif /* CONFIG_FB_MSM_MDDI_SAMSUNG_QVGA_LCD */

#ifdef CONFIG_FB_MSM_MDDI_TOSHIBA_HVGA_LCD

/* Display resolution */
#define TOSHIBA_HVGA_PANEL_XRES	320
#define TOSHIBA_HVGA_PANEL_YRES	480

static void toshiba_hvga_lcd_power_on(void)
{
	semc_delta_lcd_regulators_on();
	mdelay(11); /* spec: > 10 ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(2);  /* spec: > 1 ms */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(21); /* spec: > 20 ms */
};

static void toshiba_hvga_lcd_power_off(void)
{

	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(50); /* spec: > 4 frames */
	vreg_disable(vreg_vlcd);
	mdelay(10); /* not in spec. Request from Optics */
	vreg_disable(vreg_vlcd_io);
};

static void toshiba_hvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(4);  /* > 3ms according to spec */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(21); /* >20ms according to spec */
}

static struct panel_data_ext toshiba_hvga_panel_ext = {
	.power_on = toshiba_hvga_lcd_power_on,
	.power_off = toshiba_hvga_lcd_power_off,
	.exit_deep_standby = toshiba_hvga_lcd_exit_deep_standby,
#ifdef CONFIG_FB_MSM_SEMC_LCD_BACKLIGHT_CONTROL
	.backlight_ctrl = semc_delta_lcd_backlight_ctrl,
#endif
};

static struct msm_fb_panel_data toshiba_hvga_panel_data;

static struct platform_device mddi_toshiba_hvga_display_device = {
	.name = "mddi_toshiba_hvga",
	.id = -1,
};

static void __init msm_mddi_toshiba_hvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &toshiba_hvga_panel_data;

	printk(KERN_DEBUG "%s \n", __func__);

	panel_data->panel_info.xres = TOSHIBA_HVGA_PANEL_XRES;
	panel_data->panel_info.yres = TOSHIBA_HVGA_PANEL_YRES;
	panel_data->panel_info.type = MDDI_PANEL;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 16;
	panel_data->panel_info.clk_rate = 153600000;
	panel_data->panel_info.clk_min =  150000000;
	panel_data->panel_info.clk_max = 160000000;
	panel_data->panel_info.fb_num = 2;

	panel_data->panel_info.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	panel_data->panel_info.lcd.refx100 = 6600;
	panel_data->panel_info.lcd.v_back_porch = 1;
	panel_data->panel_info.lcd.v_front_porch = 2;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;

	panel_data->panel_ext = &toshiba_hvga_panel_ext;

	mddi_toshiba_hvga_display_device.dev.platform_data =
						&toshiba_hvga_panel_data;

	platform_device_register(&mddi_toshiba_hvga_display_device);
};

#endif /* CONFIG_FB_MSM_MDDI_TOSHIBA_HVGA_LCD */

#ifdef CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD

/* Display resolution */
#define HITACHI_HVGA_PANEL_XRES	320
#define HITACHI_HVGA_PANEL_YRES	480

static void hitachi_hvga_lcd_power_on(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	semc_delta_lcd_regulators_on();
	mdelay(1); /* spec: > 310 us */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(11); /* spec: > 10 ms */
}

static void hitachi_hvga_lcd_power_off(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(121); /* spec > 120 ms */
	vreg_disable(vreg_vlcd);
	vreg_disable(vreg_vlcd_io);
}

static void hitachi_hvga_lcd_exit_deep_standby(void)
{
	gpio_set_value(GPIO_MSM_MDDI_XRES, 0);
	mdelay(1); /* spec: > 200 us */
	gpio_set_value(GPIO_MSM_MDDI_XRES, 1);
	mdelay(11); /* spec: > 10 ms */
}

static struct panel_data_ext hitachi_hvga_panel_ext = {
	.power_on = hitachi_hvga_lcd_power_on,
	.power_off = hitachi_hvga_lcd_power_off,
	.exit_deep_standby = hitachi_hvga_lcd_exit_deep_standby,
#ifdef CONFIG_FB_MSM_SEMC_LCD_BACKLIGHT_CONTROL
	.backlight_ctrl = semc_delta_lcd_backlight_ctrl,
#endif
};

static struct msm_fb_panel_data hitachi_hvga_panel_data;

static struct platform_device mddi_hitachi_hvga_display_device = {
	.name = "mddi_hitachi_hvga",
	.id = -1,
};

static void __init msm_mddi_hitachi_hvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &hitachi_hvga_panel_data;

	printk(KERN_DEBUG "%s \n", __func__);

	panel_data->panel_info.xres = HITACHI_HVGA_PANEL_XRES;
	panel_data->panel_info.yres = HITACHI_HVGA_PANEL_YRES;
	panel_data->panel_info.type = MDDI_PANEL;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 16;
	panel_data->panel_info.clk_rate = 153600000;
	panel_data->panel_info.clk_min =  150000000;
	panel_data->panel_info.clk_max = 160000000;
	panel_data->panel_info.fb_num = 2;

	panel_data->panel_info.mddi.vdopkt = 0x0023;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	panel_data->panel_info.lcd.refx100 = 8500;
	panel_data->panel_info.lcd.v_back_porch = 1;
	panel_data->panel_info.lcd.v_front_porch = 16;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;

	panel_data->panel_ext = &hitachi_hvga_panel_ext;

	mddi_hitachi_hvga_display_device.dev.platform_data =
					&hitachi_hvga_panel_data;

	platform_device_register(&mddi_hitachi_hvga_display_device);
};
#endif	/* CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD */

static void __init msm_mddi_semc_delta_display_init(void)
{
#if defined(CONFIG_FB_MSM_MDDI_SAMSUNG_QVGA_LCD) && \
				defined(CONFIG_FB_MSM_MDDI_HITACHI_QVGA_LCD)
	/* delay 1: Samsung spec: >10ms, Hitachi spec: >10 us */
	/* delay 2: Samsung spec: >10ms, Hitachi spec: N/A */
	/* delay 3: Samsung spec: >10ms, Hitachi spec: >5 ms */
	semc_delta_lcd_power_on(11, 11, 11);
	msm_mddi_hitachi_qvga_display_device_init();
	msm_mddi_samsung_qvga_display_device_init();
#endif

#if defined(CONFIG_FB_MSM_MDDI_TOSHIBA_HVGA_LCD) && \
				defined(CONFIG_FB_MSM_MDDI_HITACHI_HVGA_LCD)
	/* delay 1: Toshiba spec: > 10ms, Hitachi spec: N/A */
	/* delay 2: Toshiba spec: > 1ms, Hitachi spec: > 310us */
	/* delay 3: Toshiba spec: > 20ms, Hitachi spec: > 10ms */
	semc_delta_lcd_power_on(11, 2, 21);
	msm_mddi_toshiba_hvga_display_device_init();
	msm_mddi_hitachi_hvga_display_device_init();
#endif
}

#endif /* CONFIG_FB_MSM_MDDI */

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[] = {
	[0] = {
		.start  = MSM_RAM_CONSOLE_START,
		.end    = MSM_RAM_CONSOLE_START+MSM_RAM_CONSOLE_SIZE-1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};
#endif

#ifdef CONFIG_PMIC_TIME
static struct platform_device pmic_time_device = {
	.name = "pmic_time",
};
#endif

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_smd,
        &msm_device_dmov,
	&msm_device_nand,
	&msm_device_otg,
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
	&msm_device_hsusb_peripheral,
	&msm_device_gadget_peripheral,
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif
	&msm_device_i2c,
	&msm_device_tssc,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_fb_device,
	&msm_device_uart_dm1,
#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300_SER
	&msm_device_uart_dm2,
	&cy8ctma300_ser_device,
#endif
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
	&msm_device_snd,
	&msm_device_adspdec,
#ifdef CONFIG_DLT001_CAMERA
	&msm_camera_sensor_dlt001,
#endif
#ifdef CONFIG_DLT002_CAMERA
	&msm_camera_sensor_dlt002,
#endif
	&msm_device_kgsl,
#if defined(CONFIG_LEDS_MSM_PMIC_FLASHLED)
	&flash_led_device,
#endif
#ifdef CONFIG_GPIO_ETS
	&semc_gpios_device,
#endif
#ifdef CONFIG_SEMC_RPC_SERVER_HANDSET
	&semc_rpc_handset_device,
#endif /* CONFIG_SEMC_RPC_SERVER_HANDSET */
#ifdef CONFIG_MSM_RPCSERVER_HANDSET
	&hs_device,
#endif
#ifdef CONFIG_LEDS_MSM_PMIC_MISC_KEYPAD
	&keypad_led_device,
#endif
#if defined(CONFIG_MSM_PMIC_LIGHTSENSOR)
	&lightsensor_device,
#endif
#if defined(CONFIG_DELTA_PROXIMITY_SENSOR)
	&delta_proximity_device,
#endif
#if defined(CONFIG_SEMC_SEPORT_PLATFORM)
	&seport_platform_device,
#endif
#ifdef CONFIG_MSM_SIM_CARD_DETECT
	&simcard_detect_device,
#endif
#ifdef CONFIG_SEMC_MSM_PMIC_VIBRATOR
	&vibrator_device,
#endif
#ifdef CONFIG_SEMC_GPIO_EXTR
	&semc_gpio_extr_device,
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
#ifdef CONFIG_SEMC_POWER
	&semc_power_device,
#endif
#ifdef CONFIG_CAPTURE_KERNEL
	&kdump_amsscoredump_device,
#endif
#ifdef CONFIG_PMIC_TIME
	&pmic_time_device,
#endif
};

/* SEMC_LUND: Added rachel patch for board config */

static void msm_fb_mddi_power_save(int on)
{
#if 0
	int flag_on = !!on;

	if (!flag_on) {
		gpio_set_value(MDDI_RST_OUT_GPIO, 0);
		mdelay(1);
	}

	msm_fb_vreg_config("gp5", flag_on);
	msm_fb_vreg_config("msme2", !flag_on);
	msm_fb_vreg_config("boost", flag_on);

	if (flag_on) {
		gpio_set_value(MDDI_RST_OUT_GPIO, 0);
		mdelay(1);
		gpio_set_value(MDDI_RST_OUT_GPIO, 1);
		gpio_set_value(MDDI_RST_OUT_GPIO, 1);
		mdelay(1);
	}
#endif
}
/* SEMC:SYS: Added by Qualcomm from 3135Y end */

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
}


extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data msm7x27_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_axi_khz = 128000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

static void sdcc_gpio_init(void)
{
	/* SD Card detection */
	if (gpio_request(49, "CARD_INS_N"))
		pr_err("failed to request gpio CARD_INS_N\n");

	/* SDC1 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc_clk"))
		pr_err("failed to request gpio sdc_clk\n");
#endif

	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
#endif
}

struct sdc_cfg_data {
	unsigned cfg_size;
	unsigned *cfg_data;
};

static unsigned sdcc1_on_cfg_data[] = {
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
};

static unsigned sdcc2_on_cfg_data[] = {
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
};

static struct sdc_cfg_data sdcc_on_cfg_data[] = {
	/* SDC1 configs */
	{
		.cfg_size = ARRAY_SIZE(sdcc1_on_cfg_data),
		.cfg_data = sdcc1_on_cfg_data,
	},
	/* SDC2 configs (WLAN) */
	{
		.cfg_size = ARRAY_SIZE(sdcc2_on_cfg_data),
		.cfg_data = sdcc2_on_cfg_data,
	},
};

static unsigned sdcc1_off_cfg_data[] = {
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned sdcc2_off_cfg_data[] = {
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
	GPIO_CFG(63, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(64, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(65, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(66, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(67, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
};

static struct sdc_cfg_data sdcc_off_cfg_data[] = {
	/* SDC1 configs off */
	{
		.cfg_size = ARRAY_SIZE(sdcc1_off_cfg_data),
		.cfg_data = sdcc1_off_cfg_data,
	},
	{
	/* SDC2 configs (WLAN) off */
		.cfg_size = ARRAY_SIZE(sdcc2_off_cfg_data),
		.cfg_data = sdcc2_off_cfg_data,
	}
};

static unsigned wifi_init_gpio_en[] = {

	GPIO_CFG(93, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* WLAN EN */

};

static void wlan_init_seq(void)
{
	int rc;
	rc = gpio_tlmm_config(wifi_init_gpio_en[0], GPIO_ENABLE);

	/* If we fail here print error and continue, this will result in higher */
	/* power consumption but if gpio_tlmm_config() really fails than we have */
	/* far bigger issues as this is the base call for config of gpio's */
	if (rc)
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, wifi_init_gpio_en[0], rc);

	/* Set device in low VIO-leakage state according to spec */
	/* This is done by toggle WLAN_EN pulewith > 10ms */
	gpio_set_value(93, 0);
	mdelay(1);
	gpio_set_value(93, 1);
	mdelay(12);
	gpio_set_value(93, 0);

}


static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
enum {
	STS_ERR = 1U << 0,
	STS_SD = 1U << 1,
	STS_WLAN   = 1U << 2,
	STS_SDCC_3 = 1U << 3,
	STS_SDCC_4 = 1U << 4,
};
static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

	if (!enable) {
		int idx = dev_id - 1;
		for (i = 0; i < sdcc_off_cfg_data[idx].cfg_size; i++) {
			rc = gpio_tlmm_config(
				sdcc_off_cfg_data[idx].cfg_data[i],
				GPIO_ENABLE);
			if (rc)
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__,
					sdcc_off_cfg_data[idx].cfg_data[i],
					rc);
		}
	}
	else {
		int idx = dev_id - 1;
		for (i = 0; i < sdcc_on_cfg_data[idx].cfg_size; i++) {
			rc = gpio_tlmm_config(
				sdcc_on_cfg_data[idx].cfg_data[i],
				GPIO_ENABLE);
			if (rc)
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__,
					sdcc_on_cfg_data[idx].cfg_data[i],
					rc);
		}
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;
	pdev = container_of(dv, struct platform_device, dev);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;
		/* Check if uSD regulator bit is set
		 * pdev->id == 1 is uSD device
		 */
		if ((vreg_sts & STS_SD) && (pdev->id == 1)) {
			rc = vreg_disable(vreg_mmc);
			msm_sdcc_setup_gpio(pdev->id, !!vdd);

			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		/* Check if WLAN regulator bit is set
		 * pdev->id == 2 is WLAN device
		 */
		else if ((vreg_sts & STS_WLAN) && (pdev->id == 2)) {
			msm_sdcc_setup_gpio(pdev->id, !!vdd);
		}
		else {
		/* SDCC 3 and SDCC 4 are not used in delta do nothing. */
		}

		clear_bit(pdev->id, &vreg_sts);
		return 0;
	}
	/* Check if uSD regulator bit not set
	 * pdev->id == 1 is uSD device
	 */
	if (((vreg_sts & STS_SD) == 0) && (pdev->id == 1)) {
		rc = vreg_set_level(vreg_mmc, 2850);
		if (!rc) {
			msm_sdcc_setup_gpio(pdev->id, !!vdd);
			rc = vreg_enable(vreg_mmc);
		}
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	/* Check if WLAN regulator bit is not set
	 * pdev->id == 2 that is WLAN device
	 */
	else if (((vreg_sts & STS_WLAN) == 0) && (pdev->id == 2)) {
			msm_sdcc_setup_gpio(pdev->id, !!vdd);
	}
	else {
	/* SDCC 3 and SDCC 4 are not used in delta do nothing. */
	}

	set_bit(pdev->id, &vreg_sts);
	return 0;
}

static unsigned int robyn_sdcc_slot_status(struct device *dev)
{
	unsigned int ret = 0;
	if (gpio_get_value(49))
		ret = 0;
	else
		ret = 1;
	return ret;
}

static struct mmc_platform_data msm7x27_sdcc_data1 = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.status = robyn_sdcc_slot_status,
	.status_irq	= MSM_GPIO_TO_INT(49),
	.irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.mmc_bus_width = MMC_CAP_4_BIT_DATA,
};

static struct mmc_platform_data msm7x27_sdcc_data2 = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
};

static void __init msm7x2x_init_mmc(void)
{
#ifdef CONFIG_CAPTURE_KERNEL
	smsm_wait_for_modem_reset();
#endif
	vreg_mmc = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_mmc));
		return;
	}

	sdcc_gpio_init();
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &msm7x27_sdcc_data1);
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	msm_add_sdcc(2, &msm7x27_sdcc_data2);
#endif
}

static struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		/* gpio_scl = 95; */
		/* gpio_sda = 96; */
		return; /* No support for secondary i2c in Semc Delta HW */
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.rmutex = 0,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");

	msm_i2c_pdata.pm_lat =
		msm7x27_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_serial_hs_platform_data bt_uart_platform_data_delta = {
	.wakeup_irq = MSM_GPIO_TO_INT(44),
	.wakeup_edge = IRQF_TRIGGER_RISING,
	.wakeup_ignore_not_needed = 1,
};

static void __init msm7x2x_init(void)
{
	if (socinfo_init() < 0)
		BUG();

	/* Toggle WLAN ENABLE */
	wlan_init_seq();

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1);
#endif
	if (cpu_is_msm7x27())
		msm7x27_clock_data.max_axi_khz = 200000;

	msm_acpu_clock_init(&msm7x27_clock_data);
	kgsl_pdata.high_axi_3d = clk_get_max_axi_khz();
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;

	msm_gadget_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata;
	msm_device_uart_dm1.dev.platform_data = &bt_uart_platform_data_delta;
	platform_add_devices(devices, ARRAY_SIZE(devices));
#if defined (CONFIG_DLT001_CAMERA) || defined (CONFIG_DLT002_CAMERA)
	config_camera_off_gpios(); /* might not be necessary */
#endif
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	platform_device_register(get_keypad_device_delta());
	msm_fb_add_devices();
	rmt_storage_add_ramfs();
	msm7x2x_init_mmc();
	bt_power_init();

#ifdef CONFIG_FB_MSM_MDDI
	msm_mddi_semc_delta_display_init();
#endif
#if defined(CONFIG_LEDS_MSM_PMIC_FLASHLED)
	msm_pmic_vreg_boost_init();
#endif
	msm_pm_set_platform_data(msm7x27_pm_data);
}
static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
       pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static void __init pmem_mdp_size_setup(char **p)
{
       pmem_mdp_size = memparse(*p, p);
}
__early_param("pmem_mdp_size=", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
       pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static void __init pmem_audio_size_setup(char **p)
{
	pmem_audio_size = memparse(*p, p);
}
__early_param("pmem_audio_size=", pmem_audio_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
       fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static void __init gpu_phys_size_setup(char **p)
{
       gpu_phys_size = memparse(*p, p);
}
__early_param("gpu_phys_size=", gpu_phys_size_setup);

#ifndef CONFIG_CAPTURE_KERNEL
static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

	size = gpu_phys_size ? : MSM_GPU_PHYS_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	kgsl_resources[1].start = __pa(addr);
	kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for KGSL\n",
		size, addr, __pa(addr));
}
#endif

static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();
	/* Technically dependent on the SoC but using machine_is
	 * macros since socinfo is not available this early and there
	 * are plans to restructure the code which will eliminate the
	 * need for socinfo.
	 */
	msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);
#ifndef CONFIG_CAPTURE_KERNEL
	msm_msm7x2x_allocate_memory_regions();
#endif

#ifdef CONFIG_CACHE_L2X0
	/* 7x27 has 256KB L2 cache:
	    64Kb/Way and 4-Way Associativity;
		R/W latency: 3 cycles;
		evmon/parity/share disabled. */
	l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

/* Startup reason from cmdline */
static int __init startup_reason_setup(char *str)
{
	startup_reason = simple_strtoul(str, NULL, 16);

	printk(KERN_DEBUG "%s: 0x%x\n", __func__, startup_reason);

	return 1;
}
__setup("startup=", startup_reason_setup);

/* USB serial number from cmdline */
static int __init board_serialno_setup(char *serialno)
{
	int ix, len;
	static char usb_serial_number[21];

	len = strlen(serialno);
	ix = 0;
	while (ix < 20) {
		if (*serialno && ix >= 20 - (len << 1)) {
			sprintf(&usb_serial_number[ix], "%02X",
					(unsigned char)*serialno);
			serialno++;
		} else {
			sprintf(&usb_serial_number[ix], "%02X", 0);
		}
		ix += 2;
	}
	usb_serial_number[20] = '\0';
#ifdef CONFIG_USB_ANDROID
	android_usb_pdata.serial_number = usb_serial_number;
	printk(KERN_INFO "USB serial number: %s\n", android_usb_pdata.serial_number);
#endif
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.serial_number = usb_serial_number;
	printk(KERN_INFO "USB serial number: %s\n", msm_hsusb_pdata.serial_number);
#endif
	return 0;
}
__setup("serialno=", board_serialno_setup);

MACHINE_START(MSM7X27_SURF, "SEMC Delta")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
#ifdef CONFIG_CAPTURE_KERNEL
	.boot_params	= PHYS_OFFSET + 0x1000,
#else
	.boot_params	= PHYS_OFFSET + 0x100,
#endif
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
