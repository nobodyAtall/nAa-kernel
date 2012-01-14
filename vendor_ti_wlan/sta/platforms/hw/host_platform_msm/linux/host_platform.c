/*
 * SdioAdapter.h
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Texas Instruments nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "linux/version.h"
#include "tidef.h"
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "host_platform.h"
#include "ioctl_init.h"

#include <asm/gpio.h>
#include <WlanDrvIf.h>
#include <asm/mach/mmc.h>

#include "proc_comm.h"

/* TODO: Platform specific */
#define MISSING_WIFI_POWER_SUPPORT


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30)
#define WLAN_EN				(93) //board delta
#else
#define WLAN_EN				(57) //board mogami
#endif



#define LOW     0
#define HIGH    1

#define OS_API_MEM_ADRR  	0x0000000
#define OS_API_REG_ADRR  	0x300000

#ifdef MISSING_WIFI_POWER_SUPPORT
/* Not all platforms have code that sets the external GPIO lines and functionality for powering on/off wifi */
/* Forward declaration */

struct platform_device *sdioDrv_get_platform_device(void);
static int wakeup_state = 0;


int wifi_power(int on)
{
	struct platform_device *ppdev = 0;
	struct mmc_platform_data *pplat = 0;
	uint32_t res = 0;

	/* Turn on/off power */
	ppdev = sdioDrv_get_platform_device();
	if (!ppdev)
		return -ENODEV;
	pplat = ppdev->dev.platform_data;
	if (!pplat)
		return -ENODEV;
	if (!pplat->translate_vdd)
		return -ENODEV;
	res = pplat->translate_vdd(&(ppdev->dev), on);

	gpio_set_value(WLAN_EN, on);
	mdelay(100);

	return res;
}
#else
/* TODO: Platform specific */
#define wifi_power robyn_wifi_power
int wifi_power(int on);
#endif /* MISSING_WIFI_POWER_SUPPORT */

/* set the GPIO to low after awaking the TNET from ELP */
int hPlatform_hardResetTnetw(void)
{
	int err;

	// Turn power OFF
	if ((err = wifi_power(0)) == 0) {
		msleep(150);
		// Turn power ON
		err = wifi_power(1);
		msleep(150);
	}
	return err;
}

int hPlatform_DevicePowerOff (void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;
	int err = 0;

	err = wifi_power(0);
	if (wakeup_state) {
		set_irq_wake(drv->irq, 0);
		wakeup_state = 0;
	}

	return err;
}

int hPlatform_DevicePowerOffSetLongerDelay (void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;
	int err = 0;

	err = wifi_power(0);
	msleep(500);
	if (wakeup_state) {
		set_irq_wake(drv->irq, 0);
		wakeup_state = 0;
	}

	return err;
}

int hPlatform_DevicePowerOn (void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;
	int err = 0;

	err = wifi_power(1);
	msleep(30);
	if (!err && !wakeup_state) {
		set_irq_wake(drv->irq, 1);
		wakeup_state = 1;
	}

	return err;
}

int hPlatform_Wlan_Hardware_Init(void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;

	drv->irq = TNETW_IRQ;
	drv->irq_flags = (unsigned long)IRQF_TRIGGER_FALLING;

	return 0;
}

void hPlatform_Wlan_Hardware_DeInit(void)
{
}

int hPlatform_initInterrupt(void *tnet_drv, void* handle_add )
{
	TWlanDrvIfObj *drv = (TWlanDrvIfObj*) tnet_drv;
	int rc;


	if (drv->irq == 0 || handle_add == NULL) {
		print_err("hPlatform_initInterrupt() bad param drv->irq=%d handle_add=0x%x !!!\n",drv->irq,(int)handle_add);
		return -EINVAL;
	}

	if ((rc = request_irq(drv->irq, handle_add, IRQF_TRIGGER_FALLING, drv->netdev->name, drv))) {
		print_err("TIWLAN: Failed to register interrupt handler\n");
		return rc;
	}

	if (gpio_request(WLAN_EN,"TI 1271") != 0) {
		return -EINVAL;
	}
	gpio_direction_output(WLAN_EN,LOW);

	return rc;
}

void hPlatform_freeInterrupt(void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;

	free_irq(drv->irq, drv);

	gpio_free(WLAN_EN);
}

void* hPlatform_hwGetRegistersAddr(TI_HANDLE OsContext)
{
	return 0;
}

void* hPlatform_hwGetMemoryAddr(TI_HANDLE OsContext)
{
	return 0;
}
