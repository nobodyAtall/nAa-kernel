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

/*--------------------------------------------------------------------------
 Module:      host_platform_sdio.h

 Purpose:     This module defines unified interface to the host platform specific
              sources and services.

--------------------------------------------------------------------------*/

#ifndef __HOST_PLATFORM_SDIO__H__
#define __HOST_PLATFORM_SDIO__H__

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <linux/version.h>

/* TODO: Platform specific constant */
/* This is extremely ugly but will do for the bringup */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30)
#define TNETW_IRQ		MSM_GPIO_TO_INT(27) // board delta
#else
#define TNETW_IRQ		MSM_GPIO_TO_INT(147) // board mogami
#endif

#define TIWLAN_IRQ_POLL_INTERVAL	    (HZ/100)


int
hPlatform_initInterrupt(
    void* tnet_drv,
    void* handle_add
);

void*
hPlatform_hwGetRegistersAddr(
    TI_HANDLE OsContext
);

void*
hPlatform_hwGetMemoryAddr(
    TI_HANDLE OsContext
);

void hPlatform_freeInterrupt(void *tnet_drv);

int  hPlatform_hardResetTnetw(void);
int  hPlatform_Wlan_Hardware_Init(void *tnet_drv);
void hPlatform_Wlan_Hardware_DeInit(void);
int  hPlatform_DevicePowerOff(void *tnet_drv);
int  hPlatform_DevicePowerOffSetLongerDelay(void *tnet_drv);
int  hPlatform_DevicePowerOn(void *tnet_drv);
#endif /* __HOST_PLATFORM_SDIO__H__ */
