/*
 * WlanDrvWext.c
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


/*
 * src/wext.c
 *
 * Support for Linux Wireless Extensions
 *
 */
#define __FILE_ID__ FILE_ID_143
#include <linux/types.h>
#include <linux/socket.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include "WlanDrvIf.h"
#include "CmdHndlr.h"
#include "CmdInterpretWext.h"
#include "privateCmd.h"
#include "DrvMain.h"
#include "CmdDispatcher.h"
#include "mlmeApi.h"
#include "802_11Defs.h"

/* Routine prototypes */

int wlanDrvWext_Handler (struct net_device *dev,
                         struct iw_request_info *info,
                         void  *iw_req,
                         void  *extra);

static struct iw_statistics *wlanDrvWext_GetWirelessStats (struct net_device *dev);

extern int wlanDrvIf_LoadFiles (TWlanDrvIfObj *drv, TLoaderFilesData *pInitInfo);
extern int wlanDrvIf_Start (struct net_device *dev);
extern int wlanDrvIf_Stop (struct net_device *dev);

/* callbacks for WEXT commands */
static const iw_handler aWextHandlers[] = {
	(iw_handler) NULL,				            /* SIOCSIWCOMMIT */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWNAME */
	(iw_handler) NULL,				            /* SIOCSIWNWID */
	(iw_handler) NULL,				            /* SIOCGIWNWID */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWFREQ */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWFREQ */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWMODE */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWMODE */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWSENS */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWSENS */
	(iw_handler) NULL,                          /* SIOCSIWRANGE - not used */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWRANGE */
	(iw_handler) NULL,		                    /* SIOCSIWPRIV - not used */
	(iw_handler) NULL,                    		/* SIOCGIWPRIV - kernel code */
	(iw_handler) NULL,                          /* SIOCSIWSTATS - not used */
	(iw_handler) wlanDrvWext_GetWirelessStats,  /* SIOCGIWSTATS - kernel code */
	(iw_handler) NULL,		                    /* SIOCSIWSPY */
	(iw_handler) NULL,		                    /* SIOCGIWSPY */
	(iw_handler) NULL,		                    /* SIOCSIWTHRSPY */
	(iw_handler) NULL,		                    /* SIOCGIWTHRSPY */
	(iw_handler) wlanDrvWext_Handler,           /* SIOCSIWAP */
	(iw_handler) wlanDrvWext_Handler,           /* SIOCGIWAP */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWMLME */
	(iw_handler) NULL,		        			/* SIOCGIWAPLIST */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWSCAN */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWSCAN */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWESSID */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWESSID */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWNICKN */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWNICKN */
	(iw_handler) NULL,				            /* -- hole -- */
	(iw_handler) NULL,				            /* -- hole -- */
	(iw_handler) NULL,		        			/* SIOCSIWRATE */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWRATE */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWRTS */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWRTS */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWFRAG */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWFRAG */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWTXPOW */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWTXPOW */
	(iw_handler) NULL,		        			/* SIOCSIWRETRY */
	(iw_handler) NULL,		        			/* SIOCGIWRETRY */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWENCODE */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWENCODE */
	(iw_handler) NULL,		        			/* SIOCSIWPOWER */
	(iw_handler) NULL,		        			/* SIOCGIWPOWER */
	(iw_handler) NULL,				            /* -- hole -- */
	(iw_handler) NULL,				            /* -- hole -- */
	(iw_handler) wlanDrvWext_Handler,           /* SIOCSIWGENIE */
	(iw_handler) NULL,		        			/* SIOCGIWGENIE */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCSIWAUTH */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCGIWAUTH */
	(iw_handler) wlanDrvWext_Handler,	        /* SIOCSIWENCODEEXT */
	(iw_handler) NULL,	            			/* SIOCGIWENCODEEXT */
	(iw_handler) wlanDrvWext_Handler, 			/* SIOCSIWPMKSA */
};

/* callbacks for private commands */
static const iw_handler aPrivateHandlers[] = {
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCIWFIRSTPRIV+0 (set) */
	(iw_handler) wlanDrvWext_Handler,		    /* SIOCIWFIRSTPRIV+1 (get) */
};

/* Describe the level of WEXT support to kernel */
static struct iw_handler_def tWextIf = {
#define	N(a)	(sizeof (a) / sizeof (a[0]))
	.standard		    = (iw_handler *) aWextHandlers,
	.num_standard		= N(aWextHandlers),
	.private		    = (iw_handler *) aPrivateHandlers,
	.num_private		= N(aPrivateHandlers),
	.private_args		= NULL,
	.num_private_args	= 0,
	.get_wireless_stats	= wlanDrvWext_GetWirelessStats,
#undef N
};

/* Initialite WEXT support - Register callbacks in kernel */
void wlanDrvWext_Init (struct net_device *dev)
{
#ifdef HOST_PLATFORM_OMAP3430
	dev->get_wireless_stats = wlanDrvWext_GetWirelessStats;
#endif
	dev->wireless_handlers = &tWextIf;

}

/* Return driver statistics */
static struct iw_statistics *wlanDrvWext_GetWirelessStats(struct net_device *dev) {
	TWlanDrvIfObj *drv = (TWlanDrvIfObj *)NETDEV_GET_PRIVATE(dev);

	return (struct iw_statistics *) cmdHndlr_GetStat (drv->tCommon.hCmdHndlr);
}

/**
 * Get Guard interval.
 * return -1 on error. 0 on long and 1 on short.
 */
static int getGI(TWlanDrvIfObj *drv)
{
	int ret;
	paramInfo_t pParam;
	TI_UINT8 *assocRsp;
	Tdot11HtCapabilitiesUnparse *htCapabilities;
	TI_UINT16 htCapabilitiesInfo;

	pParam.paramType   = ASSOC_ASSOCIATION_RESP_PARAM;
	pParam.paramLength = sizeof(TAssocReqBuffer);
	cmdDispatch_GetParam(drv->tCommon.hCmdDispatch, &pParam);

	/* Make sure buffer is big enough */
	if(pParam.content.assocReqBuffer.bufferSize < ASSOC_RESP_FIXED_DATA_LEN + sizeof(Tdot11HtCapabilitiesUnparse)) {
		return -1;
	}

	assocRsp = pParam.content.assocReqBuffer.buffer;

	ret = mlmeParser_ParseIeBuffer(NULL, &assocRsp[ASSOC_RESP_FIXED_DATA_LEN],
	                               pParam.content.assocReqBuffer.bufferSize-ASSOC_RESP_FIXED_DATA_LEN,
	                               HT_CAPABILITIES_IE_ID,
	                               (TI_UINT8**)&htCapabilities, NULL, 0);
	if(ret == TI_FALSE)
		return -1;

	if( ((TI_UINT8*)htCapabilities) + sizeof(Tdot11HtCapabilitiesUnparse) > assocRsp + pParam.content.assocReqBuffer.bufferSize) {
		os_printf("Corrupt HT_CAPABILITIES_IE. Cannot get GI.\n");
		return -1;
	}

	htCapabilitiesInfo = htCapabilities->aHtCapabilitiesIe[0];
	htCapabilitiesInfo |= htCapabilities->aHtCapabilitiesIe[1] << 8;

	if(htCapabilitiesInfo & 0x20)
		return 1;
	else
		return 0;
}

/*
 * Calculate the link speed. Taking GI time in account.
 */
static int calcLinkSpeed(TWlanDrvIfObj *drv)
{
	int shortGI = getGI(drv);
	int baseSpeed = drv->tCommon.uLinkSpeed * 100;
	int newSpeed;

	if(shortGI != 1)
		return baseSpeed;

	switch(baseSpeed)
	{
	case 6500000:
		newSpeed = 7200000;
		break;
	case 13000000:
		newSpeed = 14400000;
		break;
	case 19500000:
		newSpeed = 21700000;
		break;
	case 26000000:
		newSpeed = 28900000;
		break;
	case 39000000:
		newSpeed = 43300000;
		break;
	case 52000000:
		newSpeed = 57800000;
		break;
	case 58500000:
		newSpeed = 65000000;
		break;
	case 65000000:
		newSpeed = 72200000;
		break;
	default:
		newSpeed = ((baseSpeed / 100) * 111);

	}
	return newSpeed;
}

/* Generic callback for WEXT commands */
int wlanDrvWext_Handler (struct net_device *dev,
                         struct iw_request_info *info,
                         void *iw_req,
                         void *extra)
{
	int              rc;
	TWlanDrvIfObj   *drv = (TWlanDrvIfObj *)NETDEV_GET_PRIVATE(dev);
	ti_private_cmd_t my_command;
	struct iw_mlme   mlme;
	struct iw_scan_req scanreq;

	void             *copy_to_buf=NULL, *param3=NULL;

	os_memoryZero(drv, &my_command, sizeof(ti_private_cmd_t));
	os_memoryZero(drv, &mlme,       sizeof(struct iw_mlme));
	os_memoryZero(drv, &scanreq, sizeof(struct iw_scan_req));

	switch (info->cmd) {
	case SIOCIWFIRSTPRIV: {
		void *copy_from_buf;

		if (os_memoryCopyFromUser(drv, &my_command, ((union iwreq_data *)iw_req)->data.pointer, sizeof(ti_private_cmd_t))) {
			os_printf ("wlanDrvWext_Handler() os_memoryCopyFromUser FAILED !!!\n");
			return TI_NOK;
		}
		if (IS_PARAM_FOR_MODULE(my_command.cmd, DRIVER_MODULE_PARAM)) {
			/* If it's a driver level command, handle it here and exit */
			switch (my_command.cmd) {
			case DRIVER_INIT_PARAM:
				return wlanDrvIf_LoadFiles(drv, my_command.in_buffer);

			case DRIVER_START_PARAM:
				return wlanDrvIf_Start(dev);

			case DRIVER_STOP_PARAM:
				return wlanDrvIf_Stop(dev);

			case DRIVER_STATUS_PARAM:
				*(TI_UINT32 *)my_command.out_buffer =
				    (drv->tCommon.eDriverState == DRV_STATE_RUNNING) ? TI_TRUE : TI_FALSE;
				return TI_OK;
			}
		}
		/* if we are still here handle a normal private command*/

		if ((my_command.in_buffer) && (my_command.in_buffer_len)) {
			copy_from_buf        = my_command.in_buffer;
			my_command.in_buffer = os_memoryAlloc(drv, my_command.in_buffer_len);
			if (os_memoryCopyFromUser(drv, my_command.in_buffer, copy_from_buf, my_command.in_buffer_len)) {
				os_printf("wlanDrvWext_Handler() os_memoryCopyFromUser 1 FAILED !!!\n");
				return TI_NOK;
			}
		}
		if ((my_command.out_buffer) && (my_command.out_buffer_len)) {
			copy_to_buf          = my_command.out_buffer;
			my_command.out_buffer = os_memoryAlloc(drv, my_command.out_buffer_len);
		}
		param3 = &my_command;
	}
	break;

	case SIOCGIWRATE: {
		struct iw_param *vwrq = (struct iw_param *) iw_req;
		vwrq->value = calcLinkSpeed(drv);
		return TI_OK;
	}
	break;

	case SIOCSIWMLME: {
		os_memoryCopyFromUser(drv, &mlme, ((union iwreq_data *)iw_req)->data.pointer, sizeof(struct iw_mlme));
		param3 = &mlme;
	}
	break;
	case SIOCSIWSCAN: {
		if (((union iwreq_data *)iw_req)->data.pointer) {
			os_memoryCopyFromUser(drv, &scanreq, ((union iwreq_data *)iw_req)->data.pointer, sizeof(struct iw_scan_req));
			param3 = &scanreq;
		}
	}
	break;

	case SIOCSIWGENIE: {
		TI_UINT16 ie_length = ((union iwreq_data *)iw_req)->data.length;
		TI_UINT8 *ie_content = ((union iwreq_data *)iw_req)->data.pointer;

		if ((ie_length == 0) && (ie_content == NULL)) {
			/* Do nothing, deleting the IE */
		} else if ((ie_content != NULL) && (ie_length <= RSN_MAX_GENERIC_IE_LENGTH) && (ie_length > 0)) {
			/* One IE cannot be larger than RSN_MAX_GENERIC_IE_LENGTH bytes */
			my_command.in_buffer = os_memoryAlloc(drv, ie_length);
			os_memoryCopyFromUser(drv, my_command.in_buffer, ie_content, ie_length );
			param3 = my_command.in_buffer;
		} else {
			return TI_NOK;
		}
	}
	break;
	}
	/* If the friver is not running, return NOK */
	if (drv->tCommon.eDriverState != DRV_STATE_RUNNING) {
		if (my_command.in_buffer)
			os_memoryFree(drv, my_command.in_buffer, my_command.in_buffer_len);
		if (my_command.out_buffer)
			os_memoryFree(drv,my_command.out_buffer,my_command.out_buffer_len);
		return TI_NOK;
	}

	/* Call the Cmd module with the given user paramters */
	rc = cmdHndlr_InsertCommand(drv->tCommon.hCmdHndlr,
	                            info->cmd,
	                            info->flags,
	                            iw_req,
	                            0,
	                            extra,
	                            0,
	                            param3,
	                            NULL);
	/* Here we are after the command was completed */
	if (my_command.in_buffer) {
		os_memoryFree(drv, my_command.in_buffer, my_command.in_buffer_len);
	}
	if (my_command.out_buffer) {
		if (os_memoryCopyToUser(drv, copy_to_buf, my_command.out_buffer, my_command.out_buffer_len)) {
			os_printf("wlanDrvWext_Handler() os_memoryCopyToUser FAILED !!!\n");
			rc = TI_NOK;
		}
		os_memoryFree(drv, my_command.out_buffer, my_command.out_buffer_len);
	}

	return rc;
}
