/*
 * roleAP.c
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


/** \file roleAP.c
 *  \brief Role AP info
 *
 *  \see roleAP.h
 */


#define __FILE_ID__  FILE_ID_140

#include "siteHash.h"
#include "roleAP.h"
#include "osApi.h"
#include "report.h"
#include "802_11Defs.h"
#include "TWDriver.h"
#include "EvHandler.h"
#include "DrvMainModules.h"
#include "regulatoryDomainApi.h"

#include "paramOut.h"
#include "APExternalIf.h"
#include "wlanLinks.h"
#include "ApCmd.h"
#include "DrvMain.h"
#include "WlanDrvIf.h"
#include "rx.h"
#include "rate.h"
#include "siteMgrApi.h"

/* Constants */
#define BSS_INDEX_DEFAULT   1
#define AP_SSID_LENGTH      8
#define AP_MAX_TX_RETRY     100
/* Enumerations */

/* Typedefs */

/* roleAP object */
typedef struct {

	/* Module's handles*/
	TI_HANDLE           hReport;
	TI_HANDLE           hOs;
	TI_HANDLE           hEvHandler;
	TI_HANDLE           hTWD;
	TI_HANDLE           hSiteMgr;
	TI_HANDLE           hCtrlData;
	TI_HANDLE           hRegulatoryDomain;
	TI_HANDLE			hWlanLinks;
	TI_HANDLE			hApCmd;
	TI_HANDLE           hTxDataQ;
	TI_HANDLE           hTxMgmtQ;
	TI_HANDLE           hDrvMain;
	TI_HANDLE           hTxPort;
	TI_HANDLE           hTxCtrl;
	TI_HANDLE           hRxData;

	TBssCapabilities    tBssCapabilities;
	TI_UINT8            uBeaconTxTimeout;
	TI_UINT32			aAllocatedLinks[WLANLINKS_MAX_LINKS];
	TI_UINT32			uNumAllocatedLinks;
	TI_UINT32			uBrcstHlid;
	TI_UINT32			uGlobalHlid;
	TI_UINT8            cBssIndex;
	EKeyType            keyType;
	TI_UINT8            DefaultKeyIndex;
	TSecurityKeys       tTwdKey[MAX_WEP_KEY];
	ERoleApState		eState;
	TRoleApStats		tRoleApStats;
	TApWpsIe            tWpsIe;
	TI_UINT32           uTxPower;
} TRoleAP;


/* Structures */

/* Internal functions prototypes */
static TI_STATUS BssStart(TI_HANDLE hRoleAP, TI_UINT8 uBssIdx);
static EHwRateBitFiled ConvertRateSetToBitmap(TI_HANDLE hRoleAP, TI_UINT8 *pRates, TI_UINT8 uRatesLen);
static TI_STATUS AddStation(TI_HANDLE hRoleAP, void *pStationParams);
static TI_STATUS RemStation(TI_HANDLE hRoleAP, void *pBuf);
static TI_STATUS SaveDeauthReason(TI_HANDLE hRoleAP, TApGeneralParam *pGenParam);
static TI_STATUS SetPortStatus(TRoleAP *pRoleAP , TApGeneralParam *tGenParam);
static TI_STATUS AddStaToFW(TRoleAP *pRoleAP, TI_UINT32 uHlid, TApStationParams *pStaParams);
static TI_STATUS RemStaFromFW(TRoleAP *pRoleAP, TI_UINT32 uHlid);
static TI_STATUS AllocateWlanLink(TI_HANDLE hRoleAP, EWlanLinkType eLinkType, TApStationParams *pStaParams, TI_UINT32 *pHlid);
static TI_STATUS FreeWlanLink(TI_HANDLE hRoleAP, TI_UINT32 uHlid);
static TI_STATUS InitBssLinks(TI_HANDLE hRoleAP);
static void FillLinkInfo(TI_HANDLE hRoleAP, EWlanLinkType eType, TApStationParams *pStaParams, TWlanLinkInfo *pLinkInfo);
static void PrintRoleApDB(TRoleAP *pRoleAP);

static void BssStartCompleteCb(TI_HANDLE hRoleAP);
static TI_STATUS setRxPortStatus(TRoleAP *pRoleAP, TI_BOOL bEnable);
static TI_STATUS InactiveStaEventCB(TI_HANDLE hRoleAP, TI_CHAR *pData, TI_UINT32 uDataLen);
static TI_STATUS staInactivity(TI_HANDLE hRoleAP, void * pInBuf, void *pOutBuf);
static void FillDeauthTemplate(TI_HANDLE hRoleAP, TSetTemplate *pTemplateStruct);
static void ConfigureFrameTemplates(TI_HANDLE hRoleAP);
static void ConfigureRatePolicies(TI_HANDLE hRoleAP);
static TI_STATUS setBeaconProbeRspTempl(TRoleAP *pRoleAP);
static void setDeauthTemplate( TRoleAP * pRoleAP );
static void setQosNullDataTemplate( TRoleAP * pRoleAP);
static void setNullDataTemplate(TRoleAP * pRoleAP);
static TI_STATUS setKey(TI_HANDLE hRoleAP, TSecurityKeys *pTwdKey);
static TI_STATUS FillBeaconTemplate(TI_HANDLE hRoleAP, TApBeaconParams *pAPBeaconParams, TSetTemplate *pTemplateStruct);
static TI_STATUS FillProbeRespTemplate(TI_HANDLE hRoleAP, TApBeaconParams *pAPBeaconParams, TSetTemplate *pTemplateStruct);
static TI_STATUS sendEvent(TRoleAP *pRoleAP, TI_UINT32 uHlid, EApEvent eEvent);
static TI_STATUS maxTxRetryStaEventCB(TI_HANDLE hRoleAP, TI_CHAR *pData, TI_UINT32 uDataLen);
static void DecodeStaRates(TI_UINT8 *aRates, TI_UINT32 uRatesLen);
static void SetApRates(TI_HANDLE hRoleAP, TApRateSet *pRateParams);
static TI_STATUS RemoveBssLinks(TI_HANDLE hRoleAP, TI_BOOL bRemFromFw);
static TI_STATUS roleAP_enable (TI_HANDLE hRoleAP);
static void ResetRoleApDB(TI_HANDLE hRoleAP);

/* Public functions implementation */


/**
 * \fn     roleAP_create
 * \brief  Create roleAP object
 *
 * Allocate and clear the module's object
 */
TI_HANDLE roleAP_create(TI_HANDLE hOs)
{
	TRoleAP   *pRoleAP;

	if ((pRoleAP = os_memoryAlloc(hOs, sizeof(TRoleAP))) != NULL) {
		pRoleAP->hOs = hOs;
		os_memoryZero (hOs, (void *)pRoleAP, sizeof(*pRoleAP));
		return pRoleAP;
	} else { /* Failed to allocate control block */
		WLAN_OS_REPORT(("FATAL ERROR: roleAP_create(): Error allocating cb - aborting\n"));
		return NULL;
	}
}


/**
 * \fn     roleAP_destroy
 * \brief  Destroy RolesMgr object
 *
 * Free the module's object memory
 */
TI_STATUS roleAP_destroy(TI_HANDLE hRoleAP)
{
	TRoleAP   *pRoleAP;

	if (hRoleAP != NULL) {
		pRoleAP = (TRoleAP *)hRoleAP;

		/* Free pre-allocated control block */
		os_memoryFree(pRoleAP->hOs, pRoleAP, sizeof(TRoleAP));
	}
	return TI_OK;
}



/**
 * \fn     roleAP_init
 * \brief  Init roleAP object
 *
 * Init module's object and link its handles
 */
void roleAP_init (TStadHandlesList *pStadHandles)
{
	TRoleAP *pRoleAP = (TRoleAP *)(pStadHandles->hRoleAP);

	pRoleAP->hOs                = pStadHandles->hOs;
	pRoleAP->hReport            = pStadHandles->hReport;
	pRoleAP->hEvHandler         = pStadHandles->hEvHandler;
	pRoleAP->hTWD               = pStadHandles->hTWD;
	pRoleAP->hCtrlData          = pStadHandles->hCtrlData;
	pRoleAP->hRegulatoryDomain  = pStadHandles->hRegulatoryDomain;
	pRoleAP->hSiteMgr           = pStadHandles->hSiteMgr;
	pRoleAP->hWlanLinks         = pStadHandles->hWlanLinks;
	pRoleAP->hApCmd             = pStadHandles->hApCmd;
	pRoleAP->hTxDataQ           = pStadHandles->hTxDataQ;
	pRoleAP->hTxMgmtQ           = pStadHandles->hTxMgmtQ;
	pRoleAP->hDrvMain           = pStadHandles->hDrvMain;
	pRoleAP->hTxPort            = pStadHandles->hTxPort;
	pRoleAP->hTxDataQ           = pStadHandles->hTxDataQ;
	pRoleAP->hTxCtrl            = pStadHandles->hTxCtrl;
	pRoleAP->hRxData            = pStadHandles->hRxData;
	pRoleAP->eState				= ROLEAP_STATE_STOPPED;
}


/**
 * \fn     roleAP_SetDefaults
 * \brief  set roleAP object default values
 *
 * Set module's object default values
 */
TI_STATUS roleAP_SetDefaults (TI_HANDLE hRoleAP, TRoleApInitParams *tRoleApInitParams)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;


	pRoleAP->uBeaconTxTimeout = tRoleApInitParams->ubeaconTxTimeout;

	rxData_IntraBssBridge_Enable(pRoleAP->hRxData);

	return TI_OK;
}

/*ALEXA_TODO: remove roleAP set/get param stubs*/

/**
 * \fn     roleAP_setParam
 * \brief  roleAP object setParam API
 *
 * Module's object set param API
 */
TI_STATUS roleAP_setParam(TI_HANDLE hRoleAP, paramInfo_t *pParam)
{
	TRoleAP     *pRoleAP = (TRoleAP *)hRoleAP;
	TI_STATUS   status = TI_OK;

	if (pParam == NULL) {
		return TI_NOK;
	}


	switch (pParam->paramType) {

	case ROLE_AP_BSS_START_PARAM:
		pRoleAP->tBssCapabilities.uBssIndex = pParam->content.roleAPBssIndex;
		roleAP_start(hRoleAP, pRoleAP->tBssCapabilities.uBssIndex);
		break;

	case ROLE_AP_BSS_STOP_PARAM:
		roleAP_stop(hRoleAP, pParam->content.roleAPBssIndex);
		break;

	case ROLE_AP_SET_SSID_TYPE_PARAM:
		pRoleAP->tBssCapabilities.eSsidType = pParam->content.roleAPSsidType;
		break;

	case ROLE_AP_SET_CHANNEL_PARAM:
		pRoleAP->tBssCapabilities.uChannel = pParam->content.roleAPChannel;
		break;

	case ROLE_AP_SET_BEACON_INTERVAL_PARAM:
		pRoleAP->tBssCapabilities.uBeaconInterval = pParam->content.roleAPBeaconInterval;
		break;

	case ROLE_AP_SET_DTIM_PERIOD_PARAM:
		pRoleAP->tBssCapabilities.uDtimPeriod = pParam->content.roleAPDtimPeriod;
		break;


	case ROLE_AP_SET_SSID_PARAM:

		os_memoryZero(pRoleAP->hOs, &pRoleAP->tBssCapabilities.tSsid.str, MAX_SSID_LEN);
		os_memoryCopy(pRoleAP->hOs,
		              pRoleAP->tBssCapabilities.tSsid.str,
		              pParam->content.roleAPSsidName.Ssid,
		              pParam->content.roleAPSsidName.SsidLength);

		pRoleAP->tBssCapabilities.tSsid.len = pParam->content.roleAPSsidName.SsidLength;
		break;

	case ROLE_AP_SET_GENERIC_CMD_TO_FW_PARAM:
		TWD_SendGenricCmdToFW(pRoleAP->hTWD,
		                      pParam->content.tRoleAPGenericCmd.uCmdID,
		                      pParam->content.tRoleAPGenericCmd.buffer,
		                      pParam->content.tRoleAPGenericCmd.len);




		break;
	case ROLE_AP_PRINT_DB:
		PrintRoleApDB(pRoleAP);
		/*Alexa_TODO: move this to WlanLinks*/
		wlanLinks_PrintDB(pRoleAP->hWlanLinks);
		break;

	default:
		break;
	}

	status = TI_OK;;

	return status;
}

/**
 * \fn     roleAP_getParam
 * \brief  roleAP object getParam API
 *
 * Module's object get param API
 */
TI_STATUS roleAP_getParam(TI_HANDLE hRoleAP, paramInfo_t *pParam)
{
    TRoleAP     *pRoleAP = (TRoleAP *)hRoleAP;

    /* check handle validity */
    if (pParam == NULL) {
        return TI_NOK;
    }

    switch (pParam->paramType)
	{
		case ROLE_AP_GET_LINK_COUNTERS:
		{
			TI_UINT32 uHlid;
			TWlanLinkPeerDescr tPeerDescr;
            TLinkDataCounters *pLinkCounters = (TLinkDataCounters*)pParam->content.linkDataCounters;

			/* Get Rx Link Counters*/
            pParam->paramType = RX_DATA_LINK_COUNTERS;
			rxData_getParam(pRoleAP->hRxData, pParam);

			/* Get Rx Link Counters*/
            pParam->paramType = TX_CTRL_GET_DATA_LINK_COUNTER;
			txCtrlParams_getParam(pRoleAP->hTxCtrl, pParam);

                        /* Get Link MAC Address*/
			for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
				pLinkCounters[uHlid].validLink = TI_FALSE;

				if (pRoleAP->aAllocatedLinks[uHlid] != WLANLINKS_INVALID_HLID) {
					if ((wlanLinks_GetPeerDescr(pRoleAP->hWlanLinks, uHlid, &tPeerDescr) != TI_OK) ||
						(wlanLinks_GetLinkType(pRoleAP->hWlanLinks, uHlid, &(pLinkCounters[uHlid].linkType)) != TI_OK))
						continue;

                    pLinkCounters[uHlid].validLink = TI_TRUE;
					if (pLinkCounters[uHlid].linkType == WLANLINK_TYPE_BRCST)
						os_memorySet(pRoleAP->hOs, pLinkCounters[uHlid].aMacAddr, 0xFF, sizeof(TMacAddr));
					else
						os_memoryCopy(pRoleAP->hOs, pLinkCounters[uHlid].aMacAddr, tPeerDescr.aMacAddr, sizeof(TMacAddr));
				}
			}
		}
		break;

		default:
                break;
	}
	return TI_OK;
}


/**
 * \fn     roleAP_start
 * \brief  RolesMgr start command API
 *
 * Start RolesMgr object - send configuration & start cmd to FW
 */
TI_STATUS roleAP_start(TI_HANDLE hRoleAP, TI_UINT8 uBssIdx)
{
	TRoleAP *pRoleAP  = (TRoleAP *)hRoleAP;
	TTwdParamInfo     *pTwdParam;
	paramInfo_t       *pParam;
	TI_STATUS          tRes;
	TTwdParamInfo      tTwdParam;
	int i;
	TRroamingTriggerParams params;


	pRoleAP->tBssCapabilities.uBssIndex = uBssIdx;

	pParam    = os_memoryAlloc(pRoleAP->hOs, sizeof(paramInfo_t));
	pTwdParam = os_memoryAlloc(pRoleAP->hOs, sizeof(TTwdParamInfo));

	pParam->paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pRoleAP->hCtrlData, pParam);
	MAC_COPY (pRoleAP->tBssCapabilities.tBssid, pParam->content.ctrlDataDeviceMacAddress);
	MAC_COPY (pRoleAP->tBssCapabilities.tMacAddress, pParam->content.ctrlDataDeviceMacAddress);

	if (pRoleAP->tBssCapabilities.bUseShortPreamble) {
		TWD_CfgPreamble(pRoleAP->hTWD, PREAMBLE_SHORT);
	}

	if (pRoleAP->tBssCapabilities.bUseShortSlotTime) {
		TWD_CfgSlotTime(pRoleAP->hTWD, PHY_SLOT_TIME_SHORT);
	}

	/* configure TX power in dBm*/
	pTwdParam->paramType = TWD_TX_POWER_PARAM_ID;
	pTwdParam->content.halCtrlTxPowerDbm = regulatoryDomain_getMaxPowerAllowed(pRoleAP->hRegulatoryDomain,
	                                       pRoleAP->tBssCapabilities.uChannel,
	                                       (ERadioBand)pRoleAP->tBssCapabilities.uBand,
	                                       TI_TRUE);

	pTwdParam->content.halCtrlTxPowerDbm = TI_MIN(pRoleAP->uTxPower, pTwdParam->content.halCtrlTxPowerDbm);

	tRes = TWD_SetParam(pRoleAP->hTWD, pTwdParam);
	if (tRes != TI_OK) {
		return tRes;
	}

	/* Configures default Rate Policy */
	ConfigureRatePolicies(hRoleAP);

	/* Configure the FW with frame templates */
	ConfigureFrameTemplates(hRoleAP);


	params.maxTxRetry = AP_MAX_TX_RETRY;
	TWD_CfgMaxTxRetry (pRoleAP->hTWD, &params);

	/*Register for events*/
	TWD_RegisterEvent(pRoleAP->hTWD, TWD_OWN_EVENT_STA_REMOVE_COMPLETE, roleAP_RemoveStaCompleteCB, pRoleAP);
	TWD_EnableEvent  (pRoleAP->hTWD, TWD_OWN_EVENT_STA_REMOVE_COMPLETE);

	TWD_RegisterEvent(pRoleAP->hTWD, TWD_OWN_INACTIVE_STA_EVENT_ID, InactiveStaEventCB, pRoleAP);
	TWD_EnableEvent  (pRoleAP->hTWD, TWD_OWN_INACTIVE_STA_EVENT_ID);

	TWD_RegisterEvent(pRoleAP->hTWD, TWD_OWN_EVENT_MAX_TX_RETRY, maxTxRetryStaEventCB, pRoleAP);
	TWD_EnableEvent  (pRoleAP->hTWD, TWD_OWN_EVENT_MAX_TX_RETRY);

	if (pRoleAP->keyType != KEY_NULL)
		TxDataQ_SetEncryptFlag(pRoleAP->hTxDataQ, pRoleAP->uBrcstHlid, TI_TRUE);

	os_memorySet(pRoleAP->hOs, &pRoleAP->tRoleApStats, 0, sizeof(pRoleAP->tRoleApStats));

	os_memoryFree(pRoleAP->hOs, pParam , sizeof(paramInfo_t));
	os_memoryFree(pRoleAP->hOs, pTwdParam, sizeof(TTwdParamInfo));

	tRes = BssStart(hRoleAP, uBssIdx);
	if (tRes != TI_OK) {
		return tRes;
	}


	for (i=0; i<MAX_WEP_KEY; i++) {
		if (pRoleAP->tTwdKey[i].keyType != KEY_NULL) {
			setKey(pRoleAP,&pRoleAP->tTwdKey[i]);
		}
	}

	if (pRoleAP->keyType == KEY_WEP) {
		tTwdParam.paramType = TWD_RSN_DEFAULT_KEY_ID_PARAM_ID;
		tTwdParam.content.configureCmdCBParams.pCb = &pRoleAP->DefaultKeyIndex;
		tTwdParam.content.configureCmdCBParams.fCb = NULL;
		tTwdParam.content.configureCmdCBParams.hCb = NULL;
		tRes = TWD_SetParam (pRoleAP->hTWD, &tTwdParam);

	}

	/* Set bssid to TxCtrl module for header conversion */
	txCtrlParams_setBssId(pRoleAP->hTxCtrl, &pRoleAP->tBssCapabilities.tBssid);

	/* Synchronize start call back with completion of commands in TWD Command Queue */
	tRes = TWD_NopCmd (pRoleAP->hTWD, BssStartCompleteCb, hRoleAP);

	return tRes;
}



/**
 * \fn     roleAP_RemoveStaCompleteCB
 * \brief  FW Remove STA Complete event callback
 *
 * Start roleAP object - send stop BSS to FW
 */
TI_STATUS roleAP_RemoveStaCompleteCB(TI_HANDLE hRoleAP, TI_CHAR *pData, TI_UINT32 uDataLen)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	TI_UINT8 uHlid = *pData;

	if (uHlid >= WLANLINKS_MAX_LINKS) {
		return TI_NOK;
	}

	if (FreeWlanLink(hRoleAP,uHlid) != TI_OK) {
		return TI_NOK;
	}

	pRoleAP->tRoleApStats.uNumAuthorized--;

	/*Notify the Remove STA CMD caller that remove STA process is complete*/
	if (apCmd_ServiceCompleteCB(pRoleAP->hApCmd, 0, NULL) != TI_OK) {
		return TI_NOK;
	}

	return TI_OK;
}


/**
 * \fn     roleAP_stop
 * \brief  roleAP stop command API
 *
 * Stop roleAP object - disables Tx from network device, stops
 * global & broadcast queueus, sends stop BSS to FW
 */
TI_STATUS roleAP_stop(TI_HANDLE hRoleAP, TI_UINT8 uBssIdx)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	if (pRoleAP->eState	== ROLEAP_STATE_STOPPED) {
		return TI_OK;
	}

	pRoleAP->eState	= ROLEAP_STATE_STOPPED;

	wlanDrvIf_DisableTx (pRoleAP->hOs);
	setRxPortStatus(pRoleAP, CLOSE); /* disable Rx path */

	TWD_DisableEvent (pRoleAP->hTWD, TWD_OWN_EVENT_STA_REMOVE_COMPLETE);
	TWD_DisableEvent (pRoleAP->hTWD, TWD_OWN_INACTIVE_STA_EVENT_ID);
	TWD_DisableEvent (pRoleAP->hTWD, TWD_OWN_EVENT_MAX_TX_RETRY);

	if (RemoveBssLinks(hRoleAP, TI_TRUE) != TI_OK) {
		return TI_NOK;
	}

	ResetRoleApDB(hRoleAP);

	TWD_BssStop(pRoleAP->hTWD, uBssIdx);
	return TI_OK;
}

/**
 * \fn     roleAP_NotifyFwReset
 * \brief  notify the roleAP about FW reset
 *
 */
TI_STATUS roleAP_NotifyFwReset(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	pRoleAP->eState	= ROLEAP_STATE_RECOVERING;

	wlanDrvIf_StopTx(pRoleAP->hOs);
	setRxPortStatus(pRoleAP, CLOSE); /* disable Rx path */

	if (RemoveBssLinks(hRoleAP, TI_FALSE) != TI_OK) {
		return TI_NOK;
	}

	ResetRoleApDB(hRoleAP);

	/*Release the user mode application (it may be currently blocked waiting for the AP Cmd to complete)*/
	/*Note: this function would also command handler, but there wouldn't be queued commands since insert is disabled during the recovery */
	if (apCmd_ServiceCompleteCB(pRoleAP->hApCmd, 0, NULL) != TI_OK) {
		return TI_NOK;
	}

	return TI_OK;
}

TI_STATUS RoleAp_setApCmd(TI_HANDLE hRoleAP, TI_UINT32 cmd, void *pBuffer)
{
	TRoleAP          *pRoleAP = (TRoleAP *)hRoleAP;
	TApGeneralParam  *pGenParams;
	TApChannelParams *pChannelParams;
	TApSsidParam     *pSsidParams;
	TApWpsIe         *pWpsIe;
	TI_STATUS status = TI_OK;
	TSecurityKeys    *pTwdKey;
	TTwdParamInfo    tTwdParam;
	TI_STATUS  tRes = TI_NOK;


	switch (cmd) {
	case ROLE_AP_SET_CHANNEL:
		pChannelParams =  (TApChannelParams*)pBuffer;
		pRoleAP->tBssCapabilities.uChannel = pChannelParams->cChannel;
		break;

	case ROLE_AP_SET_DTIM_PERIOD:
		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.uDtimPeriod = (TI_UINT8)pGenParams->lValue;
		break;

	case ROLE_AP_SET_BEACON_INT:
		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.uBeaconInterval = (TI_UINT16)pGenParams->lValue;
		break;

	case ROLE_AP_USE_CTS_PROT: {
		TTwdParamInfo    *pTwdParam = os_memoryAlloc(pRoleAP->hOs, sizeof(TTwdParamInfo));;

		if (!pTwdParam) {
			return TI_NOK;
		}
		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.bUseProtection = (TI_BOOL)pGenParams->lValue;
		pTwdParam->paramType = TWD_CTS_TO_SELF_PARAM_ID;
		pTwdParam->content.halCtrlCtsToSelf = (pRoleAP->tBssCapabilities.bUseProtection) ? CTS_TO_SELF_ENABLE : CTS_TO_SELF_DISABLE;


		TWD_SetParam (pRoleAP->hTWD, pTwdParam);

		os_memoryFree(pRoleAP->hOs, pTwdParam, sizeof(TTwdParamInfo));
	}
	break;

	case ROLE_AP_SET_RTS:
		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.uRtsThreshold = (TI_UINT16)pGenParams->lValue;
		break;

	case ROLE_AP_SET_AP_SHORT_PREAMBLE: {
		EPreamble ePreamble;

		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.bUseShortPreamble = (TI_BOOL)pGenParams->lValue;

		ePreamble = (pRoleAP->tBssCapabilities.bUseShortPreamble) ? PREAMBLE_SHORT : PREAMBLE_LONG;

		TWD_CfgPreamble (pRoleAP->hTWD, ePreamble);
	}
	break;

	case ROLE_AP_SET_PRIVACY:
		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.bPrivacyEnabled = (TI_BOOL)pGenParams->lValue;
		break;

	case ROLE_AP_USE_SHORT_SLOT_TIME: {
		ESlotTime eSlotTime;

		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->tBssCapabilities.bUseShortSlotTime = (TI_BOOL)pGenParams->lValue;

		eSlotTime = (pRoleAP->tBssCapabilities.bUseShortSlotTime) ? PHY_SLOT_TIME_SHORT : PHY_SLOT_TIME_LONG;

		TWD_CfgSlotTime(pRoleAP->hTWD, eSlotTime);
	}
	break;

	case ROLE_AP_SET_RATE:
		SetApRates(hRoleAP, pBuffer);
		break;

	case ROLE_AP_CHANGE_BSS_PARAM:
		/* [LiorC] To be implemented later in hostapd */
		break;

	case ROLE_AP_SET_TX_PARAM: {
		TApTxParams *pTxParams = (TApTxParams*)pBuffer;
		TAcQosParams tAcQosParams;

		tAcQosParams.ac = (TI_UINT8)pTxParams->cQueueId; /* [LiorC] check if need to be converted */
		tAcQosParams.aifsn = (TI_UINT8)pTxParams->cAifs;
		tAcQosParams.cwMin = (TI_UINT8)pTxParams->sCwmin;
		tAcQosParams.cwMax = (TI_UINT16)pTxParams->sCwmax;
		tAcQosParams.txopLimit = (TI_UINT16)pTxParams->sTxop;

		TWD_CfgAcParams(pRoleAP->hTWD, &tAcQosParams, NULL, NULL);
	}

	break;

	case ROLE_AP_ADD_BEACON_PARAM:
		os_memoryCopy(pRoleAP->hOs, &pRoleAP->tBssCapabilities.tAPBeaconParams, (TApBeaconParams*)pBuffer, sizeof(TApBeaconParams));

		if (pRoleAP->eState == ROLEAP_STATE_STARTED) {
			/* Templates are to be updated in FW when AP Role is in started state.
			 * RoleAp_Start takes care to download templates from BssCapabilities in stoped state
			 */
			setBeaconProbeRspTempl(pRoleAP);
		}

		break;

	case ROLE_AP_SET_SSID:
		pSsidParams = (TApSsidParam*)pBuffer;
		pRoleAP->tBssCapabilities.tSsid.len = pSsidParams->iSsidLen;
		os_memoryCopy(pRoleAP->hOs,pRoleAP->tBssCapabilities.tSsid.str,pSsidParams->cSsid,pSsidParams->iSsidLen);
		break;

	case ROLE_AP_SET_SSID_TYPE: {
		TApGeneralParam *pSsidTypeParam = (TApGeneralParam *)pBuffer;
		pRoleAP->tBssCapabilities.eSsidType = (pSsidTypeParam->lValue == AP_SSID_TYPE_PUBLIC) ?
		                                      SSID_TYPE_PUBLIC : SSID_TYPE_HIDDEN;
	}
	break;

	case ROLE_AP_SET_INACTIVE_INT:
		/* [LiorC] To be implemented later in hostapd */
		break;

	case ROLE_AP_CHANGE_STATION_PARAM:
		/* [LiorC] To be implemented later in hostapd */
		break;

	case ROLE_AP_SET_PORT_STATUS:
		status = SetPortStatus(pRoleAP, (TApGeneralParam *)pBuffer);
		break;

	case ROLE_AP_COMMIT_CMD: {
		TApGeneralParam *pGenParam = (TApGeneralParam *)pBuffer;
		pRoleAP->tBssCapabilities.uInactivity = pGenParam->lValue;
		roleAP_start(hRoleAP, pRoleAP->tBssCapabilities.uBssIndex);
	}
	break;

	case ROLE_AP_STOP:
		roleAP_stop(hRoleAP, pRoleAP->tBssCapabilities.uBssIndex);
		break;

	case ROLE_AP_ADD_STATION_PARAM:
		status = AddStation(hRoleAP, pBuffer);
		break;

	case ROLE_AP_REMOVE_STATION:
		status = RemStation(hRoleAP, pBuffer);
		break;

	case ROLE_AP_SET_STA_WME:
		break;

	case ROLE_AP_SET_BSS_BRIDGE: {
		TApGeneralParam *pBssBridgeParam = (TApGeneralParam *)pBuffer;
		if(pBssBridgeParam->lValue) {
			rxData_IntraBssBridge_Enable(pRoleAP->hRxData);
		} else {
			rxData_IntraBssBridge_Disable(pRoleAP->hRxData);
		}
	}
	break;

	case ROLE_AP_DEAUTH_STATION:
		status = SaveDeauthReason(hRoleAP, (TApGeneralParam *)pBuffer);
		break;

	case ROLE_AP_SET_PROBE_WPS_IE:
		pWpsIe = (TApWpsIe*)pBuffer;
		pRoleAP->tWpsIe.iIeLen = pWpsIe->iIeLen;
		os_memoryCopy(pRoleAP->hOs,pRoleAP->tWpsIe.cIe,pWpsIe->cIe,pWpsIe->iIeLen);
		break;

	case TWD_ADD_KEY_PARAMS:
		pTwdKey = (TSecurityKeys*)pBuffer;
		pRoleAP->keyType = pTwdKey->keyType;

		// in case of wep or broadcst key keep the keys in roleAp and set the FW after BssConfig.
		if (((pTwdKey->lidKeyType == BROADCAST_LID_TYPE)|| (pTwdKey->keyType == KEY_WEP)) &&
		    (pRoleAP->eState != ROLEAP_STATE_STARTED)) {
			os_memoryCopy(pRoleAP->hOs, &pRoleAP->tTwdKey[pTwdKey->keyIndex],pBuffer, sizeof(TSecurityKeys));
		} else {
			status = setKey(pRoleAP,pTwdKey);
			TxMgmtQ_SetEncryptFlag(pRoleAP->hTxMgmtQ,pTwdKey->hlid,TI_TRUE);
		}

		break;

	case TWD_DEL_KEY_PARAMS:
		pTwdKey = (TSecurityKeys*)pBuffer;
		pTwdKey->keyType = pRoleAP->keyType ;
		tTwdParam.paramType = TWD_RSN_KEY_REMOVE_PARAM_ID;
		tTwdParam.content.configureCmdCBParams.pCb = (TI_UINT8*)pTwdKey;
		tTwdParam.content.configureCmdCBParams.fCb = NULL;
		tTwdParam.content.configureCmdCBParams.hCb = NULL;
		tRes = TWD_SetParam (pRoleAP->hTWD, &tTwdParam);

		TxMgmtQ_SetEncryptFlag(pRoleAP->hTxMgmtQ,pTwdKey->hlid,TI_FALSE);
		TxDataQ_setEncryptionFieldSizes(pRoleAP->hTxDataQ,pTwdKey->hlid,0);

		break;

	case TWD_SET_DEFAULT_KEY_PARAMS:
		pGenParams = (TApGeneralParam*)pBuffer;
		pRoleAP->DefaultKeyIndex = pGenParams->lValue;

		break;

	case ROLE_AP_REMOVE_ALL_STATION:
	case ROLE_AP_SET_STA_SHORT_PREAMBLE:
		/* Not supported for now */
		break;

	case ROLE_AP_ENABLE:
		status = roleAP_enable(hRoleAP);
		break;

	case ROLE_AP_SET_TX_POWER:
		pGenParams = (TApGeneralParam *)pBuffer;
		pRoleAP->uTxPower = pGenParams->lValue;
         break;

	case TWD_SET_CONNECTION_PHASE: {
		TTwdConnPhaseParam	tTwdConnPhaseParam;
		pGenParams = (TApGeneralParam *)pBuffer;

        os_memoryCopy(pRoleAP->hOs, tTwdConnPhaseParam.aMacAddr, pGenParams->cMac, AP_MAC_ADDR);

        tRes = TWD_SetConnectionPhase(pRoleAP->hTWD, &tTwdConnPhaseParam, NULL, NULL);
	}
		break;

	default:
		status = TI_NOK;
		break;
	}

	return status;
}


TI_STATUS RoleAp_getApCmd(TI_HANDLE hRoleAP, TI_UINT32 cmd, void *pInBuf, void *pOutBuf)
{
	TRoleAP          *pRoleAP = (TRoleAP *)hRoleAP;
	TI_STATUS status = TI_OK;
	paramInfo_t      tParam;
	TApChanHwInfo  *pChanData;

	switch (cmd) {
	case ROLE_AP_GET_STATION_PARAM:
		/* [LiorC] To be implemented later. */
		break;

	case ROLE_AP_GET_STA_INACTIVITY:
		staInactivity (hRoleAP, pInBuf, pOutBuf);
		break;

	case ROLE_AP_GET_HW:
		pChanData = (TApChanHwInfo*)pInBuf;
		tParam.paramType = SITE_MGR_NETWORK_TYPE_IN_USE;
		siteMgr_getParam(pRoleAP->hSiteMgr,&tParam);
		pChanData->mode = tParam.content.siteMgrDot11Mode;
		regDomain_GetActiveChannel(pRoleAP->hRegulatoryDomain,pChanData);
		os_memoryCopy(pRoleAP->hOs, pOutBuf, pInBuf, sizeof(TApChanHwInfo));
		break;

	default:
		status = TI_NOK;
		break;
	}

	return status;
}


/**
 * \fn     roleAp_getApState
 * \brief  get Role AP current state
 *
 */
TI_STATUS roleAp_getApState(TI_HANDLE hRoleAP, TI_UINT32 *pState)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	*pState = pRoleAP->eState;

	return TI_OK;
}

/**
 * \fn     RoleAp_DrvResetNotifyUpperLayers
 * \brief  Send driver reset event to user application - hostapd
 * \return  operation status
 */
TI_STATUS RoleAp_DrvResetNotifyUpperLayers(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	TApEvent    tApEvent;

	tApEvent.uEvent = (unsigned char) AP_EVENT_DRV_RESET;


	return EvHandlerSendEvent (pRoleAP->hEvHandler, IPC_EVENT_AP_EVENT, (TI_UINT8*) &tApEvent, sizeof (tApEvent));
}

/**************************************************************************/
/*      STATIC FUNCTIONS                                                   */
/**************************************************************************/

/**
 * \fn     BssStart
 * \brief  roleAP bss configure & start command
 *
 * Configures the BSS with the params to be received from Hostapd/INI file
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 * \param   uBssIdx - the BSS Index
 *
 * \return  TI_OK if success , otherwise - TI_NOK
 * \sa      roleAP_start()
 */
static TI_STATUS BssStart(TI_HANDLE hRoleAP, TI_UINT8 uBssIdx)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	BssStartCmd_t       bssStartPrm;

	os_memoryZero(pRoleAP->hOs, &bssStartPrm, sizeof(bssStartPrm));

	bssStartPrm.beaconInterval  = pRoleAP->tBssCapabilities.uBeaconInterval;
	bssStartPrm.dtimInterval    = pRoleAP->tBssCapabilities.uDtimPeriod;
	bssStartPrm.agingPeriod     = pRoleAP->tBssCapabilities.uInactivity;
	bssStartPrm.band            = (RadioBand_e)pRoleAP->tBssCapabilities.uBand;
	bssStartPrm.channelNumber   = pRoleAP->tBssCapabilities.uChannel;

	os_memoryCopy(pRoleAP->hOs, &bssStartPrm.ssid.ssid, &pRoleAP->tBssCapabilities.tSsid.str, MAX_SSID_LEN);
	bssStartPrm.ssid.ssidLength = pRoleAP->tBssCapabilities.tSsid.len;
	bssStartPrm.ssid.ssidType   = pRoleAP->tBssCapabilities.eSsidType;

	MAC_COPY(bssStartPrm.bssid, pRoleAP->tBssCapabilities.tBssid);

	bssStartPrm.globalHLID      = pRoleAP->uGlobalHlid;
	bssStartPrm.broadcastHLID   = pRoleAP->uBrcstHlid;
	bssStartPrm.bssIndex        = uBssIdx;
	bssStartPrm.basicRateSet    = pRoleAP->tBssCapabilities.uBasicRateBitmap;
	bssStartPrm.beaconExpiry	= pRoleAP->uBeaconTxTimeout;

	return TWD_BssStart(pRoleAP->hTWD, &bssStartPrm);
}





/**
 * \fn     ConvertRateSetToBitmap
 * \brief  roleAP Conver Rateset command
 *
 * Convert the basic rates array received by Hostapd (100k units format) to FW bitmap style
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 * \param   pRates - rates array
 * \param   pRatesLen - rates array length
 *
 * \return  TI_OK if success , otherwise - TI_NOK
 * \sa      roleAP_start()
 */
static EHwRateBitFiled ConvertRateSetToBitmap(TI_HANDLE hRoleAP, TI_UINT8 *pRates, TI_UINT8 uRatesLen)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	EHwRateBitFiled eRateSetBitmap = 0;
	TI_UINT8 i;

	for (i=0 ; i< uRatesLen ; i++) {
		eRateSetBitmap |=  TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRates[i]);
	}

	return eRateSetBitmap;
}


/**
 * \fn     AllocateWlanLink
 * \brief Allocate Wlan Link for the station
 *
 *
 * \param   hRoleAP - Handle to RoleAP object
 * \param   eLinkType - link type
 * \param   pStationParams - station parameters
 * \param   pHlid - output HLID
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS AllocateWlanLink(TI_HANDLE hRoleAP, EWlanLinkType eLinkType, TApStationParams *pStaParams, TI_UINT32 *pHlid)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	TWlanLinkInfo    linkInfo;
	TI_UINT32        uHlid;

	FillLinkInfo(hRoleAP, eLinkType, pStaParams, &linkInfo);

	/* Add the station to WlanLinks DB*/
	if (wlanLinks_AllocateNew(pRoleAP->hWlanLinks, &linkInfo, &uHlid) != TI_OK) {
		*pHlid = WLANLINKS_INVALID_HLID;
		return TI_NOK;
	}

	*pHlid = uHlid;

	/* Update my links */
	pRoleAP->aAllocatedLinks[uHlid] = uHlid;
	pRoleAP->uNumAllocatedLinks ++;

	/* Update link type in  Rx and Tx */
	txMgmtQ_SetLinkType(pRoleAP->hTxMgmtQ, uHlid, eLinkType);
	rxData_SetLinkType(pRoleAP->hRxData, uHlid, eLinkType, WLANLINK_ROLE_AP);

	return TI_OK;
}

/**
 * \fn     FreeWlanLink
 * \brief  Free Wlan Link of the AP
 *
 *
 * \param   hRoleAP - Handle to RoleAP object
 * \param   uHlid - link HLID
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS FreeWlanLink(TI_HANDLE hRoleAP, TI_UINT32 uHlid)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	if (pRoleAP->keyType != KEY_NULL) {
		/* TODO save this flag in Wilink DB */
		TxDataQ_SetEncryptFlag(pRoleAP->hTxDataQ,uHlid,TI_FALSE);
		TxDataQ_setEncryptionFieldSizes(pRoleAP->hTxDataQ,uHlid,0);
	}

	/*Free the Wlan Link*/
	if (wlanLinks_FreeLink(pRoleAP->hWlanLinks, (TI_UINT32)uHlid) != TI_OK) {
		return TI_NOK;
	}

	rxData_resetLinkCounters(pRoleAP->hRxData, pRoleAP->aAllocatedLinks[uHlid]);
	txCtrlParams_resetLinkCounters(pRoleAP->hTxCtrl, pRoleAP->aAllocatedLinks[uHlid]);
	/* Update my links */
	pRoleAP->aAllocatedLinks[uHlid] = WLANLINKS_INVALID_HLID;
	pRoleAP->uNumAllocatedLinks--;


	return TI_OK;
}

/**
 * \fn     AddStation
 * \brief  add new station in BSS
 *
 * Add station to WlanLinks DB, DataPath tables and FW DB
 *
 * \param   hRoleAP - Handle to RoleAP object
 * \param   pStationParams - station parameters
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS AddStation(TI_HANDLE hRoleAP, void *pStationParams)
{
	TRoleAP            *pRoleAP  =  (TRoleAP *)hRoleAP;
	TI_UINT32          uHlid;
	TApStationParams   *pStaParams = (TApStationParams *)pStationParams;


	pRoleAP->tRoleApStats.uAddStaCmds ++;

	if (!pStaParams) {
		return TI_NOK;
	}

	DecodeStaRates(pStaParams->cSupportedRates, pStaParams->cSupportedRatesLen);

	/*Check if the station already exists in WlanLinks DB*/
	if (wlanLinks_FindLinkByMac(pRoleAP->hWlanLinks, pStaParams->cMac, &uHlid) == TI_OK) {
		TWlanLinkInfo    linkInfo;


		FillLinkInfo(hRoleAP, WLANLINK_TYPE_SPECIFIC, pStaParams,&linkInfo);

		/* Update the station in WlanLinks DB*/
		if (wlanLinks_UpdateLinkInfo(pRoleAP->hWlanLinks, &linkInfo, uHlid) != TI_OK) {
			return TI_NOK;
		}

	} else {
		/*Allocate new regular link in Wlan Links*/
		if (AllocateWlanLink(hRoleAP, WLANLINK_TYPE_SPECIFIC, pStaParams, &uHlid) != TI_OK) {
			return TI_NOK;
		}

	}

	/* Set link state CONNECTING till we receive port authorized notification from AP manager */
	if (wlanLinks_SetLinkState(pRoleAP->hWlanLinks, WLANLINK_STATE_CONNECTING, uHlid) != TI_OK) {
		return TI_NOK;
	}

	/* Set link state to OPEN in Rx and Tx */
	rxData_SetLinkState(pRoleAP->hRxData, uHlid, RX_CONN_STATE_OPEN);

	/*Update the Data Path*/
	if (txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, uHlid, TX_CONN_STATE_EAPOL) != TI_OK) {
		return TI_NOK;
	}

	if (txDataQ_LinkMacAdd(pRoleAP->hTxDataQ, uHlid, pStaParams->cMac) != TI_OK) {
		return TI_NOK;
	}

	/*Update the FW*/
	if (AddStaToFW(pRoleAP, uHlid, pStaParams) != TI_OK) {
		return TI_NOK;
	}


	/* set Data Encrypt bit for not OPEN system */
	if (pRoleAP->keyType != KEY_NULL) {
		TxDataQ_SetEncryptFlag(pRoleAP->hTxDataQ,uHlid,TI_TRUE);
	}


	return TI_OK;
}

/**
 * \fn     AddStaToFW
 * \brief  Add station to FW
 *
 * \param   pRoleAP - Pinter to RoleAP object
 * \param   uHlid   - station HLID
 * \param pStationParams - station parameters buffer
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS AddStaToFW(TRoleAP *pRoleAP, TI_UINT32 uHlid, TApStationParams *pStaParams)
{
	TTwdAddStaParams   twdAddStaParams;

	os_memoryCopy(pRoleAP->hOs, twdAddStaParams.aMacAddr, pStaParams->cMac, AP_MAC_ADDR);
	twdAddStaParams.uHlid = uHlid;
	twdAddStaParams.uAid = pStaParams->sAid;
	twdAddStaParams.uBssIndex = pRoleAP->tBssCapabilities.uBssIndex;
	twdAddStaParams.bWme = ((pStaParams->sCapability & DOT11_CAPS_QOS_SUPPORTED) ? TI_TRUE : TI_FALSE);
	wlanLinks_GetLinkSupRates(pRoleAP->hWlanLinks, &twdAddStaParams.uSupRates, uHlid);

	/* By now our device doesn't support UAPSD, set the UAPSD parameters to 0*/
	os_memorySet(pRoleAP->hOs, twdAddStaParams.aPSDType, 0, sizeof(twdAddStaParams.aPSDType));
	twdAddStaParams.uSPLen = 0;

	return TWD_AddSta(pRoleAP->hTWD, &twdAddStaParams);
}

/**
 * \fn     RemStaFromFW
 * \brief  Remove station from FW
 *
 * \param   pRoleAP - Pinter to RoleAP object
 * \param   uHlid   - station HLID
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS RemStaFromFW(TRoleAP *pRoleAP, TI_UINT32 uHlid)
{
	TI_UINT32 uDeauthReason = 0;
	TI_BOOL	  bSendDeauth;

	if (wlanLinks_GetDeauthReason(pRoleAP->hWlanLinks, &uDeauthReason, uHlid))
		return TI_NOK;

	bSendDeauth = uDeauthReason ? TI_TRUE : TI_FALSE;

	return TWD_RemSta(pRoleAP->hTWD, uHlid, uDeauthReason, bSendDeauth);
}

/**
 * \fn     RemStation
 * \brief  remove a station
 *
 * Remove a station from WlanLinks DB, DataPath tables and FW DB
 *
 * \param   hRoleAP - Handle to RoleAP object
 * \param   pBuf - command buffer
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS RemStation(TI_HANDLE hRoleAP, void *pBuf)
{
	TRoleAP          *pRoleAP    =  (TRoleAP *)hRoleAP;
	TI_UINT32        uHlid;
	TI_UINT8         *aMacAddr = (TI_UINT8 *)pBuf;


	pRoleAP->tRoleApStats.uRemStaCmds ++;

	/*Check if the station exists in WlanLinks DB*/
	if (wlanLinks_FindLinkByMac(pRoleAP->hWlanLinks, aMacAddr, &uHlid) != TI_OK) {
		return TI_NOK;
	}

	if (txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, uHlid, TX_CONN_STATE_CLOSE) != TI_OK) {
		return TI_NOK;
	}

	rxData_SetLinkState(pRoleAP->hRxData, uHlid, RX_CONN_STATE_CLOSE);
	txDataQ_LinkMacRemove(pRoleAP->hTxDataQ, uHlid);

	/*Update the FW*/
	if (RemStaFromFW(pRoleAP, uHlid) != TI_OK) {
		return TI_NOK;
	}


	/*return Pending status in order to block the caller application till FW Remove Complete event occurs */
	return COMMAND_PENDING;
}

/**
 * \fn     SaveDeauthReason
 * \brief  save deauthentication reason for a station
 *
 * \param   hRoleAP - Handle to RoleAP object
 * \param   pGenParam - command structure
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS SaveDeauthReason(TI_HANDLE hRoleAP, TApGeneralParam *pGenParam)
{
	TRoleAP          *pRoleAP    =  (TRoleAP *)hRoleAP;
	TI_UINT8         *aMacAddr = (TI_UINT8 *)pGenParam->cMac;
	TI_UINT32		 uHlid;


	/*Check if the station exists in WlanLinks DB*/
	if (wlanLinks_FindLinkByMac(pRoleAP->hWlanLinks, aMacAddr, &uHlid) != TI_OK) {
		return TI_NOK;
	}

	if (wlanLinks_SetDeauthReason(pRoleAP->hWlanLinks, pGenParam->lValue, uHlid) != TI_OK) {
		return TI_NOK;
	}

	return TI_OK;
}

/**
 * \fn     SetPortStatus
 * \brief  Set port authorization status
 *
 * \param   pRoleAP - A pointer to RoleAP object
 * \param tGenParam - General Ap Command parameter structure
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS SetPortStatus(TRoleAP *pRoleAP , TApGeneralParam *tGenParam)
{
	TI_BOOL bAuthorized = (tGenParam->lValue ? TI_TRUE : TI_FALSE);
	TI_UINT8         *aMacAddr = (TI_UINT8 *)tGenParam->cMac;
	TI_UINT32		 uHlid;
	EWlanLinkState   eState;

	/*Check if the station exists in WlanLinks DB*/
	if (wlanLinks_FindLinkByMac(pRoleAP->hWlanLinks, aMacAddr, &uHlid) != TI_OK) {
		return TI_NOK;
	}

	if (wlanLinks_GetLinkState(pRoleAP->hWlanLinks, uHlid, &eState) != TI_OK) {
		return TI_NOK;
	}

	if (bAuthorized) {
		if (eState != WLANLINK_STATE_CONNECTED) {
			txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, uHlid, TX_CONN_STATE_OPEN);
			rxData_SetLinkState(pRoleAP->hRxData, uHlid, RX_CONN_STATE_OPEN);
			/* Set link state CONNECTED since we received port authorized notification from AP manager */
			if (wlanLinks_SetLinkState(pRoleAP->hWlanLinks, WLANLINK_STATE_CONNECTED, uHlid) != TI_OK) {
				return TI_NOK;
			}
			pRoleAP->tRoleApStats.uNumAuthorized++;
		}
	} else {
		if (eState != WLANLINK_STATE_CONNECTING) {
			txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, uHlid, TX_CONN_STATE_MGMT);
			txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, uHlid, TX_CONN_STATE_EAPOL); /* TODO[ilanb]: add it to enable EAPOL queue */
			rxData_SetLinkState(pRoleAP->hRxData, uHlid, RX_CONN_STATE_OPEN);
			/* Set link state CONNECTING since we received port authorized notification from AP manager */
			if (wlanLinks_SetLinkState(pRoleAP->hWlanLinks, WLANLINK_STATE_CONNECTING, uHlid) != TI_OK) {
				return TI_NOK;
			}
			pRoleAP->tRoleApStats.uNumAuthorized--;
		}
	}


	return  TI_OK;
}

/**
 * \fn     InitBssLinks
 * \brief  Initialize the links of the Role AP
 *
 * \param   hRoleAP - A handle to RoleAP object
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS InitBssLinks(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	TApStationParams tStaParams;
	int i;

	for (i=0; i<WLANLINKS_MAX_LINKS; i++) {
		pRoleAP->aAllocatedLinks[i] = WLANLINKS_INVALID_HLID;
	}
	pRoleAP->uNumAllocatedLinks = 0;
	pRoleAP->uBrcstHlid = pRoleAP->uGlobalHlid = WLANLINKS_INVALID_HLID;

	/*Allocate the links for Broadcast and Global traffic*/
	os_memorySet(pRoleAP->hOs, &tStaParams, 0, sizeof(tStaParams));

	/* Configure basic rates for global traffic */
	os_memoryCopy(pRoleAP->hOs, tStaParams.cSupportedRates, pRoleAP->tBssCapabilities.aBasicRateSet, pRoleAP->tBssCapabilities.uNumBasicRates);
	tStaParams.cSupportedRatesLen = pRoleAP->tBssCapabilities.uNumBasicRates;

	if (AllocateWlanLink(hRoleAP, WLANLINK_TYPE_GLOBAL, &tStaParams, &pRoleAP->uGlobalHlid) != TI_OK) {
		return TI_NOK;
	}

	/* Configure MIN basic rate for broadcast traffic */
	tStaParams.cSupportedRates[0] = pRoleAP->tBssCapabilities.uMinBasicRate;
	tStaParams.cSupportedRatesLen = 1;

	if (AllocateWlanLink(hRoleAP, WLANLINK_TYPE_BRCST, &tStaParams, &pRoleAP->uBrcstHlid) != TI_OK) {
		return TI_NOK;
	}

	return TI_OK;
}

/**
 * \fn     RemoveBssLinks
 * \brief  Remove the links of the Role AP
 *
 * \param   hRoleAP - A handle to RoleAP object
 * \param   bRemFromFw - If remove from FW is required
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static TI_STATUS RemoveBssLinks(TI_HANDLE hRoleAP, TI_BOOL bRemFromFw)
{
	TI_UINT32 uHlid;
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;
	EWlanLinkType eLinkType;


	for (uHlid=0; uHlid<WLANLINKS_MAX_LINKS; uHlid++) {
		if (pRoleAP->aAllocatedLinks[uHlid] != WLANLINKS_INVALID_HLID) {
			wlanLinks_GetLinkType(pRoleAP->hWlanLinks,  uHlid, &eLinkType);

			if (bRemFromFw && (eLinkType == WLANLINK_TYPE_SPECIFIC)) {
				if (RemStaFromFW(pRoleAP, uHlid) != TI_OK) {
					return TI_NOK;
				}
			}


			if (FreeWlanLink(hRoleAP, uHlid) != TI_OK) {
				return TI_NOK;
			}
			txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, uHlid, TX_CONN_STATE_CLOSE);
			rxData_SetLinkState(pRoleAP->hRxData, uHlid, RX_CONN_STATE_CLOSE);

			if (eLinkType == WLANLINK_TYPE_SPECIFIC)
				txDataQ_LinkMacRemove(pRoleAP->hTxDataQ, uHlid);
		}
	}
	pRoleAP->uBrcstHlid = WLANLINKS_INVALID_HLID;
	pRoleAP->uGlobalHlid = WLANLINKS_INVALID_HLID;

	return TI_OK;
}

/**
 * \fn     FillLinkInfo
 * \brief  Fill Wlan Link Info structure
 *
 * \param hRoleAP - A handle to RoleAP object
 * \param eLinkType - link type
 * \param pStationParams - station parameters buffer
 * \param pLinkInfo - output structure
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
static void FillLinkInfo(TI_HANDLE hRoleAP, EWlanLinkType eLinkType, TApStationParams *pStaParams, TWlanLinkInfo *pLinkInfo)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	pLinkInfo->eRole = WLANLINK_ROLE_AP;
	pLinkInfo->eSubRole = WLANLINK_SUBROLE_APLIKE;
	pLinkInfo->eType = eLinkType;

	MAC_COPY(pLinkInfo->tPeerDescr.aMacAddr, pStaParams->cMac);
	os_memoryCopy(pRoleAP->hOs, &pLinkInfo->tPeerDescr.tSsid, &pRoleAP->tBssCapabilities.tSsid, sizeof(pLinkInfo->tPeerDescr.tSsid));
	pLinkInfo->tPeerDescr.uBand = pRoleAP->tBssCapabilities.uBand;
	pLinkInfo->tPeerDescr.uChannel = pRoleAP->tBssCapabilities.uChannel;
	pLinkInfo->tPeerDescr.uCapablities = pStaParams->sCapability;
	pLinkInfo->tPeerDescr.uSupRatesBitmap = ConvertRateSetToBitmap(hRoleAP, &pStaParams->cSupportedRates[0], pStaParams->cSupportedRatesLen);
	pLinkInfo->uAid   = pStaParams->sAid;

	/*ALEXA_TODO: where to take the fields below values from*/
	pLinkInfo->uQuality = 0;
	pLinkInfo->uTxPow   = 0;
}


/* Fill the template based on beacon received from Hostapd */
static TI_STATUS FillBeaconTemplate(TI_HANDLE hRoleAP, TApBeaconParams *pAPBeaconParams, TSetTemplate *pTemplateStruct)
{
	TRoleAP *pRoleAP  = (TRoleAP *)hRoleAP;
	dot11_TIM_Short_t *pTimIE;
	TI_UINT32 uTempSize = 0;

	/************ Copy the beacon's Head section *********************/
	os_memoryCopy(pRoleAP->hOs, pTemplateStruct->ptr, pAPBeaconParams->cHead, pAPBeaconParams->iHeadLen);
	uTempSize += pAPBeaconParams->iHeadLen;

	/* Add TIM IE (Short style - limited to only 8 station in AP Mode !!)  */
	pTimIE = (dot11_TIM_Short_t *)(pTemplateStruct->ptr + uTempSize);
	pTimIE->hdr[0] = DOT11_TIM_ELE_ID;
	pTimIE->hdr[1] = DOT11_TIM_SHORT_PARAMS_ELE_LEN ;
	pTimIE->dtimCount = 0;
	pTimIE->dtimPeriod = 0;
	pTimIE->bmapControl = 0;
	pTimIE->partialVirtualBmap = 0;

	uTempSize += DOT11_TIM_SHORT_PARAMS_ELE_LEN + sizeof(dot11_eleHdr_t);

	/* Copy the beacon's Tail section */
	os_memoryCopy(pRoleAP->hOs, pTemplateStruct->ptr + uTempSize, pAPBeaconParams->cTail, pAPBeaconParams->iTailLen);
	uTempSize += pAPBeaconParams->iTailLen;

	pTemplateStruct->len = uTempSize;

	((beaconTemplate_t*)pTemplateStruct->ptr)->hdr.fc = ENDIAN_HANDLE_WORD(DOT11_FC_BEACON);


	return TI_OK;
}

/* Fill the template based on beacon received from Hostapd */
static TI_STATUS FillProbeRespTemplate(TI_HANDLE hRoleAP, TApBeaconParams *pAPBeaconParams, TSetTemplate *pTemplateStruct)
{
	TRoleAP *pRoleAP    = (TRoleAP *)hRoleAP;
	dot11_SSID_t          *pSsidIE;
	TI_UINT32 uTempSize = 0;

	/************ Copy the Probe Response Head section *********************/

	if (pRoleAP->tBssCapabilities.eSsidType == SSID_TYPE_PUBLIC) {
		os_memoryCopy(pRoleAP->hOs, pTemplateStruct->ptr, pAPBeaconParams->cHead, pAPBeaconParams->iHeadLen);
		uTempSize += pAPBeaconParams->iHeadLen;
	} else {
		/* Hidden SSID mode, set SSID IE in Probe Response */
		/* Copy header up to SSID IE */
		uTempSize += (WLAN_HDR_LEN + TIME_STAMP_LEN + 4 /* beacon interval and capabilities */);
		os_memoryCopy(pRoleAP->hOs, pTemplateStruct->ptr, pAPBeaconParams->cHead, uTempSize);

		/* Set SSID in Probe Response */
		pSsidIE = (dot11_SSID_t *)(pTemplateStruct->ptr + uTempSize);
		pSsidIE->hdr[0] = DOT11_SSID_ELE_ID;
		pSsidIE->hdr[1] = pRoleAP->tBssCapabilities.tSsid.len;
		os_memoryCopy(pRoleAP->hOs, pSsidIE->serviceSetId, pRoleAP->tBssCapabilities.tSsid.str, pRoleAP->tBssCapabilities.tSsid.len);

		/* Copy header reminder exept hidden SSID */
		os_memoryCopy(pRoleAP->hOs, pTemplateStruct->ptr + uTempSize + sizeof(dot11_eleHdr_t) + pSsidIE->hdr[1],
		              pAPBeaconParams->cHead + uTempSize + sizeof(dot11_eleHdr_t),
		              pAPBeaconParams->iHeadLen - uTempSize - sizeof(dot11_eleHdr_t));
		uTempSize = pAPBeaconParams->iHeadLen + pSsidIE->hdr[1];
	}

	/* Copy the Probe Response Tail section */

	/* insert WPS IE if exists */
	if (pRoleAP->tWpsIe.iIeLen != 0) {

		TI_UINT8 *pPos = pAPBeaconParams->cTail;
		TI_UINT8 *pEnd = pAPBeaconParams->cTail + pAPBeaconParams->iTailLen;
		TI_UINT8 *pWps = NULL;
		TI_UINT8 *pCopyPos = NULL;

#define WLAN_EID_VENDOR_SPECIFIC 221
#define WPS_DEV_OUI_WFA 0x0050f204
#define WPA_GET_BE32(a) ((((u32) (a)[0]) << 24) | (((u32) (a)[1]) << 16) | \
			 (((u32) (a)[2]) << 8) | ((u32) (a)[3]))

		while (pPos + 1 < pEnd) {
			if (pPos + 2 + pPos[1] > pEnd) {
				WLAN_OS_REPORT((" FillProbeRespTemplate(): wrong size of IE\n"));
				break;
			}
			if (pPos[0] == WLAN_EID_VENDOR_SPECIFIC && pPos[1] >= 4 &&
			    WPA_GET_BE32(&pPos[2]) == WPS_DEV_OUI_WFA) {
				pWps = pPos;
				WLAN_OS_REPORT((" FillProbeRespTemplate(): found WPS IE\n"));
				break;
			}
			pPos += 2 + pPos[1];
		}

		if (!pWps) {
			WLAN_OS_REPORT((" FillProbeRespTemplate(): no WPS IE was found in beacon, this is likely an error\n"));
		}

		// IDAN, TODO, add error handling

		/* copy first part of tail */
		pCopyPos = pTemplateStruct->ptr + uTempSize;
		os_memoryCopy(pRoleAP->hOs, pCopyPos, pAPBeaconParams->cTail, (char *)pWps - pAPBeaconParams->cTail);
		pCopyPos += (char *)pWps-pAPBeaconParams->cTail;

		/* copy WPS IE */
		os_memoryCopy(pRoleAP->hOs, pCopyPos, pRoleAP->tWpsIe.cIe, pRoleAP->tWpsIe.iIeLen);
		pCopyPos += pRoleAP->tWpsIe.iIeLen;

		/* copy last part of tail */
		os_memoryCopy(pRoleAP->hOs, pCopyPos, pWps + pWps[1] + 2,
		              pAPBeaconParams->iTailLen - ((char*)pWps - pAPBeaconParams->cTail) - (pWps[1] + 2));
		pCopyPos += pAPBeaconParams->iTailLen - ((char*)pWps - pAPBeaconParams->cTail) - (pWps[1] + 2);

		pTemplateStruct->len = pCopyPos - pTemplateStruct->ptr;

	} else { /* no WPS IE, just copy tail as it is */

		os_memoryCopy(pRoleAP->hOs, pTemplateStruct->ptr + uTempSize, pAPBeaconParams->cTail, pAPBeaconParams->iTailLen);
		uTempSize += pAPBeaconParams->iTailLen;
		pTemplateStruct->len = uTempSize;
	}

	((probeRspTemplate_t*)pTemplateStruct->ptr)->hdr.fc = ENDIAN_HANDLE_WORD(DOT11_FC_PROBE_RESP);

	return TI_OK;
}

/**
 * \fn     FillDeauthTemplate
 * \brief  Fill the constant fields of the deauth template
 *
 * \param   hRoleAP - Handle to RoleAP object
 * \param   pTemplateStruct - Output template
 *
 * \return  NA
 */
static void FillDeauthTemplate(TI_HANDLE hRoleAP, TSetTemplate *pTemplateStruct)
{
	TRoleAP             *pRoleAP  = (TRoleAP *)hRoleAP;
	disconnTemplate_t	*pBuffer = (disconnTemplate_t	*)pTemplateStruct->ptr;
	TI_UINT16			fc;

	os_memoryZero(pRoleAP->hOs, pBuffer, sizeof(disconnTemplate_t));

	fc = DOT11_FC_DEAUTH;

	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	pBuffer->disconnReason = 0; /* filled by firmware */

	pTemplateStruct->len = sizeof(disconnTemplate_t);
}

/**
 * \fn     ConfigureFrameTemplates
 * \brief  Configure the FW with frame templates
 *
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  NA
 */
static void ConfigureFrameTemplates(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP      = (TRoleAP *)hRoleAP;

	setBeaconProbeRspTempl(pRoleAP);
	setDeauthTemplate(pRoleAP);
	setQosNullDataTemplate(pRoleAP);
	setNullDataTemplate(pRoleAP);
}

/**
 * \fn     ConfigureRatePolicies
 * \brief  Configure the FW and the Data path with rate policies
 *
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  NA
 */
static void ConfigureRatePolicies(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP      = (TRoleAP *)hRoleAP;

	ctrlData_InitRatePolicy(pRoleAP->hCtrlData);
	ctrlData_setDataTxRatePolicies(pRoleAP->hCtrlData, pRoleAP->tBssCapabilities.uSupRateBitmap);
	ctrlData_setMgmtTxRatePolicy(pRoleAP->hCtrlData, pRoleAP->tBssCapabilities.uBasicRateBitmap);
	ctrlData_setBrcstTxRatePolicy(pRoleAP->hCtrlData, TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRoleAP->tBssCapabilities.uMinBasicRate));
}

/**
 * \fn     PrintRoleApDB
 * \brief  Print Role AP DB
 *
 * \param   pRoleAP - Pointer to RoleAP object
 *
 * \return  NA
 */
static void PrintRoleApDB(TRoleAP *pRoleAP)
{
	int i;

	os_printf("Role AP DB:\n");

	os_printf("Global HLID = %u\n", pRoleAP->uGlobalHlid);
	os_printf("Broadcast HLID = %u\n", pRoleAP->uBrcstHlid);
	os_printf("Number of allocated links = %u\n", pRoleAP->uNumAllocatedLinks);
	os_printf("Allocated links HLIDs:");
	for (i=0; i<WLANLINKS_MAX_LINKS; i++) {
		if (pRoleAP->aAllocatedLinks[i] != WLANLINKS_INVALID_HLID) {
			os_printf("%u ", pRoleAP->aAllocatedLinks[i]);
		}
	}
	os_printf("\n");
	os_printf("Role AP Statitics:\n");
	os_printf("Authorized stations = %u\n", pRoleAP->tRoleApStats.uNumAuthorized);
	os_printf("ADD_STATION commands received = %u\n", pRoleAP->tRoleApStats.uAddStaCmds);
	os_printf("REM_STATION commands received = %u\n", pRoleAP->tRoleApStats.uRemStaCmds);
}


/**
 * \fn     BssStartCompleteCb
 * \brief  Bss Start complete call back function, opens data
 * pass in broadcast & global links and network device
 *
 * Is called by TWD once Bss initialization sequence is
 * completed
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  void
 * \sa      roleAP_start()
 */
static void BssStartCompleteCb(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP  = (TRoleAP *)hRoleAP;

	/* Set Global link state to CONNECTED */
	if (wlanLinks_SetLinkState(pRoleAP->hWlanLinks, WLANLINK_STATE_CONNECTED, pRoleAP->uGlobalHlid) != TI_OK) {
	}
	/* Set link state to OPEN and to prevent error, go through state MGMT */
	txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, pRoleAP->uGlobalHlid, TX_CONN_STATE_MGMT);
	txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, pRoleAP->uGlobalHlid, TX_CONN_STATE_OPEN);
	rxData_SetLinkState(pRoleAP->hRxData, pRoleAP->uGlobalHlid, RX_CONN_STATE_OPEN);

	/* Set Broadcast link state to CONNECTED */
	if (wlanLinks_SetLinkState(pRoleAP->hWlanLinks, WLANLINK_STATE_CONNECTED, pRoleAP->uBrcstHlid) != TI_OK) {
	}
	/* Set link state to OPEN and to prevent error, go through state MGMT */
	txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, pRoleAP->uBrcstHlid, TX_CONN_STATE_MGMT);
	txMgmtQ_SetLinkState(pRoleAP->hTxMgmtQ, pRoleAP->uBrcstHlid, TX_CONN_STATE_OPEN);
	rxData_SetLinkState(pRoleAP->hRxData, pRoleAP->uBrcstHlid, RX_CONN_STATE_OPEN);

	wlanDrvIf_EnableTx (pRoleAP->hOs);
	setRxPortStatus(pRoleAP, OPEN); /* enable rx path */

	pRoleAP->eState	= ROLEAP_STATE_STARTED;
}

/**
 * \fn     setRxPortStatus
 * \brief  Enable/Disable rx path
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  void
 */
static TI_STATUS setRxPortStatus(TRoleAP *pRoleAP, portStatus_e ePortStatus)
{
	paramInfo_t *pParam;
	TI_STATUS status;


	pParam = (paramInfo_t *)os_memoryAlloc(pRoleAP->hOs, sizeof(paramInfo_t));
	if (!pParam) {
		return TI_NOK;
	}

	pParam->paramType = RX_DATA_PORT_STATUS_PARAM;
	pParam->content.rxDataPortStatus = ePortStatus;

	status = rxData_setParam(pRoleAP->hRxData, pParam);
	if (status != TI_OK) {
		os_memoryFree(pRoleAP->hOs, pParam, sizeof(paramInfo_t));
		return status;
	}

	return TI_OK;
}
/**
 * \fn     InactiveStaEventCB
 * \brief  The functions is called when Inactive STA FE event is
 * received
 *
 * Is called by TWD once FW INACTIVE_STA_EVENT_ID is
 * received
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  void
 * \sa      roleAP_SetDefaults()
 */
static TI_STATUS InactiveStaEventCB(TI_HANDLE hRoleAP, TI_CHAR *pData, TI_UINT32 uDataLen)
{
	TRoleAP 	*pRoleAP 	  = (TRoleAP *)hRoleAP;
	TI_UINT16 	uStaBitMap    = *((TI_UINT16 *) pData); /* it is safe to typecast as field offset is set properly in EvMBox */
	TI_UINT16	i, uStaBitMask;


	for (i=0, uStaBitMask=1; i<WLANLINKS_MAX_LINKS; i++, uStaBitMask <<= 1) {
		if (uStaBitMap & uStaBitMask) {

			wlanLinks_SetLinkInactivity (pRoleAP->hWlanLinks, TI_TRUE, i);
			sendEvent (pRoleAP, i, AP_EVENT_STA_AGING);
		}
	}
	return TI_OK;
}

/**
 * \fn     maxTxRetryStaEventCB
 * \brief  The functions is called when MAX TX Retry STA FE
 * event is received
 *
 * It's called by TWD once FW MAX_TX_RETRY_EVENT_ID is
 * received
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  TI_STATUS
 * \sa      roleAP_SetDefaults()
 */
static TI_STATUS maxTxRetryStaEventCB(TI_HANDLE hRoleAP, TI_CHAR *pData, TI_UINT32 uDataLen)
{
	TRoleAP 	*pRoleAP 	  = (TRoleAP *)hRoleAP;
	TI_UINT16 	uStaBitMap    = *((TI_UINT16 *) pData); /* it is safe to typecast as field offset is set properly in EvMBox */
	TI_UINT16	i, uStaBitMask;


	for (i=0, uStaBitMask=1; i<WLANLINKS_MAX_LINKS; i++, uStaBitMask <<= 1) {
		if (uStaBitMap & uStaBitMask) {

			wlanLinks_SetLinkInactivity (pRoleAP->hWlanLinks, TI_TRUE, i);
			sendEvent (pRoleAP, i, AP_EVENT_STA_MAX_TX_RETRY);
		}
	}
	return TI_OK;
}


/**
 * \fn     sendEvent
 * \brief  The functions is called to send inactivity event to
 * users space process - hostapd
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 * \param   uHlid   - host link ID
 * \param   eEvent  - Aging Event
 *
 * \return  operation status
 * \sa      roleAP_SetDefaults()
 */
static TI_STATUS sendEvent(TRoleAP *pRoleAP, TI_UINT32 uHlid, EApEvent eEvent)
{
	TI_STATUS   tRes;
	TWlanLinkPeerDescr tStaDescr;

	tRes = wlanLinks_GetPeerDescr (pRoleAP->hWlanLinks, uHlid, &tStaDescr);
	if (tRes == TI_OK) {
		TApEvent    tApEvent;

		tApEvent.uEvent = (unsigned char) eEvent;
		os_memoryCopy(pRoleAP->hOs, tApEvent.uAddr, tStaDescr.aMacAddr, sizeof (tApEvent.uAddr));


		EvHandlerSendEvent (pRoleAP->hEvHandler, IPC_EVENT_AP_EVENT, (TI_UINT8*) &tApEvent, sizeof (tApEvent));
	}
	return tRes;

}

/**
 * \fn     staInactivity
 * \brief  The functions is called to retrive STA inactivity
 * status
 *
 * Is called by ApCmd once ROLE_AP_GET_STA_INACTIVITY command is
 * invoked
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  void
 * \sa      roleAP_SetDefaults()
 */
static TI_STATUS staInactivity(TI_HANDLE hRoleAP, void * pInBuf, void *pOutBuf)
{
	TRoleAP 		* pRoleAP = (TRoleAP *)hRoleAP;
	TApGeneralParam * pInPrm  = (TApGeneralParam *) pInBuf;
	TApGeneralParam * pOutPrm = (TApGeneralParam *) pOutBuf;
	TI_UINT8		* pAdr 	  = pInPrm->cMac;
	TI_UINT32 		  uHlid;
	TI_BOOL			  bInactivity;
	TI_STATUS		  tRes = TI_NOK;


	/* Find STA in WlanLinks DB*/
	if (wlanLinks_FindLinkByMac(pRoleAP->hWlanLinks, pAdr, &uHlid) == TI_OK) {
		if (wlanLinks_GetLinkInactivity (pRoleAP->hWlanLinks, uHlid, &bInactivity) == TI_OK) {
			pOutPrm->lValue = (bInactivity) ? AP_STA_MAX_INACTIVITY : 0;

			tRes = TI_OK;
		} else {
			pOutPrm->lValue = AP_STA_MAX_INACTIVITY;
			tRes = TI_NOK;
		}
	} else {

		pOutPrm->lValue = AP_STA_MAX_INACTIVITY;
		tRes = TI_NOK;
	}

	os_memoryCopy(pRoleAP->hOs, pOutPrm->cMac, pInPrm->cMac, sizeof(pOutPrm->cMac));

	return tRes;
}


/**
 * \fn     setKey
 * \brief  set key to FW
 *
 * Is called by TWD once TWD_BssStart is completed
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  void
 * \sa      roleAP_start()
 */

static TI_STATUS setKey(TI_HANDLE hRoleAP, TSecurityKeys *pTwdKey)
{
	TRoleAP *pRoleAP  = (TRoleAP *)hRoleAP;
	TTwdParamInfo    tTwdParam;
	TI_STATUS  tRes = TI_NOK;


	tTwdParam.paramType = TWD_RSN_SECURITY_MODE_PARAM_ID;
	tTwdParam.content.rsnEncryptionStatus =pTwdKey->keyType;
	tRes = TWD_SetParam(pRoleAP->hTWD, &tTwdParam);
	if (tRes != TI_OK) {
		return tRes;
	}

	/* set the size to reserve for encryption to the tx */
	switch (pTwdKey->keyType) {
	case KEY_TKIP:
		TxDataQ_setEncryptionFieldSizes(pRoleAP->hTxDataQ,pTwdKey->hlid, IV_FIELD_SIZE);
		break;
	case KEY_AES:
		TxDataQ_setEncryptionFieldSizes(pRoleAP->hTxDataQ,pTwdKey->hlid, AES_AFTER_HEADER_FIELD_SIZE);
		break;
#ifdef GEM_SUPPORTED
	case KEY_GEM:
#endif
	case KEY_WEP:
	case KEY_NULL:
	case KEY_XCC:
	default:
		TxDataQ_setEncryptionFieldSizes (pRoleAP->hTxDataQ,pTwdKey->hlid, 0);
		break;
	}


	tTwdParam.paramType = TWD_RSN_KEY_ADD_PARAM_ID;
	tTwdParam.content.configureCmdCBParams.pCb = (TI_UINT8*)pTwdKey;
	tTwdParam.content.configureCmdCBParams.fCb = NULL;
	tTwdParam.content.configureCmdCBParams.hCb = NULL;
	tRes = TWD_SetParam (pRoleAP->hTWD, &tTwdParam);

	return tRes;
}


/**
 * \fn     setBeaconProbeRspTempl
 * \brief  The functions is called to set Beacon and Probe
 * Response templates
 *
 * Is called by ApCmd once ROLE_AP_ADD_BEACON_PARAM command is
 * invoked
 *
 * \note
 * \param   pRoleAP - pointer to RoleAP object
 *
 * \return  void
 * \sa      roleAP_Start()
 */
static TI_STATUS setBeaconProbeRspTempl(TRoleAP *pRoleAP)
{
	TSetTemplate         tTemplateStruct;
	TI_UINT32            uLen;
	TI_STATUS		  	 tRes;

	/* Allocate common buffer for Beacon and ProbeResp templates building,
	 * max size is defined by ProbeResp for hidden SSID support
	 */
	uLen = pRoleAP->tBssCapabilities.tAPBeaconParams.iHeadLen +
	       pRoleAP->tBssCapabilities.tAPBeaconParams.iTailLen + sizeof(dot11_SSID_t);

	tTemplateStruct.ptr = (TI_UINT8 *)os_memoryAlloc (pRoleAP->hOs, uLen );
	if (!tTemplateStruct.ptr) {
		return TI_NOK;
	}

	tTemplateStruct.type = AP_BEACON_TEMPLATE;
	tTemplateStruct.uRateMask = TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRoleAP->tBssCapabilities.uMinBasicRate);
	FillBeaconTemplate(pRoleAP, (TApBeaconParams*)&pRoleAP->tBssCapabilities.tAPBeaconParams, &tTemplateStruct);

	tRes = TWD_CmdTemplate (pRoleAP->hTWD, &tTemplateStruct, NULL, NULL);
	if (tRes != TI_OK) {
		os_memoryFree (pRoleAP->hOs, tTemplateStruct.ptr, uLen);
		return tRes;
	}

	tTemplateStruct.type = AP_PROBE_RESPONSE_TEMPLATE;
	tTemplateStruct.uRateMask = TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRoleAP->tBssCapabilities.uMinBasicRate);
	FillProbeRespTemplate(pRoleAP, (TApBeaconParams*)&pRoleAP->tBssCapabilities.tAPBeaconParams, &tTemplateStruct);

	tRes = TWD_CmdTemplate (pRoleAP->hTWD, &tTemplateStruct, NULL, NULL);
	if (tRes != TI_OK) {
		os_memoryFree (pRoleAP->hOs, tTemplateStruct.ptr, uLen);
		return tRes;
	}

	os_memoryFree (pRoleAP->hOs, tTemplateStruct.ptr, uLen);
	return tRes;
}
/**
 * \fn     setDeauthTemplate
 * \brief  The functions is called to set de-authentication
 * template
 *
 * The function s called by ConfigureFrameTemplates
 *
 * \note
 * \param   pRoleAP - pointer to RoleAP object
 *
 * \return  void
 * \sa      roleAP_Start()
 */
static void setDeauthTemplate( TRoleAP * pRoleAP )
{
    TSetTemplate            tTemplateStruct;
    disconnTemplate_t		tDeauthTemplate;


	tTemplateStruct.type = AP_DEAUTH_TEMPLATE;
	tTemplateStruct.ptr = (TI_UINT8*)&tDeauthTemplate;
    tTemplateStruct.uRateMask = TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRoleAP->tBssCapabilities.uMinBasicRate);
    FillDeauthTemplate(pRoleAP, &tTemplateStruct);
	TWD_CmdTemplate (pRoleAP->hTWD, &tTemplateStruct, NULL, NULL);
}
/**
 * \fn     setQosNullDataTemplate
 * \brief  The functions is called to set QOS Null Data
 * template
 *
 * The function s called by ConfigureFrameTemplates
 *
 * \note
 * \param   pRoleAP - pointer to RoleAP object
 *
 * \return  void
 * \sa      roleAP_Start()
 */
static void setQosNullDataTemplate( TRoleAP * pRoleAP)
{

    TSetTemplate            tTemplateStruct;
    QosNullDataTemplate_t   tQosNullDataTemplate;
    QosNullDataTemplate_t	*pBuffer;
    TI_UINT16				fc;
    TI_UINT16				qosControl;

    tTemplateStruct.type = QOS_NULL_DATA_TEMPLATE;
    tTemplateStruct.ptr = (TI_UINT8*)&tQosNullDataTemplate;
    tTemplateStruct.uRateMask = TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRoleAP->tBssCapabilities.uMinBasicRate);
    pBuffer =  (QosNullDataTemplate_t*)tTemplateStruct.ptr;
    os_memoryZero(pRoleAP->hOs, pBuffer, sizeof(QosNullDataTemplate_t));

    MAC_COPY (pBuffer->hdr.address2, pRoleAP->tBssCapabilities.tBssid);
    MAC_COPY (pBuffer->hdr.address3, pRoleAP->tBssCapabilities.tBssid);
    fc = DOT11_FC_DATA_NULL_QOS | (1 << DOT11_FC_FROM_DS_SHIFT);
    COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */
    qosControl = 0;  /* User priority */
    qosControl <<= QOS_CONTROL_UP_SHIFT;
    COPY_WLAN_WORD(&pBuffer->hdr.qosControl, &qosControl); /* copy with endianess handling. */


    tTemplateStruct.len = WLAN_QOS_HDR_LEN;

    TWD_CmdTemplate (pRoleAP->hTWD, &tTemplateStruct, NULL, NULL);
}
/**
 * \fn     setNullDataTemplate
 * \brief  The functions is called to set Null Data template
 *
 * The function s called by ConfigureFrameTemplates
 *
 * \note
 * \param   pRoleAP - pointer to RoleAP object
 *
 * \return  void
 * \sa      roleAP_Start()
 */
static void setNullDataTemplate(TRoleAP * pRoleAP)
{
    TSetTemplate            tTemplateStruct;
    nullDataTemplate_t      tNullDataTemplate;
    nullDataTemplate_t      *pBuffer;
    TI_UINT16				fc;

    tTemplateStruct.type = NULL_DATA_TEMPLATE;
    tTemplateStruct.ptr = (TI_UINT8*)&tNullDataTemplate;
    tTemplateStruct.uRateMask = TWD_GetBitmapByRateNumber(pRoleAP->hTWD, pRoleAP->tBssCapabilities.uMinBasicRate);
    pBuffer =  (nullDataTemplate_t*)tTemplateStruct.ptr;

    os_memoryZero(pRoleAP->hOs, pBuffer, sizeof(nullDataTemplate_t));


    /* Set BSSID address */
    MAC_COPY (pBuffer->hdr.BSSID, pRoleAP->tBssCapabilities.tBssid);
    MAC_COPY (pBuffer->hdr.SA,    pRoleAP->tBssCapabilities.tBssid);


    fc = DOT11_FC_DATA_NULL_FUNCTION | (1 << DOT11_FC_FROM_DS_SHIFT);

    COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

    tTemplateStruct.len = sizeof(dot11_mgmtHeader_t);
    TWD_CmdTemplate (pRoleAP->hTWD, &tTemplateStruct, NULL, NULL);
}
/**
 * \fn     DecodeStaRates
 * \brief  Decode STA supported rates from how they are encoded
 * in Assoc Response to the human format
 *
 * \param   aRates - Rates array
 * \param   uRatesLen - Rates array length
 *
 * \return  void
 */
static void DecodeStaRates(TI_UINT8 *aRates, TI_UINT32 uRatesLen)
{
	int i;

	for (i=0; i<uRatesLen; i++) {
		aRates[i] = rate_NetToNumber(aRates[i]);
	}
}

/**
 * \fn     SetApRates
 * \brief  Set AP supported rates received from the application
 *
 * \param   hRoleAP - RoleAP handle
 * \param   pRateParams - Rates parameters structure
 *
 * \return  void
 */
static void SetApRates(TI_HANDLE hRoleAP, TApRateSet *pRateParams)
{
	TRoleAP *pRoleAP  = (TRoleAP *)hRoleAP;
	int i;
	TI_UINT8   aApSuppRates[AP_MAX_SUPPORT_RATE];
	TI_UINT32  uMinBasicRate;

	pRoleAP->tBssCapabilities.uNumSupRates = pRateParams->cSuppRateLen;

	/*Rates come in 100 Kbps, convert it to Mbps*/
	for (i=0; i<pRateParams->cSuppRateLen; i++) {
		aApSuppRates[i] = pRateParams->aSupportedRates[i]/10;
	}


	pRoleAP->tBssCapabilities.uSupRateBitmap = ConvertRateSetToBitmap(hRoleAP, &aApSuppRates[0],pRateParams->cSuppRateLen);

	pRoleAP->tBssCapabilities.uNumBasicRates = pRateParams->cBasicRateLen;
	uMinBasicRate = 0xFFFF;

	for (i=0; i<pRateParams->cBasicRateLen; i++) {
		pRoleAP->tBssCapabilities.aBasicRateSet[i] = pRateParams->aBasicRates[i]/10;
		if (uMinBasicRate > pRoleAP->tBssCapabilities.aBasicRateSet[i])
			uMinBasicRate = pRoleAP->tBssCapabilities.aBasicRateSet[i];
	}

	pRoleAP->tBssCapabilities.uBasicRateBitmap = ConvertRateSetToBitmap(hRoleAP, &pRoleAP->tBssCapabilities.aBasicRateSet[0],pRateParams->cBasicRateLen);

	pRoleAP->tBssCapabilities.uMinBasicRate = uMinBasicRate;

}

/**
 * \fn     roleAP_enable
 * \brief  Enable roleAP
 *
 * \param   hRoleAP - Handle to RoleAP object
 *
 */
static TI_STATUS roleAP_enable (TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	/*ignore the command if we're in the Started state*/
	if (pRoleAP->eState != ROLEAP_STATE_STARTED) {
		pRoleAP->eState = ROLEAP_STATE_ENABLED;
	} else {
		return TI_NOK;
	}

	if (InitBssLinks(hRoleAP) != TI_OK) {
		return TI_NOK;
	}


	return TI_OK;
}

/**
 * \fn     ResetRoleApDB
 * \brief  Reset roleAP DB
 *
 * \param   hRoleAP - Handle to RoleAP object
 *
 */
static void ResetRoleApDB(TI_HANDLE hRoleAP)
{
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	os_memoryZero (pRoleAP->hOs, (void *)&pRoleAP->tBssCapabilities, sizeof(pRoleAP->tBssCapabilities));
	pRoleAP->DefaultKeyIndex = 0;
	pRoleAP->keyType = KEY_NULL;
	os_memoryZero (pRoleAP->hOs, (void *)&pRoleAP->tTwdKey, sizeof(pRoleAP->tTwdKey));
}

/**
 * \fn     roleAP_reportMicFailure
 * \brief  The functions is called when Mic failure packet
 * is received
 *
 * It's called by rxData_ReceivePacket
 *
 * \note
 * \param   hRoleAP - Handle to RoleAP object
 *          sta address
 */

void roleAP_reportMicFailure(TI_HANDLE hRoleAP,TI_UINT8* pMac)
{
	TI_UINT32          uHlid;
	TRoleAP *pRoleAP = (TRoleAP *)hRoleAP;

	if (wlanLinks_FindLinkByMac(pRoleAP->hWlanLinks, pMac, &uHlid) == TI_OK) {
		sendEvent (pRoleAP, uHlid, AP_EVENT_STA_MIC_FAILURE);
	} else
		WLAN_OS_REPORT(("%s: can't find hlid \n",__FUNCTION__));

}

