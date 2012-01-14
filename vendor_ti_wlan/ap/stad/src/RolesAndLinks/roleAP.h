/*
 * roleAP.h
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




/* \file   roleAP.h
*  \brief  RoleAP API definition
*
*  RoleAP module provides:
*
* 	- Save Bss confguration received by Hostapd application.
*  - Start the AP Role in FW by Sending Configuration & start/stop commands
*	- Receives commands from external AP Manager and updates WlanLinks DB and FW STA DB
*  - Updates Data Path links tables
*  \see    roleAP.c
*/

#ifndef _ROLES_AP_H_
#define _ROLES_AP_H_

#include "wlanLinks.h"
#define SEQ_CTRL_FIELD_LEN  2
#define TIMESTAMP_FIELD_LEN 8
#define MAX_WEP_KEY         8

#include "APExternalIf.h"
#include "802_11Defs.h"

/* Constants */


/* Enumerations */

typedef enum {
	ROLEAP_STATE_ENABLED,
	ROLEAP_STATE_STOPPED,
	ROLEAP_STATE_STARTED,
	ROLEAP_STATE_RECOVERING,
	ROLEAP_STATE_MAX

} ERoleApState;


typedef enum {
	ROLE_AP_RATES_IN_KBPS_10,
	ROLE_AP_RATES_IN_KBPS_20,
	ROLE_AP_RATES_IN_KBPS_55,
	ROLE_AP_RATES_IN_KBPS_60,
	ROLE_AP_RATES_IN_KBPS_90,
	ROLE_AP_RATES_IN_KBPS_110,
	ROLE_AP_RATES_IN_KBPS_120,
	ROLE_AP_RATES_IN_KBPS_180,
	ROLE_AP_RATES_IN_KBPS_240,
	ROLE_AP_RATES_IN_KBPS_360,
	ROLE_AP_RATES_IN_KBPS_480,
	ROLE_AP_RATES_IN_KBPS_540,
	ROLE_AP_RATES_IN_KBPS_MAX_SIZE

} ERoleApRatesInKbps;

/* */

typedef struct {
	TI_UINT16                   uFrameControl;
	TI_UINT16                   uDuration;
	TMacAddr                    tDestination;
	TMacAddr                    tSource;
	TMacAddr                    tBSSID;
	TI_UINT16                   uSeqCtrl;
	TI_UINT8                    tTimeStamp[8];
	TI_UINT16                   uBeaconInterval;
	TI_UINT16                   uCapabilityInfo;
	dot11_SSID_t                tSsidIE;
	dot11_RATES_t               tSupportedRatesIE;
	dot11_DS_PARAMS_t           tDSParamsIE;
} TRoleAPBeaconDataHead;


typedef struct {
	dot11_countryIE_t           tCountryIE;
	dot11_ERP_t                 tERPIE;
	dot11_RATES_t               tExtSupportedRatesIE;
	dot11_WPA_IE_t              tWPAIE;
	dot11_WME_PARAM_t           tWMEParamsIE;
	Tdot11HtCapabilitiesUnparse tHTCapabIE;
	Tdot11HtInformationUnparse  tHTInfoIE;

} TRoleAPBeaconDataTail;


/* Structures */
typedef struct {
	TRoleAPBeaconDataHead tRoleAPBeaconDataHead;
	TRoleAPBeaconDataTail tRoleAPBeaconDataTail;
} TRoleAPBeaconDataFull;


/* Bss capabilities structure */
typedef struct {
	TMacAddr                tMacAddress;
	TMacAddr                tBssid;
	TSsid                   tSsid;
	TI_UINT8                uBssIndex;
	SsidType_enum           eSsidType;
	TI_UINT8                uBand;
	TI_UINT8                uChannel;
	TI_UINT16               uBeaconInterval;
	TI_UINT8                uDtimPeriod;
	TI_UINT16               uCapabilities;
	TI_BOOL                 bUseProtection;
	TI_BOOL                 bUseShortPreamble;
	TI_BOOL                 bUseShortSlotTime;
	TI_UINT8                uNumSupRates;
	TI_UINT32               uSupRateBitmap;
	TI_UINT8				aBasicRateSet[ROLE_AP_RATES_IN_KBPS_MAX_SIZE];
	TI_UINT8                uNumBasicRates;
	TI_UINT32               uBasicRateBitmap;
	TI_UINT32               uMinBasicRate;

	TI_UINT16               uRtsThreshold;
	TI_UINT8                countryStr[AP_MAX_CNT_CHAR];
	TI_BOOL                 bPrivacyEnabled;
	TApBeaconParams         tAPBeaconParams;
	TRoleAPBeaconDataFull   tBeaconDataFull;
	TI_UINT16               uInactivity;

} TBssCapabilities;

typedef struct {
	TI_UINT32 uAddStaCmds;
	TI_UINT32 uRemStaCmds;
	TI_UINT32 uNumAuthorized;
} TRoleApStats;


/**
 * \fn     roleAP_create
 * \brief  Create roleAP object
 *
 * Allocate and clear the module's object
 *
 * \note
 * \param	hOs    - Handle to OS context
 * \return 	Handle of the allocated object, NULL if allocation failed
 * \sa     	drvMain_create
 */
TI_HANDLE roleAP_create(TI_HANDLE hOs);


/**
 * \fn     roleAP_destroy
 * \brief  Destroy RolesMgr object
 *
 * Free the module's object memory
 *
 * \note
 * \param	hRoleAP - Handle to roleAP object
 * \return 	TI_OK
 * \sa     	drvMain_destroy
 */
TI_STATUS roleAP_destroy(TI_HANDLE hRoleAP);


/**
 * \fn     roleAP_init
 * \brief  Init roleAP object
 *
 * Init module's object and link its handles
 *
 * \note
 * \param	pStadHandles - Handle to StadHandles
 * \return 	Void
 * \sa     	drvMain_Init
 */
void roleAP_init (TStadHandlesList *pStadHandles);



/**
 * \fn     roleAP_SetDefaults
 * \brief  set roleAP object default values
 *
 * Set module's object default values
 *
 * \note
 * \param	hRoleAP - Handle to roleAP object
 *
 * \return 	TI_OK
 * \sa     	drvMain_SetDefaults
 */
TI_STATUS roleAP_SetDefaults (TI_HANDLE hRoleAP, TRoleApInitParams *tRoleApInitParams);


/**
 * \fn     roleAP_setParam
 * \brief  roleAP object getParam API
 *
 * Module's object set param API
 *
 * \note
 * \param	hRoleAP - Handle to roleAP object
 * \param	pParam - Handle to generic paramInfo structure
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 * \sa     	cmdDispathcer CB
 */
TI_STATUS roleAP_setParam(TI_HANDLE hRoleAP, paramInfo_t *pParam);




/**
 * \fn     roleAP_getParam
 * \brief  roleAP object getParam API
 *
 * Module's object set param API
 *
 * \note
 * \param	hRoleAP - Handle to roleAP object
 * \param	pParam - Handle to generic paramInfo structure to be filled
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 * \sa     	cmdDispathcer CB
 */
TI_STATUS roleAP_getParam(TI_HANDLE hRoleAP, paramInfo_t *pParam);



/**
 * \fn     roleAP_start
 * \brief  RolesMgr start command API
 *
 * Start RolesMgr object - send configuration & start cmd to FW
 *
 * \note
 * \param	hRoleAP - Handle to RolesMgr object
 * \param	uBssIdx - BSS Index
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 * \sa     	roleAP_setParam
 */
TI_STATUS roleAP_start(TI_HANDLE hRoleAP, TI_UINT8 uBssIdx);




/**
 * \fn     roleAP_stop
 * \brief  roleAP stop command API
 *
 * Start roleAP object - send stop BSS to FW
 *
 * \note
 * \param	hRoleAP - Handle to RoleAP object
 * \param	bssIdx - Bss Index
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 */
TI_STATUS roleAP_stop(TI_HANDLE hRoleAP, TI_UINT8 bssIdx);


TI_STATUS RoleAp_setApCmd(TI_HANDLE hRoleAP, TI_UINT32 cmd, void *pBuffer);


TI_STATUS RoleAp_getApCmd(TI_HANDLE hRoleAP, TI_UINT32 cmd, void *pInBuf, void *pOutBuf);


/**
 * \fn     roleAP_RemoveStaCompleteCB
 * \brief  FW Remove STA Complete event callback
 *
 * Start roleAP object - send stop BSS to FW
 */
TI_STATUS roleAP_RemoveStaCompleteCB(TI_HANDLE hRoleAP, TI_CHAR *pData, TI_UINT32 uDataLen);

/**
 * \fn     roleAP_NotifyFwReset
 * \brief  notify the roleAP about FW reset
 *
 * \param	hRoleAP - Handle to RoleAP object

 */
TI_STATUS roleAP_NotifyFwReset(TI_HANDLE hRoleAP);

/**
 * \fn     roleAp_getApState
 * \brief  get Role AP current state
 *
 * \param	hRoleAP - Handle to RoleAP object
 * \param	pState - output value
 *
 */
TI_STATUS roleAp_getApState(TI_HANDLE hRoleAP, TI_UINT32 *pState);

/**
 * \fn     RoleAp_DrvResetNotifyUpperLayers
 * \brief  Send driver reset event to user application - hostapd
 *
 * \param   hRoleAP - Handle to RoleAP object
 *
 * \return  operation status
 */
TI_STATUS RoleAp_DrvResetNotifyUpperLayers(TI_HANDLE hRoleAP);

void roleAP_reportMicFailure(TI_HANDLE hRoleAP,TI_UINT8* pMac);

#endif /*  _ROLE_AP_API_H_*/

