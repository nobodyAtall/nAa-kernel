/*
 * wlanLinks.h
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

/** \file wlanLinks.h
 *  \brief WlanLinks module API
 *
 *  \see wlanLinks.c
 */

/****************************************************************************
 *                                                                          *
 *   MODULE:  wlanLinks                                                     *
 *   PURPOSE: WlanLinks Module API                                          *
 *                                                                          *
 ****************************************************************************/

#ifndef _WLAN_LINKS_H_
#define _WLAN_LINKS_H_

#include "APExternalIf.h"
#include "STADExternalIf.h"

/* Constants */

#define WLANLINKS_INVALID_HLID     0xff

/* Enumerations */


typedef enum {
	WLANLINK_STATE_FREE,
	WLANLINK_STATE_IDLE,
	WLANLINK_STATE_DISCOVERY,
	WLANLINK_STATE_CONNECTING,
	WLANLINK_STATE_CONNECTED,
	WLANLINK_STATE_DISCONNECTING,
} EWlanLinkState;

typedef enum {
	WLANLINK_SUBSTATE_CONN_NORMAL,
	WLANLINK_SUBSTATE_CONN_ROAMING
} EWlanLinkSubState;


typedef enum {
	WLANLINK_ROLE_AP,
	WLANLINK_ROLE_STA,
	WLANLINK_ROLE_IBSS,
	WLANLINK_ROLE_P2P,
	WLANLINK_ROLE_BTHS,
} EWlanLinkRole;

typedef enum {
	WLANLINK_SUBROLE_APLIKE,
	WLANLINK_SUBROLE_STALIKE,
} EWlanLinkSubRole;

/* Structures */

typedef struct {
	TMacAddr    aMacAddr;
	TSsid       tSsid;
	TI_UINT8    uBand;
	TI_UINT8    uChannel;
	TI_UINT16   uCapablities;
	TI_UINT32   uSupRatesBitmap;
} TWlanLinkPeerDescr;

typedef struct {
	EWlanLinkRole       eRole;
	EWlanLinkSubRole    eSubRole;
	EWlanLinkType       eType;
	TI_UINT32           uHlid;
	TWlanLinkPeerDescr  tPeerDescr;
	EWlanLinkState      eState;
	EWlanLinkSubState   eSubState;
	TI_UINT32           uQuality;
	TI_UINT32           uTxPow;
	TI_BOOL             bInactive;
	TI_UINT32           uAid;
	TI_UINT32			uDeauthReason;
} TWlanLinkInfo;

/* WlanLinks handle */
typedef struct {

	/* Module's handles*/
	TI_HANDLE           hReport;
	TI_HANDLE           hOs;
	TI_HANDLE           hEvHandler;
	TI_HANDLE           hTWD;
	TI_HANDLE           hSiteMgr;
	TI_HANDLE           hCtrlData;

	TI_UINT32           nFree;
	TWlanLinkInfo       aLinks[WLANLINKS_MAX_LINKS];
} TWlanLinks;



/* Function prototypes */

/**
 * \fn     wlanLinks_Create
 * \brief  create WlanLinks module
 * \param  hOs - OS handle
 * \return handle to the allocated object
 */
TI_HANDLE wlanLinks_Create(TI_HANDLE hOs);

/**
 * \fn     wlanLinks_Destroy
 * \brief  destroy WlanLinks module
 * \param  hwlanLinks - WlanLinks module handle
 * \return status TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_Destroy(TI_HANDLE hWlanLinks);

/**
 * \fn     wlanLinks_Init
 * \brief  init WlanLinks module
 * \param  pUwdHandles - UWD modules handle
 * \return status TI_OK/TI_NOK
 */
void wlanLinks_Init (TStadHandlesList *pStadHandles);

/**
 * \fn     wlanLinks_AllocateNew
 * \brief  Allocate new Wlan Link
 *
 * Note: The assumption here is that all accesses to WlanLink DB
 *       are performed in signle context
 * \param hwlanLinks - WlanLinks module handle
 * \param pLinkInfo - Wlan Link info structure
 * \param pHlid - output HLID of the allocated link
 * \return status TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_AllocateNew(TI_HANDLE hwlanLinks, TWlanLinkInfo *pLinkInfo, TI_UINT32 *uHlid);

/**
 * \fn     wlanLinks_UpdateLinkInfo
 * \brief  Update Wlan Link info except dynamic parameters
 *
 * Note: The assumption here is that all accesses to WlanLink DB
 *       are performed in signle context
 * \param hwlanLinks - WlanLinks module handle
 * \param pLinkInfo - Wlan Link info structure
 * \param uHlid - link HLID
 * \return status TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_UpdateLinkInfo(TI_HANDLE hwlanLinks, TWlanLinkInfo *pLinkInfo, TI_UINT32 uHlid);

/**
 * \fn     wlanLinks_FreeLink
 * \brief  Free Wlan Link
 *
 * Note: The assumption here is that all accesses to WlanLink DB
 *       are performed in signle context
 *
 * \param hwlanLinks - WlanLinks module handle
 * \param uHlid - HLID of the link
 * \return status TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_FreeLink(TI_HANDLE hwlanLinks, TI_UINT32 uHlid);

/**
 * \fn     wlanLinks_FindLinkByMac
 * \brief  Find WlanLink by MAC address
 *
 * Note: The assumption here is that all accesses to WlanLink DB
 *       are performed in signle context
 *
 * \param hwlanLinks - WlanLinks module handle
 * \param mac - input MAC addres
 * \param uHlid - output HLID of the link
 * \return status TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_FindLinkByMac(TI_HANDLE hwlanLinks, TI_UINT8 *mac, TI_UINT32 *uHlid);

/**
 * \fn     wlanLinks_GetPeerDescr
 * \brief  Get Peer description of a link
 *
 * Note: The assumption here is that all accesses to WlanLink DB
 *       are performed in signle context
 *
 * \param hwlanLinks - WlanLinks module handle
 * \param uHlid - HLID of the link
 * \param pPeerDescr - output buffer
 * \return status TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_GetPeerDescr(TI_HANDLE hwlanLinks, TI_UINT32 uHlid, TWlanLinkPeerDescr *pPeerDescr);

/**
 * \fn     wlanLinks_PrintDB
 * \brief  Print WlanLinks DB
 *
 * \param hwlanLinks - WlanLinks module handle
 * \return none
 */
void wlanLinks_PrintDB(TI_HANDLE hwlanLinks);

/**
 * \fn     wlanLinks_SetLinkState
 * \brief  Set Wlan Link state
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  eState - link state
 * \param  uhlid - HLID
 * \return TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_SetLinkState(TI_HANDLE hwlanLinks, EWlanLinkState eState, TI_UINT32 uHlid);

/**
 * \fn     wlanLinks_GetLinkState
 * \brief  Get Wlan Link state
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  uHlid - HLID
 * \param  eState - link state
 *
 * \return TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_GetLinkState(TI_HANDLE hwlanLinks, TI_UINT32 uHlid, EWlanLinkState *eState);

/**
 * \fn     wlanLinks_SetLinkInactivity
 * \brief  Set Wlan Link inactivity status
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  bInactivity - is true if STA is not active
 * \param uHlid - host link id
 * \return operation status: TI_OK or TI_NOK
 */
TI_STATUS wlanLinks_SetLinkInactivity(TI_HANDLE hWlanLinks, TI_BOOL bInactivity, TI_UINT32 uHlid);

/**
 * \fn     wlanLinks_GetLinkInactivity
 * \brief  Get Wlan Link inactivity status
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  uHlid - host link id
 * \param  pInactivity - pointer to return inactivity status
 * \return operation status: TI_OK or TI_NOK
 */
TI_STATUS wlanLinks_GetLinkInactivity(TI_HANDLE hWlanLinks, TI_UINT32 uHlid, TI_BOOL *pInactivity);


/**
 * \fn     wlanLinks_SetDeauthReason
 * \brief  Set Wlan Link deauthentication reason
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  uReason - deauthentication reason
 * \param  uHlid - HLID
 *
 * \return TI_OK/TI_NOK
 *
 */
TI_STATUS wlanLinks_SetDeauthReason(TI_HANDLE hwlanLinks, TI_UINT32 uReason, TI_UINT32 uHlid);

/**
 * \fn     wlanLinks_GetDeauthReason
 * \brief  Get Wlan Link deauthentication reason
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  pReason - output deauthentication reason
 * \param uHlid - HLID
 *
 * \return TI_OK/TI_NOK
 *
 */
TI_STATUS wlanLinks_GetDeauthReason(TI_HANDLE hwlanLinks, TI_UINT32 *pReason, TI_UINT32 uHlid);

/**
 * \fn     wlanLinks_GetLinkSupRates
 * \brief  Get Wlan Link supported rates bitmap
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  pRates - output rates bitmap
 * \param uHlid - HLID
 *
 * \return TI_OK/TI_NOK
 */
TI_STATUS wlanLinks_GetLinkSupRates(TI_HANDLE hwlanLinks, TI_UINT32 *pRates, TI_UINT32 uHlid);


/**
 * \fn     wlanLinks_GetLinkType
 * \brief  Get Wlan Link type
 *
 * \param  hwlanLinks - WlanLinks module handle
 * \param  uHlid - HLID
 * \param  eType - output type
 * \return TI_OK/TI_NOK
 *
 */
TI_STATUS wlanLinks_GetLinkType(TI_HANDLE hwlanLinks, TI_UINT32 uHlid, EWlanLinkType *eType);

/**
 * \fn     wlanLinks_UpdRatePoliciesUponLinkAdd
 * \brief  Add rate policy to rate policy array if doesn't exist
 *         Increment policy reference count if exists
 *
 * \param   hwlanLinks - Handle to WlanLinks object
 * \param   uRatesBitmap - link supported rates bitmap
 * \param   pRatePolicyIdx - output rate policy index
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
TI_STATUS wlanLinks_UpdRatePoliciesUponLinkAdd(TI_HANDLE hwlanLinks, TI_UINT32 uRatesBitmap, TI_UINT32 *pRatePolicyIdx);

/**
 * \fn     wlanLinks_UpdRatePoliciesUponLinkRemove
 * \brief  Decrease link rate policy referrence counter and
 * remove the policy if it becomes 0
 *
 * \param   hwlanLinks - Handle to WlanLinks object
 * \param   uHlid - removed link HLID
 *
 * \return  TI_OK if success , TI_NOK otherwise
 */
TI_STATUS wlanLinks_UpdRatePoliciesUponLinkRemove(TI_HANDLE hRoleAP, TI_UINT32 uHlid);


#endif /*  _WLAN_LINKS_H_*/


