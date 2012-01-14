/*
 * wlanLinks.c
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

/** \file wlanLinks.c
 *  \brief Wlan Links database
 *
 *  \see wlanLinks.h
 */

#define __FILE_ID__  FILE_ID_141

#include "siteHash.h"
#include "wlanLinks.h"
#include "osApi.h"
#include "report.h"
#include "802_11Defs.h"
#include "TWDriver.h"
#include "EvHandler.h"
#include "DrvMainModules.h"
#include "coreDefaultParams.h"


#include "paramOut.h"


/* Constants */

/* Enumerations */


/* Typedefs */


/* Structures */


/* Internal functions prototypes */
static void ResetLinkInfo(TWlanLinks *pWlanLinks, TI_UINT32 uHlid);
static void SetLinkInfo(TWlanLinks *pWlanLinks, TWlanLinkInfo *pLinkInfo, TI_UINT32 uHlid);

/* Public functions implementation */



/**
 * \fn     wlanLinks_Create
 * \brief  create WlanLinks module
 */
TI_HANDLE wlanLinks_Create(TI_HANDLE hOs)
{
	TWlanLinks   *pWlanLinks;

	if ((pWlanLinks = os_memoryAlloc(hOs, sizeof(TWlanLinks))) != NULL) {
		pWlanLinks->hOs = hOs;

		return pWlanLinks;
	} else { /* Failed to allocate control block */
		WLAN_OS_REPORT(("FATAL ERROR: wlanLinks_create(): Error allocating cb - aborting\n"));
		return NULL;
	}
}


/**
 * \fn     wlanLinks_Destroy
 * \brief  destroy WlanLinks module
 */
TI_STATUS wlanLinks_Destroy(TI_HANDLE hwlanLinks)
{
	TWlanLinks   *pWlanLinks;

	if (hwlanLinks != NULL) {
		pWlanLinks = (TWlanLinks *)hwlanLinks;

		/* Free pre-allocated control block */
		os_memoryFree(pWlanLinks->hOs, pWlanLinks, sizeof(TWlanLinks));
	}
	return TI_OK;
}


/**
 * \fn     wlanLinks_Init
 * \brief  init WlanLinks module
 */
void wlanLinks_Init (TStadHandlesList *pUwdHandles)
{
	int i;
	TWlanLinks *pWlanLinks = (TWlanLinks *)(pUwdHandles->hWlanLinks);

	os_memorySet(pWlanLinks->hOs, pWlanLinks, 0, sizeof(TWlanLinks));

	pWlanLinks->hOs           = pUwdHandles->hOs;
	pWlanLinks->hReport       = pUwdHandles->hReport;
	pWlanLinks->hEvHandler    = pUwdHandles->hEvHandler;
	pWlanLinks->hTWD          = pUwdHandles->hTWD;
	pWlanLinks->hCtrlData     = pUwdHandles->hCtrlData;

	pWlanLinks->nFree = WLANLINKS_MAX_LINKS;
	for (i=0; i < WLANLINKS_MAX_LINKS; i++) {
		ResetLinkInfo(pWlanLinks, i);
	}
}


/**
 * \fn     wlanLinks_AllocateNew
 * \brief  Allocate new Wlan Link
 */
TI_STATUS wlanLinks_AllocateNew(TI_HANDLE hwlanLinks, TWlanLinkInfo *pLinkInfo, TI_UINT32 *pHlid)
{
	int i;
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;
	TWlanLinkInfo  *pLinks;

	if (!pWlanLinks->nFree) {
		*pHlid = WLANLINKS_INVALID_HLID;
		return TI_NOK;
	}

	pLinks = pWlanLinks->aLinks;
	for (i=0; i < WLANLINKS_MAX_LINKS; i++) {
		if (pLinks[i].eState == WLANLINK_STATE_FREE)
			break;
	}

	/* This shouldn't happen, just if the DB is inconsistent*/
	if (i == WLANLINKS_MAX_LINKS) {
		*pHlid = WLANLINKS_INVALID_HLID;
		return TI_NOK;
	}

	pWlanLinks->nFree--;
	/* Fill link info */
	SetLinkInfo(pWlanLinks, pLinkInfo, i);

	/*set link dynamic parameters*/
	pLinks[i].eState = WLANLINK_STATE_IDLE;
	pLinks[i].eSubState = WLANLINK_SUBSTATE_CONN_NORMAL;
	pLinks[i].bInactive = TI_FALSE;

	*pHlid = pLinks[i].uHlid;

	return TI_OK;
}

/**
 * \fn     wlanLinks_UpdateLinkInfo
 * \brief  Update Wlan Link data except dynamic parameters
 */
TI_STATUS wlanLinks_UpdateLinkInfo(TI_HANDLE hwlanLinks, TWlanLinkInfo *pLinkInfo, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;
	TWlanLinkInfo  *pLinks = pWlanLinks->aLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS) {
		return TI_NOK;
	}

	if (pLinks[uHlid].eState == WLANLINK_STATE_FREE) {
		return TI_NOK;
	}

	SetLinkInfo(pWlanLinks,pLinkInfo,uHlid);

	return TI_OK;
}

/**
 * \fn     wlanLinks_FreeLink
 * \brief  Free Wlan Link
 */
TI_STATUS wlanLinks_FreeLink(TI_HANDLE hwlanLinks, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;
	TWlanLinkInfo  *link;

	if (uHlid >= WLANLINKS_MAX_LINKS) {
		return TI_NOK;
	}

	if (pWlanLinks->nFree == WLANLINKS_MAX_LINKS) {
		return TI_NOK;
	}

	link = &pWlanLinks->aLinks[uHlid];

	if (link->eState == WLANLINK_STATE_FREE) {
		return TI_NOK;
	}

	pWlanLinks->nFree++;
	ResetLinkInfo(pWlanLinks, uHlid);

	return TI_OK;
}

/**
 * \fn     wlanLinks_FindLinkByMac
 * \brief  Find WlanLink by MAC address
 */
TI_STATUS wlanLinks_FindLinkByMac(TI_HANDLE hwlanLinks, TI_UINT8 *mac, TI_UINT32 *uHlid)
{
	int i;
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	for (i=0; i<WLANLINKS_MAX_LINKS; i++) {
		if (pWlanLinks->aLinks[i].eState != WLANLINK_STATE_FREE) {
			if (!os_memoryCompare(pWlanLinks->hOs, pWlanLinks->aLinks[i].tPeerDescr.aMacAddr, mac, MAC_ADDR_LEN)) {
				*uHlid = pWlanLinks->aLinks[i].uHlid;
				return TI_OK;
			}
		}
	}

	*uHlid = WLANLINKS_INVALID_HLID;
	return TI_NOK;
}

/**
 * \fn     wlanLinks_GetPeerDescr
 * \brief  Get Peer description of a link
 */
TI_STATUS wlanLinks_GetPeerDescr(TI_HANDLE hwlanLinks, TI_UINT32 uHlid, TWlanLinkPeerDescr *pPeerDescr)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS) {
		return TI_NOK;
	}

	if (!pPeerDescr) {
		return TI_NOK;
	}

	os_memoryCopy(pWlanLinks->hOs, pPeerDescr, &pWlanLinks->aLinks[uHlid].tPeerDescr, sizeof (TWlanLinkPeerDescr));

	return TI_OK;
}

/**
 * \fn     wlanLinks_PrintDB
 * \brief  Print WlanLinks DB
 */
void wlanLinks_PrintDB(TI_HANDLE hwlanLinks)
{
	TI_UINT32 hlid;
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	os_printf("Wlan Links DB:\n");

	os_printf("Free links: %u\n", pWlanLinks->nFree);

	for (hlid=0; hlid<WLANLINKS_MAX_LINKS; hlid++) {
		if (pWlanLinks->aLinks[hlid].eState != WLANLINK_STATE_FREE) {

			TI_INT8 ssidStr[MAX_SSID_LEN+1];
			os_printf("Link HLID = %u\n", pWlanLinks->aLinks[hlid].uHlid);
			os_printf("role = %u\n", pWlanLinks->aLinks[hlid].eRole);
			os_printf("sub-Role = %u\n", pWlanLinks->aLinks[hlid].eSubRole);
			os_printf("type = %u\n", pWlanLinks->aLinks[hlid].eType);
			os_printf("state = %u\n", pWlanLinks->aLinks[hlid].eState);
			os_printf("sub-State = %u\n", pWlanLinks->aLinks[hlid].eSubState);
			os_printf("quality = %u\n", pWlanLinks->aLinks[hlid].uQuality);
			os_printf("tx Power = %u\n", pWlanLinks->aLinks[hlid].uTxPow);
			os_printf("inactive = %u\n", pWlanLinks->aLinks[hlid].bInactive);
			os_printf("AID = %u\n", pWlanLinks->aLinks[hlid].uAid);
			os_printf("Deauth Reason = %u\n", pWlanLinks->aLinks[hlid].uDeauthReason);

			os_printf("Peer Description:\n");
			os_printf("MAC = %x:%x:%x:%x:%x:%x\n", pWlanLinks->aLinks[hlid].tPeerDescr.aMacAddr[0], pWlanLinks->aLinks[hlid].tPeerDescr.aMacAddr[1], pWlanLinks->aLinks[hlid].tPeerDescr.aMacAddr[2],
			          pWlanLinks->aLinks[hlid].tPeerDescr.aMacAddr[3], pWlanLinks->aLinks[hlid].tPeerDescr.aMacAddr[4], pWlanLinks->aLinks[hlid].tPeerDescr.aMacAddr[5]);
			/* Use temporary array ssidStr for ssid string presentation */
			os_memoryCopy(pWlanLinks->hOs, ssidStr, pWlanLinks->aLinks[hlid].tPeerDescr.tSsid.str, pWlanLinks->aLinks[hlid].tPeerDescr.tSsid.len);
			ssidStr[pWlanLinks->aLinks[hlid].tPeerDescr.tSsid.len] = '\0';
			os_printf("SSID = %s, SSID length = %u\n", ssidStr, pWlanLinks->aLinks[hlid].tPeerDescr.tSsid.len);
			os_printf("band = %u\n", pWlanLinks->aLinks[hlid].tPeerDescr.uBand);
			os_printf("channel = %u\n", pWlanLinks->aLinks[hlid].tPeerDescr.uChannel);
			os_printf("capabilities = 0x%x\n", pWlanLinks->aLinks[hlid].tPeerDescr.uCapablities);
			os_printf("supported rates = 0x%x\n", pWlanLinks->aLinks[hlid].tPeerDescr.uSupRatesBitmap);
			os_printf("--------------------------------------\n");
		}
	}

}

/**
 * \fn     wlanLinks_SetLinkState
 * \brief  Set Wlan Link state
 */
TI_STATUS wlanLinks_SetLinkState(TI_HANDLE hwlanLinks, EWlanLinkState eState, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	pWlanLinks->aLinks[uHlid].eState = eState;

	return TI_OK;

}

/**
 * \fn     wlanLinks_GetLinkState
 * \brief  Get Wlan Link state
 */
TI_STATUS wlanLinks_GetLinkState(TI_HANDLE hwlanLinks, TI_UINT32 uHlid, EWlanLinkState *eState)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	if (!eState)
		return TI_NOK;

	*eState = pWlanLinks->aLinks[uHlid].eState;

	return TI_OK;

}

/**
 * \fn     wlanLinks_SetLinkInactivity
 * \brief  Set Wlan Link inactivity status
 */
TI_STATUS wlanLinks_SetLinkInactivity(TI_HANDLE hWlanLinks, TI_BOOL bInactivity, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hWlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	pWlanLinks->aLinks[uHlid].bInactive = bInactivity;

	return TI_OK;
}

/**
 * \fn     wlanLinks_GetLinkInactivity
 * \brief  Get Wlan Link nactivity status
 */
TI_STATUS wlanLinks_GetLinkInactivity(TI_HANDLE hWlanLinks, TI_UINT32 uHlid, TI_BOOL *pInactivity)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hWlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	if (!pInactivity)
		return TI_NOK;

	*pInactivity = pWlanLinks->aLinks[uHlid].bInactive;

	return TI_OK;

}

/**
 * \fn     wlanLinks_SetDeauthReason
 * \brief  Set Wlan Link deauthentication reason
 */
TI_STATUS wlanLinks_SetDeauthReason(TI_HANDLE hwlanLinks, TI_UINT32 uReason, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	pWlanLinks->aLinks[uHlid].uDeauthReason = uReason;

	return TI_OK;
}

/**
 * \fn     wlanLinks_GetDeauthReason
 * \brief  Get Wlan Link deauthentication reason
 */
TI_STATUS wlanLinks_GetDeauthReason(TI_HANDLE hwlanLinks, TI_UINT32 *pReason, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	if (!pReason)
		return TI_NOK;

	*pReason = pWlanLinks->aLinks[uHlid].uDeauthReason;

	return TI_OK;

}

/**
 * \fn     wlanLinks_GetLinkSupRates
 * \brief  Get Wlan Link supported rates bitmap
 */
TI_STATUS wlanLinks_GetLinkSupRates(TI_HANDLE hwlanLinks, TI_UINT32 *pRates, TI_UINT32 uHlid)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	if (!pRates)
		return TI_NOK;

	*pRates = pWlanLinks->aLinks[uHlid].tPeerDescr.uSupRatesBitmap;

	return TI_OK;

}

/**
 * \fn     wlanLinks_GetLinkType
 * \brief  Get Wlan Link type
 */
TI_STATUS wlanLinks_GetLinkType(TI_HANDLE hwlanLinks, TI_UINT32 uHlid, EWlanLinkType *eType)
{
	TWlanLinks *pWlanLinks = (TWlanLinks *)hwlanLinks;

	if (uHlid >= WLANLINKS_MAX_LINKS)
		return TI_NOK;

	if (!eType)
		return TI_NOK;

	*eType = pWlanLinks->aLinks[uHlid].eType;

	return TI_OK;
}


/**************************************************************************/
/*      LOCAL FUNCTIONS                                                   */
/**************************************************************************/

/**
 * \fn     ResetLinkInfo
 * \brief  Reset WlanLink entry data
 *
 * Note: The assumption here is that all accesses to WlanLink DB
 *       are performed in signle context
 * \param pwlanLinks - WlanLinks module pointer
 * \param uHlid - HLID of the link
 * \return none
 */
static void ResetLinkInfo(TWlanLinks *pWlanLinks, TI_UINT32 uHlid)
{
	os_memorySet(pWlanLinks->hOs, &(pWlanLinks->aLinks[uHlid]), 0, sizeof(pWlanLinks->aLinks[uHlid]));
	pWlanLinks->aLinks[uHlid].uHlid = uHlid;
}

/**
 * \fn     SetLinkInfo
 * \brief  Set Wlan Link date
 *
 * Note 1: The assumption here is that all accesses to WlanLink
 *       DB are performed in signle context
 * Note 2: The function doesn't set dynamic parameters of a
 * link, such as state
 * \param pWlanLinks - WlanLinks module pointer
 * \param pLinkInfo - input Link Info
 * \param uHlid - HLID of the link
 * \return none
 */
static void SetLinkInfo(TWlanLinks *pWlanLinks, TWlanLinkInfo *pLinkInfo, TI_UINT32 uHlid)
{
	TWlanLinkInfo  *aLinks = pWlanLinks->aLinks;

	aLinks[uHlid].eRole  = pLinkInfo->eRole;
	aLinks[uHlid].eSubRole  = pLinkInfo->eSubRole;
	aLinks[uHlid].eType = pLinkInfo->eType;
	os_memoryCopy(pWlanLinks->hOs, &aLinks[uHlid].tPeerDescr, &pLinkInfo->tPeerDescr, sizeof(pLinkInfo->tPeerDescr));
	aLinks[uHlid].uTxPow = pLinkInfo->uTxPow;
	aLinks[uHlid].uQuality = pLinkInfo->uQuality;
	aLinks[uHlid].uAid = pLinkInfo->uAid;
}
