/*
 * templates.c
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

/** \file reportReplvl.c
 *  \brief Report level implementation
 *
 *  \see reportReplvl.h
 */

/***************************************************************************/
/*																		   */
/*		MODULE:	reportReplvl.c											   */
/*    PURPOSE:	Report level implementation	 							   */
/*																		   */
/***************************************************************************/

#define __FILE_ID__  FILE_ID_88
#include "tidef.h"
#include "report.h"
#include "osApi.h"
#include "siteHash.h"
#include "rate.h"
#include "rsnApi.h"
#include "regulatoryDomainApi.h"
#include "siteMgrApi.h"
#include "TWDriver.h"
#include "StaCap.h"
#include "qosMngr_API.h"

#include "roleAP.h"

/********************************************/
/*		Functions Implementation 			*/
/********************************************/

/************************************************************************
 *                        buildNullTemplate								*
 ************************************************************************
DESCRIPTION: This function build a NULL data template to set to the HAL
				when joining an infrastructure network
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure


OUTPUT:


RETURN:     TI_OK

************************************************************************/
TI_STATUS buildNullTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate)
{
	paramInfo_t			param;
	nullDataTemplate_t	*pBuffer = (nullDataTemplate_t	*)pTemplate->ptr;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	TI_UINT16				fc;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(nullDataTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	MAC_COPY (pBuffer->hdr.DA, pPrimarySite->bssid);

	/* Set BSSID address */
	MAC_COPY (pBuffer->hdr.BSSID, pPrimarySite->bssid);

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	MAC_COPY (pBuffer->hdr.SA, param.content.ctrlDataDeviceMacAddress);

	fc = DOT11_FC_DATA_NULL_FUNCTION;
	fc |= (TI_TRUE << DOT11_FC_TO_DS_SHIFT);

	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	pTemplate->len = sizeof(dot11_mgmtHeader_t);
	return TI_OK;
}

/************************************************************************
 *                        buildDisconnTemplate								*
 ************************************************************************
DESCRIPTION: This function build a Death/Disassoc template to set to the HAL
				when joining an infrastructure network
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure


OUTPUT:


RETURN:     TI_OK

************************************************************************/
TI_STATUS buildDisconnTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate)
{
	paramInfo_t			param;
	TI_UINT32				size;
	disconnTemplate_t	*pBuffer = (disconnTemplate_t	*)pTemplate->ptr;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	TI_UINT16				fc;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(disconnTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	MAC_COPY (pBuffer->hdr.DA, pPrimarySite->bssid);

	/* Set BSSID address */
	MAC_COPY (pBuffer->hdr.BSSID, pPrimarySite->bssid);

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	MAC_COPY (pBuffer->hdr.SA, param.content.ctrlDataDeviceMacAddress);

	fc = DOT11_FC_DISASSOC; /* will be change by firmware to DOT11_FC_DEAUTH if needed */


	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	pBuffer->disconnReason = 0; /* filled by firmware */

	size = sizeof(disconnTemplate_t);

	pTemplate->len = size;

	return TI_OK;
}

/**
 * \fn     setDefaultProbeReqTemplate
 * \brief  set Default Probe Req Template tp the FW.
 *
 * set Default Probe Req Template tp the FW.
 *
 * \param  hSiteMgr	-	Handle to site manager
 * \return None
 * \sa
 */
void setDefaultProbeReqTemplate (TI_HANDLE	hSiteMgr)
{
	siteMgr_t	*pSiteMgr = (siteMgr_t *)hSiteMgr;
	TSetTemplate        tTemplateStruct;
	probeReqTemplate_t  tProbeReqTemplate;
	TSsid               tBroadcastSSID;

	/*
	 * Setting probe request temapltes for both bands.
	 * allocating EMPTY 32 bytes for the SSID IE, to reserve space for different SSIDs the FW will set
	 */
	tBroadcastSSID.len = MAX_SSID_LEN;
	os_memorySet (pSiteMgr->hOs, &(tBroadcastSSID.str[ 0 ]), 0, MAX_SSID_LEN);
	tTemplateStruct.ptr = (TI_UINT8 *)&tProbeReqTemplate;
	tTemplateStruct.type = PROBE_REQUEST_TEMPLATE;
	tTemplateStruct.eBand = RADIO_BAND_2_4_GHZ;
	tTemplateStruct.uRateMask = RATE_MASK_UNSPECIFIED;
	buildProbeReqTemplate (hSiteMgr, &tTemplateStruct, &tBroadcastSSID, RADIO_BAND_2_4_GHZ);
	TWD_CmdTemplate (pSiteMgr->hTWD, &tTemplateStruct, NULL, NULL);
	tTemplateStruct.eBand = RADIO_BAND_5_0_GHZ;
	buildProbeReqTemplate (hSiteMgr, &tTemplateStruct, &tBroadcastSSID, RADIO_BAND_5_0_GHZ);
	TWD_CmdTemplate (pSiteMgr->hTWD, &tTemplateStruct, NULL, NULL);
}

/************************************************************************
 *                        buildProbeReqTemplate							*
 ************************************************************************
DESCRIPTION: This function build a probe request template to set to the HAL in the scan process.
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure
			pSsid		-	Desired SSID


OUTPUT:


RETURN:     TI_OK

************************************************************************/
TI_STATUS buildProbeReqTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate, TSsid *pSsid, ERadioBand radioBand)
{
	paramInfo_t			param;
	char				*pBuf;
	int i;
	probeReqTemplate_t	*pBuffer = (probeReqTemplate_t	*)pTemplate->ptr;
	TI_UINT32			 size;
	dot11_RATES_t		*pDot11Rates;
	TI_UINT32			 len = 0, ofdmIndex = 0;
	TI_UINT32			 suppRatesLen, extSuppRatesLen;
	TI_UINT8			 ratesBuf[DOT11_MAX_SUPPORTED_RATES];
	TI_UINT8             WSCOuiIe[DOT11_OUI_LEN+1] = { 0x00, 0x50, 0xf2, 0x04};
	TI_UINT32			 supportedRateMask,basicRateMask;
	TI_UINT16			 fc = DOT11_FC_PROBE_REQ;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(probeReqTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.DA[i] = 0xFF;

	/* Set BSSID address */

	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.BSSID[i] = 0xFF;


	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	MAC_COPY (pBuffer->hdr.SA, param.content.ctrlDataDeviceMacAddress);

	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	size = sizeof(dot11_mgmtHeader_t);
	pBuf = (char *)&(pBuffer->infoElements);

	/*
	* Informataion elements
	*/
	/* SSID */
	/* It looks like it never happens. Anyway decided to check */
	if ( pSsid->len > MAX_SSID_LEN ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return TI_NOK;
	}
	((dot11_SSID_t *)(pBuf))->hdr[0] = DOT11_SSID_ELE_ID;
	((dot11_SSID_t *)(pBuf))->hdr[1] = pSsid->len;
	os_memoryCopy(pSiteMgr->hOs, pBuf + sizeof(dot11_eleHdr_t), (void *)pSsid->str, pSsid->len);
	size += sizeof(dot11_eleHdr_t) + pSsid->len;
	pBuf += sizeof(dot11_eleHdr_t) + pSsid->len;

	/* Rates */
	pDot11Rates = (dot11_RATES_t *) pBuf;

	/*
	 * Supported rates in probe request will always use the default rates for BG or A bands,
	 * regardless of the STA desired rates.
	 */
	if (radioBand == RADIO_BAND_2_4_GHZ) {
		/* Basic rates: 1,2,5.5,11 */
		basicRateMask = rate_BasicToDrvBitmap(pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[DOT11_G_MODE], TI_FALSE);
		/* Extended: 6,9,12,18,24,36,48,54 */
		supportedRateMask = rate_SupportedToDrvBitmap(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[DOT11_G_MODE], TI_FALSE);
	} else if (radioBand == RADIO_BAND_5_0_GHZ) {
		/* Basic rates: 6,12,24 */
		basicRateMask = rate_BasicToDrvBitmap(pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[DOT11_A_MODE], TI_TRUE);
		/* Extended: 9,18,24,36,48,54 */
		supportedRateMask = rate_SupportedToDrvBitmap(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[DOT11_A_MODE], TI_TRUE);
	} else {
		/* Use default and pray for the best */
		/* Basic rates: 1,2,5.5,11 */
		basicRateMask = rate_BasicToDrvBitmap(BASIC_RATE_SET_1_2_5_5_11, TI_FALSE);
		/* Extended: 6,9,12,18,24,36,48,54 */
		supportedRateMask = rate_SupportedToDrvBitmap(SUPPORTED_RATE_SET_UP_TO_54, TI_FALSE);
	}

	rate_DrvBitmapToNetStr (supportedRateMask, basicRateMask, ratesBuf, &len, &ofdmIndex);

	/* It looks like it never happens. Anyway decided to check */
	if ( len > DOT11_MAX_SUPPORTED_RATES ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return TI_NOK;
	}
	if(radioBand == RADIO_BAND_5_0_GHZ ||
	   pSiteMgr->pDesiredParams->siteMgrUseDraftNum == DRAFT_5_AND_EARLIER ||
	   ofdmIndex == len) {
		pDot11Rates->hdr[0] = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = len;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
	} else {
		pDot11Rates->hdr[0] = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr[1]);
		suppRatesLen = pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pDot11Rates = (dot11_RATES_t *) (pBuf + suppRatesLen);
		pDot11Rates->hdr[0] = DOT11_EXT_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = len - ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, &ratesBuf[ofdmIndex], pDot11Rates->hdr[1]);
		extSuppRatesLen = pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		size += suppRatesLen + extSuppRatesLen;
		pBuf += suppRatesLen + extSuppRatesLen;
	}


	/* add HT capabilities IE */
	StaCap_GetHtCapabilitiesIe (pSiteMgr->hStaCap, pBuf, &len);
	size += len;
	pBuf += len;


	/* WiFi Simple Config */
	if (pSiteMgr->includeWSCinProbeReq) {
		if(pSiteMgr->siteMgrWSCCurrMode != TIWLN_SIMPLE_CONFIG_OFF) {
			((dot11_WSC_t *)(pBuf))->hdr[0] = DOT11_WSC_PARAM_ELE_ID;
			((dot11_WSC_t *)(pBuf))->hdr[1] = DOT11_WSC_PROBE_REQ_MAX_LENGTH + DOT11_OUI_LEN + 1;
			pBuf += sizeof(dot11_eleHdr_t);
			/* This looks like a typo: 5 bytes are copied from an array of 4 bytes. Then the last byte is overriten by the next command
			         os_memoryCopy(pSiteMgr->hOs, pBuf, &WSCOuiIe, DOT11_OUI_LEN+2);*/
			os_memoryCopy(pSiteMgr->hOs, pBuf, &WSCOuiIe, DOT11_OUI_LEN+1);
			os_memoryCopy(pSiteMgr->hOs, pBuf + DOT11_OUI_LEN+1, &pSiteMgr->siteMgrWSCProbeReqParams, DOT11_WSC_PROBE_REQ_MAX_LENGTH - (DOT11_OUI_LEN+2));
			size += sizeof(dot11_eleHdr_t) + DOT11_WSC_PROBE_REQ_MAX_LENGTH + DOT11_OUI_LEN + 1;
			pBuf += sizeof(dot11_eleHdr_t) + DOT11_WSC_PROBE_REQ_MAX_LENGTH + DOT11_OUI_LEN + 1;
		}
	}
	pTemplate->len = size;

	return TI_OK;
}





/************************************************************************
 *                        buildProbeRspTemplate							*
************************************************************************/
TI_STATUS templates_buildAPBeaconTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate, TBssCapabilities *pBssCap)
{
	paramInfo_t			param;
	TI_UINT8			*pBuf;
	TI_INT32			i ,j;
	TI_UINT32			size;
	dot11_RATES_t		*pDot11Rates;
	dot11_ERP_t         *pdot11Erp;
	TI_UINT32			len = 0, ofdmIndex = 0;
	TI_BOOL		        extRates = TI_FALSE;
	TCountry			*pCountry = NULL;
	TI_UINT8			ratesBuf[DOT11_MAX_SUPPORTED_RATES];
	TI_UINT32			supportedRateMask = 0,basicRateMask = 0;
	TI_UINT16			headerFC = DOT11_FC_BEACON;
	beaconTemplate_t	*pBuffer = (beaconTemplate_t	*)pTemplate->ptr; /* beacon template holds all fiels the probe resp needs*/
	TI_UINT8            tempTimIELen = 4;


	if (pTemplate->type == AP_BEACON_TEMPLATE) {
		os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(beaconTemplate_t));
	} else {
		os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(probeRspTemplate_t));
	}


	/*
	 * Build WLAN Header:
	 * ==================
	 */

	/* Set destination address as broadcast*/
	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.DA[i] = 0xFF;

	/* Set BSSID address */
	MAC_COPY (pBuffer->hdr.BSSID, pBssCap->tBssid);

	/* Build Source address */
	MAC_COPY (pBuffer->hdr.SA, pBssCap->tMacAddress);


	COPY_WLAN_WORD(&pBuffer->hdr.fc, &headerFC);

	size = sizeof(dot11_mgmtHeader_t);
	pBuf = (TI_UINT8 *)pBuffer->timeStamp;
	/*
	* Fixed Fields
	*/
	/* we skip the timestamp field */
	size += TIME_STAMP_LEN;
	pBuf += TIME_STAMP_LEN;

	/* Beacon interval */
	COPY_WLAN_WORD(pBuf, &(pBssCap->uBeaconInterval));
	size += FIX_FIELD_LEN;
	pBuf += FIX_FIELD_LEN;



	/* capabilities information */
	pBssCap->uCapabilities |= DOT11_CAPS_ESS;

	if (pBssCap->bUseShortPreamble) {
		pBssCap->uCapabilities |= DOT11_CAPS_SHORT_PREAMBLE;
	}

	if (pBssCap->bUseShortSlotTime) {
		pBssCap->uCapabilities |= DOT11_CAPS_SHORT_SLOT_TIME;
	}

	COPY_WLAN_WORD(pBuf, &(pBssCap->uCapabilities));
	size += FIX_FIELD_LEN;
	pBuf += FIX_FIELD_LEN;



	/*
	 * Build Informataion Elements:
	 * ============================
	 */

	/* SSID IE */
	/* It looks like it never happens. Anyway decided to check */
	if ( pBssCap->tSsid.len > MAX_SSID_LEN ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return TI_NOK;
	}

	((dot11_SSID_t *)(pBuf))->hdr[0] = DOT11_SSID_ELE_ID;

	if (pTemplate->type == AP_BEACON_TEMPLATE && pBssCap->eSsidType == SSID_TYPE_HIDDEN) {
		((dot11_SSID_t *)(pBuf))->hdr[1] = 0;
		os_memoryZero(pSiteMgr->hOs, pBuf + sizeof(dot11_eleHdr_t), pBssCap->tSsid.len);
		size += sizeof(dot11_eleHdr_t);
		pBuf += sizeof(dot11_eleHdr_t);
	} else { /* Probe response Template */
		((dot11_SSID_t *)(pBuf))->hdr[1] = pBssCap->tSsid.len;
		os_memoryCopy(pSiteMgr->hOs, pBuf + sizeof(dot11_eleHdr_t), (void *)pBssCap->tSsid.str, pBssCap->tSsid.len);
		size += sizeof(dot11_eleHdr_t) + pBssCap->tSsid.len;
		pBuf += sizeof(dot11_eleHdr_t) + pBssCap->tSsid.len;
	}



	/* Rates IE */
	pDot11Rates = (dot11_RATES_t *) pBuf;

	/* [LiorC] Set Default basic Rates for phase 1 only */
	supportedRateMask = rate_GetDrvBitmapForDefaultSupporteSet ();
	basicRateMask	  = rate_GetDrvBitmapForDefaultBasicSet ();

	supportedRateMask = rate_SupportedToDrvBitmap (SUPPORTED_RATE_SET_UP_TO_54, TI_FALSE);

	rate_DrvBitmapToNetStr (supportedRateMask, basicRateMask, ratesBuf, &len, &ofdmIndex);


	if (pSiteMgr->siteMgrOperationalMode == DOT11_B_MODE) {
		pDot11Rates->hdr[0] = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = len;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
	} else {
		pDot11Rates->hdr[0] = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		extRates = TI_TRUE;
	}

	/* DS IE */
	((dot11_DS_PARAMS_t *)(pBuf))->hdr[0] = DOT11_DS_PARAMS_ELE_ID;
	((dot11_DS_PARAMS_t *)(pBuf))->hdr[1] = DOT11_DS_PARAMS_ELE_LEN;
	((dot11_DS_PARAMS_t *)(pBuf))->currChannel = pBssCap->uChannel;
	size += sizeof(dot11_eleHdr_t) + DOT11_DS_PARAMS_ELE_LEN;
	pBuf += sizeof(dot11_eleHdr_t) + DOT11_DS_PARAMS_ELE_LEN;


	/* TIM IE (Short style - limited to only 8 station in AP Mode !!) */
	if (pTemplate->type == AP_BEACON_TEMPLATE) {
		((dot11_TIM_Short_t *)(pBuf))->hdr[0] = DOT11_TIM_ELE_ID;
		((dot11_TIM_Short_t *)(pBuf))->hdr[1] = tempTimIELen; /* [LiorC] need to be changed later!!! */
		((dot11_TIM_Short_t *)(pBuf))->dtimCount = 0;
		((dot11_TIM_Short_t *)(pBuf))->dtimPeriod = 0;
		((dot11_TIM_Short_t *)(pBuf))->bmapControl = 0;
		((dot11_TIM_Short_t *)(pBuf))->partialVirtualBmap = 0;
		size += sizeof(dot11_eleHdr_t) + DOT11_TIM_SHORT_PARAMS_ELE_LEN;
		pBuf += sizeof(dot11_eleHdr_t) + DOT11_TIM_SHORT_PARAMS_ELE_LEN;
	}

	/* Country IE */
	param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
	regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);

	if(	param.content.regulatoryDomainEnabled == TI_TRUE ) {
		/* get country IE */
		param.paramType = REGULATORY_DOMAIN_COUNTRY_PARAM;
		regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain, &param);
		pCountry = param.content.pCountry;

		/* Check if a country IE was found */
		if(pCountry != NULL) {
			*pBuf = DOT11_COUNTRY_ELE_ID;
			pBuf++;
			size++;
			*pBuf = (TI_UINT8)(pCountry->len);
			pBuf++;
			size++;

			/* Note: The country structure is not byte-aligned so it is copied as follows to ensure
			           that there are no gaps in the output structure (pBuf). */

			os_memoryCopy(pSiteMgr->hOs, pBuf , &pCountry->countryIE.CountryString, DOT11_COUNTRY_STRING_LEN);
			pBuf += DOT11_COUNTRY_STRING_LEN;
			size += DOT11_COUNTRY_STRING_LEN;

			/* Loop on all tripletChannels. Each item has three fields ('i' counts rows and 'j' counts bytes). */
			for (i = 0, j = 0;  j < (pCountry->len - DOT11_COUNTRY_STRING_LEN);  i++, j+=3) {
				*(pBuf + j    ) = pCountry->countryIE.tripletChannels[i].firstChannelNumber;
				*(pBuf + j + 1) = pCountry->countryIE.tripletChannels[i].maxTxPowerLevel;
				*(pBuf + j + 2) = pCountry->countryIE.tripletChannels[i].numberOfChannels;
			}

			pBuf += (pCountry->len - DOT11_COUNTRY_STRING_LEN);
			size += (pCountry->len - DOT11_COUNTRY_STRING_LEN);
		}
	}



	/* ERP IE */
	pdot11Erp = (dot11_ERP_t *) pBuf;
	pdot11Erp->hdr[0] = DOT11_ERP_IE_ID;
	pdot11Erp->hdr[1] = 1;
	pdot11Erp->ctrl = 0;


	if (pBssCap->bUseProtection == TI_TRUE) {
		pdot11Erp->ctrl |= ERP_IE_USE_PROTECTION_MASK;
	}
	if (pBssCap->bUseShortPreamble == TI_TRUE) {
		pdot11Erp->ctrl |= ERP_IE_BARKER_PREAMBLE_MODE_MASK;
	}

	if (0/* Add condition here - ERP presence */) {
		pdot11Erp->ctrl |= ERP_IE_NON_ERP_PRESENT_MASK;
	}
	size += pdot11Erp->hdr[1] + sizeof(dot11_eleHdr_t);
	pBuf += pdot11Erp->hdr[1] + sizeof(dot11_eleHdr_t);


	/* Extended supported rates IE */
	if(extRates) {
		pDot11Rates = (dot11_RATES_t *) pBuf;
		pDot11Rates->hdr[0] = DOT11_EXT_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = len - ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, &ratesBuf[ofdmIndex], pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
	}

	/* In the future: Add RSN IE here if needed... */

	pTemplate->len = size;


	WLAN_OS_REPORT(("\n************************ %s INFO: **********************************\n", ((pTemplate->type == AP_BEACON_TEMPLATE)?"BEACON TEMPLATE":"PROBE RESPONSE TEMPLATE")));

	WLAN_OS_REPORT(("\n--------------- Supported Rates (%d) -------------------\n", ofdmIndex));

	for (i=0; i< ofdmIndex ; i++) {
		WLAN_OS_REPORT(("%d) 0x%02x\n",i, ratesBuf[i]));
	}

	WLAN_OS_REPORT(("\n--------------- Extended Supported Rates (%d) -------------------\n", (len - ofdmIndex)));
	for (i = ofdmIndex; i< len ; i++) {
		WLAN_OS_REPORT(("%d) 0x%02x\n", (i - ofdmIndex), ratesBuf[i]));
	}

	WLAN_OS_REPORT(("\n MAC ADDRESS = %02x.%02x.%02x.%02x.%02x.%02x.\n",
	                pBuffer->hdr.SA[0],pBuffer->hdr.SA[1],pBuffer->hdr.SA[2],pBuffer->hdr.SA[3],pBuffer->hdr.SA[4],pBuffer->hdr.SA[5]));
	WLAN_OS_REPORT(("\n BSSID = %02x.%02x.%02x.%02x.%02x.%02x.\n",
	                pBuffer->hdr.BSSID[0],pBuffer->hdr.BSSID[1],pBuffer->hdr.BSSID[2],pBuffer->hdr.BSSID[3],pBuffer->hdr.BSSID[4],pBuffer->hdr.BSSID[5]));
	WLAN_OS_REPORT(("\n SSID = %s: len = %d \n",pBssCap->tSsid.str, pBssCap->tSsid.len));
	WLAN_OS_REPORT(("\n Channel = %d \n",pBssCap->uChannel));
	WLAN_OS_REPORT(("\n beaconInterval = 0x%04X \n",pBssCap->uBeaconInterval));
	WLAN_OS_REPORT(("\n Capabilitiles = 0x%04X  \n" ,pBssCap->uCapabilities));
	WLAN_OS_REPORT(("\n Erp IE = 0x%02x  \n" ,pdot11Erp->ctrl));
	WLAN_OS_REPORT(("\n Supported Rate IE length = %d  \n" ,ofdmIndex));
	WLAN_OS_REPORT(("\n EXT-Supported Rate IE length = %d  \n" ,pDot11Rates->hdr[1]));
	WLAN_OS_REPORT(("\n pTemplate->len = %d \n",pTemplate->len));

	WLAN_OS_REPORT(("\n*********************************************************************\n\n", ((pTemplate->type == AP_BEACON_TEMPLATE)?"BEACON TEMPLATE":"PROBE RESPONSE TEMPLATE")));


	return TI_OK;
}





/************************************************************************
 *                        buildProbeRspTemplate							*
 ************************************************************************
DESCRIPTION: This function build a probe response template to set to the HAL
				when joining an IBSS network.
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr
				-	The template is built based on the chosen site attributes

			NOTE: This function is used to build beacon template too.
			The site manager set the template type (after thos function returns) to beacon or probe response accordingly.

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure


OUTPUT:


RETURN:     TI_OK

************************************************************************/
TI_STATUS buildProbeRspTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate)
{
	paramInfo_t			param;
	TI_UINT8			*pBuf;
	probeRspTemplate_t	*pBuffer = (probeRspTemplate_t	*)pTemplate->ptr;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	TI_INT32			i, j;
	TI_UINT32			size;
	dot11_RATES_t		*pDot11Rates;
	dot11_ERP_t         *pdot11Erp;
	TI_UINT32			len = 0, ofdmIndex = 0;
	TI_BOOL				extRates = TI_FALSE;
	TI_BOOL             useProtection,NonErpPresent,barkerPreambleType;
	TCountry			*pCountry = NULL;
	TI_UINT8			ratesBuf[DOT11_MAX_SUPPORTED_RATES];
	TI_UINT32			supportedRateMask,basicRateMask;
	TI_UINT16			headerFC = DOT11_FC_PROBE_RESP;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(probeRspTemplate_t));


	/*
	 * Build WLAN Header:
	 * ==================
	 */

	/* Set destination address */
	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.DA[i] = 0xFF;

	/* Set BSSID address */
	MAC_COPY (pBuffer->hdr.BSSID, pPrimarySite->bssid);

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	MAC_COPY (pBuffer->hdr.SA, param.content.ctrlDataDeviceMacAddress);

	COPY_WLAN_WORD(&pBuffer->hdr.fc, &headerFC);

	size = sizeof(dot11_mgmtHeader_t);
	pBuf = (TI_UINT8 *)pBuffer->timeStamp;
	/*
	* Fixed Fields
	*/
	/* we skip the timestamp field */
	size += TIME_STAMP_LEN;
	pBuf += TIME_STAMP_LEN;

	/* Beacon interval */
	COPY_WLAN_WORD(pBuf, &pPrimarySite->beaconInterval);
	size += FIX_FIELD_LEN;
	pBuf += FIX_FIELD_LEN;

	/* capabilities */
	COPY_WLAN_WORD(pBuf, &pPrimarySite->capabilities);
	size += FIX_FIELD_LEN;
	pBuf += FIX_FIELD_LEN;

	/*
	 * Build Informataion Elements:
	 * ============================
	 */

	/* SSID IE */
	/* It looks like it never happens. Anyway decided to check */
	if ( pPrimarySite->ssid.len > MAX_SSID_LEN ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return TI_NOK;
	}
	((dot11_SSID_t *)(pBuf))->hdr[0] = DOT11_SSID_ELE_ID;
	((dot11_SSID_t *)(pBuf))->hdr[1] = pPrimarySite->ssid.len;
	os_memoryCopy(pSiteMgr->hOs, pBuf + sizeof(dot11_eleHdr_t), (void *)pPrimarySite->ssid.str, pPrimarySite->ssid.len);
	size += sizeof(dot11_eleHdr_t) + pPrimarySite->ssid.len;
	pBuf += sizeof(dot11_eleHdr_t) + pPrimarySite->ssid.len;


	/* Rates IE */
	pDot11Rates = (dot11_RATES_t *) pBuf;

	if (pPrimarySite->channel == SPECIAL_BG_CHANNEL) {
		supportedRateMask = rate_GetDrvBitmapForDefaultSupporteSet ();
		basicRateMask	  = rate_GetDrvBitmapForDefaultBasicSet ();
	} else {
		supportedRateMask = pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask;
		basicRateMask     = pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
	}

	rate_DrvBitmapToNetStr (supportedRateMask, basicRateMask, ratesBuf, &len, &ofdmIndex);

	if(pSiteMgr->siteMgrOperationalMode != DOT11_G_MODE ||
	   pSiteMgr->pDesiredParams->siteMgrUseDraftNum == DRAFT_5_AND_EARLIER ||
	   ofdmIndex == len) {
		pDot11Rates->hdr[0] = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = len;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
	} else {
		pDot11Rates->hdr[0] = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		extRates = TI_TRUE;
	}

	/* DS IE */
	((dot11_DS_PARAMS_t *)(pBuf))->hdr[0] = DOT11_DS_PARAMS_ELE_ID;
	((dot11_DS_PARAMS_t *)(pBuf))->hdr[1] = DOT11_DS_PARAMS_ELE_LEN;
	((dot11_DS_PARAMS_t *)(pBuf))->currChannel = pPrimarySite->channel;
	size += sizeof(dot11_eleHdr_t) + DOT11_DS_PARAMS_ELE_LEN;
	pBuf += sizeof(dot11_eleHdr_t) + DOT11_DS_PARAMS_ELE_LEN;

	/* IBSS IE */
	((dot11_IBSS_PARAMS_t *)(pBuf))->hdr[0] = DOT11_IBSS_PARAMS_ELE_ID;
	((dot11_IBSS_PARAMS_t *)(pBuf))->hdr[1] = DOT11_IBSS_PARAMS_ELE_LEN;
	COPY_WLAN_WORD(&((dot11_IBSS_PARAMS_t *)(pBuf))->atimWindow, &pPrimarySite->atimWindow);
	size += sizeof(dot11_eleHdr_t) + DOT11_IBSS_PARAMS_ELE_LEN;
	pBuf += sizeof(dot11_eleHdr_t) + DOT11_IBSS_PARAMS_ELE_LEN;

	/* Country IE */
	param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
	regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);

	if(	param.content.regulatoryDomainEnabled == TI_TRUE ) {
		/* get country IE */
		param.paramType = REGULATORY_DOMAIN_COUNTRY_PARAM;
		regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain, &param);
		pCountry = param.content.pCountry;

		/* Check if a country IE was found */
		if(pCountry != NULL) {
			*pBuf = DOT11_COUNTRY_ELE_ID;
			pBuf++;
			size++;
			*pBuf = (TI_UINT8)(pCountry->len);
			pBuf++;
			size++;

			/* Note: The country structure is not byte-aligned so it is copied as follows to ensure
			           that there are no gaps in the output structure (pBuf). */

			os_memoryCopy(pSiteMgr->hOs, pBuf , &pCountry->countryIE.CountryString, DOT11_COUNTRY_STRING_LEN);
			pBuf += DOT11_COUNTRY_STRING_LEN;
			size += DOT11_COUNTRY_STRING_LEN;

			/* Loop on all tripletChannels. Each item has three fields ('i' counts rows and 'j' counts bytes). */
			for (i = 0, j = 0;  j < (pCountry->len - DOT11_COUNTRY_STRING_LEN);  i++, j+=3) {
				*(pBuf + j    ) = pCountry->countryIE.tripletChannels[i].firstChannelNumber;
				*(pBuf + j + 1) = pCountry->countryIE.tripletChannels[i].maxTxPowerLevel;
				*(pBuf + j + 2) = pCountry->countryIE.tripletChannels[i].numberOfChannels;
			}

			pBuf += (pCountry->len - DOT11_COUNTRY_STRING_LEN);
			size += (pCountry->len - DOT11_COUNTRY_STRING_LEN);
		}
	}

	/*ERP IE*/
	siteMgr_IsERP_Needed(pSiteMgr,&useProtection,&NonErpPresent,&barkerPreambleType);
	if (useProtection || NonErpPresent || barkerPreambleType) {
		pdot11Erp = (dot11_ERP_t *) pBuf;
		pdot11Erp->hdr[0] = DOT11_ERP_IE_ID;
		pdot11Erp->hdr[1] = 1;
		pdot11Erp->ctrl = 0;
		if (NonErpPresent)
			pdot11Erp->ctrl |= ERP_IE_NON_ERP_PRESENT_MASK;
		if (useProtection)
			pdot11Erp->ctrl |= ERP_IE_USE_PROTECTION_MASK;
		if (barkerPreambleType)
			pdot11Erp->ctrl |= ERP_IE_BARKER_PREAMBLE_MODE_MASK;
		size += pdot11Erp->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pdot11Erp->hdr[1] + sizeof(dot11_eleHdr_t);

	}


	/* Extended supported rates IE */
	if(extRates) {
		pDot11Rates = (dot11_RATES_t *) pBuf;
		pDot11Rates->hdr[0] = DOT11_EXT_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr[1] = len - ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, &ratesBuf[ofdmIndex], pDot11Rates->hdr[1]);
		size += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr[1] + sizeof(dot11_eleHdr_t);
	}

	/* no need to insert RSN information elements */

	pTemplate->len = size;

	return TI_OK;
}

/************************************************************************
 *                        buildPsPollTemplate							*
 ************************************************************************
DESCRIPTION: This function build a ps poll template
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure
			pSsid		-	Desired SSID

OUTPUT:

RETURN:     TI_OK
************************************************************************/
TI_STATUS buildPsPollTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate)
{
	paramInfo_t			param;
	TTwdParamInfo       tTwdParam;
	TI_UINT32				size;
	psPollTemplate_t	*pBuffer = (psPollTemplate_t *)pTemplate->ptr;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	TI_UINT16				fc;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(psPollTemplate_t));

	/*
	 * Header First
	 */

	/* Set BSSID address */
	MAC_COPY (pBuffer->hdr.BSSID, pPrimarySite->bssid);

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	MAC_COPY (pBuffer->hdr.TA, param.content.ctrlDataDeviceMacAddress);

	/*
	**   Building the Frame Control word (16 bits)
	** ---------------------------------------------
	** Type = Control
	** SubType = Power Save (PS) POLL,  */
	fc = DOT11_FC_PS_POLL;
	/*
	** setting the Power Management bit in the Frame control field
	** to be "Power Save mode"
	*/
	fc |= (0x1 << DOT11_FC_PWR_MGMT_SHIFT);

	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	/*
	**   Association ID
	** -----------------
	*/
	tTwdParam.paramType = TWD_AID_PARAM_ID;
	TWD_GetParam (pSiteMgr->hTWD, &tTwdParam);

	/* AID should have its two MSB bit Set to "1"*/
	pBuffer->hdr.AID = tTwdParam.content.halCtrlAid | 0xC000;

	size = sizeof(dot11_PsPollFrameHeader_t);

	pTemplate->len = size;

	return TI_OK;
}


/************************************************************************
 *                        buildQosNullDataTemplate							*
 ************************************************************************
DESCRIPTION: This function build a qos null data template
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure
			pSsid		-	Desired SSID

OUTPUT:

RETURN:     TI_OK
************************************************************************/
TI_STATUS buildQosNullDataTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate, TI_UINT8 userPriority)
{
	paramInfo_t			param;
	TI_UINT32				size;
	QosNullDataTemplate_t	*pBuffer = (QosNullDataTemplate_t	*)pTemplate->ptr;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	TI_UINT16				fc;
	TI_UINT16				qosControl;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(QosNullDataTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	if (pPrimarySite) {
		MAC_COPY (pBuffer->hdr.address1, pPrimarySite->bssid);

		/* Set BSSID address */
		MAC_COPY (pBuffer->hdr.address3, pPrimarySite->bssid);
	}

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	MAC_COPY (pBuffer->hdr.address2, param.content.ctrlDataDeviceMacAddress);

	fc = DOT11_FC_DATA_NULL_QOS | (1 << DOT11_FC_TO_DS_SHIFT);
	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	qosControl = (TI_UINT16)userPriority;
	qosControl <<= QOS_CONTROL_UP_SHIFT;
	COPY_WLAN_WORD(&pBuffer->hdr.qosControl, &qosControl); /* copy with endianess handling. */

	size = WLAN_QOS_HDR_LEN;

	pTemplate->len = size;

	return TI_OK;
}


/************************************************************************
 *                        buildArpRspTemplate								*
 ************************************************************************
DESCRIPTION: This function build a ARP Response template to set to the HAL
				when joining an infrastructure network
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr

INPUT:      pSiteMgr	-	Handle to site manager
			pTemplate	-	Pointer to the template structure


OUTPUT:


RETURN:     TI_OK

************************************************************************/
TI_STATUS buildArpRspTemplate(siteMgr_t *pSiteMgr, TSetTemplate *pTemplate, TIpAddr staIp)
{
	ArpRspTemplate_t   *pBuffer = (ArpRspTemplate_t *)pTemplate->ptr;
	paramInfo_t param;
	TI_UINT16 fc;
	TI_UINT8 *ptr = (TI_UINT8 *)pBuffer;
	TI_UINT16 offset, len;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(ArpRspTemplate_t));
	/* Fill the WLAN header fields as in the NULL template */
	buildQosNullDataTemplate(pSiteMgr, pTemplate, 0 /* userPriority*/);
	/* overwrite the fc bytes by data_QOS */
	fc = DOT11_FC_DATA_QOS | (1 << DOT11_FC_TO_DS_SHIFT);
	COPY_WLAN_WORD(&pBuffer->hdr.fc, &fc); /* copy with endianess handling. */

	pBuffer->LLC.DSAP = 0xaa;
	pBuffer->LLC.SSAP = 0xaa;
	pBuffer->LLC.Control = 0x03;
	/* pBuffer->LLC.Control.OUI these 3 bytes are zeroed already */
	pBuffer->LLC.Type = WLANTOHS((TI_UINT16)0x806);

	pBuffer->hardType = WLANTOHS((TI_UINT16)1);
	pBuffer->protType = WLANTOHS((TI_UINT16)0x800);
	pBuffer->hardSize = 6;
	pBuffer->protSize = 4;
	pBuffer->op = WLANTOHS((TI_UINT16)2); /*filled as for ARP-RSP, not for RARP_RSP */
	MAC_COPY (pBuffer->StaMac, pBuffer->hdr.address2);
	IP_COPY(pBuffer->StaIp, staIp);

	pTemplate->len = sizeof(ArpRspTemplate_t);

	/* Get QoS type */
	param.paramType = QOS_MNGR_ACTIVE_PROTOCOL;
	qosMngr_getParams(pSiteMgr->hQosMngr, &param);

	if(param.content.qosSiteProtocol == QOS_NONE) {
		/* if QOS is inactive the QoS Control field MUST be absent.
		   Remove the field and move the rest of the packet accordingly */
		offset = sizeof(dot11_header_t);
		len = sizeof(ArpRspTemplate_t) - sizeof(dot11_header_t);
		ptr= (TI_UINT8 *)&pBuffer->LLC.DSAP;
		/* hope this will work in spite of buffers overlap */
		os_memoryCopy(pSiteMgr->hOs, ptr-2, ptr, len);
		pTemplate->len -=2;
	}
	return TI_OK;
}





