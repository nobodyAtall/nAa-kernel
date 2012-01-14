/*
 * sme.c
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

/** \file  sme.c
 *  \brief SME implementation
 *
 *  \see   sme.h, smeSm.c smePrivate.h
 */


#define __FILE_ID__  FILE_ID_41
#include "smePrivate.h"
#include "GenSM.h"
#include "scanResultTable.h"
#include "smeSm.h"
#include "siteMgrApi.h"
#include "regulatoryDomainApi.h"
#include "connApi.h"


/**
 * \fn     sme_Create
 * \brief  Creates the SME module. Allocates system resources
 *
 * Creates the SME module. Allocates system resources
 *
 * \param  hOS - handle to the OS adaptation layer
 * \return Handle to the SME object
 * \sa     sme_Init, sme_SetDefaults, sme_Destroy
 */
TI_HANDLE sme_Create (TI_HANDLE hOS)
{
	TSme    *pSme;

	/* allocate space for the SME object */
	pSme = (TSme*)os_memoryAlloc (hOS, sizeof (TSme));
	if (NULL == pSme) {
		WLAN_OS_REPORT (("sme_Create: unable to allocate memor yfor SME object. SME craetion failed\n"));
		return NULL;
	}

	/*  nullify SME object */
	os_memoryZero (hOS, (void*)pSme, sizeof (TSme));

	/* store OS handle */
	pSme->hOS = hOS;

	/* Create SME generic state-machine */
	pSme->hSmeSm = genSM_Create (hOS);
	if (NULL == pSme->hSmeSm) {
		WLAN_OS_REPORT (("sme_Create: unable to create SME generic SM. SME creation failed\n"));
		sme_Destroy ((TI_HANDLE)pSme);
		return NULL;
	}

	/* Create SME scan result table */
	pSme->hScanResultTable = scanResultTable_Create (hOS);
	if (NULL == pSme->hScanResultTable) {
		WLAN_OS_REPORT (("sme_Create: unable to create scan result table. SME creation failed\n"));
		sme_Destroy ((TI_HANDLE)pSme);
		return NULL;
	}

	return (TI_HANDLE)pSme;
}

/**
 * \fn     sme_Init
 * \brief  Initializes the SME object. Store module handles
 *
 * Initializes the SME object. Store module handles
 *
 * \param  pStadHandles - pointer to the handles structure
 * \return None
 * \sa     sme_Create, sme_SetDefaults
 */
void sme_Init (TStadHandlesList *pStadHandles)
{
	TSme        *pSme = pStadHandles->hSme;

	/* Store object handles */
	pSme->hReport       = pStadHandles->hReport;
	pSme->hScanCncn     = pStadHandles->hScanCncn;
	pSme->hApConn       = pStadHandles->hAPConnection;
	pSme->hConn         = pStadHandles->hConn;
	pSme->hScr          = pStadHandles->hSCR;
	pSme->hRegDomain    = pStadHandles->hRegulatoryDomain;
	pSme->hEvHandler    = pStadHandles->hEvHandler;
	pSme->hSiteMgr      = pStadHandles->hSiteMgr;
	pSme->hRsn          = pStadHandles->hRsn;
	pSme->hDrvMain      = pStadHandles->hDrvMain;
	pSme->hTwd          = pStadHandles->hTWD;


	/* Initialize the scan result table object */
	scanResultTable_Init (pSme->hScanResultTable, pStadHandles);

	/* Initialize the SME state-machine object */
	genSM_Init (pSme->hSmeSm, pStadHandles->hReport);
}

/**
 * \fn     sme_SetDefaults
 * \brief  Set default values to the SME (and the SM and scan result table)
 *
 * Set default values to the SME (and the SM and scan result table)
 *
 * \param  hSme - handle to the SME object
 * \param  pInitParams - values read from registry / ini file
 * \return None
 * \sa     sme_Create, sme_Init
 */
void sme_SetDefaults (TI_HANDLE hSme, TSmeModifiedInitParams *pModifiedInitParams, TSmeInitParams *pInitParams)
{
	TSme        *pSme = (TSme*)hSme;

	/* copy init params */
	os_memoryCopy (pSme->hOS, &(pSme->tInitParams), pInitParams, sizeof (TSmeInitParams));

	/* initialize SME varaibles */
	pSme->bRadioOn = pModifiedInitParams->bRadioOn;
	pSme->eConnectMode = pModifiedInitParams->eConnectMode;
	pSme->eBssType = pModifiedInitParams->eDesiredBssType;
	MAC_COPY (pSme->tBssid, pModifiedInitParams->tDesiredBssid);

	pSme->tSsid.len = pModifiedInitParams->tDesiredSsid.len;
	/* It looks like it never happens. Anyway decided to check */
	if ( pSme->tSsid.len > MAX_SSID_LEN ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return;
	}
	os_memoryCopy (pSme->hOS, &(pSme->tSsid.str[ 0 ]), &(pModifiedInitParams->tDesiredSsid.str[ 0 ]), pModifiedInitParams->tDesiredSsid.len);
	if (OS_802_11_SSID_JUNK (pSme->tSsid.str, pSme->tSsid.len)) {
		pSme->eSsidType = SSID_TYPE_INVALID;
		pSme->bConnectRequired = TI_FALSE;
	} else if (0 == pSme->tSsid.len) {
		pSme->eSsidType = SSID_TYPE_ANY;
		pSme->bConnectRequired = TI_TRUE;
	} else {
		pSme->eSsidType = SSID_TYPE_SPECIFIC;
		pSme->bConnectRequired = TI_TRUE;
	}

	pSme->eLastConnectMode = CONNECT_MODE_AUTO;
	pSme->bAuthSent = TI_FALSE;
	pSme->bReselect = TI_FALSE;
	pSme->uScanCount = 0;
	pSme->bRunning = TI_FALSE;

	/* Initialize the SME state-machine */
	genSM_SetDefaults (pSme->hSmeSm, SME_SM_NUMBER_OF_STATES, SME_SM_NUMBER_OF_EVENTS, (TGenSM_matrix)tSmMatrix,
	                   SME_SM_STATE_IDLE, "SME SM", uStateDescription, uEventDescription, __FILE_ID__);

	/* register scan conecntrator CB */
	scanCncn_RegisterScanResultCB (pSme->hScanCncn, SCAN_SCC_DRIVER, sme_ScanResultCB, hSme);
}

/**
 * \fn     sme_Destroy
 * \brief  Destroys the SME object. De-allocates system resources
 *
 * Destroys the SME object. De-allocates system resources
 *
 * \param  hSme - handle to the SME object
 * \return None
 * \sa     sme_Create
 */
void sme_Destroy (TI_HANDLE hSme)
{
	TSme        *pSme = (TSme*)hSme;

	/* destroy the scan result table */
	if (NULL != pSme->hScanResultTable) {
		scanResultTable_Destroy (pSme->hScanResultTable);
	}

	/* destroy the SME generic state machine */
	if (NULL != pSme->hSmeSm) {
		genSM_Unload (pSme->hSmeSm);
	}

	/* free the SME object */
	os_memoryFree (pSme->hOS, hSme, sizeof (TSme));
}

/**
 * \fn     sme_Start
 * \brief  Starts SME operation
 *
 * Starts SME operation. Send probe request templates and send a start event to the SM.
 * Only the DrvMain module could & is call that function !!!
 *
 * \param  hSme - handle to the SME object
 * \return None
 * \sa     sme_Stop
 */
void sme_Start (TI_HANDLE hSme)
{
	TSme                *pSme = (TSme*)hSme;


	pSme->bRunning = TI_TRUE;

	/*
	 * call setDefaultProbeReqTemplate at sme_Start() due to the fact in order to set prob req template
	 * all moudules need to be set already
	 */
	setDefaultProbeReqTemplate (pSme->hSiteMgr);

	/* if radio is on, start the SM */
	if (TI_TRUE == pSme->bRadioOn) {
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_START, hSme);
	}
}

/**
 * \fn     sme_Stop
 * \brief  Stops the driver (shuts-down the radio)
 *
 * Stops the driver (shuts-down the radio)
 *
 * \param  hSme - handle to the SME object
 * \param  pCBFunc - callback function to be called when stop operation is doen
 * \param  hCBHanlde - handle to supply to the callback function
 * \return None
 * \sa     sme_Start
 */
void sme_Stop (TI_HANDLE hSme)
{
	TSme                *pSme = (TSme*)hSme;


	pSme->bRunning = TI_FALSE;

	/* mark that running flag is send a stop event to the SM */
	genSM_Event (pSme->hSmeSm, SME_SM_EVENT_STOP, hSme);
}

/**
 * \fn     sme_Restart
 * \brief  Called due to a paramter value change in site mgr. Triggers a disconnect.
 *
 * Called due to a paramter value change in site mgr. Triggers a disconnect.
 *
 * \param  hSme - handle to the SME object
 * \param  eReason - the reason for restarting the SME
 * \return None
 */
void sme_Restart (TI_HANDLE hSme)
{
	TSme                *pSme = (TSme*)hSme;


	pSme->uScanCount = 0;

	genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
}

/**
 * \fn     sme_SetParam
 * \brief  Set parameters values
 *
 * Set parameters values
 *
 * \note   Note is indicated here
 * \param  hSme - handle to the SME object
 * \param  pParam - pointer to the param to set
 * \return PARAM_NOT_SUPPORTED for an unrecognized parameter, TI_OK if successfull.
 * \sa     sme_GetParam
 */
TI_STATUS sme_SetParam (TI_HANDLE hSme, paramInfo_t *pParam)
{
	TSme                *pSme = (TSme*)hSme;


	switch (pParam->paramType) {
	case SME_RADIO_ON_PARAM:
		/* if new value is different than current one */
		if (pSme->bRadioOn != pParam->content.smeRadioOn) {
			/* set new radio on value and send an event to the state-machine accordingly */
			pSme->bRadioOn = pParam->content.smeRadioOn;
			if (TI_TRUE == pSme->bRadioOn) {
				if(TI_TRUE == pSme->bRunning) {
					genSM_Event (pSme->hSmeSm, SME_SM_EVENT_START, hSme);
				}
			} else {
				genSM_Event (pSme->hSmeSm, SME_SM_EVENT_STOP, hSme);
			}
		}
		break;

	case SME_DESIRED_SSID_PARAM:
		/* if new value is different than current one */
		if ((pSme->tSsid.len != pParam->content.smeDesiredSSID.len) ||
		    (0 != os_memoryCompare (pSme->hOS, &(pSme->tSsid.str[ 0 ]),
		                            &(pParam->content.smeDesiredSSID.str[ 0 ]), pSme->tSsid.len))) {
			/* set new desired SSID */
			os_memoryCopy (pSme->hOS, &(pSme->tSsid.str[ 0 ]), &(pParam->content.smeDesiredSSID.str[ 0 ]), pParam->content.smeDesiredSSID.len);
			pSme->tSsid.len = pParam->content.smeDesiredSSID.len;

			pSme->uScanCount = 0;

			/* now send a disconnect event */
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		}
		break;

	case SME_DESIRED_SSID_ACT_PARAM:
		pSme->bRadioOn = TI_TRUE;

		/* if new value is different than current one */
		if ((pSme->tSsid.len != pParam->content.smeDesiredSSID.len) ||
		    (0 != os_memoryCompare (pSme->hOS, &(pSme->tSsid.str[ 0 ]),
		                            &(pParam->content.smeDesiredSSID.str[ 0 ]), pSme->tSsid.len))) {
			/* set new desired SSID */
			os_memoryCopy (pSme->hOS, &(pSme->tSsid.str[ 0 ]), &(pParam->content.smeDesiredSSID.str[ 0 ]), pParam->content.smeDesiredSSID.len);
			pSme->tSsid.len = pParam->content.smeDesiredSSID.len;
		}
		/* also set SSID type and connect required flag */
		if (OS_802_11_SSID_JUNK (pSme->tSsid.str, pSme->tSsid.len)) {
			pSme->eSsidType = SSID_TYPE_INVALID;
			pSme->bConnectRequired = TI_FALSE;
		} else if (0 == pSme->tSsid.len) {
			pSme->eSsidType = SSID_TYPE_ANY;
			pSme->bConnectRequired = TI_TRUE;
		} else {
			pSme->eSsidType = SSID_TYPE_SPECIFIC;
			pSme->bConnectRequired = TI_TRUE;
		}
		pSme->uScanCount = 0;

		/* if  junk SSID */
		if(TI_FALSE == pSme->bConnectRequired) {
			pSme->bConstantScan = TI_FALSE;
		}

		/* now send a disconnect event */
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		break;

	case SME_DESIRED_BSSID_PARAM:
		/* if new value is different than current one */
		if (TI_FALSE == MAC_EQUAL (pSme->tBssid, pParam->content.smeDesiredBSSID)) {
			/* set new BSSID */
			MAC_COPY (pSme->tBssid, pParam->content.smeDesiredBSSID);
			pSme->uScanCount = 0;
			/* now send a disconnect event */
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		}
		break;

	case SME_CONNECTION_MODE_PARAM:
		/* if new value is different than current one */
		if (pSme->eConnectMode != pParam->content.smeConnectionMode) {
			/* set new connection mode */
			pSme->eConnectMode = pParam->content.smeConnectionMode;
			pSme->uScanCount = 0;
			/* now send a disconnect event */
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		}
		break;

	case SME_DESIRED_BSS_TYPE_PARAM:
		/* if new value is different than current one */
		if (pSme->eBssType != pParam->content.smeDesiredBSSType) {
			/* set new BSS type */
			pSme->eBssType = pParam->content.smeDesiredBSSType;
			pSme->uScanCount = 0;
			/* now send a disconnect event */
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		}
		break;

	case SME_WSC_PB_MODE_PARAM:

		if (pParam->content.siteMgrWSCMode.WSCMode != TIWLN_SIMPLE_CONFIG_OFF) {
			pSme->bConstantScan = TI_TRUE;
			pSme->uScanCount = 0;
			/* now send a disconnect event */
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		} else {
			pSme->bConstantScan = TI_FALSE;
		}
		break;

	default:
		return PARAM_NOT_SUPPORTED;
		/* break;*/
	}

	return TI_OK;
}

/**
 * \fn     sme_GetParam
 * \brief  Retrieves a parameter from the SME
 *
 * Retrieves a parameter from the SME
 *
 * \param  hSme - handle to the SME object
 * \param  pParam - pointer to the param to retrieve
 * \return PARAM_NOT_SUPPORTED for an unrecognized parameter, TI_OK if successfull.
 * \sa     sme_SetParam
 */
TI_STATUS sme_GetParam (TI_HANDLE hSme, paramInfo_t *pParam)
{
	TSme                *pSme = (TSme*)hSme;


	switch (pParam->paramType) {
	case SME_RADIO_ON_PARAM:
		pParam->content.smeRadioOn = pSme->bRadioOn;
		break;

	case SME_DESIRED_SSID_ACT_PARAM:
		pParam->content.smeDesiredSSID.len = pSme->tSsid.len;
		os_memoryCopy (pSme->hOS, &(pParam->content.smeDesiredSSID.str[ 0 ]),
		               &(pSme->tSsid.str[ 0 ]), pSme->tSsid.len);
		break;

	case SME_DESIRED_BSSID_PARAM:
		MAC_COPY (pParam->content.smeDesiredBSSID, pSme->tBssid);
		break;

	case SME_CONNECTION_MODE_PARAM:
		pParam->content.smeConnectionMode = pSme->eConnectMode;
		break;

	case SME_DESIRED_BSS_TYPE_PARAM:
		pParam->content.smeDesiredBSSType = pSme->eBssType;
		break;

	case SME_CONNECTION_STATUS_PARAM:
		switch (genSM_GetCurrentState (pSme->hSmeSm)) {
		case SME_SM_STATE_IDLE:
			pParam->content.smeSmConnectionStatus = eDot11RadioDisabled;
			break;
		case SME_SM_STATE_WAIT_CONNECT:
			pParam->content.smeSmConnectionStatus = eDot11Disassociated;
			break;
		case SME_SM_STATE_SCANNING:
			pParam->content.smeSmConnectionStatus = eDot11Scaning;
			break;
		case SME_SM_STATE_CONNECTING:
			pParam->content.smeSmConnectionStatus = eDot11Connecting;
			break;
		case SME_SM_STATE_CONNECTED:
			pParam->content.smeSmConnectionStatus = eDot11Associated;
			break;
		case SME_SM_STATE_DISCONNECTING:
			pParam->content.smeSmConnectionStatus = eDot11Disassociated;
			break;
		}
		break;

	default:
		return PARAM_NOT_SUPPORTED;
		/* break;*/
	}

	return TI_OK;
}

/**
 * \fn     sme_ScanResultCB
 * \brief  Callback function from scan concentrator for results and scan complete indications
 *
 * Callback function from scan concentrator for results and scan complete indications
 *
 * \param  hSme - handle to the SME object
 * \param  eStatus - the reason for calling the CB
 * \param  pFrameInfo - frame information (if the CB is called due to received frame)
 * \param  uSPSStatus - SPS attened channels (if the CB is called to inidcate an SPS scan complete)
 * \return None
 */
void sme_ScanResultCB (TI_HANDLE hSme, EScanCncnResultStatus eStatus,
                       TScanFrameInfo* pFrameInfo, TI_UINT16 uSPSStatus)
{
	TSme                *pSme = (TSme*)hSme;
	paramInfo_t	        param;


	switch (eStatus) {
		/* a frame was received - update the scan result table */
	case SCAN_CRS_RECEIVED_FRAME:


		/*
		 * in auto mode in order to find country IE only !!!
		 * filter frames according to desired SSID, in case we are also trying to find
		 * country IE in passive scan, to avoid a table overflow (in manual mode, the SME table must be equal to the app
		 * table, the app is responsible to decide which SSIDs to use for scan)
		 */
		if (CONNECT_MODE_AUTO == pSme->eConnectMode) {
			if (SSID_TYPE_SPECIFIC == pSme->eSsidType) {
#ifndef XCC_MODULE_INCLUDED
				if ((pSme->tSsid.len == pFrameInfo->parsedIEs->content.iePacket.pSsid->hdr[ 1 ]) &&
				    (0 == os_memoryCompare (pSme->hOS, &(pSme->tSsid.str[ 0 ]),
				                            &(pFrameInfo->parsedIEs->content.iePacket.pSsid->serviceSetId[ 0 ]),
				                            pSme->tSsid.len)))
#endif
				{}

			}
		}
		break;

		/* scan was completed successfully */
	case SCAN_CRS_SCAN_COMPLETE_OK:
		/* an error occured, try selecting a site anyway */
	case SCAN_CRS_SCAN_ABORTED_FW_RESET:
	case SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY:
	case SCAN_CRS_SCAN_FAILED:
	case SCAN_CRS_TSF_ERROR:

		/* stablizie the scan result table - delete its contenst if no results were recived during last scan */
		scanResultTable_SetStableState (pSme->hScanResultTable);

		if (CONNECT_MODE_AUTO == pSme->eConnectMode) {

			/* try to select a site */
			pSme->pCandidate = sme_Select (hSme);

			/* if no matching site was found */
			if (NULL == pSme->pCandidate) {
				/* for IBSS or any, if no entries where found, add the self site */
				if (pSme->eBssType == BSS_INFRASTRUCTURE) {

					genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);
					break;
				}

				{
					TI_UINT8     uDesiredChannel;

					param.paramType = SITE_MGR_DESIRED_CHANNEL_PARAM;
					siteMgr_getParam(pSme->hSiteMgr, &param);
					uDesiredChannel = param.content.siteMgrDesiredChannel;

					if (uDesiredChannel >= SITE_MGR_CHANNEL_A_MIN) {
						param.content.channelCapabilityReq.band = RADIO_BAND_5_0_GHZ;
					} else {
						param.content.channelCapabilityReq.band = RADIO_BAND_2_4_GHZ;
					}

					/*
					update the regulatory domain with the selected band
					*/
					/* Check if the selected channel is valid according to regDomain */
					param.paramType = REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES;
					param.content.channelCapabilityReq.scanOption = ACTIVE_SCANNING;
					param.content.channelCapabilityReq.channelNum = uDesiredChannel;

					regulatoryDomain_getParam (pSme->hRegDomain,&param);
					if (!param.content.channelCapabilityRet.channelValidity) {

						genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);

						break;
					}

					pSme->pCandidate = (TSiteEntry *)addSelfSite(pSme->hSiteMgr);

					if (pSme->pCandidate == NULL) {

						genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);

						break;
					}

				}
			}

			/* a connection candidate is available, send a connect event */
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT, hSme);
		}
		break;

		/*
		 * scan was stopped according to SME request (should happen when moving to disconnecting from scanning), send a
		 * connect failure event to move out of disconnecting
		 */
	case SCAN_CRS_SCAN_STOPPED:
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);
		break;

	default:
		break;
	}
}

/**
 * \fn     sme_AppScanResult
 * \brief  Callback function from scan concentrator app for results and scan complete indications
 *
 * Callback function from scan concentrator app for results and scan complete indications, used
 * for scans wehen the SME is in manual.
 *
 * \param  hSme - handle to the SME object
 * \param  eStatus - the reason for calling the CB
 * \param  pFrameInfo - frame information (if the CB is called due to received frame)
 * \param  uResultCount - number of results rceived
 * \return None
 */
void sme_AppScanResult (TI_HANDLE hSme, EScanCncnResultStatus eStatus,
                        TScanFrameInfo* pFrameInfo)
{
	TSme                *pSme = (TSme*)hSme;

	/* in manual mode, store the frame in the SME scan result table */
	if (CONNECT_MODE_MANUAL == pSme->eConnectMode) {
		switch (eStatus) {
			/* a frame was received - update the scan result table */
		case SCAN_CRS_RECEIVED_FRAME:

			break;

			/* scan was completed successfully */
		case SCAN_CRS_SCAN_COMPLETE_OK:
			/* an error occured, try selecting a site anyway */
		case SCAN_CRS_SCAN_ABORTED_FW_RESET:
		case SCAN_CRS_SCAN_STOPPED:
		case SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY:
		case SCAN_CRS_SCAN_FAILED:
		case SCAN_CRS_TSF_ERROR:

			/* stablizie the scan result table - delete its contenst if no results were recived during last scan */
			scanResultTable_SetStableState (pSme->hScanResultTable);
			break;

		default:
			break;
		}
	}
}


/**
 * \fn     sme_MeansurementScanResult
 * \brief  Callback function from Meansurement for results
 *
 * Callback function from Meansurement for results used for scans wehen the SME is in Meansurement.
 *
 * \param  hSme - handle to the SME object
 * \param  pFrameInfo - frame information (if the CB is called due to received frame)
 * \return None
 */
void sme_MeansurementScanResult (TI_HANDLE hSme, EScanCncnResultStatus eStatus, TScanFrameInfo* pFrameInfo)
{
	TSme                *pSme = (TSme*)hSme;

	switch (eStatus) {
		/* a frame was received - update the scan result table */
	case SCAN_CRS_RECEIVED_FRAME:

		break;

		/* scan was completed successfully */
	case SCAN_CRS_SCAN_COMPLETE_OK:
		/* an error occured, try selecting a site anyway */
	case SCAN_CRS_SCAN_ABORTED_FW_RESET:
	case SCAN_CRS_SCAN_STOPPED:
	case SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY:
	case SCAN_CRS_SCAN_FAILED:
	case SCAN_CRS_TSF_ERROR:

		/* stablizie the scan result table - delete its contenst if no results were recived during last scan */
		scanResultTable_SetStableState (pSme->hScanResultTable);
		break;

	default:
		break;
	}

}


/**
 * \fn     Function declaration
 * \brief  Function brief description goes here
 *
 * Function detailed description goes here
 *
 * \note   Note is indicated here
 * \param  Parameter name - parameter description
 * \param
 * \return Return code is detailed here
 * \sa     Reference to other relevant functions
 */
void sme_ReportConnStatus (TI_HANDLE hSme, mgmtStatus_e eStatusType, TI_UINT32 uStatusCode)
{
	TSme                *pSme = (TSme*)hSme;


	/* Act according to status */
	switch (eStatusType) {
		/* connection was successful */
	case STATUS_SUCCESSFUL:
		pSme->bAuthSent = TI_TRUE;
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_SUCCESS, hSme);
		break;

	case STATUS_ASSOC_REJECT:
	case STATUS_SECURITY_FAILURE:
	case STATUS_AP_DEAUTHENTICATE:
	case STATUS_AP_DISASSOCIATE:
	case STATUS_ROAMING_TRIGGER:
	case STATUS_AUTH_REJECT:
		/* Indicate the authentication and/or association was sent to the AP */
		pSme->bAuthSent = TI_TRUE;

		/* keep the disassociation status and code, for sending event to user-mode */
		pSme->tDisAssoc.eMgmtStatus = eStatusType;
		pSme->tDisAssoc.uStatusCode = uStatusCode;

		/* try to find the next connection candidate */
		pSme->pCandidate = sme_Select (hSme);
		/* if the next connection candidate exists */
		if (NULL != pSme->pCandidate) {
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT, hSme);
		} else {
			genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);
		}
		break;

		/* Note that in case of unspecified status we won't update the status. This is done since this function could be called twice */
		/* for example: apConn called this function and than SME called conn_stop and this function is called again                   */
		/* we use this status at SME, if != 0 means that assoc frame sent */
	case STATUS_UNSPECIFIED:
		pSme->bAuthSent = TI_TRUE;
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);
		break;

	default:
		break;
	}
}

/**
 * \fn     sme_ReportApConnStatus
 * \brief  Used by AP connection (and Soft-gemini) modules to report connection status
 *
 * Used by AP connection (and Soft-gemini) modules to report connection status
 *
 * \param  hSme - handle to the SME object
 * \param  eStatusType - connection status
 * \param  uStatus code - extended status information (if available)
 * \return None
 * \sa     sme_ReportConnStatus
 */
void sme_ReportApConnStatus (TI_HANDLE hSme, mgmtStatus_e eStatusType, TI_UINT32 uStatusCode)
{
	TSme                *pSme = (TSme*)hSme;


	/* Act according to status */
	switch (eStatusType) {

		/* SG re-select */
	case STATUS_SG_RESELECT:
		pSme->bReselect = TI_TRUE;
		pSme->bConnectRequired = TI_TRUE;
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);
		break;

		/* shouldn't happen (not from AP conn) */
	case STATUS_SUCCESSFUL:
		break;

	case STATUS_UNSPECIFIED:
	case STATUS_AUTH_REJECT:
	case STATUS_ASSOC_REJECT:
	case STATUS_SECURITY_FAILURE:
	case STATUS_AP_DEAUTHENTICATE:
	case STATUS_AP_DISASSOCIATE:
	case STATUS_ROAMING_TRIGGER:

		/* keep the disassociation status and code, for sending event to user-mode */
		pSme->tDisAssoc.eMgmtStatus = eStatusType;
		pSme->tDisAssoc.uStatusCode = uStatusCode;
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_CONNECT_FAILURE, hSme);
		break;

	case STATUS_DISCONNECT_DURING_CONNECT:
		genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
		break;

	default:{}

	}
}

/**
 * \fn     sme_GetSmeScanResultTableHandler
 * \brief  get the handler to the Sme Scan Result Table.
 *
 * \param  hSme - handle to the SME object
 * \param  uStatus code - extended status information (if available)
 * \return None
 * \sa     sme_ReportConnStatus
 */
void sme_GetSmeScanResultTableHandler (TI_HANDLE hSme, TI_HANDLE *hScanResultTable)
{
	TSme                *pSme = (TSme*)hSme;

	*hScanResultTable = pSme->hScanResultTable;
}

/**
 * \fn     SME_ConnectRequired
 * \brief  start connection sequence by set the flag ConnectRequired and issue DISCONNECT event.
 *         called by CommandDispatcher in OSE OS.
 *
 * \param  hSme - handle to the SME object
 * \return None
 * \sa     SME_Disconnect
 */
void SME_ConnectRequired (TI_HANDLE hSme)
{
	TSme *pSme = (TSme*)hSme;

	pSme->bRadioOn = TI_TRUE;
	pSme->uScanCount = 0;
	pSme->bConnectRequired = TI_TRUE;

	/* now send a disconnect event */
	genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
}

/**
 * \fn     SME_Disconnect
 * \brief  perform disconnect by clear the flag ConnectRequired and issue DISCONNECT event.
 *
 * \param  hSme - handle to the SME object
 * \return None
 * \sa     SME_ConnectRequired
 */
void SME_Disconnect (TI_HANDLE hSme)
{
	TSme *pSme = (TSme*)hSme;

	pSme->bConnectRequired = TI_FALSE;
	/* turn off WSC PB mode */
	pSme->bConstantScan = TI_FALSE;

	/* now send a disconnect event */
	genSM_Event (pSme->hSmeSm, SME_SM_EVENT_DISCONNECT, hSme);
}

