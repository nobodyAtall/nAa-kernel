/*
 * measurementMgr.c
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


/***************************************************************************/
/*                                                                         */
/*    MODULE:   measurementMgr.c                                           */
/*    PURPOSE:  measurement Manager module file                            */
/*                                                                         */
/***************************************************************************/




#define __FILE_ID__  FILE_ID_1
#include "measurementMgr.h"
#include "regulatoryDomainApi.h"
#include "healthMonitor.h"
#include "DrvMainModules.h"
#include "siteMgrApi.h"
#include "TrafficMonitorAPI.h"
#include "smeApi.h"
#include "sme.h"
#ifdef XCC_MODULE_INCLUDED
#include "XCCTSMngr.h"
#endif
#include "TWDriver.h"

/* default measurement parameters */
#define MEASUREMENT_CAPABILITIES_NONE                   0x00
#define MEASUREMENT_CAPABILITIES_DOT11H                 0x01
#define MEASUREMENT_CAPABILITIES_XCC_RM                 0x02


#define MEASUREMENT_BEACON_INTERVAL_IN_MICRO_SEC        1024
#define MEASUREMENT_MSEC_IN_MICRO                       1000




/********************************************************************************/
/*                      Internal functions prototypes.                          */
/********************************************************************************/

static void measurementMgr_releaseModule(measurementMgr_t *pMeasurementMgr);

static TI_BOOL measurementMgr_isTrafficIntensityHigherThanThreshold(measurementMgr_t * pMeasurementMgr);

static TI_BOOL  measurementMgr_isRequestValid(TI_HANDLE hMeasurementMgr, MeasurementRequest_t *pRequestArr[], TI_UINT8 numOfRequest);

static TI_BOOL measurementMgrSM_measureInProgress(TI_HANDLE hMeasurementMgr);





/********************************************************************************/
/*                      Interface functions Implementation.                     */
/********************************************************************************/


/**
 * Creates the Measurement Manager moodule.
 *
 * @param hOs A handle to the OS object.
 *
 * @date 16-Dec-2005
 */
TI_HANDLE measurementMgr_create(TI_HANDLE hOs)
{
	measurementMgr_t * pMeasurementMgr = NULL;
	TI_STATUS status;

	/* allocating the MeasurementMgr object */
	pMeasurementMgr = os_memoryAlloc(hOs, sizeof(measurementMgr_t));

	if (pMeasurementMgr == NULL)
		return NULL;

	os_memoryZero(hOs, pMeasurementMgr, sizeof(measurementMgr_t));
	pMeasurementMgr->hOs = hOs;

	/* creating the Measurement SM */
	status = fsm_Create(pMeasurementMgr->hOs, &(pMeasurementMgr->pMeasurementMgrSm),
	                    MEASUREMENTMGR_NUM_STATES , MEASUREMENTMGR_NUM_EVENTS);
	if (status != TI_OK) {
		measurementMgr_releaseModule(pMeasurementMgr);
		return NULL;
	}

	/* creating the sub modules of measurement module */

	/* creating Request Handler sub module */
	if ( (pMeasurementMgr->hRequestH = requestHandler_create(hOs)) == NULL) {
		measurementMgr_releaseModule(pMeasurementMgr);
		return NULL;
	}

	return(pMeasurementMgr);
}





/**
 * Configures the Measurement Manager module.
 *
 * @param pStadHandles Handles to other modules the Measurement Manager needs.
 *
 * @date 16-Dec-2005
 */
void measurementMgr_init (TStadHandlesList *pStadHandles)
{
	measurementMgr_t *pMeasurementMgr = (measurementMgr_t *)(pStadHandles->hMeasurementMgr);
	paramInfo_t param;

	/* Init Handlers */
	pMeasurementMgr->hRegulatoryDomain  = pStadHandles->hRegulatoryDomain;
	pMeasurementMgr->hXCCMngr           = pStadHandles->hXCCMngr;
	pMeasurementMgr->hSiteMgr           = pStadHandles->hSiteMgr;
	pMeasurementMgr->hTWD               = pStadHandles->hTWD;
	pMeasurementMgr->hMlme              = pStadHandles->hMlmeSm;
	pMeasurementMgr->hTrafficMonitor    = pStadHandles->hTrafficMon;
	pMeasurementMgr->hReport            = pStadHandles->hReport;
	pMeasurementMgr->hOs                = pStadHandles->hOs;
	pMeasurementMgr->hScr               = pStadHandles->hSCR;
	pMeasurementMgr->hApConn            = pStadHandles->hAPConnection;
	pMeasurementMgr->hTxCtrl            = pStadHandles->hTxCtrl;
	pMeasurementMgr->hTimer             = pStadHandles->hTimer;
	pMeasurementMgr->hSme               = pStadHandles->hSme;

	/* initialize variables to default values */
	pMeasurementMgr->Enabled = TI_TRUE;
	pMeasurementMgr->Connected = TI_FALSE;
	pMeasurementMgr->Capabilities = MEASUREMENT_CAPABILITIES_NONE;
	pMeasurementMgr->Mode = MSR_MODE_NONE;

	/* Getting management capability status */
	param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
	regulatoryDomain_getParam (pMeasurementMgr->hRegulatoryDomain, &param);
	if (param.content.spectrumManagementEnabled) {
		pMeasurementMgr->Capabilities |= MEASUREMENT_CAPABILITIES_DOT11H;
	}

	/* Init Functions */
	pMeasurementMgr->parserFrameReq = NULL;
	pMeasurementMgr->isTypeValid = NULL;
	pMeasurementMgr->buildReport = NULL;
	pMeasurementMgr->buildRejectReport = NULL;
	pMeasurementMgr->sendReportAndCleanObj = NULL;

	/* initialize variables */
	pMeasurementMgr->currentState = MEASUREMENTMGR_STATE_IDLE;
	pMeasurementMgr->isModuleRegistered = TI_FALSE;
	pMeasurementMgr->currentFrameType = MSR_FRAME_TYPE_NO_ACTIVE;
	pMeasurementMgr->measuredChannelID = 0;
	pMeasurementMgr->currentNumOfRequestsInParallel = 0;
	pMeasurementMgr->bMeasurementScanExecuted = TI_FALSE;

	/* config sub modules */
	RequestHandler_config(pMeasurementMgr->hRequestH, pStadHandles->hReport, pStadHandles->hOs);

	/* Register to the SCR module */
	scr_registerClientCB(pMeasurementMgr->hScr, SCR_CID_XCC_MEASURE, measurementMgr_scrResponseCB, (TI_HANDLE)pMeasurementMgr);

	measurementMgrSM_config ((TI_HANDLE)pMeasurementMgr);

}


TI_STATUS measurementMgr_SetDefaults (TI_HANDLE hMeasurementMgr, measurementInitParams_t * pMeasurementInitParams)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
#ifdef XCC_MODULE_INCLUDED
	TI_UINT32 currAC;
#endif

	pMeasurementMgr->trafficIntensityThreshold = pMeasurementInitParams->trafficIntensityThreshold;
	pMeasurementMgr->maxDurationOnNonServingChannel = pMeasurementInitParams->maxDurationOnNonServingChannel;

	/* allocating the measurement Activation Delay timer */
	pMeasurementMgr->hActivationDelayTimer = tmr_CreateTimer (pMeasurementMgr->hTimer);
	if (pMeasurementMgr->hActivationDelayTimer == NULL) {
		return TI_NOK;
	}

#ifdef XCC_MODULE_INCLUDED
	/* allocating the per AC TS Metrics report timers */
	for (currAC = 0; currAC < MAX_NUM_OF_AC; currAC++) {
		pMeasurementMgr->isTsMetricsEnabled[currAC] = TI_FALSE;

		pMeasurementMgr->hTsMetricsReportTimer[currAC] = tmr_CreateTimer (pMeasurementMgr->hTimer);
		if (pMeasurementMgr->hTsMetricsReportTimer[currAC] == NULL) {
			return TI_NOK;
		}
	}

	/* Check in the Registry if the station supports XCC RM */
	if (pMeasurementInitParams->XCCEnabled == XCC_MODE_ENABLED) {
		pMeasurementMgr->Capabilities |= MEASUREMENT_CAPABILITIES_XCC_RM;
	}
#endif

	return TI_OK;
}





/**
 * Sets the specified Measurement Manager parameter.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param pParam The parameter to set.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_setParam(TI_HANDLE hMeasurementMgr, paramInfo_t * pParam)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	switch (pParam->paramType) {
	case MEASUREMENT_ENABLE_DISABLE_PARAM: {

		if (pParam->content.measurementEnableDisableStatus) {
			measurementMgr_enable(pMeasurementMgr);
		} else {
			measurementMgr_disable(pMeasurementMgr);
		}

		break;
	}

	case MEASUREMENT_TRAFFIC_THRESHOLD_PARAM: {
		if ((pParam->content.measurementTrafficThreshold >= MEASUREMENT_TRAFFIC_THRSHLD_MIN) &&
		        (pParam->content.measurementTrafficThreshold <= MEASUREMENT_TRAFFIC_THRSHLD_MAX)) {

			pMeasurementMgr->trafficIntensityThreshold = pParam->content.measurementTrafficThreshold;
		}

		break;
	}


	case MEASUREMENT_MAX_DURATION_PARAM: {

		pMeasurementMgr->maxDurationOnNonServingChannel = pParam->content.measurementMaxDuration;

		break;
	}


	default: {

		return PARAM_NOT_SUPPORTED;
	}

	}

	return TI_OK;
}





/**
 * Gets the specified parameter from the Measurement Manager.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param pParam The parameter to get.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_getParam(TI_HANDLE hMeasurementMgr, paramInfo_t * pParam)
{

	switch (pParam->paramType) {

	case MEASUREMENT_GET_STATUS_PARAM: {
		WLAN_OS_REPORT(("%s: \n\n", __FUNCTION__));
		WLAN_OS_REPORT(("MeasurementMgr Status Report:\n\n"));

		WLAN_OS_REPORT(("Current State: %d\n\n", pMeasurementMgr->currentState));

		WLAN_OS_REPORT(("Connected: %d\n", pMeasurementMgr->Connected));
		WLAN_OS_REPORT(("Enabled: %d\n\n", pMeasurementMgr->Enabled));

		WLAN_OS_REPORT(("Mode: %d\n", pMeasurementMgr->Mode));
		WLAN_OS_REPORT(("Capabilities: %d\n\n", pMeasurementMgr->Capabilities));

		WLAN_OS_REPORT(("current Frame Type: %d\n", pMeasurementMgr->currentFrameType));
		WLAN_OS_REPORT(("Measured Channel: %d\n", pMeasurementMgr->measuredChannelID));
		WLAN_OS_REPORT(("Serving Channel: %d\n", pMeasurementMgr->servingChannelID));
		WLAN_OS_REPORT(("Traffic Intensity Threshold: %d\n", pMeasurementMgr->trafficIntensityThreshold));
		WLAN_OS_REPORT(("Max Duration on Nonserving Channel: %d\n", pMeasurementMgr->maxDurationOnNonServingChannel));

		break;
	}


	default: {

		return PARAM_NOT_SUPPORTED;
	}

	}

	return TI_OK;
}






/**
 * Signals the Measurement Manager that the STA is connected.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_connected(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	/* checking if measurement is enabled */
	if (pMeasurementMgr->Mode == MSR_MODE_NONE)
		return TI_OK;


	return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                              MEASUREMENTMGR_EVENT_CONNECTED, pMeasurementMgr);
}





/**
 * Signals the Measurement Manager that the STA is disconnected.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_disconnected(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;


	return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                              MEASUREMENTMGR_EVENT_DISCONNECTED, pMeasurementMgr);
}




/**
 * Enables the Measurement Manager module.
 *
 * @date 10-Jan-2006
 */
TI_STATUS measurementMgr_enable(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;


	return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                              MEASUREMENTMGR_EVENT_ENABLE, pMeasurementMgr);
}





/**
 * Disables the Measurement Manager module.
 *
 * @date 10-Jan-2006
 */
TI_STATUS measurementMgr_disable(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;


	return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                              MEASUREMENTMGR_EVENT_DISABLE, pMeasurementMgr);
}





/**
 * Destroys the Measurement Manager module.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_destroy(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t *pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	if (pMeasurementMgr == NULL)
		return TI_OK;


	measurementMgr_releaseModule (pMeasurementMgr);

	return TI_OK;
}






/**
 * Sets the Measurement Mode.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param capabilities The AP capabilities.
 * @param pIeBuffer Pointer to the list of IEs.
 * @param length Length of the IE list.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_setMeasurementMode(TI_HANDLE hMeasurementMgr, TI_UINT16 capabilities,
        TI_UINT8 * pIeBuffer, TI_UINT16 length)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	/*
	 * 11h Measurement is not supported in the current version.
	 */
	/*  if( (pMeasurementMgr->Capabilities & MEASUREMENT_CAPABILITIES_DOT11H) &&
	        (capabilities & DOT11_SPECTRUM_MANAGEMENT) )
	    {
	        pMeasurementMgr->Mode = MSR_MODE_SPECTRUM_MANAGEMENT;
	    }
	    else
	    {
	*/
#ifdef XCC_MODULE_INCLUDED

	if (pMeasurementMgr->Capabilities & MEASUREMENT_CAPABILITIES_XCC_RM) {
		pMeasurementMgr->Mode = MSR_MODE_XCC;
	} else
#endif
	{
		pMeasurementMgr->Mode = MSR_MODE_NONE;
	}



	return TI_OK;
}






/**
 * Called when a frame with type measurement request is received.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param frameType The frame type.
 * @param dataLen The length of the frame.
 * @param pData A pointer to the frame's content.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_receiveFrameRequest(TI_HANDLE hMeasurementMgr,
        EMeasurementFrameType frameType,
        TI_INT32 dataLen,
        TI_UINT8 * pData)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	TMeasurementFrameRequest * frame = &(pMeasurementMgr->newFrameRequest);
	TI_UINT16 currentFrameToken;

	/* checking if measurement is enabled */
	if (pMeasurementMgr->Mode == MSR_MODE_NONE)
		return TI_NOK;

	/* ignore broadcast/multicast request if unicast request is active */
	if (frameType != MSR_FRAME_TYPE_UNICAST && pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST) {

		return TI_NOK;
	}

	/* ignore broadcast request if multicast request is active */
	if (frameType == MSR_FRAME_TYPE_BROADCAST && pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_MULTICAST) {

		return TI_NOK;
	}


	/* Parsing the Frame Request Header */
	pMeasurementMgr->parserFrameReq(hMeasurementMgr, pData, dataLen,
	                                frame);

	frame->frameType = frameType;

	/* checking if the received token frame is the same as the one that is being processed */
	if ((requestHandler_getFrameToken(pMeasurementMgr->hRequestH, &currentFrameToken) == TI_OK)
	        && (currentFrameToken == frame->hdr->dialogToken)) {
		os_memoryZero(pMeasurementMgr->hOs, &pMeasurementMgr->newFrameRequest,
		              sizeof(TMeasurementFrameRequest));


		return TI_NOK;
	}


	/* Frame is Received for processing */
	return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                              MEASUREMENTMGR_EVENT_FRAME_RECV, pMeasurementMgr);
}





/**
 * Activates the next measurement request.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 *
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_activateNextRequest(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	requestHandler_t * pRequestH = (requestHandler_t *) pMeasurementMgr->hRequestH;
	MeasurementRequest_t * pRequestArr[MAX_NUM_REQ];
	TI_UINT8 numOfRequestsInParallel = 0;
	TI_BOOL valid;
	TI_UINT8 index;

	/* Keep note of the time we started processing the request. this will be used */
	/* to give the measurementSRV a time frame to perform the measurement operation */
	pMeasurementMgr->currentRequestStartTime = os_timeStampMs(pMeasurementMgr->hOs);

	do {
		TI_STATUS status;

		pRequestH->activeRequestID += numOfRequestsInParallel;
		pRequestH->numOfWaitingRequests -= numOfRequestsInParallel;

		for (index = 0; index < MAX_NUM_REQ; index++) {
			pRequestArr[index] = NULL;
		}
		numOfRequestsInParallel = 0;

		/* Getting the next request/requests from the request handler */
		status = requestHandler_getNextReq(pMeasurementMgr->hRequestH, TI_FALSE, pRequestArr,
		                                   &numOfRequestsInParallel);

		/* Checking if there are no waiting requests */
		if (status != TI_OK) {

			return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
			                              MEASUREMENTMGR_EVENT_SEND_REPORT, pMeasurementMgr);
		}

		/* Checking validity of request/s */
		valid = measurementMgr_isRequestValid(pMeasurementMgr, pRequestArr,
		                                      numOfRequestsInParallel);

		/* Checking if the current request is Beacon Table */
		if ( (numOfRequestsInParallel == 1) &&
		        (pRequestArr[0]->Type == MSR_TYPE_BEACON_MEASUREMENT) &&
		        (pRequestArr[0]->ScanMode == MSR_SCAN_MODE_BEACON_TABLE) ) {

			pMeasurementMgr->buildReport(hMeasurementMgr, *(pRequestArr[0]), NULL);
			valid = TI_FALSE; /* In order to get the next request/s*/
		}

	} while (valid == TI_FALSE);




	/* Ignore requests if traffic intensity is high */
	if (measurementMgr_isTrafficIntensityHigherThanThreshold(pMeasurementMgr) == TI_TRUE) {

		measurementMgr_rejectPendingRequests(pMeasurementMgr, MSR_REJECT_TRAFFIC_INTENSITY_TOO_HIGH);

		return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
		                              MEASUREMENTMGR_EVENT_SEND_REPORT, pMeasurementMgr);
	}


	pMeasurementMgr->measuredChannelID = pRequestArr[0]->channelNumber;

	/* Request resource from the SCR */
	return measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                              MEASUREMENTMGR_EVENT_REQUEST_SCR, pMeasurementMgr);
}



void measurementMgr_rejectPendingRequests(TI_HANDLE hMeasurementMgr, EMeasurementRejectReason rejectReason)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	requestHandler_t * pRequestH = (requestHandler_t *) pMeasurementMgr->hRequestH;
	MeasurementRequest_t * pRequestArr[MAX_NUM_REQ];
	TI_UINT8 numOfRequestsInParallel;

	/* reject all pending measurement requests */
	while (requestHandler_getNextReq(pMeasurementMgr->hRequestH, TI_TRUE,
	                                 pRequestArr, &numOfRequestsInParallel) == TI_OK) {

		pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr,
		                                   numOfRequestsInParallel, rejectReason);

		pRequestH->activeRequestID += numOfRequestsInParallel;
	}
}





/********************************************************************************/
/*                      Callback functions Implementation.                      */
/********************************************************************************/


/**
 * The callback called by the MeasurementSRV module when then
 * measurement operation has ended.
 *
 * @param clientObj A handle to the Measurement Manager module.
 * @param msrReply An array of replies sent by the MeasurementSRV module,
 * where each reply contains the result of a single measurement request.
 *
 * @date 01-Jan-2006
 */
void measurementMgr_MeasurementCompleteCB(TI_HANDLE clientObj, TMeasurementReply * msrReply)
{
	measurementMgr_t    *pMeasurementMgr = (measurementMgr_t *) clientObj;
	TI_UINT8            index;


	/* build a report for each measurement request/reply pair */
	for (index = 0; index < msrReply->numberOfTypes; index++) {
		pMeasurementMgr->buildReport(pMeasurementMgr, *(pMeasurementMgr->currentRequest[index]), &msrReply->msrTypes[index]);
	}

	measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                       MEASUREMENTMGR_EVENT_COMPLETE, pMeasurementMgr);
}


/**
 * The callback called when the SCR responds to the SCR request.
 *
 * @param hClient A handle to the Measurement Manager module.
 * @param requestStatus The request's status
 * @param eResource The resource for which the CB is issued
 * @param pendReason The reason of a PEND status.
 *
 * @date 01-Jan-2006
 */
void measurementMgr_scrResponseCB(TI_HANDLE hClient, EScrClientRequestStatus requestStatus,
                                  EScrResourceId eResource, EScePendReason pendReason )
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hClient;
	measurementMgrSM_Events event;


	/* If the SM is in a state where it waits for the CB, status of RUN */
	/* results in the SM asking the measurementSRV to start measurement; */
	/* otherwise we got an ABORT or a PEND reason worse than the one we */
	/* got when calling the SCR, so the SM aborts the measurement */
	if (pMeasurementMgr->currentState == MEASUREMENTMGR_STATE_WAITING_FOR_SCR) {
		if (requestStatus == SCR_CRS_RUN) {

			event = MEASUREMENTMGR_EVENT_SCR_RUN;
		} else {
			event = MEASUREMENTMGR_EVENT_ABORT;
		}
	} else {
		/* This can only occur if FW reset occurs or when higher priority client is running. */

		if (requestStatus == SCR_CRS_FW_RESET) {

			event = MEASUREMENTMGR_EVENT_FW_RESET;
		} else {

			event = MEASUREMENTMGR_EVENT_ABORT;
		}
	}

	measurementMgrSM_event((TI_UINT8 *) &(pMeasurementMgr->currentState),
	                       event, pMeasurementMgr);
}






/**
 * The callback called by the MLME.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 *
 * @date 01-Jan-2006
 */
void measurementMgr_mlmeResultCB(TI_HANDLE hMeasurementMgr, TMacAddr * bssid, mlmeFrameInfo_t * frameInfo,
                                 TRxAttr * pRxAttr, TI_UINT8 * buffer, TI_UINT16 bufferLength)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	TScanFrameInfo      tScanFrameInfo;

	if (measurementMgrSM_measureInProgress(pMeasurementMgr) == TI_FALSE) {
		return;
	}


	/* erroneous frames are notifed to the measurmenet manager to update counter
	(add counter sometimes in the future) Look at: scanCncn_ScanCompleteNotificationCB and
	scanCncn_MlmeResultCB */
	if (NULL == bssid) {
		return;
	}

	if (pMeasurementMgr == NULL || pRxAttr == NULL) {
		return;
	}


	/* build the scan frame info object */
	tScanFrameInfo.bssId = bssid;
	tScanFrameInfo.band = (ERadioBand)pRxAttr->band;
	tScanFrameInfo.channel = pRxAttr->channel;
	tScanFrameInfo.parsedIEs = frameInfo;
	tScanFrameInfo.rate = pRxAttr->Rate;
	tScanFrameInfo.rssi = pRxAttr->Rssi;
	tScanFrameInfo.snr = pRxAttr->SNR;
	tScanFrameInfo.staTSF = pRxAttr->TimeStamp;
	tScanFrameInfo.buffer = buffer;
	tScanFrameInfo.bufferLength = bufferLength;

	/* update the driver (SME) result table */
	sme_MeansurementScanResult (pMeasurementMgr->hSme, SCAN_CRS_RECEIVED_FRAME, &tScanFrameInfo);

}


/********************************************************************************/
/*                      Internal functions Implementation.                      */
/********************************************************************************/


/**
 * Releases the module's allocated objects according to the given init vector.
 *
 * @param pMeasurementMgr A handle to the Measurement Manager module.
 * @param initVec The init vector with a bit set for each allocated object.
 *
 * @date 01-Jan-2006
 */
static void measurementMgr_releaseModule (measurementMgr_t * pMeasurementMgr)
{
#ifdef XCC_MODULE_INCLUDED
	TI_UINT32 currAC;
#endif

	if (pMeasurementMgr->hActivationDelayTimer) {
		tmr_DestroyTimer (pMeasurementMgr->hActivationDelayTimer);
	}

#ifdef XCC_MODULE_INCLUDED
	for (currAC = 0; currAC < MAX_NUM_OF_AC; currAC++) {
		if (pMeasurementMgr->hTsMetricsReportTimer[currAC]) {
			tmr_DestroyTimer (pMeasurementMgr->hTsMetricsReportTimer[currAC]);
		}
	}
#endif

	if (pMeasurementMgr->pMeasurementMgrSm) {
		fsm_Unload(pMeasurementMgr->hOs, pMeasurementMgr->pMeasurementMgrSm);
	}

	if (pMeasurementMgr->hRequestH) {
		requestHandler_destroy(pMeasurementMgr->hRequestH);
	}

	os_memoryFree(pMeasurementMgr->hOs, pMeasurementMgr, sizeof(measurementMgr_t));
}



/**
 * Checks whether the traffic intensity, i.e. number of packets per seconds, is higher
 * than the preconfigured threshold.
 *
 * @param pMeasurementMgr A handle to the Measurement Manager module.
 *
 * @return True iff the traffic intensity is high
 *
 * @date 01-Jan-2006
 */
static TI_BOOL measurementMgr_isTrafficIntensityHigherThanThreshold(measurementMgr_t * pMeasurementMgr)
{
	TI_BOOL trafficIntensityHigh = TI_FALSE;
	int pcksPerSec;

	pcksPerSec = TrafficMonitor_GetFrameBandwidth(pMeasurementMgr->hTrafficMonitor);


	if (pcksPerSec >= pMeasurementMgr->trafficIntensityThreshold)
		trafficIntensityHigh = TI_TRUE;

	return trafficIntensityHigh;
}




/**
 * Checks whether the given measurement request is valid.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param pRequestArr The measurement request.
 * @param numOfRequest Number of type requests
 *
 * @return True iff the request is valid
 *
 * @date 01-Jan-2006
 */
static TI_BOOL  measurementMgr_isRequestValid(TI_HANDLE hMeasurementMgr, MeasurementRequest_t *pRequestArr[],
        TI_UINT8 numOfRequest)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	TI_UINT8 requestIndex;
	paramInfo_t param;

	/* Checking validity of the measured channel number */
	param.content.channel = pRequestArr[0]->channelNumber;
	param.paramType = REGULATORY_DOMAIN_IS_CHANNEL_SUPPORTED;
	regulatoryDomain_getParam(pMeasurementMgr->hRegulatoryDomain, &param);
	if ( !param.content.bIsChannelSupprted  ) {

		if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
			pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest,
			                                   MSR_REJECT_INVALID_CHANNEL);

		return TI_FALSE;
	}


	/* Check Validity of each request */
	for (requestIndex = 0; requestIndex < numOfRequest; requestIndex++) {

		/* Checking validity of the Request Type */
		if (pMeasurementMgr->isTypeValid(hMeasurementMgr, pRequestArr[requestIndex]->Type,
		                                 pRequestArr[requestIndex]->ScanMode) == TI_FALSE) {

			if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
				pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest,
				                                   MSR_REJECT_INVALID_MEASUREMENT_TYPE);

			return TI_FALSE;
		}

		/* For measurement types different than Beacon Table */
		if ((pRequestArr[requestIndex]->Type != MSR_TYPE_BEACON_MEASUREMENT) ||
		        (pRequestArr[requestIndex]->ScanMode != MSR_SCAN_MODE_BEACON_TABLE)) {
			/* Checking Measurement request's duration only when request is on a non-serving channel */
			if (pMeasurementMgr->servingChannelID != pRequestArr[requestIndex]->channelNumber) {
				TI_UINT8 dtimPeriod;
				TI_UINT32 beaconInterval;
				TI_UINT32 dtimDuration;


				/* Checking duration doesn't exceed given max duration */
				if (pRequestArr[requestIndex]->DurationTime > pMeasurementMgr->maxDurationOnNonServingChannel) {
					if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
						pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest,
						                                   MSR_REJECT_DURATION_EXCEED_MAX_DURATION);

					return TI_FALSE;
				}


				/* Checking DTIM */

				/* Getting the DTIM count */
				param.paramType = SITE_MGR_DTIM_PERIOD_PARAM;
				siteMgr_getParam(pMeasurementMgr->hSiteMgr, &param);
				dtimPeriod = param.content.siteMgrDtimPeriod;

				/* Getting the beacon Interval */
				param.paramType = SITE_MGR_BEACON_INTERVAL_PARAM;
				siteMgr_getParam(pMeasurementMgr->hSiteMgr, &param);
				beaconInterval = param.content.beaconInterval;

				dtimDuration = beaconInterval * MEASUREMENT_BEACON_INTERVAL_IN_MICRO_SEC/MEASUREMENT_MSEC_IN_MICRO*dtimPeriod;
				if (pRequestArr[requestIndex]->DurationTime > dtimDuration) {
					if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
						pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest,
						                                   MSR_REJECT_DTIM_OVERLAP);

					return TI_FALSE;
				}
			}
		}
	}

	return TI_TRUE;
}

static TI_BOOL measurementMgrSM_measureInProgress(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *)hMeasurementMgr;

	if (pMeasurementMgr->currentState == MEASUREMENTMGR_STATE_MEASURING)
		return TI_TRUE;

	else
		return TI_FALSE;
}


