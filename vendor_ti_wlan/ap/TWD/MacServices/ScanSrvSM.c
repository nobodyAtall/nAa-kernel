/*
 * ScanSrvSM.c
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

/** \file ScanSrvSM.c
 *  \brief This file include the scan SRV Sm implementation
 *  \
 *  \date 10-Jan-2005
 */

#define __FILE_ID__  FILE_ID_116
#include "ScanSrvSM.h"
#include "ScanSrv.h"
#include "report.h"
#include "timer.h"
#include "MacServices_api.h"
#include "PowerSrv_API.h"
#include "CmdBld.h"


/********************************************************************************/
/*                      Internal functions prototypes.                          */
/********************************************************************************/

static TI_STATUS scanSRVSM_PsFailWhileScanning( TI_HANDLE hScanSrv );
static TI_STATUS actionNop( TI_HANDLE hScanSrv );
static TI_STATUS actionUnexpected( TI_HANDLE hScanSrv );


/********************************************************************************/
/*                      Interface functions Implementation.                     */
/********************************************************************************/


/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Initialize the scan SRV SM.
 *
 * Function Scope \e Public.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_init( TI_HANDLE hScanSrv )
{
	scanSRV_t* pScanSRV = (scanSRV_t*)hScanSrv;

	fsm_actionCell_t    smMatrix[ SCAN_SRV_NUM_OF_STATES ][ SCAN_SRV_NUM_OF_EVENTS ] = {
		/* next state and actions for IDLE state */
		{
			{SCAN_SRV_STATE_PS_WAIT, scanSRVSM_requestPS},                                /*"REQUEST_PS",*/
			{SCAN_SRV_STATE_IDLE, actionUnexpected},                                      /*"PS_FAIL",*/
			{SCAN_SRV_STATE_SCANNING, scanSRVSM_startActualScan},                         /*"PS_SUCCESS",  */
			{SCAN_SRV_STATE_IDLE, actionUnexpected},                                      /*"PS_PEND",*/
			{SCAN_SRV_STATE_IDLE, actionUnexpected},                                      /*"STOP_SCAN"*/
			{SCAN_SRV_STATE_IDLE, actionNop},                                             /*"FW_RESET"*/
			{SCAN_SRV_STATE_IDLE, actionUnexpected},                                      /*"TIMER_EXPIRED"*/
			{SCAN_SRV_STATE_IDLE, actionUnexpected},                                      /*"SCAN_COMPLETE"*/
		},


		/* next state and actions for PS_WAIT state */
		{
			{SCAN_SRV_STATE_PS_WAIT, actionUnexpected},                                   /*"REQUEST_PS",*/
			{SCAN_SRV_STATE_PS_EXIT, scanSRVSM_releasePS},                                /*"PS_FAIL",*/
			{SCAN_SRV_STATE_SCANNING, scanSRVSM_startActualScan},                         /*"PS_SUCCESS",  */
			{SCAN_SRV_STATE_PS_WAIT, actionNop},                                          /*"PS_PEND",*/
			{SCAN_SRV_STATE_STOPPING, actionNop},                                         /*"STOP_SCAN"*/
			{SCAN_SRV_STATE_IDLE, scanSRVSM_handleRecovery},                              /*"FW_RESET"*/
			{SCAN_SRV_STATE_PS_WAIT, actionUnexpected},                                   /*"TIMER_EXPIRED"*/
			{SCAN_SRV_STATE_PS_WAIT, actionUnexpected},                                   /*"SCAN_COMPLETE"*/
		},

		/* next state and actions for SCANNING state */
		{
			{SCAN_SRV_STATE_SCANNING, actionUnexpected},                                  /*"REQUEST_PS",*/
			{SCAN_SRV_STATE_SCANNING, scanSRVSM_PsFailWhileScanning},                     /*"PS_FAIL",*/
			{SCAN_SRV_STATE_SCANNING, actionUnexpected},                                  /*"PS_SUCCESS",  */
			{SCAN_SRV_STATE_SCANNING, actionUnexpected},                                  /*"PS_PEND",*/
			{SCAN_SRV_STATE_STOPPING, actionNop   },                                      /*"STOP_SCAN"*/
			{SCAN_SRV_STATE_IDLE, scanSRVSM_handleRecovery},                              /*"FW_RESET"*/
			{SCAN_SRV_STATE_SCANNING, scanSRVSM_handleTimerExpiry},                       /*"TIMER_EXPIRED"*/
			{SCAN_SRV_STATE_PS_EXIT, scanSRVSM_releasePS},                                /*"SCAN_COMPLETE"*/

		},

		/* next state and actions for STOPPING state */
		{
			{SCAN_SRV_STATE_STOPPING, actionUnexpected},                                  /*"REQUEST_PS",*/
			{SCAN_SRV_STATE_PS_EXIT, scanSRVSM_releasePS},                                /*"PS_FAIL",*/
			{SCAN_SRV_STATE_PS_EXIT, scanSRVSM_releasePS},                                /*"PS_SUCCESS",  */
			{SCAN_SRV_STATE_STOPPING, actionUnexpected},                                  /*"PS_PEND",*/
			{SCAN_SRV_STATE_STOPPING, actionNop     },                                    /*"STOP_SCAN"*/
			{SCAN_SRV_STATE_IDLE, scanSRVSM_handleRecovery},                              /*"FW_RESET"*/
			{SCAN_SRV_STATE_STOPPING, scanSRVSM_handleTimerExpiry},                       /*"TIMER_EXPIRED"*/
			{SCAN_SRV_STATE_PS_EXIT, scanSRVSM_releasePS}                                 /*"SCAN_COMPLETE"*/

		} ,

		/* next state and actions for PS_EXIT state */
		{
			{SCAN_SRV_STATE_PS_EXIT, actionUnexpected},                                   /*"REQUEST_PS",*/
			{SCAN_SRV_STATE_IDLE, scanSRVSM_notifyScanComplete},                          /*"PS_FAIL",*/
			{SCAN_SRV_STATE_IDLE, scanSRVSM_notifyScanComplete},                          /*"PS_SUCCESS",  */
			{SCAN_SRV_STATE_PS_EXIT, actionNop},                                          /*"PS_PEND",*/
			{SCAN_SRV_STATE_PS_EXIT, actionNop},                                          /*"STOP_SCAN"*/
			{SCAN_SRV_STATE_IDLE, scanSRVSM_handleRecovery},                              /*"FW_RESET"*/
			{SCAN_SRV_STATE_PS_EXIT, actionUnexpected},                                   /*"TIMER_EXPIRED"*/
			{SCAN_SRV_STATE_PS_EXIT, actionUnexpected},                                   /*"SCAN_COMPLETE"*/
		}
	};

	/* initialize current state */
	pScanSRV->SMState = SCAN_SRV_STATE_IDLE;

	/* configure the state machine */
	return fsm_Config( pScanSRV->SM, (fsm_Matrix_t)smMatrix,
	                   (TI_UINT8)SCAN_SRV_NUM_OF_STATES, (TI_UINT8)SCAN_SRV_NUM_OF_EVENTS,
	                   (fsm_eventActivation_t)scanSRVSM_SMEvent, pScanSRV->hOS );
}

/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Processes an event.
 *
 * Function Scope \e Public.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \param currentState - the current scan SRV SM state.\n
 * \param event - the event to handle.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_SMEvent( TI_HANDLE hScanSrv, scan_SRVSMStates_e* currentState,
                             scan_SRVSMEvents_e event )
{
	scanSRV_t *pScanSRV = (scanSRV_t *)hScanSrv;
	TI_STATUS status = TI_OK;
	TI_UINT8 nextState;

	/* obtain the next state */
	status = fsm_GetNextState( pScanSRV->SM, *(TI_UINT8*)currentState, (TI_UINT8)event, &nextState );
	if ( status != TI_OK ) {
		return TI_NOK;
	}


	/* move */
	return fsm_Event( pScanSRV->SM, (TI_UINT8*)currentState, (TI_UINT8)event, hScanSrv );
}

/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Request to enter driver mode from the power manager module.\n
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_requestPS( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;
	TI_STATUS psStatus;


	psStatus = powerSrv_ReservePS(  pScanSRV->hPowerSrv,
	                                pScanSRV->psRequest,
	                                pScanSRV->bSendNullData,
	                                hScanSrv,
	                                MacServices_scanSRV_powerSaveCB);

	switch (psStatus) {
		/* if successful */
	case POWER_SAVE_802_11_IS_CURRENT:
		/* send a PS_SUCCESS event */
		return scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );

		/* if pending */
	case POWER_SAVE_802_11_PENDING:
	case TI_OK:
		/* send a PS_PEND event */
		return scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_PEND );

		/* if not successful */
	default:

		/* mark not to exit from driver mode (no entry was performed) */
		pScanSRV->bExitFromDriverMode = TI_FALSE;

		/* if still wishing to scan */
		if ( pScanSRV->bScanOnDriverModeFailure ) {
			/* send a PS_SUCCESS event - scan will proceed regardless of the error */
			scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );
		}
		/* otherwise, return */
		else {
			/* mark the return code */
			pScanSRV->returnStatus = TI_NOK;
			/* send a PS_FAIL event */
			scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_FAIL );
		}
		break;
	}

	return TI_OK;
}

/**
 * \\n
 * \date 6-Oct-2005\n
 * \brief Request to release PS mode from the PowerSRV , and wait for answer.\n\n
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */

TI_STATUS scanSRVSM_releasePS( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;
	TI_STATUS psStatus;

	/* stop timer */
	if ( TI_TRUE == pScanSRV->bTimerRunning ) {
		tmr_StopTimer (pScanSRV->hScanSrvTimer);
		pScanSRV->bTimerRunning = TI_FALSE;
	}

	/* if exit from driver mode requested, do so */
	if ( TI_TRUE == pScanSRV->bExitFromDriverMode ) {
		/* here we need to get an answer if we succeeded to exit driver mode */

		psStatus = powerSrv_ReleasePS(  pScanSRV->hPowerSrv,
		                                pScanSRV->bSendNullData,
		                                hScanSrv,
		                                MacServices_scanSRV_powerSaveCB);


	} else {        /* no need to exit PS - send PS_SUCCESS */
		return scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );
	}


	switch (psStatus) {
		/* if successful */
	case POWER_SAVE_802_11_IS_CURRENT:
		/* send a PS_SUCCESS event */
		return scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );

		/* if pending */
	case POWER_SAVE_802_11_PENDING:
	case TI_OK:
		/* stay in the PS_EXIT state */
		break;

		/* if not successful */
	default:

		/* send a PS_FAIL event */
		return scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_FAIL );

	}

	return TI_OK;
}

/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Send the scan command to the firmware.\n
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_startActualScan( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;


	/* start the timer */
	pScanSRV->bTimerRunning = TI_TRUE;
	tmr_StartTimer (pScanSRV->hScanSrvTimer,
	                MacServices_scanSRV_scanTimerExpired,
	                (TI_HANDLE)pScanSRV,
	                MacServices_scanSRVcalculateScanTimeout (hScanSrv, pScanSRV->scanParams, !pScanSRV->bDtimOverlapping),
	                TI_FALSE);


	/* start the scan */
	/* we send the MacServices_scanSRVCommandMailBoxCB to be called when this command is recieved */
	if ( SCAN_TYPE_SPS == pScanSRV->scanParams->scanType ) {
		pScanSRV->returnStatus = cmdBld_CmdStartSPSScan (pScanSRV->hCmdBld, pScanSRV->scanParams, pScanSRV->eScanTag,
		                         (void *)MacServices_scanSRVCommandMailBoxCB, hScanSrv);
	} else {
		pScanSRV->returnStatus = cmdBld_CmdStartScan (pScanSRV->hCmdBld, pScanSRV->scanParams, pScanSRV->eScanTag,
		                         pScanSRV->bHighPriority , (void *)MacServices_scanSRVCommandMailBoxCB,
		                         hScanSrv);
	}
	/* if scan request failed */
	if ( TI_OK != pScanSRV->returnStatus ) {

		/* send a scan complete event. This will do all necessary clean-up (timer, power manager, notifying scan complete) */
		scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_SCAN_COMPLETE );
	}

	return TI_OK;
}


/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Notifies scan complete to upper layer.\n
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_notifyScanComplete( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;
	TCmdResponseCb CB_Func;
	TI_HANDLE  CB_Handle;
	TI_STATUS PSMode;

	/* call the scan complete CB - only if not currently running from within a request context! */
	if ( TI_FALSE == pScanSRV->bInRequest ) {
		/* this means that ResponseFunc was not called yet , so we call it before ScanComplete */
		if (pScanSRV->commandResponseFunc) {
			/* must erase CB function before calling it to enable nested scans */
			CB_Func = pScanSRV->commandResponseFunc;
			CB_Handle = pScanSRV->commandResponseObj;

			pScanSRV->commandResponseFunc = NULL;
			pScanSRV->commandResponseObj = NULL;

			/* if we reached here than response status was TI_OK */
			CB_Func(CB_Handle, TI_OK);

		}
		/* if function returns TI_TRUE than we are in PS mode , else - not */
		PSMode = powerSrv_getPsStatus(pScanSRV->hPowerSrv) ? POWER_SAVE_802_11_SUCCESS : POWER_SAVE_802_11_FAIL;




		pScanSRV->scanCompleteNotificationFunc( pScanSRV->scanCompleteNotificationObj,
		                                        pScanSRV->eScanTag,
		                                        pScanSRV->uResultCount,
		                                        pScanSRV->SPSScanResult,
		                                        pScanSRV->bTSFError,
		                                        pScanSRV->returnStatus,
		                                        PSMode );
	}

	return TI_OK;
}


/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Handles a timer expiry event - starts a recovery process.
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_handleTimerExpiry( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;

	/*
	 No scan complete event will trigger recovery only after a consecutive configurable number of
	 no scan complete events occurred.
	 */
	pScanSRV->currentNumberOfConsecutiveNoScanCompleteEvents++;

	if ( pScanSRV->currentNumberOfConsecutiveNoScanCompleteEvents >=
	     pScanSRV->numberOfNoScanCompleteToRecovery ) {

		pScanSRV->currentNumberOfConsecutiveNoScanCompleteEvents = 0;

		/* mark the return status */
		pScanSRV->returnStatus = TI_NOK;

		/* mark that the timer is no longer running */
		pScanSRV->bTimerRunning = TI_FALSE;

		/* call the recovery module */
		pScanSRV->failureEventFunc(pScanSRV->failureEventObj ,NO_SCAN_COMPLETE_FAILURE);
	} else {

		/* send a top scan command, which can help solving the FW bug described above */
		if ( TI_FALSE == pScanSRV->bSPSScan ) {
			cmdBld_CmdStopScan (pScanSRV->hCmdBld, pScanSRV->eScanTag, NULL, NULL);
		} else {
			cmdBld_CmdStopSPSScan (pScanSRV->hCmdBld, pScanSRV->eScanTag, NULL, NULL);
		}

		/* imitate a scan complete event to the SM */
		pScanSRV->bTSFError = TI_FALSE;
		pScanSRV->SPSScanResult = 0xffff;
		scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_SCAN_COMPLETE );
	}

	return TI_OK;
}

/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Handles PS Fail event while in Scanning - Indicate not to Exit PS.
 * This event can be reached when Roaming is invoked while in Scanning state.
 * The PM Module is stopped and generates PS Fail to all its clients.
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
static TI_STATUS scanSRVSM_PsFailWhileScanning( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;


	pScanSRV->bExitFromDriverMode = TI_FALSE;

	return TI_OK;
}


/**
 * \\n
 * \date 17-Jan-2005\n
 * \brief Handles a FW reset event (one that was detected outside the scan SRV) by stopping the timer.
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return TI_OK if successful, TI_NOK otherwise.\n
 */
TI_STATUS scanSRVSM_handleRecovery( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;


	/* The Power Manager is responsible to exit PS mode in recovery. Also, the scan CB is not called -
	   The SCR is responsible to notify scan concentrator of the event (which actually notifies scan SRV */

	/* if timer is running - stop it */
	if ( TI_TRUE == pScanSRV->bTimerRunning ) {
		tmr_StopTimer (pScanSRV->hScanSrvTimer);
		pScanSRV->bTimerRunning = TI_FALSE;
	}

	return TI_OK;
}

/**
 * \\n
 * \date 11-Jan-2005\n
 * \brief Handles an unexpected event.\n
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return always TI_OK.\n
 */
static TI_STATUS actionUnexpected( TI_HANDLE hScanSrv )
{
	scanSRV_t *pScanSRV = (scanSRV_t*)hScanSrv;


	if ( pScanSRV->bTimerRunning ) {
		tmr_StopTimer (pScanSRV->hScanSrvTimer);
		pScanSRV->bTimerRunning = TI_FALSE;
	}

	/* we must clean the old command response CB since they are no longer relevant
	  since the state machine may be corrupted */
	pScanSRV->commandResponseFunc = NULL;
	pScanSRV->commandResponseObj = NULL;

	/* indicate the unexpected event in the return status */
	pScanSRV->returnStatus = TI_NOK;

	return TI_OK;
}

/**
 * \\n
 * \date 10-Jan-2005\n
 * \brief Handles an event that doesn't require any action.\n
 *
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \return always TI_OK.\n
 */
static TI_STATUS actionNop( TI_HANDLE hScanSrv )
{
	return TI_OK;
}


