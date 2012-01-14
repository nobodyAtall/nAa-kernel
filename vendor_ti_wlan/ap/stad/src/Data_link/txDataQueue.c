/*
 * txDataQueue.c
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


/** \file   txDataQueue.c
 *  \brief  The Tx Data Queues module.
 *
 *  \see    txDataQueue.h
 */


#define __FILE_ID__  FILE_ID_60
#include "paramOut.h"
#include "osApi.h"
#include "report.h"
#include "timer.h"
#include "queue.h"
#include "context.h"
#include "Ethernet.h"
#include "TWDriver.h"
#include "DataCtrl_Api.h"
#include "txDataQueue.h"
#include "txCtrl.h"
#include "DrvMainModules.h"
#include "bmtrace_api.h"
#include "report.h"


/* Internal Functions prototypes */
static void txDataQ_RunScheduler (TI_HANDLE hTxDataQ);
static void txDataQ_UpdateQueuesBusyState (TTxDataQ *pTxDataQ, TI_UINT32 uTidBitMap);
static void txDataQ_UpdateLinksBusyState (TTxDataQ *pTxDataQ, TI_UINT32 uLinkBitMap);
static void txDataQ_TxSendPaceTimeout (TI_HANDLE hTxDataQ, TI_BOOL bTwdInitOccured);
#ifdef TI_DBG
static void txDataQ_PrintResources (TTxDataQ *pTxDataQ);
#endif /* TI_DBG */
static void txDataQ_InitResources (TTxDataQ *pTxDataQ, TTxDataResourcesParams_t *pDataRsrcParams);
extern void wlanDrvIf_StopTx (TI_HANDLE hOs);
extern void wlanDrvIf_ResumeTx (TI_HANDLE hOs);

/***************************************************************************
*                      PUBLIC  FUNCTIONS  IMPLEMENTATION				   *
****************************************************************************/


/**
 * \fn     txDataQ_Create
 * \brief  Create the module and its queues
 *
 * Create the Tx Data module and its queues.
 *
 * \note
 * \param  hOs - Handle to the Os Abstraction Layer
 * \return Handle to the allocated Tx Data Queue module (NULL if failed)
 * \sa
 */
TI_HANDLE txDataQ_Create(TI_HANDLE hOs)
{
	TTxDataQ *pTxDataQ;

	/* allocate TxDataQueue module */
	pTxDataQ = os_memoryAlloc (hOs, (sizeof(TTxDataQ)));

	if (!pTxDataQ) {
		WLAN_OS_REPORT(("Error allocating the TxDataQueue Module\n"));
		return NULL;
	}

	/* Reset TxDataQueue module */
	os_memoryZero (hOs, pTxDataQ, (sizeof(TTxDataQ)));

	return (TI_HANDLE)pTxDataQ;
}


/**
 * \fn     txDataQ_Init
 * \brief  Save required modules handles
 *
 * Save other modules handles.
 *
 * \note
 * \param  pStadHandles  - The driver modules handles
 * \return void
 * \sa
 */
void txDataQ_Init (TStadHandlesList *pStadHandles)
{
	TTxDataQ  *pTxDataQ = (TTxDataQ *)(pStadHandles->hTxDataQ);
	TI_UINT32  uNodeHeaderOffset = TI_FIELD_OFFSET(TTxnStruct, tTxnQNode);
	TI_UINT8   uQueId;
	TDataLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	/* save modules handles */
	pTxDataQ->hContext	= pStadHandles->hContext;
	pTxDataQ->hTxCtrl	= pStadHandles->hTxCtrl;
	pTxDataQ->hOs		= pStadHandles->hOs;
	pTxDataQ->hReport	= pStadHandles->hReport;
	pTxDataQ->hTxMgmtQ	= pStadHandles->hTxMgmtQ;
	pTxDataQ->hTWD	    = pStadHandles->hTWD;

	/* Configures the Port Default status to Close */
	pTxDataQ->bDataPortEnable = TI_FALSE;

	/* Configures the NextQueId to zero => scheduler will strart from Queue 1*/
	pTxDataQ->uNextQueId = 0;
	pTxDataQ->uNextHlid = 0;

	/* init the number of the Data queue to be used */
	pTxDataQ->uNumQueues = MAX_NUM_OF_AC;

	/* init the max size of the Data queues */
	pTxDataQ->aQueueMaxSize[QOS_AC_BE] = DATA_QUEUE_DEPTH_BE;
	pTxDataQ->aQueueMaxSize[QOS_AC_BK] = DATA_QUEUE_DEPTH_BK;
	pTxDataQ->aQueueMaxSize[QOS_AC_VI] = DATA_QUEUE_DEPTH_VI;
	pTxDataQ->aQueueMaxSize[QOS_AC_VO] = DATA_QUEUE_DEPTH_VO;

	for (uQueId = 0; uQueId < pTxDataQ->uNumQueues; uQueId++) {
		pTxDataQ->aTxSendPaceThresh[uQueId] = 1;
	}

	/*
	 * init all queues in all links
	 */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */
		pLinkQ->bBusy = TI_FALSE; /* default is not busy */
		pLinkQ->bEnabled = TI_FALSE; /* default is not enabled */

		/* Create the tx data queues */
		for (uQueId = 0; uQueId < pTxDataQ->uNumQueues; uQueId++) {
			pLinkQ->aQueues[uQueId] = que_Create (pTxDataQ->hOs,
			                                      pTxDataQ->hReport,
			                                      pTxDataQ->aQueueMaxSize[uQueId],
			                                      uNodeHeaderOffset);

			/* If any Queues' allocation failed, print error, free TxDataQueue module and exit */
			if (pLinkQ->aQueues[uQueId] == NULL) {
				WLAN_OS_REPORT(("Failed to create queue\n"));
				os_memoryFree (pTxDataQ->hOs, pTxDataQ, sizeof(TTxDataQ));
				return;
			}

			/* Configure the Queues default values */
			pLinkQ->aNetStackQueueStopped[uQueId] = TI_FALSE;
		}
	}
	/* Init busy flag per AC (not also per link) */
	for (uQueId = 0; uQueId < pTxDataQ->uNumQueues; uQueId++) {
		pTxDataQ->aQueueBusy[uQueId] = TI_FALSE;
	}
	pTxDataQ->hTxSendPaceTimer = tmr_CreateTimer (pStadHandles->hTimer);
	if (pTxDataQ->hTxSendPaceTimer == NULL) {
		return;
	}

	/* Register to the context engine and get the client ID */
	pTxDataQ->uContextId = context_RegisterClient (pTxDataQ->hContext,
	                       txDataQ_RunScheduler,
	                       (TI_HANDLE)pTxDataQ,
	                       TI_TRUE,
	                       "TX_DATA",
	                       sizeof("TX_DATA"));

}


/**
 * \fn     txDataQ_SetDefaults
 * \brief  Configure module with default settings
 *
 * Init the Tx Data queues.
 * Register as the context-engine client.
 *
 * \note
 * \param  hTxDataQ - The object
 * \param  Other modules handles
 * \return TI_OK on success or TI_NOK on failure
 * \sa
 */
TI_STATUS txDataQ_SetDefaults (TI_HANDLE  hTxDataQ, txDataInitParams_t *pTxDataInitParams)
{
	TTxDataQ  *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TI_STATUS  eStatus;

	/* configure the classifier sub-module */
	eStatus = txDataClsfr_Config (hTxDataQ, &pTxDataInitParams->ClsfrInitParam);
	if (eStatus != TI_OK) {
		WLAN_OS_REPORT(("FATAL ERROR: txDataQ_SetDefaults(): txDataClsfr_Config failed - Aborting\n"));
		return eStatus;
	}

	/* Save the module's parameters settings */
	pTxDataQ->bStopNetStackTx              = pTxDataInitParams->bStopNetStackTx;
	pTxDataQ->aTxSendPaceThresh[QOS_AC_BE] = pTxDataInitParams->uTxSendPaceThresh;
	pTxDataQ->aTxSendPaceThresh[QOS_AC_BK] = pTxDataInitParams->uTxSendPaceThresh;
	pTxDataQ->aTxSendPaceThresh[QOS_AC_VI] = pTxDataInitParams->uTxSendPaceThresh;
	pTxDataQ->aTxSendPaceThresh[QOS_AC_VO] = 1;     /* Don't delay voice packts! */

	/* configure the classifier sub-module */
	txDataQ_InitResources (pTxDataQ, &pTxDataInitParams->tDataRsrcParam);


	return TI_OK;
}


/**
 * \fn     txDataQ_Destroy
 * \brief  Destroy the module and its queues
 *
 * Clear and destroy the queues and then destroy the module object.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return TI_OK - Unload succesfull, TI_NOK - Unload unsuccesfull
 * \sa
 */
TI_STATUS txDataQ_Destroy (TI_HANDLE hTxDataQ)
{
	TTxDataQ  *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TI_STATUS  status = TI_OK;
	TI_UINT32  uQueId;
	TDataLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	/* Dequeue and free all queued packets */
	txDataQ_ClearQueues (hTxDataQ);

	/*
	 * init all queues in all links
	 */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */

		/* Free Data queues */
		for (uQueId = 0 ; uQueId < pTxDataQ->uNumQueues ; uQueId++) {
			if (que_Destroy(pLinkQ->aQueues[uQueId]) != TI_OK) {
				status = TI_NOK;
			}
		}
	}

	/* free timer */
	if (pTxDataQ->hTxSendPaceTimer) {
		tmr_DestroyTimer (pTxDataQ->hTxSendPaceTimer);
	}

	/* Free Tx Data Queue Module */
	os_memoryFree (pTxDataQ->hOs, pTxDataQ, sizeof(TTxDataQ));

	return status;
}


/**
 * \fn     txDataQ_ClearQueues
 * \brief  Clear all queues
 *
 * Dequeue and free all queued packets.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa
 */
void txDataQ_ClearQueues (TI_HANDLE hTxDataQ)
{
	TI_UINT32  uHlid;

	/*
	 * init all queues in all links
	 */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		txDataQ_FlushLinkQueues(hTxDataQ, uHlid);
	}
}


/**
 * \fn     txDataQ_FlushLinkQueues
 * \brief  Flush all queues of the specific link
 *
 * Free all pending packets in link queue
 *
 * \note
 * \param  hTxDataQ - The object
 * \param  uHlid - Link ID
 * \return void
 * \sa
 */
void txDataQ_FlushLinkQueues (TI_HANDLE hTxDataQ, TI_UINT32 uHlid)
{
	TTxDataQ   *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TTxCtrlBlk *pPktCtrlBlk;
	TI_UINT32  uQueId;
	TDataLinkQ *pLinkQ;

	pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */

	/* Dequeue and free all queued packets */
	for (uQueId = 0 ; uQueId < pTxDataQ->uNumQueues ; uQueId++) {
		while (1) {
			context_EnterCriticalSection (pTxDataQ->hContext);
			pPktCtrlBlk = (TTxCtrlBlk *) que_Dequeue (pLinkQ->aQueues[uQueId]);
			context_LeaveCriticalSection (pTxDataQ->hContext);
			if (pPktCtrlBlk == NULL) {
				break;
			}
			txCtrl_FreePacket (pTxDataQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
		}
	}
}

/**
 * \fn     txDataQ_SetLinkType
 * \brief  set link type
 *
 * \note
 * \param  hTxDataQ     - The module's object
 * \param  uHlid        - link id
 * \param  eLinkType    - link state
 * \return void
 */
void txDataQ_SetLinkType (TI_HANDLE hTxDataQ, TI_UINT32 uHlid, EWlanLinkType eLinkType)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ = &pTxDataQ->aDataLinkQ[uHlid];

	pLinkQ->eType = eLinkType;


	/* save broadcast link id */
	if (pLinkQ->eType == WLANLINK_TYPE_BRCST) {
		pTxDataQ->uBcastHlid = uHlid;
	}
}

/**
 * \fn     txDataQ_InsertPacket
 * \brief  Insert packet in queue and schedule task
 *
 * This function is called by the hard_start_xmit() callback function.
 * If the packet it an EAPOL, forward it to the Mgmt-Queue.
 * Otherwise, classify the packet, enqueue it and request
 *   context switch for handling it in the driver's context.
 *
 * \note
 * \param  hTxDataQ    - The object
 * \param  pPktCtrlBlk - Pointer to the packet
 * \param  uPacketDtag - The packet priority optionaly set by the OAL
 * \return TI_OK - if the packet was queued, TI_NOK - if the packet was dropped.
 * \sa     txDataQ_Run
 */


TI_STATUS txDataQ_InsertPacket (TI_HANDLE hTxDataQ, TTxCtrlBlk *pPktCtrlBlk, TI_UINT8 uPacketDtag, TIntraBssBridge *pIntraBssBridgeParam)
{
	TTxDataQ        *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TEthernetHeader *pEthHead = (TEthernetHeader *)(pPktCtrlBlk->tTxnStruct.aBuf[0]);
	TI_STATUS        eStatus;
	TI_UINT32        uQueId;
	TI_UINT32        uQueSize;
	txCtrl_t         *pTxCtrl = (txCtrl_t *)(pTxDataQ->hTxCtrl);
	TI_BOOL          bRequestSchedule = TI_FALSE;
	TI_BOOL          bStopNetStack = TI_FALSE;
	TDataLinkQ       *pLinkQ;
	TI_UINT32        uHlid;

	/* If packet is EAPOL or from the generic Ethertype, forward it to the Mgmt-Queue and exit */
	if ((HTOWLANS(pEthHead->type) == ETHERTYPE_EAPOL) ||
	    (HTOWLANS(pEthHead->type) == pTxCtrl->genericEthertype)) {
		pPktCtrlBlk->tTxPktParams.uPktType = TX_PKT_TYPE_EAPOL;

		return txMgmtQ_Xmit (pTxDataQ->hTxMgmtQ, pPktCtrlBlk, TI_TRUE);
		/* Note: The last parameter indicates that we are running in external context */
	}

	/* Find link id by destination MAC address, if not found drop the packet */
	/* use Intra Bss bridge params*/
	if(!pIntraBssBridgeParam) {
		if (TI_UNLIKELY(MAC_MULTICAST(pEthHead->dst))) {
			uHlid = pTxDataQ->uBcastHlid;
		} else {
			if (txDataQ_LinkMacFind( hTxDataQ, &uHlid, pEthHead->dst) != TI_OK) {
				/* If the packet can't be queued drop it */
				txCtrl_FreePacket (pTxDataQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
				pTxDataQ->uLinkNotFoundCount++;
				return TI_NOK;
			}
		}
	} else {
		uHlid = pIntraBssBridgeParam->uParam;
	}
	pPktCtrlBlk->tTxDescriptor.hlid = uHlid;

	pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */

	pPktCtrlBlk->tTxPktParams.uPktType = TX_PKT_TYPE_ETHER;
	/*  set encryption bit */
	if (pLinkQ->bEncrypt) {
		SET_PKT_TYPE_ENCRYPT(pPktCtrlBlk);
	}

	/* Enter critical section to protect classifier data and queue access */
	context_EnterCriticalSection (pTxDataQ->hContext);

	/* Call the Classify function to set the TID field */
	if (txDataClsfr_ClassifyTxPacket (hTxDataQ, pPktCtrlBlk, uPacketDtag) != TI_OK) {
#ifdef TI_DBG
		pTxDataQ->uClsfrMismatchCount++;
#endif /* TI_DBG */
	}

	uQueId = aTidToQueueTable[pPktCtrlBlk->tTxDescriptor.tid];

	/* Check resources per LINK and per AC */
	if (txDataQ_AllocCheckResources( hTxDataQ, pPktCtrlBlk) != TI_OK) {
#ifdef TI_DBG
		pLinkQ->aQueueCounters[uQueId].uDroppedPacket++;
		pTxDataQ->uNoResourcesCount++;
#endif /* TI_DBG */

		/* Leave critical section */
		context_LeaveCriticalSection (pTxDataQ->hContext);
		/* If the packet can't be queued drop it - Should be out of the critical section */
		/* !!! This call should be out of the critical section */
		txCtrl_FreePacket (pTxDataQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
		return TI_NOK;
	}

	/* Enqueue the packet in the appropriate Queue */
	eStatus = que_Enqueue (pLinkQ->aQueues[uQueId], (TI_HANDLE)pPktCtrlBlk);

	/* Get number of packets in current queue */
	uQueSize = que_Size (pLinkQ->aQueues[uQueId]);

	/* If the current queue is not stopped */
	if (pTxDataQ->aQueueBusy[uQueId] == TI_FALSE) {
		/* If the queue has the desired number of packets, request switch to driver context for handling them */
		if (uQueSize == pTxDataQ->aTxSendPaceThresh[uQueId]) {
			tmr_StopTimer (pTxDataQ->hTxSendPaceTimer);
			bRequestSchedule = TI_TRUE;
		}
		/* If below Tx-Send pacing threshold, start timer to trigger packets handling if expired */
		else if (uQueSize < pTxDataQ->aTxSendPaceThresh[uQueId]) {
			tmr_StartTimer (pTxDataQ->hTxSendPaceTimer,
			                txDataQ_TxSendPaceTimeout,
			                hTxDataQ,
			                TX_SEND_PACE_TIMEOUT_MSEC,
			                TI_FALSE);
		}
	}

	/* If allowed to stop network stack and the queue is full, indicate to stop network and
	      to schedule Tx handling (both are executed below, outside the critical section!) */
	if ((pTxDataQ->bStopNetStackTx) && (uQueSize == pTxDataQ->aQueueMaxSize[uQueId])) {
		pLinkQ->aNetStackQueueStopped[uQueId] = TI_TRUE;
		bRequestSchedule = TI_TRUE;
		bStopNetStack = TI_TRUE;
	}

	/* Leave critical section */
	context_LeaveCriticalSection (pTxDataQ->hContext);

	/* If needed, schedule Tx handling */
	if (bRequestSchedule) {
		context_RequestSchedule (pTxDataQ->hContext, pTxDataQ->uContextId);
	}

	/* If needed, stop the network stack Tx */
	if (bStopNetStack) {
		/* Stop the network stack from sending Tx packets as we have at least one date queue full.
		Note that in some of the OS's (e.g Win Mobile) it is implemented by blocking the thread! */
		wlanDrvIf_StopTx (pTxDataQ->hOs);
	}

	if (eStatus != TI_OK) {
		/* If the packet can't be queued drop it */
		txCtrl_FreePacket (pTxDataQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
#ifdef TI_DBG
		pLinkQ->aQueueCounters[uQueId].uDroppedPacket++;
#endif /* TI_DBG */
	} else {
#ifdef TI_DBG
		pLinkQ->aQueueCounters[uQueId].uEnqueuePacket++;
#endif /* TI_DBG */
	}


	return eStatus;
}


/**
 * \fn     txDataQ_StopLink
 * \brief  Stop Data-Queue module access to Tx link.
 *
 * Called by the backpressure.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa     txDataQ_StartLink
 */
void txDataQ_StopLink (TI_HANDLE hTxDataQ, TI_UINT32 uHlid)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ = &pTxDataQ->aDataLinkQ[uHlid];

	pLinkQ->bBusy = TI_TRUE;
}

/**
 * \fn     txDataQ_StopQueue
 * \brief  Set queue's busy indication
 *
 * This function is called by the txCtrl_xmitData() if the queue's backpressure
 *   indication is set.
 * It sets the internal queue's Busy indication.
 *
 * \note
 * \param  hTxDataQ - The object
 * \param  uTidBitMap   - The changed TIDs busy bitmap
 * \return void
 * \sa     txDataQ_UpdateBusyMap
 */
void txDataQ_StopQueue (TI_HANDLE hTxDataQ, TI_UINT32 uTidBitMap)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	/* Set the relevant queue(s) busy flag */
	txDataQ_UpdateQueuesBusyState (pTxDataQ, uTidBitMap);
}


/**
 * \fn     txDataQ_UpdateBusyMap
 * \brief  Set queue's busy indication
 *
 * This function is called by the txCtrl if the backpressure map per TID is changed.
 * This could be as a result of Tx-Complete, admission change or association.
 * The function modifies the internal queue's Busy indication and calls the scheduler.
 *
 * \note
 * \param  hTxDataQ - The object
 * \param  uTidBitMap   - The changed TIDs busy bitmap
 * \param  uLinkBitMap   - The changed LINKs busy bitmap
 * \return void
 * \sa     txDataQ_StopQueue
 */
void txDataQ_UpdateBusyMap (TI_HANDLE hTxDataQ, TI_UINT32 tidBitMap, TI_UINT32 uLinkBitMap)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	/* Update the Link(s) mode */
	txDataQ_UpdateLinksBusyState (pTxDataQ, tidBitMap);

	/* Update the Queue(s) mode */
	txDataQ_UpdateQueuesBusyState (pTxDataQ, tidBitMap);

	/* Run the scheduler */
	txDataQ_RunScheduler (hTxDataQ);
}


/**
 * \fn     txDataQ_StopAll
 * \brief  Disable Data-Queue module access to Tx path.
 *
 * Called by the Tx-Port when the data-queue module can't access the Tx path.
 * Sets stop-all-queues indication.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa     txDataQ_WakeAll
 */
void txDataQ_StopAll (TI_HANDLE hTxDataQ)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	/* Disable the data Tx port */
	pTxDataQ->bDataPortEnable = TI_FALSE;
}


/**
 * \fn     txDataQ_WakeAll
 * \brief  Enable Data-Queue module access to Tx path.
 *
 * Called by the Tx-Port when the data-queue module can access the Tx path.
 * Clears the stop-all-queues indication and calls the scheduler.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa     txDataQ_StopAll
 */
void txDataQ_WakeAll (TI_HANDLE hTxDataQ)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	/* Enable the data Tx port */
	pTxDataQ->bDataPortEnable = TI_TRUE;

	/* Run the scheduler */
	txDataQ_RunScheduler (hTxDataQ);
}

/**
 * \fn     txDataQ_DisableLink
 * \brief  Disable Data-Queue module access to Tx link.
 *
 * Called by the Tx-Port when the data-queue module can't access the Tx link.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa     txDataQ_EnableLink
 */
void txDataQ_DisableLink (TI_HANDLE hTxDataQ, TI_UINT32 uHlid)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ = &pTxDataQ->aDataLinkQ[uHlid];

	pLinkQ->bEnabled = TI_FALSE;

}


/**
 * \fn     txDataQ_EnableLink
 * \brief  Enable Data-Queue module access to Tx link.
 *
 * Called by the Tx-Port when the data-queue module can access the Tx link.
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa     txDataQ_StopLink
 */
void txDataQ_EnableLink (TI_HANDLE hTxDataQ, TI_UINT32 uHlid)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ = &pTxDataQ->aDataLinkQ[uHlid];

	pLinkQ->bEnabled = TI_TRUE;


	/* Run the scheduler */
	txDataQ_RunScheduler (hTxDataQ);
}

/**
 * \fn     txDataQ_LinkMacAdd
 * \brief  Set MAC address for the link id.
 *
  * \return void
 * \sa     txDataQ_LinkMacAdd
 */
TI_STATUS txDataQ_LinkMacAdd (TI_HANDLE hTxDataQ, TI_UINT32 uHlid, TMacAddr tMacAddr)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	if (uHlid >= LINK_MAC_TABLE_SIZE) {
		WLAN_OS_REPORT(("%s: illegal uHlid = %d\n", __FUNCTION__, uHlid));
		return TI_NOK;
	}
	/* Enter critical section to protect links data */
	context_EnterCriticalSection (pTxDataQ->hContext);

	pTxDataQ->aLinkMac[uHlid].uValid = TI_TRUE;
	MAC_COPY (pTxDataQ->aLinkMac[uHlid].tMacAddr, tMacAddr);

	context_LeaveCriticalSection (pTxDataQ->hContext);

	return TI_OK;
}

/**
 * \fn     txDataQ_LinkMacRemove
 * \brief  Set LinkMac table entry as invalid
 *
 * \return void
 * \sa     txDataQ_LinkMacRemove
 */
void txDataQ_LinkMacRemove (TI_HANDLE hTxDataQ, TI_UINT32 uHlid)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	if (uHlid >= LINK_MAC_TABLE_SIZE) {
		WLAN_OS_REPORT(("%s: illegal uHlid = %d\n", __FUNCTION__, uHlid));
		return;
	}
	/* Enter critical section to protect links data */
	context_EnterCriticalSection (pTxDataQ->hContext);
	pTxDataQ->aLinkMac[uHlid].uValid = TI_FALSE;
	context_LeaveCriticalSection (pTxDataQ->hContext);
}


/**
 * \fn     txDataQ_LinkMacFind
 * \brief  Find entry with MAC address
 *
 * \return status
 * \sa     txDataQ_LinkMacFind
 */
TI_STATUS txDataQ_LinkMacFind (TI_HANDLE hTxDataQ, TI_UINT32 *uHlid, TMacAddr tMacAddr)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	int i;
	int j;

	/* Enter critical section to protect links data */
	context_EnterCriticalSection (pTxDataQ->hContext);
	for (i=0; i<LINK_MAC_TABLE_SIZE; i++) {
		if (!pTxDataQ->aLinkMac[i].uValid) {
			/* entry not valid, skip to next entry */
			continue;
		}
		for (j=MAC_ADDR_LEN-1; j>=0; j--) {
			if (pTxDataQ->aLinkMac[i].tMacAddr[j] != tMacAddr[j]) {
				/* different MAC, skip to next entry */
				break;
			}
		}
		if (j < 0) {
			/* Found, return index */
			*uHlid = i;
			context_LeaveCriticalSection (pTxDataQ->hContext);
			return TI_OK;
		}
	}
	context_LeaveCriticalSection (pTxDataQ->hContext);

	/* Not found */
	*uHlid = 0xff; /* for debug */
	return TI_NOK;
}

/**
 * \fn     txDataQ_CheckResources
 * \brief  Check resources per Link and per Ac
 *
 * NOTE: the caller only will protect with critical section
 *
 * \return TI_STATUS TI_NOK when there are no resources
 */
TI_STATUS txDataQ_AllocCheckResources (TI_HANDLE hTxDataQ, TTxCtrlBlk *pPktCtrlBlk)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataResources *pDataRsrc = &pTxDataQ->tDataRsrc;
	TI_UINT32 uHlid = pPktCtrlBlk->tTxDescriptor.hlid;
	TI_UINT32 uAc = WMEQosTagToACTable[pPktCtrlBlk->tTxDescriptor.tid];
	TI_BOOL bPktInUse_AboveAcMin = TI_FALSE;
	TI_BOOL bPktInUse_AboveLinkMin = TI_FALSE;
	TI_BOOL bEnqueuePacket;

	/* new packet, Increment packet in use counters */
	pDataRsrc->uPktInUse_PerAc[uAc]++;
	pDataRsrc->uPktInUse_PerLink[uHlid]++;
	/* set RSRC_ALLOCATED flag, used in txDataQ_FreeResources */
	SET_TX_CTRL_FLAG_RSRC_ALLOCATED(pPktCtrlBlk);

	/* Update Effective totals = Sum of Max ( PktInUse_PerAc [uAc],  Min_PerAc[uAc] ), uAc=0..MAX_AC */
	/* no need to calculate Sum of Max on every packet, just small check for this ac only */
	if (pDataRsrc->uPktInUse_PerAc[uAc] > pDataRsrc->uMinGuarantee_PerAc[uAc]) {
		pDataRsrc->uEffectiveTotal_Ac++;
		bPktInUse_AboveAcMin = TI_TRUE;
	}

	/* Update Effective totals = Sum of Max ( PktInUse_PerLik [uHlid],  Min_PerLink[uHlid] ), uHlid=0..MAX_LINK */
	/* no need to calculate Sum of Max on every packet, just small check for this link only */
	if (pDataRsrc->uPktInUse_PerLink[uHlid] > pDataRsrc->uMinGuarantee_PerLink) {
		pDataRsrc->uEffectiveTotal_Link++;
		bPktInUse_AboveLinkMin = TI_TRUE;
	}

	/* if both effective total are above Max total, enqueue the packet */
	if ( (pDataRsrc->uEffectiveTotal_Ac < pDataRsrc->uMaxTotal) && (pDataRsrc->uEffectiveTotal_Link < pDataRsrc->uMaxTotal) ) {
		bEnqueuePacket = TI_TRUE;
	} else
		/* In this point one of the EffectiveTotal reach the MaxTotal */
		/* if uEffectiveTotal_Ac reach Max total, check if also ac counter above min  */
		if ( (pDataRsrc->uEffectiveTotal_Ac >= pDataRsrc->uMaxTotal) && (bPktInUse_AboveAcMin) ) {
			bEnqueuePacket = TI_FALSE;
		} else
			/* if uEffectiveTotal_Link reach Max total, check if also link counter above min  */
			if ( (pDataRsrc->uEffectiveTotal_Link >= pDataRsrc->uMaxTotal) && (bPktInUse_AboveLinkMin) ) {
				bEnqueuePacket = TI_FALSE;
			}
	/* both reach Max total, but al least one is below the MinGuarantee threshold */
			else {
				bEnqueuePacket = TI_TRUE;
			}

	/* save AC in tx ctrl block, it may change when QOS is disabled and by AC downgrade algo, used in txDataQ_FreeResources */
	SET_TX_CTRL_FLAG_RSRC_AC(pPktCtrlBlk, uAc);

	if (!bEnqueuePacket) {
		/* In case of failure, the caller will free the resources */
		return TI_NOK;
	}

	return TI_OK;
}

/**
 * \fn     txDataQ_FreeResources
 * \brief  Free resources per Link and per Ac
 *
 */
void txDataQ_FreeResources (TI_HANDLE hTxDataQ, TTxCtrlBlk *pPktCtrlBlk)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataResources *pDataRsrc = &pTxDataQ->tDataRsrc;
	TI_UINT32 uHlid = pPktCtrlBlk->tTxDescriptor.hlid;
	TI_UINT32 uAc;

	/* Free TxData resources only if previous allocated by txDataQ_AllocCheckResources */
	if (!IS_TX_CTRL_FLAG_RSRC_ALLOCATED(pPktCtrlBlk)) {
		return;
	}

	/* Enter critical section to protect classifier data and queue access */
	context_EnterCriticalSection (pTxDataQ->hContext);

	/* Extract original AC (saved by txDataQ_AllocCheckResources) from tx ctrl block */
	uAc = GET_TX_CTRL_FLAG_RSRC_AC(pPktCtrlBlk);

	/* new packet, Increment packet in use counters */
	pDataRsrc->uPktInUse_PerAc[uAc]--;
	pDataRsrc->uPktInUse_PerLink[uHlid]--;

	/* Update Effective totals = Sum of Max ( PktInUse_PerAc [uAc],  Min_PerAc[uAc] ), uAc=0..MAX_AC */
	/* no need to calculate Sum of Max on every packet, just small check for this ac only */
	if (pDataRsrc->uPktInUse_PerAc[uAc] >= pDataRsrc->uMinGuarantee_PerAc[uAc]) {
		pDataRsrc->uEffectiveTotal_Ac--;
#ifdef TI_DBG
		/* sanity check */
		if (pDataRsrc->uEffectiveTotal_Ac < pDataRsrc->uEffectiveTotal_Ac_Min ) {
			WLAN_OS_REPORT(("%s: uEffectiveTotal_Ac=%d is below MIN=%d\n", __FUNCTION__, pDataRsrc->uEffectiveTotal_Ac, pDataRsrc->uEffectiveTotal_Ac_Min));
		}
#endif
	}

	/* Update Effective totals = Sum of Max ( PktInUse_PerLik [uHlid],  Min_PerLink[uHlid] ), uHlid=0..MAX_LINK */
	/* no need to calculate Sum of Max on every packet, just small check for this link only*/
	if (pDataRsrc->uPktInUse_PerLink[uHlid] >= pDataRsrc->uMinGuarantee_PerLink) {
		pDataRsrc->uEffectiveTotal_Link--;
#ifdef TI_DBG
		/* sanity check */
		if (pDataRsrc->uEffectiveTotal_Link < pDataRsrc->uEffectiveTotal_Link_Min ) {
			WLAN_OS_REPORT(("%s: uEffectiveTotal_Ac=%d is below MIN=%d\n", __FUNCTION__, pDataRsrc->uEffectiveTotal_Link, pDataRsrc->uEffectiveTotal_Link_Min));
		}
#endif
	}

	/* Leave critical section */
	context_LeaveCriticalSection (pTxDataQ->hContext);
}

/**
 * \fn     txDataQ_InitResources
 * \brief  Init resources counters per Link and per Ac
 *
 */
static void txDataQ_InitResources (TTxDataQ *pTxDataQ, TTxDataResourcesParams_t *pDataRsrcParams)
{
	TDataResources *pDataRsrc = &pTxDataQ->tDataRsrc;
	TI_UINT32 uHlid;
	TI_UINT32 uAc;

	/* Init counters per AC */
	pDataRsrc->uEffectiveTotal_Ac = 0;
	for (uAc=0; uAc<MAX_NUM_OF_AC; uAc++) {
		pDataRsrc->uPktInUse_PerAc[uAc] = 0;
		pDataRsrc->uMinGuarantee_PerAc[uAc] = pDataRsrcParams->uMinGuarantee_PerAc[uAc];
		pDataRsrc->uEffectiveTotal_Ac += pDataRsrc->uMinGuarantee_PerAc[uAc];
	}

	/* Init counters per LINK */
	pDataRsrc->uEffectiveTotal_Link = 0;
	pDataRsrc->uMinGuarantee_PerLink = pDataRsrcParams->uMinGuarantee_PerLink;
	for (uHlid=0; uHlid<WLANLINKS_MAX_LINKS; uHlid++) {
		pDataRsrc->uPktInUse_PerLink[uHlid] = 0;
		pDataRsrc->uEffectiveTotal_Link += pDataRsrc->uMinGuarantee_PerLink;
	}
#ifdef TI_DBG
	/* for sanity check */
	pDataRsrc->uEffectiveTotal_Ac_Min += pDataRsrc->uEffectiveTotal_Ac;
	pDataRsrc->uEffectiveTotal_Link_Min += pDataRsrc->uEffectiveTotal_Link;
#endif

	pDataRsrc->uMaxTotal = CTRL_BLK_ENTRIES_NUM; /* Max total is the MAX tx ctrl blocks */
}


/**
 * \fn     txDataQ_GetBcasttLink
 * \brief  Get Broadcast Link Id
 *
 * \return void
 * \sa     txDataQ_GetBcasttLink
 */

void txDataQ_GetBcastLink  (TI_HANDLE hTxDataQ, TI_UINT32 *uHlid)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	*uHlid = pTxDataQ->uBcastHlid;
}
/***************************************************************************
*                       DEBUG  FUNCTIONS  IMPLEMENTATION			       *
****************************************************************************/

#ifdef TI_DBG

/**
 * \fn     txDataQ_PrintModuleParams
 * \brief  Print Module Parameters
 *
 * Print Module Parameters
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa
 */
void txDataQ_PrintModuleParams (TI_HANDLE hTxDataQ)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TI_UINT32      qIndex;
	TDataLinkQ     *pLinkQ;
	TI_UINT32      uHlid;

	WLAN_OS_REPORT(("--------- txDataQueue_printModuleParams ----------\n\n"));

	WLAN_OS_REPORT(("bStopNetStackTx = %d\n",pTxDataQ->bStopNetStackTx));
	WLAN_OS_REPORT(("bDataPortEnable = %d\n",pTxDataQ->bDataPortEnable));
	WLAN_OS_REPORT(("uNumQueues      = %d\n",pTxDataQ->uNumQueues));
	WLAN_OS_REPORT(("uNextQueId      = %d\n",pTxDataQ->uNextQueId));
	WLAN_OS_REPORT(("uContextId      = %d\n",pTxDataQ->uContextId));

	WLAN_OS_REPORT(("uLinkNotFound   = %d\n",pTxDataQ->uLinkNotFoundCount));
	WLAN_OS_REPORT(("uNoResources    = %d\n",pTxDataQ->uNoResourcesCount));
	WLAN_OS_REPORT(("aQueueMaxSize     %2d %2d %2d %2d\n", pTxDataQ->aQueueMaxSize[0], pTxDataQ->aQueueMaxSize[1], pTxDataQ->aQueueMaxSize[2], pTxDataQ->aQueueMaxSize[3]));
	WLAN_OS_REPORT(("aTxSendPaceThresh %2d %2d %2d %2d\n", pTxDataQ->aTxSendPaceThresh[0], pTxDataQ->aTxSendPaceThresh[1], pTxDataQ->aTxSendPaceThresh[2], pTxDataQ->aTxSendPaceThresh[3]));
	WLAN_OS_REPORT(("aQueueBusy        %2d %2d %2d %2d\n", pTxDataQ->aQueueBusy[0], pTxDataQ->aQueueBusy[1], pTxDataQ->aQueueBusy[2], pTxDataQ->aQueueBusy[3]));
	/*
	 * init all queues in all links
	*/
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		TI_UINT8 *pMacAddr = pTxDataQ->aLinkMac[uHlid].tMacAddr;
		pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */

		WLAN_OS_REPORT(("Link %3d ----------------------------------------\n", uHlid));
		WLAN_OS_REPORT(("  eType=%1d, bEnabled=%1d, bBusy=%1d\n", pLinkQ->eType, pLinkQ->bEnabled, pLinkQ->bBusy));
		if (!pLinkQ->bEnabled)
			continue;
		WLAN_OS_REPORT(("  MAC(valid=%01d) = %02x:%02x:%02x:%02x:%02x:%02x\n", pTxDataQ->aLinkMac[uHlid].uValid, pMacAddr[0], pMacAddr[1], pMacAddr[2], pMacAddr[3], pMacAddr[4], pMacAddr[5] ));
		WLAN_OS_REPORT(("  aNetStackQueueStopped  %2d %2d %2d %2d\n", pLinkQ->aNetStackQueueStopped[0], pLinkQ->aNetStackQueueStopped[1], pLinkQ->aNetStackQueueStopped[2], pLinkQ->aNetStackQueueStopped[3]));

		for (qIndex = 0; qIndex < MAX_NUM_OF_AC; qIndex++) {
			WLAN_OS_REPORT(("  Que %d: ", qIndex));
			que_Print (pLinkQ->aQueues[qIndex]);
		}
	}
}


/**
 * \fn     txDataQ_PrintQueueStatistics
 * \brief  Print queues statistics
 *
 * Print queues statistics
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa
 */
void txDataQ_PrintQueueStatistics (TI_HANDLE hTxDataQ)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ     *pLinkQ;
	TI_UINT32      uHlid;

	WLAN_OS_REPORT(("-------------- txDataQueue_printStatistics -------\n\n"));

	WLAN_OS_REPORT(("uClsfrMismatchCount      = %d\n",pTxDataQ->uClsfrMismatchCount));
	WLAN_OS_REPORT(("uTxSendPaceTimeoutsCount = %d\n",pTxDataQ->uTxSendPaceTimeoutsCount));

	/*
	 * init all queues in all links
	*/
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		TI_UINT8 *pMacAddr = pTxDataQ->aLinkMac[uHlid].tMacAddr;
		pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */

		WLAN_OS_REPORT(("Link %3d, Enabled=%1d--------------------------\n", uHlid, pLinkQ->bEnabled));
		if (!pLinkQ->bEnabled)
			continue;
		WLAN_OS_REPORT(("  MAC(valid=%01d) = %02x:%02x:%02x:%02x:%02x:%02x\n", pTxDataQ->aLinkMac[uHlid].uValid, pMacAddr[0], pMacAddr[1], pMacAddr[2], pMacAddr[3], pMacAddr[4], pMacAddr[5] ));
		WLAN_OS_REPORT(("  uEnqueuePacket: %8d %8d %8d %8d\n", pLinkQ->aQueueCounters[0].uEnqueuePacket, pLinkQ->aQueueCounters[1].uEnqueuePacket, pLinkQ->aQueueCounters[2].uEnqueuePacket, pLinkQ->aQueueCounters[3].uEnqueuePacket ));
		WLAN_OS_REPORT(("  uDequeuePacket: %8d %8d %8d %8d\n", pLinkQ->aQueueCounters[0].uDequeuePacket, pLinkQ->aQueueCounters[1].uDequeuePacket, pLinkQ->aQueueCounters[2].uDequeuePacket, pLinkQ->aQueueCounters[3].uDequeuePacket ));
		WLAN_OS_REPORT(("  uRequeuePacket: %8d %8d %8d %8d\n", pLinkQ->aQueueCounters[0].uRequeuePacket, pLinkQ->aQueueCounters[1].uRequeuePacket, pLinkQ->aQueueCounters[2].uRequeuePacket, pLinkQ->aQueueCounters[3].uRequeuePacket ));
		WLAN_OS_REPORT(("  uXmittedPacket: %8d %8d %8d %8d\n", pLinkQ->aQueueCounters[0].uXmittedPacket, pLinkQ->aQueueCounters[1].uXmittedPacket, pLinkQ->aQueueCounters[2].uXmittedPacket, pLinkQ->aQueueCounters[3].uXmittedPacket ));
		WLAN_OS_REPORT(("  uDroppedPacket: %8d %8d %8d %8d\n", pLinkQ->aQueueCounters[0].uDroppedPacket, pLinkQ->aQueueCounters[1].uDroppedPacket, pLinkQ->aQueueCounters[2].uDroppedPacket, pLinkQ->aQueueCounters[3].uDroppedPacket ));
	}

	/* Print data resouces counters */
	txDataQ_PrintResources(pTxDataQ);
}

/**
 * \fn     txDataQ_ResetQueueStatistics
 * \brief  Reset queues statistics
 *
 * Reset queues statistics
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa
 */
void txDataQ_ResetQueueStatistics (TI_HANDLE hTxDataQ)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	/*
	 * init all queues in all links
	*/
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */

		os_memoryZero(pTxDataQ->hOs, &pLinkQ->aQueueCounters, sizeof(pLinkQ->aQueueCounters));
	}
	pTxDataQ->uTxSendPaceTimeoutsCount = 0;
}

/**
 * \fn     txDataQ_PrintResources
 * \brief  Print resources counters per Link and per Ac
 *
 */
static void txDataQ_PrintResources (TTxDataQ *pTxDataQ)
{
	TDataResources *pDataRsrc = &pTxDataQ->tDataRsrc;
	TI_UINT32 uHlid;

	WLAN_OS_REPORT(("txDataQ_PrintResources ---------------------------------------------\n"));
	/* Print counters per AC */
	WLAN_OS_REPORT(("PktInUse_PerAc  = %8d %8d %8d %8d\n", pDataRsrc->uPktInUse_PerAc[0], pDataRsrc->uPktInUse_PerAc[1], pDataRsrc->uPktInUse_PerAc[2], pDataRsrc->uPktInUse_PerAc[3]));
	WLAN_OS_REPORT(("MinGuare_PerAc  = %8d %8d %8d %8d\n", pDataRsrc->uMinGuarantee_PerAc[0], pDataRsrc->uMinGuarantee_PerAc[1], pDataRsrc->uMinGuarantee_PerAc[2], pDataRsrc->uMinGuarantee_PerAc[3]));
	WLAN_OS_REPORT(("EffectivTotal_Ac= %d\n", pDataRsrc->uEffectiveTotal_Ac));

	/* Print counters per Link */
	WLAN_OS_REPORT(("PktInUse_PerLink= "));
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		WLAN_OS_REPORT(("%8d, ", pDataRsrc->uPktInUse_PerLink[uHlid]));
	}
	WLAN_OS_REPORT(("\n"));
	WLAN_OS_REPORT(("uMinGuarantee_PerLink = %d\n", pDataRsrc->uMinGuarantee_PerLink));
	WLAN_OS_REPORT(("uEffectiveTotal_Link  = %d\n", pDataRsrc->uEffectiveTotal_Link));
}


#endif /* TI_DBG */



/***************************************************************************
*                      INTERNAL  FUNCTIONS  IMPLEMENTATION				   *
****************************************************************************/


/**
 * \fn     txDataQ_RunScheduler
 * \brief  The module's Tx scheduler
 *
 * This function is the Data-Queue scheduler.
 * It selects a packet to transmit from the tx queues and sends it to the TxCtrl.
 * The queues are selected in a round-robin order.
 * The function is called by one of:
 *     txDataQ_Run()
 *     txDataQ_UpdateBusyMap()
 *     txDataQ_WakeAll()
 *
 * \note
 * \param  hTxDataQ - The object
 * \return void
 * \sa
 */
static void txDataQ_RunScheduler (TI_HANDLE hTxDataQ)
{
	TTxDataQ   *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TI_UINT32  uIdleAcCount = 0;  /* Count AC queues iterations without packet transmission (for exit criteria) */
	TI_UINT32  uIdleLinkCount = 0;  /* Count Links iterations without packet transmission (for exit criteria) */
	TI_UINT32  uQueId = pTxDataQ->uNextQueId;  /* The last iteration queue */
	EStatusXmit eStatus;  /* The return status of the txCtrl_xmitData function */
	TTxCtrlBlk *pPktCtrlBlk; /* Pointer to the packet to be dequeued and sent */
	TI_UINT32  uHlid = pTxDataQ->uNextHlid;  /* The last iteration link */
	TDataLinkQ *pLinkQ;
	TI_BOOL bStopScheduler = TI_FALSE;

	while(!bStopScheduler) {
		bStopScheduler = TI_TRUE;
		for(uIdleLinkCount = 0; uIdleLinkCount < WLANLINKS_MAX_LINKS; uIdleLinkCount++) {
			/* If the Data port is closed, indicate end of current packets burst and exit */
			if ( !pTxDataQ->bDataPortEnable ) {
				TWD_txXfer_EndOfBurst (pTxDataQ->hTWD);
				return;
			}
			if (uHlid == WLANLINKS_MAX_LINKS) {
				uHlid = 0;
			}

			pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */
			/* If the link is busy (AC is full), continue to next queue. */
			if ( (!pLinkQ->bEnabled) || (pLinkQ->bBusy)) {
				uQueId = 0;
				uHlid++;
				continue;
			}

			for(uIdleAcCount = 0; uIdleAcCount < pTxDataQ->uNumQueues; uIdleAcCount++) {
				if ( !pTxDataQ->bDataPortEnable ) {
					TWD_txXfer_EndOfBurst (pTxDataQ->hTWD);
					return;
				}

				if (uQueId == pTxDataQ->uNumQueues) {
					uQueId = 0;
				}
				if (pTxDataQ->aQueueBusy[uQueId]) {
					uQueId++;
					continue;
				}
				/* Dequeue a packet in a critical section */
				context_EnterCriticalSection (pTxDataQ->hContext);
				pPktCtrlBlk = (TTxCtrlBlk *) que_Dequeue (pLinkQ->aQueues[uQueId]);
				context_LeaveCriticalSection (pTxDataQ->hContext);

				/* If the queue was empty, continue to the next queue */
				if (pPktCtrlBlk == NULL) {
					if ((pTxDataQ->bStopNetStackTx) && pLinkQ->aNetStackQueueStopped[uQueId]) {
						pLinkQ->aNetStackQueueStopped[uQueId] = TI_FALSE;
						/*Resume the TX process as our date queues are empty*/
						wlanDrvIf_ResumeTx (pTxDataQ->hOs);
					}
					uQueId++;
					continue;
				}

#ifdef TI_DBG
				pLinkQ->aQueueCounters[uQueId].uDequeuePacket++;
#endif /* TI_DBG */
				/* Send the packet */
				eStatus = txCtrl_XmitData (pTxDataQ->hTxCtrl, pPktCtrlBlk);

				/*
				 * If the return status is busy it means that the packet was not sent
				 *   so we need to requeue it for future try.
				 */
				if(eStatus == STATUS_XMIT_BUSY) {
					TI_STATUS eQueStatus;

					/* Requeue the packet in a critical section */
					context_EnterCriticalSection (pTxDataQ->hContext);
					eQueStatus = que_Requeue (pLinkQ->aQueues[uQueId], (TI_HANDLE)pPktCtrlBlk);
					if (eQueStatus != TI_OK) {
						/* If the packet can't be queued drop it */
						/* Note: may happen only if this thread was preempted between the
						   dequeue and requeue and new packets were inserted into this quque */
						txCtrl_FreePacket (pTxDataQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
#ifdef TI_DBG
						pLinkQ->aQueueCounters[uQueId].uDroppedPacket++;
#endif /* TI_DBG */
					}
					context_LeaveCriticalSection (pTxDataQ->hContext);

#ifdef TI_DBG
					pLinkQ->aQueueCounters[uQueId].uRequeuePacket++;
#endif /* TI_DBG */
					uQueId++;
					continue;
				} else if( eStatus == STATUS_XMIT_SUCCESS ) {

					bStopScheduler = TI_FALSE;
					/* Save the next que should be proceed by scheduler */
					pTxDataQ->uNextQueId = uQueId + 1;
					pTxDataQ->uNextHlid = uHlid + 1;
				}
#ifdef TI_DBG
				pLinkQ->aQueueCounters[uQueId].uXmittedPacket++;
#endif /* TI_DBG */

				uQueId++;
			} /* for(uIdleAcCount = 0; uIdleAcCount < pTxDataQ->uNumQueues; uIdleAcCount++)*/
			uHlid++;

		}  /*for(uIdleLinkCount = 0; uIdleLinkCount < WLANLINKS_MAX_LINKS; uIdleLinkCount++)*/
	}  /*while(!bStopScheduler)*/
	TWD_txXfer_EndOfBurst (pTxDataQ->hTWD);

}

/**
 * \fn     txDataQ_UpdateLinksBusyState
 * \brief  Update links' busy state
 *
 * Update the Links Mode to Busy according to the input LinkBitMap.
*               Each Link bit that is set indicates that the related Link is Busy.
*
 * \note
 * \param  hTxDataQ - The object
 * \param  uLinkBitMap   - The changed TIDs busy bitmap
 * \return void
 * \sa
 */
static void txDataQ_UpdateLinksBusyState (TTxDataQ *pTxDataQ, TI_UINT32 uLinkBitMap)
{
	TI_UINT32 uHlid;

	/* Go over the LinkBitMap and update the related link busy state */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++, uLinkBitMap >>= 1) {
		if (uLinkBitMap & 0x1) { /* this Tid is busy */
			pTxDataQ->aDataLinkQ[uHlid].bBusy = TI_TRUE;
		} else {
			pTxDataQ->aDataLinkQ[uHlid].bBusy = TI_FALSE;
		}
	}
}

/**
 * \fn     txDataQ_UpdateQueuesBusyState
 * \brief  Update queues' busy state
 *
 * Update the Queues Mode to Busy according to the input TidBitMap.
*               Each Tid that is set indicates that the related Queue is Busy.
*
 * \note
 * \param  hTxDataQ - The object
 * \param  uTidBitMap   - The changed TIDs busy bitmap
 * \return void
 * \sa
 */
static void txDataQ_UpdateQueuesBusyState (TTxDataQ *pTxDataQ, TI_UINT32 uTidBitMap)
{
	TI_UINT32 uTidIdx;

	/* Go over the TidBitMap and update the related queue busy state */
	for (uTidIdx = 0; uTidIdx < MAX_NUM_OF_802_1d_TAGS; uTidIdx++, uTidBitMap >>= 1) {
		if (uTidBitMap & 0x1) { /* this Tid is busy */
			pTxDataQ->aQueueBusy[aTidToQueueTable[uTidIdx]] = TI_TRUE;
		} else {
			pTxDataQ->aQueueBusy[aTidToQueueTable[uTidIdx]] = TI_FALSE;
		}
	}
}


/*
 * \brief   Handle Tx-Send-Pacing timeout.
 *
 * \param  hTxDataQ        - Module handle
 * \param  bTwdInitOccured - Indicate if TWD restart (recovery) occured
 * \return void
 *
 * \par Description
 * Call the Tx scheduler to handle the queued packets.
 *
 * \sa
 */
static void txDataQ_TxSendPaceTimeout (TI_HANDLE hTxDataQ, TI_BOOL bTwdInitOccured)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;

	pTxDataQ->uTxSendPaceTimeoutsCount++;

	txDataQ_RunScheduler (hTxDataQ);
}


/**
 * \fn     TxDataQ_SetEncryptFlag
 * \brief  Update link encrypted bit
 *
 * * \note
 * \param  pTxdataQ   - The module's object
 * \param  uHlid - link id
 * \return void
 * \sa
 */

void TxDataQ_SetEncryptFlag(TI_HANDLE hTxDataQ, TI_UINT32  uHlid,int flag)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ;


	pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */
	pLinkQ->bEncrypt = flag;
}

/**
 * \fn     TxDataQ_setEncryptionFieldSizes
 * \brief  update encrypted size
 *
 * * \note
 * \param  pTxdataQ   - The module's object
 * \param  uHlid - link id
 * \return void
 * \sa
 */
void TxDataQ_setEncryptionFieldSizes(TI_HANDLE hTxDataQ, TI_UINT32  uHlid,TI_UINT8 encryptionFieldSize)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ;

	pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */
	pLinkQ->cEncryptSize = encryptionFieldSize;

}

/**
 * \fn     TxDataQ_getEncryptionFieldSizes
 * \brief  update encrypted size
 *
 * * \note
 * \param  pTxdataQ   - The module's object
 * \param  uHlid - link id
 * \return void
 * \sa
 */
TI_UINT8 TxDataQ_getEncryptionFieldSizes(TI_HANDLE hTxDataQ, TI_UINT32  uHlid)
{
	TTxDataQ *pTxDataQ = (TTxDataQ *)hTxDataQ;
	TDataLinkQ *pLinkQ;

	pLinkQ = &pTxDataQ->aDataLinkQ[uHlid]; /* Link queues */
	return pLinkQ->cEncryptSize;
}



