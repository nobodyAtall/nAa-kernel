/*
 * txMgmtQueue.c
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



/** \file   txMgmtQueue.c
 *  \brief  The Tx Mgmt Queues module.
 *
 *	DESCRIPTION:
 *	============
 *	The Management-Queues module is responsible for the following tasks:
 *		1.	Queue the driver generated Tx packets, including management,
 *			EAPOL and null packets until they are transmitted.
 *			The management packets are buffered in the management-queue,
 *			and the others in the EAPOL-queue.
 *		2.	Maintain a state machine that follows the queues state and
 *			the connection states and enables specific transmission types
 *			accordingly (e.g. only management).
 *		3.	Gain access to the Tx path when the management queues are not
 *			empty, and return the access to the data queues when the
 *			management queues are empty.
 *		4.	Schedule packets transmission with strict priority of the
 *			management queue over the EAPOL queue, and according to the
 *			backpressure controls from the Port (all queues) and from the
 *			Tx-Ctrl (per queue).
 *
 *  \see    txMgmtQueue.h
 */

#define __FILE_ID__  FILE_ID_61
#include "tidef.h"
#include "paramOut.h"
#include "osApi.h"
#include "TWDriver.h"
#include "DataCtrl_Api.h"
#include "report.h"
#include "queue.h"
#include "context.h"
#include "DrvMainModules.h"
#include "Ethernet.h"
#include "APExternalIf.h"
#include "wlanLinks.h"

#define MGMT_QUEUES_TID		MAX_USER_PRIORITY

typedef enum {
	QUEUE_TYPE_MGMT,	/* Mgmt-queue  - high-priority, for mgmt packets only. */
	QUEUE_TYPE_EAPOL,	/* EAPOL-queue - low-priority, for other internal packets (EAPOL, NULL, IAPP). */
	NUM_OF_MGMT_QUEUES
} EMgmtQueueTypes;

/* State-Machine Events */
typedef enum {
	SM_EVENT_CLOSE,			/* All Tx types should be closed. */
	SM_EVENT_MGMT,			/* Allow only mgmt packets. */
	SM_EVENT_EAPOL,			/* Allow mgmt and EAPOL packets. */
	SM_EVENT_OPEN,			/* Allow all packets. */
	SM_EVENT_QUEUES_EMPTY,	/* Mgmt-aQueues are now both empty. */
	SM_EVENT_QUEUES_NOT_EMPTY /* At least one of the Mgmt-aQueues is now not empty. */
} ESmEvent;

/* State-Machine States */
typedef enum {
	SM_STATE_CLOSE,			/* All Tx path is closed. */
	SM_STATE_MGMT,			/* Only mgmt Tx is permitted. */
	SM_STATE_EAPOL,			/* Only mgmt and EAPOL Tx is permitted. */
	SM_STATE_OPEN_MGMT,		/* All Tx permitted and Mgmt aQueues are currently active (date disabled). */
	SM_STATE_OPEN_DATA		/* All Tx permitted and Data aQueues are currently active (mgmt disabled). */
} ESmState;

/* State-Machine Actions */
typedef enum {
	SM_ACTION_NULL,
	SM_ACTION_ENABLE_DATA,
	SM_ACTION_ENABLE_MGMT,
	SM_ACTION_RUN_SCHEDULER,
	SM_ACTION_STOP_ALL
} ESmAction;

/* TI_TRUE if both aQueues are empty. */
#define ARE_LINK_MGMT_QUEUES_EMPTY(aQueues)	( (que_Size(aQueues[QUEUE_TYPE_MGMT] ) == 0)  &&  \
											  (que_Size(aQueues[QUEUE_TYPE_EAPOL]) == 0) )

typedef struct {
	TI_UINT32 aEnqueuePackets[NUM_OF_MGMT_QUEUES];
	TI_UINT32 aDequeuePackets[NUM_OF_MGMT_QUEUES];
	TI_UINT32 aRequeuePackets[NUM_OF_MGMT_QUEUES];
	TI_UINT32 aDroppedPackets[NUM_OF_MGMT_QUEUES];
	TI_UINT32 aXmittedPackets[NUM_OF_MGMT_QUEUES];
	TI_UINT32 uNoResourcesCount;
} TDbgCount;

#define LINK_MGMT_QUEUES_DEPTH MGMT_QUEUES_DEPTH
/* The LinkQ object. */
typedef struct {
	EWlanLinkType   eType;
	TI_BOOL         bBusy;                         /* Link is busy. */
	TI_BOOL         bEnabled;                      /* Link is enabled */
	TI_BOOL         bEncrypt;                      /* encrypt data packet */
	ESmState        eState;                        /* The current state of the Link SM. */
	ETxConnState	eTxConnState;                  /* See typedef in module API. */
	TI_BOOL         bSendEvent_NotEmpty;           /* Used to sign if to send QUEUE_NOT_EMPTY event when switching to driver context */
	TI_HANDLE   	aQueues[NUM_OF_MGMT_QUEUES];   /* The mgmt-aQueues handles. */
	TI_BOOL         aQenabled[NUM_OF_MGMT_QUEUES]; /* Queue is enabled by the SM. */
	TDbgCount		tDbgCounters; /* Save Tx statistics per mgmt-queue. */
} TMgmtLinkQ;

/* The module object. */
typedef struct {
	/* Handles */
	TI_HANDLE		hOs;
	TI_HANDLE		hReport;
	TI_HANDLE 		hTxCtrl;
	TI_HANDLE 		hTxDataQ;
	TI_HANDLE 		hTxPort;
	TI_HANDLE 		hContext;
	TI_HANDLE       hTWD;

	TI_BOOL			bMgmtPortEnable;/* Port open for mgmt-aQueues or not. */
	TI_UINT32       uContextId;     /* ID allocated to this module on registration to context module */

	TMgmtLinkQ      aMgmtLinkQ[WLANLINKS_MAX_LINKS];   /* Link queues handles. */
	TI_BOOL         aMgmtAcBusy;                       /* Mgmt AC is busy. */
	TI_UINT32       uGlobalHlid;                       /* Generic link id */
	TI_UINT32       uLastHlid;                         /* Last scheduler visited HLID */

} TTxMgmtQ;

/* The module internal functions */
static void mgmtQueuesSM (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uHlid, ESmEvent smEvent);
static void runSchedulerNotFromSm (TTxMgmtQ *pTxMgmtQ);
static void runScheduler (TTxMgmtQ *pTxMgmtQ);
static void updateLinksBusyMap (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uLinkBitMap);
static void updateQueuesBusyMap (TTxMgmtQ *pTxMgmtQ, TI_UINT32 tidBitMap);
static void	txMgmtQ_DisableLink (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uHlid);
static void	txMgmtQ_EnableLink (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uHlid);

/*******************************************************************************
*                       PUBLIC  FUNCTIONS  IMPLEMENTATION					   *
********************************************************************************/


/**
 * \fn     txMgmtQ_Create
 * \brief  Create the module and its queues
 *
 * Create the Tx Mgmt Queue module and its queues.
 *
 * \note
 * \param  hOs - Handle to the Os Abstraction Layer
 * \return Handle to the allocated Tx Mgmt Queue module (NULL if failed)
 * \sa
 */
TI_HANDLE txMgmtQ_Create (TI_HANDLE hOs)
{
	TTxMgmtQ *pTxMgmtQ;

	/* allocate TxMgmtQueue module */
	pTxMgmtQ = os_memoryAlloc (hOs, (sizeof(TTxMgmtQ)));

	if(!pTxMgmtQ) {
		WLAN_OS_REPORT(("Error allocating the TxMgmtQueue Module\n"));
		return NULL;
	}

	/* Reset TxMgmtQueue module */
	os_memoryZero (hOs, pTxMgmtQ, (sizeof(TTxMgmtQ)));

	return (TI_HANDLE)pTxMgmtQ;
}


/**
 * \fn     txMgmtQ_Init
 * \brief  Configure module with default settings
*
 * Get other modules handles.
 * Init the Tx Mgmt queues.
 * Register as the context-engine client.
 *
 * \note
 * \param  pStadHandles  - The driver modules handles
 * \return void
 * \sa
 */
void txMgmtQ_Init (TStadHandlesList *pStadHandles)
{
	TTxMgmtQ  *pTxMgmtQ = (TTxMgmtQ *)(pStadHandles->hTxMgmtQ);
	TI_UINT32  uNodeHeaderOffset = TI_FIELD_OFFSET(TTxnStruct, tTxnQNode);
	int        uQueId;
	TMgmtLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	/* configure modules handles */
	pTxMgmtQ->hOs		= pStadHandles->hOs;
	pTxMgmtQ->hReport	= pStadHandles->hReport;
	pTxMgmtQ->hTxCtrl	= pStadHandles->hTxCtrl;
	pTxMgmtQ->hTxDataQ	= pStadHandles->hTxDataQ;
	pTxMgmtQ->hTxPort	= pStadHandles->hTxPort;
	pTxMgmtQ->hContext	= pStadHandles->hContext;
	pTxMgmtQ->hTWD	    = pStadHandles->hTWD;

	pTxMgmtQ->bMgmtPortEnable = TI_TRUE;	/* Port Default status is open (data-queues are disabled). */
	pTxMgmtQ->aMgmtAcBusy = TI_FALSE;       /* Init busy flag per for Mgmt ccess category (same for MGMT and EAPOL)  */

	/*
	 * init all queues in all links
	 */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
		pLinkQ->eTxConnState = TX_CONN_STATE_CLOSE;
		pLinkQ->eState = SM_STATE_CLOSE; /* SM default state is CLOSE. */
		pLinkQ->bSendEvent_NotEmpty = TI_FALSE;
		pLinkQ->bBusy = TI_FALSE; /* default is not busy */
		pLinkQ->bEnabled = TI_FALSE; /* default is not enabled */

		for (uQueId = 0; uQueId < NUM_OF_MGMT_QUEUES; uQueId++) {
			pLinkQ->aQueues[uQueId] = que_Create (pTxMgmtQ->hOs,
			                                      pTxMgmtQ->hReport,
			                                      LINK_MGMT_QUEUES_DEPTH,
			                                      uNodeHeaderOffset);

			/* If any Queues' allocation failed, print error, free TxMgmtQueue module and exit */
			if (pLinkQ->aQueues[uQueId] == NULL) {
				WLAN_OS_REPORT(("Failed to create queue for link %d\n", uHlid));
				os_memoryFree (pTxMgmtQ->hOs, pTxMgmtQ, sizeof(TTxMgmtQ));
				return;
			}

			pLinkQ->aQenabled[uQueId] = TI_FALSE; /* Queue is disabled */
		}
	}
	pTxMgmtQ->uLastHlid = 0; /* scheduler starts from first link */

	/* Register to the context engine and get the client ID */
	pTxMgmtQ->uContextId = context_RegisterClient (pTxMgmtQ->hContext,
	                       txMgmtQ_QueuesNotEmpty,
	                       (TI_HANDLE)pTxMgmtQ,
	                       TI_TRUE,
	                       "TX_MGMT",
	                       sizeof("TX_MGMT"));

}


/**
 * \fn     txMgmtQ_Destroy
 * \brief  Destroy the module and its queues
 *
 * Clear and destroy the queues and then destroy the module object.
 *
 * \note
 * \param  hTxMgmtQ - The module's object
 * \return TI_OK - Unload succesfull, TI_NOK - Unload unsuccesfull
 * \sa
 */
TI_STATUS txMgmtQ_Destroy (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ  *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TI_STATUS  eStatus = TI_OK;
	int        uQueId;
	TMgmtLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	/* Dequeue and free all queued packets */
	txMgmtQ_ClearQueues (hTxMgmtQ);

	/*
	 * init all queues in all links
	 */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */

		for (uQueId = 0 ; uQueId < NUM_OF_MGMT_QUEUES ; uQueId++) {
			if (que_Destroy(pLinkQ->aQueues[uQueId]) != TI_OK) {
				eStatus = TI_NOK;
			}
		}
	}

	/* free Tx Mgmt Queue Module */
	os_memoryFree (pTxMgmtQ->hOs, pTxMgmtQ, sizeof(TTxMgmtQ));

	return eStatus;
}


/**
 * \fn     txMgmtQ_ClearQueues
 * \brief  Clear all queues
 *
 * Dequeue and free all queued packets.
 *
 * \note
 * \param  hTxMgmtQ - The object
 * \return void
 * \sa
 */
void txMgmtQ_ClearQueues (TI_HANDLE hTxMgmtQ)
{
	TI_UINT32  uHlid;
	/*
	 * flush all queues in all links
	 */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		TxMgmtQ_FlushLinkQueues(hTxMgmtQ, uHlid);
	}
}


/**
 * \fn     txMgmtQ_Xmit
 * \brief  Insert non-data packet for transmission
 *
 * This function is used by the driver applications to send Tx packets other than the
 *   regular data traffic, including the following packet types:
*				- Management
*				- EAPOL
*				- NULL
*				- IAPP
 * The managment packets are enqueued to the Mgmt-queue and the others to the Eapol-queue.
 * EAPOL packets may be inserted from the network stack context, so it requires switching
 *   to the driver's context (after the packet is enqueued).
 * If the selected queue was empty before the packet insertion, the SM is called
 *   with QUEUES_NOT_EMPTY event (in case of external context, only after the context switch).
 *
 * \note
 * \param  hTxMgmtQ         - The module's object
 * \param  pPktCtrlBlk      - Pointer to the packet CtrlBlk
 * \param  bExternalContext - Indicates if called from non-driver context
 * \return TI_OK - if the packet was queued, TI_NOK - if the packet was dropped.
 * \sa     txMgmtQ_QueuesNotEmpty
 */
TI_STATUS txMgmtQ_Xmit (TI_HANDLE hTxMgmtQ, TTxCtrlBlk *pPktCtrlBlk, TI_BOOL bExternalContext)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TI_STATUS eStatus;
	TI_UINT32 uQueId;
	TI_UINT32 uQueSize;
	TI_UINT32 uHlid;
	TMgmtLinkQ *pLinkQ;

	/* Find link id by destination MAC address, if not found use global link id */
	if (pPktCtrlBlk->tTxPktParams.uPktType == TX_PKT_TYPE_MGMT) {
		/* MGMT packet, use destination MAC address from WLAN header, aBuf[0] is the WLAN header */
		if ((txDataQ_LinkMacFind( pTxMgmtQ->hTxDataQ, &uHlid ,((dot11_header_t *)(pPktCtrlBlk->tTxnStruct.aBuf[0]))->address1 )) != TI_OK) {
			uHlid = pTxMgmtQ->uGlobalHlid;
		}
	} else {
		/* EAPOL packet, use destination MAC address from ETHERNET header, aBuf[0] is the ETHERNET header */
		if ((txDataQ_LinkMacFind( pTxMgmtQ->hTxDataQ, &uHlid, ((TEthernetHeader *)(pPktCtrlBlk->tTxnStruct.aBuf[0]))->dst)) != TI_OK) {
			uHlid = pTxMgmtQ->uGlobalHlid;
		}
	}
	pPktCtrlBlk->tTxDescriptor.hlid = uHlid;

	pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */

	/* Always set highest TID for mgmt-queues packets. */
	pPktCtrlBlk->tTxDescriptor.tid = MGMT_QUEUES_TID;

	if ((pLinkQ->bEncrypt)&& (pPktCtrlBlk->tTxPktParams.uPktType == TX_PKT_TYPE_EAPOL)) {
		SET_PKT_TYPE_ENCRYPT(pPktCtrlBlk);
	}

	/* Select queue asccording to the packet type */
	uQueId = (pPktCtrlBlk->tTxPktParams.uPktType == TX_PKT_TYPE_MGMT) ? QUEUE_TYPE_MGMT : QUEUE_TYPE_EAPOL ;

	/* Enter critical section to protect queue access */
	context_EnterCriticalSection (pTxMgmtQ->hContext);

	/* Check resources per LINK and per MGMT AC (VOICE)*/
	if (txDataQ_AllocCheckResources( pTxMgmtQ->hTxDataQ, pPktCtrlBlk) != TI_OK) {
		pLinkQ->tDbgCounters.aDroppedPackets[uQueId]++;
		pLinkQ->tDbgCounters.uNoResourcesCount++;
		/* Leave critical section */
		context_LeaveCriticalSection (pTxMgmtQ->hContext);

		/* If the packet can't be queued drop it */
		/* !!! This call should be out of the critical section */
		txCtrl_FreePacket (pTxMgmtQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
		return TI_NOK;
	}

	/* Enqueue the packet in the appropriate Queue */
	eStatus = que_Enqueue (pLinkQ->aQueues[uQueId], (TI_HANDLE)pPktCtrlBlk);

	/* Get number of packets in current queue */
	uQueSize = que_Size (pLinkQ->aQueues[uQueId]);

	/* Leave critical section */
	context_LeaveCriticalSection (pTxMgmtQ->hContext);

	/* If packet enqueued successfully */
	if (eStatus == TI_OK) {
		pLinkQ->tDbgCounters.aEnqueuePackets[uQueId]++;

		/* If selected queue was empty before packet insertion */
		if (uQueSize == 1) {
			/* If called from external context (EAPOL from network), request switch to the driver's context. */
			if (bExternalContext) {
				/* Set bSendEvent_NotEmpty flag to use in driver context */
				pLinkQ->bSendEvent_NotEmpty = TI_TRUE;
				context_RequestSchedule (pTxMgmtQ->hContext, pTxMgmtQ->uContextId);
			}

			/* If already in the driver's context, call the SM with QUEUES_NOT_EMPTY event. */
			else {
				mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_QUEUES_NOT_EMPTY);
			}
		}
	}

	else {
		/* If the packet can't be queued so drop it */
		txCtrl_FreePacket (pTxMgmtQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
		pLinkQ->tDbgCounters.aDroppedPackets[uQueId]++;
	}

	return eStatus;
}


/**
 * \fn     txMgmtQ_QueuesNotEmpty
 * \brief  Context-Engine Callback
 *
 * Context-Engine Callback for processing queues in driver's context.
 * Called after driver's context scheduling was requested in txMgmtQ_Xmit().
 * Calls the SM with QUEUES_NOT_EMPTY event.
 *
 * \note
 * \param  hTxMgmtQ - The module's object
 * \return void
 * \sa     txMgmtQ_Xmit
 */
void txMgmtQ_QueuesNotEmpty (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ  *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TI_UINT32 uHlid;

	/* Call the SM with QUEUES_NOT_EMPTY event. */
	for (uHlid=0; uHlid<WLANLINKS_MAX_LINKS; uHlid++) {
		/* Call every link that is not in CLOSE state */
		if (pTxMgmtQ->aMgmtLinkQ[uHlid].bSendEvent_NotEmpty) {
			/* reset bSendEvent_NotEmpty flag to use in driver context */
			pTxMgmtQ->aMgmtLinkQ[uHlid].bSendEvent_NotEmpty = TI_FALSE;

			/* handle the event in the link state machine */
			mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_QUEUES_NOT_EMPTY);
		}
	}
}


/**
 * \fn     txMgmtQ_StopLink
 * \brief  Stop all queues transmission for the link
 *
 * \param  hTxMgmtQ   - The module's object
 * \return void
 */
void txMgmtQ_StopLink (TI_HANDLE hTxMgmtQ, TI_UINT32 uHlid)
{
	TTxMgmtQ  *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid];

	pLinkQ->bBusy = TI_TRUE;
}

/**
 * \fn     txMgmtQ_StopQueue
 * \brief  Context-Engine Callback
 *
 * This function is called by the txCtrl_xmitMgmt() if the queue's backpressure indication
 *   is set. It sets the internal queue's Busy indication.
 *
 * \note
 * \param  hTxMgmtQ   - The module's object
 * \param  uTidBitMap - The busy TIDs bitmap
 * \return void
 * \sa     txMgmtQ_UpdateBusyMap
 */
void txMgmtQ_StopQueue (TI_HANDLE hTxMgmtQ, TI_UINT32 uTidBitMap)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;

	/* Update the Queue(s) busy mode */
	updateQueuesBusyMap (pTxMgmtQ, uTidBitMap);
}


/**
 * \fn     txMgmtQ_UpdateBusyMap
 * \brief  Update the queues busy map
 *
 * This function is called by the txCtrl if the backpressure map per TID is changed.
 * This could be as a result of Tx-Complete, admission change or association.
 * The function modifies the internal queues Busy indication and calls the scheduler.
 *
 * \note
 * \param  hTxMgmtQ   - The module's object
 * \param  uTidBitMap - The busy TIDs bitmap
 * \param  uLinkBitMap - The busy Links bitmap
 * \return void
 * \sa     txMgmtQ_StopQueue
 */
void txMgmtQ_UpdateBusyMap (TI_HANDLE hTxMgmtQ, TI_UINT32 uTidBitMap, TI_UINT32 uLinkBitMap)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;

	/* Update the Queue(s) busy map. */
	updateLinksBusyMap (pTxMgmtQ, uLinkBitMap);

	/* Update the Link(s) busy map. */
	updateQueuesBusyMap (pTxMgmtQ, uTidBitMap);

	/* If the queues are not empty, run the scheduler and if they become empty update the SM. */
	runSchedulerNotFromSm (pTxMgmtQ);
}

/**
 * \fn     txMgmtQ_SuspendTx
 * \brief  Stop all Tx
 *
 * \param  hTxMgmtQ   - The module's object
 */
void txMgmtQ_SuspendTx (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;

	txMgmtQ_StopAll(hTxMgmtQ);
	txDataQ_StopAll(pTxMgmtQ->hTxDataQ);
}

/**
 * \fn     txMgmtQ_ResumeTx
 * \brief  Resume all Tx
 *
 * \param  hTxMgmtQ   - The module's object
 */
void txMgmtQ_ResumeTx (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;

	txMgmtQ_WakeAll(hTxMgmtQ);
	txDataQ_WakeAll(pTxMgmtQ->hTxDataQ);
}

/**
 * \fn     txMgmtQ_StopAll
 * \brief  enable all queues transmission
 *
 * This function is called by the Tx-Port when the whole Mgmt-queue is stopped.
 * It clears the common queues enable indication.
 *
 * \note
 * \param  hTxMgmtQ   - The module's object
 * \return void
 * \sa     txMgmtQ_WakeAll
 */
void txMgmtQ_StopAll (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;

	/* Disable the Mgmt Tx port */
	pTxMgmtQ->bMgmtPortEnable = TI_FALSE;
}

/**
 * \fn     txMgmtQ_WakeAll
 * \brief  Enable all queues transmission
 *
 * This function is called by the Tx-Port when the whole Mgmt-queue is enabled.
 * It sets the common queues enable indication and calls the scheduler.
 *
 * \note
 * \param  hTxMgmtQ   - The module's object
 * \return void
 * \sa     txMgmtQ_WakeAll
 */
void txMgmtQ_WakeAll (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;

	/* Enable the Mgmt Tx port */
	pTxMgmtQ->bMgmtPortEnable = TI_TRUE;

	/* If the queues are not empty, run the scheduler and if they become empty update the SM. */
	runSchedulerNotFromSm (pTxMgmtQ);
}

/**
 * \fn     txMgmtQ_DisableLink
 * \brief  Disable all queues transmission for the link
 *
 * \param  hTxMgmtQ   - The module's object
 * \return void
 */
static void txMgmtQ_DisableLink (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uHlid)
{
	TMgmtLinkQ *pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid];

	pLinkQ->bEnabled = TI_FALSE;
}

/**
 * \fn     txMgmtQ_EnableLink
 * \brief  Enable all queues transmission for the link
 *
 * \param  hTxMgmtQ   - The module's object
 * \return void
 * \sa     txMgmtQ_DisableLink
 */
static void txMgmtQ_EnableLink (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uHlid)
{
	TMgmtLinkQ *pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid];

	pLinkQ->bEnabled = TI_TRUE;

	/* If the queues are not empty, run the scheduler and if they become empty update the SM. */
	runSchedulerNotFromSm (pTxMgmtQ);
}


/**
 * \fn     txMgmtQ_SetConnState
 * \brief  Enable all queues transmission
 *
 * Called by the connection SM and updates the connection state from Tx perspective
 *   (i.e. which packet types are permitted).
*               Calls the local SM to handle this state change.
*
 * \note
 * \param  hTxMgmtQ     - The module's object
 * \param  eTxConnState - The new Tx connection state
 * \return void
 * \sa     mgmtQueuesSM
 */
void txMgmtQ_SetConnState (TI_HANDLE hTxMgmtQ, ETxConnState eTxConnState)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ;
	TI_UINT32 uHlid = 0; /* TODO[ilanb]: move to parameter, this API is for STA */

	pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */

	pLinkQ->eTxConnState = eTxConnState;

	/* Call the SM with the current event. */
	switch (eTxConnState) {
	case TX_CONN_STATE_CLOSE:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_CLOSE);
		break;
	case TX_CONN_STATE_MGMT:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_MGMT);
		break;
	case TX_CONN_STATE_EAPOL:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_EAPOL);
		break;
	case TX_CONN_STATE_OPEN:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_OPEN);
		break;

	default: {}
	}
}

/**
 * \fn     txMgmtQ_SetLinkType
 * \brief  set link type
 *
 * \note
 * \param  hTxMgmtQ     - The module's object
 * \param  uHlid        - link id
 * \param  eLinkType    - link type
 * \return void
 */
void txMgmtQ_SetLinkType (TI_HANDLE hTxMgmtQ, TI_UINT32 uHlid, EWlanLinkType eLinkType)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ;

	pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid];
	pLinkQ->eType = eLinkType;


	/* save global link id */
	if (pLinkQ->eType == WLANLINK_TYPE_GLOBAL) {
		pTxMgmtQ->uGlobalHlid = uHlid;
	}

	txDataQ_SetLinkType(pTxMgmtQ->hTxDataQ, uHlid, eLinkType);
}

/**
 * \fn     txMgmtQ_SetLinkState
 * \brief  set link state (enable/not)
 *
 * \note
 * \param  hTxMgmtQ     - The module's object
 * \param  uHlid        - link id
 * \param  eTxConnState - The new Tx connection state
 * \return void
 * \sa     mgmtQueuesSM
 */
TI_STATUS txMgmtQ_SetLinkState (TI_HANDLE hTxMgmtQ, TI_UINT32 uHlid, ETxConnState eTxConnState)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ;

	pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
	pLinkQ->eTxConnState = eTxConnState;


	/* Call the SM with the current event. */
	switch (eTxConnState) {
	case TX_CONN_STATE_CLOSE:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_CLOSE);
		break;
	case TX_CONN_STATE_MGMT:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_MGMT);
		break;
	case TX_CONN_STATE_EAPOL:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_EAPOL);
		break;
	case TX_CONN_STATE_OPEN:
		mgmtQueuesSM(pTxMgmtQ, uHlid, SM_EVENT_OPEN);
		break;

	default: {}
	}

	return TI_OK;
}


/*******************************************************************************
*                       INTERNAL  FUNCTIONS  IMPLEMENTATION					   *
********************************************************************************/


/**
 * \fn     mgmtQueuesSM
 * \brief  The module state-machine (static function)
 *
 * The SM follows the system management states (see ETxConnState) and the Mgmt queues
 *   status (empty or not), and contorls the Tx queues flow accordingly (mgmt and data queues).
 * For detailed explanation, see the Tx-Path LLD document!
 *
 * \note   To avoid recursion issues, all SM actions are done at the end of the function,
 *            since some of them may invoke the SM again.
 * \param  pTxMgmtQ - The module's object
 * \param  eSmEvent - The event to act upon
 * \return void
 * \sa     txMgmtQ_SetConnState
 */
static void mgmtQueuesSM (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uHlid, ESmEvent eSmEvent)
{
	TMgmtLinkQ *pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
	ESmState  ePrevState = pLinkQ->eState;
	ESmAction eSmAction  = SM_ACTION_NULL;


	switch(eSmEvent) {
	case SM_EVENT_CLOSE:
		/*
		 * Tx link is closed (expected in any state), so disable both mgmt queues
		 *   and if data-queues are active disable them.
		 */
		pLinkQ->eState = SM_STATE_CLOSE;
		pLinkQ->aQenabled[QUEUE_TYPE_MGMT]  = TI_FALSE;
		pLinkQ->aQenabled[QUEUE_TYPE_EAPOL] = TI_FALSE;
		/*
		if (ePrevState == SM_STATE_OPEN_DATA)
			eSmAction = SM_ACTION_ENABLE_MGMT;
		*/
		eSmAction = SM_ACTION_STOP_ALL;
		break;

	case SM_EVENT_MGMT:
		/*
		 * Only Mgmt packets are permitted (expected from any state):
		 *   - Enable the mgmt queue and disable the Eapol queue.
		 *   - If data-queues are active disable them (this will run the scheduler).
		 *   - Else run the scheduler (to send mgmt packets if waiting).
		 */
		pLinkQ->eState = SM_STATE_MGMT;
		pLinkQ->aQenabled[QUEUE_TYPE_MGMT]  = TI_TRUE;
		pLinkQ->aQenabled[QUEUE_TYPE_EAPOL] = TI_FALSE;
		if (ePrevState == SM_STATE_OPEN_DATA)
			eSmAction = SM_ACTION_ENABLE_MGMT;
		else
			eSmAction = SM_ACTION_RUN_SCHEDULER;
		break;

	case SM_EVENT_EAPOL:
		/*
		 * EAPOL packets are also permitted (expected in MGMT or CLOSE state), so enable the
		 *   EAPOL queue and run the scheduler (to send packets from EAPOL queue if waiting).
		 */
		pLinkQ->eState = SM_STATE_EAPOL;
		pLinkQ->aQenabled[QUEUE_TYPE_MGMT]  = TI_TRUE;
		pLinkQ->aQenabled[QUEUE_TYPE_EAPOL] = TI_TRUE;
		eSmAction = SM_ACTION_RUN_SCHEDULER;
		break;

	case SM_EVENT_OPEN:
		/*
		 * All packets are now permitted (expected in EAPOL state), so if the mgmt-queues
		 *   are empty disable them and enable the data queues.
		 */
		if ( ARE_LINK_MGMT_QUEUES_EMPTY(pLinkQ->aQueues) ) {
			pLinkQ->eState = SM_STATE_OPEN_DATA;
			pLinkQ->aQenabled[QUEUE_TYPE_MGMT]  = TI_FALSE;
			pLinkQ->aQenabled[QUEUE_TYPE_EAPOL] = TI_FALSE;
			eSmAction = SM_ACTION_ENABLE_DATA;
		} else {
			pLinkQ->eState = SM_STATE_OPEN_MGMT;
		}
		break;

	case SM_EVENT_QUEUES_EMPTY:
		/*
		 * The mgmt-queues are empty, so if in OPEN_MGMT state disable the
		 *   mgmt-queues and enable the data-queues.
		 */
		if (ePrevState == SM_STATE_OPEN_MGMT) {
			pLinkQ->eState = SM_STATE_OPEN_DATA;
			pLinkQ->aQenabled[QUEUE_TYPE_MGMT]  = TI_FALSE;
			pLinkQ->aQenabled[QUEUE_TYPE_EAPOL] = TI_FALSE;
			eSmAction = SM_ACTION_ENABLE_DATA;
		}
		break;

	case SM_EVENT_QUEUES_NOT_EMPTY:

		/* A packet was inserted to the mgmt-queues */
		/*
		 * If in OPEN_DATA state, enable mgmt-queues and disable data-queues.
		 *
		 * Note: The scheduler is not run here because it will called by
		 *   txMgmtQueue_wakeAll() which will run the scheduler.
		 */
		if (ePrevState == SM_STATE_OPEN_DATA) {
			pLinkQ->eState = SM_STATE_OPEN_MGMT;
			pLinkQ->aQenabled[QUEUE_TYPE_MGMT]  = TI_TRUE;
			pLinkQ->aQenabled[QUEUE_TYPE_EAPOL] = TI_TRUE;
			eSmAction = SM_ACTION_ENABLE_MGMT;
		}

		/*
		 * If in MGMT or EAPOL state, run the scheduler to transmit the packet.
		 */
		else if ( (ePrevState == SM_STATE_MGMT) || (ePrevState == SM_STATE_EAPOL) ) {
			eSmAction = SM_ACTION_RUN_SCHEDULER;
		}

		break;

	default:
		break;
	}


	/*
	 * Execute the required action.
	 * Note: This is done at the end of the SM because it may start a sequence that will call the SM again!
	 */
	switch (eSmAction) {
	case SM_ACTION_NULL:
		break;

	case SM_ACTION_ENABLE_DATA:
		txMgmtQ_DisableLink (pTxMgmtQ, uHlid);
		txDataQ_EnableLink (pTxMgmtQ->hTxDataQ, uHlid);
		break;

	case SM_ACTION_ENABLE_MGMT:
		txDataQ_DisableLink (pTxMgmtQ->hTxDataQ, uHlid);
		txMgmtQ_EnableLink (pTxMgmtQ, uHlid);
		break;

	case SM_ACTION_RUN_SCHEDULER:
		runScheduler(pTxMgmtQ);
		break;
	case SM_ACTION_STOP_ALL:
		txDataQ_DisableLink (pTxMgmtQ->hTxDataQ, uHlid);
		txDataQ_FlushLinkQueues(pTxMgmtQ->hTxDataQ, uHlid);
		TxMgmtQ_FlushLinkQueues(pTxMgmtQ, uHlid);
		txMgmtQ_EnableLink (pTxMgmtQ, uHlid);
		break;
	default:
		break;
	}
}


/**
 * \fn     runSchedulerNotFromSm
 * \brief  Run scheduler due to other events then from SM (static function)
 *
 * To comply with the SM behavior, this function is used for any case where the
 *    Mgmt-Queues scheduler may have work to do due to events external to the SM.
 * If the queues are not empty, this function runs the scheduler.
*				If the scheduler emptied the queues, update the SM.
 *
 * \note
 * \param  pTxMgmtQ - The module's object
 * \return void
 * \sa
 */
static void runSchedulerNotFromSm (TTxMgmtQ *pTxMgmtQ)
{
	TMgmtLinkQ *pLinkQ;
	TI_UINT32 uHlid;

	for (uHlid=0; uHlid<WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */

		/* If the queues are not empty, run the scheduler. */
		if ( !ARE_LINK_MGMT_QUEUES_EMPTY(pLinkQ->aQueues) ) {
			runScheduler (pTxMgmtQ);

			/* If the queues are now both empty, call the SM with QUEUES_EMPTY event. */
			if ( ARE_LINK_MGMT_QUEUES_EMPTY(pLinkQ->aQueues) ) {
				mgmtQueuesSM (pTxMgmtQ, uHlid, SM_EVENT_QUEUES_EMPTY);
			}
		}
	}
}


/**
 * \fn     runScheduler
 * \brief  The scheduler processing (static function)
 *
 * Loops over the mgmt-queues (high priority first) and if queue enabled and
 *   has packets, dequeue a packet and send it to the TxCtrl.
*				Exit if the port level is disabled or if couldn't send anything from both queues.
 *
 * \note   Protect the queues access against preemption from external context (EAPOL).
 * \param  pTxMgmtQ - The module's object
 * \return void
 * \sa
 */
static void runScheduler (TTxMgmtQ *pTxMgmtQ)
{
	TI_STATUS  eStatus;
	TTxCtrlBlk *pPktCtrlBlk;
	TI_UINT32  uQueId = 0; /* start from highest priority queue */
	TMgmtLinkQ *pLinkQ;
	TI_UINT32 uHlid = 0;
	TI_BOOL bQueueActive;
	TI_BOOL bLinkActive;


	while(1) {
		/* If the Mgmt port is closed or AC for mgmt is busy, exit. */
		if ( (!pTxMgmtQ->bMgmtPortEnable) || (pTxMgmtQ->aMgmtAcBusy) ) {
			return;
		}

		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
		bLinkActive = ( (!pLinkQ->bBusy) && pLinkQ->bEnabled );
		bQueueActive = pLinkQ->aQenabled[uQueId];

		/* Check that the current link and current queue are not busy and are enabled */
		if ( bLinkActive && bQueueActive) {
			/* Dequeue a packet in a critical section */
			context_EnterCriticalSection (pTxMgmtQ->hContext);
			pPktCtrlBlk = (TTxCtrlBlk *) que_Dequeue (pLinkQ->aQueues[uQueId]);
			context_LeaveCriticalSection (pTxMgmtQ->hContext);

			if (pPktCtrlBlk) {
				pLinkQ->tDbgCounters.aDequeuePackets[uQueId]++;

				/* Send the packet */
				eStatus = txCtrl_XmitMgmt (pTxMgmtQ->hTxCtrl, pPktCtrlBlk);


				/* In case the return status is busy it means that the packet wasn't handled
					 so we need to requeue the packet for future try. */
				if(eStatus == STATUS_XMIT_BUSY) {
					/* Requeue the packet in a critical section */
					context_EnterCriticalSection (pTxMgmtQ->hContext);
					que_Requeue (pLinkQ->aQueues[uQueId], (TI_HANDLE)pPktCtrlBlk);
					context_LeaveCriticalSection (pTxMgmtQ->hContext);

					pLinkQ->tDbgCounters.aRequeuePackets[uQueId]++;
				}

				/* The packet was handled by the lower Tx layers. */
				else {
					pLinkQ->tDbgCounters.aXmittedPackets[uQueId]++;

					/* Successful delivery so start next tx from the high priority queue (mgmt),
					 *	 giving it strict priority over the lower queue.
					 */
					uQueId = 0;
					continue;
				}
			}
		}

		/* If we got here we couldn't deliver a packet from current queue, so progress to lower
		 *	 priority queue and if already in lowest queue exit.
		 */
		if (bLinkActive) { /* Continue to next queue only if the link is active */
			uQueId++;
			if (uQueId < NUM_OF_MGMT_QUEUES) {
				continue;	/* Try sending from next queue (i.e. the EAPOL queue). */
			}
		}

		/*
		 * continue to next link
		 */
		uHlid++;
		if (uHlid < WLANLINKS_MAX_LINKS) {
			uQueId = 0;
			continue;
		} else {
			/* We couldn't send from both queues so indicate end of packets burst and exit. */
			TWD_txXfer_EndOfBurst (pTxMgmtQ->hTWD);
			return;
		}

	} /* End of while */

	/* Unreachable code */
}


/**
 * \fn     updateLinksBusyMap
 * \brief  Update links busy map (static function)
 *
 * \note
 * \param  pTxMgmtQ   - The module's object
 * \param  uLinkBitMap - The LINKs bitmap of the Link(s) to update
 * \return void
 * \sa
 */
static void updateLinksBusyMap (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uLinkBitMap)
{
	TI_UINT32 uHlid;

	/* Go over the LinkBitMap and update the related queue busy state */
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++, uLinkBitMap >>= 1) {
		if (uLinkBitMap & 0x1) { /* this Link is busy */
			pTxMgmtQ->aMgmtLinkQ[uHlid].bBusy = TI_TRUE;
		} else {
			pTxMgmtQ->aMgmtLinkQ[uHlid].bBusy = TI_FALSE;
		}
	}
}

/**
 * \fn     updateQueuesBusyMap
 * \brief  Update queues busy map (static function)
 *
 * Set the queues busy indication on or off according to the highest TID bit
 *    in the tidBitMap (1 = busy).
*				Note that both Mgmt and Eapol queues are mapped to TID 7.
*
 * \note
 * \param  pTxMgmtQ   - The module's object
 * \param  uTidBitMap - The TIDs bitmap of the queue(s) to update
 * \return void
 * \sa
 */
static void updateQueuesBusyMap (TTxMgmtQ *pTxMgmtQ, TI_UINT32 uTidBitMap)
{
	/* Set the queues busy indication on or off according to the highest TID bit (1 = busy). */
	if(uTidBitMap & (1 << MGMT_QUEUES_TID) ) {
		pTxMgmtQ->aMgmtAcBusy = TI_TRUE;
	} else {
		pTxMgmtQ->aMgmtAcBusy = TI_FALSE;
	}
}

/**
 * \fn     TxMgmtQ_SetEncryptFlag
 * \brief  Update link encrypted bit
 *
 * * \note
 * \param  pTxMgmtQ   - The module's object
 * \param  uHlid - link id
 * \return void
 * \sa
 */

void TxMgmtQ_SetEncryptFlag(TI_HANDLE hTxMgmtQ, TI_UINT32  uHlid,int flag)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ;


	pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
	pLinkQ->bEncrypt = flag;

}


/**
 * \fn     TxMgmtQ_FlushLinkQueues
 * \brief  Flush management queues the specific link
 *
 * * \note
 * \param  hTxMgmtQ   - The module's object
 * \param  uHlid - link id
 * \return void
 * \sa
 */
void TxMgmtQ_FlushLinkQueues(TI_HANDLE hTxMgmtQ, TI_UINT32 uHlid)
{
	TTxMgmtQ   *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TTxCtrlBlk *pPktCtrlBlk;
	TI_UINT32  uQueId;
	TMgmtLinkQ *pLinkQ;

	pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */

	/* Dequeue and free all queued packets */
	for (uQueId = 0 ; uQueId < NUM_OF_MGMT_QUEUES ; uQueId++) {
		while (1) {
			context_EnterCriticalSection (pTxMgmtQ->hContext);
			pPktCtrlBlk = (TTxCtrlBlk *) que_Dequeue (pLinkQ->aQueues[uQueId]);
			context_LeaveCriticalSection (pTxMgmtQ->hContext);
			if (pPktCtrlBlk == NULL) {
				break;
			}
			txCtrl_FreePacket (pTxMgmtQ->hTxCtrl, pPktCtrlBlk, TI_NOK);
		}
	}
}


/*******************************************************************************
*                       DEBUG  FUNCTIONS  IMPLEMENTATION					   *
********************************************************************************/

#ifdef TI_DBG

/**
 * \fn     txMgmtQ_PrintModuleParams
 * \brief  Print module's parameters (debug)
 *
 * This function prints the module's parameters.
 *
 * \note
 * \param  hTxMgmtQ - The module's object
 * \return void
 * \sa
 */
void txMgmtQ_PrintModuleParams (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TI_UINT32 uQueId;
	TMgmtLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	WLAN_OS_REPORT(("-------------- txMgmtQueue Module Params -----------------\n"));
	WLAN_OS_REPORT(("==========================================================\n"));

	WLAN_OS_REPORT(("bMgmtPortEnable = %d\n", pTxMgmtQ->bMgmtPortEnable));
	WLAN_OS_REPORT(("uContextId      = %d\n", pTxMgmtQ->uContextId));
	WLAN_OS_REPORT(("aMgmtAcBusy     = %2d\n", pTxMgmtQ->aMgmtAcBusy));

	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */

		WLAN_OS_REPORT(("Link %3d--------------------------------------------------\n", uHlid));
		WLAN_OS_REPORT(("  eState=%01d, bEnabled=%1d, bBusy=%1d\n", pLinkQ->eState, pLinkQ->bEnabled, pLinkQ->bBusy));
		if (pLinkQ->eState == SM_STATE_CLOSE)
			continue;
		WLAN_OS_REPORT(("  bSendEvent_NotEmpty = %d\n", pLinkQ->bSendEvent_NotEmpty));
		WLAN_OS_REPORT(("  eTxConnState        = %d\n", pLinkQ->eTxConnState));
		WLAN_OS_REPORT(("  aQenabled %2d %2d\n", pLinkQ->aQenabled[0], pLinkQ->aQenabled[1]));

		for(uQueId = 0; uQueId < NUM_OF_MGMT_QUEUES; uQueId++) {
			WLAN_OS_REPORT(("  Que %d: ", uQueId));
			que_Print (pLinkQ->aQueues[uQueId]);
		}
	}

	WLAN_OS_REPORT(("==========================================================\n\n"));
}


/**
 * \fn     txMgmtQ_PrintQueueStatistics
 * \brief  Print queues statistics (debug)
 *
 * This function prints the module's Tx statistics per Queue.
 *
 * \note
 * \param  hTxMgmtQ - The module's object
 * \return void
 * \sa
 */
void txMgmtQ_PrintQueueStatistics (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	WLAN_OS_REPORT(("-------------- Mgmt Queues Statistics  -------------------\n"));
	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
		WLAN_OS_REPORT(("Link %3d--------------------------------------------------\n", uHlid));
		WLAN_OS_REPORT(("  eState=%01d, bEnabled=%1d, bBusy=%1d\n", pLinkQ->eState, pLinkQ->bEnabled, pLinkQ->bBusy));
		if (pLinkQ->eState == SM_STATE_CLOSE)
			continue;

		WLAN_OS_REPORT(("  aEnqueuePackets: %8d %8d\n", pLinkQ->tDbgCounters.aEnqueuePackets[0], pLinkQ->tDbgCounters.aEnqueuePackets[1] ));
		WLAN_OS_REPORT(("  aDequeuePackets: %8d %8d\n", pLinkQ->tDbgCounters.aDequeuePackets[0], pLinkQ->tDbgCounters.aDequeuePackets[1] ));
		WLAN_OS_REPORT(("  aRequeuePackets: %8d %8d\n", pLinkQ->tDbgCounters.aRequeuePackets[0], pLinkQ->tDbgCounters.aRequeuePackets[1] ));
		WLAN_OS_REPORT(("  aXmittedPackets: %8d %8d\n", pLinkQ->tDbgCounters.aXmittedPackets[0], pLinkQ->tDbgCounters.aXmittedPackets[1] ));
		WLAN_OS_REPORT(("  aDroppedPackets: %8d %8d\n", pLinkQ->tDbgCounters.aDroppedPackets[0], pLinkQ->tDbgCounters.aDroppedPackets[1] ));
		WLAN_OS_REPORT(("  uNoResourcesCount(before enqueue) = %d \n", pLinkQ->tDbgCounters.uNoResourcesCount));
	}
}


/**
 * \fn     txMgmtQ_ResetQueueStatistics
 * \brief  Reset queues statistics (debug)
 *
 * This function Resets the module's Tx statistics per Queue.
 *
 * \note
 * \param  hTxMgmtQ - The module's object
 * \return void
 * \sa
 */
void txMgmtQ_ResetQueueStatistics (TI_HANDLE hTxMgmtQ)
{
	TTxMgmtQ *pTxMgmtQ = (TTxMgmtQ *)hTxMgmtQ;
	TMgmtLinkQ *pLinkQ;
	TI_UINT32  uHlid;

	for (uHlid = 0; uHlid < WLANLINKS_MAX_LINKS; uHlid++) {
		pLinkQ = &pTxMgmtQ->aMgmtLinkQ[uHlid]; /* Link queues */
		os_memoryZero(pTxMgmtQ->hOs, (void *)&(pLinkQ->tDbgCounters), sizeof(TDbgCount));
	}
}


#endif /* TI_DBG */

