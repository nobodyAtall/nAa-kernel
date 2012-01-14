/*
 * RxQueue.c
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


/** \file   RxQueue.c
 *  \brief  RX Queue module that responsible to support re-ordering of received packets to upper layers.
 *
 *  \see    RxQueue.h
 */
#define __FILE_ID__  FILE_ID_98
#include "tidef.h"
#include "osApi.h"
//#include "report.h"
#include "RxBuf.h"
#include "TWDriver.h"
#include "public_descriptors.h"
#include "timer.h"


/************************ static definition declaration *****************************/

#define RX_QUEUE_ARRAY_SIZE		                            8
#define RX_QUEUE_ARRAY_SIZE_BIT_MASK                        0x7 /* RX_QUEUE_ARRAY_SIZE -1 */
#define RX_QUEUE_WIN_SIZE		                            RX_QUEUE_ARRAY_SIZE
#define BA_SESSION_TIME_TO_SLEEP		                    (50)


#define BA_SESSION_IS_A_BIGGER_THAN_B(A,B)       (((((A)-(B)) & 0xFFF) < 0x7FF) && ((A)!=(B)))
#define BA_SESSION_IS_A_BIGGER_EQUAL_THAN_B(A,B) (((((A)-(B)) & 0xFFF) < 0x7FF))
#define SEQ_NUM_WRAP 0x1000
#define SEQ_NUM_MASK 0xFFF


#define TID_CLIENT_NONE	MAX_NUM_OF_802_1d_TAGS
/************************ static structures declaration *****************************/

/* structure describe one entry of save packet information in the packet queue array */
typedef struct {
	void                *pPacket;	/* Packet address of the packet */
	TI_STATUS	        tStatus;	/* RxXfer status. */
	TI_UINT16           uFrameSn;
} TRxQueuePacketEntry;

/* structure describe set of data that one Tid, also including the arras himself */
typedef struct {
	/* array packets Entries */
	TRxQueuePacketEntry aPaketsQueue [RX_QUEUE_ARRAY_SIZE];
	/* TID BA state */
	TI_BOOL	            aTidBaEstablished;
	/* index that winStar point on */
	TI_UINT32 	        aWinStartArrayInex;
	/* windows size */
	TI_UINT32	        aTidWinSize;
	/* expected sequence number (ESN) */
	TI_UINT16	        aTidExpectedSn;

	TI_UINT16			uStoredPackets;			/* number of packets in aPacketsQueue */
	TI_UINT32			uMissingPktTimeStamp;	/* timestamp [ms] when detected a missing packet (if still missing BA_SESSION_TIME_TO_SLEEP [ms] later, the queued packets will be passed). 0xffffffff means no missing packets */
} TRxQueueTidDataBase;

/* structure describe set of data that assist of manage one SA RxQueue arrays */
typedef struct {
	TRxQueueTidDataBase tSa1ArrayMng [MAX_NUM_OF_802_1d_TAGS];
} TRxQueueArraysMng;

/* main RxQueue structure in order to management the packets disordered array. */
typedef struct {
	TI_HANDLE           hOs;                        /* OS handler */
	TI_HANDLE           hReport;                    /* Report handler */
	TRxQueueArraysMng   tRxQueueArraysMng;          /* manage each Source Address RxQueue arrays */
	TPacketReceiveCb    tReceivePacketCB;           /* Receive packets CB address */
	TI_HANDLE           hReceivePacketCB_handle;    /* Receive packets CB handler */

	TI_HANDLE           hMissingPktTimer;           /* missing packets timer */
	TI_UINT8	    uMissingPktTimerClient;		/* tid (number) the timer is running for; TID_CLIENT_NONE if not running */
} TRxQueue;


/************************ static function declaration *****************************/

static TI_STATUS RxQueue_PassPacket (TI_HANDLE hRxQueue, TI_STATUS tStatus, const void *pBuffer);
static void MissingPktTimeout (TI_HANDLE hRxQueue, TI_BOOL bTwdInitOccured);

/**
 * \brief	Stop the timer guarding the waiting for a missing packet
 *
 *			Stops the timer ONLY for the specified TID - other TID's timers will
 *			still run (if started)
 *
 * \param	uTid	index of TID timer to stop
 */
static void StopMissingPktTimer(TRxQueue *pRxQueue, TI_UINT8 uTid)
{
	TI_UINT8 i;
	TI_UINT32 uMinRequestTime;
	TI_UINT8 uNextClient;
	TI_UINT32 uNowMs;
	TRxQueueTidDataBase *pTidInfo = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uTid]);

	/* mark this TID no longer needs the timer */
	pTidInfo->uMissingPktTimeStamp = 0xffffffff;

	/* if timer was not started for this TID, don't stop it */
	if ( pRxQueue->uMissingPktTimerClient != uTid ) {
		return;
	}

	uMinRequestTime = 0xffffffff;
	uNextClient = 0;
	uNowMs = os_timeStampMs(pRxQueue->hOs);

	/* stop timer */
	tmr_StopTimer(pRxQueue->hMissingPktTimer);
	pRxQueue->uMissingPktTimerClient = TID_CLIENT_NONE;

	/* find the minimum request time */
	for (i = 0; i < MAX_NUM_OF_802_1d_TAGS; i++) {
		if (pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[i].uMissingPktTimeStamp < uMinRequestTime) {
			uMinRequestTime = pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[i].uMissingPktTimeStamp;
			uNextClient = i;
		}
	}

	/* restart timer if any requests left */
	if (uMinRequestTime < 0xffffffff) {
		tmr_StartTimer (pRxQueue->hMissingPktTimer, MissingPktTimeout, pRxQueue, uMinRequestTime + BA_SESSION_TIME_TO_SLEEP - uNowMs, TI_FALSE);
		pRxQueue->uMissingPktTimerClient = uNextClient;
	}
}

/**
 * \brief	Starts the timer guarding the waiting for a missing packet
 *
 * \param	uTid	index of TID timer to start
 */
static void StartMissingPktTimer(TRxQueue *pRxQueue, TI_UINT8 uTid)
{
	/* request to clear this TID's queue */
	pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uTid].uMissingPktTimeStamp = os_timeStampMs(pRxQueue->hOs);

	/* start timer (if not started already) */
	if ( pRxQueue->uMissingPktTimerClient == TID_CLIENT_NONE ) {
		tmr_StartTimer (pRxQueue->hMissingPktTimer, MissingPktTimeout, pRxQueue, BA_SESSION_TIME_TO_SLEEP, TI_FALSE);
		pRxQueue->uMissingPktTimerClient = uTid;
	}
}

/**
 * \fn     RxQueue_Create()
 * \brief  Create the RxQueue module.
 *
 * Allocate and clear the RxQueue module object.
 *
 * \param  hOs - Handle to Os Abstraction Layer
 * \return Handle of the allocated object
 * \sa     RxQueue_Destroy
 */
TI_HANDLE RxQueue_Create (TI_HANDLE hOs)
{
	TRxQueue *pRxQueue;

	/* allocate module object */
	pRxQueue = os_memoryAlloc (hOs, sizeof(TRxQueue));

	if (!pRxQueue) {
		WLAN_OS_REPORT (("RxQueue_Create():  Allocation failed!!\n"));
		return NULL;
	}

	os_memoryZero (hOs, pRxQueue, (sizeof(TRxQueue)));

	pRxQueue->hOs = hOs;

	return (pRxQueue);
}


/**
 * \fn     RxQueue_Destroy()
 * \brief  Destroy the module.
 *
 * Free the module's queues and object.
 *
 * \param  hRxQueue - The module object
 * \return TI_OK on success or TI_NOK on failure
 * \sa     RxQueue_Create
 */
TI_STATUS RxQueue_Destroy (TI_HANDLE hRxQueue)
{
	TRxQueue *pRxQueue;

	if (hRxQueue) {
		pRxQueue = (TRxQueue *)hRxQueue;

		if (pRxQueue->hMissingPktTimer) {
			tmr_DestroyTimer (pRxQueue->hMissingPktTimer);
			pRxQueue->hMissingPktTimer = NULL;
		}

		/* free module object */
		os_memoryFree (pRxQueue->hOs, pRxQueue, sizeof(TRxQueue));

		return TI_OK;
	}

	return TI_NOK;
}


/**
 * \fn     RxQueue_Init()
 * \brief  Init required handles
 *
 * Init required handles and module variables.
 *
 * \note
 * \param  hRxQueue - The module object
 * \param  hReport - Report module Handles
 * \return TI_OK on success or TI_NOK on failure
 * \sa
 */
TI_STATUS RxQueue_Init (TI_HANDLE hRxQueue, TI_HANDLE hReport, TI_HANDLE hTimerModule)
{
	TRxQueue *pRxQueue = (TRxQueue *)hRxQueue;
	TI_UINT8  uTid;

	pRxQueue->hReport = hReport;
	pRxQueue->hMissingPktTimer = tmr_CreateTimer (hTimerModule);
	pRxQueue->uMissingPktTimerClient = TID_CLIENT_NONE;

	for (uTid = 0; uTid < MAX_NUM_OF_802_1d_TAGS; uTid++) {
		pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uTid].uMissingPktTimeStamp = 0xffffffff;
	}

	return TI_OK;
}


/**
 * \fn     RxQueue_Register_CB()
 * \brief  Register the function to be called for received Rx.
 *
 * \note
 * \param  hRxQueue - The module object
 * \param  CallBackID - event ID
 * \param  CBFunc - function address.
 * \param  CBObj - function parameter.
 * \return TI_OK on success or TI_NOK on failure
 * \sa
 */
void RxQueue_Register_CB (TI_HANDLE hRxQueue, TI_UINT32 uCallBackID, void *CBFunc, TI_HANDLE CBObj)
{
	TRxQueue* pRxQueue = (TRxQueue *)hRxQueue;


	switch (uCallBackID) {
	case TWD_INT_RECEIVE_PACKET:
		pRxQueue->tReceivePacketCB = (TPacketReceiveCb)CBFunc;
		pRxQueue->hReceivePacketCB_handle = CBObj;
		break;

	default:
		break;
	}
}


/**
 * \fn     RxQueue_CloseBaSession ()
 * \brief  Close BA session receiver and pass all packets in the TID queue to upper layer.
 *
 * \note
 * \param  hRxQueue - RxQueue handle.
 * \param  uFrameTid - TID session.
 * \return None
 * \sa
 */
void RxQueue_CloseBaSession(TI_HANDLE hRxQueue, TI_UINT8 uFrameTid)
{
	TRxQueue   *pRxQueue = (TRxQueue *)hRxQueue;
	TI_UINT32   i;


	/* Set the SA Tid pointer */
	TRxQueueTidDataBase *pTidDataBase = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uFrameTid]);

	/* TID illegal value ? */
	if (uFrameTid >= MAX_NUM_OF_802_1d_TAGS) {

		return;
	}

	if (pTidDataBase->aTidBaEstablished == TI_TRUE) {
		/* Clean BA session */
		pTidDataBase->aTidBaEstablished = TI_FALSE;

		/* Pass all valid entries at the array */
		for (i = 0; (i < RX_QUEUE_ARRAY_SIZE) && (i < RX_QUEUE_WIN_SIZE); i++) {
			if (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL) {
				RxQueue_PassPacket (pRxQueue,
				                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].tStatus,
				                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket);

				pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket = NULL;

				pTidDataBase->uStoredPackets--;
			}

			pTidDataBase->aWinStartArrayInex ++;

			/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
			pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;
		}

		/* stop missing packet timer for this TID */
		StopMissingPktTimer(pRxQueue, uFrameTid);
	}
}


/**
 * \fn     RxQueue_PassPacket()
 * \brief  Responsible on decode packet parameters and pass it to upper layer.
 *
 * \note
 * \param  hRxQueue  - RxQueue handle.
 * \param  aStatus   - RxXfer status that indicate if the upper layer should free the packet or use it.
 * \param  pFrame    - paket address of the packet
 * \param  pRxParams - address to structure of the Rx Descriptor received by FW.
 * \return TI_OK on success or TI_NOK on failure
 * \sa
 */
static TI_STATUS RxQueue_PassPacket (TI_HANDLE hRxQueue, TI_STATUS tStatus, const void *pBuffer)
{
	TRxQueue *pRxQueue = (TRxQueue *)hRxQueue;


	if (tStatus == TI_OK) {
		/* Get the mac header location in the packet Buffer */
		dot11_header_t *pMacHdr = (dot11_header_t *)(TI_UINT8*)RX_BUF_DATA(pBuffer);

		/* Handle endian for the frame control fields */
		pMacHdr->fc       = ENDIAN_HANDLE_WORD(pMacHdr->fc);
		pMacHdr->duration = ENDIAN_HANDLE_WORD(pMacHdr->duration);
		pMacHdr->seqCtrl  = ENDIAN_HANDLE_WORD(pMacHdr->seqCtrl);

	} else {
		RxIfDescriptor_t *pRxParams = (RxIfDescriptor_t*)pBuffer;

		pRxParams->status &= ~RX_DESC_STATUS_MASK;
		pRxParams->status |= RX_DESC_STATUS_DRIVER_RX_Q_FAIL;
	}



	/* Set the packet to upper layer */
	/* if the packet status not success it will be discarded */
	pRxQueue->tReceivePacketCB (pRxQueue->hReceivePacketCB_handle, pBuffer);

	return TI_OK;
}


/**
 * \fn     RxQueue_ReceivePacket()
 * \brief  Main function of the RxQueue module.
 * Responsible on reorder of the packets from the RxXfer to the RX module.
 * Call from RxXfer in order to pass packet to uppers layers.
 * In order to save disordered packets the module use array of structures per TID
 * that each entry describe a packet. The array elements is sorted in the way that
 * the winStart array index represent always the winStart packet and the lowest SN.
 * Each increment index represent index at the BA window. Array index winEnd  always
 * represent winEnd packet. The indexes of winStart and winEnd handled in cyclic manner.
 *
 * SN range      :  0 - 4095
 * winStart range:  0 - 7       [0 - (RX_QUEUE_ARRAY_SIZE - 1)]
 * winSize       :  Determined by the BA session. We limit it to maximum 8 [RX_QUEUE_ARRAY_SIZE]
 * winEnd        :  = winStart + winSize - 1
 *
 * The function functionality devided to parts:
 *   Part 1:
 * In case the module received a packet with SN equal to the expected SN:
 * "	pass it to upper layers
 * "	increases winStart and array index winStart
 * "	validate that all sequential queue packet are pass to the upper layers.
 *   Part 2:
 * In case the module received a packet with SN between winStart to winEnd:
 * "	Save it sorted at the array at index: Save index = ((SN - winStart) + index array winStart) % arraySize.
 *   Part 3:
 * In case the module received a packet with SN higher than winEnd:
 * "	Update winStart and WinEnd.
 * "	Save it sorted at the array in index winEnd index.
 * "	Pass to the upper layers all packets at the array indexes from old winStart index to the updated winStart index.
 *   Part 4 + 5:
 * In case the module received a BA event packet: [Remember: This is an Rx module - We expect BAR and not BA (as well as ADDBE / DELBA]
 * "	Update winStart and WinEnd
 * "	Pass to the upper layers all packets at the array indexes from old winStart index to the updated winStart index.
 * "	Free BA event packet via pass it to upper layers with error status.
 *
 * \note
 * \param  hRxQueue - RxQueue handle.
 * \param  aStatus - RxXfer status that indicate if the upper layer should free the packet or use it.
 * \param  pBuffer - paket address of the packet [contains the RxIfDescriptor_t added by the FW].
 * \return None
 * \sa
 */
void RxQueue_ReceivePacket (TI_HANDLE hRxQueue, const void * pBuffer)
{
	TRxQueue            *pRxQueue   = (TRxQueue *)hRxQueue;
	RxIfDescriptor_t    *pRxParams  = (RxIfDescriptor_t*)pBuffer;
	TI_UINT8            *pFrame     = RX_BUF_DATA((TI_UINT8 *)pBuffer);
	TI_STATUS            tStatus    = TI_OK;
	dot11_header_t      *pHdr       = (dot11_header_t *)pFrame;
	TI_UINT16		     uQosControl;


	COPY_WLAN_WORD(&uQosControl, &pHdr->qosControl); /* copy with endianess handling. */



	/*
	 * Retrieving the TAG from the packet itself and not from the Rx Descriptor since by now it is not correct.
	 * If the packet is a QoS packet but the tag is not TAG_CLASS_QOS_DATA or TAG_CLASS_AMSDU - force TAG_CLASS_QOS_DATA or TAG_CLASS_AMSDU.
	 *
	 * Note: in the DR TAG_CLASS_EAPOL packet handled as TAG_CLASS_QOS_DATA
	 */
	if (IS_QOS_FRAME(*(TI_UINT16*)pFrame) && (pRxParams->packet_class_tag != TAG_CLASS_QOS_DATA) && (pRxParams->packet_class_tag != TAG_CLASS_AMSDU)) {


		/* Get AMSDU bit from frame */
		if ( uQosControl & DOT11_QOS_CONTROL_FIELD_A_MSDU_BITS) {
			pRxParams->packet_class_tag = TAG_CLASS_AMSDU;
		} else {
			pRxParams->packet_class_tag = TAG_CLASS_QOS_DATA;
		}
	}



	/*
	 * packet doesn't need reorder ?
	 */

	if ((pRxParams->packet_class_tag != TAG_CLASS_QOS_DATA) && (pRxParams->packet_class_tag != TAG_CLASS_BA_EVENT) && (pRxParams->packet_class_tag != TAG_CLASS_AMSDU)) {

		RxQueue_PassPacket (pRxQueue, TI_OK, pBuffer);

		return;
	}



	/*
	 * pRxParams->type == TAG_CLASS_QOS_DATA ?
	 */

	if ((pRxParams->packet_class_tag == TAG_CLASS_QOS_DATA) || (pRxParams->packet_class_tag == TAG_CLASS_AMSDU)) {
		TI_UINT8            uFrameTid;
		TI_UINT16           uFrameSn;
		TI_UINT16		    uSequenceControl;
		TRxQueueTidDataBase *pTidDataBase;


		/* Get TID from frame */
		uFrameTid = uQosControl & DOT11_QOS_CONTROL_FIELD_TID_BITS;


		/* TID illegal value ? */
		if (uFrameTid >= MAX_NUM_OF_802_1d_TAGS) {

			RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

			return;
		}

		/* Set the SA Tid pointer */
		pTidDataBase = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uFrameTid]);

		/* TID legal value */
		/* Packet TID BA session not established ? */
		if (pTidDataBase->aTidBaEstablished != TI_TRUE) {
			RxQueue_PassPacket (pRxQueue, TI_OK, pBuffer);

			return;
		}


		/* If we got here - Packet TID BA established */

		/* Get Sequence Number from frame */
		COPY_WLAN_WORD(&uSequenceControl, &pHdr->seqCtrl); /* copy with endianess handling. */
		uFrameSn = (uSequenceControl & DOT11_SC_SEQ_NUM_MASK) >> 4;


		/*
		 * Note:
		 * The FW never sends packet, in establish TID BA, with SN less than ESN !!!
		 */


		/* Part 1 - Received Frame Sequence Number is the expected one ? */
		if (uFrameSn == pTidDataBase->aTidExpectedSn) {


			/*
			   If the expected SN received and timer was running - Stop the timer.

			   If we wait for more than one packet we should not stop the timer - This is why we are checking after the while loop, if we have
			   more packets stored, and if we have, we start the timer again.
			*/
			if (pTidDataBase->uMissingPktTimeStamp != 0xffffffff) {
				StopMissingPktTimer(pRxQueue, uFrameTid);
			}

			/* Pass the packet */
			RxQueue_PassPacket (pRxQueue, TI_OK, pBuffer);

			/* Increase expected SN to the next */
			pTidDataBase->aTidExpectedSn++;
			pTidDataBase->aTidExpectedSn &= 0xfff;  /* SN is 12 bits long */

			/* Increase the ArrayInex to the next */
			pTidDataBase->aWinStartArrayInex++;

			/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
			pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;


			/* Pass all saved queue consecutive packets with SN higher than the expected one */
			while (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL) {

				RxQueue_PassPacket (pRxQueue,
				                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].tStatus,
				                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket);

				pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket = NULL;

				pTidDataBase->aWinStartArrayInex++;

				/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
				pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

				pTidDataBase->aTidExpectedSn++;
				pTidDataBase->aTidExpectedSn &= 0xfff; /* SN is 12 bits long */

				/* Decrease the packets in queue */
				pTidDataBase->uStoredPackets--;
			}


			/* aTidExpectedSn % 0xfff in order to tack care of wrap around */
			pTidDataBase->aTidExpectedSn &= 0xfff;

			/* If there are still packets stored in the queue - start timer */
			if (pTidDataBase->uStoredPackets) {
				StartMissingPktTimer(pRxQueue, uFrameTid);
			}

			return;
		}


		/* Frame Sequence Number is lower than Expected sequence number (ISN) ? */
		if (! BA_SESSION_IS_A_BIGGER_THAN_B (uFrameSn, pTidDataBase->aTidExpectedSn)) {
			/* WLAN_OS_REPORT(("%s: ERROR - SN=%u is less than ESN=%u\n", __FUNCTION__, uFrameSn, pTidDataBase->aTidExpectedSn)); */

			RxQueue_PassPacket (pRxQueue, tStatus, pBuffer);

			return;
		}


		/* Part 2 - Frame Sequence Number between winStart and winEnd ? */
		if ((BA_SESSION_IS_A_BIGGER_THAN_B (uFrameSn, pTidDataBase->aTidExpectedSn)) &&
		                /* mean: uFrameSn <= pTidDataBase->aTidExpectedSn + pTidDataBase->aTidWinSize) */
		                ( ! BA_SESSION_IS_A_BIGGER_THAN_B (uFrameSn,(pTidDataBase->aTidExpectedSn + pTidDataBase->aTidWinSize - 1)))) {
			TI_UINT16 uSaveIndex = pTidDataBase->aWinStartArrayInex + (TI_UINT16)((uFrameSn + SEQ_NUM_WRAP - pTidDataBase->aTidExpectedSn) & SEQ_NUM_MASK);

			/* uSaveIndex % RX_QUEUE_ARRAY_SIZE */
			uSaveIndex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;


			/* Before storing packet in queue, make sure the place in the queue is vacant */
			if (pTidDataBase->aPaketsQueue[uSaveIndex].pPacket == NULL) {

				/* Store the packet in the queue */
				pTidDataBase->aPaketsQueue[uSaveIndex].tStatus  = tStatus;
				pTidDataBase->aPaketsQueue[uSaveIndex].pPacket  = (void *)pBuffer;
				pTidDataBase->aPaketsQueue[uSaveIndex].uFrameSn = uFrameSn;

				pTidDataBase->uStoredPackets++;


				/* Start Timer */
				StartMissingPktTimer(pRxQueue, uFrameTid);
			} else {

				RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);
				return;
			}

			return;
		}


		/*
		Part 3 - Frame Sequence Number higher than winEnd ?
		*/
		if ( BA_SESSION_IS_A_BIGGER_THAN_B (uFrameSn, (pTidDataBase->aTidExpectedSn + pTidDataBase->aTidWinSize - 1)) ) {
			TI_UINT32 i;
			TI_UINT16 uNewWinStartSn = (uFrameSn + SEQ_NUM_WRAP - pTidDataBase->aTidWinSize + 1) & SEQ_NUM_MASK;
			TI_UINT16 uSaveIndex;


			/* stop timer */
			if (pTidDataBase->uMissingPktTimeStamp != 0xffffffff) {
				StopMissingPktTimer(pRxQueue, uFrameTid);
			}

			/* Increase the ArrayInex to the next */
			pTidDataBase->aWinStartArrayInex++;

			/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
			pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

			/* Update the Expected SN since the current one is lost */
			pTidDataBase->aTidExpectedSn++;
			pTidDataBase->aTidExpectedSn &= 0xFFF;

			/* Pass all saved queue packets with SN lower than the new win start */
			for (i = 0;
			        BA_SESSION_IS_A_BIGGER_THAN_B(uNewWinStartSn,pTidDataBase->aTidExpectedSn) &&
			        (i < RX_QUEUE_ARRAY_SIZE) &&
			        (i < pTidDataBase->aTidWinSize);
			        i++) {

				if (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL) {
					RxQueue_PassPacket (pRxQueue,
					                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].tStatus,
					                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket);

					pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket = NULL;

					pTidDataBase->uStoredPackets--;
				}

				pTidDataBase->aWinStartArrayInex++;

				/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
				pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

				pTidDataBase->aTidExpectedSn++;
				pTidDataBase->aTidExpectedSn &= 0xFFF;

			}


			/* Calculate the new Expected SN */
			if (i == pTidDataBase->aTidWinSize) {
				pTidDataBase->aTidExpectedSn = uNewWinStartSn;
			} else {
				/* In case the uWinStartDelta lower than aTidWinSize - check if there are packets stored in Array */

				while (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL) {

					RxQueue_PassPacket ( pRxQueue,
					                     pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].tStatus,
					                     pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket
					                   );

					pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket = NULL;

					pTidDataBase->aWinStartArrayInex++;

					/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
					pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

					pTidDataBase->aTidExpectedSn++;
					pTidDataBase->aTidExpectedSn &= 0xFFF;


					pTidDataBase->uStoredPackets--;
				}
			}


			if (pTidDataBase->aTidExpectedSn == uFrameSn) {
				/* pass the packet */
				RxQueue_PassPacket (pRxQueue, tStatus, pBuffer);

				pTidDataBase->aTidExpectedSn++;
				pTidDataBase->aTidExpectedSn &= 0xfff;
			} else {
				uSaveIndex = pTidDataBase->aWinStartArrayInex + (TI_UINT16)((uFrameSn + SEQ_NUM_WRAP - pTidDataBase->aTidExpectedSn) & SEQ_NUM_MASK);


				/* uSaveIndex % RX_QUEUE_ARRAY_SIZE */
				uSaveIndex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

				/* Save the packet in the last entry of the queue */
				pTidDataBase->aPaketsQueue[uSaveIndex].tStatus = tStatus;
				pTidDataBase->aPaketsQueue[uSaveIndex].pPacket = (void *)pBuffer;
				pTidDataBase->aPaketsQueue[uSaveIndex].pPacket = (void *)pBuffer;

				pTidDataBase->uStoredPackets++;
			}



			/* aTidExpectedSn % 0xfff in order to take care of wrap around */
			pTidDataBase->aTidExpectedSn &= 0xfff;

			/* If there are still packets stored in the queue - start timer */
			if (pTidDataBase->uStoredPackets) {
				StartMissingPktTimer(pRxQueue, uFrameTid);
			}

			return;
		}
	}



	/*
	 * BA event ?
	 */

	if (pRxParams->packet_class_tag == TAG_CLASS_BA_EVENT) {

		TRxQueueTidDataBase *pTidDataBase;
		TI_UINT8            *pDataFrameBody;
		TI_UINT16           ufc;
		TI_UINT8            uFrameTid;
		TI_UINT16           uStartingSequenceNumber;
		TI_UINT16           uWinStartDelta;
		TI_UINT16           uBarControlField;
		TI_UINT16           uBaStartingSequenceControlField;
		TI_UINT16           uBAParameterField;
		TI_UINT32           i;


		/* Get the frame's sub type from its header */
		COPY_WLAN_WORD(&ufc, &pHdr->fc); /* copy with endianess handling. */

		/* Get the type to BA event */
		switch ((dot11_Fc_Sub_Type_e)(ufc & DOT11_FC_SUB_MASK)) {
		case DOT11_FC_SUB_BAR:


			/* get pointer to the frame body */
			pDataFrameBody = pFrame + sizeof(dot11_BarFrameHeader_t);

			/* Get TID from BAR frame */
			COPY_WLAN_WORD (&uBarControlField, (TI_UINT16 *)pDataFrameBody); /* copy with endianess handling. */
			uFrameTid = (uBarControlField & DOT11_BAR_CONTROL_FIELD_TID_BITS) >> 12;

			/* TID illegal value ? */
			if (uFrameTid >= MAX_NUM_OF_802_1d_TAGS) {

				RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

				return;
			}

			/* Set the SA Tid pointer */
			pTidDataBase = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uFrameTid]);

			/* TID legal value */
			/* packet TID BA not established ? */
			if (pTidDataBase->aTidBaEstablished != TI_TRUE) {

				RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

				return;
			}

			/* Get Starting Sequence number from BAR frame */
			pDataFrameBody = pDataFrameBody + 2;
			COPY_WLAN_WORD (&uBaStartingSequenceControlField, (TI_UINT16 *)pDataFrameBody); /* copy with endianess handling. */
			uStartingSequenceNumber = (uBaStartingSequenceControlField & DOT11_SC_SEQ_NUM_MASK) >> 4;


			/* Starting Sequence Number is higher than Expcted SN ? */
			if ( BA_SESSION_IS_A_BIGGER_THAN_B (uStartingSequenceNumber, pTidDataBase->aTidExpectedSn) ) {
				uWinStartDelta = uStartingSequenceNumber - pTidDataBase->aTidExpectedSn;

				/* If timer is on - stop it. Later on we may start it again [in case we still have packets stored] */
				if (pTidDataBase->uMissingPktTimeStamp != 0xffffffff) {
					StopMissingPktTimer(pRxQueue, uFrameTid);
				}

				/* Pass all saved queue packets with SN lower than the new win start */
				for (i = 0;
				        ((i < uWinStartDelta) || (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL)) &&
				        (i < RX_QUEUE_ARRAY_SIZE) &&
				        (i < RX_QUEUE_WIN_SIZE);
				        i++) {
					if (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL) {
						RxQueue_PassPacket (pRxQueue,
						                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].tStatus,
						                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket);

						pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket = NULL;
						pTidDataBase->uStoredPackets--;
					}

					pTidDataBase->aWinStartArrayInex++;

					/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
					pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;
				}

				/* If there are still packets stored - start the timer */
				if (pTidDataBase->uStoredPackets) {
					StartMissingPktTimer(pRxQueue, uFrameTid);
				}


				pTidDataBase->aTidExpectedSn = uStartingSequenceNumber;
			}
			break;


		case DOT11_FC_SUB_ACTION:
			/* get pointer to the frame body */
			pDataFrameBody = pFrame + sizeof(dot11_mgmtHeader_t);


			/* get Action field from BA action frame */
			pDataFrameBody++;
			switch (*pDataFrameBody) {
			case DOT11_BA_ACTION_ADDBA:


				/* get TID field and winSize from ADDBA action frame */
				pDataFrameBody = pDataFrameBody + 2;
				COPY_WLAN_WORD(&uBAParameterField, (TI_UINT16 *)pDataFrameBody); /* copy with endianess handling. */
				uFrameTid = (uBAParameterField & DOT11_BA_PARAMETER_SET_FIELD_TID_BITS) >> 2;


				/* TID illegal value ? */
				if (uFrameTid >= MAX_NUM_OF_802_1d_TAGS) {

					RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

					return;
				}

				/*set the SA Tid pointer */
				pTidDataBase = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uFrameTid]);

				/* TID legal value */
				/* packet TID BA established ? */
				if (pTidDataBase->aTidBaEstablished == TI_TRUE) {

					RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

					return;
				}

				/* get winSize from ADDBA action frame */
				pTidDataBase->aTidWinSize = (uBAParameterField & DOT11_BA_PARAMETER_SET_FIELD_WINSIZE_BITS) >> 6;

				/* winSize illegal value ? */
				if (pTidDataBase->aTidWinSize > RX_QUEUE_WIN_SIZE) {
					/* In case the win Size is higher than 8 the driver and the FW set it to 8 and inform the AP in ADDBA respond */
					pTidDataBase->aTidWinSize = RX_QUEUE_WIN_SIZE;
				}

				/* packet TID BA not yet established and winSize legal */
				/* establishe BA TID */
				pTidDataBase->aTidBaEstablished = TI_TRUE;

				/* get initial sequence number (ISN) from ADDBA action frame */
				pDataFrameBody = pDataFrameBody + 4;
				COPY_WLAN_WORD (&uStartingSequenceNumber, (TI_UINT16 *)pDataFrameBody); /* copy with endianess handling. */
				pTidDataBase->aTidExpectedSn = (uStartingSequenceNumber & DOT11_SC_SEQ_NUM_MASK) >> 4;
				pTidDataBase->aWinStartArrayInex = 0;
				os_memoryZero (pRxQueue->hOs, pTidDataBase->aPaketsQueue, sizeof (TRxQueuePacketEntry) * RX_QUEUE_ARRAY_SIZE);

				break;

			case DOT11_BA_ACTION_DELBA:


				/* get TID field and winSize from ADDBA action frame */
				pDataFrameBody = pDataFrameBody + 1;
				COPY_WLAN_WORD(&uBAParameterField, (TI_UINT16 *)pDataFrameBody); /* copy with endianess handling. */
				uFrameTid = (uBAParameterField & DOT11_DELBA_PARAMETER_FIELD_TID_BITS) >> 12;

				/* TID illegal value ? */
				if (uFrameTid >= MAX_NUM_OF_802_1d_TAGS) {

					RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

					return;
				}

				/*set the SA Tid pointer */
				pTidDataBase = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uFrameTid]);

				/* TID legal value */
				/* packet TID BA not established ? */
				if (pTidDataBase->aTidBaEstablished != TI_TRUE) {

					RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

					return;
				}

				RxQueue_CloseBaSession(hRxQueue, uFrameTid);
				break;

			default:

				RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

				return;
			}
			break;

		default:

			RxQueue_PassPacket (pRxQueue, TI_NOK, pBuffer);

			return;
		}

	}

	RxQueue_PassPacket (pRxQueue, tStatus, pBuffer);

	return;
}

/**
 * \brief	pass all the packets in a TID's queue
 *
 * \param	uTid	index of TID queue to clear
 */
static void SendQueuedPackets(TI_HANDLE hRxQueue, TI_UINT8 uTid)
{
	TRxQueue            *pRxQueue   = (TRxQueue *)hRxQueue;
	TRxQueueTidDataBase *pTidDataBase;
	TI_UINT32            uPacketsChecked = 0;

	/* Set the SA Tid pointer */
	pTidDataBase = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uTid]);

	/* Find the first stored packet */
	while ( (uPacketsChecked < RX_QUEUE_ARRAY_SIZE) && /* avoid infinite loop */
	                (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket == NULL) ) {
		pTidDataBase->aWinStartArrayInex++;

		/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
		pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

		pTidDataBase->aTidExpectedSn++;
		pTidDataBase->aTidExpectedSn &= 0xFFF;

		uPacketsChecked++;
	}

	/* Send all packets in order */
	while ((pTidDataBase->uStoredPackets > 0) && (pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket != NULL)) {
		RxQueue_PassPacket (pRxQueue,
		                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].tStatus,
		                    pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket);

		pTidDataBase->aPaketsQueue[pTidDataBase->aWinStartArrayInex].pPacket = NULL;

		pTidDataBase->aWinStartArrayInex++;

		/* aWinStartArrayInex % RX_QUEUE_ARRAY_SIZE */
		pTidDataBase->aWinStartArrayInex &= RX_QUEUE_ARRAY_SIZE_BIT_MASK;

		pTidDataBase->aTidExpectedSn++;
		pTidDataBase->aTidExpectedSn &= 0xFFF;

		pTidDataBase->uStoredPackets--;
	}
}

/*
Function Name : MissingPktTimeout

Description   : This function sends all consecutive old packets stored in a specific TID queue to the upper layer.

                This function is called on timer wake up.
                [The timer is started when we have stored packets in the RxQueue].


Parameters    : hRxQueue        - A handle to the RxQueue structure.
                bTwdInitOccured - Not used.

Returned Value: void
*/
static void MissingPktTimeout (TI_HANDLE hRxQueue, TI_BOOL bTwdInitOccured)
{
	TRxQueue *pRxQueue   = (TRxQueue *)hRxQueue;
	TI_UINT8  uTid;
	TI_UINT32 uNowMs = os_timeStampMs(pRxQueue->hOs);
	TI_UINT32 uMinRequestTime = 0xffffffff; /* the next request-time to be served */
	TI_UINT8  uTidClient = TID_CLIENT_NONE;

	pRxQueue->uMissingPktTimerClient = TID_CLIENT_NONE;

	for (uTid = 0; uTid < MAX_NUM_OF_802_1d_TAGS; uTid++) {
		TRxQueueTidDataBase *pTidInfo = &(pRxQueue->tRxQueueArraysMng.tSa1ArrayMng[uTid]);

		/* skip if no request was made to clear this queue */
		if ( (pTidInfo->uMissingPktTimeStamp == 0xffffffff) ||
		                (pTidInfo->uStoredPackets == 0) ) {
			continue;
		}

		/* if this tid expired, clear its queue */
		if ( uNowMs - pTidInfo->uMissingPktTimeStamp >= BA_SESSION_TIME_TO_SLEEP ) {
			SendQueuedPackets(pRxQueue, uTid);
			pTidInfo->uMissingPktTimeStamp = 0xffffffff;
		} else {
			/* keep uMinRequestTime as minimum */
			if (pTidInfo->uMissingPktTimeStamp < uMinRequestTime) {
				uMinRequestTime = pTidInfo->uMissingPktTimeStamp;
				uTidClient = uTid;
			}
		}
	}

	/* if any TID needs clearing, restart timer */
	if (uMinRequestTime < 0xffffffff) {
		tmr_StartTimer (pRxQueue->hMissingPktTimer, MissingPktTimeout, pRxQueue, uMinRequestTime + BA_SESSION_TIME_TO_SLEEP - uNowMs, TI_FALSE);
		pRxQueue->uMissingPktTimerClient = uTidClient;
	}
}
