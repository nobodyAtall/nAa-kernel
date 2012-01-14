/*
 * rx.h
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
/*																		   */
/*		MODULE:	rx.h													   */
/*    	PURPOSE:	Rx module functions header file						   */
/*																		   */
/***************************************************************************/
#ifndef _RX_DATA_H_
#define _RX_DATA_H_

#include "paramOut.h"
#include "TWDriver.h"
#include "wlanLinks.h"

#define DEF_EXCLUDE_UNENCYPTED				TI_FALSE
#define DEF_EAPOL_DESTINATION				OS_ABS_LAYER
#define DEF_RX_PORT_STATUS					CLOSE

typedef struct {
	TI_UINT32		excludedFrameCounter;
	TI_UINT32		rxDroppedDueToVLANIncludedCnt;
	TI_UINT32		rxWrongBssTypeCounter;
	TI_UINT32		rxWrongBssIdCounter;
	TI_UINT32      rcvUnicastFrameInOpenNotify;
} rxDataDbgCounters_t;

typedef struct {
	TI_UINT32      recvFromWlan;
        TI_UINT32      recvBytesFromWlan;
	TI_UINT32      discardWrongHlid;
	TI_UINT32      discardHlidClose;
	TI_UINT32      sendToHostapd;
	TI_UINT32      discardBeaconRoleAp;
	TI_UINT32      sendToMlme;
	TI_UINT32      sendToDataDispatcher;
	TI_UINT32      discardUnknownClass;
} rxDataLinkCounters_t;

/*                         |                           |                         |
 31 30 29 28 | 27 26 25 24 | 23 22 21 20 | 19 18 17 16 | 15 14 13 12 | 11 10 9 8 | 7 6 5 4 | 3 2 1 0
                           |                           |                         |
*/


typedef enum {
	DATA_IAPP_PACKET  = 0,
	DATA_EAPOL_PACKET = 1,
	DATA_DATA_PACKET  = 2,
	DATA_VLAN_PACKET  = 3,
	MAX_NUM_OF_RX_DATA_TYPES
} rxDataPacketType_e;


/* Rx Link Info */
/* Rx connection states updated by the connection SM. */
typedef enum {
	RX_CONN_STATE_CLOSE,
	RX_CONN_STATE_OPEN
} ERxConnState;

typedef struct {
	EWlanLinkType eType;        /* Link type */
	ERxConnState  eState;       /* Link state */
	EWlanLinkRole eRole;        /* Link Role type */
} TRxLinkInfo;

typedef void (*rxData_pBufferDispatchert) (TI_HANDLE hRxData , void *pBuffer, TRxAttr *pRxAttr);


typedef struct {
	/* Handles */
	TI_HANDLE	 		hCtrlData;
	TI_HANDLE	 		hTWD;
	TI_HANDLE			hMlme;
	TI_HANDLE			hOs;
	TI_HANDLE			hRsn;
	TI_HANDLE			hReport;
	TI_HANDLE			hSiteMgr;
	TI_HANDLE			hXCCMgr;
	TI_HANDLE           hEvHandler;
	TI_HANDLE           hTimer;
	TI_HANDLE           RxEventDistributor;
	TI_HANDLE           hThroughputTimer;
	TI_HANDLE			hPowerMgr;
#ifdef AP_MODE_ENABLED
	TI_HANDLE			hRoleAp;
#endif
	TI_BOOL             rxThroughputTimerEnable;
	TI_BOOL             rxDataExcludeUnencrypted;
	TI_BOOL             rxDataExludeBroadcastUnencrypted;
	eapolDestination_e 	rxDataEapolDestination;

	portStatus_e  		rxDataPortStatus;

	/* Rx Data Filters */
	filter_e            filteringDefaultAction;
	TI_BOOL             filteringEnabled;
	TI_BOOL             isFilterSet[MAX_DATA_FILTERS];
	TRxDataFilterRequest filterRequests[MAX_DATA_FILTERS];

	/* Counters */
	rxDataCounters_t	rxDataCounters;
	rxDataDbgCounters_t	rxDataDbgCounters;

	rxData_pBufferDispatchert rxData_dispatchBuffer[MAX_NUM_OF_RX_PORT_STATUS][MAX_NUM_OF_RX_DATA_TYPES];

	TI_INT32				prevSeqNum;

	TI_UINT32           uLastDataPktRate;  /* save Rx packet rate for statistics */

	TI_BOOL			    reAuthInProgress;
	TI_HANDLE			reAuthActiveTimer;
	TI_UINT32			reAuthActiveTimeout;

	/* Info per link */
	TRxLinkInfo           aRxLinkInfo[WLANLINKS_MAX_LINKS];   /* information per link */
	rxDataLinkCounters_t rxDataLinkCounters[WLANLINKS_MAX_LINKS]; /* counters per link */


	/* Generic Ethertype support */
	TI_UINT16           genericEthertype;
	/* Bss Bridge Support*/
	TI_BOOL             bssBridgeEnable;
	TI_HANDLE           hTxDataQueue;
} rxData_t;

/* Public function definition */
/**
 * \fn     rxData_IntraBssBridge_Enable
 * \brief  Enable Intra BSS Bridge in Rx path
 *
 *
 * \return N/A
 */
extern void rxData_IntraBssBridge_Enable(TI_HANDLE hRxData);
/**
 * \fn     rxData_IntraBssBridge_Disable
 * \brief  Disable (Isolate) Intra BSS Bridge in Rx path
 *
 *
 * \return N/A
 */
extern void rxData_IntraBssBridge_Disable(TI_HANDLE hRxData);

#endif
