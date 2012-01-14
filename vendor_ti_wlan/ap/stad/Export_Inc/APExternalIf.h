/*
 * APExternalIf.h
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


/****************************************************************************/
/*                                                                          */
/*    MODULE:   APExternaIf.h                                       */
/*    PURPOSE:                                                              */
/*                                                                          */
/****************************************************************************/
#ifndef __AP_EXTERNAL_IF_H__
#define __AP_EXTERNAL_IF_H__


/** \file  APExternaIf.h
 *  \brief AP External APIs
 */


/***********/
/* defines */
/***********/
#define AP_MAX_SUPPORT_RATE  12
#define AP_MAX_KEY_LEN       32
#define AP_MAC_ADDR          6
#define AP_MAX_CNT_CHAR      3
#define AP_MAX_HEAD_BUFF     256
#define AP_MAX_TAIL_BUFF     512
#define AP_MAX_SSID_LEN      32
#define AP_MAX_WPS_IE_LEN    256
/* Private eth type for menegmant packet */
#define AP_MGMT_ETH_TYPE            0x8899
#define AP_CONTROL_WORD_LEN         2
#define AP_CTRL_HDR_RX              0
#define AP_CTRL_HDR_TX_SUCCESS      1
#define AP_CTRL_HDR_TX_FAIL         2
#define AP_CTRL_HDR_ENC BIT(3)
#define AP_STA_MAX_INACTIVITY		1000000

#define AP_MAX_CHAN_NUM             50
#define AP_NUM_OF_CHANNELS_24       14
#define AP_NUM_OF_CHANNELS_5        180

#define AP_MAX_TX_POWER             250

#define SIOCIWAPPRIV     SIOCIWFIRSTPRIV + 2

/*********************/
/* enumeration types */
/*********************/
typedef enum {
	AP_TWD_PARAMS = 0x0100,
	AP_ROLEAP_PARAMS = 0x0200,
	AP_MAX_MODULES = 0x0300
} EApModule;


/* This file contains the definitions for the parameters that can be Set/Get from outside.
    The parmeters that can be Set/Get from inside the driver only are defined in the file paramIn.h */

/****************************************************************************
                                PARAMETERS ISSUE
    Each parameter in the system is defined as UINT32. The parameter
    structue is as following:

 bit   31   30 - 24     23    22 - 16    15 - 8       7 - 0
    +-----+----------+-----+----------+-----------+-----------+
    | Set | Reserved | Get | Reserved | Module    | Parameter |
    | bit |          | bit |          | number    | number    |
    +-----+----------+-----+----------+-----------+-----------+

  The 'set' bit indicates whteher this parameter can be set from OS abstraction layer.
  The 'get' bit indicates whteher this parameter can be get from OS abstraction layer.
  (All the parameters can be Get/Set from insied the driver.)
  The module number indicated who is the oner of the parameter.
  The parameter number is the parameter unique number used to identify it.

****************************************************************************/

/** \def SET_BIT
 * \brief Bitmaks of bit which indicates if the Command is SET Command
 */
#define	SET_BIT         			0x08000000
/** \def GET_BIT
 * \brief Bitmaks of bit which indicates if the Command is GET Command
 */
#define	GET_BIT				        0x00800000
/** \def ASYNC_PARAM
 * \brief Bitmaks of bit which indicates if the access to the Command Parameter is Async
 */
#define ASYNC_PARAM					0x00010000
/** \def ALLOC_NEEDED_PARAM
 * \brief Bitmaks of bit which indicates if that the data is not allocated in the paramInfo structure
 */
#define ALLOC_NEEDED_PARAM			0x00020000



typedef enum {
	TWD_ADD_KEY_PARAMS                	  =   SET_BIT | AP_TWD_PARAMS | 0x01,
	TWD_DEL_KEY_PARAMS                	  =   SET_BIT | AP_TWD_PARAMS | 0x02,
	TWD_SET_DEFAULT_KEY_PARAMS         	  =   SET_BIT | AP_TWD_PARAMS | 0x03,
        TWD_SET_CONNECTION_PHASE		  =   SET_BIT | AP_TWD_PARAMS | 0x04,
	ROLE_AP_ADD_STATION_PARAM               =   SET_BIT | AP_ROLEAP_PARAMS | 0x01,
	ROLE_AP_CHANGE_STATION_PARAM            =   SET_BIT | AP_ROLEAP_PARAMS | 0x02,
	ROLE_AP_GET_STATION_PARAM               =   GET_BIT | AP_ROLEAP_PARAMS | 0x03,
	ROLE_AP_SET_TX_PARAM                    =   SET_BIT | AP_ROLEAP_PARAMS | 0x04,
	ROLE_AP_CHANGE_BSS_PARAM                =   SET_BIT | AP_ROLEAP_PARAMS | 0x05,
	ROLE_AP_ADD_BEACON_PARAM                =   SET_BIT | AP_ROLEAP_PARAMS | 0x06,
	ROLE_AP_SET_PORT_STATUS                 =   SET_BIT | AP_ROLEAP_PARAMS | 0x07,
	ROLE_AP_SET_STA_SHORT_PREAMBLE          =   SET_BIT | AP_ROLEAP_PARAMS | 0x08,
	ROLE_AP_SET_STA_WME                     =   SET_BIT | AP_ROLEAP_PARAMS | 0x09,
	ROLE_AP_USE_CTS_PROT                    =   SET_BIT | AP_ROLEAP_PARAMS | 0x0A,
	ROLE_AP_SET_INACTIVE_INT                =   SET_BIT | AP_ROLEAP_PARAMS | 0x0B,
	ROLE_AP_USE_SHORT_SLOT_TIME             =   SET_BIT | AP_ROLEAP_PARAMS | 0x0C,
	ROLE_AP_COMMIT_CMD                      =   SET_BIT | AP_ROLEAP_PARAMS | 0x0D,
	ROLE_AP_SET_COUNTRY                     =   SET_BIT | AP_ROLEAP_PARAMS | 0x0E,
	ROLE_AP_SET_RTS                         =   SET_BIT | AP_ROLEAP_PARAMS | 0x0F,
	ROLE_AP_SET_PRIVACY                     =   SET_BIT | AP_ROLEAP_PARAMS | 0x10,
	ROLE_AP_SET_AP_SHORT_PREAMBLE           =   SET_BIT | AP_ROLEAP_PARAMS | 0x11,
	ROLE_AP_SET_RATE                        =   SET_BIT | AP_ROLEAP_PARAMS | 0x12,
	ROLE_AP_REMOVE_STATION                  =   SET_BIT | AP_ROLEAP_PARAMS | 0x13,
	ROLE_AP_REMOVE_ALL_STATION              =   SET_BIT | AP_ROLEAP_PARAMS | 0x14,
	ROLE_AP_SET_DTIM_PERIOD                 =   SET_BIT | AP_ROLEAP_PARAMS | 0x15,
	ROLE_AP_SET_BEACON_INT                  =   SET_BIT | AP_ROLEAP_PARAMS | 0x16,
	ROLE_AP_SET_CHANNEL                     =   SET_BIT | AP_ROLEAP_PARAMS | 0x17,
	ROLE_AP_SET_SSID                        =   SET_BIT | AP_ROLEAP_PARAMS | 0x18,
	ROLE_AP_GET_SSID                        =   GET_BIT | AP_ROLEAP_PARAMS | 0x19,
	ROLE_AP_GET_STA_INACTIVITY              =   GET_BIT | AP_ROLEAP_PARAMS | 0x1A,
	ROLE_AP_DEAUTH_STATION                  =   SET_BIT | AP_ROLEAP_PARAMS | 0x1B,
	ROLE_AP_SET_BSS_BRIDGE                  =   SET_BIT | AP_ROLEAP_PARAMS | 0x1C,
	ROLE_AP_SET_SSID_TYPE                   =   SET_BIT | AP_ROLEAP_PARAMS | 0x1D,
	ROLE_AP_GET_HW                          =   GET_BIT | AP_ROLEAP_PARAMS | 0x1E,
	ROLE_AP_ENABLE	                      =   SET_BIT | AP_ROLEAP_PARAMS | 0x1F,
	ROLE_AP_STOP		                      =   SET_BIT | AP_ROLEAP_PARAMS | 0x20,
	ROLE_AP_SET_PROBE_WPS_IE                =   SET_BIT | AP_ROLEAP_PARAMS | 0x21,
	ROLE_AP_SET_TX_POWER                    =   SET_BIT | AP_ROLEAP_PARAMS | 0x22
} EApExternalCmd;


/* chipher type */
typedef enum {
	AP_WEP_CIPHER,
	AP_TKIP_CIPHER,
	AP_CCMP_CIPHER,
	AP_IGTK_CIPHER,
	AP_MAX_CIPHER
} EApCipherType;


/********************/
/* Structures types */
/********************/
/* menegament frame header */
typedef struct {
	unsigned short  sCtrlHdr;
} TApFrameHeader;



/* ROLE_AP_ADD_STATION_PARAM */
typedef struct {
	char  cMac[AP_MAC_ADDR];
	char  cSupportedRates[AP_MAX_SUPPORT_RATE];
	char  cSupportedRatesLen;
	int  ilistenInterval;
	unsigned short sAid;
	unsigned short sCapability;
	int iFlag;
} TApStationParams;


/* ROLE_AP_GET_STATION_PARAM*/
typedef struct {
	char  cMac[AP_MAC_ADDR];
	unsigned int iInactiveTime;
	int iRxBytes;
	int iTxBytes;
} TApStationInfo;

/* ROLE_AP_GET_STA_INACTIVITY */
/* uses TApGeneralParam */

typedef struct {
	char  cMac[AP_MAC_ADDR];
	unsigned long  lValue;
} TApGeneralParam;

/* ROLE_AP_SET_CHANNEL */
typedef struct {
	char cChannel;
} TApChannelParams;

/*ROLE_AP_SET_TX_PARAM */
typedef struct {
	char  cQueueId;
	unsigned short sTxop;
	unsigned short sCwmin;
	unsigned short sCwmax;
	char  cAifs;
} TApTxParams;

/* TWD_ADD_KEY_PARAMS */
typedef struct {
	char  cMac[AP_MAC_ADDR];
	char  cKeyIndex;
	char  cAlg;
	char  cTxKey;
	char  ckeyLen;
	char  cKey[AP_MAX_KEY_LEN];
} TApAddKeyParams;

/* ROLE_AP_ADD_BEACON_PARAM */
typedef struct {
	char  cHead[AP_MAX_HEAD_BUFF];
	char  cTail[AP_MAX_TAIL_BUFF];
	int iHeadLen;
	int iTailLen;
	unsigned short sBeaconIntval;
	int iDtimIntval;
} TApBeaconParams;

typedef struct {
	char cIe[AP_MAX_WPS_IE_LEN];
	int iIeLen;
} TApWpsIe;

/* ROLE_AP_SET_COUNTRY */
typedef struct {
	char cCountry[AP_MAX_CNT_CHAR];
} TApCntParam;

/* ROLE_AP_SET_RATE */
typedef struct {
	char            cMode;
	unsigned short  aSupportedRates[AP_MAX_SUPPORT_RATE];
	unsigned short  aBasicRates[AP_MAX_SUPPORT_RATE];
	char            cSuppRateLen;
	char            cBasicRateLen;
} TApRateSet;

/* ROLE_AP_SET_SSID */
typedef struct {
	char cSsid[AP_MAX_SSID_LEN];
	int iSsidLen;
} TApSsidParam;

typedef enum {
	AP_SSID_TYPE_PUBLIC,
	AP_SSID_TYPE_HIDDEN,
	AP_SSID_TYPE_MAX
} EApSsidType;

/* AP Event */
typedef enum {
	AP_EVENT_STA_AGING = 1,
	AP_EVENT_STA_MIC_FAILURE,
	AP_EVENT_STA_MAX_TX_RETRY,
	AP_EVENT_DRV_RESET,
	AP_EVENT_MAX
} EApEvent;

typedef struct {
	unsigned char uEvent;    /* uEvent must be first field and has a byte value corresponding to EApEvent enum */
	unsigned char uAddr[AP_MAC_ADDR];
} TApEvent;

typedef enum {
	AP_DOT11_B_MODE    = 1,
	AP_DOT11_A_MODE    = 2,
	AP_DOT11_G_MODE    = 3,
	AP_DOT11_DUAL_MODE = 4,
	AP_DOT11_N_MODE    = 5,
	AP_DOT11_MAX_MODE
} EApDot11Mode;

typedef struct {
	unsigned short chan; /* channel number (IEEE 802.11) */
	unsigned short freq; /* frequency in MHz */
	unsigned char  max_tx_power; /* maximum transmit power in dBm */
} TApChanData;

typedef struct {
	char           cCountry[3];
	char           Chan24str[AP_NUM_OF_CHANNELS_24];
	char           Chan5str[AP_NUM_OF_CHANNELS_5];
	unsigned char  mode;
	unsigned char  numOfAChan;
	unsigned char  numOfGChan;
	unsigned char  numOfBChan;
	unsigned char  MaxtxPower;
} TApChanHwInfo;


#endif /*__AP_EXTERNAL_IF_H__*/


