/*
 * ApCmd.c
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


/** \file   ApCmd.c
 *  \brief  The AP Command-Hnadler module.
 *
 *  \see    ApCmd.h
 */

#define __FILE_ID__  FILE_ID_142

#include "WlanDrvIf.h"
#include "CmdHndlr.h"
#include "roleAP.h"
#include "ApCmd.h"
#include "APExternalIf.h"


#define GET_AP_MODULE_NUMBER(x)  ((x & 0x0000FF00))

typedef struct {
	/* Module's handles*/
	TI_HANDLE           hReport;
	TI_HANDLE           hOs;
	TI_HANDLE           hEvHandler;
	TI_HANDLE           hTWD;
	TI_HANDLE           hRoleAp;
	TI_HANDLE           hCmdHndlr;
	TI_HANDLE           hWlanLinks;
	TConfigCommand      *pAsyncCmd;
} TApCmd;


/* for temporal debug */
typedef struct {
	TApStationParams  staParam;
	TApGeneralParam   GenParam;
	TApChannelParams  ChannelParam;
	TApSsidParam      SsidParam;
	TApGeneralParam   GeneralParam;
	TApBeaconParams   BeaconParams;
} TApCmd_Type;

/**
 * \fn     apCmd_create
 * \brief  Create apCmd object
 *
 * Allocate and clear the module's object
 */
TI_HANDLE apCmd_create(TI_HANDLE hOs)
{
	TApCmd   *pApCmd;

	if ((pApCmd = os_memoryAlloc(hOs, sizeof(TApCmd))) != NULL) {
		pApCmd->hOs = hOs;

		return pApCmd;
	} else { /* Failed to allocate control block */
		WLAN_OS_REPORT(("\n FATAL ERROR: apCmd_create(): Error allocating cb - aborting\n"));
		return NULL;
	}
}


/**
 * \fn     apCmd_destroy
 * \brief  Destroy apCmd object
 *
 * Free the module's object memory
 */
TI_STATUS apCmd_destroy(TI_HANDLE hApCmd)
{
	TApCmd   *pApCmd;

	if (hApCmd != NULL) {
		pApCmd = (TApCmd *)hApCmd;


		/* Free pre-allocated control block */
		os_memoryFree(pApCmd->hOs, pApCmd, sizeof(TApCmd));
	}
	return TI_OK;
}


/**
 * \fn     apCmd_init
 * \brief  Init apCmd object
 *
 * Init module's object and link its handles
 */

void apCmd_init(TStadHandlesList *pStadHandles)
{

	TApCmd *pApCmd = (TApCmd *)(pStadHandles->hApCmd);

	pApCmd->hOs          = pStadHandles->hOs;
	pApCmd->hReport      = pStadHandles->hReport;
	pApCmd->hEvHandler   = pStadHandles->hEvHandler;
	pApCmd->hTWD         = pStadHandles->hTWD;
	pApCmd->hRoleAp      = pStadHandles->hRoleAP;
	pApCmd->hCmdHndlr    = pStadHandles->hCmdHndlr;
	pApCmd->hWlanLinks   = pStadHandles->hWlanLinks;


}

/**
 * \fn     apCmd_Execute
 * \brief  execute Ap Commands
 *
 * execute external AP commands and dispatch to relevan module
 */

int apCmd_Execute(TI_HANDLE hApCmd, TConfigCommand *pCmdObj)
{
	TApCmd *pApCmd = hApCmd;
	TI_UINT32  uModuleNumber;
	TI_STATUS  tRes = TI_NOK;
	TApCmd_Type *pParamInfo;
	ti_private_cmd_t *pMyCmd = (ti_private_cmd_t *)pCmdObj->param3;
	TApAddKeyParams  tKeyParam;
	TTwdParamInfo    tTwdParam;
	TSecurityKeys    tTwdKey;
	TI_UINT32        uHlid;

	pParamInfo = (TApCmd_Type*)os_memoryAlloc(pApCmd->hOs,sizeof(TApCmd_Type));
	if(!pParamInfo) {
		return TI_NOK;
	}

	uModuleNumber = GET_AP_MODULE_NUMBER(pMyCmd->cmd);

	switch (uModuleNumber) {
	case AP_ROLEAP_PARAMS: {
		ERoleApState eRoleApState;

		switch (pMyCmd->cmd) {
		case ROLE_AP_ADD_STATION_PARAM :
			break;
		case ROLE_AP_CHANGE_STATION_PARAM:
			break;
		case ROLE_AP_GET_STATION_PARAM:
			break;
		case ROLE_AP_SET_TX_PARAM:
			break;
		case ROLE_AP_CHANGE_BSS_PARAM:
			break;
		case ROLE_AP_SET_PORT_STATUS:
			break;
		case ROLE_AP_SET_STA_SHORT_PREAMBLE:
			break;
		case ROLE_AP_SET_STA_WME:
			break;
		case ROLE_AP_USE_CTS_PROT:
			break;
		case ROLE_AP_SET_INACTIVE_INT:
			break;
		case ROLE_AP_USE_SHORT_SLOT_TIME:
			break;
		case ROLE_AP_SET_PRIVACY:
			break;
		case ROLE_AP_SET_AP_SHORT_PREAMBLE:
			break;
		case ROLE_AP_SET_RATE:
			break;
		case ROLE_AP_REMOVE_STATION:
			break;
		case ROLE_AP_REMOVE_ALL_STATION:
			break;
		case ROLE_AP_SET_DTIM_PERIOD:
			memcpy(&pParamInfo->GeneralParam,pMyCmd->in_buffer,pMyCmd->in_buffer_len);
			break;
		case ROLE_AP_SET_BEACON_INT:
			memcpy(&pParamInfo->GeneralParam,pMyCmd->in_buffer,pMyCmd->in_buffer_len);
			break;
		case ROLE_AP_SET_CHANNEL:
			memcpy(&pParamInfo->ChannelParam,pMyCmd->in_buffer,pMyCmd->in_buffer_len);
			break;
		case ROLE_AP_ADD_BEACON_PARAM:
			memcpy(&pParamInfo->BeaconParams,pMyCmd->in_buffer,pMyCmd->in_buffer_len);
			break;
		case ROLE_AP_COMMIT_CMD:
			break;
		case ROLE_AP_SET_COUNTRY:
			break;
		case ROLE_AP_SET_RTS:
			break;
		case ROLE_AP_SET_SSID:
			memcpy(&pParamInfo->SsidParam,pMyCmd->in_buffer,pMyCmd->in_buffer_len);
			break;
		case ROLE_AP_SET_SSID_TYPE:
			break;
		case ROLE_AP_GET_STA_INACTIVITY:
			break;
		case ROLE_AP_DEAUTH_STATION:
			break;

		case ROLE_AP_SET_BSS_BRIDGE:
			break;

		case ROLE_AP_GET_HW:
			break;

		case ROLE_AP_SET_PROBE_WPS_IE:
			break;

		case ROLE_AP_STOP:
			break;

                case ROLE_AP_GET_LINK_COUNTERS:
                        break;

		default:
			break;

		}

		roleAp_getApState(pApCmd->hRoleAp, &eRoleApState);

		/*If AP is stopped or in recovery now - discard the command */
		if (((eRoleApState == ROLEAP_STATE_STOPPED) || (eRoleApState == ROLEAP_STATE_RECOVERING)) && (pMyCmd->cmd != ROLE_AP_ENABLE)) {
			tRes = TI_OK;
			break;
		}

		if (pMyCmd->cmd & SET_BIT)
			tRes = RoleAp_setApCmd(pApCmd->hRoleAp,pMyCmd->cmd,pMyCmd->in_buffer);
		else
			tRes = RoleAp_getApCmd(pApCmd->hRoleAp,pMyCmd->cmd,pMyCmd->in_buffer,pMyCmd->out_buffer);

		break;
	}
	case AP_TWD_PARAMS:

		switch (pMyCmd->cmd) {
		case TWD_ADD_KEY_PARAMS:

			memcpy(&tKeyParam,pMyCmd->in_buffer,pMyCmd->in_buffer_len);


			/* Configure Security status in TWD */
			switch (tKeyParam.cAlg) {
			case AP_WEP_CIPHER:
				tTwdParam.content.rsnEncryptionStatus = TWD_CIPHER_WEP;
				tTwdKey.keyType 	= KEY_WEP;
				break;

			case AP_TKIP_CIPHER:
				tTwdParam.content.rsnEncryptionStatus = TWD_CIPHER_TKIP;
				tTwdKey.keyType 	= KEY_TKIP;
				break;

			case AP_CCMP_CIPHER:
				tTwdParam.content.rsnEncryptionStatus = TWD_CIPHER_AES_CCMP;
				tTwdKey.keyType 	= KEY_AES;
				break;

			case AP_IGTK_CIPHER:
				/* TODO */
			default:
                                os_memoryFree(pApCmd->hOs, pParamInfo, sizeof(TApCmd_Type));
				return TI_NOK;
			}

			if (tKeyParam.cAlg == AP_TKIP_CIPHER) {
				os_memoryCopy(pApCmd->hOs,(TI_UINT8*)(((TI_UINT8*)&tTwdKey.encKey)+24),(TI_UINT8*)(((TI_UINT8*)&tKeyParam.cKey)+16),8);
				os_memoryCopy(pApCmd->hOs,(TI_UINT8*)((TI_UINT8*)&tTwdKey.micTxKey),(TI_UINT8*)(((TI_UINT8*)&tKeyParam.cKey)+16),8);
				os_memoryCopy(pApCmd->hOs,(TI_UINT8*)((TI_UINT8*)&tTwdKey.micRxKey),(TI_UINT8*)(((TI_UINT8*)&tKeyParam.cKey)+24),8);
				os_memoryCopy(pApCmd->hOs,(TI_UINT8*)(((TI_UINT8*)&tTwdKey.encKey)+16),(TI_UINT8*)(((TI_UINT8*)&tKeyParam.cKey)+24),8);
				os_memoryCopy(pApCmd->hOs,&tTwdKey.encKey,&tKeyParam.cKey,16);
			} else
				os_memoryCopy(pApCmd->hOs, &tTwdKey.encKey, &tKeyParam.cKey, tKeyParam.ckeyLen);

			tTwdKey.keyIndex = tKeyParam.cKeyIndex;
			tTwdKey.encLen = tKeyParam.ckeyLen;

			if (MAC_NULL(tKeyParam.cMac)) {
				/* broadcast key */
				os_memorySet( pApCmd->hOs, tTwdKey.macAddress, 0xFF, AP_MAC_ADDR );
				tTwdKey.hlid = 1;
				if (tTwdKey.keyType == KEY_WEP)
					tTwdKey.lidKeyType = WEP_DEFAULT_LID_TYPE;
				else
					tTwdKey.lidKeyType = BROADCAST_LID_TYPE;
			} else {
				/* unicast key */
				if (wlanLinks_FindLinkByMac(pApCmd->hWlanLinks, tKeyParam.cMac, &uHlid) != TI_OK) {
                                        os_memoryFree(pApCmd->hOs, pParamInfo, sizeof(TApCmd_Type));
					return TI_NOK;
				}


				os_memoryCopy(pApCmd->hOs,&tTwdKey.macAddress,&tKeyParam.cMac,AP_MAC_ADDR);
				tTwdKey.hlid = uHlid;
				tTwdKey.lidKeyType = UNICAST_LID_TYPE;
			}

			RoleAp_setApCmd(pApCmd->hRoleAp,TWD_ADD_KEY_PARAMS,(void*)&tTwdKey);
			break;

		case TWD_DEL_KEY_PARAMS:

			memcpy(&tKeyParam,pMyCmd->in_buffer,pMyCmd->in_buffer_len);
			tTwdKey.keyIndex = tKeyParam.cKeyIndex;
			tTwdKey.encLen = tKeyParam.ckeyLen;

			if (tKeyParam.cMac == NULL) {
				/* broadcast key */
				os_memoryCopy(pApCmd->hOs,tTwdKey.macAddress,(unsigned short *)0xFFFFFFFF,AP_MAC_ADDR);
				tTwdKey.hlid = 1;
				tTwdKey.lidKeyType = WEP_DEFAULT_LID_TYPE;
			} else {
				/* unicast key */
				if (wlanLinks_FindLinkByMac(pApCmd->hWlanLinks, tKeyParam.cMac, &uHlid) != TI_OK) {
                                        os_memoryFree(pApCmd->hOs, pParamInfo, sizeof(TApCmd_Type));
					return TI_NOK;
				}


				os_memoryCopy(pApCmd->hOs,&tTwdKey.macAddress,&tKeyParam.cMac,AP_MAC_ADDR);
				tTwdKey.hlid = uHlid;
				tTwdKey.lidKeyType = UNICAST_LID_TYPE;
			}

			RoleAp_setApCmd(pApCmd->hRoleAp,TWD_DEL_KEY_PARAMS,(void*)&tTwdKey);

			break;

		case TWD_SET_DEFAULT_KEY_PARAMS:

			RoleAp_setApCmd(pApCmd->hRoleAp,TWD_SET_DEFAULT_KEY_PARAMS,pMyCmd->in_buffer);

			break;

		case TWD_SET_CONNECTION_PHASE:

			RoleAp_setApCmd(pApCmd->hRoleAp,TWD_SET_CONNECTION_PHASE,pMyCmd->in_buffer);

			break;

		}

		break;

	default:{}

	}


	if (tRes == COMMAND_PENDING) {
		pApCmd->pAsyncCmd = pCmdObj;
	} else {
		pApCmd->pAsyncCmd = NULL;
	}

        os_memoryFree(pApCmd->hOs, pParamInfo, sizeof(TApCmd_Type));
	return tRes;
}

/**
 * \fn     apCmd_ServiceCompleteCB
 * \brief  calling by driver module to end pending commands
 *
 **/

int apCmd_ServiceCompleteCB (TI_HANDLE hApCmd, int status, void *buffer)
{
	TApCmd *pApCmd = (TApCmd *)hApCmd;

	if (pApCmd->pAsyncCmd == NULL) {
		return TI_OK;
	}

	pApCmd->pAsyncCmd->return_code = status;

	pApCmd->pAsyncCmd = NULL;

	/* Call the Cmd module to complete command processing */
	cmdHndlr_Complete (pApCmd->hCmdHndlr);

	/* Call commands handler to continue handling further commands if queued */
	cmdHndlr_HandleCommands (pApCmd->hCmdHndlr);

	return TI_OK;
}


