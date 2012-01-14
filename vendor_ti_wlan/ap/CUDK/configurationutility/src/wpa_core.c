/*
 * wpa_core.c
 *
 * Copyright 2001-2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************
*
*   MODULE:  Wpa_Core.c
*
*   PURPOSE:
*
*   DESCRIPTION:
*   ============
*
*
****************************************************************************/

/* includes */
/************/

#ifdef ANDROID
#include <unistd.h>
#endif

/* Linux only file */
#include <netinet/if_ether.h>

#include "cu_osapi.h"
#include "TWDriver.h"
#include "ipc_wpa.h"
#include "wpa_core.h"
#include "oserr.h"


/* defines */
/***********/
#ifdef CONFIG_WPS
#define WSC_MODE_OFF	0
#define WSC_MODE_PIN	1
#define WSC_MODE_PBC	2
#endif

/* local types */
/***************/
/* Network configuration block - holds candidate connection parameters */
typedef struct {
	S32 mode;
	S32 proto;
	S32 key_mgmt;
	S32 auth_alg;
	S32 pair_wise;
	S32 group;
	U8 pass_phrase[WPACORE_MAX_PSK_STRING_LENGTH];
	U8 wep_key[4][32];
	U8 default_wep_key;
	U8 wep_key_length;
#ifdef CONFIG_WPS
	U8	WscMode;
	PS8 pWscPin;
#endif
	S32 eap;
	U8 Identity[WPACORE_MAX_CERT_PASSWORD_LENGTH];
	U8 private_key_passwd[WPACORE_MAX_CERT_PASSWORD_LENGTH];
	U8 private_key[WPACORE_MAX_CERT_PASSWORD_LENGTH];
	U8 client_cert[WPACORE_MAX_CERT_FILE_NAME_LENGTH];
	U8 password[WPACORE_MAX_CERT_PASSWORD_LENGTH];
	U8 anyWpaMode;
#ifdef XCC_MODULE_INCLUDED
	U16 XCC;
#endif
} TWpaCore_WpaSupplParams;

typedef struct {
	OS_802_11_AUTHENTICATION_MODE AuthMode;
	OS_802_11_ENCRYPTION_TYPES EncryptionTypePairWise;
	OS_802_11_ENCRYPTION_TYPES EncryptionTypeGroup;
} TWpaCore_WpaParams;

/* Module control block */
typedef struct TWpaCore {
	THandle hIpcWpa;

	S32 CurrentNetwork;

	TWpaCore_WpaSupplParams WpaSupplParams;
	TWpaCore_WpaParams WpaParams;
} TWpaCore;

/* local variables */
/*******************/

/* local fucntions */
/*******************/
static VOID WpaCore_InitWpaParams(TWpaCore* pWpaCore)
{
}

/* functions */
/*************/
THandle WpaCore_Create(PS32 pRes, PS8 pSupplIfFile)
{
	return 0;
}

VOID WpaCore_Destroy(THandle hWpaCore)
{
}

#ifdef XCC_MODULE_INCLUDED
S32 WpaCore_SetXCC(THandle hWpaCore, U16 XCCConfig)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	pWpaCore->WpaSupplParams.XCC = XCCConfig;

	return TI_OK;
}
#endif

S32 WpaCore_SetAuthMode(THandle hWpaCore, OS_802_11_AUTHENTICATION_MODE AuthMode)
{
	return OK;
}


S32 WpaCore_GetAuthMode(THandle hWpaCore, PU32 pAuthMode)
{
	return OK;
}

S32 WpaCore_SetEncryptionPairWise(THandle hWpaCore, OS_802_11_ENCRYPTION_TYPES EncryptionType)
{
	return OK;
}

S32 WpaCore_SetPrivacyEap(THandle hWpaCore, OS_802_11_EAP_TYPES EapType)
{
	return OK;
}

S32 WpaCore_GetEncryptionPairWise(THandle hWpaCore, OS_802_11_ENCRYPTION_TYPES* pEncryptionType)
{
	return OK;
}

S32 WpaCore_SetEncryptionGroup(THandle hWpaCore, OS_802_11_ENCRYPTION_TYPES EncryptionType)
{
	return OK;
}

S32 WpaCore_GetEncryptionGroup(THandle hWpaCore, OS_802_11_ENCRYPTION_TYPES* pEncryptionType)
{
	return OK;
}

S32 WpaCore_SetCredentials(THandle hWpaCore, PU8 Identity, PU8 Passward)
{
	return OK;
}

S32 WpaCore_SetCertificate(THandle hWpaCore, PU8 Filepath)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	os_memcpy((PVOID)pWpaCore->WpaSupplParams.client_cert, (PVOID)Filepath, os_strlen((PS8)Filepath));

	return OK;

}

S32 WpaCore_SetPskPassPhrase(THandle hWpaCore, PU8 pPassPhrase)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	os_memcpy((PVOID)pWpaCore->WpaSupplParams.pass_phrase, (PVOID)pPassPhrase, os_strlen((PS8)pPassPhrase));

	return OK;
}

S32 WpaCore_StopSuppl(THandle hWpaCore)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	IpcWpa_Command(pWpaCore->hIpcWpa, (PS8)"TERMINATE", TRUE);

	return OK;
}

S32 WpaCore_ChangeSupplDebugLevels(THandle hWpaCore, S32 Level1, S32 Level2, S32 Level3)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;
	S8 cmd[100];

	os_sprintf(cmd, (PS8)"CHANGE_SUPPLICANT_DEBUG %ld %ld %ld", Level1, Level2, Level3);
	IpcWpa_Command(pWpaCore->hIpcWpa, cmd, TRUE);

	return OK;
}

S32 WpaCore_AddKey(THandle hWpaCore, OS_802_11_WEP* pKey)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;
	U32 WepKeyIndx;

	WepKeyIndx = pKey->KeyIndex & 0x7FFFFFFF;

	if ((pKey->KeyIndex & 0x80000000) == 0x80000000) {
		/* Add "1" to the default wep key index - since "0" is used to indicate no default wep key */
		pWpaCore->WpaSupplParams.default_wep_key = WepKeyIndx + 1;
	}

	/* If key length wasn't set so far - set it according to current key */
	if (pWpaCore->WpaSupplParams.wep_key_length == 0) {
		pWpaCore->WpaSupplParams.wep_key_length = pKey->KeyLength;
	} else {
		if (pWpaCore->WpaSupplParams.wep_key_length != pKey->KeyLength) return ECUERR_WPA_CORE_ERROR_KEY_LEN_MUST_BE_SAME;
	}

	os_memcpy(&pWpaCore->WpaSupplParams.wep_key[WepKeyIndx][0], pKey->KeyMaterial, pKey->KeyLength);

	return OK;
}

S32 WpaCore_GetDefaultKey(THandle hWpaCore, U32* pDefaultKeyIndex)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	*pDefaultKeyIndex = pWpaCore->WpaSupplParams.default_wep_key;

	return OK;
}

#ifdef CONFIG_WPS
S32 WpaCore_StartWpsPIN(THandle hWpaCore)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	pWpaCore->WpaSupplParams.WscMode = WSC_MODE_PIN;

	return OK;
}

S32 WpaCore_StartWpsPBC(THandle hWpaCore)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	pWpaCore->WpaSupplParams.WscMode = WSC_MODE_PBC;

	return OK;
}

S32 WpaCore_StopWps(THandle hWpaCore)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	pWpaCore->WpaSupplParams.WscMode = WSC_MODE_OFF;

	return OK;
}

S32 WpaCore_SetPin(THandle hWpaCore, PS8 pPinStr)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;
	int len = os_strlen(pPinStr);

	if (len == 0)
		return ECUERR_WPA_CORE_ERROR_IVALID_PIN;

	pWpaCore->WpaSupplParams.pWscPin = (PS8)os_MemoryCAlloc(len, sizeof(char));
	if(!pWpaCore->WpaSupplParams.pWscPin)
		return ECUERR_WPA_CORE_ERROR_CANT_ALLOC_PIN;

	os_strcpy(pWpaCore->WpaSupplParams.pWscPin, pPinStr);

	return OK;
}
#endif /* CONFIG_WPS */

S32 WpaCore_SetAnyWpaMode(THandle hWpaCore, U8 anyWpaMode)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	pWpaCore->WpaSupplParams.anyWpaMode = anyWpaMode;

	return OK;
}

S32 WpaCore_GetAnyWpaMode(THandle hWpaCore, PU8 pAnyWpaMode)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;

	*pAnyWpaMode = pWpaCore->WpaSupplParams.anyWpaMode;

	return OK;
}


S32 WpaCore_SetBssType(THandle hWpaCore, U32 BssType)
{
	return OK;
}

S32 WpaCore_GetBssType(THandle hWpaCore, PU32 pBssType)
{
	return OK;
}

S32 WpaCore_SetSsid(THandle hWpaCore, OS_802_11_SSID* ssid, TMacAddr bssid)
{
	return OK;
}

S32 WpaCore_Disassociate(THandle hWpaCore)
{
	TWpaCore* pWpaCore = (TWpaCore*)hWpaCore;
	S8 cmd[256];

	os_sprintf(cmd, (PS8)"DISABLE_NETWORK %d", pWpaCore->CurrentNetwork);
	if (IpcWpa_Command(pWpaCore->hIpcWpa, cmd, 1)) {
		os_error_printf(CU_MSG_ERROR, (PS8)"Failed to disconnect from current ssid\n");
		return ECUERR_WPA_CORE_ERROR_FAILED_DISCONNECT_SSID;
	}

	pWpaCore->CurrentNetwork = -1;
	IpcWpa_Command(pWpaCore->hIpcWpa, (PS8)"SAVE_CONFIG", 0);

#if 0 /* for futur WPS work */
	if(pWpaCore->CurrentNetwork == -1) {
		os_sprintf(cmd, (PS8)"LIST_NETWORKS");
		if (IpcWpa_Command(pWpaCore->hIpcWpa, cmd, 1)) {
			os_error_printf(CU_MSG_ERROR, (PS8)"Failed to disconnect from current ssid\n");
			return ECUERR_WPA_CORE_ERROR_FAILED_DISCONNECT_SSID;
		}
	} else {
		os_sprintf(cmd, (PS8)"DISABLE_NETWORK %d", pWpaCore->CurrentNetwork);
		if (IpcWpa_Command(pWpaCore->hIpcWpa, cmd, 1)) {
			os_error_printf(CU_MSG_ERROR, (PS8)"Failed to disconnect from current ssid\n");
			return ECUERR_WPA_CORE_ERROR_FAILED_DISCONNECT_SSID;
		}

		pWpaCore->CurrentNetwork = -1;
		IpcWpa_Command(pWpaCore->hIpcWpa, (PS8)"SAVE_CONFIG", 0);
	}
#endif /* #if 0 for futur WPS work */

	return OK;
}

