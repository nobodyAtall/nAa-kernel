/*
 * cu_hostapd.h
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

/****************************************************************************/
/*                                                                          */
/*    MODULE:   cu_hostapd.h                                                */
/*    PURPOSE:                                                              */
/*                                                                          */
/****************************************************************************/
#ifndef _CU_HOSTAPD_H_
#define _CU_HOSTAPD_H_


#define MAX_NUM_OF_KEYS	   80
#define MAX_VALUE_LENGTH   100
#define MAX_KEY_LENGTH     100
#define MAX_KEY_DESCRIPTION 250

#ifndef ANDROID
#define HOSTAPD_FILE_NAME_ORIGINAL "hostapd.conf"
#define HOSTAPD_FILE_NAME_TEMP     "hostapd_temp.conf"
#else
#define HOSTAPD_FILE_NAME_ORIGINAL "/data/misc/wifi/hostapd.conf"
#define HOSTAPD_FILE_NAME_TEMP     "/data/misc/wifi/hostapd_temp.conf"
#endif

#include "osTIType.h"

typedef struct {
	TI_UINT8 key[MAX_KEY_LENGTH];
	TI_UINT8 uKeyLength;
	TI_UINT8 value[MAX_VALUE_LENGTH];
	TI_UINT8 uValueLength;

} TKeyValuePair;


typedef struct {
	TKeyValuePair keys[MAX_NUM_OF_KEYS];
	TI_BOOL       aKeyModifiedFlag[MAX_NUM_OF_KEYS];
	TI_UINT8      uNumOfKeysFound; /* Holds the actual number of keys found in file */

} TDictionary;


typedef enum {
	HOSTAPD_PARAM_SSID = 1,
	HOSTAPD_PARAM_COUNTRY_CODE,
	HOSTAPD_PARAM_80211D_ENABLED,
	HOSTAPD_PARAM_80211_OPERATION_MODE,
	HOSTAPD_PARAM_CHANNEL_NUMBER,
	HOSTAPD_PARAM_BEACON_INTERVAL,
	HOSTAPD_PARAM_DTIM_PERIOD,
	HOSTAPD_PARAM_RTS_CTS_THRESHOLD,
	HOSTAPD_PARAM_FRAG_THRESHOLD,
	HOSTAPD_PARAM_SUPPORTED_RATES,
	HOSTAPD_PARAM_BASIC_RATE_SET,
	HOSTAPD_PARAM_SHORT_PREAMBLE,
	HOSTAPD_PARAM_AUTH_ALGO,
	HOSTAPD_PARAM_STATIC_WEP_DEFAULT_KEY,
	HOSTAPD_PARAM_WEP_KEY_0,
	HOSTAPD_PARAM_WEP_KEY_1,
	HOSTAPD_PARAM_WEP_KEY_2,
	HOSTAPD_PARAM_WEP_KEY_3,
	HOSTAPD_PARAM_PRIVATE_KEY_PASSPHRASE,
	HOSTAPD_PARAM_WPA_ENABLED,
	HOSTAPD_PARAM_PRE_SHARED_KEY_WPA_PSK,
	HOSTAPD_PARAM_WPA_KEY_MGMT,
	HOSTAPD_PARAM_PRE_SHARED_KEY_WPA_WPA,
	HOSTAPD_PARAM_PRE_SHARED_KEY_WPA_WPA2,
	HOSTAPD_PARAM_GTK_REKEY_INTERVAL,
	HOSTAPD_PARAM_STRICT_REKEY,
	HOSTAPD_PARAM_GMK_REKEY_INTERVAL,
	HOSTAPD_PARAM_WPS_STATE,
	HOSTAPD_PARAM_WPS_CONFIG_METHODS,
	HOSTAPD_PARAM_WPS_PIN,
	HOSTAPD_PARAM_AP_INACTIVITY,
	HOSTAPD_PARAM_AP_HIDDEN_SSID,
	HOSTAPD_PARAM_AP_SETUP_LOCKED,
	HOSTAPD_PARAM_EAP_SERVER,
	HOSTAPD_PARAM_MAC_ADDR_ACL,
	HOSTAPD_PARAM_ACCEPT_MAC_FILE,
	HOSTAPD_PARAM_DENY_MAC_FILE,
	HOSTAPD_PARAM_IGNORE_BROADCAST_SSID,
	HOSTAPD_PARAM_AP_TABLE_MAX_SIZE,
	HOSTAPD_PARAM_AP_TABLE_EXP_TIME,
	HOSTAPD_PARAM_LAST

} EHostapdParams;




typedef struct {
	TI_INT8 keyName[MAX_KEY_LENGTH];
	TI_INT8 keyDescription[MAX_KEY_DESCRIPTION];
} keyInfo;

static keyInfo hostapdKeysList[HOSTAPD_PARAM_LAST] = {
	{"ssid", "Max length of 32 chars"},
	{"country_code", "2 chars length"},
	{"ieee80211d", "0 = Disabled, 1 = Enabled"},
	{"hw_mode","a = IEEE 802.11a, b = IEEE 802.11b, g = IEEE 802.11g"},
	{"channel", "Channel number"},
	{"beacon_int", "Beacon interval in millisecond"},
	{"dtim_period", "Number of beacons between DTIMs"},
	{"supported_rates", "Supported rate set in 100 kbps units i.e. 55 = 5.5 Mbits/sec"},
	{"basic_rates", "Basic rate set in 100 kbps units"},
	{"preamble", "0 = short preamble enabled, 1 = short preamble disabled"},
	{"auth_algs", "bit 0 = Open System Authentication, bit 1 = Shared Key Authentication (requires WEP)"},
	{"wep_default_key","key number to use when transmitting, range 0..3"},
	{"wep_key0","WEP key index 0"},
	{"wep_key1","WEP key index 1"},
	{"wep_key2","WEP key index 2"},
	{"wep_key3","WEP key index 3"},
	{"private_key_passwd","Passphrase for private key"},
	{"wpa","bit0 = WPA, bit1 = IEEE 802.11i/RSN (WPA2)"},
	{"wpa_psk","WPA pre-shared keys for WPA-PSK (64 hex digits)"},
	{"wpa_passphrase", "Secret passphrase 8..63 characters"},
	{"wpa_key_mgmt", "Set of accepted key management algorithms (WPA-PSK, WPA-EAP, or both)."},
	{"wpa_pairwise","Possible values: CCMP, TKIP"},
	{"rsn_pairwise","Possible values: CCMP, TKIP"},
	{"wpa_group_rekey","Time interval for rekeying GTK in seconds"},
	{"wpa_strict_rekey","Rekey GTK current GTK is leaving the BSS, 1 = enabled"},
	{"wpa_gmk_rekey", "Time interval for rekeying GMK in seconds"},
	{"wps_state", "0 = WPS disabled (default), 1 = WPS enabled, not configured, 2 = WPS enabled, configured"},
	{"config_methods","List of the supported configuration methods, values: label display push_button keypad"},
	{"ap_pin","Access point PIN for initial configuration and adding Registrars"},
	{"ap_max_inactivity","Inactivity time to disaccociate station"},
	{"ignore_broadcast_ssid","Hidden SSID configuration: 0=Disabled, 1=Enabled, 2=Clear SSID(ASCII 0)"},
	{"ap_setup_locked","0 = can't add new Enrollees 1 = can continue to add new Enrollees"},
	{"eap_server","0 = external RADIUS authentication  1 = integrated EAP server"},
	{"macaddr_acl","0 = accept unless in deny list 1 = deny unless in accept list 2 = use external RADIUS server"},
	{"accept_mac_file","0 = Accept/deny lists are read from separate files /etc/hostapd.accept"},
	{"deny_mac_file","0 = Accept/deny lists are read from separate files /etc/hostapd.deny"},
	{"ignore_broadcast_ssid","0 = disable 1 = send empty SSID in beacon 2 = clear SSID but keep the original length"},
	{"ap_table_max_size", "Maximum number of entries kept in AP table for detecting OLBC"},
	{"ap_table_expiration_time", "Number of seconds of no frames received after which entry may be deleted"}
};

void CuHostapd_ShowStatus(void);
void CuHostapd_SaveChanges(void);
void CuHostapd_Destroy (void);
void CuHostapd_LoadConfFileToMemory(void);
void CuHostapd_UpdateKeyInDictionary(TKeyValuePair *pKeyVal);
void CuHostapd_PrintMenu(void);


#endif  /* _CU_HOSTAPD_H_ */


