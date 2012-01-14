/*
 * driver_ti.h
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

#ifndef _DRIVER_TI_H_
#define _DRIVER_TI_H_

#include "wireless_copy.h"
#include "common.h"
#include "driver.h"
#include "l2_packet.h"
#include "eloop.h"
#include "priv_netlink.h"
#include "driver_wext.h"
#include "wpa_ctrl.h"
#include "wpa_supplicant_i.h"
#include "config.h"
#include "ieee802_11_defs.h"
#include "cu_ostypes.h"
#include "STADExternalIf.h"
#include "convert.h"

#define TIWLAN_DRV_NAME         "tiwlan0"

#define NUMBER_SCAN_CHANNELS_FCC        11
#define NUMBER_SCAN_CHANNELS_ETSI       13
#define NUMBER_SCAN_CHANNELS_MKK1       14

#define RX_SELF_FILTER			0
#define RX_BROADCAST_FILTER		1
#define RX_IPV4_MULTICAST_FILTER	2
#define RX_IPV6_MULTICAST_FILTER	3

#define MAX_NUMBER_SEQUENTIAL_ERRORS	4

typedef enum {
	BLUETOOTH_COEXISTENCE_MODE_ENABLED = 0,
	BLUETOOTH_COEXISTENCE_MODE_DISABLED,
	BLUETOOTH_COEXISTENCE_MODE_SENSE
} EUIBTCoexMode;

struct wpa_driver_ti_data {
	void *wext; /* private data for driver_wext */
	void *ctx;
	char ifname[IFNAMSIZ + 1];
	int ioctl_sock;
	u8 own_addr[ETH_ALEN];          /* MAC address of WLAN interface */
	int driver_is_loaded;           /* TRUE/FALSE flag if driver is already loaded and can be accessed */
	int scan_type;                  /* SCAN_TYPE_NORMAL_ACTIVE or  SCAN_TYPE_NORMAL_PASSIVE */
	int scan_channels;              /* Number of allowed scan channels */
	unsigned int link_speed;        /* Link Speed */
	u32 btcoex_mode;		/* BtCoex Mode */
	int last_scan;			/* Last scan type */
#ifdef CONFIG_WPS
	struct wpabuf *probe_req_ie;    /* Store the latest probe_req_ie for WSC */
#endif
	int errors;			/* Number of sequential errors */
	int specific_scan;		/* Scan for specific AP? */
};
#endif
