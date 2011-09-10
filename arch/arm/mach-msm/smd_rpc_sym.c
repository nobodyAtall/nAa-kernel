/*
Copyright (c) 2009, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

Alternatively, provided that this notice is retained in full, this software
may be relicensed by the recipient under the terms of the GNU General Public
License version 2 ("GPL") and only version 2, in which case the provisions of
the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
software under the GPL, then the identification text in the MODULE_LICENSE
macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
recipient changes the license terms to the GPL, subsequent recipients shall
not relicense under alternate licensing terms, including the BSD or dual
BSD/GPL terms.  In addition, the following license statement immediately
below and between the words START and END shall also then apply when this
software is relicensed under the GPL:

START

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License version 2 and only version 2 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

END

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/



#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/module.h>

struct sym {
	const char *str;
};

const char *smd_rpc_syms[] = {
	"CB CM",				/*0x30000000*/
	"CB DB",				/*0x30000001*/
	"CB SND",				/*0x30000002*/
	"CB WMS",				/*0x30000003*/
	"CB PDSM",				/*0x30000004*/
	"CB MISC_MODEM_APIS",			/*0x30000005*/
	"CB MISC_APPS_APIS",			/*0x30000006*/
	"CB JOYST",				/*0x30000007*/
	"CB VJOY",				/*0x30000008*/
	"CB JOYSTC",				/*0x30000009*/
	"CB ADSPRTOSATOM",			/*0x3000000A*/
	"CB ADSPRTOSMTOA",			/*0x3000000B*/
	"CB I2C",				/*0x3000000C*/
	"CB TIME_REMOTE",			/*0x3000000D*/
	"CB NV",				/*0x3000000E*/
	"CB CLKRGM_SEC",			/*0x3000000F*/
	"CB RDEVMAP",				/*0x30000010*/
	"CB FS_RAPI",				/*0x30000011*/
	"CB PBMLIB",				/*0x30000012*/
	"CB AUDMGR",				/*0x30000013*/
	"CB MVS",				/*0x30000014*/
	"CB DOG_KEEPALIVE",			/*0x30000015*/
	"CB GSDI_EXP",				/*0x30000016*/
	"CB AUTH",				/*0x30000017*/
	"CB NVRUIMI",				/*0x30000018*/
	"CB MMGSDILIB",				/*0x30000019*/
	"CB CHARGER",				/*0x3000001A*/
	"CB UIM",				/*0x3000001B*/
	"CB UNDEFINED",
	"CB PDSM_ATL",				/*0x3000001D*/
	"CB FS_XMOUNT",				/*0x3000001E*/
	"CB SECUTIL",				/*0x3000001F*/
	"CB MCCMEID",				/*0x30000020*/
	"CB PM_STROBE_FLASH",			/*0x30000021*/
	"CB DSHDR_JCDMA_APIS",			/*0x30000022*/
	"CB SMD_BRIDGE",			/*0x30000023*/
	"CB SMD_PORT_MGR",			/*0x30000024*/
	"CB BUS_PERF",				/*0x30000025*/
	"CB BUS_MON_REMOTE",			/*0x30000026*/
	"CB MC",				/*0x30000027*/
	"CB MCCAP",				/*0x30000028*/
	"CB MCCDMA",				/*0x30000029*/
	"CB MCCDS",				/*0x3000002A*/
	"CB MCCSCH",				/*0x3000002B*/
	"CB MCCSRID",				/*0x3000002C*/
	"CB SNM",				/*0x3000002D*/
	"CB MCCSYOBJ",				/*0x3000002E*/
	"CB DS707_APIS",			/*0x3000002F*/
	"CB UNDEFINED",
	"CB DSRLP_APIS",			/*0x30000031*/
	"CB RLP_APIS",				/*0x30000032*/
	"CB DS_MP_SHIM_MODEM",			/*0x30000033*/
	"CB DSHDR_APIS",			/*0x30000034*/
	"CB DSHDR_MDM_APIS",			/*0x30000035*/
	"CB DS_MP_SHIM_APPS",			/*0x30000036*/
	"CB HDRMC_APIS",			/*0x30000037*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB PMAPP_OTG",				/*0x3000003A*/
	"CB DIAG",				/*0x3000003B*/
	"CB GSTK_EXP",				/*0x3000003C*/
	"CB DSBC_MDM_APIS",			/*0x3000003D*/
	"CB HDRMRLP_MDM_APIS",			/*0x3000003E*/
	"CB HDRMRLP_APPS_APIS",			/*0x3000003F*/
	"CB HDRMC_MRLP_APIS",			/*0x30000040*/
	"CB PDCOMM_APP_API",			/*0x30000041*/
	"CB DSAT_APIS",				/*0x30000042*/
	"CB RFM",				/*0x30000043*/
	"CB CMIPAPP",				/*0x30000044*/
	"CB DSMP_UMTS_MODEM_APIS",		/*0x30000045*/
	"CB DSMP_UMTS_APPS_APIS",		/*0x30000046*/
	"CB DSUCSDMPSHIM",			/*0x30000047*/
	"CB TIME_REMOTE_ATOM",			/*0x30000048*/
	"CB CLKRGM_EVENT",			/*0x30000049*/
	"CB SD",				/*0x3000004A*/
	"CB MMOC",				/*0x3000004B*/
	"CB WLAN_ADP_FTM",			/*0x3000004C*/
	"CB WLAN_CP_CM",			/*0x3000004D*/
	"CB FTM_WLAN",				/*0x3000004E*/
	"CB SDCC_CPRM",				/*0x3000004F*/
	"CB CPRMINTERFACE",			/*0x30000050*/
	"CB DATA_ON_MODEM_MTOA_APIS",		/*0x30000051*/
	"CB UNDEFINED",
	"CB MISC_MODEM_APIS_NONWINMOB",		/*0x30000053*/
	"CB MISC_APPS_APIS_NONWINMOB",		/*0x30000054*/
	"CB PMEM_REMOTE",			/*0x30000055*/
	"CB TCXOMGR",				/*0x30000056*/
	"CB UNDEFINED",
	"CB BT",				/*0x30000058*/
	"CB PD_COMMS_API",			/*0x30000059*/
	"CB PD_COMMS_CLIENT_API",		/*0x3000005A*/
	"CB PDAPI",				/*0x3000005B*/
	"CB LSA_SUPL_DSM",			/*0x3000005C*/
	"CB TIME_REMOTE_MTOA",			/*0x3000005D*/
	"CB FTM_BT",				/*0x3000005E*/
	"CB DSUCSDAPPIF_APIS",			/*0x3000005F*/
	"CB PMAPP_GEN",				/*0x30000060*/
	"CB PM_LIB",				/*0x30000061*/
	"CB KEYPAD",				/*0x30000062*/
	"CB HSU_APP_APIS",			/*0x30000063*/
	"CB HSU_MDM_APIS",			/*0x30000064*/
	"CB ADIE_ADC_REMOTE_ATOM",		/*0x30000065*/
	"CB TLMM_REMOTE_ATOM",			/*0x30000066*/
	"CB UI_CALLCTRL",			/*0x30000067*/
	"CB UIUTILS",				/*0x30000068*/
	"CB PRL",				/*0x30000069*/
	"CB HW",				/*0x3000006A*/
	"CB OEM_RAPI",				/*0x3000006B*/
	"CB WMSPM",				/*0x3000006C*/
	"CB BTPF",				/*0x3000006D*/
	"CB CLKRGM_SYNC_EVENT",			/*0x3000006E*/
	"CB USB_APPS_RPC",			/*0x3000006F*/
	"CB USB_MODEM_RPC",			/*0x30000070*/
	"CB ADC",				/*0x30000071*/
	"CB CAMERAREMOTED",			/*0x30000072*/
	"CB SECAPIREMOTED",			/*0x30000073*/
	"CB DSATAPI",				/*0x30000074*/
	"CB CLKCTL_RPC",			/*0x30000075*/
	"CB BREWAPPCOORD",			/*0x30000076*/
	"CB UNDEFINED",
	"CB WLAN_TRP_UTILS",			/*0x30000078*/
	"CB GPIO_RPC",				/*0x30000079*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB L1_DS",				/*0x3000007C*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB OSS_RRCASN_REMOTE",			/*0x3000007F*/
	"CB PMAPP_OTG_REMOTE",			/*0x30000080*/
	"CB PING_MDM_RPC",			/*0x30000081*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB WM_BTHCI_FTM",			/*0x30000084*/
	"CB WM_BT_PF",				/*0x30000085*/
	"CB IPA_IPC_APIS",			/*0x30000086*/
	"CB UKCC_IPC_APIS",			/*0x30000087*/
	"CB UNDEFINED",
	"CB VBATT_REMOTE",			/*0x30000089*/
	"CB MFPAL_FPS",				/*0x3000008A*/
	"CB DSUMTSPDPREG",			/*0x3000008B*/
	"CB LOC_API",				/*0x3000008C*/
	"CB UNDEFINED",
	"CB CMGAN",				/*0x3000008E*/
	"CB ISENSE",				/*0x3000008F*/
	"CB TIME_SECURE",			/*0x30000090*/
	"CB HS_REM",				/*0x30000091*/
	"CB ACDB",				/*0x30000092*/
	"CB NET",				/*0x30000093*/
	"CB LED",				/*0x30000094*/
	"CB DSPAE",				/*0x30000095*/
	"CB MFKAL",				/*0x30000096*/
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB UNDEFINED",
	"CB TEST_API",				/*0x3000009B*/
	"CB REMOTEFS_SRV_API",			/*0x3000009C*/
	"CB ISI_TRANSPORT",			/*0x3000009D*/
	"CB OEM_FTM",				/*0x3000009E*/
	"CB TOUCH_SCREEN_ADC",			/*0x3000009F*/
	"CB SMD_BRIDGE_APPS",			/*0x300000A0*/
	"CB SMD_BRIDGE_MODEM",			/*0x300000A1*/
};

static struct sym_tbl {
	const char **data;
	int size;
} tbl = { smd_rpc_syms, ARRAY_SIZE(smd_rpc_syms)};

const char *smd_rpc_get_sym(uint32_t val)
{
	int idx = val & 0xFFFF;
	if (idx < tbl.size) {
		if (val & 0x01000000)
			return tbl.data[idx];
		else
			return tbl.data[idx] + 3;
	}
	return 0;
}
EXPORT_SYMBOL(smd_rpc_get_sym);

