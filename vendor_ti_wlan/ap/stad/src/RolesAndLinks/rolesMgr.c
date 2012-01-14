/*
 * rolesMgr.c
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


/** \file rolesMgr.c
 *  \brief Roles Manager info
 *
 *  \see rolesMgr.h
 */


#define __FILE_ID__  FILE_ID_139
#include "rolesMgr.h"
#include "roleAP.h"
#include "osApi.h"
#include "report.h"
#include "802_11Defs.h"
#include "TWDriver.h"
#include "EvHandler.h"
#include "DrvMainModules.h"

#include "siteHash.h"


/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* Internal functions prototypes */

/* Public functions implementation */





/**
 * \fn     rolesMgr_create
 * \brief  Create RolesMgr object
 *
 * Allocate and clear the module's object
 */
TI_HANDLE rolesMgr_create(TI_HANDLE hOs)
{
	tRolesMgr   *pRolesMgr;

	if ((pRolesMgr = os_memoryAlloc(hOs, sizeof(tRolesMgr))) != NULL) {
		pRolesMgr->hOs = hOs;

		return pRolesMgr;
	} else { /* Failed to allocate control block */
		WLAN_OS_REPORT(("\n FATAL ERROR: rolesMgr_create(): Error allocating cb - aborting\n"));
		return NULL;
	}
}




/**
 * \fn     rolesMgr_destroy
 * \brief  Destroy RolesMgr object
 *
 * Free the module's object memory
 */
TI_STATUS rolesMgr_destroy(TI_HANDLE hRolesMgr)
{
	tRolesMgr   *pRolesMgr;

	if (hRolesMgr != NULL) {
		pRolesMgr = (tRolesMgr *)hRolesMgr;

		/* Free pre-allocated control block */
		os_memoryFree(pRolesMgr->hOs, pRolesMgr, sizeof(tRolesMgr));
	}
	return TI_OK;
}



/**
 * \fn     rolesMgr_init
 * \brief  Init RolesMgr object
 *
 * Init module's object and link its handles
 */
void rolesMgr_init (TStadHandlesList *pStadHandles)
{
	tRolesMgr *pRolesMgr = (tRolesMgr *)(pStadHandles->hRolesMgr);

	pRolesMgr->hOs           = pStadHandles->hOs;
	pRolesMgr->hReport       = pStadHandles->hReport;
	pRolesMgr->hEvHandler    = pStadHandles->hEvHandler;
	pRolesMgr->hTWD          = pStadHandles->hTWD;

	pRolesMgr->hSiteMgr      = pStadHandles->hSiteMgr;
	pRolesMgr->hSME          = pStadHandles->hSme;
	pRolesMgr->hScr          = pStadHandles->hSCR;

	pRolesMgr->hRoleAP       = pStadHandles->hRoleAP;

}



/**
 * \fn     rolesMgr_SetDefaults
 * \brief  set RolesMgr object default values
 *
 * Set module's object default values
 */
TI_STATUS rolesMgr_SetDefaults (TI_HANDLE hRolesMgr, TRolesMgrInitParams *pInitParams)
{
	tRolesMgr *pRolesMgr = (tRolesMgr *)hRolesMgr;

	pRolesMgr->uActiveRoleType = pInitParams->uActiveRoleType;;

	return TI_OK;
}


/**
 * \fn     rolesMgr_setParam
 * \brief  RolesMgr object setParam API
 *
 * Module's object set param API
 */
TI_STATUS rolesMgr_setParam(TI_HANDLE hRolesMgr, paramInfo_t *pParam)
{
	TI_STATUS   status = TI_OK;

	if (pParam == NULL) {
		return TI_NOK;
	}


	switch (pParam->paramType) {
	case 0: {

	}
	break;

	default:
		break;
	}

	return status;
}



/**
 * \fn     rolesMgr_getParam
 * \brief  RolesMgr object getParam API
 *
 * Module's object set param API
 */
TI_STATUS rolesMgr_getParam(TI_HANDLE hRolesMgr, paramInfo_t *pParam)
{
	return TI_NOK;
}



/**
 * \fn     rolesMgr_start
 * \brief  RolesMgr start command API
 *
 * Start RolesMgr object - send configuration & start cmd to FW
 */
TI_STATUS rolesMgr_start(TI_HANDLE hRolesMgr)
{
	tRolesMgr *pRolesMgr = (tRolesMgr *)hRolesMgr;

	/* set SCR group according to connection mode - to be removed later [liorC] */
	scr_setGroup (pRolesMgr->hScr, SCR_GID_DRV_SCAN);

	return roleAP_start(pRolesMgr->hRoleAP, 0);
}
