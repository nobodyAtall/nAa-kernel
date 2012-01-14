/*
 * rolesMgr.h
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


/* \file   rolesMgr.h
 *  \brief  RolesMgr API definition
 *
 *  RoleAP module provides:
 *
 * 	- Save Bss confguration received by Hostapd application.
 *  - Start the AP Role in FW by Sending Configuration & start/stop commands
 *  \see    rolesMgr.c
 */

#ifndef _ROLES_MGR_H_
#define _ROLES_MGR_H_


#include "paramOut.h"

/* Constants */

/* Enumerations */

/* Structures */



/* rolesMgr object */
typedef struct {

	TI_HANDLE   hReport;
	TI_HANDLE   hOs;
	TI_HANDLE   hEvHandler;
	TI_HANDLE   hTWD;
	TI_HANDLE   hSiteMgr;
	TI_HANDLE   hSME;
	TI_HANDLE   hScr;
	TI_HANDLE   hRoleAP;

	TI_UINT8    uActiveRoleType;
} tRolesMgr;



/**
 * \fn     rolesMgr_create
 * \brief  Create RolesMgr object
 *
 * Allocate and clear the module's object
 *
 * \note
 * \param	hOs    - Handle to OS context
 * \return 	Handle of the allocated object, NULL if allocation failed
 * \sa     	drvMain_create
 */
TI_HANDLE rolesMgr_create(TI_HANDLE hOs);



/**
 * \fn     rolesMgr_destroy
 * \brief  Destroy RolesMgr object
 *
 * Free the module's object memory
 *
 * \note
 * \param	hRolesMgr - Handle to RolesMgr object
 * \return 	TI_OK
 * \sa     	drvMain_destroy
 */
TI_STATUS rolesMgr_destroy(TI_HANDLE hRolesMgr);


/**
 * \fn     rolesMgr_init
 * \brief  Init RolesMgr object
 *
 * Init module's object and link its handles
 *
 * \note
 * \param	pStadHandles - Handle to StadHandles
 * \return 	Void
 * \sa     	drvMain_Init
 */
void rolesMgr_init (TStadHandlesList *pStadHandles);


/**
 * \fn     rolesMgr_SetDefaults
 * \brief  set RolesMgr object default values
 *
 * Set module's object default values
 *
 * \note
 * \param	hRolesMgr - Handle to RolesMgr object
 * \param	pInitParams - Handle to object's init params structure
 *
 * \return 	TI_OK
 * \sa     	drvMain_SetDefaults
 */
TI_STATUS rolesMgr_SetDefaults (TI_HANDLE hRolesMgr, TRolesMgrInitParams *pInitParams);


/**
 * \fn     rolesMgr_setParam
 * \brief  RolesMgr object setParam API
 *
 * Module's object set param API
 *
 * \note
 * \param	hRolesMgr - Handle to RolesMgr object
 * \param	pParam - Handle to generic paramInfo structure
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 * \sa     	cmdDispathcer CB
 */
TI_STATUS rolesMgr_setParam(TI_HANDLE hRolesMgr, paramInfo_t *pParam);



/**
 * \fn     rolesMgr_getParam
 * \brief  RolesMgr object getParam API
 *
 * Module's object set param API
 *
 * \note
 * \param	hRolesMgr - Handle to RolesMgr object
 * \param	pParam - Handle to generic paramInfo structure to be filled
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 * \sa     	cmdDispathcer CB
 */
TI_STATUS rolesMgr_getParam(TI_HANDLE hRolesMgr, paramInfo_t *pParam);



/**
 * \fn     rolesMgr_start
 * \brief  RolesMgr start command API
 *
 * Start RolesMgr object - send configuration & start cmd to FW
 *
 * \note
 * \param	hRolesMgr - Handle to RolesMgr object
 *
 * \return 	TI_OK if success , otherwise - TI_NOK
 * \sa     	drvMain_Sm
 */
TI_STATUS rolesMgr_start(TI_HANDLE hRolesMgr);


#endif /*  _ROLES_MGR_API_H_*/

