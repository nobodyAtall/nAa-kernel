/*
 * ApCmd.h
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


/* \file   ApCmd.h
*  \brief  ApCmd API definition
*
*  ApCmd module provides:
*
* 	- translate and dispatch of hostapd commands.
*  -
*  \see    ApCmd.c
*/


#ifndef _AP_CMD_H_
#define _AP_CMD_H_

#include "CmdHndlr.h"

/**
 * \fn     apCmd_Execute
 * \brief  execute AP external command
 *
 * translate and dispatch hostapd commands
 *
 * \note
 * \param	handle - Handle to ApCmd module , cmdObj
 * \return 	success/fail to process the cmd
 * \sa     	cmdHndlr_HandleCommands
 */

int apCmd_Execute(TI_HANDLE hApCmd, TConfigCommand *cmdObj);
/**
 * \fn     apCmd_init
 * \brief  Init apCmd_init object
 *
 * Init module's object and link its handles
 *
 * \note
 * \param	pStadHandles - Handle to StadHandles
 * \return 	Void
 * \sa     	drvMain_Init
 */

void apCmd_init(TStadHandlesList *pStadHandles);
/**
 * \fn     apCmd_create
 * \brief  Create apCmd_create object
 *
 * Allocate and clear the module's object
 *
 * \note
 * \param	hOs    - Handle to OS context
 * \return 	Handle of the allocated object, NULL if allocation failed
 * \sa     	drvMain_create
 */

TI_HANDLE apCmd_create(TI_HANDLE hOs);

/**
 * \fn     apCmd_destroy
 * \brief  Destroy apCmd_destroy object
 *
 * Free the module's object memory
 *
 * \note
 * \param	hApCmd - Handle to apCmd_destroy object
 * \return 	TI_OK
 * \sa     	drvMain_destroy
 */

TI_STATUS apCmd_destroy(TI_HANDLE hApCmd);

/**
 * \fn     apCmd_ServiceCompleteCB
 * \brief  calling by driver module to end pending commands
 *
 * * \note
 * \param	hApCmd - Handle to apCmd
 *          status - command status
 *
 * * \return 	TI_OK
 * \sa     	RoleAp_ApSetParam *
 **/

int apCmd_ServiceCompleteCB (TI_HANDLE hApCmd, int status, void *buffer);



#endif /* _AP_CMD_H_ */
