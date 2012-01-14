/*
 * SdioAdapter.h
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

#ifndef __MSM_SDIODRV_API_H
#define __MSM_SDIODRV_API_H

#include <asm/types.h>
#include <linux/mmc/mmc.h>

#define TIWLAN_SDIO_BUSWIDE MMC_BUS_WIDTH_4
#define TIWLAN_SDIO_CLOCK 24576000L

/* Card Common Control Registers (CCCR) */

#define CCCR_SDIO_REVISION                  0x00
#define CCCR_SD_SPECIFICATION_REVISION      0x01
#define CCCR_IO_ENABLE                      0x02
#define CCCR_IO_READY                       0x03
#define CCCR_INT_ENABLE                     0x04
#define CCCR_INT_PENDING                    0x05
#define CCCR_IO_ABORT                       0x06
#define CCCR_BUS_INTERFACE_CONTOROL         0x07
#define CCCR_CARD_CAPABILITY	            0x08
#define CCCR_COMMON_CIS_POINTER             0x09 /*0x09-0x0B*/
#define CCCR_FNO_BLOCK_SIZE	                0x10 /*0x10-0x11*/
#define FN0_CCCR_REG_32	                    0x64


/* Pprotocol defined constants */

#define SD_IO_GO_IDLE_STATE		  		    0
#define SD_IO_SEND_RELATIVE_ADDR	  	    3
#define SDIO_CMD5			  			    5
#define SD_IO_SELECT_CARD		  		    7
#define SDIO_CMD52		 	 			    52
#define SDIO_CMD53		 	 			    53
#define SDIO_SHIFT(v,n)                     (v<<n)
#define SDIO_RWFLAG(v)                      (SDIO_SHIFT(v,31))
#define SDIO_FUNCN(v)                       (SDIO_SHIFT(v,28))
#define SDIO_RAWFLAG(v)                     (SDIO_SHIFT(v,27))
#define SDIO_BLKM(v)                        (SDIO_SHIFT(v,27))
#define SDIO_OPCODE(v)                      (SDIO_SHIFT(v,26))
#define SDIO_ADDRREG(v)                     (SDIO_SHIFT(v,9))


#define VDD_VOLTAGE_WINDOW                  0xffffc0
/********************************************************************/
/*	SDIO driver functions prototypes                                */
/********************************************************************/
int sdioDrv_ConnectBus     (void *       fCbFunc,
                            void *       hCbArg,
                            unsigned int uBlkSizeShift,
                            unsigned int uSdioThreadPriority);


int sdioDrv_DisconnectBus  (void);

int sdioDrv_ExecuteCmd     (unsigned int uCmd,
                            unsigned int uArg,
                            unsigned int uRespType,
                            void *       pResponse,
                            unsigned int uLen);

int sdioDrv_ReadSync           (unsigned int uFunc,
                                unsigned int uHwAddr,
                                void *       pData,
                                unsigned int uLen,
                                unsigned int bBlkMode,
                                unsigned int bIncAddr,
                                unsigned int bMore);

int sdioDrv_WriteSync          (unsigned int uFunc,
                                unsigned int uHwAddr,
                                void *       pData,
                                unsigned int uLen,
                                unsigned int bBlkMode,
                                unsigned int bIncAddr,
                                unsigned int bMore);

int sdioDrv_ReadDirectBytes(unsigned int  uFunc,
                            unsigned int  uHwAddr,
                            unsigned char *pData,
                            unsigned int  uLen,
                            unsigned int  bMore);

int sdioDrv_WriteDirectBytes(unsigned int  uFunc,
                             unsigned int  uHwAddr,
                             unsigned char *pData,
                             unsigned int  uLen,
                             unsigned int  bMore);

void sdioDrv_register_pm(int (*wlanDrvIf_Start)(void),
                         int (*wlanDrvIf_Stop)(void));


#endif
