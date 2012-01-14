/*
 * fwDebug.c
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



/** \file  FWDebug.c
 *
 *  \see   FWDebug.h
 */

#define __FILE_ID__  FILE_ID_103
#include "tidef.h"
#include "fwDebug_api.h"
#include "FwEvent_api.h"
#include "osApi.h"
#include "report.h"
#include "BusDrv.h"
#include "TwIf.h"



#define DMA_SIZE_BUF 256

#define TWD_SDIO_VALIDATION_TXN_SIZE_DEFAULT    (8000)
#define TWD_SDIO_VALIDATION_TXN_SIZE_MAX        (8192)
#define TWD_SDIO_VALIDATION_NUM_LOOPS_DEFAULT   (1)
#define TWD_SDIO_VALIDATION_NUM_LOOPS_MAX       (100)
#define TWD_SDIO_VALIDATION_TXN_ADDRESS         (0x40000)


typedef enum {
	FWDEBUG_SDIO_TEST_STATE_WRITE_READ = 0, /* 0 */
	FWDEBUG_SDIO_TEST_STATE_COMPARE         /* 1 */

} EFwDebugSdioTestState;

typedef struct {
	TI_HANDLE       hOs;
	TI_HANDLE       hReport;
	TI_HANDLE		hTwif;
	TI_HANDLE		hFwEvent;

	TFwDubCallback	fCb;
	TI_HANDLE hCb;

	TI_UINT8*		pReadBuf;

	TTxnStruct		tTxn;
	TI_UINT8*		pDMABuf;

	/* SDIO Validation Mechanism */
	TI_UINT8*       pSdioTestWriteBuf;          /* Buffer we write data into for SDIO validation mechanism */
	TI_UINT8*       pSdioTestReadBuf;           /* Buffer we read data from for SDIO validation mechanism */
	TI_UINT32       uSdioTestTxnSize;           /* Transaction size for SDIO test */
	TI_UINT32       uSdioTestNumOfLoops;        /* Number of times to perform the SDIO test */
	TI_UINT32       uSdioTestCurrentLoopNum;    /* Current iteration in SDIO test */
	TTxnStruct		tSdioTestWriteTxnSt;        /* Struct used for write transaction */
	TTxnStruct		tSdioTestReadTxnSt;         /* Struct used for read transaction */
	TI_BOOL         bSdioTestPassed;            /* TRUE is test passed. FALSE if test failed */
	TI_UINT32       uSdioTestZero;              /* Used for writing 0 to register 0x300804 to halt coretex - It is here in the global struct
                                                   and not local in the using function in order to avoid a situation in which the halt coretex
                                                   transaction occurs after the function is done - in such case a local variable willbe garbaged.
                                                 */

	EFwDebugSdioTestState eSdioTestState;       /* Current state in SDIO test */

} TFwDebug;

/* Local functions */
static void     fwDbg_WriteAddrCb   (TI_HANDLE hFwDebug,TTxnStruct* pTxn);
static void     fwDbg_ReadAddrCb    (TI_HANDLE hFwDebug,TTxnStruct* pTxn);
static void     fwDbg_SdioValidationSM (TI_HANDLE hFwDebug,TTxnStruct* pTxnNotUsed);



/*
 * \brief	Create the FW Debug module
 *
 * \param  hOs  - Handle to OS
 * \return The created object
 *
 * \par Description
 * This function will allocate memory to FW Debug module.
 *
 * \sa
 */
TI_HANDLE fwDbg_Create (TI_HANDLE hOs)
{
	TFwDebug *pFwDebug = (TFwDebug *)os_memoryAlloc(hOs,sizeof(TFwDebug));

	if (pFwDebug == NULL) {
		WLAN_OS_REPORT (("FATAL ERROR: fwDbg_Create(): Error Creating TFwDebug - Aborting\n"));
		return NULL;
	}

	/* reset module object */
	os_memoryZero (hOs, pFwDebug, sizeof (TFwDebug));
	pFwDebug->hOs = hOs;

	return pFwDebug;
}


/*
 * \brief	Initialize the module
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \param  hReport - Handle to report
 * \param  hTwif - Handle to TWIF
 * \param  hFwEvent - Handle to fwEvent
 * \return none
 *
 * \par Description
 *
 *
 * \sa
 */
void fwDbg_Init (TI_HANDLE hFwDebug,
                 TI_HANDLE hReport,
                 TI_HANDLE hTwif,
                 TI_HANDLE hFwEvent)
{
	TFwDebug* pFwDebug = (TFwDebug*)hFwDebug;
	pFwDebug->hReport  = hReport;
	pFwDebug->hTwif	   = hTwif;
	pFwDebug->hFwEvent = hFwEvent;

	/* Allocate DMA memory for read write transact */
	pFwDebug->pDMABuf = (TI_UINT8*)os_memoryAlloc(pFwDebug->hOs,DMA_SIZE_BUF);

	/* Init SDIO test variables */
	pFwDebug->pSdioTestWriteBuf = NULL;
	pFwDebug->pSdioTestReadBuf  = NULL;
}


/*
 * \brief	Destroy the object
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \return none
 *
 * \par Description
 * Deallocate the object memory
 *
 * \sa
 */
void fwDbg_Destroy (TI_HANDLE hFwDebug)
{
	TFwDebug* pFwDebug = (TFwDebug*)hFwDebug;

	if (pFwDebug) {
		if (pFwDebug->pDMABuf) {
			os_memoryFree(pFwDebug->hOs,pFwDebug->pDMABuf,DMA_SIZE_BUF);
		}
		os_memoryFree(pFwDebug->hOs,pFwDebug,sizeof(pFwDebug));
	}
}



/*
 * \brief	This function is validating the SDIO lines by write / read / compare operations.
 *
 * \param  hFwDebug    - Handle to FW Debug.
 * \param  uNumOfLoops - Number of times to run the validation test.
 * \param  uTxnSize    - Size of the transaction (in bytes) to use in the validation test.
 *
 * \return TI_STATUS
 *
 * \par Description
 * This function is validating the SDIO lines by writing data to the chip
 * memory, and then reading it and compares it to the written data.
 * The following steps are taken:
 *  1. Disables ELP so the chip won't go to sleep.
 *  2. Disables all interrupts - This is done to disable Watchdog so no recovery will be done on the driver side.
 *  3. Halts the coretex.
 *  4. Make the read / write / compare - Most of this part is done by calling fwDbg_SdioValidationSM().
 *
 * \sa
 */
TI_STATUS fwDbg_ValidateSdio(TI_HANDLE hFwDebug, TI_UINT32 uNumOfLoops, TI_UINT32 uTxnSize)
{
	TFwDebug   *pFwDebug       = (TFwDebug *)hFwDebug;

	TI_UINT8    aDataBuffer[8] = { 'B', 'E', 'E', 'F', '4' ,'4' ,'7' ,'8' };  /* Constant data to fill in write buffer */
	TI_UINT32   uWriteBufIdx   = 0;                                           /* Index in write buffer - used to fill data inside */

	TTxnStruct *pTxn           = &pFwDebug->tTxn;




	WLAN_OS_REPORT(("----- SDIO Validation Test ----> Started [Performing %d iterations of %d bytes in transaction]. \n\n",
	                uNumOfLoops, uTxnSize));



	/* Parameter range check - Make sure uTxnSize is in range */
	if ( (uTxnSize == 0) || (uTxnSize > TWD_SDIO_VALIDATION_TXN_SIZE_MAX) ) {
		WLAN_OS_REPORT(("fwDbg_ValidateSdio() - uTxnSize (%d) is out of range. Set to %d. \n",
		                uTxnSize, TWD_SDIO_VALIDATION_TXN_SIZE_DEFAULT));

		uTxnSize = TWD_SDIO_VALIDATION_TXN_SIZE_DEFAULT;
	}

	/* Parameter range check - Make sure uNumOfLoops is in range */
	if (uNumOfLoops > TWD_SDIO_VALIDATION_NUM_LOOPS_MAX) {
		WLAN_OS_REPORT(("fwDbg_ValidateSdio() - uNumOfLoops (%d) is out of range. Set to %d. \n",
		                uNumOfLoops, TWD_SDIO_VALIDATION_NUM_LOOPS_DEFAULT));

		uNumOfLoops = TWD_SDIO_VALIDATION_NUM_LOOPS_DEFAULT;
	}


	/*
	   1. Disable ELP:
	      ------------
	      - Call twIf_Awake() - it is not enough for disabling the ELP.
	      - Only after the next transaction, it is promised that the chip will be awake (ELP disabled)
	*/

	twIf_Awake(pFwDebug->hTwif);


	/* 2. Disable all interrupts */
	/*    ---------------------- */

	fwEvent_MaskAllFwInterrupts(pFwDebug->hFwEvent);


	/* 3. Halt Coretex */
	/*    ------------ */

	pFwDebug->uSdioTestZero = 0;

	TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
	BUILD_TTxnStruct(pTxn, ACX_REG_ECPU_CONTROL, &(pFwDebug->uSdioTestZero), REGISTER_SIZE, NULL, pFwDebug)

	twIf_Transact(pFwDebug->hTwif, pTxn);


	/* 4. Make the read / write / compare */
	/*    ------------------------------- */


	/* Allocate memory for pFwDebug->pSdioTestWriteBuf */
	pFwDebug->pSdioTestWriteBuf = (TI_UINT8 *)os_memoryAlloc(pFwDebug->hOs, uTxnSize);


	/* Assert Allocation */
	if (NULL == pFwDebug->pSdioTestWriteBuf) {
		WLAN_OS_REPORT(("fwDbg_ValidateSdio() - Allocation for write buffer failed! \n"));

		return TI_NOK;
	}


	/* Allocate memory for pFwDebug->pSdioTestReadBuf */
	pFwDebug->pSdioTestReadBuf = (TI_UINT8 *)os_memoryAlloc(pFwDebug->hOs, uTxnSize);

	/* Assert Allocation */
	if (NULL == pFwDebug->pSdioTestReadBuf) {
		WLAN_OS_REPORT(("fwDbg_ValidateSdio() - Allocation for read buffer failed! \n"));

		/* Free pre-allocated pFwDebug->pSdioTestWriteBuf */
		os_memoryFree(pFwDebug->hOs, pFwDebug->pSdioTestWriteBuf, uTxnSize);

		return TI_NOK;
	}

	/* Set pFwDebug struct fields */
	pFwDebug->uSdioTestTxnSize        = uTxnSize;
	pFwDebug->uSdioTestNumOfLoops     = uNumOfLoops;
	pFwDebug->uSdioTestCurrentLoopNum = 0;
	pFwDebug->eSdioTestState          = FWDEBUG_SDIO_TEST_STATE_WRITE_READ;
	pFwDebug->bSdioTestPassed         = TI_TRUE;



	/* Fill data in pFwDebug->pSdioTestWriteBuf */
	for (uWriteBufIdx = 0; uWriteBufIdx < uTxnSize; uWriteBufIdx++) {
		pFwDebug->pSdioTestWriteBuf[uWriteBufIdx] = aDataBuffer[uWriteBufIdx % sizeof(aDataBuffer)];
	}

	/* Call the SM function to perform the Read / Write / Compare test */
	fwDbg_SdioValidationSM(hFwDebug, NULL);


	return TI_OK;
}

/*
 * \brief	Write Address to FW
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \param  Address - Absolute HW address
 * \param  Length - Length in byte to write
 * \param  Buffer - Buffer to copy to FW
 * \param  fCb - CB function
 * \param  hCb - CB Handle
 * \return none
 *
 * \par Description
 * Write buffer to HW must receive length in byte max size 256 bytes
 * address must be absolute HW address.
 *
 * \sa
 */
TI_STATUS fwDbg_WriteAddr (TI_HANDLE hFwDebug,
                           TI_UINT32 Address,
                           TI_UINT32 Length,
                           TI_UINT8* Buffer,
                           TFwDubCallback fCb,
                           TI_HANDLE hCb)
{
	TI_STATUS rc;
	TTxnStruct *pTxn;
	TFwDebug* pFwDebug = (TFwDebug*)hFwDebug;

	pTxn = &pFwDebug->tTxn;

	/* check if length is large than default threshold */
	if (Length > DMA_SIZE_BUF) {
		return TXN_STATUS_ERROR;
	}

	pFwDebug->fCb = fCb;
	pFwDebug->hCb = hCb;
	/* copy the given buffer to DMA buffer */
	os_memoryCopy(pFwDebug->hOs,pFwDebug->pDMABuf,Buffer,Length);
	/* Build the command TxnStruct */
	TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
	/* Applying a CB in case of an async read */
	BUILD_TTxnStruct(pTxn, Address, pFwDebug->pDMABuf, Length,(TTxnDoneCb)fwDbg_WriteAddrCb, pFwDebug)
	rc = twIf_Transact(pFwDebug->hTwif,pTxn);

	return rc;
}


/*
 * \brief	Read Address to FW
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \param  Address - Absolute HW address
 * \param  Length - Length in byte to write
 * \param  Buffer - Buffer to copy to FW
 * \param  fCb - CB function
 * \param  hCb - CB Handle
 * \return none
 *
 * \par Description
 * Read from HW, must receive length in byte max size 256 bytes
 * address must be absolute HW address.
 *
 * \sa
 */
TI_STATUS fwDbg_ReadAddr (TI_HANDLE hFwDebug,
                          TI_UINT32 Address,
                          TI_UINT32 Length,
                          TI_UINT8* Buffer,
                          TFwDubCallback fCb,
                          TI_HANDLE hCb)
{
	TI_STATUS rc;
	TTxnStruct *pTxn;
	TFwDebug *pFwDebug = (TFwDebug*)hFwDebug;
	pTxn = &pFwDebug->tTxn;
	/* check if length is large than default threshold */
	if (Length > DMA_SIZE_BUF) {
		return TXN_STATUS_ERROR;
	}

	pFwDebug->fCb = fCb;
	pFwDebug->hCb = hCb;
	pFwDebug->pReadBuf = Buffer;

	/* Build the command TxnStruct */
	TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_READ, TXN_INC_ADDR)
	/* Applying a CB in case of an async read */
	BUILD_TTxnStruct(pTxn, Address, pFwDebug->pDMABuf, Length,(TTxnDoneCb)fwDbg_ReadAddrCb, pFwDebug)
	rc = twIf_Transact(pFwDebug->hTwif,pTxn);
	if (rc == TXN_STATUS_COMPLETE) {
		/* copy from DMA buufer to given buffer */
		os_memoryCopy(pFwDebug->hOs,pFwDebug->pReadBuf,pFwDebug->pDMABuf,Length);
	}
	return rc;
}


/*
 * \brief	Write CB function
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \param  pTxn - pointer ti Transact
 * \return none
 *
 * \par Description
 * This function called from TWIF upon Async Write
 *
 * \sa
 */
static void fwDbg_WriteAddrCb (TI_HANDLE hFwDebug,TTxnStruct* pTxn)
{
	TFwDebug *pFwDebug = (TFwDebug*)hFwDebug;

	if (pFwDebug->fCb && pFwDebug->hCb) {
		pFwDebug->fCb(pFwDebug->hCb);
	}
}


/*
 * \brief	Read CB function
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \param  pTxn - pointer ti Transact
 * \return none
 *
 * \par Description
 * This function called from TWIF upon Async Read
 *
 * \sa
 */
static void fwDbg_ReadAddrCb (TI_HANDLE hFwDebug,TTxnStruct* pTxn)
{
	TFwDebug *pFwDebug = (TFwDebug*)hFwDebug;
	/* copy from DMA buufer to given buffer */
	os_memoryCopy(pFwDebug->hOs,pFwDebug->pReadBuf,pFwDebug->pDMABuf,pTxn->aLen[0]);

	if (pFwDebug->fCb && pFwDebug->hCb) {
		pFwDebug->fCb(pFwDebug->hCb);
	}
}


/*
 * \brief
 *
 * \param  hFwDebug    - Handle to FW Debug
 * \param  pTxnNotUsed - Pointer to Transacton struct [Not used. Must be present due to CB compatibility]
 * \return none
 *
 * \par Description
 * This function
 *
 * \sa
 */
static void fwDbg_SdioValidationSM (TI_HANDLE hFwDebug, TTxnStruct* pTxnNotUsed)
{
	TFwDebug   *pFwDebug              = (TFwDebug*)hFwDebug;

	TI_UINT8   *pWriteBuf             = pFwDebug->pSdioTestWriteBuf;
	TI_UINT8   *pReadBuf              = pFwDebug->pSdioTestReadBuf;
	TI_UINT32   uTxnSize              = pFwDebug->uSdioTestTxnSize;

	TTxnStruct *pTxn                  = NULL;           /* Used for write / read transactions */

	TI_STATUS   rc                    = TI_OK;
	TI_UINT32   uComparisonErrorCount = 0;              /* Counts the comparison errors per iteration */
	TI_UINT32   uBufIdx               = 0;



	while (1) {
		switch (pFwDebug->eSdioTestState) {

		case FWDEBUG_SDIO_TEST_STATE_WRITE_READ:

			/* Write Transaction - Write data to chip */

			pTxn = &pFwDebug->tSdioTestWriteTxnSt;

			TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
			BUILD_TTxnStruct(pTxn, TWD_SDIO_VALIDATION_TXN_ADDRESS, pWriteBuf, uTxnSize, NULL, pFwDebug)  /* No CB is needed here. We wait for the read operation to finish */

			rc = twIf_Transact(pFwDebug->hTwif, pTxn);

			if (rc == TXN_STATUS_ERROR) {
				WLAN_OS_REPORT(("fwDbg_SdioValidationSM() - Write SDIO transaction has failed! \n"));

				/* Free allocated buffers */
				os_memoryFree(pFwDebug->hOs, pWriteBuf, uTxnSize);
				os_memoryFree(pFwDebug->hOs, pReadBuf, uTxnSize);

				return;
			}


			/* Read Transaction - Read data from chip */

			pTxn = &pFwDebug->tSdioTestReadTxnSt;

			TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_READ, TXN_INC_ADDR)
			BUILD_TTxnStruct(pTxn, TWD_SDIO_VALIDATION_TXN_ADDRESS, pReadBuf, uTxnSize, (TTxnDoneCb)fwDbg_SdioValidationSM, pFwDebug)

			rc = twIf_Transact(pFwDebug->hTwif, pTxn);

			if (rc == TXN_STATUS_ERROR) {
				WLAN_OS_REPORT(("fwDbg_SdioValidationSM() - Read SDIO transaction has failed! \n"));

				/* Free allocated buffers */
				os_memoryFree(pFwDebug->hOs, pWriteBuf, uTxnSize);
				os_memoryFree(pFwDebug->hOs, pReadBuf, uTxnSize);

				return;
			}

			/* Update state */
			pFwDebug->eSdioTestState = FWDEBUG_SDIO_TEST_STATE_COMPARE;

			/* Updating the loop number is dove only in FWDEBUG_SDIO_TEST_STATE_COMPARE */

			break;


		case FWDEBUG_SDIO_TEST_STATE_COMPARE:

			/* Compare read data to written data */
			for (uBufIdx = 0; uBufIdx < uTxnSize; uBufIdx++) {
				if (pWriteBuf[uBufIdx] != pReadBuf[uBufIdx]) {
					uComparisonErrorCount++;
				}
			}

			/* Print error message in case of comparison error */
			if (uComparisonErrorCount) {
				WLAN_OS_REPORT((" fwDbg_SdioValidationSM() - Found %d errors in iteration %d. \n",
				                uComparisonErrorCount, pFwDebug->uSdioTestCurrentLoopNum));

				/* Reset uComparisonErrorCount for next iterations, and set bSdioTestPassed to mark test as failed */
				uComparisonErrorCount     = 0;
				pFwDebug->bSdioTestPassed = TI_FALSE;
			}


			/* Update loop number and state */
			pFwDebug->uSdioTestCurrentLoopNum++;
			pFwDebug->eSdioTestState = FWDEBUG_SDIO_TEST_STATE_WRITE_READ;


			/* If this is the last loop free allocated memory and return */
			if (pFwDebug->uSdioTestCurrentLoopNum == pFwDebug->uSdioTestNumOfLoops) {
				if (pFwDebug->bSdioTestPassed) {
					WLAN_OS_REPORT(("----- SDIO Validation Test Completed ----> Passed. \n\n"));
				} else {
					WLAN_OS_REPORT(("----- SDIO Validation Test Completed ----> Failed. \n\n"));
				}

				/* Free allocated buffers */
				os_memoryFree(pFwDebug->hOs, pWriteBuf, uTxnSize);
				os_memoryFree(pFwDebug->hOs, pReadBuf, uTxnSize);

				return;
			}


			break;


		default:

			WLAN_OS_REPORT(("fwDbg_SdioValidationSM() - eSdioTestState is invalid (%d) !! \n", pFwDebug->eSdioTestState));

			/* Free allocated buffers */
			os_memoryFree(pFwDebug->hOs, pWriteBuf, uTxnSize);
			os_memoryFree(pFwDebug->hOs, pReadBuf, uTxnSize);

			return;

		}   /* Switch */


		/* If the transactionis still pending - return.
		   fwDbg_SdioValidationSM() will be called after transaction completion (since it is supplied as the CB function),
		*/
		if (rc == TXN_STATUS_PENDING) {
			return;
		}

	}   /* While loop */
}

/*
 * \brief	Check HW address
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \return TI_TRUE, TI_FALSE
 *
 * \par Description
 * This function called to check the given address to be a valid memory address.
 *
 * \sa
 */
TI_BOOL fwDbg_isValidMemoryAddr (TI_HANDLE hFwDebug, TI_UINT32 Address, TI_UINT32 Length)
{
	TFwDebug *pFwDebug = (TFwDebug*)hFwDebug;

	return twIf_isValidMemoryAddr(pFwDebug->hTwif, Address, Length);
}


/*
 * \brief	Check HW address
 *
 * \param  hFwDebug  - Handle to FW Debug
 * \return TI_TRUE, TI_FALSE
 *
 * \par Description
 * This function called to check the given address to be a valid register address.
 *
 * \sa
 */
TI_BOOL fwDbg_isValidRegAddr (TI_HANDLE hFwDebug, TI_UINT32 Address, TI_UINT32 Length)
{
	TFwDebug *pFwDebug = (TFwDebug*)hFwDebug;

	return twIf_isValidRegAddr(pFwDebug->hTwif, Address, Length);
}

