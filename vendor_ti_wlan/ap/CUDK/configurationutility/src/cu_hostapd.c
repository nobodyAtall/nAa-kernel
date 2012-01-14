/*
 * cu_hostapd.c
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

#include <stdio.h>

#include "cu_osapi.h"
#include "convert.h"
#include "cu_common.h"
#include "cu_os.h"
#include "ipc_event.h"
#include "oserr.h"
#include "cu_hostapd.h"

static TDictionary  *pDictionary = NULL;


static int  IsLineContainsKey(TI_UINT8 *pLine);
static void PrintDictionary(TDictionary *pDic);
static void FillDictionary(FILE *pFile,  TDictionary *pDic);
static void FlushDictionaryToFile(TI_UINT8 *fileName, TDictionary *pDic);
static void GetKeyValueFromLine(TI_UINT8 *pLine, TKeyValuePair *pKeyValPair);
static void CopyFile(TI_UINT8 *filePathSource, TI_UINT8 *filePathDest);
static TI_BOOL IsKeyInDictionary(keyInfo *pKeyInfo);
static char* GetValueByKeyName(char *pKeyName, TI_BOOL *refKeyModified);
static void removeKey( TI_UINT8 index);


void CuHostapd_PrintMenu(void)
{
	int  i;
	char *pKeyDesc;
	TI_BOOL keyInFile;

	for (i =0 ; i< HOSTAPD_PARAM_LAST ; i++) {
		keyInFile = IsKeyInDictionary(&hostapdKeysList[i]);
		printf("%d. %s => %s %s\n", (i+1), hostapdKeysList[i].keyName,
		       &hostapdKeysList[i].keyDescription, keyInFile ? " ":"N/A") ;
	}
}

static char* GetValueByKeyName(char *pKeyName, TI_BOOL *refKeyModified)
{
	int i;

	for (i =0 ; i< pDictionary->uNumOfKeysFound ; i++) {
		if (0 == os_strcmp((PS8)pDictionary->keys[i].key, pKeyName)) {
			*refKeyModified = pDictionary->aKeyModifiedFlag[i];
			return  (PS8)pDictionary->keys[i].value;
		}
	}

	*refKeyModified = TI_FALSE;
	return "N/A";
}

static TI_BOOL IsKeyInDictionary(keyInfo *pKeyInfo)
{
	int i;

	for (i =0 ; i< pDictionary->uNumOfKeysFound ; i++) {
		if (0 == os_strcmp((PS8)pDictionary->keys[i].key, (PS8)pKeyInfo->keyName)) {
			return TI_TRUE;
		}
	}

	return TI_FALSE;
}

void CuHostapd_SaveChanges(void)
{
	FlushDictionaryToFile((TI_UINT8*)HOSTAPD_FILE_NAME_ORIGINAL, pDictionary);
	/*IpcEvent_ReconfigHostapd();*/
}

void CuHostapd_ShowStatus(void)
{
	int     i;
	char    *pVal = NULL;
	TI_BOOL bKeyWasModified = TI_FALSE;

	for (i =0 ; i < HOSTAPD_PARAM_LAST ; i++) {
		pVal = GetValueByKeyName(hostapdKeysList[i].keyName, &bKeyWasModified);
		printf("%d. %s%s = %s\n", (i+1), (bKeyWasModified == TI_TRUE? "*" : "") ,
		       hostapdKeysList[i].keyName, pVal) ;
	}

	printf("\n *key was modified but has not been applied yet. \n");
}

void CuHostapd_UpdateKeyInDictionary(TKeyValuePair *pKeyVal)
{
	int i;
	TI_BOOL isFound = TI_FALSE;

	for (i=0 ; i< pDictionary->uNumOfKeysFound ; i++) {
		if (os_strcmp((PS8)pDictionary->keys[i].key, (PS8)pKeyVal->key) == 0) { /* if key found store it in dictionary */

			if( pKeyVal->uValueLength == 0 || pKeyVal->uValueLength > MAX_VALUE_LENGTH) {
				/* Remove record */
				printf("Remove key(%d) = %s, value = %s\n", i, pDictionary->keys[i].key, pDictionary->keys[i].value);
				removeKey(i);
			} else {
				// printf("Key to update found: old value = %s, NewValue = %s \n",pDictionary->keys[i].value, pKeyVal->value);
				os_memcpy((PVOID)pDictionary->keys[i].value, (PVOID)pKeyVal->value, pKeyVal->uValueLength);
				pDictionary->keys[i].value[pKeyVal->uValueLength] = '\0';
				pDictionary->keys[i].uValueLength = pKeyVal->uValueLength;
				pDictionary->aKeyModifiedFlag[i] = TI_TRUE;
			}
			return;
		}
	}
	/* Key was not found in dictionary. Try to find key in hostapdKeysList */
	for(i = 0; i < HOSTAPD_PARAM_LAST; i++) {
		if (0 == os_strcmp((PS8)hostapdKeysList[i].keyName, (PS8)pKeyVal->key)) {
			isFound = TI_TRUE;
			break;
		}
	}
	if(isFound) {
		if( pKeyVal->uValueLength == 0 ||  pKeyVal->uValueLength > MAX_VALUE_LENGTH) {
			printf("Illegal value!!!\n");
			return;
		}
		/* Add new record to dictionary */

		i = pDictionary->uNumOfKeysFound;

		os_memcpy((PVOID)pDictionary->keys[i].key, (PVOID)pKeyVal->key, (PVOID)pKeyVal->uKeyLength);

		pDictionary->keys[i].key[pKeyVal->uKeyLength] = '\0';
		os_memcpy((PVOID)pDictionary->keys[i].value, (PVOID)pKeyVal->value, pKeyVal->uValueLength);
		pDictionary->keys[i].value[pKeyVal->uValueLength] = '\0';
		pDictionary->keys[i].uValueLength = pKeyVal->uValueLength;
		pDictionary->aKeyModifiedFlag[i] = TI_TRUE;
		pDictionary->uNumOfKeysFound++;

		printf("New record <<%s=%s>> was added to hostapd configuration.\n",pDictionary->keys[i].key,pDictionary->keys[i].value);
	} else {
		printf("Error! The '%s' Key was not found or disabled! \n", pKeyVal->key);
	}

}

void CuHostapd_Destroy (void)
{
	printf("\n ***** Destroying Hostapd!!  ***** \n");
	os_MemoryFree(pDictionary);
}

void CuHostapd_LoadConfFileToMemory (void)
{
	FILE	 *pFile;


	/* Duplicate the original file and work on it only from now on */
	CopyFile((TI_UINT8*)HOSTAPD_FILE_NAME_ORIGINAL, (TI_UINT8*)HOSTAPD_FILE_NAME_TEMP);

	pFile = os_fopen (HOSTAPD_FILE_NAME_ORIGINAL, OS_FOPEN_READ);

	if (pFile == NULL) {
		perror ("\nError opening file\n");
		return;
	} else {
		pDictionary = os_MemoryAlloc(sizeof(TDictionary));
		FillDictionary(pFile, pDictionary);
		os_fclose ((PVOID)pFile);
	}
}


static void FillDictionary(FILE *pFile,  TDictionary *pDic)
{
	TI_UINT8	 line[100];
	TI_UINT8	 i=0;

	pDic->uNumOfKeysFound = 0;

	while ( os_fgets(line, 99, (PVOID)pFile) != NULL ) {
		if (IsLineContainsKey(line)) {
			GetKeyValueFromLine(line, &pDic->keys[i]);
			pDic->uNumOfKeysFound++;
			i++;
		}
	}
}


static void PrintDictionary(TDictionary *pDic)
{
	int i;

	printf("Dictionary Keys Values list:\n");
	for (i=0 ; i< pDic->uNumOfKeysFound ; i++) {
		printf("%d. %s = %s  \n", i, pDic->keys[i].key ,pDic->keys[i].value);
	}
}


static int IsLineContainsKey(TI_UINT8 *pLine)
{

	if ((*pLine >= 'a' && *pLine <= 'z') || (*pLine >= 'A' && *pLine <= 'Z')) {
		return 1;
	}

	return 0;

};


static void GetKeyValueFromLine(TI_UINT8 *pLine, TKeyValuePair *pKeyValPair)
{

	TI_UINT8 endOfLineDelimiter = 10;
	TI_UINT8 *pTI_UINT8 = pLine;
	TI_UINT8 i=0;


	pKeyValPair->uKeyLength = 0;
	pKeyValPair->uValueLength = 0;


	while (*pTI_UINT8 != '=') {
		pKeyValPair->key[i] = *pTI_UINT8;
		pKeyValPair->uKeyLength++;
		pTI_UINT8 ++;
		i++;
	}

	pKeyValPair->key[i] = '\0';
	pTI_UINT8++;

	i=0;
	while (*pTI_UINT8 != endOfLineDelimiter) {
		pKeyValPair->value[i] = *pTI_UINT8;
		pKeyValPair->uValueLength++;
		pTI_UINT8++;
		i++;
	}

	pKeyValPair->value[i] = '\0';
}




static void FlushDictionaryToFile(TI_UINT8 *fileName, TDictionary *pDic)
{
	FILE	*pFile;
	int i=0;

	TI_UINT8 *header = "\n\n#####################  HOSTAPD Configuration File - TEXAS INSTRUMENTS #####################\n";

	pFile = os_fopen (fileName, OS_FOPEN_WRITE);
	if(pFile == NULL) {
		printf("Can't open file %s\n", fileName);
		return;
	}
	fprintf(pFile, "%s\n\n", header);

	for (i=0 ; i< pDic->uNumOfKeysFound ; i++) {
		fprintf(pFile, "%s=%s\n", pDic->keys[i].key, pDic->keys[i].value);
	}

	/* After flushing reset the 'Modified' flag */
	for (i =0 ; i < MAX_NUM_OF_KEYS ; i++) {
		pDic->aKeyModifiedFlag[i] = TI_FALSE;
	}

	fclose(pFile);

}


static void CopyFile(TI_UINT8 *filePathSource, TI_UINT8 *filePathDest)
{
	FILE	*pFileSource, *pFileDest;
	int     uFileSize = 0;
	char    *pFileBuffer;


	pFileSource = os_fopen (filePathSource, OS_FOPEN_READ_BINARY);
	pFileDest = os_fopen (filePathDest, OS_FOPEN_WRITE_BINARY);

	if (pFileSource == 0 || pFileDest == 0) {
		printf("\n Error while opening hostapd config file! \n");
		return;
	}

	uFileSize = os_getFileSize(pFileSource);
	pFileBuffer = os_MemoryAlloc(uFileSize);
	os_fread(pFileBuffer, 1, uFileSize, pFileSource);
	os_fwrite(pFileBuffer, 1, uFileSize, pFileDest);

	/* Free the resources */
	os_MemoryFree(pFileBuffer);
	os_fclose((PVOID)pFileDest);
	os_fclose((PVOID)pFileSource);

}

static void removeKey( TI_UINT8 index)
{
	TI_UINT8 i;

	for(i = index; i < pDictionary->uNumOfKeysFound - 1; i++) {
		os_memcpy(&pDictionary->keys[i], &pDictionary->keys[i+1], sizeof(TKeyValuePair));
		pDictionary->aKeyModifiedFlag[i] = pDictionary->aKeyModifiedFlag[i+1];
	}
	pDictionary->uNumOfKeysFound--;
}
