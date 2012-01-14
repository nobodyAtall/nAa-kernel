/*
 * siteHash.c
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

/** \file siteHash.c
 *  \brief Site Hash implementation
 *
 *  \see siteHash.h
 */

/****************************************************************************/
/*																			*/
/*		MODULE:	siteHash.c													*/
/*    PURPOSE:	Site Hash implementation 									*/
/*																			*/
/***************************************************************************/

#define __FILE_ID__  FILE_ID_84
#include "tidef.h"
#include "report.h"
#include "osApi.h"
#include "siteMgrApi.h"
#include "siteHash.h"
#include "smeApi.h"


/****************************************************************************************************************

	This file implements the site hash mechanism. This mechanism is used for faster access to the sites information.
	It is compound of the following:
		1.	hash function	-	which maps the 4 last bits of the BSSID to an entry in the hash table.
		2.	hash table		-	each entry in the table points to a linked list of site entries
		3.	site table		-	each entry holds a site information

	In order to find a site in the site table, we operate the hash function on the site's BSSID.
	We receive a hash entry. We go over the linked list pointed by this hash entry until we find the site entry.
*****************************************************************************************************************/

#define WLAN_NUM_OF_MISSED_SACNS_BEFORE_AGING 2


/********************************************/
/*		Functions Implementations			*/
/********************************************/
/************************************************************************
 *                        siteMgr_resetSiteTable						*
 ************************************************************************
DESCRIPTION: reset the following things:
				-	Mgmt parameters structure
				-	Site table
				-	Hash table
				-	Primary site pointer
				-	Number of sites

INPUT:      hSiteMgr				-	Handle to site mgr


OUTPUT:

RETURN:     TI_OK

************************************************************************/
TI_STATUS siteMgr_resetSiteTable(TI_HANDLE	hSiteMgr, siteTablesParams_t	*pSiteTableParams)
{
	int i;
	siteMgr_t		*pSiteMgr = (siteMgr_t *)hSiteMgr;

	/* It looks like it never happens. Anyway decided to check */
	if ( pSiteTableParams->maxNumOfSites > MAX_SITES_BG_BAND ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return TI_NOK;
	}
	os_memoryZero(pSiteMgr->hOs, &pSiteTableParams->siteTable[0], sizeof(siteEntry_t)*pSiteTableParams->maxNumOfSites);

	for (i = 0; i < pSiteTableParams->maxNumOfSites; i++) {
		pSiteTableParams->siteTable[i].index = i;
		pSiteTableParams->siteTable[i].siteType = SITE_NULL;
		pSiteTableParams->siteTable[i].beaconRecv = TI_FALSE;
		pSiteTableParams->siteTable[i].dtimPeriod = 1;
	}

	pSiteTableParams->numOfSites = 0;

	pSiteMgr->pSitesMgmtParams->pPrimarySite = NULL;

	return TI_OK;
}

/************************************************************************
 *                        findSiteEntry									*
 ************************************************************************
DESCRIPTION: Perform the following things:
			-	Compute the site's hash entry based on the site BSSID and hash function
			-	Look fotr the site entry in the linked list pointed by the hash entry
			-	If the site is found in the site table, returns a pointer to the site entry
			-	If the site is not found, return NULL.

INPUT:      pSiteMgr	-	Handle to site mgr
			mac			-	The site BSSID


OUTPUT:

RETURN:     Pointer to the site entry if site found, NULL otherwise

************************************************************************/
siteEntry_t	*findSiteEntry(siteMgr_t		*pSiteMgr,
                           TMacAddr 		*mac)
{
	siteTablesParams_t      *pCurrentSiteTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;
	siteEntry_t             *pSiteEntry;
	TI_UINT8                 tableIndex=2, i;

	/* It looks like it never happens. Anyway decided to check */
	if ( pCurrentSiteTable->maxNumOfSites > MAX_SITES_BG_BAND ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return NULL;
	}

	do {
		tableIndex--;
		for (i = 0; i < pCurrentSiteTable->maxNumOfSites; i++) {
			pSiteEntry = &(pCurrentSiteTable->siteTable[i]);

			if (MAC_EQUAL (pSiteEntry->bssid, *mac)) {
				return pSiteEntry;
			}

		}
		if ((pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE) &&
		        (tableIndex==1)) {  /* change site table */
			if (pCurrentSiteTable == &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables) {
				pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
			} else {
				pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
			}
		}

	} while (tableIndex>0);





	return NULL;
}

/************************************************************************
 *                        findAndInsertSiteEntry									*
 ************************************************************************
DESCRIPTION: Perform the following things:
			-	Compute the site's hash entry based on the site BSSID and hash function
			-	Look for the site entry in the linked list pointed by the hash entry
			-	If the site is found in the site table, returns a pointer to the site entry
			-	If the site is not found in the site table, tries to add the site
				-	If succeeds, returns a pointer to the site entry
				-	Otherwise, returns NULL

INPUT:      pSiteMgr	-	Handle to site mgr
			mac			-	The site BSSID
            band        -   The site band


OUTPUT:

RETURN:     Pointer to the site entry if site found/inserted, NULL otherwise

************************************************************************/
siteEntry_t	*findAndInsertSiteEntry(siteMgr_t		*pSiteMgr,
                                    TMacAddr    	*mac,
                                    ERadioBand      band)
{
	TI_UINT8             i, emptySiteIndex=0, nextSite2Remove=0;
	siteEntry_t         *pSiteEntry, *pPrimarySite=pSiteMgr->pSitesMgmtParams->pPrimarySite;
	sitesMgmtParams_t   *pSitesMgmtParams  = pSiteMgr->pSitesMgmtParams;
	siteTablesParams_t  *pCurrentSiteTable;
	TI_BOOL              firstEmptySiteFound = TI_FALSE;
	TI_UINT32            oldestTS;


	/* choose site table according to AP's band */
	if ( RADIO_BAND_2_4_GHZ == band ) {
		pCurrentSiteTable = &(pSitesMgmtParams->dot11BG_sitesTables);
	} else if (RADIO_BAND_5_0_GHZ == band) {
		pCurrentSiteTable = (siteTablesParams_t*) &(pSitesMgmtParams->dot11A_sitesTables);
	} else {
		pCurrentSiteTable = &(pSitesMgmtParams->dot11BG_sitesTables);
	}

	/* Set the first TS to a site which is not the Primary site */
	if (pPrimarySite != &(pCurrentSiteTable->siteTable[0])) {
		oldestTS = pCurrentSiteTable->siteTable[0].localTimeStamp;
	} else {
		oldestTS = pCurrentSiteTable->siteTable[1].localTimeStamp;
	}
	/* It looks like it never happens. Anyway decided to check */
	if ( pCurrentSiteTable->maxNumOfSites > MAX_SITES_BG_BAND ) {
		handleRunProblem(PROBLEM_BUF_SIZE_VIOLATION);
		return NULL;
	}
	/* Loop all the sites till the desired MAC is found */
	for (i = 0; i < pCurrentSiteTable->maxNumOfSites; i++) {
		pSiteEntry = &(pCurrentSiteTable->siteTable[i]);

		if (MAC_EQUAL (pSiteEntry->bssid, *mac)) {

			return pSiteEntry;
		} else if (pSiteEntry->siteType == SITE_NULL) {   /* Save the first empty site, in case the
            desired MAC is not found */
			if (!firstEmptySiteFound) {
				emptySiteIndex = i;
				firstEmptySiteFound=TI_TRUE;
			}

		} else if (oldestTS == pSiteEntry->localTimeStamp) {  /* Save the oldest site's index, according to TS */
			nextSite2Remove = i;
		}
	}

	if ((!firstEmptySiteFound) || (pCurrentSiteTable->numOfSites>=pCurrentSiteTable->maxNumOfSites)) {
		/* No NULL entry has been found. Remove the oldest site */
		pSiteEntry =  &(pCurrentSiteTable->siteTable[nextSite2Remove]);
		removeSiteEntry(pSiteMgr, pCurrentSiteTable, pSiteEntry);
		emptySiteIndex = nextSite2Remove;

	}


	pCurrentSiteTable->numOfSites++;

	pSiteEntry = &(pCurrentSiteTable->siteTable[emptySiteIndex]);

	/* fill the entry with the station mac */
	MAC_COPY (pSiteEntry->bssid, *mac);

	/* Some parameters have to be initialized immediately after entry allocation */

	if (pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
		pSiteEntry->currentSlotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;



	return pSiteEntry;
}

/************************************************************************
 *                        removeSiteEntry								*
 ************************************************************************
DESCRIPTION: Removes the site entry from the site table

INPUT:      pSiteMgr		   - Handle to site mgr
            pCurrSiteTblParams - Pointer to current site table parameters
            hashPtr			   - Pointer to the site entry


OUTPUT:

RETURN:

************************************************************************/
void removeSiteEntry(siteMgr_t  *pSiteMgr,
                     siteTablesParams_t  *pCurrSiteTblParams,
                     siteEntry_t         *pSiteEntry)
{
	TI_UINT8			index;

	if (pSiteEntry == NULL) {
		return;
	}

	if (pCurrSiteTblParams->numOfSites == 0) {
		return;
	}


	pCurrSiteTblParams->numOfSites--;

	/* Clean the rest of the entry structure */
	index = pSiteEntry->index;     /* keep the index of the siteTable entry */
	os_memoryZero(pSiteMgr->hOs, pSiteEntry, sizeof(siteEntry_t));

	/* This is not required!!!! - Remove!!*/
	pSiteEntry->dtimPeriod = 1;
	pSiteEntry->siteType = SITE_NULL;
	pSiteEntry->index = index;   /* restore the index of the siteTable */

	/* if removing previous primary site - update the link */
	if (pSiteEntry == pSiteMgr->pSitesMgmtParams->pPrevPrimarySite) {
		pSiteMgr->pSitesMgmtParams->pPrevPrimarySite = NULL;
	}

	return;
}

