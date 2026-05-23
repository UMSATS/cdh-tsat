/*
 * StorageManager.c
 *
 *  Created on: Jan 13, 2026
 *      Author: adriv
 */

//
// INCLUDES
#include "../Inc/StorageManager.h"

#include <time.h>
#include <string.h>
#include <stdlib.h>

#include <Dhara_Wrapper.h>

#include "utils.h"
#include "telemetry.h"


//
// DEFINES
const uint8_t STORAGE_MAGIC=42;
//static uint8_t sm_read_buf[PAGESIZE];
static uint8_t sm_write_buf[PAGESIZE];

// Function Declarations
dhara_sector_t Storage_Find_Empty_Sector();
int Storage_Add_SectorNode(DataType dataType, uint8_t sectorType, dhara_sector_t sector, int32_t sequence);

//###############################################
//##############    SECTOR TYPE    ##############
//###############################################

// Starts at 0 for ACTIVE and anything greater is a the backup number, TODO write better

//
// HEADER STRUCT
typedef struct {
	uint8_t magic;

	uint8_t dataType;
	uint8_t sectorType;

	// 1 is the first in sequence of sectors
	uint32_t sequence;

	uint16_t offset;


}SectorHeader;

//#####################################################
//##############    DATA TYPE CONFIGS    ##############
//#####################################################

#define NUM_TYPES 4
#define MAX_BACKUPS 2


const uint32_t NUM_OF_BACKUPS[NUM_TYPES] = {
		// TYPE     // NUMBER OF BACKUPS
		[RAW]          =0,
		[TELEM]        =2,
		[LOG]          =1,
		[FIRMWARE]     =0
};

SectorNode* SECTOR_LIST_HEAD[NUM_TYPES][MAX_BACKUPS + 1] = {
    [RAW] = {NULL},
    [TELEM] = {NULL},
    [LOG] = {NULL},
    [FIRMWARE] = {NULL}
};


//#############################################
//##############    FUNCTIONS    ##############
//#############################################

int Storage_Init(){

	//dhara_sector_t capacity = Dhara_Capacity();
	dhara_sector_t capacity = 100;

	for(dhara_sector_t i=0;i<capacity;i++){

		dhara_error_t err=DHARA_E_NONE;
		SectorHeader hdr;

		//
		// READ HEADER
		Dhara_Read(i, (uint8_t*)&hdr, sizeof(SectorHeader), &err);
		if(err){ continue;}

		// Checks if Magic exist in header
		if(hdr.magic!=STORAGE_MAGIC){
			Dhara_Erase(i, &err);
			continue;
		}

		// Checks if valid DataType
		if(hdr.dataType >= NUM_TYPES){
			Dhara_Erase(i,&err);
			continue;
		}



		// Checks if valid SectorType
		if(hdr.sectorType<=NUM_OF_BACKUPS[hdr.dataType]){

			Storage_Add_SectorNode(hdr.dataType, hdr.sectorType, i, hdr.sequence);

		}else{
			Dhara_Erase(i, &err);
			continue;
		}

	}
	return 1;
}

int Storage_Write(const DataType dType, const uint16_t sequence, const uint8_t *data, const uint16_t dataSize){

	// Checks if valid DataType
	if (dType < 0 || dType >= NUM_TYPES)
		{ return -1; }

	// SECTOR_HEAD DOESN'T EXIST
	if(SECTOR_LIST_HEAD[dType][0]==NULL){
		dhara_sector_t sector = Storage_Find_Empty_Sector();

		Storage_Add_SectorNode(dType, 0, sector, 0);
	}

	// Checks if sequence exists
	if(sequence>SECTOR_LIST_HEAD[dType][0]->sequence)
		{ return -2; }

	// Searching for sequence in list
	SectorNode* cur=SECTOR_LIST_HEAD[dType][0];
	while(cur!=NULL){

		if(cur->sequence==sequence){
			break;
		}

		cur=cur->nextSector;
	}
	if(cur==NULL)
		{ return -3; }

	// Checks if dataSize is Too Large
	if(dataSize+sizeof(SectorHeader)>PAGESIZE)
		{ return -4; }



	dhara_error_t err=DHARA_E_NONE;

	// data array
	uint8_t *mData = sm_write_buf;



	memset(mData, 0xFF, PAGESIZE);

	// Fetch header
	SectorHeader *hdr = (SectorHeader*)mData;

	// Setup header
	hdr->dataType=dType;
	hdr->magic=STORAGE_MAGIC;
	hdr->offset=dataSize;
	hdr->sectorType=0;
	hdr->sequence=sequence;

	// Append data to array
	memcpy(mData + sizeof(SectorHeader),data,dataSize);


	// Write to Sector
	Dhara_Write(cur->s, mData, PAGESIZE, &err);
	if(err) { return -5; }


	// TODO figure out when to call sync
	Dhara_Force_Sync(&err);
	if (err) {  return -3;  }// Sync fail


	// Great Success!
	return 0;
}

int Storage_Append(const DataType dType, const uint8_t *data, const uint16_t dataSize){

	//
	//  ERROR CHECKS
	//

	// Checks if valid DataType
	if (dType < 0 || dType >= NUM_TYPES)
	    return -1;

	// Checks if dataSize is Too Large
	if(dataSize+sizeof(SectorHeader)>PAGESIZE)
		return -2;

	// Checks if SECTOR_LIST_HEAD[type][0] exists
	if(SECTOR_LIST_HEAD[dType][0]==NULL){
		dhara_sector_t sector = Storage_Find_Empty_Sector();

		Storage_Add_SectorNode(dType, 0, sector, 0);
	}


	//
	//  Actual Function
	//


	// Reading Data On Flash
	dhara_error_t err=DHARA_E_NONE;
	uint8_t *mData = sm_write_buf;

	Dhara_Read(SECTOR_LIST_HEAD[dType][0]->s, mData, PAGESIZE, &err);
	if(err) return -3;

	// Locating Header Info
	SectorHeader *hdr = (SectorHeader*)mData;

	//
	// Checks if header is contained on sector
	if(hdr->magic!=STORAGE_MAGIC){

		Dhara_Erase(SECTOR_LIST_HEAD[dType][0]->s, &err);
		if(err) return -6;

		// Setup hdr values
		hdr->magic=STORAGE_MAGIC;
		hdr->dataType=dType;
		hdr->sectorType=0;
		hdr->sequence=SECTOR_LIST_HEAD[dType][0]->sequence;
		hdr->offset=0;

	}



	//
	// Checking if Sector has room for new Data
	if (PAGESIZE - sizeof(SectorHeader) - hdr->offset >= dataSize){

		// Append data to array
		memcpy(mData + sizeof(SectorHeader) + hdr->offset,data,dataSize);

		// Update offset
		hdr->offset += dataSize;

		// Write back
		Dhara_Write(SECTOR_LIST_HEAD[dType][0]->s, mData, PAGESIZE, &err);
		if(err) return -5;

	//
	// Not Enough Room in Sector for new Data
	}else{

		// Finding New Sector
		dhara_sector_t emptySector=Storage_Find_Empty_Sector();
		if(emptySector==INVALID_SECTOR) return -4;


		// Switch To New Sector
		Storage_Add_SectorNode(dType, 0, emptySector, SECTOR_LIST_HEAD[dType][0]->sequence+1);
		memset(mData, 0xFF, PAGESIZE);


		// Fetch header
		SectorHeader *hdr = (SectorHeader*)mData;


		// Setup header
		hdr->dataType=dType;
		hdr->magic=STORAGE_MAGIC;
		hdr->offset=dataSize;
		hdr->sectorType=0;
		hdr->sequence=SECTOR_LIST_HEAD[dType][0]->sequence;

		// Append data to array
		memcpy(mData + sizeof(SectorHeader),data,dataSize);

		// Write to Sector
		Dhara_Write(SECTOR_LIST_HEAD[dType][0]->s, mData, PAGESIZE, &err);
		if(err) return -5;

	}


	// TODO figure out when to call sync
	Dhara_Force_Sync(&err);
	if (err) {  return -3;  }// Sync fail


	// Great Success!
	return 0;
}

int Storage_Send_To_Backup(const DataType dType){

	// Checks if valid DataType
	if (dType < 0 || dType >= NUM_TYPES)
		{ return -1; }


	int32_t backups = NUM_OF_BACKUPS[dType];

	for(int32_t sType = backups; sType >= 0; sType--){

		if(sType == backups){

			Storage_Delete_Sector_List(dType, sType);

		}else{

			SectorNode* cur = SECTOR_LIST_HEAD[dType][sType];

			SECTOR_LIST_HEAD[dType][sType + 1] = cur;
			SECTOR_LIST_HEAD[dType][sType] = NULL;

			while(cur != NULL){

				dhara_error_t err=DHARA_E_NONE;
				uint8_t *mData = sm_write_buf;

				Dhara_Read(cur->s, mData, PAGESIZE, &err);
				if(err) return -2;

				SectorHeader *hdr = (SectorHeader*)mData;

				hdr->sectorType++;

				Dhara_Write(cur->s, mData, PAGESIZE, &err);
				if(err) return -3;

				cur = cur->nextSector;
			}
		}
	}

	return 0;
}

int Storage_Get_SectorNode(const DataType dType, const uint8_t sType, const uint16_t sequence, SectorNode** sectorNode){

	// Checks if valid DataType
	if (dType < 0 || dType >= NUM_TYPES)
		return -1;

	// checking if address pointer is null
	if(sectorNode==NULL)
		return -2;

	// Checks if sequence exists
	if(sequence>SECTOR_LIST_HEAD[dType][sType]->sequence)
		return -3;

	// Checks if sType exists
	if(sType>NUM_OF_BACKUPS[dType])
		return -4;


	// Searching List for correct sequence
	SectorNode* cur=SECTOR_LIST_HEAD[dType][sType];

	while(cur!=NULL){

		if(cur->sequence==sequence){
			(*sectorNode)=cur;
			return 0;
		}

		cur=cur->nextSector;
	}

	return -5;
}

int Storage_Read(const SectorNode* sectorNode, uint8_t* data, uint32_t dataSize){

	// Checks if sectorNode exists
	if (sectorNode==NULL)
		return -1;

	// Checks if data can fit in one page
	if(dataSize>PAGESIZE)
		return -2;


	//
	// READING SECTOR DATA
	dhara_error_t err=DHARA_E_NONE;

	Dhara_Read(sectorNode->s, data, dataSize, &err);
	if (err) {  return -3;  }

	return 0;
}

int Storage_Delete_Sector(DataType dType, uint8_t sType, const uint16_t sequence){

	// Checks if valid DataType
	if (dType < 0 || dType >= NUM_TYPES)
		return -1;

	// Checks if sType exists
	if(sType>NUM_OF_BACKUPS[dType])
		return -2;

	// Checks if list head exists
	if(SECTOR_LIST_HEAD[dType][sType]==NULL)
		return -3;

	// Checks if sequence exists
	if(sequence>SECTOR_LIST_HEAD[dType][sType]->sequence)
		return -4;


	dhara_error_t err=DHARA_E_NONE;
	SectorNode* cur=SECTOR_LIST_HEAD[dType][sType];

	// Checks if cur sequence is the desired sequence
	if(cur->sequence==sequence){
		SECTOR_LIST_HEAD[dType][sType]=cur->nextSector;

		Dhara_Erase(cur->s, &err);
		if (err) {  return -5;  }

		free(cur);
	}else{

		while(cur->nextSector!=NULL && cur->sequence>=sequence){
			SectorNode* nextCur=cur->nextSector;
			cur->sequence=cur->sequence-1;

			if(nextCur->sequence==sequence){
				cur->nextSector=nextCur->nextSector;

				Dhara_Erase(nextCur->s, &err);
				if (err) {  return -5;  }

				free(nextCur);

				break;
			}

			cur=nextCur;
		}

	}



	return 0;
}

int Storage_Delete_Sector_List(DataType dType, uint8_t sType){

	// Checks if valid DataType
	if (dType < 0 || dType >= NUM_TYPES)
		return -1;

	// Checks if sType exists
	if(sType>NUM_OF_BACKUPS[dType])
		return -2;

	// Checks if list head exists
	if(SECTOR_LIST_HEAD[dType][sType]==NULL)
		return -3;

	dhara_error_t err=DHARA_E_NONE;
	SectorNode* cur=SECTOR_LIST_HEAD[dType][sType];

	while(cur!=NULL){
		SectorNode* temp=cur;
		cur=cur->nextSector;

		Dhara_Erase(temp->s, &err);
		if (err) {  return -4;  }

		free(temp);
	}

	SECTOR_LIST_HEAD[dType][sType]=NULL;

	return 0;
}

int Storage_Erase(const SectorNode* sectorNode){

    dhara_error_t err = DHARA_E_NONE;
    SectorHeader hdr;

    Dhara_Read(sectorNode->s, (uint8_t*)&hdr, sizeof(SectorHeader), &err);
    if(err) return -1;

    hdr.offset = 0;

    Dhara_Write(sectorNode->s, (uint8_t*)&hdr, sizeof(SectorHeader), &err);
    if(err) return -2;

    return 0;
}

//###################################################
//##############    HELPER FUNCTIONS    #############
//###################################################

/*
 * FUNCTION: Storage_Find_Empty_Sector
 *
 * DESCRIPTION: Finds first empty sector and returns it
 *
 *
 * RETURNS:
 * 		empty .
*/
dhara_sector_t Storage_Find_Empty_Sector(){

	for(dhara_sector_t i=0;i<Dhara_Capacity();i++){

		dhara_error_t err=DHARA_E_NONE;
		SectorHeader hdr = {0};

		Dhara_Read(i, (uint8_t *)&hdr, sizeof(hdr), &err);
		if (err) continue;

		if(hdr.magic!=STORAGE_MAGIC){
			Dhara_Erase(i, &err);
			if(err){ continue;}

			return i;
		}
	}

	return INVALID_SECTOR;
}

/*
 * FUNCTION: Storage_Add_SectorNode
 *
 * DESCRIPTION: Adds node in order from greater to smallest
 *
 *
 * RETURNS:
 * 		0 if no error
 * 		-1 if memory allocation failed
*/
int Storage_Add_SectorNode(DataType dataType, uint8_t sectorType, dhara_sector_t sector, int32_t sequence){
	SectorNode* newNode=malloc(sizeof(SectorNode));
	if(newNode == NULL)
	    return -1;

	newNode->s=sector;
	newNode->sequence=sequence;
	newNode->nextSector=NULL;

	SectorNode* cur=SECTOR_LIST_HEAD[dataType][sectorType];

	// If List Is Empty
	if(cur==NULL){
		SECTOR_LIST_HEAD[dataType][sectorType]=newNode;
		return 0;
	}

	// TODO Handle if sequence duplication
	while(cur!=NULL){

		// Is newNode greater then cur
		if(cur->sequence<sequence){
			newNode->nextSector=cur;
			SECTOR_LIST_HEAD[dataType][sectorType]=newNode;

			return 0;
		}

		// If nextNode is NULL
		if(cur->nextSector==NULL){
			cur->nextSector=newNode;

			return 0;
		}


		cur=cur->nextSector;
	}

	return 0;
}

//#############################################
//##############    UNIT TEST    ##############
//#############################################

int Storage_Unit_Test(uint8_t* data, uint32_t dataSize){

	uint8_t final[PAGESIZE];
	for(uint32_t i=0;i<PAGESIZE;i++){
		final[i]=255;
	}

	// Fetch header
	SectorHeader *hdr = (SectorHeader*)final;


	// Setup header
	hdr->dataType=TELEM;
	hdr->magic=STORAGE_MAGIC;
	hdr->offset=dataSize;
	hdr->sectorType=0;
	hdr->sequence=0;

	// If given data is too large
	if(dataSize>PAGESIZE-sizeof(SectorHeader)){
		return -1;
	}

	memcpy(final+sizeof(SectorHeader), data, dataSize);

	//
	//    WRITE TEST
	//

	// Deleting active list of the TELEM type
	Storage_Delete_Sector_List(TELEM, 0);
	{
		// Writing data to TELEM storage
		Storage_Write(TELEM, 0, data, dataSize);

		uint8_t readData[PAGESIZE];
		SectorNode* sectorNode = NULL;
		// Fetch head of link list
		Storage_Get_SectorNode(TELEM, 0, 0, &sectorNode);

		// Read data associated with link list node
		Storage_Read(sectorNode, readData, PAGESIZE);

		// Checking if data is the same as final
		if(memcmp(final,readData,PAGESIZE)!=0){
			return -2;
		}
	}

	//
	//    APPEND TEST
	//

	// Deleting active list of the TELEM type
	Storage_Delete_Sector_List(TELEM, 0);
	{
		// Appending data to TELEM storage
		Storage_Append(TELEM, data, dataSize);

		uint8_t readData[PAGESIZE];
		SectorNode* sectorNode = NULL;
		// Fetch head of link list
		Storage_Get_SectorNode(TELEM, 0, 0, &sectorNode);

		// Read data associated with link list node
		Storage_Read(sectorNode, readData, PAGESIZE);

		// Checking if data is the same as final
		if(memcmp(final,readData,PAGESIZE)!=0){
			return -2;
		}
	}

	//
	//    BACKUP CHECK
	//

	// Sending active data list to backup
	Storage_Send_To_Backup(TELEM);
	{ // Reading backup
		hdr->sectorType=1;
		uint8_t readData[PAGESIZE];
		SectorNode* sectorNode = NULL;
		// Fetch head of link list
		Storage_Get_SectorNode(TELEM, 1, 0, &sectorNode);

		// Read data associated with link list node
		Storage_Read(sectorNode, readData, PAGESIZE);

		// Checking if data is the same as final
		if(memcmp(final,readData,PAGESIZE)!=0){
			return -2;
		}
	}
	{ // Reading active (Should be different)
		hdr->sectorType=0;// TODO find backup error
		uint8_t readData[PAGESIZE];
		SectorNode* sectorNode = NULL;
		// Fetch head of link list
		Storage_Get_SectorNode(TELEM, 0, 0, &sectorNode);

		// Read data associated with link list node
		Storage_Read(sectorNode, readData, PAGESIZE);

		// Checking if data is the same as final
		if(memcmp(final,readData,PAGESIZE)==0){
			return -2;
		}
	}

	return 1;
}
