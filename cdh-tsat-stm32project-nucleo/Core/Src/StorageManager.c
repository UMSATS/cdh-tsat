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

#include <Dhara_Wrapper.h>

#include "utils.h"
#include "telemetry.h"



//
// DEFINES
const uint8_t STORAGE_MAGIC=42;

dhara_sector_t Find_Empty_Sector();


//###############################################
//##############    SECTOR TYPE    ##############
//###############################################

//
// SECTOR TYPES
typedef enum {
	ACTIVE=0b00000000,
	BACKUP=0b00001111
}SectorType;

// Backup Sector Helper Macro
#define MAKE_SECTOR_TYPE(group, type) \
        ((((group) & 0x0F) << 4) | ((type) & 0x0F))

#define GET_SECTOR_TYPE(x)   ((x) & 0x0F)
#define GET_BACKUP_GROUP(x)  (((x) & 0xF0) >> 4)

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

/*
 * Desc: Most data types have two sector types, one is the active sectors in which we are currently writing to, the other
 * 		is the backup sectors.
 * 		The backup system is currently very simple, the BackupCount variable will just hold how many backup sector chunks(previous
 * 		active sector chunk), we do not care how many there are.
 *
 * 		Inside the header of each sector will contain the magic, which is just a number to verify that the sector is valid.
 * 		second is the dataType, this will be raw,telem,log or firm.
 * 		third is the sectorType, this is if the sector is a backup sector or an active sector
 * 		forth is the sequence, this is the order number for the sector within the sector chunk
 * 		fifth is the data offset, where the end of the data is in the sector
 *
 * 	Note: One thing about the Sector types is that for the backup type it will use the first 4 bits as the backup count and the last 4 bits
 * 		as the active sector define, so 00011111 is the backup number 1.
*/


#define NUM_TYPES 4

// RAW


// TELEM
#define TEL_NUM_OF_BACKUPS 2


// LOG
#define LOG_NUM_OF_BACKUPS 1

// FIRMWARE
#define FIRM_NUM_OF_BACKUPS 0


dhara_sector_t largestSector=0;

dhara_sector_t TYPE_SECTORS[NUM_TYPES] = {
		// TYPE     // SECTOR
		[RAW]          =INVALID_SECTOR,
		[TELEM]        =INVALID_SECTOR,
		[LOG]          =INVALID_SECTOR,
		[FIRMWARE]     =INVALID_SECTOR
};

dhara_sector_t TYPE_SEQUENCE[NUM_TYPES] = {
		// TYPE     // SECTOR
		[RAW]          =0,
		[TELEM]        =0,
		[LOG]          =0,
		[FIRMWARE]     =0
};


//#############################################
//##############    FUNCTIONS    ##############
//#############################################

int Storage_Init(){

	for(dhara_sector_t i=0;i<Dhara_Capacity();i++){

		dhara_error_t err;
		SectorHeader hdr;



		//
		// READ HEADER
		Dhara_Read(i, (uint8_t*)&hdr, sizeof(SectorHeader)-sizeof(dhara_sector_t), &err);
		if(err){ continue;}

		if(hdr.magic!=STORAGE_MAGIC){
			Dhara_Erase(i, &err);
			continue;
		}

		largestSector=i;

		//
		// TYPE SWITCH
		switch (hdr.dataType){

		//
		// RAW DATA TYPE
		case RAW:

			//
			// ACTIVE SECTOR
			if(hdr.sectorType==ACTIVE){

				if(TYPE_SEQUENCE[RAW]<hdr.sequence){
					TYPE_SEQUENCE[RAW]=hdr.sequence;
					TYPE_SECTORS[RAW]=i;
				}


			//
			// Invalid SECTOR
			}else{

				Dhara_Erase(i, &err);
				if(err){ continue;}

			}

			break;

		//
		// TELEMETRY DATA TYPE
		case TELEM:

			//
			// ACTIVE SECTOR
			if(hdr.sectorType==ACTIVE){

				if(TYPE_SEQUENCE[TELEM]<hdr.sequence){
					TYPE_SEQUENCE[TELEM]=hdr.sequence;
					TYPE_SECTORS[TELEM]=i;
				}

			//
			// BACKUP SECTOR
			}else if(GET_SECTOR_TYPE(hdr.sectorType)==BACKUP){

				if(GET_BACKUP_GROUP(hdr.sectorType)<TEL_NUM_OF_BACKUPS){
					// manage backups

				}else{

					Dhara_Erase(i, &err);
					if(err){ continue;}

				}

			//
			// Invalid SECTOR
			}else{

				Dhara_Erase(i, &err);
				if(err){ continue;}

			}

			break;

		//
		// LOG DATA TYPE
		case LOG:

			//
			// ACTIVE SECTOR
			if(hdr.sectorType==ACTIVE){

				if(TYPE_SEQUENCE[LOG]<hdr.sequence){
					TYPE_SEQUENCE[LOG]=hdr.sequence;
					TYPE_SECTORS[LOG]=i;
				}

			//
			// BACKUP SECTOR
			}else if(GET_SECTOR_TYPE(hdr.sectorType)==BACKUP){

				if(GET_BACKUP_GROUP(hdr.sectorType)<LOG_NUM_OF_BACKUPS){
					// manage backups

				}else{

					Dhara_Erase(i, &err);
					if(err){ continue;}

				}

			//
			// Invalid SECTOR
			}else{

				Dhara_Erase(i, &err);
				if(err){ continue;}

			}

			break;

		//
		// FIRMWARE DATA TYPE
		case FIRMWARE:

			//
			// ACTIVE SECTOR
			if(hdr.sectorType==ACTIVE){

				if(TYPE_SEQUENCE[FIRMWARE]<hdr.sequence){
					TYPE_SEQUENCE[FIRMWARE]=hdr.sequence;
					TYPE_SECTORS[FIRMWARE]=i;
				}

			//
			// BACKUP SECTOR
			}else if(GET_SECTOR_TYPE(hdr.sectorType)==BACKUP){

				if(GET_BACKUP_GROUP(hdr.sectorType)<FIRM_NUM_OF_BACKUPS){
					// manage backups

				}else{

					Dhara_Erase(i, &err);
					if(err){ continue;}

				}

			//
			// Invalid SECTOR
			}else{

				Dhara_Erase(i, &err);
				if(err){ continue;}

			}

			break;

		//
		// DEFAULT CASE
		default:
			largestSector--;
			Dhara_Erase(i, &err);
			if(err){ continue;}

			break;
		}
	}


	return -1;
}

int Storage_Write(const DataType type, const uint16_t sequence, const uint8_t *data, const uint16_t dataSize){

	// Checks for valid DataType
	if (type < 0 || type >= NUM_TYPES)
		return -4;



	dhara_error_t err;

	// data array
	uint8_t mData[PAGESIZE];



	memset(mData, 0xFF, PAGESIZE);

	// Fetch header
	SectorHeader *hdr = (SectorHeader*)mData;

	// Setup header
	hdr->dataType=type;
	hdr->magic=STORAGE_MAGIC;
	hdr->offset=dataSize;
	hdr->sectorType=ACTIVE;
	hdr->sequence=TYPE_SEQUENCE[type];

	// Append data to array
	memcpy(mData + sizeof(SectorHeader),data,dataSize);

	// Write to Sector
	Dhara_Write(TYPE_SECTORS[type], mData, PAGESIZE, &err);
	if(err) return -3;


	// Moves to next number in sequence
	TYPE_SEQUENCE[type]++;



	// Great Success!
	return 0;
}

int Storage_Append(const DataType type, const uint8_t *data, const uint16_t dataSize){

	// Checks for valid DataType
	if (type < 0 || type >= NUM_TYPES)
	    return -4;



	dhara_error_t err;

	// data array
	uint8_t mData[PAGESIZE];



	Dhara_Read(TYPE_SECTORS[type], mData, PAGESIZE, &err);
	if(err) return -1;

	// Fetch header
	SectorHeader *hdr = (SectorHeader*)mData;

	//
	// HAS ROOM
	if (PAGESIZE - sizeof(SectorHeader) - hdr->offset >= dataSize){

		// Append data to array
		memcpy(mData + sizeof(SectorHeader) + hdr->offset,data,dataSize);

		// Update offset
		hdr->offset += dataSize;

		// Write back
		Dhara_Write(TYPE_SECTORS[type], mData, PAGESIZE, &err);
		if(err) return -3;

	}else{
		// Find New Sector
		dhara_sector_t emptySector=Find_Empty_Sector();
		if(emptySector==INVALID_SECTOR) return -2;

		if(emptySector>largestSector){
			largestSector=emptySector;
		}

		// Switch To New Sector
		TYPE_SECTORS[type]=emptySector;

		memset(mData, 0xFF, PAGESIZE);

		// Fetch header
		SectorHeader *hdr = (SectorHeader*)mData;

		// Setup header
		hdr->dataType=type;
		hdr->magic=STORAGE_MAGIC;
		hdr->offset=dataSize;
		hdr->sectorType=ACTIVE;
		hdr->sequence=TYPE_SEQUENCE[type];

		// Append data to array
		memcpy(mData + sizeof(SectorHeader),data,dataSize);

		// Write to Sector
		Dhara_Write(TYPE_SECTORS[type], mData, PAGESIZE, &err);
		if(err) return -3;


		// Moves to next number in sequence
		TYPE_SEQUENCE[type]++;
	}


	// Great Success!
	return 0;
}

int Storage_Send_To_Backup(const DataType type){
	// TODO SET BACKUP ARRAY SIZE TO MATCH CURRENTS ACTIVE COUNT

	// Checks for valid DataType
	if (type < 0 || type >= NUM_TYPES)
		return -4;

	for(dhara_sector_t i=0;i<=largestSector;i++){

	}

	return 1;
}

int Storage_Read_Active(const DataType type, const uint8_t sectorSequence, uint8_t* data, uint32_t* dataSize){

	// Checks for valid DataType
	if (type < 0 || type >= NUM_TYPES)
		return -4;

	// Checks if sectorSequence exists
	if(sectorSequence <= 0 || sectorSequence >= TYPE_SEQUENCE[type])
		return -5;


	for(dhara_sector_t i=0;i<=largestSector;i++){

	}

	return 1;
}

int Storage_Read_Backup(const DataType type, const uint8_t backupGroup, uint8_t* data, uint32_t* dataSize){

	// Checks for valid DataType
	if (type < 0 || type >= NUM_TYPES)
		return -4;

	return 1;
}

//###################################################
//##############    HELPER FUNCTIONS    #############
//###################################################

/*
 * FUNCTION: Find_Empty_Sector
 *
 * DESCRIPTION: Finds first empty sector and returns it
 *
 *
 * RETURNS:
 * 		empty sector number on success or -1 if an error occurs.
*/
dhara_sector_t Find_Empty_Sector(){

	for(dhara_sector_t i=0;i<Dhara_Capacity();i++){

		dhara_error_t err;
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
