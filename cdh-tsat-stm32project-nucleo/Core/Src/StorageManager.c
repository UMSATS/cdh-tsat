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

#include <Dhara_Wrapper.h>

#include "../../Core/Inc/utils.h"
#include "../Inc/telemetry.h"



//
// DEFINES
const uint8_t STORAGE_MAGIC=42;

//###############################################
//##############    SECTOR TYPE    ##############
//###############################################


typedef enum {
	ACTIVE=0b00000000,
	BACKUP=0b00001111
}SectorType;

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


// RAW

uint32_t curRawSeqence=0;
dhara_sector_t rawActive=INVALID_SECTOR;

// TELEM
#define TEL_NUM_OF_BACKUPS 2

uint32_t curTelSeqence=0;
dhara_sector_t telActive=INVALID_SECTOR;

// KEEPS TRACK OF THE NUMBER OF BACKUPS STORED
uint16_t telBackupCount=0;

// LOG
#define LOG_NUM_OF_BACKUPS 1

uint32_t curLogSeqence=0;
dhara_sector_t logActive=INVALID_SECTOR;

// KEEPS TRACK OF THE NUMBER OF BACKUPS STORED
uint16_t logBackupCount=0;

// FIRMWARE
#define FIRM_NUM_OF_BACKUPS 0

uint32_t curFirmSeqence=0;
dhara_sector_t firmActive=INVALID_SECTOR;

// KEEPS TRACK OF THE NUMBER OF BACKUPS STORED
uint16_t firmBackupCount=0;// TODO EITHER READ AND WRITE BACKUP NUMBER OR


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
			continue;
		}// TODO maybe delete the sector?



		//
		// TYPE SWITCH
		switch (hdr.dataType){

		//
		// RAW DATA TYPE
		case RAW:

			//
			// ACTIVE SECTOR
			if(hdr.sectorType==ACTIVE){

				if(curRawSeqence<hdr.sequence){
					curRawSeqence=hdr.sequence;
					rawActive=i;
				}


			//
			// BACKUP SECTOR
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

				if(curTelSeqence<hdr.sequence){
					curTelSeqence=hdr.sequence;
					telActive=i;
				}

			//
			// BACKUP SECTOR
			}else if(hdr.sectorType==BACKUP&&TEL_NUM_OF_BACKUPS>0){

				// TODO manage backups

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

				if(curLogSeqence<hdr.sequence){
					curLogSeqence=hdr.sequence;
					logActive=i;
				}

			//
			// BACKUP SECTOR
			}else if(hdr.sectorType==BACKUP&&LOG_NUM_OF_BACKUPS>0){

				// TODO manage backups

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

				if(curFirmSeqence<hdr.sequence){
					curFirmSeqence=hdr.sequence;
					firmActive=i;
				}

			//
			// BACKUP SECTOR
			}else if(hdr.sectorType==BACKUP&&FIRM_NUM_OF_BACKUPS>0){

				// TODO manage backups

			}else{

				Dhara_Erase(i, &err);
				if(err){ continue;}

			}

			break;

		//
		// DEFAULT CASE
		default:

			Dhara_Erase(i, &err);
			if(err){ continue;}

			break;
		}
	}


	return -1;
}

int Storage_Write(const DataType type, const uint16_t sequence, const uint8_t *data, const uint16_t dataSize){

	// RAW
	if(type==RAW){

	}

	// TELEMETRY
	if(type==TELEM){

	}

	// DATA LOGS
	if(type==LOG){

	}

	// FIRMWARE
	if(type==FIRMWARE){

	}

	return -1;
}

int Storage_Append(const DataType type, const uint8_t *data, const uint16_t dataSize){

	// RAW
	if(type==RAW){

	}

	// TELEMETRY
	if(type==TELEM){

	}

	// DATA LOGS
	if(type==LOG){

	}

	// FIRMWARE
	if(type==FIRMWARE){

	}

	return -1;
}

int Storage_Send_To_Backup(const DataType type){
	// TODO SET BACKUP ARRAY SIZE TO MATCH CURRENTS ACTIVE COUNT
	return 1;
}

int Storage_Read_Active(const DataType type, uint8_t* data, uint32_t* dataSize){
	return 1;
}

int Storage_Read_Backup(const DataType type, uint8_t* data, uint32_t* dataSize){
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
int Find_Empty_Sector(){

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

	return -1;
}
