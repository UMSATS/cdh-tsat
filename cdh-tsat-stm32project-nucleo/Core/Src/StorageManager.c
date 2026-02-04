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

// RAW

uint16_t curRawSeqence=0;
dhara_sector_t rawActive=INVALID_SECTOR;

// TELEM
#define TEL_NUM_OF_BACKUPS 2

uint16_t curTelSeqence=0;
dhara_sector_t telActive=INVALID_SECTOR;

// KEEPS TRACK OF THE NUMBER OF BACKUPS STORED
uint16_t telBackupCount=0;

// LOG
#define LOG_NUM_OF_BACKUP 1

uint16_t curLogSeqence=0;
dhara_sector_t logActive=INVALID_SECTOR;

// KEEPS TRACK OF THE NUMBER OF BACKUPS STORED
uint16_t logBackupCount=0;

// FIRMWARE
#define FIRM_NUM_OF_BACKUP 0

uint16_t curFirmSeqence=0;
dhara_sector_t firmActive=INVALID_SECTOR;

// KEEPS TRACK OF THE NUMBER OF BACKUPS STORED
uint16_t firmBackupCount=0;// TODO EITHER READ AND WRITE BACKUP NUMBER OR


//#############################################
//##############    FUNCTIONS    ##############
//#############################################

int Storage_Init(){

	for(dhara_sector_t i=0;i<Dhara_Capacity();i++){

		dhara_error_t err;
		RawSectorHeader hdr;

		Dhara_Read(i, (uint8_t*)&hdr, sizeof(SectorHeader)-sizeof(dhara_sector_t), &err);
		if(err){ continue;}

		if(hdr.magic!=STORAGE_MAGIC){
			continue;
		}// TODO maybe delete the sector?


		switch (hdr.type){
		case RAW:

			if(hdr.sectorType==ACTIVE){

				// IF UNINITIALIZED AND SECTOR HAS NOT PREVIOUS SECTOR AKA FIRST IN LIST
				if(rawActive.magic!=STORAGE_MAGIC&&hdr.prevSector==INVALID_SECTOR){

					rawActive=hdr;
					rawActive.curSector=i;

				}else{

					Dhara_Erase(i, &err);
					if(err){ continue;}

				}
			}else{

				Dhara_Erase(i, &err);
				if(err){ continue;}

			}

			break;
		case TELEM:

			if(hdr.state==ACTIVE){

				// IF UNINITIALIZED AND SECTOR HAS NOT PREVIOUS SECTOR AKA FIRST IN LIST
				if(telActive.magic!=STORAGE_MAGIC&&hdr.prevSector==INVALID_SECTOR){

					telActive=hdr;
					telActive.curSector=i;

				}else{

					Dhara_Erase(i, &err);
					if(err){ continue;}

				}
			}else{

				//TODO SETUP BACKUP

			}

			break;
		case LOG:

			if(hdr.state==ACTIVE){

				// IF UNINITIALIZED AND SECTOR HAS NOT PREVIOUS SECTOR AKA FIRST IN LIST
				if(logActive.magic!=STORAGE_MAGIC&&hdr.prevSector==INVALID_SECTOR){

					logActive=hdr;
					logActive.curSector=i;


				}else{

					Dhara_Erase(i, &err);
					if(err){ continue;}

				}
			}else{

				//TODO SETUP BACKUP

			}

			break;
		case FIRMWARE:

			break;
		default:
			Dhara_Erase(i, &err);
			if(err){ continue;}

			break;
		}
	}


	return -1;
}

int Storage_Write(const DataType type, const dhara_sector_t s, const uint8_t *data, const uint16_t dataSize){

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

int Storage_Append(const DataType type, const dhara_sector_t s, const uint8_t *data, const uint16_t dataSize){

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
}

int Storage_Read_Active(const DataType type, uint8_t* data, uint32_t* dataSize){

}

int Storage_Read_Backup(const DataType type, uint8_t* data, uint32_t* dataSize){

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
		SectorHeader hdr;

		Dhara_Read(i, &hdr, sizeof(hdr), &err);
		if(err){ continue;}

		if(hdr.magic!=STORAGE_MAGIC){
			Dhara_Erase(i, &err);
			if(err){ continue;}

			return i;
		}
	}

	return -1;
}
