/*
 * StorageManager.c
 *
 *  Created on: Jan 13, 2026
 *      Author: adriv
 */

//
// INCLUDES
#include "../Inc/StorageManager.h"

#include <Dhara_Wrapper.h>

#include "../Inc/telemetry.h"



//
// DEFINES
#define DEFAULT_NUM_OF_SECTORS 4 //4 * 2048 bytes = 8192 bytes

const uint8_t STORAGE_MAGIC=42;

//###############################################
//##############    SECTOR TYPE    ##############
//###############################################


enum SectorType{
	ACTIVE=0,
	BACKUP=1
};

struct SectorHeader{
	uint8_t magic;

	uint8_t magic;
	uint8_t type;
	uint8_t state;

	uint8_t sequence;//TODO figure out
};


//#####################################################
//##############    DATA TYPE STRUCTS    ##############
//#####################################################

struct StorageStruct{

	// Sector
	dhara_sector_t s;

	// Current Offset
	uint32_t offset;

	//
	uint8_t hasRoom=1;
};

//#####################################################
//##############    DATA TYPE CONFIGS    ##############
//#####################################################

// RAW

StorageStruct rawActive[DEFAULT_NUM_OF_SECTORS];
uint16_t rawActiveCount=0;

// TELEM
#define TEL_NUM_OF_BACKUPS 2

StorageStruct telActive[DEFAULT_NUM_OF_SECTORS];
uint16_t telActiveCount=0;

StorageStruct telBackup[TEL_NUM_OF_BACKUPS][DEFAULT_NUM_OF_SECTORS];
uint16_t telBackupCount=0;

// LOG
#define LOG_NUM_OF_BACKUP 1

StorageStruct logActive[DEFAULT_NUM_OF_SECTORS];
uint16_t logActiveCount=0;

StorageStruct logBackup[TEL_NUM_OF_BACKUPS][DEFAULT_NUM_OF_SECTORS];
uint16_t logBackupCount=0;

// FIRMWARE
#define FIRM_NUM_OF_BACKUP 0

StorageStruct firmActive[DEFAULT_NUM_OF_SECTORS];
uint16_t firmActiveCount=0;

StorageStruct firmBackup[TEL_NUM_OF_BACKUPS][DEFAULT_NUM_OF_SECTORS];
uint16_t firmBackupCount=0;


//#############################################
//##############    FUNCTIONS    ##############
//#############################################

int Storage_Init(){

	for(uint32_t i=0;i<Dhara_Capacity();i++){

		dhara_error_t err;
		SectorHeader hdr;

		Dhara_Read(i, &hdr, sizeof(hdr), &err);
		if(err){ continue;}

		if(hdr.magic!=STORAGE_MAGIC){
			continue;
		}

		// RAW
		if(data[0]==raw){

		}

		// TELEMETRY
		if(data[0]==TELEM){

			Current_Sector++;
		}

		// DATA LOGS
		if(data[0]==LOG){

			Current_Sector++;
		}

		// FIRMWARE
		if(data[0]==FIRMWARE){

			Current_Sector++;
		}
	}
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
