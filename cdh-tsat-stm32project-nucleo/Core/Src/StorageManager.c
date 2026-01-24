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
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rtc.h"

#include "../Inc/telemetry.h"



//
// DEFINES
const uint8_t STORAGE_MAGIC=42;

//###############################################
//##############    SECTOR TYPE    ##############
//###############################################


typedef enum {
	ACTIVE=0,
	BACKUP=1
}SectorType;

typedef struct {
	uint8_t magic;

	uint8_t type;
	uint8_t state;
	uint8_t stateNumber;

	uint16_t offset;

	dhara_sector_t nextSector;
	dhara_sector_t prevSector;

	dhara_sector_t curSector;
}SectorHeader;

//#####################################################
//##############    DATA TYPE CONFIGS    ##############
//#####################################################

// RAW

SectorHeader rawActive={ .magic=0};

// TELEM
#define TEL_NUM_OF_BACKUPS 2

SectorHeader telActive={ .magic=0};

SectorHeader telBackup[TEL_NUM_OF_BACKUPS]={0};
uint16_t telBackupCount=0;

// LOG
#define LOG_NUM_OF_BACKUP 1

SectorHeader logActive={ .magic=0};

SectorHeader logBackup[TEL_NUM_OF_BACKUPS]={0};
uint16_t logBackupCount=0;

// FIRMWARE
#define FIRM_NUM_OF_BACKUP 0

SectorHeader firmActive={ .magic=0};

SectorHeader firmBackup[TEL_NUM_OF_BACKUPS]={0};
uint16_t firmBackupCount=0;


//#############################################
//##############    FUNCTIONS    ##############
//#############################################

int Storage_Init(){

	for(dhara_sector_t i=0;i<Dhara_Capacity();i++){

		dhara_error_t err;
		SectorHeader hdr;

		Dhara_Read(i, (uint8_t*)&hdr, sizeof(SectorHeader)-sizeof(dhara_sector_t), &err);
		if(err){ continue;}

		if(hdr.magic!=STORAGE_MAGIC){
			continue;
		}


		switch (hdr.type){
		case RAW:

			if(hdr.state==ACTIVE){

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


					RTC_HandleTypeDef hrtc;

					RTC_TimeTypeDef sTime;
					RTC_DateTypeDef sDate;

					HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


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
