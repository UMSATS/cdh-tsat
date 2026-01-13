 /*
 * FILENAME: StorageManager.h
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Jan 13, 2026
 */

#ifndef INC_STORAGEMANAGER_H_
#define INC_STORAGEMANAGER_H_

//
// INCLUDES
#include <Dhara_Wrapper.h>

#include "../Inc/telemetry.h"


//
// DEFINES
#define MAX_NUM_OF_ACTIVE_SECTORS 16 //16 * 2048 bytes = 32768 bytes

//
// Data Type used to determine the configuration for how the data is stored
enum DataType{
	raw,
	telem,
	log,
	firmware
};


//#####################################################
//##############    DATA TYPE CONFIGS    ##############
//#####################################################

// RAW

//TODO see if raw should just input at a given sector. maybe write function can take in a sector number for desired sector to write to,
//ie. if 1 is given then data will be written/appended to given sector, write will replace and append will read and add to back, will need 
//protection from when sector is too full when data append request is submitted.
// I like this idea, uint32_t storageWrite(DataType type, int sector "-1 to write to empty sector/create new at back") returns the sector
// that it wrote to so the user can store the value.

//TODO create a status enum that describes if the write was successful or not and other errors that might occur

// TELEM 
#define TEL_NUM_OF_BACKUPS 2

uint8_t telActive[MAX_NUM_OF_ACTIVE_SECTORS];
uint8_t telActiveCount=0; 

uint8_t telActive[TEL_NUM_OF_BACKUPS*MAX_NUM_OF_ACTIVE_SECTORS];
uint8_t telBackupCount=0;

// LOG
#define LOG_NUM_OF_BACKUP 1

uint8_t logActive[MAX_NUM_OF_ACTIVE_SECTORS];
uint8_t logActiveCount=0; 

uint8_t logActive[LOG_NUM_OF_BACKUP*MAX_NUM_OF_ACTIVE_SECTORS];
uint8_t logBackupCount=0;

// FIRMWARE
#define FIRM_NUM_OF_BACKUP 0

uint8_t firmActive[MAX_NUM_OF_ACTIVE_SECTORS];
uint8_t firmActiveCount=0; 

uint8_t firmActive[FIRM_NUM_OF_BACKUP*MAX_NUM_OF_ACTIVE_SECTORS];
uint8_t firmBackupCount=0;


//#############################################
//##############    FUNCTIONS    ##############
//#############################################

uint32_t storageWrite(DataType type, uint8_t sector);
uint32_t storageAppend(DataType type, uint8_t sector);

uint32_t storageSendToBackup(DataType type);


// TODO Maybe private functions
uint32_t findEmptySector();

#endif /* INC_STORAGEMANAGER_H_ */
