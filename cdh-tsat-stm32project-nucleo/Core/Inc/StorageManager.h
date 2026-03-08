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

//###############################################################################################
//Include Directives
//###############################################################################################
#include <dhara/error.h>
#include <stdint.h>
#include <dhara/map.h>


//
// DEFINES
#define INVALID_SECTOR ((dhara_sector_t)-1)

//
// Data Type used to determine the configuration for how the data is stored
typedef enum {
	RAW = 0,
	TELEM = 1,
	LOG = 2,
	FIRMWARE = 3
}DataType;



//#############################################
//##############    FUNCTIONS    ##############
//#############################################

/*
 * FUNCTION: Storage_Init
 *
 * DESCRIPTION: Initializes the storage sectors found on NAND.
 *
 * RETURNS:
 * 		The sector written to if successful or -1 if an error occurs.
*/
int Storage_Init();

/*
 * FUNCTION: Storage_Write
 *
 * DESCRIPTION: Write data to a logical sector.
 *
 * NOTE:
 *     - If s is set as -1, a new sector will be used.
 *
 * VARIABLES:
 * 		type is the desired data type
 *      sequence is the desired sector in the sector sequence you want to write to
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *
 * RETURNS:
 * 		The sector written to if successful or -1 if an error occurs.
*/
int Storage_Write(const DataType type, const uint16_t sequence, const uint8_t *data, const uint16_t dataSize);

/*
 * FUNCTION: Storage_Append
 *
 * DESCRIPTION: Appends data to a logical sector.
 *
 * VARIABLES:
 * 		type is the desired data type
 * 		sequence is the sequence number within the sector batch
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *
 * RETURNS:
 * 		0 on success or a negative number if an error occurs.
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 Data Type Fail
 * 		-2 Sequence Does Not Exits
 * 		-3 dataSize is Too Large
 * 		-4 Failed Write
*/
int Storage_Append(const DataType type, const uint8_t *data, const uint16_t dataSize);

/*
 * FUNCTION: Storage_Send_To_Backup
 *
 * DESCRIPTION: Sends the active sectors to the backup sectors
 *
 * Note:
 * 	   - This also handles
 *
 * VARIABLES:
 *      type is the desired data type
 *
 * RETURNS:
 * 		0 on success or a negative number if an error occurs.
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 Data Type Fail
 * 		-2 dataSize is Too Large
 * 		-3 Read Fail
 * 		-4 Fail to Find or New Free Sector
 * 		-5 Failed Write
*/
int Storage_Send_To_Backup(const DataType type);

/*
 * FUNCTION: Storage_Fetch_Sectors
 *
 * DESCRIPTION:
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Storage_Fetch_Sectors(const DataType dType, const uint8_t sType, uint8_t* sectorArray, uint32_t* sectorArraySize);

/*
 * FUNCTION: Storage_Read
 *
 * DESCRIPTION:
 *
 *
 * VARIABLES:
 *      type is the desired data type
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Storage_Read(const DataType dType, const uint8_t sType, const uint16_t sequence, uint8_t* data, uint32_t* dataSize);

/*
 * FUNCTION: Storage_Read_Sector
 *
 * DESCRIPTION:
 *
 *
 * VARIABLES:
 *
 * RETURNS:
 * 		0 on success or a negative number if an error occurs.
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 Data Type Fail
 * 		-2 Sector Type Fail
 * 		-3 Sequence Does Not Exits
 * 		-4 Failed To Find Sector
*/
int Storage_Read_Sector(const uint32_t sector, uint8_t* data, uint32_t* dataSize);

/*
 * FUNCTION: Storage_Trim
 *
 * DESCRIPTION: Trims down current storage area by replacing empty sector near the front with filled sectors.
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Storage_Trim();


#endif /* INC_STORAGEMANAGER_H_ */
