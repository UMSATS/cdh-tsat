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
const dhara_sector_t INVALID_SECTOR=((dhara_sector_t)-1);

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
 * FUNCTION: Storage_Write
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
 *      s is the desired sector
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *
 * RETURNS:
 * 		The sector written to if successful or -1 if an error occurs.
*/
int Storage_Write(const DataType type, const dhara_sector_t s, const uint8_t *data, const uint16_t dataSize);

/*
 * FUNCTION: Storage_Append
 *
 * DESCRIPTION: Appends data to a logical sector.
 *
 * NOTE:
 *     - If s is set as -1, a new sector will be used. TODO fix
 *
 * VARIABLES:
 * 		type is the desired data type
 *      s is the desired sector
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *
 * RETURNS:
 * 		The sector written to if successful or -1 if an error occurs. TODO add case where data is too large for sector
*/
int Storage_Append(const DataType type, const dhara_sector_t s, const uint8_t *data, const uint16_t dataSize);

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
 * 		0 on success or -1 if an error occurs.
*/
int Storage_Send_To_Backup(const DataType type);

/*
 * FUNCTION: Storage_Read_Active
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
int Storage_Read_Active(const DataType type, uint8_t* data, uint32_t* dataSize);

/*
 * FUNCTION: Storage_Read_Backup
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
int Storage_Read_Backup(const DataType type, uint8_t* data, uint32_t* dataSize);


#endif /* INC_STORAGEMANAGER_H_ */
