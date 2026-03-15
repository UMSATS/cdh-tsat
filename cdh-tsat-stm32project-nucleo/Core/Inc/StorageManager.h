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


//###############################################################################################
//                                          SECTOR NODE
//###############################################################################################

typedef struct SectorNode{

	// SECTOR AND SEQUENCE NUMBER
	dhara_sector_t s;
	int32_t sequence;

	struct SectorNode* nextSector;
}SectorNode;


//###############################################################################################
//                                        STORAGE MANAGER
//###############################################################################################

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



//###############################################################################################
//                                           FUNCTIONS
//###############################################################################################

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
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 Invalid DataType
 * 		-2 Sequence Does Not Exits
 * 		-3 Sequence could not be found in list
 * 		-4 dataSize is Too Large
 * 		-5 Failed Write
*/
int Storage_Write(const DataType dType, const uint16_t sequence, const uint8_t *data, const uint16_t dataSize);

/*
 * FUNCTION: Storage_Append
 *
 * DESCRIPTION: Appends data to a logical sector.
 *
 * VARIABLES:
 * 		type is the desired data type
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *
 * RETURNS:
 * 		0 on success or a negative number if an error occurs.
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 Invalid DataType
 * 		-2 dataSize is Too Large
 * 		-3 Read Fail
 * 		-4 Fail to Find or New Free Sector
 * 		-5 Failed Write
*/
int Storage_Append(const DataType dType, const uint8_t *data, const uint16_t dataSize);

/*
 * FUNCTION: Storage_Send_To_Backup
 *
 * DESCRIPTION: Sends the active sectors to the backup sectors
 *
 * Note:
 * 	   - This also calls the method to delete unnecessary Lists
 *
 * VARIABLES:
 *      type is the desired data type
 *
 * RETURNS:
 * 		0 on success or a negative number if an error occurs.
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 Invalid DataType
 * 		-2 Read Fail
 * 		-3 Write Fail
*/
int Storage_Send_To_Backup(const DataType dType);

/* FUNCTION: Storage_Get_SectorNode
*
* DESCRIPTION: Fetches a specific SectorNode from the given DataType, SectorType and sequence number
*
*
* VARIABLES:
*		dType
*		sType
*		sequence
*		sectorNode
*
* RETURNS:
* 		0 on success or a negative number if an error occurs.
*
* ERROR CODES
* 		0 no error
* 		-1 Invalid DataType
* 		-2 address pointer is null
* 		-3 sequence number is out of range
* 		-4 invalid SectorType
* 		-5 sequence number is not found in list
*/
int Storage_Get_SectorNode(const DataType dType, const uint8_t sType, const uint16_t sequence, SectorNode** sectorNode);

/*
 * FUNCTION: Storage_Read
 *
 * DESCRIPTION: Reads sector of given sector node
 *
 *
 * VARIABLES:
 *		sectorNode is the contains the desired sector to read
 *      data it the pointer that stores the read data
 *      dataSize is the size of the read data (Max Size is PAGESIZE)
 *
 * RETURNS:
 * 		0 on success or a negative number if an error occurs.
 *
 * ERROR CODES
 * 		0 no error
 * 		-1 SectorNode is NULL
 * 		-2 dataSize is larger then PAGESIZE
 * 		-3 Dhara_Read failed
*/
int Storage_Read(const SectorNode* sectorNode, uint8_t* data, uint32_t dataSize);

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
