/*
 * FILENAME: Dhara_Wrapper.h
 *
 * DESCRIPTION: Contains the wrapper functions that will be used to interact with the Dhara library
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Oct. 4 2025
 */

#pragma once

//For Nand struct
#define NAND_LOG2_PAGE_SIZE 11
#define NAND_LOG2_PAGE_PER_BLOCK 6
#define NAND_NUM_OF_BLOCKS 1024
#define NAND_NUM_OF_SPARES 20
#define NAND_NUM_AVAILABLE_BLOCKS NAND_NUM_OF_BLOCKS-NAND_NUM_OF_SPARES


//######################################################
//#####################    INFO    #####################
//######################################################


/*
 * :::    How Dhara Functions    :::
 *
 * - Dhara uses "Sectors" as the interaction point with NAND pages.
 *
 * - A sector has a NAND page addressed to it and that page updates every time a dhara_map_write function is called. Instead of rewriting the
 *   page (which would require the NAND block to be erased) Dhara instead just writes to a new empty page and marks the previous as "obsolete" (trimmed).
 *
 * - It keeps track of these changes in a journal which is stored on the NAND after a dhara_map_sync is called.
 *
 * - With these trimmed pages, gaps occur in the NAND block between assigned pages, this is where Dharas garbage
 *   collection system comes in. Dharas collects all the used pages in a block and transfers
 *   them to a new block(and updates sectors mapping) allowing the previous block to be deleted and reused later.
 *
 * - The Dhara Wrapper functions create a safe and easy way to interact with the dhara library insuring that the dhara functions are used correctly.
*/





//###############################################################################################
//Include Directives
//###############################################################################################
#include <dhara/error.h>
#include <stdint.h>

/*
 * FUNCTION: Dhara_Init
 *
 * DESCRIPTION: Initializes the Dhara Library.
 *
 * NOTES:
 *  - If the journal is not found, then all data is deleted and Dhara is reinitialized.
 *	- error will still occur since the journal isn't synced/written to the nand yet.
 *	- The journal will be put onto nand when a write call occurs.
 *
 * RETURNS:
 * 		A Dhara_error_t,
 * 			if equal to DHARA_E_NONE, then initialization was successful,
 * 			if equal to DHARA_E_TOO_BAD, then journal was not found and map was reinitialized.
*/
dhara_error_t Dhara_Init(void);


//###############################################################################
//###########################    WRAPPER FUNCTIONS    ###########################
//###############################################################################



/*
 * FUNCTION: Dhara_Clear
 *
 * DESCRIPTION: Clear the map (delete all sectors).
 *
*/
void Dhara_Clear();

/*
 * FUNCTION: Dhara_Capacity
 *
 * DESCRIPTION: Obtain the maximum capacity of the map.
 *
 * RETURNS:
 * 		A uint32_t that represents the capacity(total sectors).
*/
uint32_t Dhara_Capacity();

/*
 * FUNCTION: Dhara_Size
 *
 * DESCRIPTION: Obtain the current number of allocated sectors.
 *
 * RETURNS:
 * 		A uint32_t that represents the amount of used sectors.
*/
uint32_t Dhara_Size();

/*
 * FUNCTION: Dhara_Find
 *
 * DESCRIPTION: Find the physical page which holds the current data for given sector.
 *
 * NOTE:
 *     - If the sector doesn't exist, the error is E_NOT_FOUND.
 *
 * VARIABLES:
 *      s is the desired sector
 *      loc is the variable that will store the page address
 *      err is the variable that will the store error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Find(uint32_t s, uint32_t *loc, dhara_error_t *err);

/*
 * FUNCTION: Dhara_Read
 *
 * DESCRIPTION: Read from the given logical sector.
 *
 * NOTE:
 *     - If the sector is unmapped, a blank page (0xff) will be returned.
 *     - If dataSize is greater then PAGESIZE, read will fail.
 *
 * VARIABLES:
 *      s is the desired sector
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Read(uint32_t s, uint8_t *data, const uint16_t dataSize, dhara_error_t *err);

/*
 * FUNCTION: Dhara_Write
 *
 * DESCRIPTION: Write data to a logical sector.
 *
 * NOTE:
 *     - If dataSize is greater then PAGESIZE, write will fail.
 *
 * VARIABLES:
 *      s is the desired sector
 *      data is the data buffer
 *      dataSize is the size of the data buffer
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Write(uint32_t s, const uint8_t *data, const uint16_t dataSize, dhara_error_t *err);

/*
 * FUNCTION: Dhara_Copy_Page
 *
 * DESCRIPTION: Copy any flash page to a logical sector.
 *
 * NOTE:
 *     - Recommend AVOID using and use Dhara_Copy_Page.
 *
 * VARIABLES:
 *      src is the source page
 *      dst is the destination sector
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Copy_Page(uint32_t src, uint32_t dst, dhara_error_t *err);

/*
 * FUNCTION: Dhara_Copy_Sector
 *
 * DESCRIPTION: Copy one sector to another.
 *
 * NOTE:
 *     - If the source sector is unmapped, the destination sector will be trimmed.
 *
 * VARIABLES:
 *      src is the source sector
 *      dst is the destination sector
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Copy_Sector(uint32_t src, uint32_t dst, dhara_error_t *err);

/*
 * FUNCTION: Dhara_Erase
 *
 * DESCRIPTION: Delete a logical sector.
 *
 * NOTE:
 *     - You don't necessarily need to do this, but it's a useful hint
 *       if you no longer require the sector's data to be kept.
 *
 *     - If order is non-zero, it specifies that all sectors in the
 *       (2**order)-aligned group of s are to be deleted.
 *
 * VARIABLES:
 *      s is the desired sector
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Erase(uint32_t s, dhara_error_t *err);

/*
 * FUNCTION: Dhara_Force_Sync
 *
 * DESCRIPTION: Synchronize the map.
 *
 * NOTE:
 *     - Once this returns successfully, all changes to date are persistent
 *       and durable. Conversely, there is no guarantee that unsynchronized
 *       changes will be persistent.
 *
 * VARIABLES:
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_Force_Sync(dhara_error_t *err);

/* Perform one garbage collection step. You can do this whenever you
 * like, but it's not necessary -- garbage collection happens
 * automatically and is interleaved with other operations.
 */
/*
 * FUNCTION: Dhara_GC
 *
 * DESCRIPTION: Perform one garbage collection step.
 *
 * NOTE:
 *     - You can do this whenever you like, but it's not necessary -- garbage
 *       collection happens automatically and is interleaved with other operations.
 *
 * VARIABLES:
 *      err is the variable that will store the error
 *
 * RETURNS:
 * 		0 on success or -1 if an error occurs.
*/
int Dhara_GC(dhara_error_t *err);

