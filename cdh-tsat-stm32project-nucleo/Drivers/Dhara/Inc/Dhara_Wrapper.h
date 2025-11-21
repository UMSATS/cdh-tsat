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

//###############################################################################################
//Include Directives
//###############################################################################################
#include <dhara/error.h>
#include <stdint.h>

/*
 * FUNCTION: Dhara_Init
 *
 * DESCRIPTION: Reads the internal Bad Block Management (BBM) Look Up Table (LUT) and stores the
 *              result in a given buffer.
 *
 * NOTES:
 *  - Does a thing :D
 *
 * RETURNS:
 * 		A Dhara_error_t, if not equal to DHARA_E_NON then initialization failed.
 */
dhara_error_t Dhara_Init(void);


//###############################################################################
//###########################    WRAPPER FUNCTIONS    ###########################
//###############################################################################

/* Clear the map (delete all sectors). */
void Dhara_Clear();

/* Obtain the maximum capacity of the map. */
uint32_t Dhara_Capacity();

/* Obtain the current number of allocated sectors. */
uint32_t Dhara_Size();

/* Find the physical page which holds the current data for this sector.
 * Returns 0 on success or -1 if an error occurs. If the sector doesn't
 * exist, the error is E_NOT_FOUND.
 */
int Dhara_Find(uint32_t s, uint32_t *loc, dhara_error_t *err);

/* Read from the given logical sector. If the sector is unmapped, a
 * blank page (0xff) will be returned.
 */
int Dhara_Read(uint32_t s, uint8_t *data, const uint16_t dataSize, dhara_error_t *err);

/* Write data to a logical sector. */
int Dhara_Write(uint32_t s, const uint8_t *data, const uint16_t dataSize, dhara_error_t *err);

/* Copy any flash page to a logical sector.
 * Recommend AVOID using this unless you know what you are doing
*/
int Dhara_Copy_Page(uint32_t src, uint32_t dst, dhara_error_t *err);

/* Copy one sector to another. If the source sector is unmapped, the
 * destination sector will be trimmed.
 */
int Dhara_Copy_Sector(uint32_t src, uint32_t dst, dhara_error_t *err);

/* Delete a logical sector. You don't necessarily need to do this, but
 * it's a useful hint if you no longer require the sector's data to be
 * kept.
 *
 * If order is non-zero, it specifies that all sectors in the
 * (2**order)-aligned group of s are to be deleted.
 */
int Dhara_Erase(uint32_t s, dhara_error_t *err);

/* Synchronize the map. Once this returns successfully, all changes to
 * date are persistent and durable. Conversely, there is no guarantee
 * that unsynchronized changes will be persistent.
 */
int Dhara_Force_Sync(dhara_error_t *err);

/* Perform one garbage collection step. You can do this whenever you
 * like, but it's not necessary -- garbage collection happens
 * automatically and is interleaved with other operations.
 */
int Dhara_GC(dhara_error_t *err);

