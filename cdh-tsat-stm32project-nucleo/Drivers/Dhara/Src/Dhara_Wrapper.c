/*
 * FILENAME: dhara.c
 *
 * DESCRIPTION: Contains the wrapper functions that will be used to interact with the Dhara library
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Oct. 4 2025
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include <string.h>

#include <dhara/map.h>
#include <dhara/nand.h>
#include <Dhara_Wrapper.h>

#include "W25N_driver.h"



//################################################################################
//##########################    DHARA INITIALIZATION    ##########################
//################################################################################

//Map defines
#define GC_RATIO 4

struct dhara_map my_map;

uint8_t journal_buffer[PAGESIZE];

uint8_t dharaUsedSpareCount=0;

//Defines
static uint8_t dhara_read_buf[PAGESIZE];
static uint8_t dhara_write_buf[PAGESIZE];



dhara_error_t Dhara_Init(void){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	W25N_BBM_LUT_Size(&dharaUsedSpareCount);


	dhara_map_init(&my_map, &my_nand, journal_buffer, GC_RATIO);
	uint8_t result=dhara_map_resume(&my_map, &err);

	//Journal Could Not Be Found
	if(err!=DHARA_E_NONE||result==-1){
		//Deletes all data and restarts dhara
		for(int i=0;i<NAND_NUM_OF_BLOCKS;i++){
			status=W25N_Erase(i*64);

			if (status != W25N_ERASE_OK) {
				err=DHARA_E_TOO_BAD;
				goto error;
			}
		}


		dhara_map_init(&my_map, &my_nand, journal_buffer, GC_RATIO);
		dhara_map_resume(&my_map, &err);
		//NOTE: error will still occur since the journal isn't synced/written to the nand yet.
		//The journal will be put onto nand when a write call occurs.

		err=DHARA_E_NOT_FOUND;
	}

error:
	return err;
}



//###############################################################################
//###########################    WRAPPER FUNCTIONS    ###########################
//###############################################################################

void Dhara_Clear()
{
	dhara_map_clear(&my_map);
}

uint32_t Dhara_Capacity()
{
	return dhara_map_capacity(&my_map);
}

/* Obtain the current number of allocated sectors. */
uint32_t Dhara_Size()
{
	return dhara_map_size(&my_map);
}

int Dhara_Find(uint32_t s, uint32_t *loc, dhara_error_t *err)
{
	return dhara_map_find(&my_map, s, loc, err);
}

int Dhara_Read(uint32_t s, uint8_t *data, const uint16_t dataSize, dhara_error_t *err)
{
	if(dataSize==PAGESIZE){
		return dhara_map_read(&my_map, s, data, err);
	}
	if(dataSize<PAGESIZE){
		int status = dhara_map_read(&my_map, s, dhara_read_buf, err);

		// Read FAILED!!!
		if(status==-1){return status;}

		memcpy(data,dhara_read_buf,dataSize);

		return status;
	}

	// If dataSize is greater then the page size
	return -1;
}

int Dhara_Write(uint32_t s, const uint8_t *data, uint16_t dataSize, dhara_error_t *err)
{
	if(dataSize==PAGESIZE){
		return dhara_map_write(&my_map, s, data, err);
	}
	if(dataSize<PAGESIZE){
		uint8_t *newData = dhara_write_buf;

		// Copies data to new array of correct size and fills the rest with empty datapoints
		memcpy(newData, data, dataSize);
		for(unsigned int i=dataSize;i<PAGESIZE;i++){
			newData[i]=0xFF;
		}

		return dhara_map_write(&my_map, s, newData, err);
	}else{//data is larger then a single page
		return -1;
	}
}

int Dhara_Copy_Page(uint32_t src, uint32_t dst, dhara_error_t *err)
{
	return dhara_map_copy_page(&my_map, src, dst, err);
}

int Dhara_Copy_Sector(uint32_t src, uint32_t dst, dhara_error_t *err)
{
	return dhara_map_copy_sector(&my_map, src, dst, err);
}

int Dhara_Erase(uint32_t s, dhara_error_t *err)
{
	return dhara_map_trim(&my_map, s, err);
}

int Dhara_Force_Sync(dhara_error_t *err)
{
	return dhara_map_sync(&my_map, err);
}

int Dhara_GC(dhara_error_t *err)
{
	return dhara_map_gc(&my_map,err);
}

