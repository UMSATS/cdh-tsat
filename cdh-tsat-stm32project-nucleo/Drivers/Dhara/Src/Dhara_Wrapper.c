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

uint8_t journal_buffer[PAGE_SIZE];



dhara_error_t Dhara_Init(void){
	for(int i=0;i<NAND_NUM_OF_BLOCKS;i++){
		W25N_Erase(i*64);
	}

	dhara_error_t err=DHARA_E_NONE;

	uint8_t data[2048];

	memset(data, 0xFF, 2048);      // pad with erased value

	// Copy Dhara-provided data into beginning of full buffer
	data[0]=0;data[1]=1;data[2]=2;data[3]=3;data[4]=4;data[5]=5;data[6]=6;


	dhara_map_init(&my_map, &my_nand, journal_buffer, GC_RATIO);
	dhara_map_resume(&my_map, &err);

	//Journal Could Not Be Found
	if(err!=DHARA_E_NONE||1){
		//Deletes all data and restarts dhara
		for(int i=0;i<my_nand.num_blocks+20;i++){
			W25N_Erase(i*64);
		}
		err=DHARA_E_NONE;
	}

	dhara_map_write(&my_map, 0, data, &err);
	err=DHARA_E_NONE;

	dhara_map_sync(&my_map, &err);
	err=DHARA_E_NONE;

	// Now power-cycle simulation
	memset(&my_map, 0, sizeof(my_map));
	dhara_map_init(&my_map, &my_nand, journal_buffer, GC_RATIO);

	// Resume again — should now find journal magic
	dhara_map_resume(&my_map, &err);
	err=DHARA_E_NONE;

	uint8_t readData[(1<<my_nand.log2_page_size)];

	dhara_map_read(&my_map, 0, readData, &err);
	err=DHARA_E_NONE;

	dhara_map_resume(&my_map, &err);
	err=DHARA_E_NONE;

	dhara_map_sync(&my_map, &err);
	err=DHARA_E_NONE;

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
	if(dataSize==PAGE_SIZE){
		return dhara_map_read(&my_map, s, data, err);
	}
	if(dataSize<PAGE_SIZE){
		uint8_t fullData[PAGE_SIZE];
		int status=dhara_map_read(&my_map, s, fullData, err);

		// Read FAILED!!!
		if(status==-1){return status;}

		memcpy(data,fullData,dataSize);

		return status;
	}

	//TODO SEE IF THIS IS A GOOD IDEA OR IF WE SHOULD HANDLE DIFFERENTLY
	// If dataSize is greater then the page size
	return -1;
}

int Dhara_Write(uint32_t s, const uint8_t *data, uint16_t dataSize, dhara_error_t *err)
{
	if(dataSize==PAGE_SIZE){
		return dhara_map_write(&my_map, s, data, err);
	}
	if(dataSize<PAGE_SIZE){
		uint8_t newData[PAGE_SIZE];

		// Copies data to new array of correct size and fills the rest with empty datapoints
		memcpy(newData, data, dataSize);
		for(unsigned int i=dataSize;i<PAGE_SIZE;i++){
			newData[i]=0xFF;
		}

		return dhara_map_write(&my_map, s, newData, err);
	}else{//TODO figure out what to do when data is tooo large for a single page
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

