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
#include <Dhara_Wrapper.h>

#include "W25N_driver.h"



//################################################################################
//##########################    FLASH INITIALIZATION    ##########################
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
