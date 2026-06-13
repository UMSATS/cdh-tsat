/*
 * FILENAME: fetch_telem_data.c
 *
 *  AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 *  CREATED ON: May 30, 2026
 */

#include "stm32l4xx_hal.h"
#include "StorageManager.h"
#include "cmsis_os.h"

//#############################################
//##############    FUNCTIONS    ##############
//#############################################

void StartFetchTelemData(void *argument){
	// Task Variables

	for(;;){
		SectorNode* cur = NULL;
		Storage_Get_SectorNode(TELEM, 0, 0, &cur);

		while(cur!=NULL){

			// Writing data to TELEM storage
			uint8_t readData[PAGESIZE];

			// Read data associated with link list node
			Storage_Read(cur, readData, PAGESIZE);

			// Transmit Data
			//TransmitDataFunction(readData);

			cur=cur->nextSector;
		}

		Storage_Send_To_Backup(TELEM);

		// Orbit simulation
		osDelay(1);
	}
}
