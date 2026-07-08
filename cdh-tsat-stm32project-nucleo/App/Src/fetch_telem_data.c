/*
 * FILENAME: fetch_telem_data.c
 *
 *  AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 *  CREATED ON: May 30, 2026
 */

#include <string.h>

#include "stm32l4xx_hal.h"
#include "StorageManager.h"
#include "cmsis_os.h"
#include "telemetry.h"
#include "main.h"

/*
 * #######################    Task Description    #######################
 *
 * This file contains the task todo
 *
*/

// TODO REMOVE TEMP QUEUE FUNCTION AND REPLACE WITH REAL QUEUE
osMessageQueueId_t tempQueueID;
void tempQueue(osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout){}


// Mutex declaration
extern osMutexId_t telemStorageMutexHandle;

//#############################################
//##############    FUNCTIONS    ##############
//#############################################

void StartFetchTelemData(void *argument){
	// Task Variables

	for(;;){

		// Todo use Thread flags, Storage write and read task (gives better control maybe)
		// fix mutex stuck situation. ideas: copy whole link list and data and then push to backup, put copied data into queue. just believe in the queue being emptied.
		// copy just the sector numbers (not the data) into a temporary linked list you own, release the mutex, then read from flash using your own list copy and queue the data, then call Send_To_Backup..
		// could make a storage manager function that maybe pops the link list of something, but this will also need a put function to put the link list into the backup spot.

		// Acquire Storage mutex
		osMutexAcquire(telemStorageMutexHandle, osWaitForever);

		SectorNode* cur = NULL;
		Storage_Get_SectorNode(TELEM, 0, 0, &cur);  // sType 0  = active branch

		while(cur!=NULL){

			// Writing data to TELEM storage
			uint8_t readData[PAGESIZE];


			// Read data associated with link list node
			Storage_Read(cur, readData, PAGESIZE);

			// Fetches Header
			SectorHeader *hdr = (SectorHeader*)readData;

			// Fetches data section
			uint8_t *dataStart = readData + sizeof(SectorHeader);

			// Parse readData into telem messages
			for(uint32_t i=0;i<hdr->offset;i+=sizeof(TelemetryMessage_t)){
				// Fetching telem data
				TelemetryMessage_t msg;
				memcpy(&msg, dataStart + i, sizeof(TelemetryMessage_t));


				// Put transmit data into queue
				osMessageQueuePut(tempQueueID,&msg,0,osWaitForever);// TODO handle mutex when queue is stuck full, maybe out of range
			}

			cur=cur->nextSector;
		}

		// Move active to backup first, so writes can continue safely
		Storage_Send_To_Backup(TELEM);

		// Releasing Storage mutex
		osMutexRelease(telemStorageMutexHandle);

		// Orbit simulation
		osDelay(10000);
	}
}
