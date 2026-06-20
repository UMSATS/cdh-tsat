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

		// Acquire Storage mutex
		osMutexAcquire(telemStorageMutexHandle, osWaitForever);

		// Move active to backup first, so writes can continue safely
		Storage_Send_To_Backup(TELEM);

		SectorNode* cur = NULL;
		Storage_Get_SectorNode(TELEM, 1, 0, &cur);  // sType 1 = first backup level

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
				osMessageQueuePut(tempQueueID,&msg,0,osWaitForever);
			}

			cur=cur->nextSector;
		}

		// Releasing Storage mutex
		osMutexRelease(telemStorageMutexHandle);

		// Orbit simulation
		osDelay(10000);
	}
}
