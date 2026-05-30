/*
 * FILENAME: telemetry_handling.h
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *  - Jagrit Sharma (jagrit.sharma@umsats.ca)
 *
 * CREATED ON: Feb 15, 2025
 */

#ifndef INC_TELEMETRY_HANDLING_H_
#define INC_TELEMETRY_HANDLING_H_

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "telemetry.h"
#include "tuk/can_wrapper/telemetry_id.h"

extern osMessageQueueId_t telemQueueHandle;

#define MAX_NUM_OF_BUFFERS 10


typedef struct {
	uint8_t key;
	uint8_t sequence_number;
	uint8_t packets;
	uint8_t data[MAX_NUM_OF_PACKET*DATA_SIZE];
	uint8_t active;
	uint32_t timestamp;
}TelemetryBuffer_t;


/**
* @brief Function implementing the telemHandler thread.
* @param argument: Not used
* @retval None
*/
void StartTelemHandler(void *argument);

#endif /* INC_TELEMETRY_HANDLING_H_ */
