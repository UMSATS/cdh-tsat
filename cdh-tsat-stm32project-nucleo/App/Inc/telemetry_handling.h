/*
 * telemetry_handling.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_TELEMETRY_HANDLING_H_
#define INC_TELEMETRY_HANDLING_H_

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "telemetry.h"

extern osMessageQueueId_t telemHandlerHanHandle;


/**
* @brief Function implementing the telemHandler thread.
* @param argument: Not used
* @retval None
*/
void StartTelemHandler(void *argument);

#endif /* INC_TELEMETRY_HANDLING_H_ */
