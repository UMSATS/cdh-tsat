/*
 * command_handling.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_COMMAND_HANDLING_H_
#define INC_COMMAND_HANDLING_H_

#include "tuk/tuk.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

//extern osMessageQueueId_t canQueueHandle;
//extern osThreadId_t stm32ResetHandle;
//extern osThreadId_t flashUnitTestHandle;
//extern osThreadId_t mramUnitTestHandle;
//extern osThreadId_t deployAHandle;
//extern osThreadId_t deployBHandle;
//extern osThreadId_t getTasksNumHandle;
//extern osThreadId_t timeTagTaskInitQueueHandle;
//extern osThreadId_t setRTCQueueHandle;
//extern osThreadId_t getRTCHandle;
extern osThreadId_t telemHandlerHanHandle;

/**
* Handles incoming messages from the CAN bus. Called by CAN Wrapper Module.
*
* @param hcan Handle for the source CAN peripheral.
* @param msg The received message.
*/
void On_CAN_Message_Ready(const CAN_HandleTypeDef *hcan, const CANMessage *msg);

void On_CAN_Error(const CANWrapper_ErrorInfo *error);

#endif /* INC_COMMAND_HANDLING_H_ */
