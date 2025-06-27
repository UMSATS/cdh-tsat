/*
 * command_handling.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_COMMAND_HANDLING_H_
#define INC_COMMAND_HANDLING_H_

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "can.h"

extern osMessageQueueId_t canQueueHandle;
extern osThreadId_t stm32ResetHandle;
extern osThreadId_t flashUnitTestHandle;
extern osThreadId_t mramUnitTestHandle;
extern osThreadId_t deployAHandle;
extern osThreadId_t deployBHandle;
extern osThreadId_t getTasksNumHandle;
extern osThreadId_t timeTagTaskInitQueueHandle;
extern osThreadId_t setRTCQueueHandle;
extern osThreadId_t getRTCHandle;



/* USER CODE BEGIN Header_StartCanCmdHandler */
/**
* @brief Function implementing the canCmdHandler thread.
* @param argument: Not used
* @retval None
*/
void StartCanCmdHandler(void *argument);

#endif /* INC_COMMAND_HANDLING_H_ */
