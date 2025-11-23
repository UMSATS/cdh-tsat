/*
 * time_tagged_task.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_TASK_CONTROL_H_
#define INC_TASK_CONTROL_H_

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "utils.h"

extern osMessageQueueId_t timeTagTaskInitQueueHandle;
extern RTC_HandleTypeDef hrtc;


/**
* @brief Function implementing the timeTagTaskInit thread.
* @param argument: Not used
* @retval None
*/
void StartTimeTagTaskInit(void *argument);


/**
* @brief Function implementing the getTasksNum thread.
* @param argument: Not used
* @retval None
*/
void StartGetTasksNum(void *argument);


/**
* @brief Function implementing the timeTagTask thread.
* @param argument: Not used
* @retval None
*/
void StartTimeTagTask(void *argument);

#endif /* INC_TASK_CONTROL_H_ */
