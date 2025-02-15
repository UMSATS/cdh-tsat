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
#include "can.h"

extern osMessageQueueId_t timeTagTaskInitQueueHandle;
extern RTC_HandleTypeDef hrtc;

void StartTimeTagTaskInit(void *argument);

#endif /* INC_TASK_CONTROL_H_ */
