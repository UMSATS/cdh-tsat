/*
 * rtc.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include <stdio.h>
#include <stdlib.h>

#include "MAX6822_driver.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "utils.h"

extern osMessageQueueId_t setRTCQueueHandle;;
extern RTC_HandleTypeDef hrtc;


/* USER CODE BEGIN StartSetRTC */
/**
* @brief Function implementing the StartSetRTC thread.
* @param argument: Not used
* @retval None
*/
void StartSetRTC(void *argument);


/* USER CODE BEGIN StartGetRTC */
/**
* @brief Function implementing the StartGetRTC thread.
* @param argument: Not used
* @retval None
*/
void StartGetRTC(void *argument);


#endif /* INC_RTC_H_ */
