/*
 * deployment_tasks.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_DEPLOYMENT_TASKS_H_
#define INC_DEPLOYMENT_TASKS_H_

#include "stm32l4xx_hal.h"
#include "LTC1154_driver.h"
#include "cmsis_os.h"


/* USER CODE BEGIN Header_StartDeployA */
/**
* @brief Function implementing the deployA thread.
* @param argument: Not used
* @retval None
*/
void StartDeployA(void *argument);


/* USER CODE BEGIN Header_StartDeployB */
/**
* @brief Function implementing the deployB thread.
* @param argument: Not used
* @retval None
*/
void StartDeployB(void *argument);


#endif /* INC_DEPLOYMENT_TASKS_H_ */
