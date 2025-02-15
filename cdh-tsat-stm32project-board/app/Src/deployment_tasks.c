/*
 * deployment_tasks.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#include "deployment_tasks.h"

void StartDeployA(void *argument)
{
  /* USER CODE BEGIN StartDeployA */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    LTC1154_Enable();
  }
  osThreadExit();
  /* USER CODE END StartDeployA */
}


void StartDeployB(void *argument)
{
  /* USER CODE BEGIN StartDeployB */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    LTC1154_On();
  }
  osThreadExit();
  /* USER CODE END StartDeployB */
}
