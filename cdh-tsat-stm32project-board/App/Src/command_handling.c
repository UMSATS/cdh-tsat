/*
 * command_handling.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#include "command_handling.h"

/* USER CODE BEGIN Header_StartCanCmdHandler */
/**
* @brief Function implementing the canCmdHandler thread.
* @param argument: Not used
* @retval None
*/
void StartCanCmdHandler(void *argument)
{
  /* USER CODE BEGIN StartCanCmdHandler */
  CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(canQueueHandle, &can_message, NULL, osWaitForever);
    switch (can_message.command)
    {
      case 0x40:
        osThreadFlagsSet(stm32ResetHandle, 0x0001);
        break;
      case 0x41:
        osThreadFlagsSet(flashUnitTestHandle, 0x0001);
        break;
      case 0x42:
        osThreadFlagsSet(mramUnitTestHandle, 0x0001);
        break;
      case 0x43:
        osThreadFlagsSet(deployAHandle, 0x0001);
        break;
      case 0x44:
        osThreadFlagsSet(deployBHandle, 0x0001);
        break;
      case 0x47:
        osThreadFlagsSet(getTasksNumHandle, 0x0001);
        break;
      case 0x48:
        osMessageQueuePut(timeTagTaskInitQueueHandle, &can_message, 0, 0);
        break;
      case 0x49:
        osMessageQueuePut(setRTCQueueHandle, &can_message, 0, 0);
        break;
      case 0x4A:
        osThreadFlagsSet(getRTCHandle, 0x0001);
        break;
      default:
        break;
    }
  }
  osThreadExit();
  /* USER CODE END StartCanCmdHandler */
}
