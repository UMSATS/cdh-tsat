/*
 * unit_tests.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#include "unit_tests.h"

void StartFlashUnitTest(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    W25N_StatusTypeDef test_result = Test_W25N();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
}


void StartMramUnitTest(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    HAL_StatusTypeDef test_result = AS3001204_Test_MRAM_Driver();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
}

void StartDharaUnitTest(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    dhara_error_t test_result = Dhara_Test();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
}

void StartStorageManagerUnitTest(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    uint8_t temp[7] = {0};

    int test_result = Storage_Unit_Test(temp, 7);

    //TODO: Add CAN message transmit
  }
  osThreadExit();
}
