/*
 * telemetry_handling.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#include "telemetry_handling.h"

void StartTelemHandler(void *argument)
{
  TelemetryMessage_t telemetry_message;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(telemQueueHandle, &telemetry_message, NULL, osWaitForever);
    switch(telemetry_message.id)
    {
      //TODO: Implement telemetry handling
    }
  }
  osThreadExit();
}
