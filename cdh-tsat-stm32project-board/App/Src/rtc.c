/*
 * rtc.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */
#include "rtc.h"

void StartSetRTC(void *argument)
{
  /* USER CODE BEGIN StartSetRTC */
  CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osMessageQueueGet(setRTCQueueHandle, &can_message, NULL, osWaitForever);

    HAL_StatusTypeDef operation_status;
    uint32_t unix_timestamp = four_byte_array_to_uint32(can_message.data);
    RTC_TimeTypeDef rtc_time = unix_timestamp_to_rtc_time(unix_timestamp);
    RTC_DateTypeDef rtc_date = unix_timestamp_to_rtc_date(unix_timestamp);

    operation_status = HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

error:
    if (operation_status != HAL_OK)
    {
      //TODO: Implement error handling for StartSetRTC
    }
  }
  osThreadExit();
}


void StartGetRTC(void *argument)
{
  /* USER CODE BEGIN StartGetRTC */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    HAL_StatusTypeDef operation_status;
    RTC_TimeTypeDef rtc_time;
    RTC_DateTypeDef rtc_date;
    uint32_t unix_timestamp;
    uint8_t response_data[6] = {0,0,0,0,0,0};

    operation_status = HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
    if (operation_status != HAL_OK) goto error;

    unix_timestamp = rtc_to_unix_timestamp(rtc_time, rtc_date);
    uint32_to_four_byte_array(unix_timestamp, response_data);

    //TODO: Add CAN message transmit

error:
    if (operation_status != HAL_OK)
    {
      //TODO: Implement error handling for StartGetRTC
    }
  }
  osThreadExit();
  /* USER CODE END StartGetRTC */
}
