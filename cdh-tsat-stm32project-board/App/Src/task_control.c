/*
 * time_tagged_task.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */
#include <task_control.h>

void StartTimeTagTaskInit(void *argument)
{
  /* USER CODE BEGIN StartTimeTagTaskInit */
  CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osMessageQueueGet(timeTagTaskInitQueueHandle, &can_message, NULL, osWaitForever);

    HAL_StatusTypeDef operation_status;
    RTC_AlarmTypeDef rtc_alarm;
    uint32_t unix_timestamp = four_byte_array_to_uint32(can_message.data);
    RTC_TimeTypeDef rtc_time = unix_timestamp_to_rtc_time(unix_timestamp);
    RTC_DateTypeDef rtc_date = unix_timestamp_to_rtc_date(unix_timestamp);

    rtc_alarm.AlarmTime = rtc_time;
    rtc_alarm.AlarmMask = RTC_ALARMMASK_NONE;
    rtc_alarm.SubSeconds = 0;
    rtc_alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    rtc_alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    rtc_alarm.AlarmDateWeekDay = rtc_date.Date;
    rtc_alarm.Alarm = RTC_ALARM_A;

    operation_status = HAL_RTC_SetAlarm_IT(&hrtc, &rtc_alarm, RTC_FORMAT_BIN);

    //TODO: Implement error handling for StartTimeTagTaskInit
  }
  osThreadExit();
  /* USER CODE END StartTimeTagTaskInit */
}


void StartGetTasksNum(void *argument)
{
  /* USER CODE BEGIN StartGetTasksNum */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    uint8_t tasks_num = (uint8_t) osThreadGetCount();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
  /* USER CODE END StartGetTasksNum */
}


void StartTimeTagTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from RTC alarm ISR
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    HAL_StatusTypeDef operation_status;
    CANMessage_t ack_message =
    {
      .priority = 0b0000111,
      .SenderID = 0x1,
      .DestinationID = 0x1,
      .command = 0x01,
      .data = {0x48, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}
    };

    operation_status = CAN_Transmit_Message(ack_message);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

error:
    if (operation_status != HAL_OK)
    {
      //TODO: Implement error handling for StartTimeTagTask
    }
  }
  osThreadExit();
}

