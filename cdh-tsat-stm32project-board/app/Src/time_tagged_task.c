/*
 * time_tagged_task.c
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */
#include "time_tagged_task.h"

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
