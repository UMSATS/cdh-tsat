/*
 * FILENAME: telemetry_handling.c
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Feb 15, 2025
 */

#include "telemetry_handling.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rtc.h"

void StartTelemHandler(void *argument)
{
  TelemetryMessage_t telemetry_message;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(telemQueueHandle, &telemetry_message, NULL, osWaitForever);
    switch(telemetry_message.key>>4)
    {
    case TEL_PCB_TEMP:

    	// TODO check for HAL errors

    	TelemetryMessage_t temp;

		RTC_HandleTypeDef hrtc;

		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		temp.timestamp=rtc_to_unix_timestamp(sTime, sDate);// TIMESTAMP IS IN UNIX, CHECK telemetry.h file

		break;
    case TEL_MCU_TEMP:

		break;
    case TEL_RSSI:

		break;
    case TEL_CONVERTER_STATUS:

		break;
	case TEL_BATTERY_TEMP:

		break;
	case TEL_BATTERY_VOLTAGE:

		break;
	case TEL_BATTERY_CURRENT:

		break;
	case TEL_COULOMB_COUNT:

		break;
	case TEL_SOLAR_PANEL_TEMP:

		break;
	case TEL_SOLAR_PANEL_CURRENT:

		break;
	case TEL_MAGNETIC_FIELD:

		break;
	case TEL_ANGULAR_VELOCITY:

		break;
	case TEL_WELL_TEMP:

		break;
	case TEL_WELL_LUMINOSITY:

		break;
    default:
		break;
    }
  }
  osThreadExit();
}
