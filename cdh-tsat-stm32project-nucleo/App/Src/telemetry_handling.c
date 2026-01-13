/*
 * FILENAME: telemetry_handling.c
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Feb 15, 2025
 */

#include "telemetry_handling.h"

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
