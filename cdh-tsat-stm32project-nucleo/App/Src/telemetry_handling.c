/*
 * FILENAME: telemetry_handling.c
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *  - Jagrit Sharma (jagrit.sharma@umsats.ca)
 *
 * CREATED ON: Feb 15, 2025
 */

#include "telemetry_handling.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rtc.h"

#include "tuk/tuk.h"
#include "utils.h"

#include "StorageManager.h"

#define TIMEOUT_MS 5000

uint8_t get_expected_packets(uint8_t key) {
	TelemetryID telem_id = GET_TELEMETRY_ID(key);
    if (telem_id == TEL_BATTERY_VOLTAGE || telem_id == TEL_MAGNETIC_FIELD || telem_id == TEL_ANGULAR_VELOCITY) {
        return 2;
    }
    return 1;
}

void writeToFlash(TelemetryBuffer_t *buffer){
	TelemetryMessage_t telemMessage = {0};

	telemMessage.key = buffer->key;
	telemMessage.sequence_number = buffer->sequence_number;

	RTC_HandleTypeDef hrtc;

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	memcpy(telemMessage.data, buffer->data, MAX_NUM_OF_PACKET * DATA_SIZE);

	telemMessage.timestamp=rtc_to_unix_timestamp(sTime, sDate);

	Storage_Append(TELEM, (const uint8_t *)&telemMessage, sizeof(telemMessage));
}

void cleanTelemBuffers(TelemetryBuffer_t *telemBuffers){
	uint32_t TICK_FREQ = osKernelGetTickFreq();
	uint32_t TIMEOUT_TICKS = (uint32_t)TIMEOUT_MS * TICK_FREQ / 1000;

	uint32_t current_tick = osKernelGetTickCount();

	for(int i = 0; i < MAX_NUM_OF_BUFFERS; i++){
		if(telemBuffers[i].active == 1){
			if((current_tick - telemBuffers[i].timestamp) > TIMEOUT_TICKS){
				telemBuffers[i].active = 0;
			}
		}
	}
}

void StartTelemHandler(void *argument)
{
  CANMessage telemetry_can_message;
  static TelemetryBuffer_t telemBuffers[MAX_NUM_OF_BUFFERS] = {0};

  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(telemQueueHandle, &telemetry_can_message, NULL, osWaitForever);

    cleanTelemBuffers(telemBuffers);

    uint8_t key = telemetry_can_message.body[0];
    uint8_t sequence_number = telemetry_can_message.body[1];
    uint8_t packet_num = telemetry_can_message.body[2];
    uint8_t *data = &telemetry_can_message.body[3];

    TelemetryBuffer_t *buffer = NULL;

    for(int i = 0; i < MAX_NUM_OF_BUFFERS; i++){
    	if(telemBuffers[i].key == key && telemBuffers[i].sequence_number == sequence_number && telemBuffers[i].active == 1){
    		buffer = &telemBuffers[i];
    		buffer->packets++;
    		break;
    	}
    }

    if (buffer == NULL){
    	for(int i = 0; i < MAX_NUM_OF_BUFFERS; i++){
    		if(telemBuffers[i].active == 0){
    			buffer = &telemBuffers[i];
    			memset(buffer, 0, sizeof(TelemetryBuffer_t));
    			buffer->key = key;
    			buffer->sequence_number = sequence_number;
    			buffer->packets++;
    			buffer->active = 1;
    			buffer->timestamp = osKernelGetTickCount();
    			break;
    		}
    	}
    }

    memcpy(&buffer->data[packet_num * DATA_SIZE], data, DATA_SIZE);

    if(buffer->packets == get_expected_packets(key)){
    	writeToFlash(buffer);
    	buffer->active = 0;
    }

  }
  osThreadExit();
}
