/*
 * FILENAME: utils.c
 *
 * DESCRIPTION: Util functions for use with CDH application code.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Nov. 6, 2023
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "utils.h"

//###############################################################################################
//Public Functions
//###############################################################################################
uint32_t four_byte_array_to_uint32(uint8_t *p_buffer)
{
  return (uint32_t)((*(p_buffer + 3) << 24) | (*(p_buffer + 2) << 16) | (*(p_buffer + 1) << 8) | *p_buffer);
}

void uint32_to_four_byte_array(uint32_t value32, uint8_t *p_buffer)
{
  *(p_buffer + 3) = (uint8_t)(value32 >> 24);
  *(p_buffer + 2) = (uint8_t)(value32 >> 16);
  *(p_buffer + 1) = (uint8_t)(value32 >> 8);
  *(p_buffer) = (uint8_t)(value32);
}

uint32_t rtc_to_unix_timestamp(RTC_TimeTypeDef rtc_time, RTC_DateTypeDef rtc_date)
{
  //TODO: Implement this function
}

RTC_TimeTypeDef unix_timestamp_to_rtc_time(uint32_t unix_timestamp)
{
  //TODO: Implement this function
}

RTC_DateTypeDef unix_timestamp_to_rtc_date(uint32_t unix_timestamp)
{
  //TODO: Implement this function
}
