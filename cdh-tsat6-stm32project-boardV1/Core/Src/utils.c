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

uint32_t rtc_to_unix_timestamp(RTC_TimeTypeDef rtc_value)
{
  //TODO: Implement this function
}

RTC_TimeTypeDef unix_timestamp_to_rtc(uint32_t unix_timestamp)
{
  //TODO: Implement this function
}
