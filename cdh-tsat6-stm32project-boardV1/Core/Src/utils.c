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
uint32_t rtc_to_unix_timestamp(RTC_TimeTypeDef rtc_value)
{
  //TODO: Implement this function
}

RTC_TimeTypeDef unix_timestamp_to_rtc(uint32_t unix_timestamp)
{
  //TODO: Implement this function
}
