/*
 * FILENAME: utils.c
 *
 * DESCRIPTION: Util functions for use with CDH application code.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Arnav Gupta (arnav.gupta@umsats.ca)
 *
 * CREATED ON: Nov. 6, 2023
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>
#include <time.h>
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
	struct tm date_and_time = {
		.tm_sec = rtc_time.Seconds
		,.tm_min = rtc_time.Minutes
		,.tm_hour = rtc_time.Hours

		,.tm_mday = rtc_date.Date
		,.tm_mon = rtc_date.Month - 1
		,.tm_year = rtc_date.Year + 70

		,.tm_isdst = 0
	};

	// Resolve RTC_MONTH BCD inconsistencies
	if (date_and_time.tm_mon >= 10) { date_and_time.tm_mon -= 6; }

	return (uint32_t) mktime(&date_and_time);
}

RTC_TimeTypeDef unix_timestamp_to_rtc_time(uint32_t unix_timestamp)
{
	time_t unixFormatted = (time_t)unix_timestamp;

	struct tm date_and_time = *gmtime(&unixFormatted);

	RTC_TimeTypeDef rtc_time = {
		.Hours = date_and_time.tm_hour
		,.Minutes = date_and_time.tm_min
		,.Seconds = date_and_time.tm_sec
	};

	return rtc_time;
}

RTC_DateTypeDef unix_timestamp_to_rtc_date(uint32_t unix_timestamp)
{
	time_t unixFormatted = (time_t)unix_timestamp;

	struct tm date_and_time = *gmtime(&unixFormatted);

	RTC_DateTypeDef rtc_date = {
		.Date = date_and_time.tm_mday
		,.Month = date_and_time.tm_mon + 1
		,.Year = date_and_time.tm_year - 70
		,.WeekDay = date_and_time.tm_wday
	};

	// Resolve RTC_MONTH BCD inconsistencies
	if (rtc_date.Month >= 10) { rtc_date.Month += 6; }

	// Resolve RTC_WEEKDAY inconsistency
	if (rtc_date.WeekDay == 0) { rtc_date.WeekDay = 7; }

	return rtc_date;
}
