/*
 * FILENAME: utils.h
 *
 * DESCRIPTION: Util functions for use with CDH application code.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Nov. 6, 2023
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_
//###############################################################################################
// Include Directives
//###############################################################################################
#include <stdint.h>
#include "stm32l4xx_hal.h"

//###############################################################################################
// Public Function Prototypes
//###############################################################################################
/*
 * FUNCTION: four_byte_array_to_uint32
 *
 * DESCRIPTION: Returns the uint32 representation of the given 4-byte array. The 4-byte array is
 *              interpreted as element 3 being the MSB & element 0 being the LSB.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to element 0 of the 4-byte array.
 */
uint32_t four_byte_array_to_uint32(uint8_t *p_buffer);

/*
 * FUNCTION: uint32_to_four_byte_array
 *
 * DESCRIPTION: Fills the given 4-byte array with the given 32-bit value. The 4-byte array is
 *              interpreted as element 3 being the MSB & element 0 being the LSB.
 *
 * PARAMETERS:
 *  value32: The 32-bit value.
 *  p_buffer: Pointer to element 0 of the 4-byte array.
 */
void uint32_to_four_byte_array(uint32_t value32, uint8_t *p_buffer);

/*
 * FUNCTION: rtc_to_unix_timestamp
 *
 * DESCRIPTION: Returns the Unix timestamp representation of the given RTC value.
 *
 * PARAMETERS:
 *  rtc_time: The current RTC time value.
 *  rtc_date: The current RTC date value.
 */
uint32_t rtc_to_unix_timestamp(RTC_TimeTypeDef rtc_time, RTC_DateTypeDef rtc_date);

/*
 * FUNCTION: unix_timestamp_to_rtc_time
 *
 * DESCRIPTION: Returns the RTC time value representation of the given Unix timestamp.
 *
 * PARAMETERS:
 *  unix_timestamp: The current Unix timestamp.
 */
RTC_TimeTypeDef unix_timestamp_to_rtc_time(uint32_t unix_timestamp);

/*
 * FUNCTION: unix_timestamp_to_rtc_date
 *
 * DESCRIPTION: Returns the RTC date value representation of the given Unix timestamp.
 *
 * PARAMETERS:
 *  unix_timestamp: The current Unix timestamp.
 */
RTC_DateTypeDef unix_timestamp_to_rtc_date(uint32_t unix_timestamp);

/*
 * FUNCTION: exponentialFilter
 *
 * DESCRIPTION: Implements a simple exponential moving average filter.
 *              Used for filtering magnetic field measurements in B-dot algorithm.
 *
 * PARAMETERS:
 *  curr: Current measurement value
 *  prev: Previous filtered value
 *  alpha: Filter coefficient (0 < alpha <= 1), where alpha=1 means no filtering
 *
 * RETURNS: Filtered value
 */
float exponentialFilter(float curr, float prev, float alpha);

/*
 * FUNCTION: MAG_ConvertToTeslas
 *
 * DESCRIPTION: Converts raw magnetometer readings to Tesla units.
 *              This is a placeholder implementation - adjust sensitivity based on your magnetometer.
 *
 * PARAMETERS:
 *  raw: Array of 3 raw magnetometer readings
 *  tesla: Output array for converted values in Tesla
 */
void MAG_ConvertToTeslas(int16_t raw[3], float tesla[3]);

#endif /* INC_UTILS_H_ */
