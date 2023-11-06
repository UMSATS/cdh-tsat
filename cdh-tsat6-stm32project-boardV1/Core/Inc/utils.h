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
 * FUNCTION: rtc_to_unix_timestamp
 *
 * DESCRIPTION: Returns the Unix timestamp representation of the given RTC struct.
 *
 * PARAMETERS:
 *  rtc_value: The current RTC value.
 */
uint32_t rtc_to_unix_timestamp(RTC_TimeTypeDef rtc_value);

/*
 * FUNCTION: unix_timestamp_to_rtc
 *
 * DESCRIPTION: Returns the RTC struct representation of the given Unix timestamp.
 *
 * PARAMETERS:
 *  unix_timestamp: The current Unix timestamp.
 */
RTC_TimeTypeDef unix_timestamp_to_rtc(uint32_t unix_timestamp);

#endif /* INC_UTILS_H_ */
