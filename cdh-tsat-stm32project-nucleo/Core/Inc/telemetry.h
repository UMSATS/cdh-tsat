/*
 * FILENAME: telemetry.h
 *
 * DESCRIPTION: Telemetry data application code header file.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Nov. 5, 2023
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_
//###############################################################################################
// Include Directives
//###############################################################################################
#include <stdint.h>
#include "stm32l4xx_hal.h"

//###############################################################################################
// Structs
//###############################################################################################
typedef struct{
    uint8_t key; // Telemetry key
    uint8_t sequence_number; // Sequence number
    uint8_t timestamp[4]; // Bytes 4->0: Month, Day, Hours, Minutes, Seconds
    uint8_t data[7]; // Telemetry data
} TelemetryMessage_t;

#endif /* INC_TELEMETRY_H_ */
