/*
 * FILENAME: telemetry.h
 *
 * DESCRIPTION: Telemetry data application code header file.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Andrew Driver (andrew.driver@umsats.ca)
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
// Defines
//###############################################################################################

#define MAX_NUM_OF_PACKET 2
#define DATA_SIZE 4

// Telemetry Data Size
#define TEL_PCB_TEMP_SIZE 2
#define TEL_MCU_TEMP_SIZE 2
#define TEL_RSSI_SIZE 1
#define TEL_CONVERTER_STATUS_SIZE 1
#define TEL_BATTERY_VOLTAGE_SIZE 6
#define TEL_BATTERY_CURRENT_SIZE 2
#define TEL_SOLAR_PANEL_CURRENT_SIZE 2
#define TEL_COULOMB_COUNT_SIZE 2
#define TEL_SOLAR_PANEL_TEMP_SIZE 2
#define TEL_BATTERY_TEMP_SIZE 2
#define TEL_MAGNETIC_FIELD_SIZE 7
#define TEL_ANGULAR_VELOCITY_SIZE 6
#define TEL_WELL_TEMP_SIZE 2
#define TEL_WELL_LUMINOSITY_SIZE 2

//###############################################################################################
// Structs
//###############################################################################################
typedef struct{
    uint8_t key; // Telemetry key, consists of the Telemetry Type Id and the Variant Number
    uint8_t sequence_number; // Sequence number
    uint32_t timestamp; // 4 BYTES UNIX TIME
    uint8_t data[MAX_NUM_OF_PACKET*DATA_SIZE]; // Telemetry data.
} TelemetryMessage_t;

#endif /* INC_TELEMETRY_H_ */
