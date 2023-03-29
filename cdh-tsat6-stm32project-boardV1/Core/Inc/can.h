#ifndef INCLUDE_CAN_H_
#define INCLUDE_CAN_H_
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// UMSATS 2022
//
// License:
//  Available under MIT license.
//
// Repository:
//  Github: https://github.com/UMSATS/cdh-tsat6
//
// File Description:
//  Functions for CAN initialization, message reception, and message transmission.
//
// History
// 2022-05-25 by Graham Driver
// - Created/Adapted from tsat5.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLDUES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "stm32l4xx_hal.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DECLARATIONS & Definitions
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define MAX_CAN_DATA_LENGTH  8
#define RECEIVED_DESTINATION_ID_MASK  0x3
#define SOURCE_ID  0x3 // The ID number of the device MAX VALUE: 0x3

extern CAN_HandleTypeDef hcan1; // Set this to the CAN type found in generated main.c file
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct{
	uint8_t priority; // Priority of the message MAX VALUE: 0x1F
	uint8_t DestinationID; // The ID number of the destination device MAX VALUE: 0x3
	uint8_t command;	// The command value
	uint8_t data[7];	// Message
} CANMessage_t;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


void CAN_Boot(); // Initializes and Starts the CAN


void CAN_Transmit_Message(
		CANMessage_t myMessage // Uses the message struct to send messages
);


void CAN_Message_Received();    // Interrupt handler for CAN BUS
#endif /* INCLUDE_CAN_H_ */
