/*
 * FILENAME: can.h
 *
 * DESCRIPTION: Functions for CAN initialization, message reception, and message transmission.
 *
 * Link to Documentation: https://drive.google.com/file/d/1HHNWpN6vo-JKY5VvzY14uecxMsGIISU7/view?usp=share_link
 *
 * AUTHORS:
 *  - Graham Driver (graham.driver@umsats.ca)
 *
 * CREATED ON: May 25, 2022
 */

#ifndef INCLUDE_CAN_H_
#define INCLUDE_CAN_H_
//###############################################################################################
// Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
// Define Directives & Extern Variables
//###############################################################################################
#define MAX_CAN_DATA_LENGTH  8
#define RECEIVED_SENDER_ID_MASK  0xC
#define RECEIVED_DESTINATION_ID_MASK  0x3
#define SOURCE_ID  0x1 // The ID number of the device MAX VALUE: 0x3

extern CAN_HandleTypeDef hcan1; // Set this to the CAN type found in generated main.c file

//###############################################################################################
// Structs
//###############################################################################################
typedef struct{
	uint8_t priority; // Priority of the message, MAX VALUE: 0x7F
	uint8_t SenderID; // The ID number of the sending device, MAX VALUE: 0x3
	uint8_t DestinationID; // The ID number of the destination device, MAX VALUE: 0x3
	uint8_t command; // The command value
	uint8_t data[7]; // Message
} CANMessage_t;

//###############################################################################################
// Public Function Prototypes
//###############################################################################################
HAL_StatusTypeDef CAN_Init(); // Initializes and Starts the CAN Bus

HAL_StatusTypeDef CAN_Transmit_Message(
        CANMessage_t myMessage // Uses the message struct to send messages
);

HAL_StatusTypeDef CAN_Message_Received(); // Interrupt handler for the CAN Bus

HAL_StatusTypeDef CAN_Send_Default_ACK(
        CANMessage_t myMessage // The message that the default ACK should be sent for
);

#endif /* INCLUDE_CAN_H_ */
