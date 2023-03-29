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
//  Functions for CAN initialization, message reception, and message transmission. Received messages are read into a Queue, which
//  can be handled by a dedicated task.
//
// History
// 2022-05-25 by Graham Driver
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES and Constants
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include "can.h"


/**
 * @brief Boots the CAN Bus
 * 
 * @return HAL_StatusTypeDef 
 */
void CAN_Boot(){
	CAN_FilterTypeDef  		sFilterConfig;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1); // Turn on CANBus

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/**
 * @brief Used to send messages over CAN
 * @param message A 8 byte message
 */
void CAN_Transmit_Message(CANMessage_t myMessage){
	uint32_t txMailbox; // Transmit Mailbox
	CAN_TxHeaderTypeDef txMessage;
	
	// TX Message Parameters
	uint16_t ID = (myMessage.priority << 4) | (SOURCE_ID << 2) | (myMessage.DestinationID);
	uint8_t message[8] = {myMessage.command, myMessage.data[0], myMessage.data[1], myMessage.data[2], myMessage.data[3], myMessage.data[4], myMessage.data[5],myMessage.data[6]};

	txMessage.StdId = ID;
	txMessage.IDE = CAN_ID_STD;
	txMessage.RTR = CAN_RTR_DATA;
	txMessage.DLC = MAX_CAN_DATA_LENGTH;
	HAL_CAN_AddTxMessage(&hcan1, &txMessage, message, &txMailbox);
}


/**
 * @brief Interrupt Handler for received CAN messages.
 */
void CAN_Message_Received(){
	CAN_RxHeaderTypeDef rxMessage; // Received Message Header
	uint8_t rxData[8]; // Received data
	uint8_t receivedDestinationId; // ID of Received Message

	// Message Sent To Queue
	/* Get RX message */
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxMessage, rxData);
	receivedDestinationId = RECEIVED_DESTINATION_ID_MASK & rxMessage.StdId;
	if(receivedDestinationId == SOURCE_ID){
		// *NOTE* program custom handling per your subsystem here.
		CANMessage_t ping;
		ping.DestinationID = 0x2;
		ping.command = rxData[0];
		ping.priority = 1;
		for(uint8_t i = 0; i <= 7; i++){
			ping.data[i] = rxData[i+1] + 1;
		}
		CAN_Transmit_Message(ping);
	}
}
