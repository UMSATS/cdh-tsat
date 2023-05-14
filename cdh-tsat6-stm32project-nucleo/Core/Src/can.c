/*
 * FILENAME: can.c
 *
 * DESCRIPTION: Functions for CAN initialization, message reception, and message transmission.
 *              Received messages are read into a Queue, which can be handled by a dedicated task.
 *
 * Link to Documentation: https://drive.google.com/file/d/1HHNWpN6vo-JKY5VvzY14uecxMsGIISU7/view?usp=share_link
 *
 * AUTHORS:
 *  - Graham Driver (graham.driver@umsats.ca)
 *
 * CREATED ON: May 25, 2022
 */

//###############################################################################################
// Include Directives
//###############################################################################################
#include <stdio.h>
#include "can.h"

//###############################################################################################
//Public Functions
//###############################################################################################
/**
 * @brief Boots the CAN Bus
 * 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef CAN_Init(){
    HAL_StatusTypeDef operation_status;

	CAN_FilterTypeDef sFilterConfig;
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

	operation_status = HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	if (operation_status != HAL_OK) goto error;
	operation_status = HAL_CAN_Start(&hcan1); // Turn on the CAN Bus
	if (operation_status != HAL_OK) goto error;

	operation_status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

error:
    return operation_status;
}

/**
 * @brief Used to send messages over CAN
 *
 * @param myMessage: An 8 byte message
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Transmit_Message(CANMessage_t myMessage){
	uint32_t txMailbox; // Transmit Mailbox
	CAN_TxHeaderTypeDef txMessage;
	
	// TX Message Parameters
	uint16_t ID = (myMessage.priority << 4) | (SOURCE_ID << 2) | (myMessage.DestinationID);
	uint8_t message[8] = {myMessage.command, myMessage.data[0], myMessage.data[1], myMessage.data[2], myMessage.data[3], myMessage.data[4], myMessage.data[5],myMessage.data[6]};

	txMessage.StdId = ID;
	txMessage.IDE = CAN_ID_STD;
	txMessage.RTR = CAN_RTR_DATA;
	txMessage.DLC = MAX_CAN_DATA_LENGTH;

	return HAL_CAN_AddTxMessage(&hcan1, &txMessage, message, &txMailbox);
}

/**
 * @brief Interrupt Handler for received CAN messages
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Message_Received(){
    HAL_StatusTypeDef operation_status;
	CAN_RxHeaderTypeDef rxMessage; // Received Message Header
	uint8_t rxData[8]; // Received data
	uint8_t receivedDestinationId; // ID of Received Message

	/* Get RX message */
	operation_status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxMessage, rxData);
	if (operation_status != HAL_OK) goto error;
	receivedDestinationId = RECEIVED_DESTINATION_ID_MASK & rxMessage.StdId;

	if(receivedDestinationId == SOURCE_ID){
	    // *NOTE* Send message to queue per your subsystem here

		// *NOTE* program custom handling per your subsystem here
		CANMessage_t ping;
		ping.DestinationID = 0x2;
		ping.command = rxData[0];
		ping.priority = 1;
		for(uint8_t i = 0; i <= 6; i++){
			ping.data[i] = rxData[i+1] + 1;
		}
		operation_status = CAN_Transmit_Message(ping);
	}

error:
	return operation_status;
}
