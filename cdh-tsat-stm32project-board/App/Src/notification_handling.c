/*
 * notification_handling.c
 *
 *  Created on: Jan 25, 2026
 *      Author: jagritsharma
 */


#include "notification_handling.h"
#include "mram_partitions.h"
#include "AS3001204_driver.h"
#include "tuk/tuk.h"

extern CAN_HandleTypeDef hcan1;


static void Handle_Payload_Startup()
{
	PayloadState_t payload_state;

	AS3001204_Read_Memory((uint8_t *) &payload_state, PAYLOAD_STATE_MRAM_ADDRESS, sizeof(PayloadState_t));

	uint8_t msg_body[CAN_MAX_BODY_SIZE] = {0};

	SET_MSG_DATA(msg_body, 0, uint16_t, payload_state.active_envs);
	CANWrapper_Transmit(&hcan1, NODE_PAYLOAD, CMD_PLD_SET_ACTIVE_ENVS, msg_body);

	for (int i = 0; i < 16; i++) {
		uint8_t msg_body[CAN_MAX_BODY_SIZE] = {0};

		SET_MSG_DATA(msg_body, 0, uint8_t, i);
		SET_MSG_DATA(msg_body, 1, float, payload_state.setpoints[i]);

		CANWrapper_Transmit(&hcan1, NODE_PAYLOAD, CMD_PLD_SET_SETPOINT, msg_body);
	}

	SET_MSG_DATA(msg_body, 0, float, payload_state.tolerance);
	CANWrapper_Transmit(&hcan1, NODE_PAYLOAD, CMD_PLD_SET_TOLERANCE, msg_body);

}

void StartNotifHandler(void *argument)
{
	CANMessage msg;

	for(;;)
	{
		osMessageQueueGet(notificationQueueHandle, &msg, NULL, osWaitForever);

		NotificationID notif_id = GET_MSG_DATA(msg.body, 0, NotificationID);

		if(msg.sender == NODE_PAYLOAD && notif_id == NOTIFICATION_STARTUP)
		{
			Handle_Payload_Startup();
		}

	}

}

