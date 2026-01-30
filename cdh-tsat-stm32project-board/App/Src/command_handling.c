/*
 * command_handling.c
 */

#include "command_handling.h"
#include "tuk/tuk.h"

void On_CAN_Message_Ready(const CAN_HandleTypeDef *hcan, const CANMessage *msg)
{
	switch (msg->cmd)
	{
	case CMD_COMM_RESET:
		osThreadFlagsSet(stm32ResetHandle, 0x0001);
		break;
	case CMD_CDH_TEST_FLASH:
		osThreadFlagsSet(flashUnitTestHandle, 0x0001);
		break;
	case CMD_CDH_TEST_MRAM:
		osThreadFlagsSet(mramUnitTestHandle, 0x0001);
		break;
	case CMD_CDH_ENABLE_ANTENNA:
		osThreadFlagsSet(deployAHandle, 0x0001);
		break;
	case CMD_CDH_DEPLOY_ANTENNA:
		osThreadFlagsSet(deployBHandle, 0x0001);
		break;
	case CMD_CDH_SET_RTC:
		osMessageQueuePut(setRTCQueueHandle, msg, 0, 0);
		break;
	case CMD_CDH_GET_RTC:
		osThreadFlagsSet(getRTCHandle, 0x0001);
		break;
	case CMD_CDH_PROCESS_NOTIFICATION:
		osMessageQueuePut(notificationQueueHandle, msg, 0, 0);
		break;
	default:
		break;
	}
}

void On_CAN_Error(const CANWrapper_ErrorInfo *error)
{
	// TODO: Handle CAN errors.
}

