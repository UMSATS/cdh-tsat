/*
 * command_handling.c
 */

#include "command_handling.h"
#include "tuk/tuk.h"

void On_CAN_Message_Ready(const CAN_HandleTypeDef *hcan, const CANMessage *msg)
{
	// TODO: These command ID's are OLD and must be updated. Refer to command reference.
	switch (msg->cmd)
	{
//	case 0x40:
//		osThreadFlagsSet(stm32ResetHandle, 0x0001);
//		break;
//	case 0x41:
//		osThreadFlagsSet(flashUnitTestHandle, 0x0001);
//		break;
//	case 0x42:
//		osThreadFlagsSet(mramUnitTestHandle, 0x0001);
//		break;
//	case 0x43:
//		osThreadFlagsSet(deployAHandle, 0x0001);
//		break;
//	case 0x44:
//		osThreadFlagsSet(deployBHandle, 0x0001);
//		break;
//	case 0x47:
//		osThreadFlagsSet(getTasksNumHandle, 0x0001);
//		break;
//	case 0x48:
//		osMessageQueuePut(timeTagTaskInitQueueHandle, msg, 0, 0);
//		break;
//	case 0x49:
//		osMessageQueuePut(setRTCQueueHandle, msg, 0, 0);
//		break;
//	case 0x4A:
//		osThreadFlagsSet(getRTCHandle, 0x0001);
//		break;
	case CMD_CDH_PROCESS_TELEMETRY_REPORT:
		osMessageQueuePut(telemQueueHandle, msg, 0, 0);
		break;
	default:
		break;
	}
}

void On_CAN_Error(const CANWrapper_ErrorInfo *error)
{
	// TODO: Handle CAN errors.
}

