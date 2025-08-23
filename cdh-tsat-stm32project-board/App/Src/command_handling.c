/*
 * command_handling.c
 */

#include "command_handling.h"
#include "tuk/tuk.h"
#include "tuk/can_wrapper/telemetry_id.h"

extern float g_magnetic_field[3];

// Called when a new CAN message arrives
void On_CAN_Message_Ready(const CAN_HandleTypeDef *hcan, const CANMessage *msg)
{
	// TODO: These command ID's are OLD and must be updated. Refer to command reference.
	switch (msg->cmd)
	{
	case 0x40:
		osThreadFlagsSet(stm32ResetHandle, 0x0001);
		break;
	case 0x41:
		osThreadFlagsSet(flashUnitTestHandle, 0x0001);
		break;
	case 0x42:
		osThreadFlagsSet(mramUnitTestHandle, 0x0001);
		break;
	case 0x43:
		osThreadFlagsSet(deployAHandle, 0x0001);
		break;
	case 0x44:
		osThreadFlagsSet(deployBHandle, 0x0001);
		break;
	case 0x47:
		osThreadFlagsSet(getTasksNumHandle, 0x0001);
		break;
	case 0x48:
		osMessageQueuePut(timeTagTaskInitQueueHandle, msg, 0, 0);
		break;
	case 0x49:
		osMessageQueuePut(setRTCQueueHandle, msg, 0, 0);
		break;
	case 0x4A:
		osThreadFlagsSet(getRTCHandle, 0x0001);
		break;
	case CMD_CDH_PROCESS_TELEMETRY_REPORT:
	{
		TelemetryID tel_id = GET_TELEMETRY_ID(msg->body[0]);
		if (tel_id == TEL_MAGNETIC_FIELD && msg->sender == NODE_ADCS)
		{
			/* Expect three float values starting at byte 1 */
			memcpy(&g_magnetic_field[0], &msg->body[1], sizeof(float) * 3);

			/* Set an OS flag or notify waiting task here */
			osThreadFlagsSet(calculateBDotHandle, 0x0001);
		}
		break;
	}
	default:
		break;
	}
}

void On_CAN_Error(const CANWrapper_ErrorInfo *error)
{
	// TODO: Handle CAN errors.
}

