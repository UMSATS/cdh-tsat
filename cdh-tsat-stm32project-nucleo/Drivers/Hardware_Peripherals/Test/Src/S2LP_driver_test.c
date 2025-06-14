/*
 * S2LP_driver_test.c
 *
 *  Created on: Feb 25, 2025
 *      Author: drive
 */

#include "S2LP_driver_test.h"


S2LP_StatusTypeDef Test_S2LP_Transmission(uint8_t *message, uint32_t messagesToSend, uint32_t size, uint32_t delay){
	uint8_t S2LPStatusRegisters[2] = {0};
	uint8_t txFIFOSize = 0;
	uint32_t i = 0;
	while(i < messagesToSend){
		S2LP_Send_Command(COMMAND_SABORT);
		S2LP_Send_Command(COMMAND_LOCKTX);
		S2LP_Get_Status(S2LPStatusRegisters);

		while((S2LPStatusRegisters[1] >> 0x1) != S2LP_STATE_LOCKON){
			S2LP_Get_Status(S2LPStatusRegisters);
		}

		if((S2LPStatusRegisters[1] >> 0x1) == S2LP_STATE_LOCKON){
	          S2LP_Write_TX_Fifo(size, message);
	          S2LP_Check_TX_FIFO_Status(&txFIFOSize);
	          if (txFIFOSize > 0) {
 	              S2LP_Send_Command(COMMAND_TX);
 	              i++;
	              HAL_Delay(delay);
	          } else {
	              printf("TX FIFO is empty, not sending TX command.\n");
	          }
	      }
	      else{
	          __NOP();
	      }
	}

	return S2LP_HAL_OK;
}

