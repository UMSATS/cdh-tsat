/*
 * FILENAME: AS3001204_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the AS3001204-0054X0ISAY MRAM unit.
 *
 * AUTHORS:
 *  - Gabriel Young (gabriel.young@umsats.ca)
 *  - Om Sevak (Om.Sevak@umsats.ca)
 *
 * Created on: Nov. 14, 2022
 */


#include "AS3001204_driver.h"


//###############################################################################################
// Driver Functions
//###############################################################################################

HAL_StatusTypeDef AS3001204_Write_Enable() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_ENABLE);
}

HAL_StatusTypeDef AS3001204_Write_Disable(){
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_DISABLE);
}

HAL_StatusTypeDef AS3001204_Enter_Hibernate() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_HIBERNATE);
}

HAL_StatusTypeDef AS3001204_Enter_Deep_Power_Down() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_DEEP_PWDOWN);
}

HAL_StatusTypeDef AS3001204_Exit_Deep_Power_Down() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_EXIT_DEEP_PWDOWN)
}

HAL_StatusTypeDef AS3001204_Software_Reset_Enable() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_SOFT_RESET_ENABLE);
}

HAL_StatusTypeDef AS3001204_Software_Reset(){
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_SOFT_RESET);
}

//###############################################################################################
// Helper Functions
//###############################################################################################

HAL_StatusTypeDef AS3001204_Send_Basic_Command(uint8_t opcode) {

    HAL_StatusTypeDef isError;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);

    return isError;
}
