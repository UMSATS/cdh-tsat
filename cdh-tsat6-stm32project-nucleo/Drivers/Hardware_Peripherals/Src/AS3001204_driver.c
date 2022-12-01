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


#include "../Inc/AS3001204_driver.h"

//###############################################################################################
// Helper Function Prototypes
//###############################################################################################

HAL_StatusTypeDef AS3001204_Send_Basic_Command(uint8_t opcode);
HAL_StatusTypeDef AS3001204_Read_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes);
HAL_StatusTypeDef AS3001204_Write_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes);
HAL_StatusTypeDef AS3001204_SPI_Transmit_Memory_Address(uint32_t address);

//###############################################################################################
// Driver Functions
//###############################################################################################

// Basic commands
HAL_StatusTypeDef AS3001204_Write_Enable() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_ENABLE);
}

HAL_StatusTypeDef AS3001204_Write_Disable() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_DISABLE);
}

HAL_StatusTypeDef AS3001204_Enter_Hibernate() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_HIBERNATE);
}

HAL_StatusTypeDef AS3001204_Enter_Deep_Power_Down() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_DEEP_PWDOWN);
}

HAL_StatusTypeDef AS3001204_Exit_Deep_Power_Down() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_EXIT_DEEP_PWDOWN);
}

HAL_StatusTypeDef AS3001204_Software_Reset_Enable() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_SOFT_RESET_ENABLE);
}

HAL_StatusTypeDef AS3001204_Software_Reset() {
    AS3001204_Send_Basic_Command(AS3001204_OPCODE_SOFT_RESET);
}

// Read register functions
HAL_StatusTypeDef AS3001204_Read_Status_Register(uint8_t *p_buffer) {
    AS3001204_Read_Register(AS3001204_OPCODE_READ_STATUS_REG, p_buffer, AS3001204_STATUS_REG_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Config_Registers(uint8_t *p_buffer) {
    AS3001204_Read_Register(AS3001204_OPCODE_READ_CONFIG_REGS, p_buffer, AS3001204_CONFIG_REGS_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Device_ID(uint8_t *p_buffer) {
    AS3001204_Read_Register(AS3001204_OPCODE_READ_DEVICE_ID, p_buffer, AS3001204_DEVICE_ID_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Unique_ID(uint8_t *p_buffer) {
    AS3001204_Read_Register(AS3001204_OPCODE_READ_UNIQUE_ID, p_buffer, AS3001204_UNIQUE_ID_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Augmented_Array_Protection_Register(uint8_t *p_buffer) {
    AS3001204_Read_Register(AS3001204_OPCODE_READ_AAP_REG, p_buffer, AS3001204_AAP_REG_LENGTH);
}

// Write register functions
HAL_StatusTypeDef AS3001204_Write_Status_Register(uint8_t *p_buffer) {
    AS3001204_Write_Enable();
    AS3001204_Write_Register(AS3001204_OPCODE_WRITE_STATUS_REG, p_buffer, AS3001204_STATUS_REG_LENGTH);
}

HAL_StatusTypeDef AS3001204_Write_Config_Registers(uint8_t *p_buffer) {
    AS3001204_Write_Enable();
    AS3001204_Write_Register(AS3001204_OPCODE_WRITE_CONFIG_REGS, p_buffer, AS3001204_CONFIG_REGS_LENGTH);
}

HAL_StatusTypeDef AS3001204_Write_Augmented_Array_Protection_Register(uint8_t *p_buffer) {
    AS3001204_Write_Enable();
    AS3001204_Write_Register(AS3001204_OPCODE_WRITE_AAP_REG, p_buffer, AS3001204_AAP_REG_LENGTH);
}


// Read/write memory functions
HAL_StatusTypeDef AS3001204_Read_Memory(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_READ_MEMORY;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Receive (&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;

}

HAL_StatusTypeDef AS3001204_Write_Memory(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_WRITE_MEMORY;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    AS3001204_Write_Enable();

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;
}


// Read/write augmented storage array functions
HAL_StatusTypeDef AS3001204_Read_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_READ_AUG_STORAGE;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Receive (&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;

}

HAL_StatusTypeDef AS3001204_Write_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_WRITE_AUG_STORAGE;
    uint8_t delay = AS3001204_WRITE_AUG_STORAGE_DELAY;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    AS3001204_Write_Enable();

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;

    // Transmitting 8 bytes of zeros to wait for output
    isError = HAL_SPI_Transmit(&AS3001204_SPI, &delay, sizeof(delay), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;
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

HAL_StatusTypeDef AS3001204_Read_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes) {
    
    HAL_StatusTypeDef isError;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Receive (&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;
}


HAL_StatusTypeDef AS3001204_Write_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;
}


HAL_StatusTypeDef AS3001204_SPI_Transmit_Memory_Address(uint32_t address) {

    HAL_StatusTypeDef isError;
    // Addresses for this device are only 3 bytes
    uint8_t word_24bit_high_byte = address >> 16;
    uint8_t word_24bit_mid_byte  = address >> 8;
    uint8_t word_16bit_low_byte  = address;

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &word_24bit_high_byte, 1, AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    isError = HAL_SPI_Transmit(&AS3001204_SPI, &word_24bit_mid_byte, 1, AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    isError = HAL_SPI_Transmit(&AS3001204_SPI, &word_16bit_low_byte, 1, AS3001204_SPI_DELAY);

error:
    return isError;
}

