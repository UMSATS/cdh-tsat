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
 *
 * CONTENTS:
 *   1. Includes
 *   2. Opcode definitions
 *   3. Private function prototypes
 *       3.1. Private driver functions
 *       3.2. Internal helper functions
 *   4. Public driver function definitions
 *       4.1. Basic commands
 *       4.2. Read registers
 *       4.3. Write registers
 *       4.4. Read/write memory
 *       4.5. Read/write Augmented Storage Array
 *   5. Private driver function definitions
 *   6. Internal helper function definitions
 */


// ###############################################################################################
//  1. Includes
// ###############################################################################################
#include "../Inc/AS3001204_driver.h"

// ###############################################################################################
//  2. Opcode definitions (see datasheet pp. 32-36)
// ###############################################################################################

// Control operations (1-0-0 type)
#define AS3001204_OPCODE_WRITE_ENABLE		0x06
#define AS3001204_OPCODE_WRITE_DISABLE		0x04
#define AS3001204_OPCODE_ENTER_DEEP_PWDOWN	0xb9
#define AS3001204_OPCODE_ENTER_HIBERNATE	0xba
#define AS3001204_OPCODE_EXIT_DEEP_PWDOWN	0xab
#define AS3001204_OPCODE_SOFT_RESET_ENABLE	0x66
#define AS3001204_OPCODE_SOFT_RESET			0x99

// Read register operations (1-0-1 type)
#define AS3001204_OPCODE_READ_STATUS_REG 	0x05
#define AS3001204_OPCODE_READ_CONFIG_REGS 	0x46
#define AS3001204_OPCODE_READ_DEVICE_ID 	0x9f
#define AS3001204_OPCODE_READ_UNIQUE_ID 	0x4c
#define AS3001204_OPCODE_READ_AAP_REG 		0x14

// Write register operations (1-0-1 type)
#define AS3001204_OPCODE_WRITE_STATUS_REG 	0x01
#define AS3001204_OPCODE_WRITE_CONFIG_REGS 	0x87
#define AS3001204_OPCODE_WRITE_AAP_REG 		0x1a

// Memory operations (1-1-1 type)
#define AS3001204_OPCODE_READ_MEMORY 		0x03
#define AS3001204_OPCODE_WRITE_MEMORY 		0x02
#define AS3001204_OPCODE_READ_AUG_STORAGE 	0x4b
#define AS3001204_OPCODE_WRITE_AUG_STORAGE 	0x42

// Delay bytes to await response from augmented storage array
// (see timing diagram, datasheet pp. 38-39)
#define AS3001204_READ_AUG_STORAGE_DELAY 	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


// ###############################################################################################
//  3. Private function prototypes
// ###############################################################################################

/*
 * 3.1. Private driver functions 
 *
 * FUNCTIONS:   AS3001204_Write_Enable, AS3001204_Write_Disable,
 *              AS3001204_Software_Reset_Enable,
 *
 * DESCRIPTION: These functions send basic commands to the MRAM device, which consist only
 *              of an opcode (no memory addresses or datastreams to read/write).
 *              Note that the write enable and disable functions relate to the MRAM's software
 *              write protection, which is separate from the write protect pin.
 */
static HAL_StatusTypeDef AS3001204_Write_Enable();
static HAL_StatusTypeDef AS3001204_Write_Disable();
static HAL_StatusTypeDef AS3001204_Software_Reset_Enable();

/*
 * 3.2. Internal helper functions
 */
static HAL_StatusTypeDef AS3001204_Send_Basic_Command(uint8_t opcode);
static HAL_StatusTypeDef AS3001204_Read_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes);
static HAL_StatusTypeDef AS3001204_Write_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes);
static HAL_StatusTypeDef AS3001204_SPI_Transmit_Memory_Address(uint32_t address);


// ###############################################################################################
//  4. Public driver function definitions
// ###############################################################################################

// ----------------------
//  4.1. Basic commands
// ----------------------

HAL_StatusTypeDef AS3001204_Enter_Hibernate() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_HIBERNATE);
}

HAL_StatusTypeDef AS3001204_Enter_Deep_Power_Down() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_DEEP_PWDOWN);
}

HAL_StatusTypeDef AS3001204_Exit_Deep_Power_Down() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_EXIT_DEEP_PWDOWN);
}


HAL_StatusTypeDef AS3001204_Software_Reset() {
	HAL_StatusTypeDef isError;
	isError = AS3001204_Software_Reset_Enable();
	if (isError != HAL_OK) return isError;

	isError = AS3001204_Send_Basic_Command(AS3001204_OPCODE_SOFT_RESET);
	return isError;
}

// ----------------------
//  4.2. Read registers
// ----------------------

// TODO: Should we null-check here, or at a lower/higher level? -NJR
HAL_StatusTypeDef AS3001204_Read_Status_Register(uint8_t *p_buffer) {
    return AS3001204_Read_Register(AS3001204_OPCODE_READ_STATUS_REG, p_buffer, AS3001204_STATUS_REG_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Config_Registers(uint8_t *p_buffer) {
    return AS3001204_Read_Register(AS3001204_OPCODE_READ_CONFIG_REGS, p_buffer, AS3001204_CONFIG_REGS_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Device_ID(uint8_t *p_buffer) {
    return AS3001204_Read_Register(AS3001204_OPCODE_READ_DEVICE_ID, p_buffer, AS3001204_DEVICE_ID_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Unique_ID(uint8_t *p_buffer) {
    return AS3001204_Read_Register(AS3001204_OPCODE_READ_UNIQUE_ID, p_buffer, AS3001204_UNIQUE_ID_LENGTH);
}

HAL_StatusTypeDef AS3001204_Read_Augmented_Array_Protection_Register(uint8_t *p_buffer) {
    return AS3001204_Read_Register(AS3001204_OPCODE_READ_AAP_REG, p_buffer, AS3001204_AAP_REG_LENGTH);
}


// -----------------------
//  4.3. Write registers
// -----------------------

HAL_StatusTypeDef AS3001204_Write_Status_Register(uint8_t *p_buffer) {
    return AS3001204_Write_Register(AS3001204_OPCODE_WRITE_STATUS_REG, p_buffer, AS3001204_STATUS_REG_LENGTH);
}

HAL_StatusTypeDef AS3001204_Write_Config_Registers(uint8_t *p_buffer) {
    return AS3001204_Write_Register(AS3001204_OPCODE_WRITE_CONFIG_REGS, p_buffer, AS3001204_CONFIG_REGS_LENGTH);
}

HAL_StatusTypeDef AS3001204_Write_Augmented_Array_Protection_Register(uint8_t *p_buffer) {
    return AS3001204_Write_Register(AS3001204_OPCODE_WRITE_AAP_REG, p_buffer, AS3001204_AAP_REG_LENGTH);
}


// -------------------------
//  4.4. Read/write memory
// -------------------------

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
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);

    AS3001204_Write_Enable();

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);
    return isError;
}



// -----------------------------------
//  4.5. R/W Augmented Storage Array
// -----------------------------------

HAL_StatusTypeDef AS3001204_Read_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_READ_AUG_STORAGE;
    uint8_t delay[] = AS3001204_READ_AUG_STORAGE_DELAY;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;

    // TODO: Check the behaviour of the wait time here with a logic analyzer. -NJR
    // Transmitting 8 bytes of zeros to wait for output from aug. storage array
    isError = HAL_SPI_Transmit(&AS3001204_SPI, delay, sizeof(delay), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Receive (&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;

}

HAL_StatusTypeDef AS3001204_Write_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_WRITE_AUG_STORAGE;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);

    AS3001204_Write_Enable();

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);
    return isError;
}


// ###############################################################################################
//  5. Private driver function definitions
// ###############################################################################################

static HAL_StatusTypeDef AS3001204_Write_Enable() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_ENABLE);
}

static HAL_StatusTypeDef AS3001204_Write_Disable() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_DISABLE);
}

static HAL_StatusTypeDef AS3001204_Software_Reset_Enable() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_SOFT_RESET_ENABLE);
}


// ###############################################################################################
//  6. Internal helper function definitions
// ###############################################################################################

static HAL_StatusTypeDef AS3001204_Send_Basic_Command(uint8_t opcode) {

    HAL_StatusTypeDef isError;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);

    return isError;
}

static HAL_StatusTypeDef AS3001204_Read_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes) {
    
    HAL_StatusTypeDef isError;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Receive(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;
}


static HAL_StatusTypeDef AS3001204_Write_Register(uint8_t opcode, uint8_t *p_buffer, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);

    isError = AS3001204_Write_Enable();
    if (isError != HAL_OK) goto error;

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);
    return isError;
}


static HAL_StatusTypeDef AS3001204_SPI_Transmit_Memory_Address(uint32_t address) {
    
    // Separate 3 bytes of address from the uint32 (most significant byte ignored)
    uint8_t address_bytes[3] = {(address >> 16) & 0xff, (address >> 8) & 0xff, (address) & 0xff};

    return HAL_SPI_Transmit(&AS3001204_SPI, address_bytes, 3, AS3001204_SPI_DELAY);
}
