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
 *   3. Internal helper function prototypes
 *   4. Public driver function definitions
 *       4.1. Basic commands
 *       4.2. Read registers
 *       4.3. Write registers
 *       4.4. Read/write memory
 *       4.5. Read/write Augmented Storage Array
 *       4.6. Device initialization
 *   5. Internal helper function definitions
 */


// ###############################################################################################
//  1. Includes
// ###############################################################################################
#include "AS3001204_driver.h"


// ###############################################################################################
//  2. Opcode definitions (see datasheet pp. 32-36)
// ###############################################################################################

// Control operations (1-0-0 type)
#define AS3001204_OPCODE_WRITE_DISABLE      0x04
#define AS3001204_OPCODE_WRITE_ENABLE		0x06
#define AS3001204_OPCODE_ENTER_HIBERNATE    0xba
#define AS3001204_OPCODE_ENTER_DEEP_PWDOWN	0xb9
#define AS3001204_OPCODE_EXIT_DEEP_PWDOWN	0xab

// Read register operations (1-0-1 type)
#define AS3001204_OPCODE_READ_STATUS_REG 	0x05
#define AS3001204_OPCODE_READ_CONFIG_REGS 	0x46
#define AS3001204_OPCODE_READ_DEVICE_ID 	0x9f
#define AS3001204_OPCODE_READ_UNIQUE_ID 	0x4c
#define AS3001204_OPCODE_READ_ASP_REG 		0x14

// Write register operations (1-0-1 type)
#define AS3001204_OPCODE_WRITE_STATUS_REG 	0x01
#define AS3001204_OPCODE_WRITE_CONFIG_REGS 	0x87
#define AS3001204_OPCODE_WRITE_ASP_REG 		0x1a

// Memory operations (1-1-1 type)
#define AS3001204_OPCODE_READ_MEMORY 		0x03
#define AS3001204_OPCODE_WRITE_MEMORY 		0x02
#define AS3001204_OPCODE_READ_AUG_STORAGE 	0x4b
#define AS3001204_OPCODE_WRITE_AUG_STORAGE 	0x42


// ###############################################################################################
//  3. Internal helper function prototypes
// ###############################################################################################

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

// Note: Our initialization settings include auto-disabling the software write enable following
//       every write instruction.
HAL_StatusTypeDef AS3001204_Write_Disable() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_DISABLE);
}

HAL_StatusTypeDef AS3001204_Write_Enable() {
    return AS3001204_Send_Basic_Command(AS3001204_OPCODE_WRITE_ENABLE);
}

HAL_StatusTypeDef AS3001204_Enter_Hibernate() {
    HAL_StatusTypeDef isError;
    isError = AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_HIBERNATE);
    if (isError != HAL_OK) return isError;
    HAL_Delay(1); // Ensure the AS3001204 has completed entering hibernation (~3us)
    return isError;
}

HAL_StatusTypeDef AS3001204_Exit_Hibernate() {
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1); // Ensure the AS3001204 has completed exiting hibernation (~450us)
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return HAL_OK;
}

HAL_StatusTypeDef AS3001204_Enter_Deep_Power_Down() {
    HAL_StatusTypeDef isError;
    isError = AS3001204_Send_Basic_Command(AS3001204_OPCODE_ENTER_DEEP_PWDOWN);
    if (isError != HAL_OK) return isError;
    HAL_Delay(1); // Ensure the AS3001204 has completed entering deep power down (~3us)
    return isError;
}

HAL_StatusTypeDef AS3001204_Exit_Deep_Power_Down() {
    HAL_StatusTypeDef isError;
    isError = AS3001204_Send_Basic_Command(AS3001204_OPCODE_EXIT_DEEP_PWDOWN);
    if (isError != HAL_OK) return isError;
    HAL_Delay(1); // Ensure the AS3001204 has completed exiting deep power down (~400us)
    return isError;
}

// ----------------------
//  4.2. Read registers
// ----------------------

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

HAL_StatusTypeDef AS3001204_Read_ASP_Register(uint8_t *p_buffer) {
    return AS3001204_Read_Register(AS3001204_OPCODE_READ_ASP_REG, p_buffer, AS3001204_ASP_REG_LENGTH);
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

HAL_StatusTypeDef AS3001204_Write_ASP_Register(uint8_t *p_buffer) {
    return AS3001204_Write_Register(AS3001204_OPCODE_WRITE_ASP_REG, p_buffer, AS3001204_ASP_REG_LENGTH);
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
    
    isError = HAL_SPI_Receive(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;

}

HAL_StatusTypeDef AS3001204_Write_Memory(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_WRITE_MEMORY;

    // This is a separate command, it does its own CS flipping
    isError = AS3001204_Write_Enable();
    if (isError != HAL_OK) return isError;

    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

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

// ------------------------------------------
//  4.5. Read/write Augmented Storage Array
// ------------------------------------------

HAL_StatusTypeDef AS3001204_Read_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_READ_AUG_STORAGE;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_SPI_Transmit_Memory_Address(address);
    if (isError != HAL_OK) goto error;

    //The AS3001204 requires minimum 8 latency cycles at 40 MHz between inputting the address & receiving the data.
    //That means there must be a minimum 200 ns delay.
    //The overhead between calling AS3001204_SPI_Transmit_Memory_Address(...) & HAL_SPI_Receive(...) separately
    //as opposed to a single HAL_SPI_TransmitReceive(...) call produces far more than a 200 ns delay for us.
    //Therefore, there is no need to configure the AS3001204 to output latency cycles for this command.
    
    isError = HAL_SPI_Receive(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    return isError;

}

HAL_StatusTypeDef AS3001204_Write_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes) {

    HAL_StatusTypeDef isError;
    uint8_t opcode = AS3001204_OPCODE_WRITE_AUG_STORAGE;

    // This is a separate command, it does its own CS flipping
    isError = AS3001204_Write_Enable();
    if (isError != HAL_OK) return isError;

    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

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

// -----------------------------
//  4.6. Device initialization
// -----------------------------

HAL_StatusTypeDef AS3001204_Init() {

	HAL_StatusTypeDef isError;

	/*
	 * Status register:
	 *  Enable hardware write protection
	 *  Leave other features disabled
	 */
	uint8_t STATUS_REG_INIT = 0x80;

	/*
	 * Configuration registers:
	 *  1. Unlock status register protection options; disable ASA write protection
	 *  2. Single SPI; zero memory read latency
	 *  3. Leave output driver strength unchanged; leave read wrapping disabled
	 *  4. Enforce software write enable (WREN) as prerequisite to all memory write instructions
	 */
	uint8_t CONFIG_REGS_INIT[AS3001204_CONFIG_REGS_LENGTH] = {0x00, 0x00, 0x60, 0x04};

	/*
	 * Augmented Storage Array protection register:
	 *  Leave ASA section-specific protection disabled
	 */
	uint8_t ASP_REG_INIT = 0x00;

	// Ensure the AS3001204 has completed internal initialization (~250us)
	HAL_Delay(1);

	// Set GPIO pins
	HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);

	// Write to registers
	isError = AS3001204_Write_Status_Register(&STATUS_REG_INIT);
	if (isError != HAL_OK) goto error;
	isError = AS3001204_Write_Config_Registers(CONFIG_REGS_INIT);
	if (isError != HAL_OK) goto error;
	isError = AS3001204_Write_ASP_Register(&ASP_REG_INIT);

error:
	return isError;

}


// ###############################################################################################
//  5. Internal helper function definitions
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

    // This is a separate command, it does its own CS flipping
    isError = AS3001204_Write_Enable();
    if (isError != HAL_OK) return isError;

    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &opcode, sizeof(opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;
    
    isError = HAL_SPI_Transmit(&AS3001204_SPI, p_buffer, num_of_bytes, AS3001204_SPI_DELAY);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);
    if (isError == HAL_OK)
        HAL_Delay(1); // Ensure the AS3001204 has completed the register write cycle (~5us)
    return isError;

}

static HAL_StatusTypeDef AS3001204_SPI_Transmit_Memory_Address(uint32_t address) {
    
    // Separate 3 bytes of address from the uint32 (most significant byte ignored)
    uint8_t address_bytes[3] = {(address >> 16) & 0xff, (address >> 8) & 0xff, (address) & 0xff};

    return HAL_SPI_Transmit(&AS3001204_SPI, address_bytes, 3, AS3001204_SPI_DELAY);

}
