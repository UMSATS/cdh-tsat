/*
 * FILENAME: W25N_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the W25N01GVZEIG NAND Flash.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * CREATED ON: Oct. 30, 2022
 */

#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "W25N_driver.h"


//###############################################################################################
//Driver Functions
//###############################################################################################
HAL_StatusTypeDef W25N_Device_Reset()
{
	HAL_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_DEVICE_RESET;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

HAL_StatusTypeDef W25N_Read_JEDEC_ID(uint8_t *p_buffer)
{
	HAL_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_READ_JEDEC_ID;
	uint8_t dummy_byte = W25N_DUMMY_BYTE;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
	if (operation_status != HAL_OK) goto error;

	operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, 3, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

HAL_StatusTypeDef W25N_Read_Status_Register(uint8_t register_address, uint8_t *p_buffer)
{
	HAL_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_READ_STATUS_REGISTER;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_address, 1, W25N_SPI_DELAY);
	if (operation_status != HAL_OK) goto error;

	operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, 1, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

HAL_StatusTypeDef W25N_Write_Status_Register(uint8_t register_address, uint8_t register_value)
{
    HAL_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_WRITE_STATUS_REGISTER;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_address, 1, W25N_SPI_DELAY);
	if (operation_status != HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_value, 1, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

HAL_StatusTypeDef W25N_Write_Enable()
{
    HAL_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_WRITE_ENABLE;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

HAL_StatusTypeDef W25N_Write_Disable()
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_WRITE_DISABLE;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

HAL_StatusTypeDef W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_BAD_BLOCK_MANAGEMENT;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(logical_block_address);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(physical_block_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

HAL_StatusTypeDef W25N_Read_BBM_LUT(uint8_t *p_buffer)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_READ_BBM_LUT;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;

    operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, W25N_BBM_LUT_NUM_OF_BYTES, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

//NOTE: CHECK IF ACTUALLY PAGE ADDRESS (ANY PAGE ADDRESS WITHIN BLOCK WE WANT TO ERASE)
//OR BLOCK ADDRESS (DATASHEET SAYS PAGE ADDRESS)
HAL_StatusTypeDef W25N_Block_Erase_128KB(uint16_t page_address)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_BLOCK_ERASE_128KB;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

HAL_StatusTypeDef W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_LOAD_PROGRAM_DATA;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(column_address);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, p_buffer, num_of_bytes, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

HAL_StatusTypeDef W25N_Program_Execute(uint16_t page_address)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_PROGRAM_EXECUTE;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

HAL_StatusTypeDef W25N_Page_Data_Read(uint16_t page_address)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_PAGE_DATA_READ;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

HAL_StatusTypeDef W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    HAL_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_READ_DATA;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(column_address);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;

    operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, num_of_bytes, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

//###############################################################################################
//Helper Functions
//###############################################################################################
HAL_StatusTypeDef W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit)
{
    HAL_StatusTypeDef operation_status;
    uint8_t word_16bit_high_byte = word_16bit >> 8; //value is truncated & high byte stored
    uint8_t word_16bit_low_byte = word_16bit; //value is truncated & low byte stored

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &word_16bit_high_byte, 1, W25N_SPI_DELAY);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &word_16bit_low_byte, 1, W25N_SPI_DELAY);

error:
    return operation_status;
}
