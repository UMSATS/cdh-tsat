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
W25N_StatusTypeDef W25N_Device_Reset()
{
    W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_DEVICE_RESET;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;

    HAL_Delay(1); //The device needs max 500us to reset (no commands accepted during this time)

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Read_JEDEC_ID(uint8_t *p_buffer)
{
	W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_READ_JEDEC_ID;
	uint8_t dummy_byte = W25N_DUMMY_BYTE;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, 3, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Read_Status_Register(uint8_t register_address, uint8_t *p_buffer)
{
	W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_READ_STATUS_REGISTER;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_address, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, 1, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Write_Status_Register(uint8_t register_address, uint8_t register_value)
{
    W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_WRITE_STATUS_REGISTER;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_address, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_value, 1, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Write_Enable()
{
    W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_WRITE_ENABLE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Write_Disable()
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_WRITE_DISABLE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_BAD_BLOCK_MANAGEMENT;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(logical_block_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(physical_block_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Read_BBM_LUT(uint8_t *p_buffer)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_READ_BBM_LUT;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, W25N_BBM_LUT_NUM_OF_BYTES, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

//NOTE: CHECK IF ACTUALLY PAGE ADDRESS (ANY PAGE ADDRESS WITHIN BLOCK WE WANT TO ERASE)
//OR BLOCK ADDRESS (DATASHEET SAYS PAGE ADDRESS)
W25N_StatusTypeDef W25N_Block_Erase_128KB(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_BLOCK_ERASE_128KB;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_LOAD_PROGRAM_DATA;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(column_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, p_buffer, num_of_bytes, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Program_Execute(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_PROGRAM_EXECUTE;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Page_Data_Read(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_PAGE_DATA_READ;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_READ_DATA;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(column_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, num_of_bytes, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

//###############################################################################################
//High-Level Driver Functions
//###############################################################################################
void W25N_One_Time_Init()
{
    //this function should be completed stepping through one step at a time with the debugger
}

//make sure to note in function description that this unlocks all blocks for writing
//make sure to note in function desciptiion what configuration is set by the status register writes
W25N_StatusTypeDef W25N_Init()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_1_contents = 0b00000010;
    uint8_t register_1_address = 0xA0;
    uint8_t register_2_contents = 0b00011000;
    uint8_t register_2_address = 0xB0;

    HAL_Delay(1); //Ensure the W25N has completed internal initialization (~500us)

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET); //deselect the W25N
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET); //disable all write/program/erase functionality
    HAL_GPIO_WritePin(W25N_nHOLD_GPIO, W25N_nHOLD_PIN, GPIO_PIN_SET); //allow device operations (disable hold state)

    operation_status = W25N_Write_Status_Register(register_1_address, register_1_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Status_Register(register_2_address, register_2_contents);

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Wait_Until_Not_Busy()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if (!(register_contents & 0b00000001)) return W25N_READY;

    for (int i = 0; i < 10; i++) //10ms is the maximum possible busy time
    {
        HAL_Delay(1);
        operation_status = W25N_Read_Status_Register(register_address, &register_contents);
        if (operation_status != W25N_HAL_OK) goto error;
        if (!(register_contents & 0b00000001)) return W25N_READY;
    }
    operation_status = W25N_HANGING;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_LUT_Full()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b01000000) == 0b01000000) return W25N_LUT_FULL;
    else if ((register_contents & 0b01000000) == 0b00000000) return W25N_LUT_HAS_ROOM;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_ECC_Status()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b00110000) == 0b00000000) return W25N_ECC_CORRECTION_UNNECESSARY;
    else if ((register_contents & 0b00110000) == 0b00010000) return W25N_ECC_CORRECTION_OK;
    else if ((register_contents & 0b00110000) == 0b00100000) return W25N_ECC_CORRECTION_ERROR;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_Program_Failure()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b00001000) == 0b00001000) return W25N_PROGRAM_ERROR;
    else if ((register_contents & 0b00001000) == 0b00000000) return W25N_PROGRAM_OK;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_Erase_Failure()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b00000100) == 0b00000100) return W25N_ERASE_ERROR;
    else if ((register_contents & 0b00000100) == 0b00000000) return W25N_ERASE_OK;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

//should check ECC status after read operation & return status (higher level code takes care of what to do)
W25N_StatusTypeDef W25N_Read()
{

}

//should check program failure & return status (higher level code takes care of what to do)
W25N_StatusTypeDef W25N_Write()
{

}

//should check erase failure & return status (higher level code takes care of what to do)
W25N_StatusTypeDef W25N_Erase()
{

}

//maybe have something for testing the block erase function to see if it's page address or not
//write this up in main, the code won't need to ever be run again once we know.

//###############################################################################################
//Helper Functions
//###############################################################################################
W25N_StatusTypeDef W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit)
{
    W25N_StatusTypeDef operation_status;
    uint8_t word_16bit_high_byte = word_16bit >> 8; //value is truncated & high byte stored
    uint8_t word_16bit_low_byte = word_16bit; //value is truncated & low byte stored

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &word_16bit_high_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &word_16bit_low_byte, 1, W25N_SPI_DELAY);

error:
    return operation_status;
}

uint16_t W25N_Page_Address_To_Block_Address(uint16_t page_address)
{
    return page_address / W25N_PAGES_PER_BLOCK;
}
