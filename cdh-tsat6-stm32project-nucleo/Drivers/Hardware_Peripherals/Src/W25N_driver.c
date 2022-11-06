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
void W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address)
{
    uint8_t opcode = OPCODE_BAD_BLOCK_MANAGEMENT;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(logical_block_address);
    W25N_SPI_Transmit_Word_16Bit(physical_block_address);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

void W25N_Read_BBM_LUT(uint8_t *p_buffer)
{
    uint8_t opcode = OPCODE_READ_BBM_LUT;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&W25N_SPI, p_buffer, BBM_LUT_NUM_OF_BYTES, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

//NOTE: CHECK IF ACTUALLY PAGE ADDRESS (ANY PAGE ADDRESS WITHIN BLOCK WE WANT TO ERASE)
//OR BLOCK ADDRESS (DATASHEET SAYS PAGE ADDRESS)
void W25N_Block_Erase_128KB(uint16_t page_address)
{
    uint8_t opcode = OPCODE_BLOCK_ERASE_128KB;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(page_address);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

void W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    uint8_t opcode = OPCODE_LOAD_PROGRAM_DATA;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(column_address);
    HAL_SPI_Transmit(&W25N_SPI, p_buffer, num_of_bytes, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

void W25N_Program_Execute(uint16_t page_address)
{
    uint8_t opcode = OPCODE_PROGRAM_EXECUTE;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(page_address);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

void W25N_Page_Data_Read(uint16_t page_address)
{
    uint8_t opcode = OPCODE_PAGE_DATA_READ;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(page_address);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

void W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    uint8_t opcode = OPCODE_READ_DATA;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(column_address);
    HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&W25N_SPI, p_buffer, num_of_bytes, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
}

//###############################################################################################
//Helper Functions
//###############################################################################################
void W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit)
{
    uint8_t word_16bit_high_byte = word_16bit >> 8; //value is truncated & high byte stored
    uint8_t word_16bit_low_byte = word_16bit; //value is truncated & low byte stored

    HAL_SPI_Transmit(&W25N_SPI, &word_16bit_high_byte, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&W25N_SPI, &word_16bit_low_byte, 1, HAL_MAX_DELAY);
}
