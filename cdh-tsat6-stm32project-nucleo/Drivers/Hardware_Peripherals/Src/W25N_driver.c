/*
 * W25N_driver.c
 *
 *  Created on: Oct. 30, 2022
 *      Author: Owner
 */

#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "W25N_driver.h"


void W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    uint8_t opcode = OPCODE_READ_DATA;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, HAL_MAX_DELAY);
    W25N_SPI_Transmit_Word_16Bit(column_address);
    HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&W25N_SPI, p_buffer, num_of_bytes, HAL_MAX_DELAY);
}


//helper functions
void W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit)
{
    uint8_t word_16bit_high_byte = word_16bit >> 8; //value is truncated & high byte stored
    uint8_t word_16bit_low_byte = word_16bit; //value is truncated & low byte stored

    HAL_SPI_Transmit(&W25N_SPI, &word_16bit_high_byte, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&W25N_SPI, &word_16bit_low_byte, 1, HAL_MAX_DELAY);
}
