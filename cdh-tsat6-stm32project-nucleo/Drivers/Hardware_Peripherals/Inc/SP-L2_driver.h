/*
 * FILENAME: SP-L2_driver.h
 *
 * DESCRIPTION: STM32L4 header driver file for the SP-L2 UHF radio transceiver
 *
 * AUTHORS:
 *  - Graham Driver (graham.driver@umsats.ca)
 *
 * CREATED ON: November 13th 2023
 */

#ifndef HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_
//###############################################################################################
// Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"
#include "main.h"

typedef enum
{
	SPL2_HAL_OK                     = HAL_OK,      //0x00
	SPL2_HAL_ERROR                  = HAL_ERROR,   //0x01
	SPL2_HAL_BUSY                   = HAL_BUSY,    //0x02
	SPL2_HAL_TIMEOUT                = HAL_TIMEOUT, //0x03
} SPL2_StatusTypeDef;

//###############################################################################################
// Function Prototypes
//###############################################################################################


/**
 * @brief Sends Message over SPI to the radio
 *
 * *Note* The Radio will use SPI 2 on the mcu.
 */
SPL2_StatusTypeDef SPL2_Spi_Send_Message(uint8_t * pData, size_t numToSend);


/**
 * @brief Receives Message over SPI to the radio
 *
 *
 */
SPL2_StatusTypeDef SPL2_Spi_Receive_Message(uint8_t * pData, size_t numToReceive);


/**
 * @brief Transmit & Receives Message over SPI to the radio
 *
 *
 */
SPL2_StatusTypeDef SPL2_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive);


/**
 * @brief Writes to the S2-LP registers.
 *
 *
 */
SPL2_StatusTypeDef S2LP_Spi_Write_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Reads the S2-LP registers.
 *
 *
 */
SPL2_StatusTypeDef S2LP_Spi_Read_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Writes to the S2-LP TX FIFO.
 *
 *
 */
SPL2_StatusTypeDef S2LP_Spi_Write_TX_Fifo(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Reads the S2-LP RX FIFO.
 *
 *
 */
SPL2_StatusTypeDef S2LP_Spi_Read_RX_Fifo(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Reads the S2-LP RX FIFO.
 *
 *
 */
SPL2_StatusTypeDef S2LP_Spi_Send_Command(uint8_t address, uint8_t n_regs, uint8_t* buffer);

/**
 * @brief Shuts down SP-L2.
 *
 * *NOTE* Page 24 of SP-L2 documentation Describes Reset procedure
 */
SPL2_StatusTypeDef S2LP_Reset();



#endif /* HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_ */
