/*
 * FILENAME: SP-L2_driver.h
 *
 * DESCRIPTION: STM32L4 header driver file for the SP-L2 UHF radio transceiver
 *
 * AUTHORS:
 *  - Graham Driver (graham.driver@umsats.ca)
 *
 * CREATED ON: November 13th 2023
 * 
 * Functions left to write (Add or Remove as necessary):
 * S2LP_Spi_Write_Registers
 * S2LP_Spi_Read_Registers
 * S2LP_Reset
 */

#ifndef HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_
//###############################################################################################
// Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define S2LP_CS_SELECT 0
#define S2LP_CS_RELEASE 1

//###############################################################################################
// Radio State Codes
//###############################################################################################

typedef enum
{
S2LP_STATE_STANDBY		=	0x02,
S2LP_STATE_SLEEP_A 		=	0x01,
S2LP_STATE_SLEEP_B 		=	0x03,
S2LP_STATE_READY		=	0x00,
S2LP_STATE_LOCKST		=	0x14,
S2LP_STATE_LOCKON		=	0x0C,
S2LP_STATE_RX			=	0x30,
S2LP_STATE_TX			=	0x5C,
S2LP_STATE_SYNTH_SETUP	=	0x50,
S2LP_STATE_WAIT_SLEEP	=	0x7C,
} S2LP_STATE;


//###############################################################################################
// Radio Error Codes
//###############################################################################################
typedef enum
{
	S2LP_HAL_OK                     = HAL_OK,      	//0x00
	S2LP_HAL_ERROR                  = HAL_ERROR,   	//0x01
	S2LP_HAL_BUSY                   = HAL_BUSY,    	//0x02
	S2LP_HAL_TIMEOUT                = HAL_TIMEOUT, 	//0x03
	S2LP_TX_FIFO_FULL,
} S2LP_StatusTypeDef;

//###############################################################################################
// Function Prototypes
//###############################################################################################


/**
 * @brief Sends Message over SPI to the radio
 *
 * *Note* The Radio will use SPI 2 on the mcu.
 */
S2LP_StatusTypeDef S2LP_SPI_Transmit_Message(uint8_t * pData, size_t numToSend);


/**
 * @brief Receives Message over SPI to the radio
 *
 *
 */
S2LP_StatusTypeDef S2LP_SPI_Receive_Message(uint8_t * pData, size_t numToReceive);


/**
 * @brief Transmit & Receives Message over SPI to the radio
 *
 *
 */
S2LP_StatusTypeDef S2LP_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive);


/**
 * @brief Writes to the S2-LP registers.
 *
 *
 */
S2LP_StatusTypeDef S2LP_Spi_Write_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Reads the S2-LP registers.
 *
 *
 */
S2LP_StatusTypeDef S2LP_Spi_Read_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer);

/**
 * @brief This function will read the register that contains a counter for the amount of bytes in TX FIFO.
 * 
 * @param lengthBuffer The amount of bytes in the FIFO
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Check_TX_FIFO_Status(uint8_t * lengthBuffer);

/**
 * @brief This function will send data to the S2LP driver if there is space.
 * If there is no space available in the FIFO it will return S2LP_TX_FIFO_FULL.
 * 
 * @param size The size of the packet that is being sent
 * @param buffer A buffer containing data to be sent
 * @return S2LP_StatusTypeDef 
 * 
 * @note The TX FIFO can only store 128 bytes 
 */
S2LP_StatusTypeDef S2LP_Write_TX_Fifo(uint8_t size, uint8_t* buffer);


/**
 * @brief Reads given amount of bytes from RX FIFO.
 * 
 * @param n_bytes number of bytes to read
 * @param buffer buffer for received bytes
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Read_RX_FIFO(uint8_t n_bytes, uint8_t* buffer);

/**
 * @brief Fetches a count of RX values waiting in the FIFO
 * 
 * @param lengthBuffer return buffer for length
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Check_RX_FIFO_Status(uint8_t * lengthBuffer);

/**
 * @brief Will send the given command to the radio transceiver.
 * 
 * @param commandCode Command for S2LP
 * @return S2LP_StatusTypeDef 
 * 
 * @note This function does not check for compliance with the S2LP
 * current state for the given command.
 */
S2LP_StatusTypeDef S2LP_Send_Command(uint8_t commandCode);

/**
 * @brief Shuts down SP-L2.
 *
 * *NOTE* Page 24 of SP-L2 documentation Describes Reset procedure
 */
S2LP_StatusTypeDef S2LP_Reset();

/**
 * @brief Will either pull down chip select when passed S2LP_CS_SELECT,
 * or released high with S2LP_CS_RELEASE.
 * 
 * @param sel Use S2LP_CS_SELECT and S2LP_CS_RELEASE to select the device or release it
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_nCS(uint8_t sel);

#endif /* HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_ */
