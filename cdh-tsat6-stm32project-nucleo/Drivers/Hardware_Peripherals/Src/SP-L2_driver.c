/*
 * FILENAME: SP-L2_driver.c
 *
 * DESCRIPTION: STM32L4 driver file for the SP-L2 UHF radio transceiver
 *
 * AUTHORS:
 *  - Graham Driver (graham.driver@umsats.ca)
 *
 * CREATED ON: November 13th 2023
 */

#include "SP-L2_driver.h"

SPL2_StatusTypeDef SPL2_Spi_Send_Message(uint8_t * pData, size_t numToSend){

	return HAL_SPI_Transmit(&hspi2, pData, numToSend, HAL_MAX_DELAY);
}

SPL2_StatusTypeDef SPL2_Spi_Receive_Message(uint8_t * pData, size_t numToReceive){

	return HAL_SPI_Receive_DMA(&hspi2, pData, numToReceive);
}

SPL2_StatusTypeDef SPL2_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive){

	return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, numTransmitReceive);
}
