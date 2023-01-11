/*
 * radio_spi_Si4464.c
 *
 *  Created on: Jan. 3, 2023
 *      Authors:
 *      Graham Driver
 *      Nikolaus J. Reichert <nikolaus.reichert@umsats.ca>
 */
#include <stdio.h>
#include "radio_spi_Si4464.h"


/**
 * @brief Send data to the Radio.
 * Effectively a wrapper for HAL_SPI_Transmit().
 *
 * @param pData A pointer to the data to send.
 * @param numToSend The number of bytes to send.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Radio_SPI_Transmit_Message(uint8_t * pData, size_t numToSend){
	// TODO: Check which form of SPI Transfer function we should use: Blocking, IT, or DMA.
	// For now, Daigh noted we should use Blocking. -NJR
	// HAL_SPI_Transmit_IT(&hspi2, pData, sizeof(pData));
	return HAL_SPI_Transmit(&hspi2, pData, numToSend, HAL_MAX_DELAY);
}

/**
 * @brief Receive up to numToReceive bytes of data from the Radio.
 * Effecvively a wrapper for HAL_SPI_Receive().
 *
 * @param pData A pointer to where to receive data.
 * @param numToReceive The maximum number of bytes to receive.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Radio_SPI_Receive_Message(uint8_t * pData, size_t numToReceive){
	// See note on Radio_SPI_Transmit_Message(). -NJR
	// HAL_SPI_Receive_IT(&hspi2, pData, sizeof(pData));
	return HAL_SPI_Receive(&hspi2, pData, numToReceive, HAL_MAX_DELAY);
}

/**
 * @brief Transmit/Receive numTransmittedReceived bytes to/from the Radio.
 * Effectively a wrapper for HAL_SPI_TransmitReceive().
 *
 * @param pTxData A pointer to the data to send.
 * @param pRxData A pointer to where to receive data.
 * @prarm numTransmittedReceived The number of bytes to handle.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Radio_SPI_Transmit_Receive_Message(uint8_t * pTxData, uint8_t * pRxData, size_t numTransmittedReceived){

	// HAL_SPI_TransmitReceive_IT(&hspi2, pTxData, pRxData, sizeof(pTxData));
	return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, numTransmittedReceived, HAL_MAX_DELAY);
}

/**
 * @brief Transmit Callback Function
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi2){
	// Handle Transmit Callback
	// Thoughts: Not really sure what a transmit callback would be useful for. -GD
	return;
}

/**
 * @brief Receive Callback Function
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi2){
	// Handle Receive Callback
	// Thoughts: The data should now be moved into a RTOS queue? -GD
	return;
}

/**
 * @brief Transmit/Receive Callback Function
 */
void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi2){
	// Handle Transmit Receive Callback
	// Thoughts: Similar to the regular receive the data can be moved into a RTOS Queue? - GD
	return;
}
