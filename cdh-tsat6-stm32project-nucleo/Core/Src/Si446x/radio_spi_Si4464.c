/*
 * radio_spi_Si4464.c
 *
 *  Created on: Jan. 3, 2023
 *      Author: Graham Driver
 */
#include <stdio.h>
#include "Si446x/radio_spi_Si4464.h"


/**
 * @brief Send non-blocking SPI message to SI4464 Radio
 * @param pData 8 bit data buffer
 */
void Radio_SPI_Transmit_Message(uint8_t * pData){
	HAL_SPI_Transmit_IT(&hspi2, pData, sizeof(pData));
}

/**
 * @brief Receive non-blocking SPI message from SI4464 Radio
 * @param pData 8 bit data buffer
 */
void Radio_SPI_Receive_Message(uint8_t * pData){
	HAL_SPI_Receive_IT(&hspi2, pData, sizeof(pData));
}

/**
 * @brief Transmit/Receive non-blocking SPI message to/from SI4464 Radio
 * @param pTxData 8 bit transmit data buffer
 * @param pRxData 8 bit receive data buffer
 * @note Ensure the buffer size of pTxData and pRxData are the same.
 */
void Radio_SPI_Transmit_Receive_Message(uint8_t * pTxData, uint8_t * pRxData){
	HAL_SPI_TransmitReceive_IT(&hspi2, pTxData, pRxData, sizeof(pTxData));
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
