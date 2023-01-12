/*
 * radio_spi_Si4464.c
 *
 *  Created on: Jan. 3, 2023
 *      Authors:
 *      Graham Driver
 *      Nikolaus J. Reichert <nikolaus.reichert@umsats.ca>
 */
#include <stdio.h>
#include <Si4464_driver.h>


HAL_StatusTypeDef Radio_SPI_Transmit_Message(uint8_t * pData, size_t numToSend){
	// TODO: Check which form of SPI Transfer function we should use: Blocking, IT, or DMA.
	// For now, Daigh noted we should use Blocking. -NJR
	// HAL_SPI_Transmit_IT(&hspi2, pData, sizeof(pData));
	return HAL_SPI_Transmit(&hspi2, pData, numToSend, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Radio_SPI_Receive_Message(uint8_t * pData, size_t numToReceive){
	// See note on Radio_SPI_Transmit_Message(). -NJR
	// HAL_SPI_Receive_IT(&hspi2, pData, sizeof(pData));
	return HAL_SPI_Receive(&hspi2, pData, numToReceive, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Radio_SPI_Transmit_Receive_Message(uint8_t * pTxData, uint8_t * pRxData, size_t numTransmittedReceived){

	// HAL_SPI_TransmitReceive_IT(&hspi2, pTxData, pRxData, sizeof(pTxData));
	return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, numTransmittedReceived, HAL_MAX_DELAY);
}

// TODO: Add pin for SDN and HAL variables.
HAL_StatusTypeDef Si4464_Reset_Device()
{
	// Page 21 of the datasheet implies that the minimum time is 10us? -NJR
	HAL_StatusTypeDef operation_status = HAL_OK;
	// operation_status = HAL_GPIO_WritePin();
	if (operation_status != HAL_OK) goto error;
	HAL_Delay(1);
	// operation_status = HAL_GPIO_WritePin();
	if (operation_status != HAL_OK) goto error;

error:
	return operation_status;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi2){
	// Handle Transmit Callback
	// Thoughts: Not really sure what a transmit callback would be useful for. -GD
	return;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi2){
	// Handle Receive Callback
	// Thoughts: The data should now be moved into a RTOS queue? -GD
	return;
}

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi2){
	// Handle Transmit Receive Callback
	// Thoughts: Similar to the regular receive the data can be moved into a RTOS Queue? - GD
	return;
}
