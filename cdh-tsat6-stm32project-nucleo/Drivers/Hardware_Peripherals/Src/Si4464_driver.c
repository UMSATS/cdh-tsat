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

HAL_StatusTypeDef Si4464_Send_Command(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size, uint8_t *returned_bytes, size_t return_size)
{
	// HACK ALERT!!!!!!!!!!!!!!
	// TODO: DO NULL CHECKING HERE!
	// OUR CURRENT ERROR TYPEDEFS DO NOT ALLOW MORE THAN ONE ERROR CODE!
	// Maybe create a new error enum that builds off HAL_StatusTypeDef? -NJR
	HAL_StatusTypeDef operation_status = HAL_OK;
	HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, RESET);

	operation_status = Radio_SPI_Transmit_Message(&command_byte, 1);
	if (operation_status != HAL_OK) goto error;

	operation_status = Radio_SPI_Transmit_Message(argument_bytes, arg_size);
	if (operation_status != HAL_OK) goto error;

	// TODO: Look into moving this into a function? -NJR
	do {
	operation_status

	} while (is_ready = false;)

error:
	HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, SET);
	return operation_status;
}

HAL_StatusTypeDef Si4464_Send_Command_Ignore_Received(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size)
{
	return Si4464_Send_Command(command_byte, argument_bytes, arg_size, NULL, 0);
}

// TODO: Add pin for SDN and HAL variables.
void Si4464_Reset_Device()
{
	// Page 21 of the datasheet implies that the minimum time is 10us? -NJR
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, SET);
	HAL_Delay(1); //  TODO: Fix when we have RTOS set up. -NJR
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, RESET);
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
