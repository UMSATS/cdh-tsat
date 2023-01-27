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
#include "Si4464_command_codes.h"
#include "Si4464_driver.h"

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

bool Si4464_Get_CTS() {
	uint8_t command = SI4464_READ_COMMAND_BUFFER;
	uint8_t response = 0x00;

	// HACK ALERT!!!!!
	// This does not use the HAL_StatusTypeDef for returning! maybe figure out a returnable struct for error handling? -NJR
	Radio_SPI_Transmit_Message(&command, 1);
	Radio_SPI_Receive_Message(&response, 1);

	return (response == 0xFF);
}

HAL_StatusTypeDef Si4464_Execute_Command_Stream(uint8_t command_stream[], size_t stream_len)
{
	size_t i = 0;

	uint8_t command;
	uint8_t *args;
	size_t arg_len;

	HAL_StatusTypeDef command_success = HAL_OK;

	while (i < stream_len && command_stream[i] != 0x00 && command_success == HAL_OK) {
		arg_len = command_stream[i] - 1;
		command = command_stream[i + 1];
		args = command_stream + (i + 2);
		
		command_success = Si4464_Send_Command_Ignore_Received(command, args, arg_len);
	}

	return command_success;
}

HAL_StatusTypeDef Si4464_Send_Command(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size, uint8_t *returned_bytes, size_t return_size)
{
	// HACK ALERT!!!!!!!!!!!!!!
	// TODO: DO NULL CHECKING HERE!
	// OUR CURRENT ERROR TYPEDEFS DO NOT ALLOW MORE THAN ONE ERROR CODE!
	// Maybe create a new error enum that builds off HAL_StatusTypeDef? -NJR
	HAL_StatusTypeDef operation_status = HAL_OK;
	Si4464_Nsel(0);

	operation_status = Radio_SPI_Transmit_Message(&command_byte, 1);
	if (operation_status != HAL_OK) goto error;

	operation_status = Radio_SPI_Transmit_Message(argument_bytes, arg_size);
	if (operation_status != HAL_OK) goto error;

	// TODO: Get a timeout here? -NJR
	while (!Si4464_Get_CTS()) {
		// Nothing.
	}

	operation_status = Radio_SPI_Receive_Message(returned_bytes, return_size);
	if (operation_status != HAL_OK) goto error;

error:
	Si4464_Nsel(1);
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
	Si4464_Nsel(1);
	HAL_Delay(1); //  TODO: Fix when we have RTOS set up. -NJR
	Si4464_Nsel(0);
}

void Si4464_Nsel(uint8_t sel){
	if(sel){
		HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, SET);
	}
	else{
		HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, RESET);
	}
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
