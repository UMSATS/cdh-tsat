/*
 * radio_spi_Si4464.c
 *
 *  Created on: Jan. 3, 2023
 *      Authors:
 *      Graham Driver
 *      Nikolaus J. Reichert <nikolaus.reichert@umsats.ca>
 */
#include <stdio.h>
#include <string.h>

#include "Si4464_driver.h"
#include "Si4464_command_codes.h"
#include "Si4464_driver.h"
#include "Si4464_driver_config.h"

static uint8_t POWER_UP_ARRAY[] = RADIO_CONFIGURATION_DATA_ARRAY;


/************************************************************
 *
 * Si4464 High Level Commands
 *
 ************************************************************/

Si4464_StatusTypeDef Si4464_Get_Part_Info(Si4464PartInfo *info_data)
{
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	if (!info_data) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	// Otherwise...
	
	uint8_t reply_data[8] = {0x00};

	operation_status = Si4464_Send_Command(SI4464_PART_INFO, NULL, 0, reply_data, sizeof(reply_data));
	if (operation_status != SI4464_HAL_OK) goto error;

	// Initialize the struct.
	info_data->chip_rev = reply_data[0];
	info_data->part_number = (((uint16_t) reply_data[1]) << 8) | (uint16_t) reply_data[2];
	info_data->part_build = reply_data[3];
	info_data->id = (((uint16_t) reply_data[4]) << 8) | (uint16_t) reply_data[5];
	info_data->customer = reply_data[6];
	info_data->rom_id = reply_data[7];

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Get_Function_Info(Si4464FunctionInfo *info_data)
{
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	if (!info_data) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	// Otherwise...
	
	uint8_t reply_data[6] = {0x00};

	operation_status = Si4464_Send_Command(SI4464_FUNC_INFO, NULL, 0, reply_data, sizeof(reply_data));
	if (operation_status != SI4464_HAL_OK) goto error;

	// Initialize the struct.
	info_data->ext_revision = reply_data[0];
	info_data->branch_revision = reply_data[1];
	info_data->internal_revision = reply_data[2];
	info_data->patch_id = (((uint16_t) reply_data[3]) << 8) | (uint16_t) reply_data[4];
	info_data->functional_mode = reply_data[5];

error:
	return operation_status;
}


/************************************************************
 *
 * Basic Si4464 Commands & Utilities
 *
 ************************************************************/
Si4464_StatusTypeDef Si4464_Send_Command(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size, uint8_t *returned_bytes, size_t return_size)
{
	// HACK ALERT!!!!!!!!!!!!!!
	// TODO: DO NULL CHECKING HERE!
	// OUR CURRENT ERROR TYPEDEFS DO NOT ALLOW MORE THAN ONE ERROR CODE!
	// Maybe create a new error enum that builds off Si4464_StatusTypeDef? -NJR
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	Si4464_Nsel(0);

	operation_status = Radio_SPI_Transmit_Message(&command_byte, 1);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (arg_size != 0) {
		operation_status = Radio_SPI_Transmit_Message(argument_bytes, arg_size);
		if (operation_status != SI4464_HAL_OK) goto error;
	}

	// TODO: Get a timeout here? -NJR
	// TODO: This may be unnecessary, but what we are trying to do is only bring Si4464_Nsel high if CTS Fails. Leave Nsel low if not. -NJR
	uint32_t cts = 0;
	do {
		Si4464_Nsel(1);
		Si4464_Nsel(0);
		cts = Si4464_Get_CTS();
	} while (!cts);
	
	if (return_size != 0) {
		operation_status = Radio_SPI_Receive_Message(returned_bytes, return_size);
		if (operation_status != SI4464_HAL_OK) goto error;
	}

error:
	Si4464_Nsel(1);
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Init_Device() {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	Si4464_Reset_Device();

	operation_status = Si4464_Execute_Command_Stream(POWER_UP_ARRAY, sizeof(POWER_UP_ARRAY));
	if (operation_status != SI4464_HAL_OK) goto error;

error:
	return operation_status;
}


Si4464_StatusTypeDef Si4464_Send_Command_Ignore_Received(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size)
{
	return Si4464_Send_Command(command_byte, argument_bytes, arg_size, NULL, 0);
}


Si4464_StatusTypeDef Si4464_Execute_Command_Stream(uint8_t command_stream[], size_t stream_len)
{
	size_t i = 0;

	uint8_t command;
	uint8_t *args;
	size_t arg_len;

	Si4464_StatusTypeDef command_success = SI4464_HAL_OK;

	while (i < stream_len && command_stream[i] != 0x00 && command_success == SI4464_HAL_OK) {
		arg_len = command_stream[i] - 1;
		command = command_stream[i + 1];
		args = command_stream + (i + 2);

		command_success = Si4464_Send_Command_Ignore_Received(command, args, arg_len);

		// we just transmitted command_stream[i] bytes so
		// skip that many bytes plus the size byte. -NJR
		i = i + command_stream[i] + 1;
	}

	return command_success;
}


// TODO: Add pin for SDN and HAL variables.
void Si4464_Reset_Device()
{
	// Page 21 of the datasheet implies that the minimum time is 10us? -NJR
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, GPIO_PIN_SET);
	HAL_Delay(1); //  TODO: Fix when we have RTOS set up. -NJR
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}


bool Si4464_Get_CTS() {
	uint8_t command[] = {SI4464_READ_COMMAND_BUFFER, 0x00};
	uint8_t response[] = {0x00, 0x00};

	// HACK ALERT!!!!!
	// This does not use the Si4464_StatusTypeDef for returning! maybe figure out a returnable struct for error handling? -NJR
	Radio_SPI_Transmit_Receive_Message(command, response, 2);

	return (response[1] == 0xFF);
}


void Si4464_Nsel(uint8_t sel){
	if(sel){
		HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, SET);
	}
	else{
		HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, RESET);
	}
}


Si4464_StatusTypeDef writeTxBuffer(uint8_t lengthTxData, uint8_t *txData){
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	Si4464_Nsel(0);
	while(!Si4464_Get_CTS()){
		// Wait
	}
	operation_status = Si4464_Send_Command(SI4464_TX_FIFO_WRITE, txData, lengthTxData, NULL, 0); // Add bytes to the transmit buffer
	Si4464_Nsel(1);
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Get_Prop(uint8_t group, uint8_t num_props, uint8_t start_prop, uint8_t *returned_bytes){
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	if (!returned_bytes) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	uint8_t args[] = {group, num_props, start_prop};

	operation_status = Si4464_Send_Command(SI4464_GET_PROPERTY, args, 3, returned_bytes, num_props);
	if (operation_status != SI4464_HAL_OK) goto error;

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Set_Props(uint8_t group, uint8_t num_props, uint8_t start_prop, uint8_t *bytes_to_send){
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	if (!bytes_to_send) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	if (num_props > 12 || num_props < 1) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	uint8_t args[15] = {0};
	args[0] = group;
	args[1] = num_props;
	args[2] = start_prop;

	memcpy(args + 3, bytes_to_send, num_props); // TODO: TRIPLE CHECK THIS! -NJR

	operation_status = Si4464_Send_Command_Ignore_Received(SI4464_SET_PROPERTY, args, 3 + num_props);
	if (operation_status != SI4464_HAL_OK) goto error;

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Set_One_Prop(uint8_t group, uint8_t start_prop, uint8_t byte_to_send){
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	// TODO: I don't feel super comfortable about passing the address of an argument here. What
	// if it's an argument passed by register? Maybe read the ARM EABI Reference to double check. -NJR

	operation_status = Si4464_Set_Props(group, 1, start_prop, &byte_to_send);
	if (operation_status != SI4464_HAL_OK) goto error;

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Set_Power_State(Si4464PowerState state) {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	uint8_t int_state = (uint8_t) state & 0x0F;

	operation_status = Si4464_Send_Command_Ignore_Received(SI4464_CHANGE_STATE, &int_state, 1);
	if (operation_status != SI4464_HAL_OK) goto error;

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Write_TX_FIFO(uint8_t src[], size_t num_bytes_to_send, size_t *num_bytes_sent) {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	size_t num_space_available = 0;
	size_t num_can_send = 0;
	size_t free_space_after_write = 0;

	// TODO: Check for reading more than max buffer size.
	if (!src || !num_bytes_sent || num_bytes_to_send == 0) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	operation_status = Si4464_Get_TX_FIFO_Free_Space(&num_space_available);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (num_bytes_to_send > num_space_available) {
		num_can_send = num_space_available;
	} else {
		num_can_send = num_bytes_to_send;
	}
	// TODO: Do error checking on length of buffer.

	operation_status = Si4464_Send_Command_Ignore_Received(SI4464_TX_FIFO_WRITE, src, num_can_send);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Si4464_Get_TX_FIFO_Free_Space(&free_space_after_write);
	if (operation_status != SI4464_HAL_OK) goto error;

	// TODO: This assumes the write is practically atomic (No TX is happening during write. -NJR
	*num_bytes_sent = num_space_available - free_space_after_write;

	// TODO: "This command does not cause CTS to go low, and can be sent while CTS is low. This command
	// has no response to be read and thus there is no need to monitor CTS after sending this command."
	//
	// Does this mean that we **shouldn't** CTS? -NJR

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Get_TX_FIFO_Free_Space(size_t *returned_size) {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	uint8_t fifo_info[2] = {0};
	uint8_t fifo_info_args = {0};

	if (!returned_size) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	operation_status = Si4464_Send_Command(SI4464_FIFO_INFO, &fifo_info_args, 1, fifo_info, 2);
	if (operation_status != SI4464_HAL_OK) goto error;

	*returned_size = (size_t) fifo_info[1];

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Get_RX_FIFO_Len(size_t *returned_size) {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	uint8_t fifo_info[2] = {0};
	uint8_t fifo_info_args = {0};

	if (!returned_size) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	operation_status = Si4464_Send_Command(SI4464_FIFO_INFO, &fifo_info_args, 1, fifo_info, 2);
	if (operation_status != SI4464_HAL_OK) goto error;

	*returned_size = (size_t) fifo_info[0];

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Read_RX_FIFO(uint8_t dest[], size_t max_buffer_size, size_t *num_bytes_returned) {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	uint8_t command = SI4464_RX_FIFO_READ;

	size_t num_to_get = 0;
	size_t num_available = 0;

	// TODO: Check for reading more than max buffer size.

	if (!dest || !num_bytes_returned || max_buffer_size == 0) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	operation_status = Si4464_Get_RX_FIFO_Len(&num_available);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (max_buffer_size > num_available) {
		num_to_get = num_available;
	} else {
		num_to_get = max_buffer_size;
	}

	// Looks like we shouldn't CTS here.
	//
	// > This command is used to read data byte(s) from the RX FIFO. The READ_RX_FIFO
	// > command should be clocked in on SDI and the reply should be clocked out on
	// > SDO without deasserting NSEL. If you read more data bytes than the RX FIFO
	// > contains it will generate a FIFO Underflow interrupt event.
	operation_status = Radio_SPI_Transmit_Message(&command, 1);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Radio_SPI_Receive_Message(dest, num_to_get);
	if (operation_status != SI4464_HAL_OK) goto error;

	*num_bytes_returned = num_to_get;

error:
	return operation_status;
}

Si4464_StatusTypeDef Si4464_Get_Channel(uint8_t *out_channel) {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	uint8_t returned_data[] = {0};

	if (!out_channel) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

	operation_status = Si4464_Send_Command(SI4464_REQUEST_DEVICE_STATE, NULL, 0, returned_data, sizeof(returned_data));
	if (operation_status != SI4464_HAL_OK) goto error;

	*out_channel = returned_data[1];

error:
	return operation_status;
}

//Si4464_StatusTypeDef Si4464_Set_Channel(uint8_t channel) {
//
//}

// TODO: Have a parameter for selecting a channel.
Si4464_StatusTypeDef Si4464_Transmit(Si4464PowerState state_after_tx, size_t len) {
	Si4464_StatusTypeDef operation_status = HAL_OK;
	uint8_t args[4] = {0};

	args[0] = 0; // Channel
	args[1] = (state_after_tx << 4) | (SI4464_NO_RETRANSMIT << 2) | (SI4464_TRANSMIT_NOW << 0);
	args[2] = (uint8_t) ((len & 0x1F00) >> 8);
	args[3] = (uint8_t) (len & 0x00FF);

	operation_status = Si4464_Send_Command_Ignore_Received(SI4464_START_TX, args, 4);
	if (operation_status != SI4464_HAL_OK) goto error;

error:
	return operation_status;
}


/***********************************************************
 *
 * Low Level HAL Wrappers
 *
 ***********************************************************/
Si4464_StatusTypeDef Radio_SPI_Transmit_Message(uint8_t * pData, size_t numToSend){
	// TODO: Check which form of SPI Transfer function we should use: Blocking, IT, or DMA.
	// For now, Daigh noted we should use Blocking. -NJR
	// HAL_SPI_Transmit_IT(&hspi2, pData, sizeof(pData));
	return HAL_SPI_Transmit(&hspi2, pData, numToSend, HAL_MAX_DELAY);
}


Si4464_StatusTypeDef Radio_SPI_Receive_Message(uint8_t * pData, size_t numToReceive){
	// See note on Radio_SPI_Transmit_Message(). -NJR
	// HAL_SPI_Receive_IT(&hspi2, pData, sizeof(pData));
	return HAL_SPI_Receive(&hspi2, pData, numToReceive, HAL_MAX_DELAY);
}


Si4464_StatusTypeDef Radio_SPI_Transmit_Receive_Message(uint8_t * pTxData, uint8_t * pRxData, size_t numTransmittedReceived){

	// HAL_SPI_TransmitReceive_IT(&hspi2, pTxData, pRxData, sizeof(pTxData));
	return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, numTransmittedReceived, HAL_MAX_DELAY);
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
