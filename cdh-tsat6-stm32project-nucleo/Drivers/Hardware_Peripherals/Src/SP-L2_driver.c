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

S2LP_StatusTypeDef S2LP_SPI_Transmit_Message(uint8_t *pData, size_t numToSend){

	return HAL_SPI_Transmit(&hspi2, pData, numToSend, HAL_MAX_DELAY);
}


S2LP_StatusTypeDef S2LP_SPI_Receive_Message(uint8_t *pData, size_t numToReceive){

	return HAL_SPI_Receive(&hspi2, pData, numToReceive, HAL_MAX_DELAY);
}


S2LP_StatusTypeDef S2LP_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive){

	return HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, numTransmitReceive, HAL_MAX_DELAY);
}


S2LP_StatusTypeDef S2LP_Check_TX_FIFO_Status(uint8_t *lengthBuffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// Should we pull down here??? Probably not?
	// We should probably create a typedef of all/most used registers
	status = S2LP_Spi_Read_Registers(0x8D, 1, &lengthBuffer);
  if(status != S2LP_HAL_OK) goto error;


	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Check_RX_FIFO_Status(uint8_t *lengthBuffer){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// Should we pull down here??? Probably not?
	status = S2LP_Spi_Read_Registers(0x90, 2, &lengthBuffer);
	if(status != S2LP_HAL_OK) goto error;


	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Write_TX_Fifo(uint8_t size, uint8_t *buffer){
	// Will not check if FIFO is full yet but feature should be implemented in final release

	S2LP_StatusTypeDef status;
	uint8_t storedBytes = 0;

	// First we need to check how many messages are in the FIFO
	status = S2LP_Check_TX_FIFO_Status(&storedBytes); // Change naming standard away from SPI if not directly an SPI command?
	if(status != S2LP_HAL_OK) goto error;

	// If there is room in FIFO send bytes (FIFO can store 128 bytes)
	// Else return FIFO full status
	if(storedBytes + size >= 128){

		// Pull down CS
		S2LP_nCS(S2LP_CS_SELECT);

		// Send one byte of zeros to indicate we are writing an address 
		status = S2LP_SPI_Transmit_Message(0xFF00, 1);
		if(status != S2LP_HAL_OK) goto error;

		// Send our data to FIFO
		status = S2LP_SPI_Transmit_Message(buffer, size);
		if(status != S2LP_HAL_OK) goto error;

		// Pull up CS to stop communication
		S2LP_nCS(S2LP_CS_SELECT);
	}
	else{
		return S2LP_TX_FIFO_FULL; // FIFO is full cannot send data
	}

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Read_RX_FIFO(uint8_t n_bytes, uint8_t *buffer){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	uint8_t avaliableBytes = 0;
	uint8_t numToFetch = 0;

	// First we need to check how many messages are in the FIFO
	status = S2LP_CHECK_RX_FIFO_STATUS(&avaliableBytes); // Change naming standard away from SPI if not directly an SPI command?
	if(status != S2LP_HAL_OK) goto error;

	// If there are enough bytes ready for requested amount count
	// Else get as many as avaliable
	if(avaliableBytes > n_bytes){
		numToFetch = n_bytes;
	}
	else{
		numToFetch = avaliableBytes;
	}

	// Pull down to select
	status = S2LP_nCS(S2LP_CS_SELECT);
	if(status != S2LP_HAL_OK) goto error;

	status = S2LP_SPI_Transmit_Message(0xFF80, 2);
	if(status != S2LP_HAL_OK) goto error;

	status = S2LP_SPI_Receive_Message(buffer, numToFetch);
	if(status != S2LP_HAL_OK) goto error;

	// Pull up to release
	status = S2LP_nCS(S2LP_CS_RELEASE);
	if(status != S2LP_HAL_OK) goto error;

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Send_Command(uint8_t commandCode){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	
	// Select radio
	S2LP_nCS(S2LP_CS_SELECT);

	// Indicate we are sending a command
	status = S2LP_SPI_Transmit_Message(0x80, 1);
	if(status != S2LP_HAL_OK) goto error;

	// Send command byte
	status = S2LP_SPI_Transmit_Message(commandCode, 1);
	if(status != S2LP_HAL_OK) goto error;

	// Release Radio
	S2LP_nCS(S2LP_CS_RELEASE);

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_nCS(uint8_t sel){

	if(sel){
		HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, GPIO_PIN_RESET);
	}
}


S2LP_StatusTypeDef S2LP_Get_Status(uint8_t *returnStatus){
	
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// Select radio
	S2LP_nCS(S2LP_CS_SELECT);

	// Get two bytes of the status registers
	status = S2LP_SPI_Receive_Message(returnStatus, 2);
	if(status != S2LP_HAL_OK) goto error;

	// Release Radio
	S2LP_nCS(S2LP_CS_RELEASE);

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Software_Reset(){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// Sends the command to reset the board
	status = S2LP_Send_Command(COMMAND_SRES);
	if(status != S2LP_HAL_OK) goto error;

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Hardware_Reset(){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	uint8_t S2LPStatusRegisters[2];
	uint8_t S2LPStatus = 0;

	// Set the SDN pin high
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, GPIO_PIN_SET);

	// Wait 1ms
	HAL_Delay(1);

	// Pull pin low again
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, GPIO_PIN_RESET);

	// Wait another 2ms so S2LP is ready
	HAL_Delay(2);

	// Get status of the radio to check if it has turned on
	status = S2LP_Get_Status(&S2LPstatus);
	if(status != S2LP_HAL_OK) goto error;

	// If the radio is in the ready state the reset is done otherwise there
	// was some sort of error in the reset.
	if((S2LPStatusRegisters[0] & 0x7F) >> 1 != S2LP_STATE_READY){
		status = S2LP_RESET_FAIL;
		goto error;
	}

	error:
		return status;
}
