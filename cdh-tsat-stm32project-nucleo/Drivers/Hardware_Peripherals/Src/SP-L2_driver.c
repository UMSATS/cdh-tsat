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
#include "main.h"

extern SPI_HandleTypeDef S2LP_SPI;

// Directives



S2LP_StatusTypeDef S2LP_SPI_Transmit_Message(uint8_t* pData, size_t numToSend){

	return HAL_SPI_Transmit(&S2LP_SPI, pData, numToSend, 100);
}


S2LP_StatusTypeDef S2LP_SPI_Receive_Message(uint8_t *pData, size_t numToReceive){

	return HAL_SPI_Receive(&S2LP_SPI, pData, numToReceive, 100);
}


S2LP_StatusTypeDef S2LP_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive){

	return HAL_SPI_TransmitReceive(&S2LP_SPI, pTxData, pRxData, numTransmitReceive, HAL_MAX_DELAY);
}


S2LP_StatusTypeDef S2LP_Spi_Write_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	uint8_t header = 0x0;

	S2LP_nCS(S2LP_CS_SELECT);

		// Sender Header Byte for a write
		status = S2LP_SPI_Transmit_Message(&header, 1);
		if(status != S2LP_HAL_OK) goto error;

		// Indicate we are sending a command
		status = S2LP_SPI_Transmit_Message(&address, 1);
		if(status != S2LP_HAL_OK) goto error;

		// Indicate we are sending a command
		status = S2LP_SPI_Transmit_Message(buffer, n_regs);
		if(status != S2LP_HAL_OK) goto error;

		// Release Radio
	S2LP_nCS(S2LP_CS_RELEASE);

	error:
			return status;
}


S2LP_StatusTypeDef S2LP_Spi_Read_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	uint8_t header = 0x1;

		S2LP_nCS(S2LP_CS_SELECT);

			status = S2LP_SPI_Transmit_Message(&header, 1);
			if(status != S2LP_HAL_OK) goto error;

			// Indicate we are sending a command
			status = S2LP_SPI_Transmit_Message(&address, 1);
			if(status != S2LP_HAL_OK) goto error;

			// Indicate we are sending a command
			status = S2LP_SPI_Receive_Message(buffer, n_regs);
			if(status != S2LP_HAL_OK) goto error;

			// Release Radio
		S2LP_nCS(S2LP_CS_RELEASE);

		error:
				return status;

}


S2LP_StatusTypeDef S2LP_Send_Command(uint8_t commandCode){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	
	uint8_t data[2];
	data[0] = 0x80;
	data[1] = commandCode;
	// Select radio
	S2LP_nCS(S2LP_CS_SELECT);

	// Indicate we are sending a command
	status = S2LP_SPI_Transmit_Message(&data, 2);
	if(status != S2LP_HAL_OK) goto error;

	// Release Radio
	S2LP_nCS(S2LP_CS_RELEASE);

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Check_TX_FIFO_Status(uint8_t *lengthBuffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// We should probably create a typedef or define all/most used registers
	uint8_t txFIFOStatusRegister = 0x8F;

	// Should we pull down here??? Probably not?
	status = S2LP_Spi_Read_Registers(txFIFOStatusRegister, 1, lengthBuffer);
	if(status != S2LP_HAL_OK) goto error;


	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Check_RX_FIFO_Status(uint8_t *lengthBuffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// We should probably create a typedef or define all/most used registers
	uint8_t rxFIFOStatusRegister = 0x90;

	// Should we pull down here??? Probably not?
	status = S2LP_Spi_Read_Registers(rxFIFOStatusRegister, 1, lengthBuffer);
	if(status != S2LP_HAL_OK) goto error;


	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Write_TX_Fifo(uint8_t size, uint8_t *buffer){
	// Will not check if FIFO is full yet but feature should be implemented in final release
	S2LP_StatusTypeDef status;

	uint8_t opcode[2] = {0x00, 0xFF}; // First byte indicates write, second indicates to FIFO

	uint8_t storedBytes = 0;

	// First we need to check how many messages are in the FIFO
	status = S2LP_Check_TX_FIFO_Status(&storedBytes);
	if(status != S2LP_HAL_OK) goto error;

	// If there is room in FIFO send bytes (FIFO can store 128 bytes)
	// Else return FIFO full status
	if(storedBytes + size <= 128){

		// Pull down CS
		S2LP_nCS(S2LP_CS_SELECT);

		// Send one byte of zeros to indicate we are writing an address 
		status = S2LP_SPI_Transmit_Message(&opcode, 2);
		if(status != S2LP_HAL_OK) goto error;

		// Send our data to FIFO
		status = S2LP_SPI_Transmit_Message(buffer, size);
		if(status != S2LP_HAL_OK) goto error;

		// Pull up CS to stop communication
		S2LP_nCS(S2LP_CS_RELEASE);
	}
	else{
		return S2LP_TX_FIFO_FULL; // FIFO is full cannot send data
	}

	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Read_RX_FIFO(uint8_t n_bytes, uint8_t *buffer){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	uint8_t opcode[2] = {0x01, 0xFF};	// First byte indicates read, second indicates FIFO
	uint8_t avaliableBytes = 0;
	uint8_t numToFetch = 0;

	// First we need to check how many messages are in the FIFO
	status = S2LP_Check_RX_FIFO_Status(&avaliableBytes);
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

	status = S2LP_SPI_Transmit_Message(&opcode, 2);
	if(status != S2LP_HAL_OK) goto error;

	status = S2LP_SPI_Receive_Message(buffer, numToFetch);
	if(status != S2LP_HAL_OK) goto error;

	// Pull up to release
	status = S2LP_nCS(S2LP_CS_RELEASE);
	if(status != S2LP_HAL_OK) goto error;

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

	return S2LP_HAL_OK;
}


S2LP_StatusTypeDef S2LP_Get_Status(uint8_t* returnStatus){
	
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

	// Set the SDN pin high
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, GPIO_PIN_SET);

	// Wait 1ms
	HAL_Delay(1); // This may need to be changed to comply with RTOS

	// Pull pin low again
	HAL_GPIO_WritePin(UHF_SDN_GPIO_Port, UHF_SDN_Pin, GPIO_PIN_RESET);

	// Wait another 2ms so S2LP is ready
	HAL_Delay(2); // This may need to be changed to comply with RTOS

	// Get status of the radio to check if it has turned on
	status = S2LP_Get_Status(&S2LPStatusRegisters);
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


void SpiritBaseConfiguration(void)
{
  uint8_t tmp[6];

  tmp[0] = 0x72; /* reg. SYNT3 (0x05) */
  tmp[1] = 0x2A; /* reg. SYNT2 (0x06) */
  tmp[2] = 0x3D; /* reg. SYNT1 (0x07) */
  tmp[3] = 0x71; /* reg. SYNT0 (0x08) */
  tmp[4] = 0x2F; /* reg. IF_OFFSET_ANA (0x09) */
  tmp[5] = 0xC2; /* reg. IF_OFFSET_DIG (0x0A) */
  S2LP_Spi_Write_Registers(0x05, 6, &tmp);
  tmp[0] = 0x92; /* reg. MOD4 (0x0E) */
  tmp[1] = 0xA7; /* reg. MOD3 (0x0F) */
  tmp[2] = 0x27; /* reg. MOD2 (0x10) */
  S2LP_Spi_Write_Registers(0x0E, 3, &tmp);
  tmp[0] = 0xA3; /* reg. MOD0 (0x12) */
  tmp[1] = 0x13; /* reg. CHFLT (0x13) */
  S2LP_Spi_Write_Registers(0x12, 2, &tmp);
  tmp[0] = 0x55; /* reg. ANT_SELECT_CONF (0x1F) */
  S2LP_Spi_Write_Registers(0x1F, 1, &tmp);
  tmp[0] = 0x00; /* reg. PCKTCTRL3 (0x2E) */
  tmp[1] = 0x01; /* reg. PCKTCTRL2 (0x2F) */
  tmp[2] = 0x30; /* reg. PCKTCTRL1 (0x30) */
  S2LP_Spi_Write_Registers(0x2E, 3, &tmp);
  tmp[0] = 0x01; /* reg. PROTOCOL1 (0x3A) */
  S2LP_Spi_Write_Registers(0x3A, 1, &tmp);
  tmp[0] = 0x40; /* reg. FIFO_CONFIG3 (0x3C) */
  tmp[1] = 0x40; /* reg. FIFO_CONFIG2 (0x3D) */
  tmp[2] = 0x40; /* reg. FIFO_CONFIG1 (0x3E) */
  tmp[3] = 0x40; /* reg. FIFO_CONFIG0 (0x3F) */
  tmp[4] = 0x41; /* reg. PCKT_FLT_OPTIONS (0x40) */
  S2LP_Spi_Write_Registers(0x3C, 5, &tmp);
  tmp[0] = 0x16; /* reg. PA_POWER8 (0x5A) */
  S2LP_Spi_Write_Registers(0x5A, 1, &tmp);
  tmp[0] = 0x07; /* reg. PA_POWER0 (0x62) */
  tmp[1] = 0x01; /* reg. PA_CONFIG1 (0x63) */
  S2LP_Spi_Write_Registers(0x62, 2, &tmp);
}

