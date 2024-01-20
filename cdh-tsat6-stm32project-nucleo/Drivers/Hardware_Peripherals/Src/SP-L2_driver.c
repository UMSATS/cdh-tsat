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

#define S2LP_WRITE_TX_FIFO_HEADER_CODE	0xFF00
#define S2LP_READ_RX_FIFO_HEADER_CODE 	0xFF80
#define S2LP_SEND_COMMAND_CODE			0x80


S2LP_StatusTypeDef S2LP_SPI_Transmit_Message(uint8_t *pData, size_t numToSend){

	return HAL_SPI_Transmit(&S2LP_SPI, pData, numToSend, HAL_MAX_DELAY);
}


S2LP_StatusTypeDef S2LP_SPI_Receive_Message(uint8_t *pData, size_t numToReceive){

	return HAL_SPI_Receive(&S2LP_SPI, pData, numToReceive, HAL_MAX_DELAY);
}


S2LP_StatusTypeDef S2LP_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive){

	return HAL_SPI_TransmitReceive(&S2LP_SPI, pTxData, pRxData, numTransmitReceive, HAL_MAX_DELAY);
}

S2LP_StatusTypeDef S2LP_Spi_Write_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	S2LP_nCS(S2LP_CS_SELECT);

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

		S2LP_nCS(S2LP_CS_SELECT);

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

S2LP_StatusTypeDef S2LP_Check_TX_FIFO_Status(uint8_t *lengthBuffer){
	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// Should we pull down here??? Probably not?
	// We should probably create a typedef of all/most used registers
	status = S2LP_Spi_Read_Registers(0x8D, 1, lengthBuffer);
	if(status != S2LP_HAL_OK) goto error;


	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Check_RX_FIFO_Status(uint8_t *lengthBuffer){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;

	// Should we pull down here??? Probably not?
	status = S2LP_Spi_Read_Registers(0x90, 2, lengthBuffer);
	if(status != S2LP_HAL_OK) goto error;


	error:
		return status;
}


S2LP_StatusTypeDef S2LP_Write_TX_Fifo(uint8_t size, uint8_t *buffer){
	// Will not check if FIFO is full yet but feature should be implemented in final release

	S2LP_StatusTypeDef status;

	uint8_t opcode = S2LP_WRITE_TX_FIFO_HEADER_CODE;
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
		status = S2LP_SPI_Transmit_Message(&opcode, 1);
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

	uint8_t opcode = S2LP_READ_RX_FIFO_HEADER_CODE;
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


S2LP_StatusTypeDef S2LP_Send_Command(uint8_t commandCode){

	S2LP_StatusTypeDef status = S2LP_HAL_OK;
	
	uint8_t data[2];
	data[0] = S2LP_SEND_COMMAND_CODE;
	data[1] = commandCode;
	// Select radio
	S2LP_nCS(S2LP_CS_SELECT);

	// Indicate we are sending a command
	status = S2LP_SPI_Transmit_Message(data, 2);
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

	return S2LP_HAL_OK;
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

void SpiritBaseConfiguration(void)
{
  uint8_t tmp[6];

  tmp[0] = 0x52; /* reg. SYNT3 (0x05) */
  tmp[1] = 0x29; /* reg. SYNT2 (0x06) */
  tmp[2] = 0xD8; /* reg. SYNT1 (0x07) */
  tmp[3] = 0x9E; /* reg. SYNT0 (0x08) */
  tmp[4] = 0x29; /* reg. IF_OFFSET_ANA (0x09) */
  tmp[5] = 0xB7; /* reg. IF_OFFSET_DIG (0x0A) */
  S2LP_Spi_Write_Registers(0x05, 6, tmp);
  tmp[0] = 0x05; /* reg. MOD2 (0x10) */
  S2LP_Spi_Write_Registers(0x10, 1, tmp);
  tmp[0] = 0x55; /* reg. ANT_SELECT_CONF (0x1F) */
  tmp[1] = 0x00; /* reg. CLOCKREC2 (0x20) */
  S2LP_Spi_Write_Registers(0x1F, 2, tmp);
  tmp[0] = 0x00; /* reg. PCKTCTRL3 (0x2E) */
  tmp[1] = 0x01; /* reg. PCKTCTRL2 (0x2F) */
  tmp[2] = 0x30; /* reg. PCKTCTRL1 (0x30) */
  S2LP_Spi_Write_Registers(0x2E, 3, tmp);
  tmp[0] = 0x01; /* reg. PROTOCOL1 (0x3A) */
  S2LP_Spi_Write_Registers(0x3A, 1, tmp);
  tmp[0] = 0x41; /* reg. PCKT_FLT_OPTIONS (0x40) */
  S2LP_Spi_Write_Registers(0x40, 1, tmp);
  tmp[0] = 0x4C; /* reg. CSMA_CONFIG3 (0x4C) */
  S2LP_Spi_Write_Registers(0x4C, 1, tmp);
  tmp[0] = 0x14; /* reg. PA_POWER8 (0x5A) */
  S2LP_Spi_Write_Registers(0x5A, 1, tmp);
  tmp[0] = 0x07; /* reg. PA_POWER0 (0x62) */
  tmp[1] = 0x01; /* reg. PA_CONFIG1 (0x63) */
  tmp[2] = 0x88; /* reg. PA_CONFIG0 (0x64) */
  S2LP_Spi_Write_Registers(0x62, 3, tmp);
}

