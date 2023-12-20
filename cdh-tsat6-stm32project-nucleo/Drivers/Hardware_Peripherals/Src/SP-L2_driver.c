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

SPL2_StatusTypeDef SPL2_Check_TX_FIFO_Status(uint8_t * lengthBuffer){
	SPL2_StatusTypeDef status = SPL2_HAL_OK;

	// Should we pull down here??? Probably not?
	// We should probably create a typedef of all/most used registers
	status = S2LP_Spi_Read_Registers(0x8D, 1, &lengthBuffer);
	if(status != SPL2_HAL_OK) goto error;


	error:
		return status;
}


SPL2_StatusTypeDef S2LP_Write_TX_Fifo(uint8_t size, uint8_t* buffer){
	// Will not check if FIFO is full yet but feature should be implemented in final release

	SPL2_StatusTypeDef status;
	uint8_t storedBytes = 0;

	// First we need to check how many messages are in the FIFO
	status = SPL2_Check_TX_FIFO_Status(&storedBytes); // Change naming standard away from SPI if not directly an SPI command?
	if(status != SPL2_HAL_OK) goto error;

	// If there is room in FIFO send bytes (FIFO can store 128 bytes)
	// Else return FIFO full status
	if(storedBytes + size >= 128){

		// Pull down CS
		S2LP_nCS(S2LP_CS_SELECT);

		// Send one byte of zeros to indicate we are writing an address 
		status = SPL2_Spi_Send_Message(0xFF00, 1);
		if(status != SPL2_HAL_OK) goto error;

		// Send our data to FIFO
		status = SPL2_Spi_Send_Message(data, size);
		if(status != SPL2_HAL_OK) goto error;

		// Pull up CS to stop communication
		S2LP_nCS(S2LP_CS_SELECT);
	}
	else{
		return SPL2_TX_FIFO_FULL; // FIFO is full cannot send data
	}

	error:
		return status;
}


SPL2_StatusTypeDef S2LP_nCS(uint8_t sel){

	if(sel){
		HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(UHF_nCS_GPIO_Port, UHF_nCS_Pin, GPIO_PIN_RESET);
	}

}