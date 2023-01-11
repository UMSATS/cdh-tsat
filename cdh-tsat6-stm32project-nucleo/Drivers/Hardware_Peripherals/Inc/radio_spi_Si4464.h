/*
 * radio_spi_Si4464.h
 *
 *  Created on: Jan. 3, 2023
 *      Author: Graham Driver
 */

#ifndef INC_SI446X_RADIO_SPI_SI4464_H_
#define INC_SI446X_RADIO_SPI_SI4464_H_

#include "stm32l4xx_hal.h"

// The SPI bus used for the radio module
extern SPI_HandleTypeDef hspi2;


/*
 * Using the SPI in an interrupt mode means we must setup callback functions
 * for the TX and RX. The functions below are the Transmit, Receive and Transmit Receive Respectively
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi2);

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi2);

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi2);

// Transmit and Receive Functions
HAL_StatusTypeDef Radio_SPI_Transmit_Message(uint8_t * pData, size_t numToSend);

HAL_StatusTypeDef Radio_SPI_Receive_Message(uint8_t * pData, size_t numToReceive);

HAL_StatusTypeDef Radio_SPI_Transmit_Receive_Message(uint8_t * pTxData, uint8_t * pRxData, size_t numTransmittedReceived);

#endif /* INC_SI446X_RADIO_SPI_SI4464_H_ */
