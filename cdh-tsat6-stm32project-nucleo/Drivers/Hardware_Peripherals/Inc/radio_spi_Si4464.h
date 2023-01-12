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

/**
 * @brief Transmit Callback Function
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi2);

/**
 * @brief Receive Callback Function
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi2);

/**
 * @brief Transmit/Receive Callback Function
 */
void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi2);

// Transmit and Receive Functions

/**
 * @brief Send data to the Radio.
 * Effectively a wrapper for HAL_SPI_Transmit().
 *
 * @param pData A pointer to the data to send.
 * @param numToSend The number of bytes to send.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Radio_SPI_Transmit_Message(uint8_t * pData, size_t numToSend);

/**
 * @brief Receive up to numToReceive bytes of data from the Radio.
 * Effecvively a wrapper for HAL_SPI_Receive().
 *
 * @param pData A pointer to where to receive data.
 * @param numToReceive The maximum number of bytes to receive.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Radio_SPI_Receive_Message(uint8_t * pData, size_t numToReceive);

/**
 * @brief Transmit/Receive numTransmittedReceived bytes to/from the Radio.
 * Effectively a wrapper for HAL_SPI_TransmitReceive().
 *
 * @param pTxData A pointer to the data to send.
 * @param pRxData A pointer to where to receive data.
 * @prarm numTransmittedReceived The number of bytes to handle.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Radio_SPI_Transmit_Receive_Message(uint8_t * pTxData, uint8_t * pRxData, size_t numTransmittedReceived);

/**
 * @brief Reset the Radio via its SDN Pin.
 *
 * @returns the status code returned by the HAL.
 */
HAL_StatusTypeDef Si4464_Reset_Device();

#endif /* INC_SI446X_RADIO_SPI_SI4464_H_ */
