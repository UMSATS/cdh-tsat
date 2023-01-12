/*
 * radio_spi_Si4464.h
 *
 *  Created on: Jan. 3, 2023
 *      Author: Graham Driver
 */

#ifndef HARDWARE_PERIPHERALS_INC_SI4464_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_SI4464_DRIVER_H_

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

/**
 * @brief Send a command to the Radio module.
 *
 * @param command_byte The command to send.
 * @param argument_bytes Arguments associated with the command.
 * @param arg_size The number of argument bytes.
 * @param returned_bytes A pointer to an empty array to place returned data in.
 * @param return_size The number of bytes expected to be returned
 *
 * @returns The status code returned by the HAL.
 *
 * Note: if return_size is more than what the API Description states, the function will return garbage values for those bytes.
 */
HAL_StatusTypeDef Si4464_Send_Command(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size, uint8_t *returned_bytes, size_t return_size);

/**
 * @brief Wrapper for Si4464_Send_Command(), throwing out any ignored data.
 */
HAL_StatusTypeDef Si4464_Send_Command_Ignore_Received(uint8_t command_byte, uint8_t *argument_bytes, size_t arg_size);



#endif /* INC_SI446X_RADIO_SPI_SI4464_H_ */

