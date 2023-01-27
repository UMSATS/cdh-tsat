/*
 * radio_spi_Si4464.h
 *
 *  Created on: Jan. 3, 2023
 *      Authors:
 *      Graham Driver
 *      Nikolaus J. Reichert <nikolaus.reichert@umsats.ca>
 */

#ifndef HARDWARE_PERIPHERALS_INC_SI4464_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_SI4464_DRIVER_H_

#include <stdbool.h>

#include "stm32l4xx_hal.h"
#include "main.h"

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
 */
void Si4464_Reset_Device();

/**
 * @brief Send a stream of commands, of the following format:
 * 
 * {NUM_BYTES_0, COMMAND_0, DATA_0, ..., DATA_N, NUM_BYTES_1, COMMAND_1, DATA_0, ...}
 * 
 * @param command_stream The stream in question.
 * @param stream_len The length of the stream.
 * @returns HAL_OK if all commands succeed, otherwise the error code of the failing command.
 */
HAL_StatusTypeDef Si4464_Execute_Command_Stream(uint8_t command_stream[], size_t stream_len);

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

/**
 * @brief Check if the Radio Module is ready to return requested data.
 *
 * @returns true if it is ready (received byte was 0xFF)
 * @returns false if not (received byte most likely will be 0x00).
 */
bool Si4464_Get_CTS();

/**
 * @brief Set the chip select pin of the Si4464
 *
 * @param sel 0 for pull low and 1 for pull high
 */
void Si4464_Nsel(uint8_t sel);

#endif /* INC_SI446X_RADIO_SPI_SI4464_H_ */

