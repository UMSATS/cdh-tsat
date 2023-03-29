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

typedef struct {
    uint8_t chip_rev;
    uint16_t part_number;
    uint8_t part_build;
    uint16_t id;
    uint8_t customer;
    uint8_t rom_id;
} Si4464PartInfo;

typedef struct {
    uint8_t ext_revision;
    uint8_t branch_revision;
    uint8_t internal_revision;
    uint16_t patch_id;
    uint8_t functional_mode;
} Si4464FunctionInfo;

typedef enum {
	SI4464_STATE_NOCHANGE = 0x00,
	SI4464_STATE_SLEEP = 0x01,
	SI4464_STATE_SPI_ACTIVE = 0x02,
	SI4464_STATE_READY = 0x03,
	SI4464_STATE_TX_TUNE = 0x05,
	SI4464_STATE_RX_TUNE = 0x06,
	SI4464_STATE_TX = 0x07,
	SI4464_STATE_RX = 0x08,
} Si4464PowerState;


/*
 * TODO: Add detail to function description
 * Writes bytes sent to it to the TX FIFO
 */
HAL_StatusTypeDef writeTxBuffer(uint8_t lengthTxData, uint8_t *txData);

/*
 * TODO: Add detail to function description
 * Will send a message using the radio
 */
HAL_StatusTypeDef Si4464_Transmit_Message(uint8_t lengthTxData, uint8_t *txData);

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
 * @brief Get the "Function Revision Information" of the Si4464.
 * 
 * @param info_data A pointer to the data struct to return data with.
 * @returns HAL_OK if command succeeded, otherwise the respective error.
 */
HAL_StatusTypeDef Si4464_Get_Function_Info(Si4464FunctionInfo *info_data);

/**
 * @brief Get the Part Information of the Si4464.
 * 
 * @param info_data A pointer to the data struct to return data with.
 * @returns HAL_OK if command succeeded, otherwise the respective error.
 */
HAL_StatusTypeDef Si4464_Get_Part_Info(Si4464PartInfo *info_data);

/**
 * @brief Get some number of properties in a given group.
 *
 * @param group The Property Group to access.
 * @param num_props The number of properties to get.
 * @param start_prop The property index to start on.
 * @param returned_bytes An array to store returned fields.
 *
 * NOTE: Expects that returned_bytes has at least num_props bytes of space!
 *
 * @returns HAL_OK if all commands succeeded, otherwise the error returned by the HAL or other called functions.
 */
HAL_StatusTypeDef Si4464_Get_Prop(uint8_t group, uint8_t num_props, uint8_t start_prop, uint8_t *returned_bytes);

/**
 * @brief Set some number of properties in a given group.
 *
 * @param group The Property Group to access.
 * @param num_props The number of properties to set.
 * @param start_prop The property index to start on.
 * @param bytes_to_send An array of data to send.
 *
 * NOTE: Expects that returned_bytes has at least num_props bytes available to send!
 *
 * NOTE: Expects that num_props is less than or equal to 12 (the maximum number of props modifiable) and greater than 0.
 *
 * @returns HAL_OK if all commands succeeded, otherwise the error returned by the HAL or other called functions.
 */
HAL_StatusTypeDef Si4464_Set_Props(uint8_t group, uint8_t num_props, uint8_t start_prop, uint8_t *bytes_to_send);

/**
 * @brief A shortcut for setting only one property.
 *
 * @param group The Property Group to access.
 * @param start_prop The property index to start on.
 * @param byte_to_send The data to send.
 *
 * @returns HAL_OK if all commands succeeded, otherwise the error returned by the HAL or other called functions.
 */
HAL_StatusTypeDef Si4464_Set_One_Prop(uint8_t group, uint8_t start_prop, uint8_t byte_to_send);

/**
 * @brief Set the chip select pin of the Si4464
 *
 * @param sel 0 for pull low and 1 for pull high
 */
void Si4464_Nsel(uint8_t sel);

#endif /* INC_SI446X_RADIO_SPI_SI4464_H_ */

