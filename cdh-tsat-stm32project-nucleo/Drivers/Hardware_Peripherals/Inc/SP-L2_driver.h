/*
 * FILENAME: SP-L2_driver.h
 *
 * DESCRIPTION: STM32L4 header driver file for the SP-L2 UHF radio transceiver
 *
 * AUTHORS:
 *  - Graham Driver (graham.driver@umsats.ca)
 *
 * CREATED ON: November 13th 2023
 * 
 * Functions left to write (Add or Remove as necessary):
 * S2LP_Spi_Write_Registers
 * S2LP_Spi_Read_Registers
 * S2LP_Reset (Written but untested)
 */

#ifndef HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_
//###############################################################################################
// Include Directives
//###############################################################################################
#include <stdint.h>
#include "stm32l4xx_hal.h"

#define S2LP_SPI	hspi2

#define S2LP_CS_SELECT 0
#define S2LP_CS_RELEASE 1

/* list of the command codes of S2-LP */
#define	COMMAND_TX                                          ((uint8_t)(0x60)) /*!< Start to transmit; valid only from READY */
#define	COMMAND_RX                                          ((uint8_t)(0x61)) /*!< Start to receive; valid only from READY */
#define	COMMAND_READY                                       ((uint8_t)(0x62)) /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
#define	COMMAND_STANDBY                                     ((uint8_t)(0x63)) /*!< Go to STANDBY; valid only from READY */
#define	COMMAND_SLEEP                                       ((uint8_t)(0x64)) /*!< Go to SLEEP; valid only from READY */
#define	COMMAND_LOCKRX                                      ((uint8_t)(0x65)) /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
#define	COMMAND_LOCKTX                                      ((uint8_t)(0x66)) /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
#define	COMMAND_SABORT                                      ((uint8_t)(0x67)) /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
#define	COMMAND_SRES                                        ((uint8_t)(0x70)) /*!< Reset of all digital part, except SPI registers */
#define	COMMAND_FLUSHRXFIFO                                 ((uint8_t)(0x71)) /*!< Clean the RX FIFO; valid from all states */
#define	COMMAND_FLUSHTXFIFO                                 ((uint8_t)(0x72)) /*!< Clean the TX FIFO; valid from all states */

/* interrupt bit masks */
#define IRQ_RX_DATA_READY (1 << 0) /* RX data ready, radio in ready state */
#define IRQ_TX_DATA_SENT  (1 << 2) /* TX data sent, radio in ready state */

//###############################################################################################
// Radio State Codes
//###############################################################################################

typedef enum
{
S2LP_STATE_STANDBY		=	0x02,
S2LP_STATE_SLEEP_A 		=	0x01,
S2LP_STATE_SLEEP_B 		=	0x03,
S2LP_STATE_READY		=	0x00,
S2LP_STATE_LOCKST		=	0x14,
S2LP_STATE_LOCKON		=	0x0C,
S2LP_STATE_RX			=	0x30,
S2LP_STATE_TX			=	0x5C,
S2LP_STATE_SYNTH_SETUP	=	0x50,
S2LP_STATE_WAIT_SLEEP	=	0x7C,
} S2LP_STATE;


//###############################################################################################
// Radio Error Codes
//###############################################################################################
typedef enum
{
	S2LP_HAL_OK                     = HAL_OK,      	//0x00
	S2LP_HAL_ERROR                  = HAL_ERROR,   	//0x01
	S2LP_HAL_BUSY                   = HAL_BUSY,    	//0x02
	S2LP_HAL_TIMEOUT                = HAL_TIMEOUT, 	//0x03
	S2LP_TX_FIFO_FULL,
	S2LP_RESET_FAIL,
} S2LP_StatusTypeDef;

//###############################################################################################
// Function Prototypes
//###############################################################################################


/**
 * @brief Sends message over SPI to the radio transceiver
 * 
 * @param pData A pointer to the data to be sent
 * @param numToSend Number of bytes to be sent
 * @return S2LP_StatusTypeDef 
 * @note The Radio will use SPI 2 on the mcu.
 */
S2LP_StatusTypeDef S2LP_SPI_Transmit_Message(uint8_t * pData, size_t numToSend);


/**
 * @brief Receives message over SPI from the radio
 * 
 * @param pData A pointer to a buffer for the data
 * @param numToReceive Number of bytes to receieve
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_SPI_Receive_Message(uint8_t * pData, size_t numToReceive);


/**
 * @brief Transmit to and Receive message from radio over SPI 
 * 
 * @param pTxData A pointer to the data to be sent
 * @param pRxData A pointer to the data to be received
 * @param numTransmitReceive The number of bytes to transmit/receieve
 * @return S2LP_StatusTypeDef 
 * @note Make sure you intend to transmit and receive the same amount of data
 */
S2LP_StatusTypeDef S2LP_SPI_Transmit_Receive_Message(uint8_t *pTxData, uint8_t *pRxData, size_t numTransmitReceive);


/**
 * @brief Writes to the S2-LP registers.
 * 
 * @param address	The register address
 * @param n_regs 	Number of bytes to write
 * @param buffer 	Pointer to a buffer containing the data
 * @return S2LP_StatusTypeDef 
 * @note Buffer is not a pointer right now as the auto generated config function from STM does not use a pointer
 * we could change it in the future.
 */
S2LP_StatusTypeDef S2LP_Spi_Write_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Reads the S2-LP registers.
 * 
 * @param address 	Address of the register to read
 * @param n_regs 	Number of registers to read
 * @param buffer 	Buffer to hold data from read
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Spi_Read_Registers(uint8_t address, uint8_t n_regs, uint8_t* buffer);


/**
 * @brief Will send the given command to the radio transceiver.
 * 
 * @param commandCode Command for S2LP
 * @return S2LP_StatusTypeDef 
 * 
 * @note This function does not check for compliance with the S2LP
 * current state for the given command.
 */
S2LP_StatusTypeDef S2LP_Send_Command(uint8_t commandCode);


/**
 * @brief This function will read the register that contains a counter for the amount of bytes in TX FIFO.
 * 
 * @param lengthBuffer A pointer to buffer for received byte
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Check_TX_FIFO_Status(uint8_t * lengthBuffer);


/**
 * @brief Fetches a count of RX values waiting in the FIFO
 * 
 * @param lengthBuffer Pointer to buffer for received bytes
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Check_RX_FIFO_Status(uint8_t * lengthBuffer);


/**
 * @brief This function will send data to the S2LP driver if there is space.
 * If there is no space available in the FIFO it will return S2LP_TX_FIFO_FULL.
 * 
 * @param size The size of the packet that is being sent (in bytes)
 * @param buffer Pointer to buffer containing data to be sent
 * @return S2LP_StatusTypeDef 
 * 
 * @note The TX FIFO can only store 128 bytes 
 */
S2LP_StatusTypeDef S2LP_Write_TX_Fifo(uint8_t size, uint8_t* buffer);


/**
 * @brief Reads given amount of bytes from RX FIFO.
 * 
 * @param n_bytes Number of bytes to read
 * @param buffer Pointer to buffer for received byte
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_Read_RX_FIFO(uint8_t n_bytes, uint8_t* buffer);


/**
 * @brief Resets the S2LP via the SDN pin.
 * 
 * @note As far as I understand the shutdown pin must be set high for 
 * at least 1us then set low and it will take up to another 2ms to reset the device.
 * Check page 24 of SP-L2 documentation Describes Reset procedure
 * 
 * @warning DO NOT USE THIS FUNCTION AS THE CURRENT HARDWARE 
 * DESIGN DOES NOT PROPERLY IMPLEMENT IT.
 */
S2LP_StatusTypeDef S2LP_Hardware_Reset();

/**
 * @brief Performs a software reset on the S2LP from documentation: 
 * "Reset the S2-LP state machine and registers values"
 * 
 * @return S2LP_StatusTypeDef 
 * 
 * @note Shouldn't need to check status of uC as it can run at anytime.
 */
S2LP_StatusTypeDef S2LP_Software_Reset();


/**
 * @brief Will either pull down chip select when passed S2LP_CS_SELECT,
 * or released high with S2LP_CS_RELEASE.
 * 
 * @param sel Use S2LP_CS_SELECT and S2LP_CS_RELEASE to select the device or release it
 * @return S2LP_StatusTypeDef 
 */
S2LP_StatusTypeDef S2LP_nCS(uint8_t sel);


/**
 * @brief Will fetch the status bits of the S2LP
 * radio transceiver.
 * 
 * @param returnStatus Buffer for status bits
 * @return S2LP_StatusTypeDef 
 * 
 * @note Note 100% sure this will actually fetch 
 * the status registers needs to be tested.
 */
S2LP_StatusTypeDef S2LP_Get_Status(uint8_t *returnStatus);

/**
 * @brief This function is generated by the S2-LP DK. It contains all the configurations that are loaded into the
 * radio on boot.
 * 
 */
void SpiritBaseConfiguration(void);


#endif /* HARDWARE_PERIPHERALS_INC_SP_L2_DRIVER_H_ */
