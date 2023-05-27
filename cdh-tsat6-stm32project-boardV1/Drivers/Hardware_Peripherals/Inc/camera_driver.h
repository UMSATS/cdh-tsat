/*
 * FILENAME: camera_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the piCAM Space Grade Camera from Skyfox Labs.
 *
 * AUTHORS:
 *  - Syed Abraham Ahmed (syed.ahmed@umsats.ca)
 *  - Amer El Eissawi (amer.eleissawi@umsats.ca)
 *
 * CREATED ON: March 2, 2023
 */
#ifndef HARDWARE_PERIPHERALS_INC_CAMERA_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_CAMERA_DRIVER_H_

/*----------------------------------------------------------------------------------------------
Include Directives
-----------------------------------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include <stdint.h>

/*----------------------------------------------------------------------------------------------
Public Define Directives
-----------------------------------------------------------------------------------------------*/

//Initial Definitions
#define piCAM_UART					huart4
#define piCAM_DMA					hdma_uart4_rx
#define piCAM_UART_IRQn				UART4_IRQn

#define piCAM_UART_DELAY			HAL_MAX_DELAY


//Pin Definitions
#define piCAM_ON_GPIO				GPIOA
#define piCAM_ON_PIN 				GPIO_PIN_3

#define piCAM_RX_GPIO				GPIOA
#define piCAM_RX_PIN				GPIO_PIN_1

#define piCAM_TX_GPIO				GPIOA
#define piCAM_TX_PIN				GPIO_PIN_0

#define piCAM_FSH_GPIO				GPIOA
#define piCAM_FSH_PIN				GPIO_PIN_2

#define piCAM_BYTES_PER_SENTENCE	67
#define piCAM_PAYLOAD_LENGTH	 	131071

#define PICAM_CURRENT_SENTENCE_OFFSET 1
/*----------------------------------------------------------------------------------------------
Public Type Definitions
-----------------------------------------------------------------------------------------------*/
typedef enum
{

	piCAM_HAL_OK            = HAL_OK,      //0x00
	piCAM_HAL_ERROR    		= HAL_ERROR,   //0x01
	piCAM_HAL_BUSY          = HAL_BUSY,    //0x02
	piCAM_HAL_TIMEOUT       = HAL_TIMEOUT, //0x03
	piCAM_READY            	= 0x04

} piCAM_StatusTypeDef;

/*----------------------------------------------------------------------------------------------
Public Driver Function Prototypes
-----------------------------------------------------------------------------------------------*/

/*
 * FUNCTION: piCAM_init
 *
 * DESCRIPTION: Initializes GPIO and UART for piCAM use.
 *
 * NOTES:
 * 	- Creates a UART4 instance
 * 	- Sets baud rate to 115200
 * 	- Initializes all UART registers to communicate with piCAM
 */
piCAM_StatusTypeDef piCAM_Init();

/*
 * FUNCTION: piCAM_DMA_Init
 *
 * DESCRIPTION: Initializes DMA for piCAM use.
 *
 * NOTES:
 *	- Simply configures DMA for UART4
 */
void piCAM_DMA_Init();

/*
 * FUNCTION: piCAM_DMA_Start
 *
 * DESCRIPTION: Starts DMA for piCAM use.
 *
 * NOTES:
 *	- Receives data from piCAM onto Payload static buffer
 *	- The initial while loop ensures the UART register is cleared by receiving "tmp"
 */
piCAM_StatusTypeDef piCAM_DMA_Start();

/*
 * FUNCTION: piCAM_Receive_Check
 *
 * DESCRIPTION: Follows the nnnn current sentence portion of the raw data. If it is FACE, it stops
 * the read cycle and calls piCAM_Process_Image();
 *
 * NOTES:
 * -
 */
piCAM_StatusTypeDef piCAM_Receive_Check();

/*
 * FUNCTION: piCAM_Boot_Sequence
 *
 * DESCRIPTION: Follows the boot up sequence for the piCAM.
 *
 * NOTES:
 *  - Disables UART Interface
 * 	- ON, RX, and TX are low for at least a second
 * 	- ON shall be held logic 1
 *	- Activating logic 1 on RX and TX
 *	- Enables UART Interface
 */
void piCAM_Boot_Up_Sequence();
/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to capture a daylight image
 *
 * NOTES: 
 * 	- Sends the "d" command to piCAM
 */
piCAM_StatusTypeDef piCAM_Capture_Daylight();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to capture a medium ambient light image
 *
 * NOTES: 
 * 	- Sends the "m" command to piCAM
 */
piCAM_StatusTypeDef piCAM_Capture_Mediumlight();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to capture a night time image
 *
 * NOTES: 
 * 	- Sends the "n" command to piCAM
 */
piCAM_StatusTypeDef piCAM_Capture_Nightlight();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to send a test string with telemetry measurments
 * while also flashing the camera LED.
 *
 * NOTES: 
 * 	- Sends the "t" command to piCAM
 */
piCAM_StatusTypeDef piCAM_Status_Test();

/*
 * FUNCTION: piCAM_Process_Image
 *
 * DESCRIPTION: Converts the ASCI representation of the image sent by piCAM into
 *  a binary array representing the jpeg image.
 *
 * NOTES: 
 * 	- The binary array is padded with 0's where the length of the array of ASCI
 *  data is larger than that of the converted binary image.
 */
piCAM_StatusTypeDef piCAM_Process_Image();

/*----------------------------------------------------------------------------------------------
Private Helper Function Prototypes
-----------------------------------------------------------------------------------------------*/

/*
 * FUNCTION: piCAM_ASCII_Byte_to_Binary
 *
 * DESCRIPTION: Converts a single byte represented by ASCI data in a byte array into a 
 * binary byte.
 *
 */
uint8_t piCAM_ASCII_Byte_to_Binary(uint8_t *);

/*
 * FUNCTION: piCAM_ASCI_Word_to_Binary
 *
 * DESCRIPTION: Converts a single word represented by ASCI data in a byte array into a
 * binary word.
 *
 */
uint16_t piCAM_ASCI_Word_to_Binary(uint8_t *);

/*----------------------------------------------------------------------------------------------
Public Testing Function Prototypes
-----------------------------------------------------------------------------------------------*/

/*
 * FUNCTION: piCAM_Test_Procedure
 *
 * DESCRIPTION: Tests piCAM functionality by sending a test command and receiving a test string.
 * Following captures and recieves an image from piCAM which is then processed.
 *
 * NOTES:
 * 	- Boots up piCAM
 * 	- Sends a test command to piCAM
 *  - Sends a capture command to piCAM
 * 	- Receives image from piCAM
 * 	- Processes image
 */
void piCAM_Test_Procedure();

#endif /* HARDWARE_PERIPHERALS_INC_CAMERA_DRIVER_H_ */
