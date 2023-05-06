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

/************************************************************************************************
/Include Directives
************************************************************************************************/
#include "stm32l4xx_hal.h"

#include <stdint.h>

/************************************************************************************************
/Public Define Directives
************************************************************************************************/

//Initial Definitions
#define piCAM_UART					huart4
#define piCAM_UART_DELAY			HAL_MAX_DELAY

//Pin Definitions
#define piCAM_ON_GPIO				GPIOA
#define piCAM_ON_PIN 				GPIO_PIN_10

#define piCAM_RX_GPIO				GPIOC
#define piCAM_RX_PIN				GPIO_PIN_11

#define piCAM_TX_GPIO				GPIOA
#define piCAM_TX_PIN				GPIO_PIN_0

#define piCAM_FSH_GPIO				GPIOA
#define piCAM_FSH_PIN				GPIO_PIN_9

#define piCAM_BYTES_PER_SENTANCE
/************************************************************************************************
/Public Type Definitions
/************************************************************************************************/
typedef enum
{

	piCAM_HAL_OK            = HAL_OK,      //0x00
	piCAM_HAL_ERROR    		= HAL_ERROR,   //0x01
	piCAM_HAL_BUSY          = HAL_BUSY,    //0x02
	piCAM_HAL_TIMEOUT       = HAL_TIMEOUT, //0x03
	piCAM_READY            	= 0x04

} piCAM_StatusTypeDef;

/************************************************************************************************
/Public Driver Function Prototype
/************************************************************************************************/

/*
 * FUNCTION: piCAM_Boot_Sequence
 *
 * DESCRIPTION: Follows the boot up sequence for the piCAM.
 *
 * NOTES:
 * 	- ON, RX, and TX are low for at least a second
 * 	- ON shall be held log 1
 *	- Activating log 1 on RX and TX
 */

piCAM_StatusTypeDef piCAM_Boot_Sequence();

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
piCAM_StatusTypeDef piCAM_init();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to capture a daylight image
 *
 * NOTES: 
 * 	- Sends the "d" commmand to piCAM
 */
piCAM_StatusTypeDef piCAM_Capture_Daylight();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to capture a medium ambient light image
 *
 * NOTES: 
 * 	- Sends the "m" commmand to piCAM
 */
piCAM_StatusTypeDef piCAM_Capture_Mediumlight();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to capture a night time image
 *
 * NOTES: 
 * 	- Sends the "n" commmand to piCAM
 */
piCAM_StatusTypeDef piCAM_Capture_Nightlight();

/*
 * FUNCTION: piCAM_Capture_Daylight
 *
 * DESCRIPTION: Commands piCAM to send a test string with telemetry measurments
 * while also flashing the camera LED.
 * 
 *
 * NOTES: 
 * 	- Sends the "t" commmand to piCAM
 */
piCAM_StatusTypeDef piCAM_Status_Test();

#endif /* HARDWARE_PERIPHERALS_INC_CAMERA_DRIVER_H_ */
