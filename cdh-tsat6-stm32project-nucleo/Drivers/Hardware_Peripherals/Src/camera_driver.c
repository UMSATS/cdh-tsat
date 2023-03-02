/*
 * FILENAME: camera_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the piCAM Space Grade Camera from Skyfox Labs.
 *
 * AUTHORS:
 *  - Syed Abraham Ahmed (syed.ahmd@umsats.ca)
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
#define piCAM_UART 			huart4
#define piCAM_UART_DELAY	HAL_MAX_DELAY

//Pin Definitions
#define piCAM_ON_GPIO		GPIOA
#define piCAM_ON_PIN 		GPIO_PIN_10

#define piCAM_RX_GPIO		GPIOC
#define piCAM_RX_PIN		GPIO_PIN_11

#define piCAM_TX_GPIO		GPIOA
#define piCAM_TX_PIN		GPIO_PIN_0

#define piCAM_FSH_GPIO		GPIOA
#define piCAM_FSH_PIN		GPIO_PIN_9

/************************************************************************************************
/Public Type Definitions
************************************************************************************************/
typedef enum
{
    piCAM_HAL_OK            = HAL_OK,      //0x00
	piCAM_HAL_ERRO          = HAL_ERROR,   //0x01
	piCAM_HAL_BUSY          = HAL_BUSY,    //0x02
	piCAM_HAL_TIMEOUT       = HAL_TIMEOUT, //0x03
	piCAM_READY            	= 0x04

} piCAM_StatusTypeDef;

/************************************************************************************************
/Public Driver Functions
************************************************************************************************/



#endif /* HARDWARE_PERIPHERALS_INC_CAMERA_DRIVER_H_ */
