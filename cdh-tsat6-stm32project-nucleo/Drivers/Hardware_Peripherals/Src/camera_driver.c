/*
 * FILENAME: camera_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the piCAM Space Grade Camera from Skyfox Labs.
 *
 * AUTHORS:
 *  - Syed Abraham Ahmed (syed.ahmed@umsats.ca)
 *  - Amer El Eissawi (amer.eleissawi@umsats.ca)
 *
 * CREATED ON: March 2, 2023
 */

/************************************************************************************************
/Include Directives
/************************************************************************************************/
#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "camera_driver.h"

/************************************************************************************************
/Global Variable Declarations
/************************************************************************************************/
extern UART_HandleTypeDef piCAM_UART;

/************************************************************************************************
/Driver Function Prototypes
/************************************************************************************************

/************************************************************************************************
/Public Driver Function Definitions
/************************************************************************************************/
piCAM_StatusTypeDef piCAM_Boot_Sequence()
{
}

piCAM_StatusTypeDef piCAM_init()
{
    // Initialize the GPIO port clock
	__HAL_RCC_GPIOC_CLK_ENABLE();

    // Initialize UART4 to be used a piCAM_UART
    piCAM_UART.Instance = UART4;
    piCAM_UART.Init.BaudRate = 115200;
    piCAM_UART.Init.WordLength = UART_WORDLENGTH_8B;
    piCAM_UART.Init.StopBits = UART_STOPBITS_1;
    piCAM_UART.Init.Parity = UART_PARITY_NONE;
    piCAM_UART.Init.Mode = UART_MODE_TX_RX;
    piCAM_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    piCAM_UART.Init.OverSampling = UART_OVERSAMPLING_16;
    piCAM_UART.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    piCAM_UART.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&piCAM_UART) != HAL_OK)
    {
        return piCAM_HAL_ERROR;
    }
}

piCAM_StatusTypeDef piCAM_Capture_Daylight()
{
    uint8_t command[] = "d";
    return HAL_UART_Transmit(&piCAM_UART, command, sizeof(command), HAL_MAX_DELAY);
}

piCAM_StatusTypeDef piCAM_Capture_Mediumlight()
{
    uint8_t command[] = "m";
    return HAL_UART_Transmit(&piCAM_UART, command, sizeof(command), HAL_MAX_DELAY);
}

piCAM_StatusTypeDef piCAM_Capture_Nightlight()
{
    uint8_t command[] = "n";
    return HAL_UART_Transmit(&piCAM_UART, command, sizeof(command), HAL_MAX_DELAY);
}

piCAM_StatusTypeDef piCAM_Status_Test()
{
    uint8_t command[] = "t";
    return HAL_UART_Transmit(&piCAM_UART, command, sizeof(command), HAL_MAX_DELAY);
}