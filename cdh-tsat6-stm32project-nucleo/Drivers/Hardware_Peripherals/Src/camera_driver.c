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
    if (HAL_UART_Init(&piCAM_UART) != piCAM_HAL_OK)
    {
    	return piCAM_HAL_ERROR;
    }
    return piCAM_HAL_OK;
}

piCAM_StatusTypeDef piCAM_Boot_Up_Sequence()
{
    //Disables UART4 To Prevent Bootstrapping
    disable_piCAM_UART();

    //Hold the lines LOW for 1 second
    HAL_GPIO_WritePin(piCAM_ON_GPIO, piCAM_ON_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(piCAM_RX_GPIO, piCAM_RX_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(piCAM_TX_GPIO, piCAM_TX_PIN, GPIO_PIN_RESET);

    HAL_Delay(1000);

    //Set ON signal to logic HIGH
    HAL_GPIO_WritePin(piCAM_ON_GPIO, piCAM_ON_PIN, GPIO_PIN_SET);

    //Activate idle levels (logic HIGH) on RXD and TXD
    HAL_GPIO_WritePin(piCAM_RX_GPIO, piCAM_RX_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(piCAM_TX_GPIO, piCAM_TX_PIN, GPIO_PIN_SET);

    //Enables UART4
    enable_piCAM_UART();
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

piCAM_StatusTypeDef piCAM_Process_Image(uint8_t *outputImage)
{
    uint8_t *firstFree = outputImage;
    uint8_t *iterator = outputImage;
    uint16_t currentSentence = piCAM_ASCI_Word_to_Binary(iterator + 1);
    uint16_t totalSentences = piCAM_ASCI_Word_to_Binary(iterator + 5);
    uint32_t imageLength = ((uint32_t)totalSentences) * 67;
    iterator += 9;

    while (currentSentence != totalSentences + 1)
    {
        for (int i = 0; i < 28; i++)
        {
            *firstFree = piCAM_ASCII_Byte_to_Binary(iterator);
            firstFree++;
            iterator += 2;
        }
        currentSentence = piCAM_ASCI_Word_to_Binary(iterator + 4);
        iterator += 11;
    }

    for (int i = 0; firstFree != &outputImage[imageLength]; i++)
    {
        *firstFree = 0x00;
        firstFree++;
    }
    return piCAM_HAL_OK;
}

/************************************************************************************************
/Private Helper Function Definitions
/************************************************************************************************/
uint8_t piCAM_ASCII_Byte_to_Binary(uint8_t *convert)
{
    const uint8_t binary[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    const uint8_t digits[] = "0123456789ABCDEF";
    uint8_t lowNibble = 0;
    uint8_t highNibble = 0;
    // Set the value of the low and high nibble
    for (int i = 0; i < 16; i++)
    {
        if (convert[0] == digits[i])
        {
            highNibble = binary[i];
        }
        if (convert[1] == digits[i])
        {
            lowNibble = binary[i];
        }
    }
    // Return the value of the byte
    return (highNibble << 4) | lowNibble; 
}

uint16_t piCAM_ASCI_Word_to_Binary(uint8_t *convert)
{
    uint16_t highByte = piCAM_ASCII_Byte_to_Binary(convert);
    uint16_t lowByte = piCAM_ASCII_Byte_to_Binary(convert + 2);
    return (highByte << 8) | lowByte;
}

void disable_piCAM_UART()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//DeInitializes the UART4 interface
    HAL_UART_DeInit(&piCAM_UART);

    //Disables UART4 Interrupt
    HAL_NVIC_DisableIRQ(piCAM_UART_IRQn);

    /*Configure GPIO pin : CAM_TX_ANTI_BOOTSTRAP_Pin */
    GPIO_InitStruct.Pin = piCAM_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(piCAM_TX_GPIO, &GPIO_InitStruct);

    /*Configure GPIO pin : CAM_RX_ANTI_BOOTSTRAP_Pin */
    GPIO_InitStruct.Pin = piCAM_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(piCAM_RX_GPIO, &GPIO_InitStruct);
}

void enable_piCAM_UART()
{
	//DeInitializes GPIO RX and TX Pins
	HAL_GPIO_DeInit(piCAM_RX_GPIO, piCAM_RX_PIN);
	HAL_GPIO_DeInit(piCAM_TX_GPIO, piCAM_TX_PIN);

	//Initializes the UART4 interface
    piCAM_init();

    //Enables UART4 Interrupt
    HAL_NVIC_EnableIRQ(piCAM_UART_IRQn);
}
