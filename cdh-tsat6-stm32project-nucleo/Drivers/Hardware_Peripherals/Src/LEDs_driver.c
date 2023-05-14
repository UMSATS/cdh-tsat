/*
 * FILENAME: LEDs_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the 3 LEDs.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Mar. 18, 2023
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"
#include "LEDs_driver.h"

//###############################################################################################
//Public Driver Functions
//###############################################################################################
void LEDs_Init()
{
    HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO, LED2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO, LED3_PIN, GPIO_PIN_RESET);
}

void LED1_On()
{
    HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET);
}

void LED2_On()
{
    HAL_GPIO_WritePin(LED2_GPIO, LED2_PIN, GPIO_PIN_SET);
}

void LED3_On()
{
    HAL_GPIO_WritePin(LED3_GPIO, LED3_PIN, GPIO_PIN_SET);
}

void LED1_Off()
{
    HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_RESET);
}

void LED2_Off()
{
    HAL_GPIO_WritePin(LED2_GPIO, LED2_PIN, GPIO_PIN_RESET);
}

void LED3_Off()
{
    HAL_GPIO_WritePin(LED3_GPIO, LED3_PIN, GPIO_PIN_RESET);
}

void LED1_Toggle()
{
    HAL_GPIO_TogglePin(LED1_GPIO, LED1_PIN);
}

void LED2_Toggle()
{
    HAL_GPIO_TogglePin(LED2_GPIO, LED2_PIN);
}

void LED3_Toggle()
{
    HAL_GPIO_TogglePin(LED3_GPIO, LED3_PIN);
}
