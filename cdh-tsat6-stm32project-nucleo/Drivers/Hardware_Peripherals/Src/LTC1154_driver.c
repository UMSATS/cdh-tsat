/*
 * FILENAME: LTC1154_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the LTC1154 high side micropower MOSFET driver.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Apr. 2, 2023
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"
#include "LTC1154_driver.h"

//###############################################################################################
//Public Driver Functions
//###############################################################################################
void LTC1154_Init()
{
    HAL_GPIO_WritePin(LTC1154_IN_GPIO, LTC1154_IN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LTC1154_nEN_GPIO, LTC1154_nEN_PIN, GPIO_PIN_SET);
}

void LTC1154_On()
{
    HAL_GPIO_WritePin(LTC1154_IN_GPIO, LTC1154_IN_PIN, GPIO_PIN_SET);
}

void LTC1154_Off()
{
    HAL_GPIO_WritePin(LTC1154_IN_GPIO, LTC1154_IN_PIN, GPIO_PIN_RESET);
}

void LTC1154_Enable()
{
    HAL_GPIO_WritePin(LTC1154_nEN_GPIO, LTC1154_nEN_PIN, GPIO_PIN_RESET);
}

void LTC1154_Disable()
{
    HAL_GPIO_WritePin(LTC1154_nEN_GPIO, LTC1154_nEN_PIN, GPIO_PIN_SET);
}
