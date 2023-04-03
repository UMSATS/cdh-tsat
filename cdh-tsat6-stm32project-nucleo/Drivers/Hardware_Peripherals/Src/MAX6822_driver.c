/*
 * FILENAME: MAX6822_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the MAX6822RUK+T watchdog / voltage supervisor.
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
#include "MAX6822_driver.h"

//###############################################################################################
//Public Driver Functions
//###############################################################################################
void MAX6822_Init()
{
    HAL_GPIO_WritePin(M_nReset_GPIO, M_nReset_PIN, GPIO_PIN_SET);
}

void MAX6822_Manual_Reset()
{
    HAL_GPIO_WritePin(M_nReset_GPIO, M_nReset_PIN, GPIO_PIN_RESET);
}

void MAX6822_WDI_Toggle()
{
    HAL_GPIO_TogglePin(WDI_GPIO, WDI_PIN);
}
