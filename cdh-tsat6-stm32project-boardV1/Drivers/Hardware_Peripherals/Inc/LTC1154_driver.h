/*
 * FILENAME: LTC1154_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the LTC1154 high side micropower MOSFET driver.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Apr. 2, 2023
 */

#ifndef HARDWARE_PERIPHERALS_INC_LTC1154_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_LTC1154_DRIVER_H_
//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################
#define LTC1154_IN_GPIO      GPIOA
#define LTC1154_IN_PIN       GPIO_PIN_2

#define LTC1154_nEN_GPIO     GPIOA
#define LTC1154_nEN_PIN      GPIO_PIN_1

//###############################################################################################
//Public Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: LTC1154_Init
 *
 * DESCRIPTION: Turn off & disable the LTC1154 operation.
 */
void LTC1154_Init();

/*
 * FUNCTION: LTC1154_On
 *
 * DESCRIPTION: Turn the LTC1154 on.
 *
 * NOTES:
 *  - This function will only activate the LTC1154 operation when the device is enabled.
 */
void LTC1154_On();

/*
 * FUNCTION: LTC1154_Off
 *
 * DESCRIPTION: Turn the LTC1154 off.
 */
void LTC1154_Off();

/*
 * FUNCTION: LTC1154_Enable
 *
 * DESCRIPTION: Enable the LTC1154 operations.
 */
void LTC1154_Enable();

/*
 * FUNCTION: LTC1154_Disable
 *
 * DESCRIPTION: Disable the LTC1154 operations.
 */
void LTC1154_Disable();


#endif /* HARDWARE_PERIPHERALS_INC_LTC1154_DRIVER_H_ */
