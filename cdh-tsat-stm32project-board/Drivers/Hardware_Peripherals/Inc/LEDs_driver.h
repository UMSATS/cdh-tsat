/*
 * FILENAME: LEDs_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the 3 LEDs.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Mar. 18, 2023
 */

#ifndef HARDWARE_PERIPHERALS_INC_LEDS_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_LEDS_DRIVER_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################
#define LED1_GPIO     GPIOB
#define LED1_PIN      GPIO_PIN_2

#define LED2_GPIO     GPIOB
#define LED2_PIN      GPIO_PIN_10

#define LED3_GPIO     GPIOB
#define LED3_PIN      GPIO_PIN_11

//###############################################################################################
//Public Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: LEDs_Init
 *
 * DESCRIPTION: Turn all 3 LEDs off.
 */
void LEDs_Init();

/*
 * FUNCTION: LED1_On
 *
 * DESCRIPTION: Turn LED1 on.
 */
void LED1_On();

/*
 * FUNCTION: LED2_On
 *
 * DESCRIPTION: Turn LED2 on.
 */
void LED2_On();

/*
 * FUNCTION: LED3_On
 *
 * DESCRIPTION: Turn LED3 on.
 */
void LED3_On();

/*
 * FUNCTION: LED1_Off
 *
 * DESCRIPTION: Turn LED1 off.
 */
void LED1_Off();

/*
 * FUNCTION: LED2_Off
 *
 * DESCRIPTION: Turn LED2 off.
 */
void LED2_Off();

/*
 * FUNCTION: LED3_Off
 *
 * DESCRIPTION: Turn LED3 off.
 */
void LED3_Off();

/*
 * FUNCTION: LED1_Toggle
 *
 * DESCRIPTION: Toggle LED1.
 */
void LED1_Toggle();

/*
 * FUNCTION: LED2_Toggle
 *
 * DESCRIPTION: Toggle LED2.
 */
void LED2_Toggle();

/*
 * FUNCTION: LED3_Toggle
 *
 * DESCRIPTION: Toggle LED3.
 */
void LED3_Toggle();

#endif /* HARDWARE_PERIPHERALS_INC_LEDS_DRIVER_H_ */
