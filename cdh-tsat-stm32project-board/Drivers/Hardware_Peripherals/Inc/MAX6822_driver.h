/*
 * FILENAME: MAX6822_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the MAX6822RUK+T watchdog / voltage supervisor.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *
 * CREATED ON: Mar. 18, 2023
 */

#ifndef HARDWARE_PERIPHERALS_INC_MAX6822_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_MAX6822_DRIVER_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################
#define WDI_GPIO          GPIOB
#define WDI_PIN           GPIO_PIN_6

#define M_nReset_GPIO     GPIOB
#define M_nReset_PIN      GPIO_PIN_7

//###############################################################################################
//Public Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: MAX6822_Init
 *
 * DESCRIPTION: Set the M_nReset pin HIGH.
 *
 * NOTES:
 *  - Keeping the M_nReset pin HIGH will keep the MAX6822 output reset pin HIGH, allowing the 
 *    STM32 to continue running.
 */
void MAX6822_Init();

/*
 * FUNCTION: MAX6822_Manual_Reset
 *
 * DESCRIPTION: Set the M_nReset pin LOW.
 *
 * NOTES:
 *  - Setting the M_nReset pin LOW will trigger the MAX6822 to set its output reset pin LOW, 
 *    resetting the STM32.
 *  - The MAX6822 output reset pin is held LOW while M_nReset is held LOW, and for the timeout 
 *    period of ~200ms after M_nReset returns HIGH.
 *  - The MAX6822 contains an internal 50k pullup resistor for the M_nReset input, which means
 *    the M_nReset line will be pulled HIGH once the STM32 enters a reset state.
 */
void MAX6822_Manual_Reset();

/*
 * FUNCTION: MAX6822_WDI_Toggle
 *
 * DESCRIPTION: Toggle the WDI pin.
 *
 * NOTES:
 *  - If the WDI pin is not toggled within a 1.6 second window, the MAX6822 will set its ouptut
 *    reset pin LOW, resetting the STM32.
 *  - The 1.6 second window is restarted after every successful WDI toggle, or after the MAX6822
 *    returns its output reset pin HIGH following a reset assertion.
 *  - Since the 1.6 second window is restarted following a reset assertion, the STM32 must 
 *    continue to call this function while performing initialization tasks, if performing all the
 *    initialization tasks takes close to or greater than 1.6 seconds.
 */
void MAX6822_WDI_Toggle();

#endif /* HARDWARE_PERIPHERALS_INC_MAX6822_DRIVER_H_ */
