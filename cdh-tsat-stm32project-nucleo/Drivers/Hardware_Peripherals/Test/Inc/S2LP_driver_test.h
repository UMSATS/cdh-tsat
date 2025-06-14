/*
 * S2LP_driver_test.h
 *
 *  Created on: Feb 25, 2025
 *      Author: drive
 */

#ifndef HARDWARE_PERIPHERALS_TEST_INC_S2LP_DRIVER_TEST_H_
#define HARDWARE_PERIPHERALS_TEST_INC_S2LP_DRIVER_TEST_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "S2LP_driver.h"


S2LP_StatusTypeDef Test_S2LP_Transmission(uint8_t *message, uint32_t messagesToSend, uint32_t size, uint32_t delay);

S2LP_StatusTypeDef Test_S2LP_Receiving(uint8_t *message, uint32_t messagesToSend, uint32_t size, uint32_t delay);

#endif /* HARDWARE_PERIPHERALS_TEST_INC_S2LP_DRIVER_TEST_H_ */
