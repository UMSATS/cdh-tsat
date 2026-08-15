/*
 * unit_tests.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_UNIT_TESTS_H_
#define INC_UNIT_TESTS_H_

#include <Dhara_Wrapper.h>
#include <StorageManager.h>
#include "Dhara_test.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "W25N_driver.h"
#include "W25N_driver_test.h"
#include "AS3001204_driver.h"
#include "AS3001204_driver_test.h"

/**
* @brief Function implementing the flashUnitTest thread.
* @param argument: Not used
* @retval None
*/
void StartFlashUnitTest(void *argument);


/**
* @brief Function implementing the mramUnitTest thread.
* @param argument: Not used
* @retval None
*/
void StartMramUnitTest(void *argument);

/**
* @brief Function implementing the mramUnitTest thread.
* @param argument: Not used
* @retval None
*/
void StartDharaUnitTest(void *argument);

/**
* @brief Function implementing the mramUnitTest thread.
* @param argument: Not used
* @retval None
*/
void StartStorageManagerUnitTest(void *argument);

#endif /* INC_UNIT_TESTS_H_ */
