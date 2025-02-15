/*
 * bdot_algorithm.h
 *
 *  Created on: Feb 15, 2025
 *      Author: drive
 */

#ifndef INC_BDOT_ALGORITHM_H_
#define INC_BDOT_ALGORITHM_H_

#include <stdio.h>
#include <stdlib.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"


extern osThreadId_t calculateBDotHandle;


void StartBDot(void *argument);


#endif /* INC_BDOT_ALGORITHM_H_ */
