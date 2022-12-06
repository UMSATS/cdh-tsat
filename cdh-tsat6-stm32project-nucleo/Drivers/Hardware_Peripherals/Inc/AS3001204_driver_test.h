/*
 * FILENAME: AS3001204_driver_test.h
 *
 * DESCRIPTION: Unit test for the AS3001204-0054X0ISAY MRAM driver
 * 	            (see AS3001204_driver.{c,h})
 *
 * AUTHORS:
 *  - Gabriel Young (gabriel.young@umsats.ca)
 *
 * Created on: Dec. 6, 2022
 */

#ifndef HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_TEST_H_
#define HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_TEST_H_

#include "stm32l4xx_hal.h"


//###############################################################################################
// Read/write register tests
//###############################################################################################

/*
 * FUNCTIONS:   AS3001204_Test_RW_Status_Register, AS3001204_Test_RW_Config_Registers,
 * 	            AS3001204_Test_RW_Augmented_Array_Protection_Register
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to read and write to the
 * 	            MRAM unit's status register, configuration registers, and augmented array
 * 	            protection register, respectively. These functions only write valid data to the
 * 	            registers - writing invalid bits to registers could cause undefined behaviour.
 *
 * RETURN:      0 if successful, 1 if not.
 */
unsigned int AS3001204_Test_RW_Status_Register();
unsigned int AS3001204_Test_RW_Config_Registers();
unsigned int AS3001204_Test_RW_Augmented_Array_Protection_Register();

//###############################################################################################
// Read/write memory tests
//###############################################################################################

/*
 * FUNCTIONS:   AS3001204_Test_RW_Memory, AS3001204_Test_RW_Augmented_Storage
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to read and write the
 * 	            main memory array, and the augmented storage section of memory.
 *
 * RETURN:      0 if successful, 1 if not.
 */
unsigned int AS3001204_Test_RW_Memory();
unsigned int AS3001204_Test_RW_Augmented_Storage();


#endif /* HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_TEST_H_ */