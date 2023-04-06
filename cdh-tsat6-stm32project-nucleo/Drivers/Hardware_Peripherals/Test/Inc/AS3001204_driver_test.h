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

#ifndef HARDWARE_PERIPHERALS_AS3001204_DRIVER_TEST_H_
#define HARDWARE_PERIPHERALS_AS3001204_DRIVER_TEST_H_

#include "stm32l4xx_hal.h"


/*
 * Read ID registers test
 *
 * FUNCTIONS:   AS3001204_Test_Read_ID_Registers
 *
 * DESCRIPTION: This function is a simple test that reads the (read-only) device ID and
 * 				unique ID registers of the AS3001204 MRAM unit, and compares their
 * 				contents to known values.
 *
 * RETURN:      HAL_OK if successful; HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT if not.
 */
HAL_StatusTypeDef AS3001204_Test_Read_ID_Registers();

/*
 * Read/write register tests
 *
 * FUNCTIONS:   AS3001204_Test_RW_Status_Register, AS3001204_Test_RW_Config_Registers,
 * 	            AS3001204_Test_RW_ASP_Register
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to read and write to the
 * 	            MRAM unit's status register, configuration registers, and augmented storage array
 * 	            protection register, respectively. These functions only write valid data to the
 * 	            registers - writing invalid bits to registers could cause undefined behaviour.
 *
 * RETURN:      HAL_OK if successful; HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT if not.
 */
HAL_StatusTypeDef AS3001204_Test_RW_Status_Register();
HAL_StatusTypeDef AS3001204_Test_RW_Config_Registers();
HAL_StatusTypeDef AS3001204_Test_RW_ASP_Register();

/*
 * Read/write memory tests
 *
 * FUNCTIONS:   AS3001204_Test_RW_Memory, AS3001204_Test_RW_Augmented_Storage
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to read and write the
 * 	            main memory array, and the augmented storage section of memory.
 *
 * RETURN:      HAL_OK if successful; HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT if not.
 */
HAL_StatusTypeDef AS3001204_Test_RW_Memory();
HAL_StatusTypeDef AS3001204_Test_RW_Augmented_Storage();

/*
 * Enter/exit hibernate and deep power down tests
 *
 * FUNCTIONS:   AS3001204_Test_Enter_Exit_Hibernate, AS3001204_Test_Enter_Exit_Deep_Power_Down
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to enter and exit hibernate
 *              and deep power down modes of operation.
 *
 * RETURN:      HAL_OK if successful; HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT if not.
 */
HAL_StatusTypeDef AS3001204_Test_Enter_Exit_Hibernate();
HAL_StatusTypeDef AS3001204_Test_Enter_Exit_Deep_Power_Down();

/*
 * Complete test suite
 * 
 * FUNCTION:    AS3001204_Test_MRAM_Driver
 *
 * DESCRIPTION:	This function runs through the complete suite of unit tests available for
 *              the MRAM driver.
 *
 * RETURN:      HAL_OK if successful; HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT if not.
 */
HAL_StatusTypeDef AS3001204_Test_MRAM_Driver();


#endif /* HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_TEST_H_ */
