/*
 * FILENAME: Si4464_driver_test.c
 *
 * DESCRIPTION: Unit test source file for the Si4464 RF Chip STM32L4 driver.
 *
 * AUTHORS:
 *  - Nikolaus Reichert (nikolaus.reichert@umsats.ca)
 *
 * CREATED ON: Mar. 14, 2023
 */

#include "Si4464_driver_test.h"
#include "Si4464_driver.h"


//###############################################################################################
//Public Unit Test Functions
//###############################################################################################
HAL_StatusTypeDef Test_Si4464_Reset_Device()
{


}

//###############################################################################################
//Public Complete Unit Test Function
//###############################################################################################
HAL_StatusTypeDef Test_Si4464()
{
    HAL_StatusTypeDef operation_status;

    //the following functions are executed in order of dependencies

    //unit test functions
    operation_status = Test_Si4464_Get_CTS();
    if (operation_status != HAL_OK) goto error;

    operation_status = Test_Si4464_Reset_Device();
    if (operation_status != HAL_OK) goto error;

error:
    return operation_status;
}
