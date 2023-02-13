/*
 * FILENAME: W25N_driver_test.h
 *
 * DESCRIPTION: Unit test header file for the W25N01GVZEIG NAND Flash STM32L4 driver.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * CREATED ON: Dec. 28, 2022
 */

#ifndef HARDWARE_PERIPHERALS_TEST_INC_W25N_DRIVER_TEST_H_
#define HARDWARE_PERIPHERALS_TEST_INC_W25N_DRIVER_TEST_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include "W25N_driver.h"

//###############################################################################################
//Public Unit Test Function Prototypes
//###############################################################################################
/*
 * FUNCTION: Test_W25N_Device_Reset
 *
 * DESCRIPTION: Unit test function for the W25N_Device_Reset function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Write_Status_Register, Test_W25N_Read_Status_Register
 *  - Assumes OTP area is not locked & SR-1 is not locked.
 *  - If the unit test exits prematurely, the W25N should be power cycled so that the status 
 *    register contents are reset.
 */
W25N_StatusTypeDef Test_W25N_Device_Reset();

/*
 * FUNCTION: Test_W25N_Read_JEDEC_ID
 *
 * DESCRIPTION: Unit test function for the W25N_Read_JEDEC_ID function.
 *
 * NOTES:
 *  - Not dependent on passing any tests.
 *  - Note2
 */
W25N_StatusTypeDef Test_W25N_Read_JEDEC_ID();

/*
 * FUNCTION: Test_W25N_Read_Status_Register
 *
 * DESCRIPTION: Unit test function for the W25N_Read_Status_Register function.
 *
 * NOTES:
 *  - Not dependent on passing any tests.
 *  - Test must be performed after power cycling the W25N.
 *  - Test must be performed before Test_W25N_Write_Status_Register.
 */
W25N_StatusTypeDef Test_W25N_Read_Status_Register();

/*
 * FUNCTION: Test_W25N_Write_Status_Register
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Status_Register function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read_Status_Register
 *  - Test must be performed after power cycling the W25N.
 *  - If the unit test exits prematurely, the W25N should be power cycled so that the status 
 *    register contents are reset.
 */
W25N_StatusTypeDef Test_W25N_Write_Status_Register();

/*
 * FUNCTION: Test_W25N_Write_Enable
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Enable function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read_Status_Register
 *  - Test must be performed after power cycling the W25N.
 *  - If the unit test exits prematurely, the W25N should be power cycled so that the write 
 *    enable latch is reset.
 */
W25N_StatusTypeDef Test_W25N_Write_Enable();

/*
 * FUNCTION: Test_W25N_Write_Disable
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Disable function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read_Status_Register
 *  - Test must be performed directly after Test_W25N_Write_Enable.
 *  - If the unit test exits prematurely, the W25N should be power cycled so that the write 
 *    enable latch is reset.
 */
W25N_StatusTypeDef Test_W25N_Write_Disable();

/*
 * FUNCTION: Test_W25N_Load_Program_Data
 *
 * DESCRIPTION: Unit test function for the W25N_Load_Program_Data function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read
 *  - Note2
 */
W25N_StatusTypeDef Test_W25N_Load_Program_Data();

/*
 * FUNCTION: Test_W25N_Execute_Erase
 *
 * DESCRIPTION: Unit test function for the W25N_Program_Execute & W25N_Block_Erase_128KB functions.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Load_Program_Data, Test_W25N_Read
 *  - Multiple functions are tested since they are interdependent in terms of testing.
 *  - If the unit test exits prematurely, the W25N should be power cycled so that the write 
 *    enable latch is reset.
 *  - If the unit test exits prematurely, the W25N's final block should be erased so that any 
 *    test data is erased. This final block contains the page (page address = 0xFFFF) which is 
 *    used for this unit test.
 */
W25N_StatusTypeDef Test_W25N_Execute_Erase();

/*
 * FUNCTION: Test_W25N_Read
 *
 * DESCRIPTION: Unit test function for the W25N_Page_Data_Read & W25N_Read_Data functions.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Write_Status_Register
 *  - Multiple functions are tested since they are interdependent in terms of testing.
 *  - Assumes OTP area is not locked & SR-1 is not locked.
 *  - If the unit test exits prematurely, the W25N should be power cycled so that the status 
 *    register contents are reset.
 */
W25N_StatusTypeDef Test_W25N_Read();

//###############################################################################################
//Public High-Level Unit Test Function Prototypes
//###############################################################################################
/*
 * FUNCTION: Test_W25N_High_Level_Read
 *
 * DESCRIPTION: Unit test function for the W25N_Read function.
 *
 * NOTES:
 *  - Depends on passing: All low-level unit test functions
 *  - Note2
 *  - Note3
 */
W25N_StatusTypeDef Test_W25N_High_Level_Read();

/*
 * FUNCTION: Test_W25N_High_Level_Write
 *
 * DESCRIPTION: Unit test function for the W25N_Write function.
 *
 * NOTES:
 *  - Depends on passing: All low-level unit test functions, Test_W25N_High_Level_Read
 *  - Note2
 *  - Note3
 */
W25N_StatusTypeDef Test_W25N_High_Level_Write();

/*
 * FUNCTION: Test_W25N_High_Level_Erase
 *
 * DESCRIPTION: Unit test function for the W25N_Erase function.
 *
 * NOTES:
 *  - Depends on passing: All low-level unit test functions, Test_W25N_High_Level_Read, Test_W25N_High_Level_Write
 *  - Note2
 *  - Note3
 */
W25N_StatusTypeDef Test_W25N_High_Level_Erase();

//###############################################################################################
//Public Complete Unit Test Function Prototype
//###############################################################################################
/*
 * FUNCTION: Test_W25N
 *
 * DESCRIPTION: Complete unit test function for the W25N01GVZEIG NAND Flash STM32L4 driver.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
W25N_StatusTypeDef Test_W25N();

#endif /* HARDWARE_PERIPHERALS_TEST_INC_W25N_DRIVER_TEST_H_ */
