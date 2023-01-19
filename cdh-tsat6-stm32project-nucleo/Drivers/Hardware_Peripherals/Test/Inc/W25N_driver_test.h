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
//Unit Test Function Prototypes
//###############################################################################################
/*
 * FUNCTION: Test_W25N_Device_Reset
 *
 * DESCRIPTION: Unit test function for the W25N_Device_Reset function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Write_Status_Register, Test_W25N_Read_Status_Register
 *  - Assumes OTP area is not locked & SR-1 is not locked.
 */
void Test_W25N_Device_Reset();

/*
 * FUNCTION: Test_W25N_Read_JEDEC_ID
 *
 * DESCRIPTION: Unit test function for the W25N_Read_JEDEC_ID function.
 *
 * NOTES:
 *  - Not dependent on passing any tests.
 *  - Note2
 */
void Test_W25N_Read_JEDEC_ID();

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
void Test_W25N_Read_Status_Register();

/*
 * FUNCTION: Test_W25N_Write_Status_Register
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Status_Register function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read_Status_Register
 *  - Test must be performed after power cycling the W25N.
 */
void Test_W25N_Write_Status_Register();

/*
 * FUNCTION: Test_W25N_Write_Enable
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Enable function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read_Status_Register
 *  - Test must be performed after power cycling the W25N.
 */
void Test_W25N_Write_Enable();

/*
 * FUNCTION: Test_W25N_Write_Disable
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Disable function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read_Status_Register
 *  - Test must be performed directly after Test_W25N_Write_Enable
 */
void Test_W25N_Write_Disable();

/*
 * FUNCTION: Test_W25N_Block_Erase_128KB
 *
 * DESCRIPTION: Unit test function for the W25N_Block_Erase_128KB function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Load_Program_Data, Test_W25N_Program_Execute, Test_W25N_Read
 *  - Note2
 */
void Test_W25N_Block_Erase_128KB();

/*
 * FUNCTION: Test_W25N_Load_Program_Data
 *
 * DESCRIPTION: Unit test function for the W25N_Load_Program_Data function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Read
 *  - Note2
 */
void Test_W25N_Load_Program_Data();

/*
 * FUNCTION: Test_W25N_Program_Execute
 *
 * DESCRIPTION: Unit test function for the W25N_Program_Execute function.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Load_Program_Data, Test_W25N_Read
 *  - Note2
 */
void Test_W25N_Program_Execute();

/*
 * FUNCTION: Test_W25N_Read
 *
 * DESCRIPTION: Unit test function for the W25N_Page_Data_Read & W25N_Read_Data functions.
 *
 * NOTES:
 *  - Depends on passing: Test_W25N_Write_Status_Register
 *  - Multiple functions are tested since they are interdependent in terms of testing.
 */
void Test_W25N_Read();

//###############################################################################################
//Complete Unit Test Function Prototype
//###############################################################################################
/*
 * FUNCTION: Unit_Test_W25N
 *
 * DESCRIPTION: Complete unit test function for the W25N01GVZEIG NAND Flash STM32L4 driver.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Unit_Test_W25N();

#endif /* HARDWARE_PERIPHERALS_TEST_INC_W25N_DRIVER_TEST_H_ */
