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
 *  - Note1
 *  - Note2
 */
void Test_W25N_Device_Reset();

/*
 * FUNCTION: Test_W25N_Read_JEDEC_ID
 *
 * DESCRIPTION: Unit test function for the W25N_Read_JEDEC_ID function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Read_JEDEC_ID();

/*
 * FUNCTION: Test_W25N_Read_Status_Register
 *
 * DESCRIPTION: Unit test function for the W25N_Read_Status_Register function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Read_Status_Register();

/*
 * FUNCTION: Test_W25N_Write_Status_Register
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Status_Register function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Write_Status_Register();

/*
 * FUNCTION: Test_W25N_Write_Enable
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Enable function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Write_Enable();

/*
 * FUNCTION: Test_W25N_Write_Disable
 *
 * DESCRIPTION: Unit test function for the W25N_Write_Disable function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Write_Disable();

/*
 * FUNCTION: Test_W25N_Bad_Block_Management
 *
 * DESCRIPTION: Unit test function for the W25N_Bad_Block_Management function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Bad_Block_Management();

/*
 * FUNCTION: Test_W25N_Read_BBM_LUT
 *
 * DESCRIPTION: Unit test function for the W25N_Read_BBM_LUT function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Read_BBM_LUT();

/*
 * FUNCTION: Test_W25N_Block_Erase_128KB
 *
 * DESCRIPTION: Unit test function for the W25N_Block_Erase_128KB function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Block_Erase_128KB();

/*
 * FUNCTION: Test_W25N_Load_Program_Data
 *
 * DESCRIPTION: Unit test function for the W25N_Load_Program_Data function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Load_Program_Data();

/*
 * FUNCTION: Test_W25N_Program_Execute
 *
 * DESCRIPTION: Unit test function for the W25N_Program_Execute function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Program_Execute();

/*
 * FUNCTION: Test_W25N_Page_Data_Read
 *
 * DESCRIPTION: Unit test function for the W25N_Page_Data_Read function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Page_Data_Read();

/*
 * FUNCTION: Test_W25N_Read_Data
 *
 * DESCRIPTION: Unit test function for the W25N_Read_Data function.
 *
 * NOTES:
 *  - Note1
 *  - Note2
 */
void Test_W25N_Read_Data();

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
