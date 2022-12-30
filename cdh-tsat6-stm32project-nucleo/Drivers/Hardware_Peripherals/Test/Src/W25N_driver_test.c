/*
 * FILENAME: W25N_driver_test.c
 *
 * DESCRIPTION: Unit test source file for the W25N01GVZEIG NAND Flash STM32L4 driver.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * CREATED ON: Dec. 28, 2022
 */

#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "W25N_driver.h"
#include "W25N_driver_test.h"


//###############################################################################################
//Unit Test Functions
//###############################################################################################
void Test_W25N_Device_Reset()
{
    //depends on passing Test_W25N_Write_Status_Register
    //depends on passing Test_W25N_Read_Status_Register
}

//no dependencies
void Test_W25N_Read_JEDEC_ID()
{
    uint8_t test_array[3];
    uint8_t jedec_id[3] = {0xEF, 0xAA, 0x21};

    W25N_Read_JEDEC_ID(test_array);

    assert(!memcmp(test_array, jedec_id, 3));
}

void Test_W25N_Read_Status_Register()
{
    //must be done after power cycling the chip
    //must be done before Test_W25N_Write_Status_Register

    //compare to known state of one of the status registers after power cycle
}

void Test_W25N_Write_Status_Register()
{
    //depends on passing Test_W25N_Read_Status_Register
}

void Test_W25N_Write_Enable()
{
    //depends on passing Test_W25N_Read_Status_Register
}

void Test_W25N_Write_Disable()
{
    //depends on passing Test_W25N_Read_Status_Register
}

void Test_W25N_Block_Erase_128KB()
{
    //depends on passing Test_W25N_Load_Program_Data
    //depends on passing Test_W25N_Program_Execute
    //depends on passing Test_W25N_Read

    //shouldn't read data from previous tests since each test should be independent
    //this test will call its own read and write functions
}

void Test_W25N_Load_Program_Data()
{
    //depends on passing Test_W25N_Read
}

void Test_W25N_Program_Execute()
{
    //depends on passing Test_W25N_Load_Program_Data
    //depends on passing Test_W25N_Read

    //will have to read a different page in between to clear the data buffer
}

void Test_W25N_Read()
{
    //depends on passing Test_W25N_Write_Status_Register (to access read-only pages)
    //tests both W25N_Page_Data_Read & W25N_Read_Data since they are interdependent

    //using read-only parameter page data bc it's defined in the datasheet
    //THIS DATA HAS BEEN CHECKED, DON'T NEED TO DOUBLE CHECK IT
    uint8_t parameter_data[254] =
        {0x4F, 0x4E, 0x46, 0x49, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x57, 0x49, 0x4E, 0x42, 0x4F, 0x4E, 0x44, 0x20, 0x20, 0x20,
         0x20, 0x20, 0x57, 0x32, 0x35, 0x4E, 0x30, 0x31, 0x47, 0x56, 0x20, 0x20, 0x20, 0x20,
         0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
         0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x04,
         0x00, 0x00, 0x01, 0x00, 0x01, 0x14, 0x00, 0x01, 0x05, 0x01, 0x00, 0x00, 0x04, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x02, 0x10, 0x27, 0x32, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
         0x00, 0x00};
    //check first 254 bytes against data
    //ignore next 2 bytes
    //check next 254 bytes against data
    //ignore next 2 bytes
    //check next 254 bytes against data
    //ignore next 2 bytes
    //check remaining 1280 bytes against 0x00

    //note we check the 2048 data bytes, we don't worry about the ECC bytes
}

//###############################################################################################
//Complete Unit Test Function
//###############################################################################################
void Unit_Test_W25N()
{
    //DON'T FORGET ABOUT PRE-INITIALIZATION BEFORE DOING THESE UNIT TESTS
    //(NOT IN THIS UNIT TEST CODE, ONLY HAS TO BE DONE ONCE AFTER ALL)
    //YOU HAVE TO SCAN THE WHOLE THING FOR BAD BLOCKS
    //Function "W25N_One_Time_Init()" will be written in W25N higher-level driver functions
    //ALSO CHECK HOW ERASE FUNCTION WORKS BEFORE YOU DO THESE UNIT TESTS

    //the following functions are executed in order of dependencies
    Test_W25N_Read_JEDEC_ID();
    Test_W25N_Read_Status_Register();
    Test_W25N_Write_Status_Register();
    Test_W25N_Write_Enable();
    Test_W25N_Write_Disable();
    Test_W25N_Device_Reset();
    Test_W25N_Read();
    Test_W25N_Load_Program_Data();
    Test_W25N_Program_Execute();
    Test_W25N_Block_Erase_128KB();
}
