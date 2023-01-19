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
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_default_contents = 0b00011000;
    uint8_t register_written_contents = 0b01011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Write_Status_Register(register_address, register_written_contents);
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Device_Reset();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) exit(1);

    assert((register_contents & 0b11111000) == register_default_contents);
}

void Test_W25N_Read_JEDEC_ID()
{
    HAL_StatusTypeDef operation_status;
    uint8_t test_array[3];
    uint8_t jedec_id[3] = {0xEF, 0xAA, 0x21};

    operation_status = W25N_Read_JEDEC_ID(test_array);
    if (operation_status != HAL_OK) exit(1);

    assert(memcmp(test_array, jedec_id, 3) == 0);
}

void Test_W25N_Read_Status_Register()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_default_contents = 0b00011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) exit(1);

    assert((register_contents & 0b11111000) == register_default_contents);
}

void Test_W25N_Write_Status_Register()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_written_contents = 0b01011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Write_Status_Register(register_address, register_written_contents);
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) exit(1);

    assert((register_contents & 0b11111000) == register_written_contents);
}

void Test_W25N_Write_Enable()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Write_Enable();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) exit(1);

    assert((register_contents & 0b00000010) == 0b00000010);
}

void Test_W25N_Write_Disable()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Write_Disable();
    if (operation_status != HAL_OK) exit(1);

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) exit(1);

    assert((register_contents & 0b00000010) == 0b00000000);
}

void Test_W25N_Block_Erase_128KB()
{
    //shouldn't read data from previous tests since each test should be independent
    //this test will call its own read and write functions
}

void Test_W25N_Load_Program_Data()
{
    
}

void Test_W25N_Program_Execute()
{
    //will have to read a different page in between to clear the data buffer
}

void Test_W25N_Read()
{
    //must write to status register to access read-only pages
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
