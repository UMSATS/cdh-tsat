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
HAL_StatusTypeDef Test_W25N_Device_Reset()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_default_contents = 0b00011000;
    uint8_t register_written_contents = 0b01011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Write_Status_Register(register_address, register_written_contents);
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Device_Reset();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) return operation_status;

    assert((register_contents & 0b11111000) == register_default_contents);
}

HAL_StatusTypeDef Test_W25N_Read_JEDEC_ID()
{
    HAL_StatusTypeDef operation_status;
    uint8_t test_array[3];
    uint8_t test_array_size = sizeof(test_array) / sizeof(test_array[0]);
    uint8_t jedec_id[3] = {0xEF, 0xAA, 0x21};

    operation_status = W25N_Read_JEDEC_ID(test_array);
    if (operation_status != HAL_OK) return operation_status;

    assert(memcmp(test_array, jedec_id, test_array_size) == 0);
}

HAL_StatusTypeDef Test_W25N_Read_Status_Register()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_default_contents = 0b00011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) return operation_status;

    assert((register_contents & 0b11111000) == register_default_contents);
}

HAL_StatusTypeDef Test_W25N_Write_Status_Register()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_written_contents = 0b01011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Write_Status_Register(register_address, register_written_contents);
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) return operation_status;

    assert((register_contents & 0b11111000) == register_written_contents);
}

HAL_StatusTypeDef Test_W25N_Write_Enable()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Write_Enable();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) return operation_status;

    assert((register_contents & 0b00000010) == 0b00000010);
}

HAL_StatusTypeDef Test_W25N_Write_Disable()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Write_Disable();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != HAL_OK) return operation_status;

    assert((register_contents & 0b00000010) == 0b00000000);
}

HAL_StatusTypeDef Test_W25N_Load_Program_Data()
{
    HAL_StatusTypeDef operation_status;
    uint16_t column_address = 0x0000;
    uint8_t data_array[2048];
    uint16_t data_array_size = sizeof(data_array) / sizeof(data_array[0]);
    uint8_t data_buffer_contents[2048];

    uint8_t counter = 0x00;
    for (int i = 0; i < data_array_size; i++)
    {
        data_array[i] = counter;
        counter++;
    }
    
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Load_Program_Data(&data_array, column_address, data_array_size);
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Data(&data_buffer_contents, column_address, data_array_size);
    if (operation_status != HAL_OK) return operation_status;

    assert(memcmp(data_array, data_buffer_contents, data_array_size) == 0);
}

HAL_StatusTypeDef Test_W25N_Execute_Erase()
{
    HAL_StatusTypeDef operation_status;
    uint16_t column_address = 0x0000;
    uint16_t page_address = 0xFFFF
    uint8_t zero_data_array[2048] = {0x00}; //entire array will be initialized to 0x00
    uint8_t data_array[2048];
    uint16_t data_array_size = sizeof(data_array) / sizeof(data_array[0]);
    uint8_t data_buffer_contents[2048];
    
    uint8_t counter = 0xFF;
    for (int i = 0; i < data_array_size; i++)
    {
        data_array[i] = counter;
        counter--;
    }

    //test inital erase
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Page_Data_Read(page_address);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Read_Data(&data_buffer_contents, column_address, data_array_size);
    if (operation_status != HAL_OK) return operation_status;

    assert(memcmp(zero_data_array, data_buffer_contents, data_array_size) == 0);

    //test program execute
    operation_status = W25N_Load_Program_Data(&data_array, column_address, data_array_size);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Program_Execute(page_address);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Page_Data_Read(page_address);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Read_Data(&data_buffer_contents, column_address, data_array_size);
    if (operation_status != HAL_OK) return operation_status;

    assert(memcmp(data_array, data_buffer_contents, data_array_size) == 0);

    //test erasing the newly written data
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Page_Data_Read(page_address);
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;
    operation_status = W25N_Read_Data(&data_buffer_contents, column_address, data_array_size);
    if (operation_status != HAL_OK) return operation_status;

    assert(memcmp(zero_data_array, data_buffer_contents, data_array_size) == 0);
}

HAL_StatusTypeDef Test_W25N_Read()
{
    HAL_StatusTypeDef operation_status;
    uint8_t register_address = 0xB0;
    uint8_t register_value_OTP_enter = 0b01011000;
    uint8_t register_value_OTP_exit = 0b00011000;
    uint16_t column_address = 0x0000;
    uint16_t parameter_page_address = 0x0001
    uint16_t data_buffer_size = sizeof(data_buffer_size) / sizeof(data_buffer_size[0]);
    uint8_t data_buffer_contents[2048];

    //the OTP parameter page is defined in the datasheet
    //since the data on this page is known, it will be used to verify the read functions
    uint8_t parameter_data_array[254] =
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
    uint8_t zero_data_array[1280] = {0x00}; //entire array will be initialized to 0x00
    
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    //enter OTP access mode to read parameter page
    operation_status = W25N_Write_Status_Register(register_address, register_value_OTP_enter);
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Page_Data_Read(parameter_page_address);
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != HAL_OK) return operation_status;

    operation_status = W25N_Read_Data(&data_buffer_contents, column_address, data_buffer_size);
    if (operation_status != HAL_OK) return operation_status;

    //compare bytes 0-253 to parameter data
    assert(memcmp(parameter_data_array, data_buffer_contents, 254) == 0);
    //compare bytes 256-509 to parameter data
    assert(memcmp(parameter_data_array, data_buffer_contents + 256, 254) == 0);
    //compare bytes 512-765 to parameter data
    assert(memcmp(parameter_data_array, data_buffer_contents + 512, 254) == 0);
    //compare bytes 768-2047 to zero data
    assert(memcmp(zero_data_array, data_buffer_contents + 768, 1280) == 0);

    //exit OTP access mode to return to the main memory array
    operation_status = W25N_Write_Status_Register(register_address, register_value_OTP_exit);
    if (operation_status != HAL_OK) return operation_status;
}

//you need to write enable before each write operation

//you need to init before unit test
//should be done in main though, not in unit test function since different things

//###############################################################################################
//Complete Unit Test Function
//###############################################################################################
void Test_W25N()
{
    HAL_StatusTypeDef operation_status;

    //DON'T FORGET ABOUT PRE-INITIALIZATION BEFORE DOING THESE UNIT TESTS
    //(NOT IN THIS UNIT TEST CODE, ONLY HAS TO BE DONE ONCE AFTER ALL)
    //YOU HAVE TO SCAN THE WHOLE THING FOR BAD BLOCKS
    //ALSO CHECK HOW ERASE FUNCTION WORKS BEFORE YOU DO THESE UNIT TESTS

    //set the write protect pin low to enable writing
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);

    //the following functions are executed in order of dependencies
    operation_status = Test_W25N_Read_JEDEC_ID();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Read_Status_Register();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Write_Status_Register();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Write_Enable();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Write_Disable();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Device_Reset();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Read();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Load_Program_Data();
    if (operation_status != HAL_OK) goto error;
    operation_status = Test_W25N_Execute_Erase();
    if (operation_status != HAL_OK) goto error;

error:
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    exit(1);
}
