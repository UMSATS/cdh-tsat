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

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "W25N_driver.h"
#include "W25N_driver_test.h"

//###############################################################################################
//Driver Function Prototypes
//###############################################################################################
W25N_StatusTypeDef W25N_Read_Status_Register(uint8_t register_address, uint8_t *p_buffer);
W25N_StatusTypeDef W25N_Write_Status_Register(uint8_t register_address, uint8_t register_value);
W25N_StatusTypeDef W25N_Write_Enable();
W25N_StatusTypeDef W25N_Write_Disable();
W25N_StatusTypeDef W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address);
W25N_StatusTypeDef W25N_Block_Erase_128KB(uint16_t page_address);
W25N_StatusTypeDef W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);
W25N_StatusTypeDef W25N_Program_Execute(uint16_t page_address);
W25N_StatusTypeDef W25N_Page_Data_Read(uint16_t page_address);
W25N_StatusTypeDef W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);

//###############################################################################################
//High-Level Driver Function Prototypes
//###############################################################################################
W25N_StatusTypeDef W25N_Wait_Until_Not_Busy();

//###############################################################################################
//Public Unit Test Functions
//###############################################################################################
W25N_StatusTypeDef Test_W25N_Device_Reset()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_default_contents = 0b00011000;
    uint8_t register_written_contents = 0b01011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Write_Status_Register(register_address, register_written_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Device_Reset();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    assert((register_contents & 0b11111000) == register_default_contents);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Read_JEDEC_ID()
{
    W25N_StatusTypeDef operation_status;
    uint8_t test_array[3];
    uint8_t test_array_size = sizeof(test_array) / sizeof(test_array[0]);
    uint8_t jedec_id[3] = {0xEF, 0xAA, 0x21};

    operation_status = W25N_Read_JEDEC_ID(test_array);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(test_array, jedec_id, test_array_size) == 0);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Read_Status_Register()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_default_contents = 0b00011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    assert((register_contents & 0b11111000) == register_default_contents);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Write_Status_Register()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xB0;
    uint8_t register_written_contents = 0b01011000;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Write_Status_Register(register_address, register_written_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    assert((register_contents & 0b11111000) == register_written_contents);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Write_Enable()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    assert((register_contents & 0b00000010) == 0b00000010);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Write_Disable()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Write_Disable();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;

    assert((register_contents & 0b00000010) == 0b00000000);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Load_Program_Data()
{
    W25N_StatusTypeDef operation_status;
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
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Load_Program_Data(data_array, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Data(data_buffer_contents, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(data_array, data_buffer_contents, data_array_size) == 0);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Execute_Erase()
{
    W25N_StatusTypeDef operation_status;
    uint16_t column_address = 0x0000;
    uint16_t page_address = 0xFFFF;
    uint8_t erased_data_array[2048]; //entire array will be initialized to 0xFF
    uint8_t data_array[2048];
    uint16_t data_array_size = sizeof(data_array) / sizeof(data_array[0]);
    uint8_t data_buffer_contents[2048];
    
    for (int i = 0; i < data_array_size; i++)
    {
        erased_data_array[i] = 0xFF;
    }

    uint8_t counter = 0xFF;
    for (int i = 0; i < data_array_size; i++)
    {
        data_array[i] = counter;
        counter--;
    }

    //test inital erase
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Page_Data_Read(page_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Read_Data(data_buffer_contents, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(erased_data_array, data_buffer_contents, data_array_size) == 0);

    //test program execute
    operation_status = W25N_Load_Program_Data(data_array, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Program_Execute(page_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Page_Data_Read(page_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Read_Data(data_buffer_contents, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(data_array, data_buffer_contents, data_array_size) == 0);

    //test erasing the newly written data
    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Page_Data_Read(page_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Read_Data(data_buffer_contents, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(erased_data_array, data_buffer_contents, data_array_size) == 0);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_Read()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_address = 0xB0;
    uint8_t register_value_OTP_enter = 0b01011000;
    uint8_t register_value_OTP_exit = 0b00011000;
    uint16_t column_address = 0x0000;
    uint16_t parameter_page_address = 0x0001;
    uint8_t data_buffer_contents[2048];
    uint16_t data_buffer_size = sizeof(data_buffer_contents) / sizeof(data_buffer_contents[0]);

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
    if (operation_status != W25N_HAL_OK) goto error;

    //enter OTP access mode to read parameter page
    operation_status = W25N_Write_Status_Register(register_address, register_value_OTP_enter);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Page_Data_Read(parameter_page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Read_Data(data_buffer_contents, column_address, data_buffer_size);
    if (operation_status != W25N_HAL_OK) goto error;

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

error:
    return operation_status;
}

//###############################################################################################
//Public High-Level Unit Test Functions
//###############################################################################################
W25N_StatusTypeDef Test_W25N_High_Level_Read()
{
    W25N_StatusTypeDef operation_status;
    uint16_t column_address = 0x0000;
    uint16_t page_address = 0xFFFF;
    uint8_t data_array[2048];
    uint16_t data_array_size = sizeof(data_array) / sizeof(data_array[0]);
    uint8_t data_buffer_contents[2048];

    uint8_t counter = 0x00;
    for (int i = 0; i < data_array_size; i++)
    {
        data_array[i] = counter;
        counter += 2;
    }
    
    //erase the test page
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    //write test data to the test page
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Load_Program_Data(data_array, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Program_Execute(page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    //read test data from the test page
    operation_status = W25N_Read(data_buffer_contents, page_address, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(data_array, data_buffer_contents, data_array_size) == 0);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_High_Level_Write()
{
    W25N_StatusTypeDef operation_status;
    uint16_t column_address = 0x0000;
    uint16_t page_address = 0xFFFF;
    uint8_t data_array[2048];
    uint16_t data_array_size = sizeof(data_array) / sizeof(data_array[0]);
    uint8_t data_buffer_contents[2048];

    uint8_t counter = 0x00;
    for (int i = 0; i < data_array_size; i++)
    {
        data_array[i] = counter;
        counter -= 2;
    }
    
    //erase the test page
    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    //write test data to the test page
    operation_status = W25N_Write(data_array, page_address, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    //read test data from the test page
    operation_status = W25N_Read(data_buffer_contents, page_address, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(data_array, data_buffer_contents, data_array_size) == 0);

error:
    return operation_status;
}

W25N_StatusTypeDef Test_W25N_High_Level_Erase()
{
    W25N_StatusTypeDef operation_status;
    uint16_t column_address = 0x0000;
    uint16_t page_address = 0xFFFF;
    uint8_t erased_data_array[2048]; //entire array will be initialized to 0xFF
    uint8_t data_array[2048];
    uint16_t data_array_size = sizeof(data_array) / sizeof(data_array[0]);
    uint8_t data_buffer_contents[2048];

    for (int i = 0; i < data_array_size; i++)
    {
        erased_data_array[i] = 0xFF;
    }

    uint8_t counter = 0x00;
    for (int i = 0; i < data_array_size; i++)
    {
        data_array[i] = counter;
        counter += 3;
    }
    
    //erase the test page
    operation_status = W25N_Erase(page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    //write test data to the test page
    operation_status = W25N_Write(data_array, page_address, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    //erase the test page
    operation_status = W25N_Erase(page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    //read test data from the test page
    operation_status = W25N_Read(data_buffer_contents, page_address, column_address, data_array_size);
    if (operation_status != W25N_HAL_OK) goto error;

    assert(memcmp(erased_data_array, data_buffer_contents, data_array_size) == 0);

error:
    return operation_status;
}

//###############################################################################################
//Public Complete Unit Test Function
//###############################################################################################
W25N_StatusTypeDef Test_W25N()
{
    W25N_StatusTypeDef operation_status;

    //set the write protect pin low to enable writing
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);

    //the following functions are executed in order of dependencies
    operation_status = Test_W25N_Read_JEDEC_ID();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Read_Status_Register();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Write_Status_Register();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Write_Disable();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Device_Reset();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Read();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Load_Program_Data();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_Execute_Erase();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = Test_W25N_High_Level_Read();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_High_Level_Write();
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = Test_W25N_High_Level_Erase();
    if (operation_status != W25N_HAL_OK) goto error;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);

    return W25N_HAL_OK;

error:
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    return operation_status;
}
