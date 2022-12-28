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

#include "W25N_driver.h"
#include "W25N_driver_test.h"


//###############################################################################################
//Unit Test Functions
//###############################################################################################
void Test_W25N_Device_Reset()
{

}

void Test_W25N_Read_JEDEC_ID()
{

}

void Test_W25N_Read_Status_Register()
{

}

void Test_W25N_Write_Status_Register()
{

}

void Test_W25N_Write_Enable()
{

}

void Test_W25N_Write_Disable()
{

}

void Test_W25N_Bad_Block_Management()
{

}

void Test_W25N_Read_BBM_LUT()
{

}

void Test_W25N_Block_Erase_128KB()
{

}

void Test_W25N_Load_Program_Data()
{

}

void Test_W25N_Program_Execute()
{

}

void Test_W25N_Page_Data_Read()
{

}

void Test_W25N_Read_Data()
{

}

//###############################################################################################
//Complete Unit Test Function
//###############################################################################################
void Unit_Test_W25N()
{
    Test_W25N_Device_Reset();
    Test_W25N_Read_JEDEC_ID();
    Test_W25N_Read_Status_Register();
    Test_W25N_Write_Status_Register();
    Test_W25N_Write_Enable();
    Test_W25N_Write_Disable();
    Test_W25N_Bad_Block_Management();
    Test_W25N_Read_BBM_LUT();
    Test_W25N_Block_Erase_128KB();
    Test_W25N_Load_Program_Data();
    Test_W25N_Program_Execute();
    Test_W25N_Page_Data_Read();
    Test_W25N_Read_Data();
}
