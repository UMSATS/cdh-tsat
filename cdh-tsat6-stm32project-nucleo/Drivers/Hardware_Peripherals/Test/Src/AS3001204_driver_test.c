/*
 * FILENAME: AS3001204_driver_test.c
 *
 * DESCRIPTION: Unit test for the AS3001204-0054X0ISAY MRAM driver
 * 	            (see AS3001204_driver.{c,h})
 *
 * AUTHORS:
 *  - Gabriel Young (gabriel.young@umsats.ca)
 *  - Om Sevak (om.sevak@umsats.ca)
 *
 * Created on: Dec. 6, 2022
 */


#include "../Inc/AS3001204_driver.h"
#include "../Inc/AS3001204_driver_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//###############################################################################################
// Defining test data
//###############################################################################################

//static uint8_t STATUS_REG_DEFAULT = 0x00;
static uint8_t STATUS_REG_TEST    = 0x60;

//static uint8_t CONFIG_REGS_DEFAULT[AS3001204_CONFIG_REGS_LENGTH] = {0x00, 0x00, 0x60, 0x05};
static uint8_t CONFIG_REGS_TEST   [AS3001204_CONFIG_REGS_LENGTH] = {0x05, 0x0f, 0x73, 0x06};

static uint8_t DEVICE_ID[AS3001204_DEVICE_ID_LENGTH] = {0xE6, 0x01, 0x01, 0x02};

//static uint8_t AAP_REG_DEFAULT = 0x00;
static uint8_t AAP_REG_TEST    = 0xff;

static uint32_t MEM_TEST_ADDRESS = 0xabba;
static uint32_t AAP_TEST_ADDRESS = 0x0000;

static char *SAMPLE_DATA = "All human beings are born free and equal in dignity and rights. \
They are endowed with reason and conscience and should act towards one another in a spirit of brotherhood.";


//###############################################################################################
// Read/write register tests
//###############################################################################################

unsigned int AS3001204_Test_RW_Status_Register() {

    HAL_StatusTypeDef isError; 
    uint8_t p_buffer = 0x00;

    AS3001204_Read_Status_Register(&p_buffer);

    p_buffer = 0x00;

    isError = AS3001204_Write_Status_Register(&STATUS_REG_TEST);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Read_Status_Register(&p_buffer);
    if (isError != HAL_OK) goto error;

    if (p_buffer == STATUS_REG_TEST) return 0;

error:
    return 1;
}

unsigned int AS3001204_Test_RW_Config_Registers() {

    HAL_StatusTypeDef isError;
    uint8_t p_buffer[AS3001204_CONFIG_REGS_LENGTH];

    isError = AS3001204_Write_Config_Registers(CONFIG_REGS_TEST);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Read_Config_Registers(p_buffer);
    if (isError != HAL_OK) goto error;

    if (memcmp(p_buffer, CONFIG_REGS_TEST, AS3001204_CONFIG_REGS_LENGTH) == 0) return 0;

error:
    return 1;
    
}


unsigned int AS3001204_Test_RW_Augmented_Array_Protection_Register() {
    
    HAL_StatusTypeDef isError; 
    uint8_t p_buffer = 0x00;

    isError = AS3001204_Write_Augmented_Array_Protection_Register(&AAP_REG_TEST);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Read_Augmented_Array_Protection_Register(&p_buffer);
    if (isError != HAL_OK) goto error;

    if (p_buffer == AAP_REG_TEST) return 0;

error:
    return 1;
    
}


//###############################################################################################
// Read/write memory tests
//###############################################################################################

unsigned int AS3001204_Test_RW_Memory() {
    
    HAL_StatusTypeDef isError;
    char p_buffer[strlen(SAMPLE_DATA)];
    
    isError = AS3001204_Write_Memory((unsigned char *) SAMPLE_DATA, MEM_TEST_ADDRESS, strlen(SAMPLE_DATA));
    if (isError != HAL_OK) goto error;
    
    isError = AS3001204_Read_Memory((uint8_t *) p_buffer, MEM_TEST_ADDRESS, strlen(SAMPLE_DATA));
    if (isError != HAL_OK) goto error;
    
    return strcmp((char *) p_buffer, SAMPLE_DATA);
    
error:
    return 1;
}


unsigned int AS3001204_Test_RW_Augmented_Storage() {
    
    HAL_StatusTypeDef isError;
    char p_buffer[strlen(SAMPLE_DATA)];
    
    isError = AS3001204_Write_Augmented_Storage((unsigned char *) SAMPLE_DATA, AAP_TEST_ADDRESS, strlen(SAMPLE_DATA));
    if (isError != HAL_OK) goto error;
    
    isError = AS3001204_Read_Augmented_Storage((uint8_t *) p_buffer, AAP_TEST_ADDRESS, strlen(SAMPLE_DATA));
    if (isError != HAL_OK) goto error;
    
    return strcmp((char *) p_buffer, SAMPLE_DATA);
    
error:
    return 1;
    
}

unsigned int AS3001204_Test_Read_ID_Registers() {

    HAL_StatusTypeDef isError;
    uint8_t devIDBuffer[AS3001204_DEVICE_ID_LENGTH];
    uint8_t uniIDBuffer[AS3001204_UNIQUE_ID_LENGTH];

    isError = AS3001204_Read_Device_ID(devIDBuffer);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Read_Unique_ID(uniIDBuffer);
    if (isError != HAL_OK) goto error;

    return devIDBuffer != DEVICE_ID;

error:
    return 1;
}


//###############################################################################################
// Complete test suite routine
//###############################################################################################

int AS3001204_Test_Mram_Driver() {
    int numFailed = 0;

    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);

    // numFailed += AS3001204_Test_Read_ID_Registers();

    numFailed += AS3001204_Test_RW_Status_Register();
    // numFailed += AS3001204_Test_RW_Config_Registers();
    // numFailed += AS3001204_Test_RW_Augmented_Array_Protection_Register();
    // numFailed += AS3001204_Test_RW_Memory();
    // numFailed += AS3001204_Test_RW_Augmented_Storage();

    // This function sucks and won't be included right now, see note above
    // numFailed += AS3001204_Test_Write_Disable();


    // Note: no tests currently for any of the following driver functions:
    // (note that Write Enable is tested implicitly through its use in other functions)
    //  AS3001204_Write_Disable();
    //	AS3001204_Enter_Hibernate();
    //	AS3001204_Enter_Deep_Power_Down();
    //	AS3001204_Exit_Deep_Power_Down();
    //	AS3001204_Software_Reset_Enable();
    //	AS3001204_Software_Reset();

    return numFailed;
}





