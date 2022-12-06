/*
 * FILENAME: AS3001204_driver_test.c
 *
 * DESCRIPTION: Unit test for the AS3001204-0054X0ISAY MRAM driver
 * 	            (see AS3001204_driver.{c,h})
 *
 * AUTHORS:
 *  - Gabriel Young (gabriel.young@umsats.ca)
 *
 * Created on: Dec. 6, 2022
 */


#include "../Inc/AS3001204_driver.h"
#include <stdlib.h>
#include <string.h>

//###############################################################################################
// Defining test data
//###############################################################################################
//static uint8_t STATUS_REG_DEFAULT = 0x00;
static uint8_t STATUS_REG_TEST    = 0x7c;

//static uint8_t CONFIG_REGS_DEFAULT[AS3001204_CONFIG_REGS_LENGTH] = {0x00, 0x00, 0x60, 0x05};
static uint8_t CONFIG_REGS_TEST[AS3001204_CONFIG_REGS_LENGTH] = {0x05, 0x0f, 0x73, 0x06};

//static uint8_t AAP_REG_DEFAULT = 0x00;
static uint8_t AAP_REG_TEST    = 0xff;

static const char *SAMPLE_DATA = "All human beings are born free and equal in dignity and rights. \
They are endowed with reason and conscience and should act towards one another in a spirit of brotherhood.";


//###############################################################################################
// Read/write register tests
//###############################################################################################

unsigned int AS3001204_Test_RW_Status_Register() {

    HAL_StatusTypeDef isError; 
    uint8_t p_buffer = 0x00;

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
    uint8_t *p_buffer = malloc(AS3001204_CONFIG_REGS_LENGTH);

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
	uint8_t *p_buffer = malloc(sizeof SAMPLE_DATA);
	
	isError = AS3001204_Write_Memory((uint8_t *) SAMPLE_DATA, 0xabba, sizeof SAMPLE_DATA);
	if (isError != HAL_OK) goto error;
	
	isError = AS3001204_Read_Memory(p_buffer, 0xabba, sizeof SAMPLE_DATA);
	if (isError != HAL_OK) goto error;
	
	if (memcmp(p_buffer, SAMPLE_DATA, strlen(SAMPLE_DATA)) == 0) return 0;
	
error:
    return 1;
}


unsigned int AS3001204_Test_RW_Augmented_Storage() {
	
	HAL_StatusTypeDef isError;
	uint8_t *p_buffer = malloc(sizeof SAMPLE_DATA);
	
	isError = AS3001204_Write_Augmented_Storage((uint8_t *) &SAMPLE_DATA, 0x00, sizeof SAMPLE_DATA);
	if (isError != HAL_OK) goto error;
	
	isError = AS3001204_Read_Augmented_Storage(p_buffer, 0x00, sizeof SAMPLE_DATA);
	if (isError != HAL_OK) goto error;
	
	if (memcmp(p_buffer, SAMPLE_DATA, strlen(SAMPLE_DATA) == 0)) return 0;
	
error:
    return 1;
	
}
