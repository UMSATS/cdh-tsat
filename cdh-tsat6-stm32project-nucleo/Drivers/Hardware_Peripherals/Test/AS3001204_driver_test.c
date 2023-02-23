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
#include "AS3001204_driver_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//###############################################################################################
// Defining test functions
//###############################################################################################



//########################################
// Read/write register tests
//########################################

/*
 * FUNCTIONS:   AS3001204_Test_RW_Status_Register, AS3001204_Test_RW_Config_Registers,
 * 	            AS3001204_Test_RW_Augmented_Array_Protection_Register
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to read and write to the
 * 	            MRAM unit's status register, configuration registers, and augmented array
 * 	            protection register, respectively. These functions only write valid data to the
 * 	            registers - writing invalid bits to registers could cause undefined behaviour.
 *
 * RETURN:      0 if successful, 1 if not.
 */

unsigned int AS3001204_Test_RW_Status_Register();
unsigned int AS3001204_Test_RW_Config_Registers();
unsigned int AS3001204_Test_RW_Augmented_Array_Protection_Register();


//########################################
// Read/write memory tests
//########################################

/*
 * FUNCTIONS:   AS3001204_Test_RW_Memory, AS3001204_Test_RW_Augmented_Storage
 *
 * DESCRIPTION: These functions test the AS3001204 driver's ability to read and write the
 * 	            main memory array, and the augmented storage section of memory.
 *
 * RETURN:      0 if successful, 1 if not.
 */
unsigned int AS3001204_Test_RW_Memory();
unsigned int AS3001204_Test_RW_Augmented_Storage();

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
	free(p_buffer);
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
	free(p_buffer);
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
	free(p_buffer);
    return 1;
	
}

void AS3001204_Test_Read_ID_Registers() {

	HAL_StatusTypeDef isError;
	// uint8_t *devIDBuffer = malloc(AS3001204_DEVICE_ID_LENGTH);
	// uint8_t *uniIDBuffer = malloc(AS3001204_UNIQUE_ID_LENGTH);
	uint8_t devIDBuffer[AS3001204_DEVICE_ID_LENGTH];
	uint8_t uniIDBuffer[AS3001204_UNIQUE_ID_LENGTH];

	isError = AS3001204_Read_Device_ID(devIDBuffer);
	if (isError != HAL_OK) goto error;

	isError = AS3001204_Read_Unique_ID(uniIDBuffer);
	if (isError != HAL_OK) goto error;

	int result = memcmp(devIDBuffer, AS3001204_DEVICE_ID, AS3001204_DEVICE_ID_LENGTH);
	//printf("Device ID: %hhn\nUnique ID: %hhn\n", devIDBuffer, uniIDBuffer);

error:
	free(devIDBuffer);
	free(uniIDBuffer);
	printf("Failed to read device/unique IDs\n");
}

unsigned int AS3001204_Test_Write_Disable(){
	
	HAL_StatusTypeDef isError;
	
	isError = AS3001204_Write_Disable();
	if (isError != HAL_OK) goto error;
	return 0;

error:
	printf("Failed to write disable\n");
	return 1;
}

unsigned int AS3001204_Test_Software_Reset_Enable(){
	
	HAL_StatusTypeDef isError;
	
	isError = AS3001204_Software_Reset_Enable();
	if (isError != HAL_OK) goto error;
	return 0;

error:
	printf("Failed to enable software reset\n");
	return 1;
}

unsigned int AS3001204_Test_Software_Reset(){
	
}

//###############################################################################################
// Testing routine
//###############################################################################################


int AS3001204_Test_Mram_Driver(){
		int numFailed = 0;

	AS3001204_Test_Read_ID_Registers();
//
//	numFailed += AS3001204_Test_RW_Status_Register();
//	numFailed += AS3001204_Test_RW_Config_Registers();
//	numFailed += AS3001204_Test_RW_Augmented_Array_Protection_Register();
//	numFailed += AS3001204_Test_RW_Memory();
//	numFailed += AS3001204_Test_RW_Augmented_Storage();
//	numFailed += AS3001204_Test_Write_Disable();

	// Functions not tested in the above (note WREN is indirectly tested)

//	AS3001204_Enter_Hibernate();
//	AS3001204_Enter_Deep_Power_Down();
//	AS3001204_Exit_Deep_Power_Down();
//	AS3001204_Software_Reset_Enable();
//	AS3001204_Software_Reset();

	return numFailed;
}





