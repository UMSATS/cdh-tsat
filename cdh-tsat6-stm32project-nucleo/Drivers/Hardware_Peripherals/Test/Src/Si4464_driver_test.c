/*
 * FILENAME: Si4464_driver_test.c
 *
 * DESCRIPTION: Unit test source file for the Si4464 RF Chip STM32L4 driver.
 *
 * AUTHORS:
 *  - Nikolaus Reichert (nikolaus.reichert@umsats.ca)
 *
 * CREATED ON: Mar. 14, 2023
 */

#include "Si4464_driver_test.h"
#include "Si4464_driver.h"
#include "Si4464_command_codes.h"

// TODO: Move this to a new file, alongside all other props. -NJR
#define SI4464_MODEM_GROUP 0x20
#define SI4464_MODEM_MOD_TYPE 0x01

//###############################################################################################
//Public Unit Test Functions
//###############################################################################################
HAL_StatusTypeDef Test_Si4464_Reset_Device(){
	HAL_StatusTypeDef operation_status = HAL_OK;
	uint8_t original_value = 0x73; // Random bit pattern to know if we read stuff correctly. -NJR
	uint8_t new_value = 0xAC;
	uint8_t post_reset_value = 0x73;

	Si4464_Reset_Device();

	operation_status = Si4464_Get_Prop(SI4464_MODEM_GROUP,  1,  SI4464_MODEM_MOD_TYPE, &original_value);
	if (operation_status != HAL_OK) goto error;

	operation_status = Si4464_Set_One_Prop(SI4464_MODEM_GROUP, SI4464_MODEM_MOD_TYPE, 0xAC);
	if (operation_status != HAL_OK) goto error;

	Si4464_Reset_Device();

	operation_status = Si4464_Get_Prop(SI4464_MODEM_GROUP,  1,  SI4464_MODEM_MOD_TYPE, &post_reset_value);
	if (operation_status != HAL_OK) goto error;

	if (post_reset_value != original_value) {
		operation_status = HAL_ERROR;
	}

error:
	return operation_status;
}

HAL_StatusTypeDef Test_Si4464_Get_CTS(){
	// Modified Send_Command code, with hardcoded command and stripped out reply section.
	HAL_StatusTypeDef operation_status = HAL_OK;
	Si4464_Nsel(0);

	uint8_t command = Si4464_NOP;

	operation_status = Radio_SPI_Transmit_Message(&command, 1);
	if (operation_status != HAL_OK) goto error;

	uint32_t retries = 0;

	uint32_t cts = 0;
	do {
		retries++;
		Si4464_Nsel(1);
		Si4464_Nsel(0);
		cts = Si4464_Get_CTS();
	} while (!cts && retries < 1000);

	if (retries >= 1000) {
		operation_status = HAL_ERROR;
	}

error:
	Si4464_Nsel(1);
	return operation_status;
}

HAL_StatusTypeDef Test_Si4464_Get_Set_Props(){
	HAL_StatusTypeDef operation_status = HAL_OK;

	uint8_t original_data[MAX_PROP_WRITE_NUM] = {0x00};
	// Fill buffer with indicator that we aren't able to read stuff.

	for (size_t i = 0; i < MAX_PROP_WRITE_NUM; i++) {
		original_data[i] = 0x73;
	}

	uint8_t data_to_write[MAX_PROP_WRITE_NUM] = {0x00};

	uint8_t after_write_data[MAX_PROP_WRITE_NUM] = {0x00};

	for (size_t i = 0; i < MAX_PROP_WRITE_NUM; i++) {
		after_write_data[i] = 0xAC;
	}



error:
	return operation_status;
}


//###############################################################################################
//Public Complete Unit Test Function
//###############################################################################################
HAL_StatusTypeDef Test_Si4464()
{
    HAL_StatusTypeDef operation_status;

    //the following functions are executed in order of dependencies

    //unit test functions
    operation_status = Test_Si4464_Get_CTS();
    if (operation_status != HAL_OK) goto error;

    operation_status = Test_Si4464_Get_Set_Props();
    if (operation_status != HAL_OK) goto error;

    operation_status = Test_Si4464_Reset_Device();
    if (operation_status != HAL_OK) goto error;

    operation_status = Test_Si4464_

error:
    return operation_status;
}
