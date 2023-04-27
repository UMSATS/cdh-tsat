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
#include "Si4464_command_codes.h"
#include "Si4464_driver_config.h"

#include <string.h>

// TODO: Move this to a new file, alongside all other props. -NJR
#define SI4464_MODEM_GROUP 0x20
#define SI4464_MODEM_CHFLT_GROUP 0x21

#define SI4464_MODEM_MOD_TYPE 0x01

#define SI4464_MAX_PROP_WRITE_NUM 12

#define SI4464_MODEM_CHFLT_RX1_CHFLT_COE 0x00

#define SI4464_NUM_FIFO_SRC_BYTES 12

static uint8_t POWER_UP_ARRAY[] = RADIO_CONFIGURATION_DATA_ARRAY;

//###############################################################################################
//Public Unit Test Functions
//###############################################################################################
Si4464_StatusTypeDef Test_Si4464_Reset_Device(){
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	uint8_t original_value = 0x73; // Random bit pattern to know if we read stuff correctly. -NJR
	uint8_t post_reset_value = 0x73;

	// Reset first, grab the data, write, reset, then read. original_value and post_reset_value should
	// be the same. -NJR
	Si4464_Reset_Device();

	Si4464_Init_Device();

	operation_status = Si4464_Get_Prop(SI4464_MODEM_GROUP,  1,  SI4464_MODEM_MOD_TYPE, &original_value);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Si4464_Set_One_Prop(SI4464_MODEM_GROUP, SI4464_MODEM_MOD_TYPE, 0xAC);
	if (operation_status != SI4464_HAL_OK) goto error;

	Si4464_Reset_Device();

	Si4464_Init_Device();

	operation_status = Si4464_Get_Prop(SI4464_MODEM_GROUP,  1,  SI4464_MODEM_MOD_TYPE, &post_reset_value);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (post_reset_value != original_value) {
		operation_status = SI4464_HAL_ERROR;
	}

error:
	return operation_status;
}

Si4464_StatusTypeDef Test_Si4464_Init_Device() {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	// Just check if it works right now.

	operation_status = Si4464_Init_Device();
	if (operation_status != SI4464_HAL_OK) goto error;

	// TODO: Assert that some registers are set to their default value or something.

error:
	return operation_status;
}

Si4464_StatusTypeDef Test_Si4464_Get_CTS() {
	// Modified Send_Command code, with hardcoded command and stripped out reply section. -NJR
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	Si4464_Nsel(0);

	uint8_t command = SI4464_NOP;

	operation_status = Radio_SPI_Transmit_Message(&command, 1);
	if (operation_status != SI4464_HAL_OK) goto error;

	uint32_t retries = 0;

	uint32_t cts = 0;
	do {

		retries++;
		Si4464_Nsel(1);
		Si4464_Nsel(0);
		cts = Si4464_Get_CTS();
	} while (!cts && retries < 1000);

	if (retries >= 1000) {
		operation_status = SI4464_HAL_ERROR;
	}

error:
	Si4464_Nsel(1);
	return operation_status;
}

Si4464_StatusTypeDef Test_Si4464_Get_Set_Props(){
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	uint8_t original_data[SI4464_MAX_PROP_WRITE_NUM] = {0x00};
	uint8_t data_to_write[SI4464_MAX_PROP_WRITE_NUM] = {0x00};
	uint8_t after_write_data[SI4464_MAX_PROP_WRITE_NUM] = {0x00};
	// Fill buffer with indicator that we aren't able to read stuff.

	for (size_t i = 0; i < SI4464_MAX_PROP_WRITE_NUM; i++) {
		original_data[i] = 0x73;
	}



	Si4464_Execute_Command_Stream(POWER_UP_ARRAY, sizeof(POWER_UP_ARRAY));


	operation_status = Si4464_Get_Prop(SI4464_MODEM_CHFLT_GROUP, 12, SI4464_MODEM_CHFLT_RX1_CHFLT_COE, original_data);
	if (operation_status != SI4464_HAL_OK) goto error;

	// Worst case scenario, we flip all bits in each byte of the property.
	for (size_t i = 0; i < SI4464_MAX_PROP_WRITE_NUM; i++) {
		data_to_write[i] = ~original_data[i];
	}

	operation_status = Si4464_Set_Props(SI4464_MODEM_CHFLT_GROUP, 12, SI4464_MODEM_CHFLT_RX1_CHFLT_COE, data_to_write);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Si4464_Get_Prop(SI4464_MODEM_CHFLT_GROUP, 12, SI4464_MODEM_CHFLT_RX1_CHFLT_COE, after_write_data);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (memcmp(data_to_write, after_write_data, SI4464_MAX_PROP_WRITE_NUM) != 0) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

error:
	return operation_status;
}

Si4464_StatusTypeDef Test_Si4464_Set_One_Prop() {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;

	uint8_t original_value = 0x73;
	uint8_t data_to_write;
	uint8_t after_write_data;

	operation_status = Si4464_Get_Prop(SI4464_MODEM_CHFLT_GROUP, 1, SI4464_MODEM_CHFLT_RX1_CHFLT_COE, &original_value);
	if (operation_status != SI4464_HAL_OK) goto error;

	data_to_write = ~original_value;

	operation_status = Si4464_Set_One_Prop(SI4464_MODEM_CHFLT_GROUP, 1, data_to_write);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Si4464_Get_Prop(SI4464_MODEM_CHFLT_GROUP, 1, SI4464_MODEM_CHFLT_RX1_CHFLT_COE, &after_write_data);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (after_write_data != data_to_write) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

error:
	return operation_status;
}

Si4464_StatusTypeDef Test_Si4464_Send_Get_TX_FIFO() {
	Si4464_StatusTypeDef operation_status = SI4464_HAL_OK;
	uint8_t fifo_src[SI4464_NUM_FIFO_SRC_BYTES] = {0};
	size_t num_sent = 123567890; // Dummy value.

	size_t num_reported_pre_send = 1234567890;
	size_t num_reported_post_send = 1234567890;

	for (size_t i = 0; i < SI4464_NUM_FIFO_SRC_BYTES; i++) {
		fifo_src[i] = 0xAA;
	}

	operation_status = Si4464_Get_TX_FIFO_Free_Space(&num_reported_pre_send);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Si4464_Write_TX_FIFO(fifo_src, SI4464_NUM_FIFO_SRC_BYTES, &num_sent);
	if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Si4464_Get_TX_FIFO_Free_Space(&num_reported_post_send);
	if (operation_status != SI4464_HAL_OK) goto error;

	if (num_reported_pre_send - num_reported_post_send != SI4464_NUM_FIFO_SRC_BYTES) {
		operation_status = SI4464_HAL_ERROR;
		goto error;
	}

error:
	return operation_status;
}


//###############################################################################################
//Public Complete Unit Test Function
//###############################################################################################
Si4464_StatusTypeDef Test_Si4464()
{
    Si4464_StatusTypeDef operation_status;

    // The following functions are executed in order of dependencies

    // Unit test functions
    operation_status = Test_Si4464_Init_Device();
    if (operation_status != SI4464_HAL_OK) goto error;

    HAL_Delay(100);

    operation_status = Test_Si4464_Get_CTS();
    if (operation_status != SI4464_HAL_OK) goto error;

    operation_status = Test_Si4464_Get_Set_Props();
    if (operation_status != SI4464_HAL_OK) goto error;

	operation_status = Test_Si4464_Send_Get_TX_FIFO();
	if (operation_status != SI4464_HAL_OK) goto error;

    operation_status = Test_Si4464_Reset_Device();
    if (operation_status != SI4464_HAL_OK) goto error;

    operation_status = Si4464_Init_Device(); // Clean up everything.
    if (operation_status != SI4464_HAL_OK) goto error;

error:
    return operation_status;
}
