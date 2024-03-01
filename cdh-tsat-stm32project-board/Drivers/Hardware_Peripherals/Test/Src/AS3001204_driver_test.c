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


#include "stm32l4xx_hal.h"
#include "AS3001204_driver.h"
#include "AS3001204_driver_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//###############################################################################################
// Read/write register tests
//###############################################################################################

HAL_StatusTypeDef AS3001204_Test_Read_ID_Registers() {

    const uint8_t DEVICE_ID[AS3001204_DEVICE_ID_LENGTH] = {0xe6, 0x01, 0x01, 0x02};
    //NOTE: THE UNIQUE ID IS SET TO THE MRAM ON THE TSAT-7 ENGINEERING MODEL
    //      RUNNING THIS TEST ON ANY OTHER MODEL WILL FAIL, SINCE THE UNIQUE ID WILL BE DIFFERENT
    const uint8_t UNIQUE_ID[AS3001204_UNIQUE_ID_LENGTH] = {0xa4, 0x00, 0x01, 0xee, 0x0e, 0x01, 0x00, 0x14};

    HAL_StatusTypeDef isError;
    uint8_t devIDBuffer[AS3001204_DEVICE_ID_LENGTH];
    uint8_t uniIDBuffer[AS3001204_UNIQUE_ID_LENGTH];

    // These are read-only registers; we simply read their contents and compare to expected values
    isError = AS3001204_Read_Device_ID(devIDBuffer);
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Read_Unique_ID(uniIDBuffer);
    if (isError != HAL_OK) goto error;

    if (memcmp(devIDBuffer, DEVICE_ID, AS3001204_DEVICE_ID_LENGTH) != 0 ||
            memcmp(uniIDBuffer, UNIQUE_ID, AS3001204_UNIQUE_ID_LENGTH) != 0)
        isError = HAL_ERROR;

error:
    return isError;

}

HAL_StatusTypeDef AS3001204_Test_RW_Status_Register() {

    uint8_t STATUS_REG_DEFAULT = STATUS_REG_INIT;
    uint8_t STATUS_REG_TEST = 0xc0;

    HAL_StatusTypeDef isError; 
    uint8_t reg_contents;

    // Write test data
    isError = AS3001204_Write_Status_Register(&STATUS_REG_TEST);
    if (isError != HAL_OK) goto error;

    // Read back and verify test data
    isError = AS3001204_Read_Status_Register(&reg_contents);
    if (isError != HAL_OK) goto error;

    if (reg_contents != STATUS_REG_TEST) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Status_Register(&STATUS_REG_DEFAULT);

error:
    return isError;

}

HAL_StatusTypeDef AS3001204_Test_RW_Config_Registers() {

    uint8_t CONFIG_REGS_DEFAULT[AS3001204_CONFIG_REGS_LENGTH] = CONFIG_REGS_INIT;
    uint8_t CONFIG_REGS_TEST[AS3001204_CONFIG_REGS_LENGTH] = {0x05, 0x0f, 0x74, 0x04};

    HAL_StatusTypeDef isError;
    uint8_t test_buffer[AS3001204_CONFIG_REGS_LENGTH];

    // Write test data
    isError = AS3001204_Write_Config_Registers(CONFIG_REGS_TEST);
    if (isError != HAL_OK) goto error;

    // Read back and verify test data
    isError = AS3001204_Read_Config_Registers(test_buffer);
    if (isError != HAL_OK) goto error;

    if (memcmp(test_buffer, CONFIG_REGS_TEST, AS3001204_CONFIG_REGS_LENGTH) != 0) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Config_Registers(CONFIG_REGS_DEFAULT);

error:
    return isError;
    
}

HAL_StatusTypeDef AS3001204_Test_RW_ASP_Register() {
    
    // (A)ugmented (S)torage Array (P)rotection Register
    uint8_t ASP_REG_DEFAULT = ASP_REG_INIT;
    uint8_t ASP_REG_TEST = 0xff;

    HAL_StatusTypeDef isError; 
    uint8_t reg_contents;

    // Write test data
    isError = AS3001204_Write_ASP_Register(&ASP_REG_TEST);
    if (isError != HAL_OK) goto error;

    // Read back and verify test data
    isError = AS3001204_Read_ASP_Register(&reg_contents);
    if (isError != HAL_OK) goto error;

    if (reg_contents != ASP_REG_TEST) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_ASP_Register(&ASP_REG_DEFAULT);

error:
    return isError;
    
}


//###############################################################################################
// Read/write memory tests
//###############################################################################################

HAL_StatusTypeDef AS3001204_Test_RW_Memory() {
    
    const uint32_t MEM_TEST_ADDRESS = 0x00abba;

    // Article 1 of the United Nations Declaration of Human Rights
    uint8_t SAMPLE_DATA[] = "All human beings are born free and equal in dignity and rights. \
    They are endowed with reason and conscience and should act towards one another in a spirit of brotherhood.";
    uint16_t SAMPLE_DATA_LENGTH = sizeof(SAMPLE_DATA)/sizeof(SAMPLE_DATA[0]);

    HAL_StatusTypeDef isError;
    uint8_t test_buffer[SAMPLE_DATA_LENGTH];
    uint8_t erased_data_buffer[SAMPLE_DATA_LENGTH]; //entire buffer will be initialized to 0xff

    for (int i = 0; i < SAMPLE_DATA_LENGTH; i++)
    {
        erased_data_buffer[i] = 0xff;
    }

    // Write test data
    isError = AS3001204_Write_Memory(SAMPLE_DATA, MEM_TEST_ADDRESS, SAMPLE_DATA_LENGTH);
    if (isError != HAL_OK) goto error;
    
    // Read back and verify test data
    isError = AS3001204_Read_Memory(test_buffer, MEM_TEST_ADDRESS, SAMPLE_DATA_LENGTH);
    if (isError != HAL_OK) goto error;
    
    if (memcmp(test_buffer, SAMPLE_DATA, SAMPLE_DATA_LENGTH) != 0) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Memory(erased_data_buffer, MEM_TEST_ADDRESS, SAMPLE_DATA_LENGTH);
    
error:
    return isError;

}

HAL_StatusTypeDef AS3001204_Test_RW_Augmented_Storage() {
    
    const uint32_t ASA_TEST_ADDRESS = 0x000000;

    uint8_t SAMPLE_DATA_ASA[] = "UMSATS is the best club on campus!!!";
    uint16_t SAMPLE_DATA_ASA_LENGTH = sizeof(SAMPLE_DATA_ASA)/sizeof(SAMPLE_DATA_ASA[0]);

    HAL_StatusTypeDef isError;
    uint8_t test_buffer[SAMPLE_DATA_ASA_LENGTH];
    uint8_t erased_data_buffer[SAMPLE_DATA_ASA_LENGTH]; //entire buffer will be initialized to 0xff

    for (int i = 0; i < SAMPLE_DATA_ASA_LENGTH; i++)
    {
        erased_data_buffer[i] = 0xff;
    }
    
    // Write test data
    isError = AS3001204_Write_Augmented_Storage(SAMPLE_DATA_ASA, ASA_TEST_ADDRESS, SAMPLE_DATA_ASA_LENGTH);
    if (isError != HAL_OK) goto error;
    
    // Read back and verify test data
    isError = AS3001204_Read_Augmented_Storage(test_buffer, ASA_TEST_ADDRESS, SAMPLE_DATA_ASA_LENGTH);
    if (isError != HAL_OK) goto error;
    
    if (memcmp(test_buffer, SAMPLE_DATA_ASA, SAMPLE_DATA_ASA_LENGTH) != 0) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Augmented_Storage(erased_data_buffer, ASA_TEST_ADDRESS, SAMPLE_DATA_ASA_LENGTH);
    
error:
    return isError;
    
}


//###############################################################################################
// Write disable test
//###############################################################################################

HAL_StatusTypeDef AS3001204_Test_Write_Disable() {

    const uint32_t MEM_TEST_ADDRESS = 0x00abba;
    uint8_t SAMPLE_DATA1 = 0x0a;
    uint8_t SAMPLE_DATA2 = 0x0b;
    uint8_t ERASED_DATA = 0xff;

    HAL_StatusTypeDef isError;
    uint8_t test_data;

    // Write sample data 1
    isError = AS3001204_Write_Memory(&SAMPLE_DATA1, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    // Write enable
    // Note: This command is sent since the AS3001204 is configured to automatically write
    //       disable after a successful write
    isError = AS3001204_Write_Enable();
    if (isError != HAL_OK) goto error;

    // Write disable
    isError = AS3001204_Write_Disable();
    if (isError != HAL_OK) goto error;

    // Write sample data 2
    // Note: AS3001204_Write_Memory(...) calls AS3001204_Write_Enable()
    //       So, we must send the write command independently
    uint8_t memory_write_opcode = 0x02;

    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_RESET);

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &memory_write_opcode, sizeof(memory_write_opcode), AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    // Separate 3 bytes of address from the uint32 (most significant byte ignored)
    uint8_t address_bytes[3] = {(MEM_TEST_ADDRESS >> 16) & 0xff, (MEM_TEST_ADDRESS >> 8) & 0xff, (MEM_TEST_ADDRESS) & 0xff};
    isError = HAL_SPI_Transmit(&AS3001204_SPI, address_bytes, 3, AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    isError = HAL_SPI_Transmit(&AS3001204_SPI, &SAMPLE_DATA2, 1, AS3001204_SPI_DELAY);
    if (isError != HAL_OK) goto error;

    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);

    // Read back and verify test data
    isError = AS3001204_Read_Memory(&test_data, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    if (test_data != SAMPLE_DATA1) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Memory(&ERASED_DATA, MEM_TEST_ADDRESS, 1);

error:
    HAL_GPIO_WritePin(AS3001204_nCS_GPIO, AS3001204_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AS3001204_nWP_GPIO, AS3001204_nWP_PIN, GPIO_PIN_RESET);
    return isError;

}


//###############################################################################################
// Enter/exit hibernate and deep power down tests
//###############################################################################################

HAL_StatusTypeDef AS3001204_Test_Enter_Exit_Hibernate() {

    const uint32_t MEM_TEST_ADDRESS = 0x00abba;
    uint8_t SAMPLE_DATA1 = 0x01;
    uint8_t SAMPLE_DATA2 = 0x02;
    uint8_t ERASED_DATA = 0xff;

    HAL_StatusTypeDef isError;
    uint8_t test_data;

    // Write sample data 1
    isError = AS3001204_Write_Memory(&SAMPLE_DATA1, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    // Enter hibernate
    isError = AS3001204_Enter_Hibernate();
    if (isError != HAL_OK) goto error;

    // Write sample data 2
    isError = AS3001204_Write_Memory(&SAMPLE_DATA2, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    // Exit hibernate
    isError = AS3001204_Exit_Hibernate();
    if (isError != HAL_OK) goto error;

    // Read back and verify test data
    isError = AS3001204_Read_Memory(&test_data, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    if (test_data != SAMPLE_DATA1) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Memory(&ERASED_DATA, MEM_TEST_ADDRESS, 1);

error:
    return isError;

}

HAL_StatusTypeDef AS3001204_Test_Enter_Exit_Deep_Power_Down() {

    const uint32_t MEM_TEST_ADDRESS = 0x00abba;
    uint8_t SAMPLE_DATA = 0x0a;
    uint8_t ERASED_DATA = 0xff;

    HAL_StatusTypeDef isError;
    uint8_t test_data;

    // Write sample data
    isError = AS3001204_Write_Memory(&SAMPLE_DATA, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    // Enter deep power down
    isError = AS3001204_Enter_Deep_Power_Down();
    if (isError != HAL_OK) goto error;

    // Exit deep power down
    isError = AS3001204_Exit_Deep_Power_Down();
    if (isError != HAL_OK) goto error;

    // Read back and verify test data
    isError = AS3001204_Read_Memory(&test_data, MEM_TEST_ADDRESS, 1);
    if (isError != HAL_OK) goto error;

    if (test_data != SAMPLE_DATA) {
        isError = HAL_ERROR;
        goto error;
    }

    // Restore to default
    isError = AS3001204_Write_Memory(&ERASED_DATA, MEM_TEST_ADDRESS, 1);

    error:
        return isError;

}


//###############################################################################################
// Complete test suite routine
//###############################################################################################

HAL_StatusTypeDef AS3001204_Test_MRAM_Driver() {

    HAL_StatusTypeDef isError;

    isError = AS3001204_Test_Read_ID_Registers();
    if (isError != HAL_OK) goto error;
    isError = AS3001204_Test_RW_Status_Register();
    if (isError != HAL_OK) goto error;
    isError = AS3001204_Test_RW_Config_Registers();
    if (isError != HAL_OK) goto error;
    isError = AS3001204_Test_RW_ASP_Register();
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Test_RW_Memory();
    if (isError != HAL_OK) goto error;
    isError = AS3001204_Test_RW_Augmented_Storage();
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Test_Write_Disable();
    if (isError != HAL_OK) goto error;

    isError = AS3001204_Test_Enter_Exit_Hibernate();
    if (isError != HAL_OK) goto error;
    isError = AS3001204_Test_Enter_Exit_Deep_Power_Down();

    // Note: AS3001204_Write_Enable() is tested implicitly through its use in other functions

error:
    return isError;

}
