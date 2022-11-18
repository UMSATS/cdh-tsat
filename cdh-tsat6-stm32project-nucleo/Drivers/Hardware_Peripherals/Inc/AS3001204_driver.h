/*
 * FILENAME: AS3001204_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the AS3001204-0054X0ISAY MRAM unit.
 *
 * AUTHORS:
 *  - Gabriel Young (gabriel.young@umsats.ca)
 *  - Om Sevak (Om.Sevak@umsats.ca)
 *
 * Created on: Nov. 14, 2022
 */

#ifndef HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_

//###############################################################################################
// Includes
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
// Define Directives
//###############################################################################################
#define AS3001204_SPI			hspi2
#define AS3001204_SPI_DELAY		HAL_MAX_DELAY

#define AS3001204_nCS_GPIO		GPIOC
#define AS3001204_nCS_PIN		GPIO_PIN_8

#define AS3001204_nWP_GPIO		GPIOC
#define AS3001204_nWP_PIN		GPIO_PIN_9


// Opcodes (datasheet pp. 32-36)
// Control operations (1-0-0 type)
#define AS3001204_OPCODE_WRITE_ENABLE		0x06
#define AS3001204_OPCODE_WRITE_DISABLE		0x04
#define AS3001204_OPCODE_ENTER_DEEP_PWDOWN	0xb9
#define AS3001204_OPCODE_ENTER_HIBERNATE	0xba
#define AS3001204_OPCODE_EXIT_DEEP_PWDOWN	0xab
#define AS3001204_OPCODE_SOFT_RESET_ENABLE	0x66
#define AS3001204_OPCODE_SOFT_RESET			0x99

// Read register operations (1-0-1 type)
#define AS3001204_OPCODE_READ_STATUS_REG	0x05
#define AS3001204_OPCODE_READ_CONFIG_REGS	0x46
#define AS3001204_OPCODE_READ_DEVICE_ID		0x9f
#define AS3001204_OPCODE_READ_UNIQUE_ID		0x4c
#define AS3001204_OPCODE_READ_AAP_REG		0x14

// Write register operations (1-0-1 type)
#define AS3001204_OPCODE_WRITE_STATUS_REG	0x01
#define AS3001204_OPCODE_WRITE_CONFIG_REGS	0x87
#define AS3001204_OPCODE_WRITE_AAP_REG		0x1a

// Register lengths (in bytes)
#define AS3001204_STATUS_REG_LENGTH         1
#define AS3001204_CONFIG_REGS_LENGTH        4
#define AS3001204_DEVICE_ID_LENGTH          4
#define AS3001204_UNIQUE_ID_LENGTH          8
#define AS3001204_AAP_REG_LENGTH            1

// Memory operations (1-1-1 type)
#define AS3001204_OPCODE_READ_MEMORY		0x03
#define AS3001204_OPCODE_WRITE_MEMORY		0x02
#define AS3001204_OPCODE_READ_AUG_STORAGE	0x4b
#define AS3001204_OPCODE_WRITE_AUG_STORAGE	0x42

#define AS3001204_WRITE_AUG_STORAGE_DELAY   0x0000 0000 0000 0000


//###############################################################################################
// Global Variable Declarations
//###############################################################################################
extern SPI_HandleTypeDef AS3001204_SPI;


//###############################################################################################
// Driver Function Prototypes
//###############################################################################################

/*
 * FUNCTION: AS3001204_Write_Enable, AS3001204_Write_Disable, AS3001204_Enter_Hibernate,
 *           AS3001204_Enter_Deep_Power_Down, AS3001204_Exit_Deep_Power,
 *           AS3001204_Software_Reset_Enable, AS3001204_Software_Reset
 *
 * DESCRIPTION: ...
 *
 * NOTES:
 *  - notes go here
 *  - ....
 *
 */
HAL_StatusTypeDef AS3001204_Write_Enable();
HAL_StatusTypeDef AS3001204_Write_Disable();
HAL_StatusTypeDef AS3001204_Enter_Hibernate();
HAL_StatusTypeDef AS3001204_Enter_Deep_Power_Down();
HAL_StatusTypeDef AS3001204_Exit_Deep_Power_Down();
HAL_StatusTypeDef AS3001204_Software_Reset_Enable();
HAL_StatusTypeDef AS3001204_Software_Reset();

HAL_StatusTypeDef AS3001204_Read_Status_Register(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Config_Registers(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Device_ID(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Unique_ID(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Augmented_Array_Protection_Register(uint8_t *p_buffer);

HAL_StatusTypeDef AS3001204_Write_Status_Register(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Write_Config_Registers(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Write_Augmented_Array_Protection_Register(uint8_t *p_buffer);

HAL_StatusTypeDef AS3001204_Read_Memory(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes);
HAL_StatusTypeDef AS3001204_Write_Memory(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes);

HAL_StatusTypeDef AS3001204_Read_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes);
HAL_StatusTypeDef AS3001204_Write_Augmented_Storage(uint8_t *p_buffer, uint32_t address, uint16_t num_of_bytes);



#endif  HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_
