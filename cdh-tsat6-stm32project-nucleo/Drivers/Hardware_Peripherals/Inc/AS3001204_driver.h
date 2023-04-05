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
 * 
 * CONTENTS:
 *   1. Includes
 *   2. Define directives
 *   3. Global declarations
 *   4. Public driver function prototypes
 *       4.1. Basic commands
 *       4.2. Read registers
 *       4.3. Write registers
 *       4.4. Read/write memory
 *       4.5. Read/write Augmented Storage Array
 *       4.6. Device initialization
 */

#ifndef HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_

// ###############################################################################################
//  1. Includes
// ###############################################################################################
#include "stm32l4xx_hal.h"

// ###############################################################################################
//  2. Define directives
// ###############################################################################################

// Peripheral pins & handles
#define AS3001204_SPI 			hspi2
#define AS3001204_SPI_DELAY 	HAL_MAX_DELAY

#define AS3001204_nCS_GPIO 		GPIOC
#define AS3001204_nCS_PIN 		GPIO_PIN_8

#define AS3001204_nWP_GPIO 		GPIOC
#define AS3001204_nWP_PIN 		GPIO_PIN_9

// Register lengths (in bytes)
#define AS3001204_STATUS_REG_LENGTH 		1
#define AS3001204_CONFIG_REGS_LENGTH 		4
#define AS3001204_DEVICE_ID_LENGTH 			4
#define AS3001204_UNIQUE_ID_LENGTH 			8
#define AS3001204_ASP_REG_LENGTH 			1

// ###############################################################################################
//  3. Global declarations
// ###############################################################################################
extern SPI_HandleTypeDef AS3001204_SPI;

// ###############################################################################################
//  4. Public driver function prototypes
// ###############################################################################################

/*
 * 4.1. Basic commands
 *
 * FUNCTIONS:   AS3001204_Enter_Hibernate, AS3001204_Exit_Hibernate,
 *              AS3001204_Enter_Deep_Power_Down, AS3001204_Exit_Deep_Power_Down,
 *              AS3001204_Software_Reset
 *
 * DESCRIPTION: These functions send basic commands to the MRAM device, which consist only
 *              of an opcode (no memory addresses or datastreams to read/write).
 */
HAL_StatusTypeDef AS3001204_Enter_Hibernate();
HAL_StatusTypeDef AS3001204_Exit_Hibernate();
HAL_StatusTypeDef AS3001204_Enter_Deep_Power_Down();
HAL_StatusTypeDef AS3001204_Exit_Deep_Power_Down();
HAL_StatusTypeDef AS3001204_Software_Reset();

/*
 * 4.2. Read registers
 *
 * FUNCTIONS:   AS3001204_Read_Status_Register, AS3001204_Read_Config_Registers,
 *              AS3001204_Read_Device_ID, AS3001204_Read_Unique_ID,
 *              AS3001204_Read_ASP_Register
 *
 * DESCRIPTION: These functions read reserved registers from the MRAM device.
 *              Refer to the definitions above for the length of each register.
 *              The functionality of these registers can be found at datasheet
 *              pp. 23-31.
 *
 * PARAMETERS:
 *  - p_buffer: Pointer to a C buffer of type uint8_t into which the byte(s)
 *              of the MRAM device register will be read.
 */
HAL_StatusTypeDef AS3001204_Read_Status_Register(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Config_Registers(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Device_ID(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_Unique_ID(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Read_ASP_Register(uint8_t *p_buffer);

/*
 * 4.3. Write registers
 *
 * FUNCTIONS:   AS3001204_Write_Status_Register, AS3001204_Write_Config_Registers,
 *              AS3001204_Write_ASP_Register
 *
 * DESCRIPTION: These functions write reserved registers to the MRAM device.
 *              Refer to the definitions above for the length of each register.
 *              The functionality of these registers can be found at datasheet
 *              pp. 23-31.
 *
 * PARAMETERS:
 *  - p_buffer: Pointer to a C buffer of type uint8_t whose contents will be
 *              written into the MRAM device register.
 *
 * NOTES:
 *  - The Write Enable (WREN) directive must be sent to the MRAM device before
 *    data can be written to any registers. This form of write protection is
 *    separate from the Write Protect (WP) pin, and the Top/Bottom Memory Array
 *    Protection option (see datasheet pp. 23-24). WREN requirements can be
 *    configured in Config Register 4 (see datasheet p. 31).
 */
HAL_StatusTypeDef AS3001204_Write_Status_Register(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Write_Config_Registers(uint8_t *p_buffer);
HAL_StatusTypeDef AS3001204_Write_ASP_Register(uint8_t *p_buffer);

/*
 * 4.4. Read/write memory
 *
 * FUNCTIONS:   AS3001204_Read_Memory, AS3001204_Write_Memory
 *
 * DESCRIPTION: These functions read from (write to) the memory array of
 *              the MRAM device.
 *
 * PARAMETERS:
 *  - p_buffer:     Pointer to a C buffer of type uint8_t into (from) which
 *                  data will be read from (written to) the MRAM memory array.
 *  - address:      The 24-bit memory array address to read from (write to).
 *  - num_of_bytes: The number of bytes to read/write.
 *
 * NOTES:
 *  - The Write Enable (WREN) directive must be sent to the MRAM device before
 *    data can be written to the memory array. This form of write protection is
 *    separate from the Write Protect (WP) pin, and the Top/Bottom Memory Array
 *    Protection option (see datasheet pp. 23-24). WREN requirements can be
 *    configured in Config Register 4 (see datasheet p. 31).
 */
HAL_StatusTypeDef AS3001204_Read_Memory(uint8_t *p_buffer, uint32_t address,
                                        uint16_t num_of_bytes);
HAL_StatusTypeDef AS3001204_Write_Memory(uint8_t *p_buffer, uint32_t address,
                                         uint16_t num_of_bytes);

/*
 * 4.5. Read/write Augmented Storage Array
 *
 * FUNCTIONS:   AS3001204_Read_Augmented_Storage,
 *              AS3001204_Write_Augmented_Storage
 *
 * DESCRIPTION: These functions read from (write to) the Augmented Storage
 *              Array portion of the MRAM memory array.
 *
 * PARAMETERS:
 *  - p_buffer:     Pointer to a C buffer of type uint8_t into (from) which
 *                  data will be read from (written to) the MRAM memory array.
 *  - address:      The 24-bit memory array address to read from (write to).
 *  - num_of_bytes: The number of bytes to read/write.
 *
 * NOTES:
 *  - The Augmented Storage Array is the first 256 bytes (0x000000..0x0000FF)
 *    of the MRAM memory array. It is divided into eight 32-byte segments which
 *    can be independently write-protected by way of the Augmented Array
 *    Protection Register.
 *  - Unlike reading from the standard memory array, reading from the ASA
 *    is an operation with latency. (see timing diagram, datasheet pp. 38-39)
 *    However, the HAL SPI functions fulfill these latency requirements without
 *    needing to make the AS3001204 implement any latency cycles.
 */
HAL_StatusTypeDef AS3001204_Read_Augmented_Storage(uint8_t *p_buffer, uint32_t address,
                                                   uint16_t num_of_bytes);
HAL_StatusTypeDef AS3001204_Write_Augmented_Storage(uint8_t *p_buffer, uint32_t address,
                                                    uint16_t num_of_bytes);


/*
 * 4.6. Device initialization
 *
 * FUNCTIONS:	AS3001204_Init
 *
 * DESCRIPTION: Initializes the AS3001204.
 *
 * NOTES:
 *  - This function ensures the internal initialization is complete by delaying for 1 ms.
 *  - The states of nCS & nWP are set:
 *     nCS is set HIGH: this deselects the AS3001204
 *     nWP is set LOW: this write protects the AS3001204
 *  - The states of the Register bits are set:
 *     SR:  Enable hardware write protection; leave other features disabled
 *     ASP: Leave ASA section-specific protection disabled
 *     CR1: Lock status register protection options; enable ASA write protection
 *     CR2: Single SPI; zero memory read latency
 *     CR3: Leave output driver strength unchanged; leave read wrapping disabled
 *     CR4: Enforce software write enable (WREN) as prerequisite to all memory write instructions
 */
HAL_StatusTypeDef AS3001204_Init();


#endif  // HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_
