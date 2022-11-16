/*
 * FILENAME: AS3001204_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the AS3001204-0054X0ISAY MRAM unit.
 *
 * AUTHORS:
 *  - Gabriel Young (gabriel.young@umsats.ca)
 *
 * Created on: Nov. 14, 2022
 */

#ifndef HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_


//###############################################################################################
// Define Directives
//###############################################################################################
#define AS3001024_SPI			hspi2

#define AS3001024_nCS_GPIO		GPIOC
#define AS3001024_nCS_PIN		GPIO_PIN_8

#define AS3001024_nWP_GPIO		GPIOC
#define AS3001024_nWP_PIN		GPIO_PIN_9

#define AS3001024_SPI_DELAY		HAL_MAX_DELAY


// Opcodes (datasheet pp. 32-36)
// Control operations (1-0-0 mode)
#define AS3001024_OPCODE_NO_OPERATION		0x00
#define AS3001024_OPCODE_WRITE_ENABLE		0x06
#define AS3001024_OPCODE_WRITE_DISABLE		0x04
#define AS3001024_OPCODE_ENTER_DEEP_PWDOWN	0xb9
#define AS3001024_OPCODE_ENTER_HIBERNATE	0xba
#define AS3001024_OPCODE_EXIT_DEEP_PWDOWN	0xab
#define AS3001024_OPCODE_SOFT_RESET_ENABLE	0x66
#define AS3001024_OPCODE_SOFT_RESET			0x99

// Read register operations (1-0-1 mode)
#define AS3001024_OPCODE_READ_STATUS_REG	0x05
#define AS3001024_OPCODE_READ_CONFIG_REGS	0x46
#define AS3001024_OPCODE_READ_DEVICE_ID		0x9f
#define AS3001024_OPCODE_READ_UNIQUE_ID		0x4c
#define AS3001024_OPCODE_READ_AAP_REG		0x14

// Write register operations (1-0-1 mode)
#define AS3001024_OPCODE_WRITE_STATUS_REG	0x01
#define AS3001024_OPCODE_WRITE_CONFIG_REGS	0x87
#define AS3001024_OPCODE_WRITE_AAP_REG		0x1a

// Memory operations (1-1-1 mode) (assuming we're using SDR and SPI)
#define AS3001024_OPCODE_READ_MEMORY		0x03
#define AS3001024_OPCODE_WRITE_MEMORY		0x02
// omitted fast read & write options (50 vs 108 MHz max)
#define AS3001024_OPCODE_READ_AUG_STORAGE	0x4b
#define AS3001024_OPCODE_WRITE_AUG_STORAGE	0x42


//###############################################################################################
// Global Variable Declarations
//###############################################################################################
extern SPI_HandleTypeDef AS3001024_SPI;


//###############################################################################################
// Driver Function Prototypes
//###############################################################################################

// driver functions go here



//###############################################################################################
// Helper Function Prototypes
//###############################################################################################

// helper functions go here




#endif /* HARDWARE_PERIPHERALS_INC_AS3001204_DRIVER_H_ */
