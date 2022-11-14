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

// TODO: update GPIO pins as required below
#define W25N_nCS_GPIO			GPIOB
#define W25N_nCS_PIN			GPIO_PIN_15

#define W25N_nWP_GPIO			GPIOC
#define W25N_nWP_PIN			GPIO_PIN_6

#define W25N_nHOLD_GPIO			GPIOC
#define W25N_nHOLD_PIN			GPIO_PIN_7



#define AS3001024_SPI_DELAY		HAL_MAX_DELAY

// TODO: Remove if not required
#define AS3001024_DUMMY_BYTE	0x00

// TODO: All opcodes here...
#define AS3001024_OPCODE_...	0xFF



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
