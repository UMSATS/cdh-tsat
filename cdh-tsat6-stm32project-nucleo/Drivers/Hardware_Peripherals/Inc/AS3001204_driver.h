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


// Register addresses (datasheet p.22)
#define AS3001024_REGIST_STATUS		0x00
#define AS3001024_REGIST_CONFIG1	0x02
#define AS3001024_REGIST_CONFIG2	0x03
#define AS3001024_REGIST_CONFIG3	0x04
#define AS3001024_REGIST_CONFIG4	0x05
#define AS3001024_REGIST_DEVICEID	0x30
#define AS3001024_REGIST_UNIQUEID	0x40


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
