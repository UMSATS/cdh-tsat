/*
 * FILENAME: temperature_sensor.h
 *
 * DESCRIPTION: STM32L4 temperature sensor.
 *
 * AUTHORS:
 *  - Christian Ventura (christian.ventura@umsats.ca)
 *
 * CREATED ON: Sept. 17, 2024
 */

#ifndef INC_TEMPERATURE_SENSOR_H_
#define INC_TEMPERATURE_SENSOR_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################

// Define addresses for calibration values with a tolerance of ±5 degrees Celsius.
#define TS_CAL1_ADDR   ((uint32_t*)0x1FFF75A8) // Memory address with the next value is 0x1FFF75A9.
#define TS_CAL2_ADDR   ((uint32_t*)0x1FFF75CA) // Memory address with the next value is 0x1FFF75CB
#define TS_CAL1_CELSIUS    30.0 // Calibration value for TS_CAL1 in degrees Celsius.
#define TS_CAL2_CELSIUS    130.0 // Calibration value for TS_CAL2 in degrees Celsius

#define ADC_TIMEOUT    20 // Time out value of the ADC conversion in milliseconds.
#define SCALED_INT_DECIMAL_PLACES    2 // Amount of decimal places to present in the getTemperatureToScaledInt() method.

//###############################################################################################
//Public Function Prototypes
//###############################################################################################

/*
 * FUNCTION: readCalibrationValue
 *
 * DESCRIPTION: Return the calibration value based on the first memory address.
 *
 * NOTES:
 *  - The actual temperature calibration value is stored in two consecutive memory locations.
 *  - The values for STM32L4xxx is:
 *    - Calibration 1: first = 0x10 (at memory address 0x1FFF75A8), 
 *                     second = 0x04 (at memory address 0x1FFF75A9).
 *    - Calibration 2: first = 0x6A (at memory address 0x1FFF75CA), 
 *                     second = 0x05 (at memory address 0x1FFF75CB).
 *  - The address parameter is assumed to be the first memory address (Ex: 0x1FFF75CA for Calibration)
 * 
 * PARAMETERS:
 *  uint32_t* address: The first memory address of the calibration value.
 * 
 * readCalibrationValue SPECIFIC RETURNS:
 *  The calibration value that is read from memory.
 */
uint16_t readCalibrationValue(uint32_t* address);

/*
 * FUNCTION: getTScal1
 *
 * DESCRIPTION: Get the first calibration value, which is at 30 degrees C.
 *
 * NOTES:
 *  - This refers to the TS_CAL1_ADDR value.
 * 
 * getTScal1 SPECIFIC RETURNS:
 *  The first calibration value.
 */
uint16_t getTScal1();

/*
 * FUNCTION: getTScal2
 *
 * DESCRIPTION: Get the second calibration value, which is at 130 degrees C.
 *
 * NOTES:
 *  - This refers to the TS_CAL2_ADDR value.
 * 
 * getTScal2 SPECIFIC RETURNS:
 *  The second calibration value.
 */
uint16_t getTScal2();

/*
 * FUNCTION: readADC
 *
 * DESCRIPTION: Read the temperature using ADC to obtain the ADC 
 *              value of the internal microcontroller's temperature.
 * 
 * readADC SPECIFIC RETURNS:
 *  The ADC value of the internal microcontroller's microcontroller.
 */
uint16_t readADC(ADC_HandleTypeDef* hadc1);

/*
 * FUNCTION: readTemperature
 *
 * DESCRIPTION: Based on calibration values and ADC values from the 
 *              temperature sensor, calculate and return the temperature.
 * NOTES: 
 *  - The ADC polling function could likely be a blocking function.
 *  - It may be useful to find an interrupt-based function to align with FreeRTOS.
 *
 * PARAMETERS:
 *  ADC_HandleTypeDef* hadc1: The ADC handle.
 * 
 * readTemperature SPECIFIC RETURNS:
 *  The temperature value.
 */
float readTemperature(ADC_HandleTypeDef* hadc1);

/*
 * FUNCTION: getRoundedTemperature
 *
 * DESCRIPTION: Get the rounded temperature at the nearest integer.
 *
 * PARAMETERS:
 *  ADC_HandleTypeDef* hadc1: The ADC handle.
 * 
 * getRoundedTemperature SPECIFIC RETURNS:
 *  The rounded temperature value at the nearest integer.
 */
int16_t getRoundedTemperature(ADC_HandleTypeDef* hadc1);

/*
 * FUNCTION: getTemperatureToScaledInt
 *
 * DESCRIPTION: Get the temperature multiplied by 100 in integer 
 *              form to include the first two decimal places.
 *
 * PARAMETERS:
 *  ADC_HandleTypeDef* hadc1: The ADC handle.
 * 
 * getTemperatureToScaledInt SPECIFIC RETURNS:
 *  The temperature in integer form that is multiplied by 100 to include the first two decimal places.
 */
int16_t getTemperatureToScaledInt(ADC_HandleTypeDef* hadc1);

#endif /* INC_TEMPERATURE_SENSOR_H_ */
