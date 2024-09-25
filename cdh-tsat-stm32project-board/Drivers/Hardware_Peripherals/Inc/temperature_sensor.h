/*
 * temp_sensor_test.h
 *
 *  Created on: Sep 17, 2024
 *      Author: Christian Joel V
 */

#ifndef INC_TEMPERATURE_SENSOR_H_
#define INC_TEMPERATURE_SENSOR_H_

#include "stm32l4xx_hal.h"

// Define addresses for calibration values with a tolerance of ±5 degrees Celsius.
#define TS_CAL1_ADDR   ((uint32_t*)0x1FFF75A8)
#define TS_CAL2_ADDR   ((uint32_t*)0x1FFF75CA)
#define TS_CAL1_CELSIUS    30.0 // Calibration value for TS_CAL1 in degrees Celsius.
#define TS_CAL2_CELSIUS    130.0 // Calibration value for TS_CAL2 in degrees Celsius

// Time out value of the ADC conversion in milliseconds.
#define ADC_TIMEOUT    20

// Amount of decimal places to present in the getTemperatureToScaledInt() method.
#define SCALED_INT_DECIMAL_PLACES    2

// Necessary functions.
uint16_t readCalibrationValue(uint32_t* address);
uint16_t getTScal1();
uint16_t getTScal2();
uint16_t readADC(ADC_HandleTypeDef* hadc1);
float readTemperature(ADC_HandleTypeDef* hadc1);
int16_t getRoundedTemperature(ADC_HandleTypeDef* hadc1);
int16_t getTemperatureToScaledInt(ADC_HandleTypeDef* hadc1);

#endif /* INC_TEMPERATURE_SENSOR_H_ */
