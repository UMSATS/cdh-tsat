/*
 * temp_sensor_test.c
 *
 *  Created on: Sep 17, 2024
 *      Author: Christian Joel V
 */

#include <stdio.h>
#include <temperature_sensor.h>
#include <math.h>

/*
 * A method to read a 16-bit calibration value from two consecutive memory addresses.
 *
 * Calibration 1: first = 0x10, second = 0x04.
 * = 0x10 << 8 | 0x04
 * = 0x1000 | 0x04
 * = 0x1004
 * = 4100 (converted to decimal)
 *
 * Calibration 2: first = 0x6A, second = 0x05.
 * = 0x6A << 8 | 0x05
 * = 0x6A00 | 0x05
 * = 0x6A05
 * = 27141 (converted to decimal)
 */
uint16_t readCalibrationValue(uint32_t* address){
	// Convert address to a byte pointer.
	uint8_t* bytePointer = (uint8_t*)address;

	// Read the two consecutive bytes
	uint16_t first = *(bytePointer),
			 second = *(bytePointer + 1);

    // Combine them into a 16-bit value.
    return (uint16_t)(first << 8 | second);
}

// Method to get the first calibration.
uint16_t getTScal1(){
	return readCalibrationValue(TS_CAL1_ADDR);
}

// Method to get the second calibration.
uint16_t getTScal2(){
	return readCalibrationValue(TS_CAL2_ADDR);
}

// Methods to read the temperature sensor using ADC.
uint16_t readADC(ADC_HandleTypeDef* hadc1){
	// Start the ADC conversion.
    HAL_ADC_Start(hadc1);

    // Wait for the conversion to complete.
    HAL_ADC_PollForConversion(hadc1, ADC_TIMEOUT);

    // Retrieve the ADC value.
    uint16_t ADC_value = HAL_ADC_GetValue(hadc1);

    // Stop the ADC conversion.
    HAL_ADC_Stop(hadc1);

    return ADC_value;
}

// Method to get the temperature value with a tolerance of ±5 degrees Celsius.
float readTemperature(ADC_HandleTypeDef* hadc1){
	// Get the ADC and calibration values.
	uint16_t ADC_value = readADC(hadc1), ts_cal1 = getTScal1(), ts_cal2 = getTScal2();

	// Calculate the temperature.
	return TS_CAL1_CELSIUS + (ADC_value - ts_cal1) * (TS_CAL2_CELSIUS - TS_CAL1_CELSIUS) / (ts_cal2 - ts_cal1);
}

// Method to round the temperature.
int16_t getRoundedTemperature(ADC_HandleTypeDef* hadc1){
	return round(readTemperature(hadc1));
}

// Method to get the temperature multiplied by 100 in integer form to include the first two decimal places.
int16_t getTemperatureToScaledInt(ADC_HandleTypeDef* hadc1){
	return (int16_t)(readTemperature(hadc1)*pow(10, SCALED_INT_DECIMAL_PLACES));
}
