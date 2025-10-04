/*
 * FILENAME: bdot_algorithm_test.h
 *
 * DESCRIPTION: Unit test for the B-dot detumbling algorithm
 *              (see bdot_algorithm.{c,h})
 *
 * AUTHORS:
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * Created on: Jan. 7, 2025
 */

#ifndef INC_BDOT_ALGORITHM_TEST_H_
#define INC_BDOT_ALGORITHM_TEST_H_

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/*
 * Test result structure
 */
typedef struct {
    bool passed;
    const char* test_name;
    const char* error_message;
    float expected_value;
    float actual_value;
    float tolerance;
} BdotTestResult_t;

/*
 * Test suite result structure
 */
typedef struct {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    BdotTestResult_t* results;
} BdotTestSuiteResult_t;

/*
 * Basic functionality tests
 *
 * FUNCTIONS:   Bdot_Test_ExponentialFilter, Bdot_Test_Initialization, Bdot_Test_FirstCall
 *
 * DESCRIPTION: These functions test basic B-dot algorithm functionality including
 *              exponential filtering, initialization, and first call behavior.
 *
 * RETURN:      BdotTestResult_t with test results
 */
BdotTestResult_t Bdot_Test_ExponentialFilter(void);
BdotTestResult_t Bdot_Test_Initialization(void);
BdotTestResult_t Bdot_Test_FirstCall(void);

/*
 * B-dot calculation tests
 *
 * FUNCTIONS:   Bdot_Test_StaticField, Bdot_Test_LinearChange, Bdot_Test_SinusoidalField
 *
 * DESCRIPTION: These functions test B-dot calculation with various magnetic field
 *              scenarios: static field (should produce zero), linear change (constant B-dot),
 *              and sinusoidal field (oscillating B-dot).
 *
 * RETURN:      BdotTestResult_t with test results
 */
BdotTestResult_t Bdot_Test_StaticField(void);
BdotTestResult_t Bdot_Test_LinearChange(void);
BdotTestResult_t Bdot_Test_SinusoidalField(void);

/*
 * Edge case tests
 *
 * FUNCTIONS:   Bdot_Test_ZeroTimeDelta, Bdot_Test_LargeFieldValues, Bdot_Test_NoiseRejection
 *
 * DESCRIPTION: These functions test edge cases and robustness: zero time delta,
 *              large field values, and noise rejection capabilities.
 *
 * RETURN:      BdotTestResult_t with test results
 */
BdotTestResult_t Bdot_Test_ZeroTimeDelta(void);
BdotTestResult_t Bdot_Test_LargeFieldValues(void);
BdotTestResult_t Bdot_Test_NoiseRejection(void);

/*
 * Integration tests
 *
 * FUNCTIONS:   Bdot_Test_SimulinkComparison, Bdot_Test_StateMachine
 *
 * DESCRIPTION: These functions test integration aspects: comparison with Simulink
 *              results and state machine behavior.
 *
 * RETURN:      BdotTestResult_t with test results
 */
BdotTestResult_t Bdot_Test_SimulinkComparison(void);
BdotTestResult_t Bdot_Test_StateMachine(void);

/*
 * Test suite runner
 *
 * FUNCTION:    Bdot_RunAllTests
 *
 * DESCRIPTION: Runs all B-dot algorithm tests and returns comprehensive results.
 *              This is the main function to call for complete testing.
 *
 * RETURN:      BdotTestSuiteResult_t with all test results
 */
BdotTestSuiteResult_t Bdot_RunAllTests(void);

/*
 * Utility functions
 *
 * FUNCTIONS:   Bdot_PrintTestResults, Bdot_FreeTestResults
 *
 * DESCRIPTION: Utility functions for printing test results and cleaning up memory.
 */
void Bdot_PrintTestResults(const BdotTestSuiteResult_t* results);
void Bdot_FreeTestResults(BdotTestSuiteResult_t* results);

/*
 * Test data generation
 *
 * FUNCTIONS:   Bdot_GenerateTestData_Linear, Bdot_GenerateTestData_Sinusoidal
 *
 * DESCRIPTION: Functions to generate test data for various scenarios.
 */
void Bdot_GenerateTestData_Linear(float* field_data, uint32_t num_samples, float dt, float slope[3]);
void Bdot_GenerateTestData_Sinusoidal(float* field_data, uint32_t num_samples, float dt, float amplitude[3], float frequency[3]);

#endif /* INC_BDOT_ALGORITHM_TEST_H_ */
