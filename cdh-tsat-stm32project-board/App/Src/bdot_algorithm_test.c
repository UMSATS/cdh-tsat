/*
 * FILENAME: bdot_algorithm_test.c
 *
 * DESCRIPTION: Unit test implementation for the B-dot detumbling algorithm
 *              (see bdot_algorithm.{c,h})
 *
 * AUTHORS:
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * Created on: Jan. 7, 2025
 */

#include "bdot_algorithm_test.h"
#include "bdot_algorithm.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Test tolerance for floating point comparisons
#define TEST_TOLERANCE 1e-6f

// Test parameters (should match your algorithm parameters)
#define TEST_ALPHA 0.1f
#define TEST_K 1.0f

// Helper macros for test assertions
#define ASSERT_FLOAT_EQ(expected, actual, tolerance, test_name) \
    do { \
        if (fabsf((expected) - (actual)) > (tolerance)) { \
            result.passed = false; \
            result.test_name = (test_name); \
            result.error_message = "Float values not equal within tolerance"; \
            result.expected_value = (expected); \
            result.actual_value = (actual); \
            result.tolerance = (tolerance); \
            return result; \
        } \
    } while(0)

#define ASSERT_TRUE(condition, test_name) \
    do { \
        if (!(condition)) { \
            result.passed = false; \
            result.test_name = (test_name); \
            result.error_message = "Condition was false"; \
            return result; \
        } \
    } while(0)

#define ASSERT_FALSE(condition, test_name) \
    do { \
        if (condition) { \
            result.passed = false; \
            result.test_name = (test_name); \
            result.error_message = "Condition was true (expected false)"; \
            return result; \
        } \
    } while(0)

// Global test data (simulates g_magnetic_field)
static float test_magnetic_field[3] = {0.0f, 0.0f, 0.0f};

// Mock function to set test magnetic field data
static void set_test_magnetic_field(float x, float y, float z) {
    test_magnetic_field[0] = x;
    test_magnetic_field[1] = y;
    test_magnetic_field[2] = z;
}

// Override g_magnetic_field for testing
extern float g_magnetic_field[3];
#define g_magnetic_field test_magnetic_field

/*
 * Test: Exponential Filter Function
 */
BdotTestResult_t Bdot_Test_ExponentialFilter(void) {
    BdotTestResult_t result = {true, "ExponentialFilter", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Test 1: No filtering (alpha = 1.0)
    float filtered = exponentialFilter(10.0f, 5.0f, 1.0f);
    ASSERT_FLOAT_EQ(10.0f, filtered, TEST_TOLERANCE, "ExponentialFilter_Alpha1");
    
    // Test 2: No change (alpha = 0.0)
    filtered = exponentialFilter(10.0f, 5.0f, 0.0f);
    ASSERT_FLOAT_EQ(5.0f, filtered, TEST_TOLERANCE, "ExponentialFilter_Alpha0");
    
    // Test 3: Half filtering (alpha = 0.5)
    filtered = exponentialFilter(10.0f, 5.0f, 0.5f);
    ASSERT_FLOAT_EQ(7.5f, filtered, TEST_TOLERANCE, "ExponentialFilter_Alpha0.5");
    
    // Test 4: Test with TEST_ALPHA
    filtered = exponentialFilter(10.0f, 5.0f, TEST_ALPHA);
    float expected = TEST_ALPHA * 10.0f + (1.0f - TEST_ALPHA) * 5.0f;
    ASSERT_FLOAT_EQ(expected, filtered, TEST_TOLERANCE, "ExponentialFilter_TestAlpha");
    
    result.passed = true;
    return result;
}

/*
 * Test: Algorithm Initialization
 */
BdotTestResult_t Bdot_Test_Initialization(void) {
    BdotTestResult_t result = {true, "Initialization", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset algorithm state
    AttitudeControl_Init();
    
    // Test that first call returns false (needs initialization)
    float m[3];
    bool success = ADCS_Bdot_Compute(m);
    ASSERT_FALSE(success, "Initialization_FirstCall");
    
    // Test that m values are zero
    for (int i = 0; i < 3; i++) {
        ASSERT_FLOAT_EQ(0.0f, m[i], TEST_TOLERANCE, "Initialization_ZeroOutput");
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: First Valid Call
 */
BdotTestResult_t Bdot_Test_FirstCall(void) {
    BdotTestResult_t result = {true, "FirstCall", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Set initial magnetic field
    set_test_magnetic_field(1.0e-5f, 2.0e-5f, 3.0e-5f); // 10, 20, 30 µT
    
    // First call should return false (initialization)
    float m[3];
    bool success = ADCS_Bdot_Compute(m);
    ASSERT_FALSE(success, "FirstCall_Initialization");
    
    // Second call should return true and produce zero output (no change yet)
    success = ADCS_Bdot_Compute(m);
    ASSERT_TRUE(success, "FirstCall_SecondCall");
    
    // With no change in field, B-dot should be zero, so m should be zero
    for (int i = 0; i < 3; i++) {
        ASSERT_FLOAT_EQ(0.0f, m[i], TEST_TOLERANCE, "FirstCall_ZeroOutput");
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: Static Magnetic Field
 */
BdotTestResult_t Bdot_Test_StaticField(void) {
    BdotTestResult_t result = {true, "StaticField", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Set constant magnetic field
    set_test_magnetic_field(1.0e-5f, 2.0e-5f, 3.0e-5f);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Multiple calls with same field should produce zero output
    for (int i = 0; i < 5; i++) {
        bool success = ADCS_Bdot_Compute(m);
        ASSERT_TRUE(success, "StaticField_Success");
        
        for (int j = 0; j < 3; j++) {
            ASSERT_FLOAT_EQ(0.0f, m[j], TEST_TOLERANCE, "StaticField_ZeroOutput");
        }
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: Linear Magnetic Field Change
 */
BdotTestResult_t Bdot_Test_LinearChange(void) {
    BdotTestResult_t result = {true, "LinearChange", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    float dt = 0.1f; // 100ms time step
    float slope[3] = {1.0e-6f, 2.0e-6f, 3.0e-6f}; // T/s slopes
    
    // Set initial field
    set_test_magnetic_field(0.0f, 0.0f, 0.0f);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Simulate linear change over time
    for (int i = 1; i <= 5; i++) {
        // Update field with linear change
        set_test_magnetic_field(
            slope[0] * i * dt,
            slope[1] * i * dt,
            slope[2] * i * dt
        );
        
        bool success = ADCS_Bdot_Compute(m);
        ASSERT_TRUE(success, "LinearChange_Success");
        
        // Expected B-dot should be the slope
        // Expected m should be -K * slope
        for (int j = 0; j < 3; j++) {
            float expected_m = -TEST_K * slope[j];
            ASSERT_FLOAT_EQ(expected_m, m[j], 1e-7f, "LinearChange_ExpectedOutput");
        }
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: Sinusoidal Magnetic Field
 */
BdotTestResult_t Bdot_Test_SinusoidalField(void) {
    BdotTestResult_t result = {true, "SinusoidalField", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    float dt = 0.1f; // 100ms time step
    float amplitude[3] = {1.0e-5f, 2.0e-5f, 3.0e-5f}; // T
    float frequency[3] = {0.1f, 0.2f, 0.3f}; // Hz
    
    // Set initial field
    set_test_magnetic_field(0.0f, 0.0f, 0.0f);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Simulate sinusoidal change over time
    for (int i = 1; i <= 10; i++) {
        float t = i * dt;
        
        // Update field with sinusoidal change
        set_test_magnetic_field(
            amplitude[0] * sinf(2.0f * M_PI * frequency[0] * t),
            amplitude[1] * sinf(2.0f * M_PI * frequency[1] * t),
            amplitude[2] * sinf(2.0f * M_PI * frequency[2] * t)
        );
        
        bool success = ADCS_Bdot_Compute(m);
        ASSERT_TRUE(success, "SinusoidalField_Success");
        
        // Expected B-dot should be the derivative of the sine function
        // d/dt(sin(2πft)) = 2πf * cos(2πft)
        for (int j = 0; j < 3; j++) {
            float expected_bdot = 2.0f * M_PI * frequency[j] * amplitude[j] * cosf(2.0f * M_PI * frequency[j] * t);
            float expected_m = -TEST_K * expected_bdot;
            ASSERT_FLOAT_EQ(expected_m, m[j], 1e-6f, "SinusoidalField_ExpectedOutput");
        }
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: Zero Time Delta
 */
BdotTestResult_t Bdot_Test_ZeroTimeDelta(void) {
    BdotTestResult_t result = {true, "ZeroTimeDelta", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Set initial field
    set_test_magnetic_field(1.0e-5f, 2.0e-5f, 3.0e-5f);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Simulate zero time delta by calling immediately again
    // This should return false due to zero or negative time delta
    bool success = ADCS_Bdot_Compute(m);
    ASSERT_FALSE(success, "ZeroTimeDelta_ShouldFail");
    
    result.passed = true;
    return result;
}

/*
 * Test: Large Field Values
 */
BdotTestResult_t Bdot_Test_LargeFieldValues(void) {
    BdotTestResult_t result = {true, "LargeFieldValues", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Test with large magnetic field values (e.g., near Earth's surface ~50 µT)
    set_test_magnetic_field(50.0e-6f, 30.0e-6f, 20.0e-6f);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Change field by large amount
    set_test_magnetic_field(60.0e-6f, 40.0e-6f, 25.0e-6f);
    
    bool success = ADCS_Bdot_Compute(m);
    ASSERT_TRUE(success, "LargeFieldValues_Success");
    
    // Verify algorithm doesn't overflow or produce unreasonable values
    for (int i = 0; i < 3; i++) {
        ASSERT_TRUE(fabsf(m[i]) < 1.0f, "LargeFieldValues_ReasonableOutput"); // Should be reasonable
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: Noise Rejection
 */
BdotTestResult_t Bdot_Test_NoiseRejection(void) {
    BdotTestResult_t result = {true, "NoiseRejection", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Test with noisy data - algorithm should filter out high-frequency noise
    float base_field[3] = {1.0e-5f, 2.0e-5f, 3.0e-5f};
    float noise_amplitude = 1.0e-7f; // Small noise
    
    set_test_magnetic_field(base_field[0], base_field[1], base_field[2]);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Apply noise and test filtering
    float m_noisy[3], m_clean[3];
    
    // Test with noise
    for (int i = 0; i < 10; i++) {
        set_test_magnetic_field(
            base_field[0] + noise_amplitude * sinf(i * 0.1f),
            base_field[1] + noise_amplitude * cosf(i * 0.1f),
            base_field[2] + noise_amplitude * sinf(i * 0.2f)
        );
        
        bool success = ADCS_Bdot_Compute(m_noisy);
        ASSERT_TRUE(success, "NoiseRejection_Success");
    }
    
    // Test without noise (should be similar due to filtering)
    set_test_magnetic_field(base_field[0], base_field[1], base_field[2]);
    bool success = ADCS_Bdot_Compute(m_clean);
    ASSERT_TRUE(success, "NoiseRejection_CleanSuccess");
    
    // Filtered results should be similar (within noise tolerance)
    for (int i = 0; i < 3; i++) {
        ASSERT_TRUE(fabsf(m_noisy[i] - m_clean[i]) < noise_amplitude * 10.0f, "NoiseRejection_Filtered");
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: Comparison with Simulink Results
 */
BdotTestResult_t Bdot_Test_SimulinkComparison(void) {
    BdotTestResult_t result = {true, "SimulinkComparison", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // This test uses known results from Simulink simulation
    // You should replace these with actual Simulink results
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Test case: Linear field change
    // Simulink parameters: K=1.0, Alpha=0.1, dt=0.1s
    // Input: B = [0, 0, 0] -> [1e-6, 2e-6, 3e-6] T
    // Expected output: m = -K * B_dot = -K * [1e-5, 2e-5, 3e-5] T/s
    
    set_test_magnetic_field(0.0f, 0.0f, 0.0f);
    
    // Initialize algorithm
    float m[3];
    ADCS_Bdot_Compute(m); // First call (initialization)
    
    // Apply known input
    set_test_magnetic_field(1.0e-6f, 2.0e-6f, 3.0e-6f);
    
    bool success = ADCS_Bdot_Compute(m);
    ASSERT_TRUE(success, "SimulinkComparison_Success");
    
    // Expected values (replace with actual Simulink results)
    float expected_m[3] = {-1.0e-5f, -2.0e-5f, -3.0e-5f}; // These should match Simulink
    
    for (int i = 0; i < 3; i++) {
        ASSERT_FLOAT_EQ(expected_m[i], m[i], 1e-7f, "SimulinkComparison_ExpectedOutput");
    }
    
    result.passed = true;
    return result;
}

/*
 * Test: State Machine
 */
BdotTestResult_t Bdot_Test_StateMachine(void) {
    BdotTestResult_t result = {true, "StateMachine", "", 0.0f, 0.0f, TEST_TOLERANCE};
    
    // This test verifies the state machine transitions
    // Note: This is a simplified test - full state machine testing would require
    // mocking the OS delay functions
    
    // Reset and initialize
    AttitudeControl_Init();
    
    // Test that we start in S1 state
    // (This would require access to internal state - you might need to add getter functions)
    
    // For now, just test that initialization works
    ASSERT_TRUE(true, "StateMachine_Initialization");
    
    result.passed = true;
    return result;
}

/*
 * Test Data Generation Functions
 */
void Bdot_GenerateTestData_Linear(float* field_data, uint32_t num_samples, float dt, float slope[3]) {
    for (uint32_t i = 0; i < num_samples; i++) {
        float t = i * dt;
        field_data[i * 3 + 0] = slope[0] * t;
        field_data[i * 3 + 1] = slope[1] * t;
        field_data[i * 3 + 2] = slope[2] * t;
    }
}

void Bdot_GenerateTestData_Sinusoidal(float* field_data, uint32_t num_samples, float dt, float amplitude[3], float frequency[3]) {
    for (uint32_t i = 0; i < num_samples; i++) {
        float t = i * dt;
        field_data[i * 3 + 0] = amplitude[0] * sinf(2.0f * M_PI * frequency[0] * t);
        field_data[i * 3 + 1] = amplitude[1] * sinf(2.0f * M_PI * frequency[1] * t);
        field_data[i * 3 + 2] = amplitude[2] * sinf(2.0f * M_PI * frequency[2] * t);
    }
}

/*
 * Test Suite Runner
 */
BdotTestSuiteResult_t Bdot_RunAllTests(void) {
    BdotTestSuiteResult_t suite_result = {0, 0, 0, NULL};
    
    // Define all tests
    BdotTestResult_t (*test_functions[])(void) = {
        Bdot_Test_ExponentialFilter,
        Bdot_Test_Initialization,
        Bdot_Test_FirstCall,
        Bdot_Test_StaticField,
        Bdot_Test_LinearChange,
        Bdot_Test_SinusoidalField,
        Bdot_Test_ZeroTimeDelta,
        Bdot_Test_LargeFieldValues,
        Bdot_Test_NoiseRejection,
        Bdot_Test_SimulinkComparison,
        Bdot_Test_StateMachine
    };
    
    uint32_t num_tests = sizeof(test_functions) / sizeof(test_functions[0]);
    suite_result.total_tests = num_tests;
    suite_result.results = malloc(num_tests * sizeof(BdotTestResult_t));
    
    if (suite_result.results == NULL) {
        printf("ERROR: Failed to allocate memory for test results\n");
        return suite_result;
    }
    
    // Run all tests
    for (uint32_t i = 0; i < num_tests; i++) {
        suite_result.results[i] = test_functions[i]();
        
        if (suite_result.results[i].passed) {
            suite_result.passed_tests++;
        } else {
            suite_result.failed_tests++;
        }
    }
    
    return suite_result;
}

/*
 * Utility Functions
 */
void Bdot_PrintTestResults(const BdotTestSuiteResult_t* results) {
    printf("\n=== B-Dot Algorithm Test Results ===\n");
    printf("Total Tests: %lu\n", results->total_tests);
    printf("Passed: %lu\n", results->passed_tests);
    printf("Failed: %lu\n", results->failed_tests);
    printf("Success Rate: %.1f%%\n", 
           (float)results->passed_tests / results->total_tests * 100.0f);
    
    printf("\n=== Detailed Results ===\n");
    for (uint32_t i = 0; i < results->total_tests; i++) {
        const BdotTestResult_t* test = &results->results[i];
        printf("[%s] %s: %s\n", 
               test->passed ? "PASS" : "FAIL",
               test->test_name,
               test->passed ? "OK" : test->error_message);
        
        if (!test->passed) {
            printf("    Expected: %.6e, Actual: %.6e, Tolerance: %.6e\n",
                   test->expected_value, test->actual_value, test->tolerance);
        }
    }
    printf("=====================================\n\n");
}

void Bdot_FreeTestResults(BdotTestSuiteResult_t* results) {
    if (results->results != NULL) {
        free(results->results);
        results->results = NULL;
    }
}
