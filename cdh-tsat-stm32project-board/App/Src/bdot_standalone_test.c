/*
 * FILENAME: bdot_standalone_test.c
 *
 * DESCRIPTION: Standalone test runner for B-dot algorithm
 *              This can be used for development and debugging without the full system
 *
 * AUTHORS:
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * Created on: Jan. 7, 2025
 */

#include "bdot_algorithm_test.h"
#include <stdio.h>

/*
 * Standalone test runner - can be called from main() for testing
 */
void Bdot_RunStandaloneTests(void) {
    printf("Starting B-dot Algorithm Standalone Tests...\n");
    printf("==========================================\n");
    
    // Run all tests
    BdotTestSuiteResult_t results = Bdot_RunAllTests();
    
    // Print results
    Bdot_PrintTestResults(&results);
    
    // Clean up
    Bdot_FreeTestResults(&results);
    
    printf("B-dot Algorithm Tests Complete!\n");
    printf("==========================================\n");
}

/*
 * Quick test function for individual test verification
 */
void Bdot_RunQuickTest(void) {
    printf("Running Quick B-dot Test...\n");
    
    // Test exponential filter
    BdotTestResult_t filter_test = Bdot_Test_ExponentialFilter();
    printf("Exponential Filter Test: %s\n", filter_test.passed ? "PASS" : "FAIL");
    
    // Test initialization
    BdotTestResult_t init_test = Bdot_Test_Initialization();
    printf("Initialization Test: %s\n", init_test.passed ? "PASS" : "FAIL");
    
    // Test static field
    BdotTestResult_t static_test = Bdot_Test_StaticField();
    printf("Static Field Test: %s\n", static_test.passed ? "PASS" : "FAIL");
    
    printf("Quick Test Complete!\n");
}
