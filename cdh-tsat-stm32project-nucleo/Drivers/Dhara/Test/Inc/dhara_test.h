/*
 * Dhara_test.h
 *
 * Author
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 *  Created on: Nov 20, 2025
*/

#pragma once

#include <dhara/error.h>


//############################################################################
//########################    NAND.C FUNCTION TEST    ########################
//############################################################################

dhara_error_t Dhara_Test_NAND_Write();

dhara_error_t Dhara_Test_NAND_Read();

dhara_error_t Dhara_Test_NAND_Erase();

dhara_error_t Dhara_Test_NAND_Is_Bad();

dhara_error_t Dhara_Test_NAND_Mark_As_Bad();

dhara_error_t Dhara_Test_NAND_Is_Free();

dhara_error_t Dhara_Test_NAND_Copy();


//###################################################################
//########################    NAND.C TEST    ########################
//###################################################################

dhara_error_t Dhara_Test_NAND();


//#############################################################################
//########################    WRAPPER FUNCTION TEST    ########################
//#############################################################################

dhara_error_t Dhara_Test_Wrapper_Clear();

dhara_error_t Dhara_Test_Wrapper_Capacity();

dhara_error_t Dhara_Test_Wrapper_Size();

dhara_error_t Dhara_Test_Wrapper_Find();

dhara_error_t Dhara_Test_Wrapper_Read();

dhara_error_t Dhara_Test_Wrapper_Write();

dhara_error_t Dhara_Test_Wrapper_Copy_Page();

dhara_error_t Dhara_Test_Wrapper_Copy_Sector();

dhara_error_t Dhara_Test_Wrapper_Erase();

dhara_error_t Dhara_Test_Wrapper_Force_Sync();

dhara_error_t Dhara_Test_Wrapper_GC();


//####################################################################
//########################    WRAPPER TEST    ########################
//####################################################################

dhara_error_t Dhara_Test_Wrapper();


//#######################################################################
//########################    SIMULATION TEST    ########################
//#######################################################################

dhara_error_t Dhara_Test_Simulated_Power_Loss();


//##########################################################################
//########################    FULL TEST FUNCTION    ########################
//##########################################################################

dhara_error_t Dhara_Test();
