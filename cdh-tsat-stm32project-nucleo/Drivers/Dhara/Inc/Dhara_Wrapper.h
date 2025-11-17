/*
 * FILENAME: Dhara_Wrapper.h
 *
 * DESCRIPTION: Contains the wrapper functions that will be used to interact with the Dhara library
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Oct. 4 2025
 */

#pragma once

//For Nand struct
#define NAND_LOG2_PAGE_SIZE 11
#define NAND_LOG2_PAGE_PER_BLOCK 6
#define NAND_NUM_OF_BLOCKS 1024
#define NAND_NUM_OF_SPARES 20
#define NAND_NUM_AVAILABLE_BLOCKS NAND_NUM_OF_BLOCKS-NAND_NUM_OF_SPARES
//For Journal size
#define PAGE_SIZE   (1 << NAND_LOG2_PAGE_SIZE)

//###############################################################################################
//Include Directives
//###############################################################################################
#include <dhara/error.h>

//Dhara NAND Struct is located in .c file

extern struct dhara_map my_map;
extern const struct dhara_nand my_nand;

dhara_error_t Dhara_Init(void);
