/*
 *
 *
 *
 *
 *
 *
 *
*/
#pragma once

#include <dhara/error.h>

//Dhara NAND Struct is located in .c file

extern struct dhara_map my_map;
extern const struct dhara_nand my_nand;

dhara_error_t Dhara_Init(void);
