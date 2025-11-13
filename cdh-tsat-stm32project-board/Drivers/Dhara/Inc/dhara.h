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


#include <dhara/nand.h>
#include <dhara/map.h>

#include "W25N_driver.h"

//Dhara NAND Struct is located in .c file

extern struct dhara_map my_map;
extern const struct dhara_nand my_nand;

void flash_init(void);
