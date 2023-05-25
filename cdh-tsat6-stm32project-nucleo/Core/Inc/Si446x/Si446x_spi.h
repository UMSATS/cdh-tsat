/*
 * Project: Si4463 Radio Library for AVR and Arduino
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/si4463-radio-library-avr-arduino/
 */

#ifndef SI446X_SPI_H_
#define SI446X_SPI_H_

#include "Si446x/Si446x_defs.h"

/*
 * Ensures SPI bus is initialized after STM32 HAL does its job. -NJR
 */
void spi_init(void);


/*
 * Transmits (and receives) SPI data over bus.
 */
uint8_t spi_transfer(uint8_t data);

/*
 * Transmits SPI Data. A wrapper for spi_transfer().
 */
void spi_transfer_nr(uint8_t data);


/*
 * Handles CS Line.
 */
uint8_t cselect(void);

uint8_t cdeselect(void);

// Makes use of the behaviour of the for loop conditionals run code in block once while Chipselected, then switching it off at the end.
#define CHIPSELECT()	for(uint8_t _cs = cselect(); _cs; _cs = cdeselect())

#endif /* SI446X_SPI_H_ */
