/*
 * Project: Si4463 Radio Library for AVR and Arduino
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/si4463-radio-library-avr-arduino/
 */

#include "stm32l4xx_hal.h"
#include "Si446x/Si446x_spi.h"
#include "Si446x/Si446x_config.h"

// NOTE: The SPI SS pin must be configured as an output for the SPI controller to run in master mode, even if you're using a different pin for SS!
// Also remember that some AVRs use different registers to enable pull-ups
// Don't forget to check Si446x_config.h for the CSN, SDN and IRQ pin setup

// SS = Output high
// MOSI = Output low
// MISO = Input with pullup
// SCK = Output low
// Max SPI clock of Si446x is 10MHz

// TODO: Setup registers for SPI1_CR relating to correct port values, speeds, modes, etc...


void spi_init()
{
	SPI_PORT->CR1 |= SPI_CR1_SPE; // Enable SPI.
}

uint8_t cselect(void)
{
	CSN_PORT->BSRR = 1 << (SI446X_CSN_BIT + 16); // Bitshifts left to lower half of register.
	return 1;
}

uint8_t cdeselect(void)
{
	CSN_PORT->BSRR = 1 << (SI446X_CSN_BIT); // Bitshifts left to the upper half of the register.
	return 0;
}

uint8_t spi_transfer(uint8_t data)
{
    *(volatile uint8_t *) &SPI_PORT->DR = data; // Transmit
    while((SPI_PORT->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE)
        ;

    // Adapted from STM32 HAL files.
    return *(volatile uint8_t *)&SPI_PORT->DR; // Receive
}

void spi_transfer_nr(uint8_t data)
{
	(void) spi_transfer(data);
}
