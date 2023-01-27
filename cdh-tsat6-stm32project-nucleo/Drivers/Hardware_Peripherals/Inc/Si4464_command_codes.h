/*
 * Si4464_command_codes.h
 *
 *  Created on: Jan 27, 2023
 *      Author: Graham Driver
 *
 *	Description: This header contains command codes associated with the
 *	Si4464 SPI API.
 */

#ifndef HARDWARE_PERIPHERALS_INC_SI4464_COMMAND_CODES_H_
#define HARDWARE_PERIPHERALS_INC_SI4464_COMMAND_CODES_H_

#define SI4464_POWER_UP 			0x02;
#define SI4464_PATCH_IMAGE			0x04;
#define SI4464_NOP					0x00;
#define SI4464_PART_INFO			0X01;
#define SI4464_FUNC_INFO			0x10;
#define SI4464_SET_PROPERTY			0x11;
#define SI4464_GET_PROPERTY			0x12;
#define SI4464_GPIO_PIN_CFG			0x13;
#define SI4464_FIFO_INFO			0x15;
#define SI4464_IRCAL				0x17;
#define SI4464_GET_INT_STATUS		0x20;
#define SI4464_REQUEST_DEVICE_STATE	0x33;
#define SI4464_CHANGE_STATE			0x34;
#define SI4464_GET_ADC_READING		0x14;
#define SI4464_GET_PACKET_INFO		0x16;
#define SI4464_PROTOCOL_CFG			0x18;
#define SI4464_GET_PH_STATUS		0x21;
#define SI4464_GET_MODEM_STATUS		0x22;
#define SI4464_GET_CHIP_STATUS		0x23;
#define SI4464_RX_HOP				0x36;
#define SI4464_FRR_A				0x50;
#define SI4464_FRR_B				0x51;
#define SI4464_FRR_C				0x53;
#define SI4464_FEE_D				0x57;
#define SI4464_READ_COMMAND_BUFFER	0x44;
#define SI4464_TX_FIFO_WRITE		0x66;
#define SI4464_RX_FIFO_READ			0x77;
#define SI4464_START_TX				0x31;
#define SI4464_START_RX				0x32;


#endif /* HARDWARE_PERIPHERALS_INC_SI4464_COMMAND_CODES_H_ */
