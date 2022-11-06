/*
 * W25N_driver.h
 *
 *  Created on: Oct. 30, 2022
 *      Author: Owner
 */

#ifndef HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_

//Define Directives
#define W25N_SPI          hspi1

#define W25N_nCS_GPIO     GPIOB
#define W25N_nCS_PIN      GPIO_PIN_15

#define W25N_nWP_GPIO     GPIOC
#define W25N_nWP_PIN      GPIO_PIN_6

#define W25N_nHOLD_GPIO   GPIOC
#define W25N_nHOLD_PIN    GPIO_PIN_7

#define W25N_DUMMY_BYTE                    0x00

#define OPCODE_DEVICE_RESET                0xFF
#define OPCODE_READ_JEDEC_ID               0x9F
#define OPCODE_READ_STATUS_REGISTER        0x0F
#define OPCODE_WRITE_STATUS_REGISTER       0x1F
#define OPCODE_WRITE_ENABLE                0x06
#define OPCODE_WRITE_DISABLE               0x04
#define OPCODE_BAD_BLOCK_MANAGEMENT        0xA1
#define OPCODE_READ_BBM_LUT                0xA5
#define OPCODE_BLOCK_ERASE_128KB           0xD8
#define OPCODE_LOAD_PROGRAM_DATA           0x02
#define OPCODE_PROGRAM_EXECUTE             0x10
#define OPCODE_PAGE_DATA_READ              0x13
#define OPCODE_READ_DATA                   0x03

#define BBM_LUT_NUM_OF_BYTES    80 //(4 bytes per entry) * (20 entries) = 80 bytes

//Global Variable Declarations
extern SPI_HandleTypeDef W25N_SPI;

//Function Prototypes
void W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address);
void W25N_Read_BBM_LUT(uint8_t *p_buffer);
void W25N_Block_Erase_128KB(uint16_t page_address);
void W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);
void W25N_Program_Execute(uint16_t page_address);
void W25N_Page_Data_Read(uint16_t page_address);
void W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);

void W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit);

#endif /* HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_ */
