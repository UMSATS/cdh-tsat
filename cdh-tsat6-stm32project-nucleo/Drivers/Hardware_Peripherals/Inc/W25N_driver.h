/*
 * W25N_driver.h
 *
 *  Created on: Oct. 30, 2022
 *      Author: Owner
 */

#ifndef HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_

#define DEVICE_RESET                0xFF
#define READ_JEDEC_ID               0x9F
#define READ_STATUS_REGISTER        0x0F
#define WRITE_STATUS_REGISTER       0x1F
#define WRITE_ENABLE                0x06
#define WRITE_DISABLE               0x04
#define BAD_BLOCK_MANAGEMENT        0xA1
#define READ_BBM_LUT                0xA5
#define LAST_ECC_FAILURE_PAGE_ADR   0xA9
#define BLOCK_ERASE_128KB           0xD8
#define RANDOM_LOAD_PROGRAM_DATA    0x84
#define PROGRAM_EXECUTE             0x10
#define PAGE_DATA_READ              0x13
#define READ_DATA                   0x03

#endif /* HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_ */
