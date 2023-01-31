/*
 * FILENAME: W25N_driver.h
 *
 * DESCRIPTION: STM32L4 driver header file for the W25N01GVZEIG NAND Flash.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * CREATED ON: Oct. 30, 2022
 */

#ifndef HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_
#define HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>
#include "stm32l4xx_hal.h"

//###############################################################################################
//Public Define Directives
//###############################################################################################
#define W25N_PAGES_PER_BLOCK         64
#define W25N_BBM_LUT_NUM_OF_BYTES    80 //(4 bytes per entry) * (20 entries) = 80 bytes

//###############################################################################################
//Public Type Definitions
//###############################################################################################
typedef enum
{
    W25N_HAL_OK                     = HAL_OK,      //0x00
    W25N_HAL_ERROR                  = HAL_ERROR,   //0x01
    W25N_HAL_BUSY                   = HAL_BUSY,    //0x02
    W25N_HAL_TIMEOUT                = HAL_TIMEOUT, //0x03
    W25N_READY                      = 0x04,
    W25N_HANGING                    = 0x05,
    W25N_LUT_HAS_ROOM               = 0x06,
    W25N_LUT_FULL                   = 0x07,
    W25N_ECC_CORRECTION_UNNECESSARY = 0x08,
    W25N_ECC_CORRECTION_OK          = 0x09,
    W25N_ECC_CORRECTION_ERROR       = 0x0A,
    W25N_PROGRAM_OK                 = 0x0B,
    W25N_PROGRAM_ERROR              = 0x0C,
    W25N_ERASE_OK                   = 0x0D,
    W25N_ERASE_ERROR                = 0x0E
} W25N_StatusTypeDef;

//###############################################################################################
//Public Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_Device_Reset
 *
 * DESCRIPTION: Terminates current internal operations and allows the device to return to
 *              its default power -on state and lose all the current volatile settings.
 *
 * NOTES:
 *  - Device will take approximately tRST to reset. (tRST can be 5us - 500us)
 *  - Recommended to check the BUSY bit in Status Register before issuing the Reset
 *    command.
 *  - Data corruption may happen if there is an ongoing internal Erase or Program operation.
 *    Its recommended by the manufacturer to check the BUSY bit in Status Register before
 *    issuing the Reset command.
 *
 * PARAMETERS: No parameters.
 */
W25N_StatusTypeDef W25N_Device_Reset();

/*
 * FUNCTION: W25N_Read_JEDEC_ID
 *
 * DESCRIPTION: Reads the JEDEC ID allowing debugging.
 *
 * NOTES:
 *  - This function can only read straight bytes.
 *
 * PARAMETERS:
 *  *p_buffer
 */
W25N_StatusTypeDef W25N_Read_JEDEC_ID(uint8_t *p_buffer);

/*
 * FUNCTION: W25N_Read_BBM_LUT
 *
 * DESCRIPTION: Reads the internal Bad Block Management (BBM) Look Up Table (LUT) and stores the
 *              result in a given buffer.
 *
 * NOTES:
 *  - Data is filled into the buffer beginning with the 0th 16-bit logical block address (LBA),
 *    then the 0th 16-bit physical block address, then the 1st LBA, then the 1st PBA, ..., then
 *    the 19th LBA, then the 19th PBA. This is a total of 20 LBA-PBA links for a total of 80
 *    bytes.
 *  - Available unused LBA-PBA links will contain all 0x0000 data.
 *  - The MSB bits LBA[15:14] of each link are used to indicate the status of the link:
 *      00 - The link is available to use.
 *      10 - This link is enabled and it is a valid link.
 *      11 - This link was enabled, but it is not valid any more.
 *      01 - Not applicable.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the buffer which will contain the BBM LUT. This buffer should have a
 *            size of at least 80 bytes in order to store the entire BBM LUT.
 */
W25N_StatusTypeDef W25N_Read_BBM_LUT(uint8_t *p_buffer);

//###############################################################################################
//Public High-Level Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_Init
 *
 * DESCRIPTION: 
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 */
W25N_StatusTypeDef W25N_Init();

/*
 * FUNCTION: W25N_Establish_BBM_Link
 *
 * DESCRIPTION: 
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 *
 * PARAMETERS:
 *  logical_block_address: ParameterNote1
 *  physical_block_address: ParameterNote2
 */
W25N_StatusTypeDef W25N_Establish_BBM_Link(uint16_t logical_block_address, uint16_t physical_block_address);

/*
 * FUNCTION: W25N_Read
 *
 * DESCRIPTION: 
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 *
 * PARAMETERS:
 *  p_buffer: ParameterNote1
 *  page_address: ParameterNote2
 *  column_address: ParameterNote3
 *  num_of_bytes: ParameterNote4
 */
W25N_StatusTypeDef W25N_Read(uint8_t *p_buffer, uint16_t page_address, uint16_t column_address, uint16_t num_of_bytes);

/*
 * FUNCTION: W25N_Write
 *
 * DESCRIPTION: 
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 *
 * PARAMETERS:
 *  p_buffer: ParameterNote1
 *  page_address: ParameterNote2
 *  column_address: ParameterNote3
 *  num_of_bytes: ParameterNote4
 */
W25N_StatusTypeDef W25N_Write(uint8_t *p_buffer, uint16_t page_address, uint16_t column_address, uint16_t num_of_bytes);

/*
 * FUNCTION: W25N_Erase
 *
 * DESCRIPTION: 
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 *
 * PARAMETERS:
 *  page_address: ParameterNote1
 */
W25N_StatusTypeDef W25N_Erase(uint16_t page_address);

//###############################################################################################
//Public Helper Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_Page_Address_To_Block_Address
 *
 * DESCRIPTION: 
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 *
 * PARAMETERS:
 *  page_address: ParameterNote1
 */
uint16_t W25N_Page_Address_To_Block_Address(uint16_t page_address);

#endif /* HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_ */
