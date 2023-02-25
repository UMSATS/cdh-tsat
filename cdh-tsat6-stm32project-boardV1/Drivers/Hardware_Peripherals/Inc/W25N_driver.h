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
#define W25N_SPI          hspi1

#define W25N_nCS_GPIO     GPIOB
#define W25N_nCS_PIN      GPIO_PIN_15

#define W25N_nWP_GPIO     GPIOC
#define W25N_nWP_PIN      GPIO_PIN_6

#define W25N_nHOLD_GPIO   GPIOC
#define W25N_nHOLD_PIN    GPIO_PIN_7

#define W25N_SPI_DELAY    HAL_MAX_DELAY

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
 * FUNCTION: W25N_Read_JEDEC_ID
 *
 * DESCRIPTION: Reads the JEDEC ID of the W25N device.
 *
 * NOTES:
 *  - The JEDEC ID of the W25N01GVZEIG NAND Flash is 0xEFAA21
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the buffer which will contain the outputted JEDEC ID.
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
 * DESCRIPTION: Initializes the W25N.
 *
 * NOTES:
 *  - This high-level function ensures the internal initialization is complete by delaying for 10 ms.
 *  - The states of nCS, nWP, & nHOLD are set:
 *     nCS is set HIGH: this deselects the W25N
 *     nWP is set LOW: this write protects the W25N
 *     nHOLD is set HIGH: this disables the hold state, allowing normal device operations
 *  - The states of the Status Register bits are set:
 *     SR-1: SR-1 can be changed, all blocks are unlocked for writing, nWP will be used for 
 *           write protection
 *     SR-2: The OTP area is not locked, OTP access mode is not active, SR-1 is not locked, ECC
 *           is enabled, the W25N is in buffer read mode
 */
W25N_StatusTypeDef W25N_Init();

/*
 * FUNCTION: W25N_Establish_BBM_Link
 *
 * DESCRIPTION: Adds a link between a logical block address (LBA) and a physical block address
 *              (PBA) to the internal Bad Block Management (BBM) Look Up Table (LUT). The logical
 *              block address is the address for the bad block that will be replaced by the good
 *              block indicated by the physical block address.
 *
 * NOTES:
 *  - This high-level function waits until the W25N is not busy & sets the WEL before establishing 
 *    the BBM link.
 *  - Up to 20 links can be established in the LUT. If all 20 links have been written, the LUT-F
 *    bit in the Status Register will become a 1, and no more LBA-PBA links can be established.
 *    Therefore, prior to issuing the BBM command, the LUT-F bit should be checked to confirm if
 *    spare links are still available in the LUT.
 *  - Registering the same address in multiple PBAs is prohibited.
 *  - The parameters are BLOCK ADDRESSES, not page addresses within the specific block
 *
 * PARAMETERS:
 *  logical_block_address: Address of the bad block which will be replaced by the good block.
 *  physical_block_address: Address of the good block which will replace the bad block.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_HANGING: The W25N is hanging since it was busy for longer than 10ms.
 *  W25N_LUT_FULL: The BBM LUT is full.
 */
W25N_StatusTypeDef W25N_Establish_BBM_Link(uint16_t logical_block_address, uint16_t physical_block_address);

/*
 * FUNCTION: W25N_Read
 *
 * DESCRIPTION: Read data from a single physical memory page and store the result in the given buffer.
 *
 * NOTES:
 *  - This high-level function waits until the W25N is not busy before issuing the read commands.
 *  - This high-level function checks the ECC status of the data after the read is complete.
 *  - ECC will be used for each page, making each page effectively 2048 data bytes followed by 64 
 *    ECC bytes (2112 total bytes).
 *  - The 0th data byte in the buffer will be loaded from the page at the given column address. 
 *    The 1st data byte in the buffer will be loaded from the page at the given column address + 1, etc.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the buffer which will contain the output data bytes.
 *  page_address: Address of the physical memory page to read the data from.
 *  column_address: Starting byte address within the physical memory page to read the data from.
 *  num_of_bytes: Number of bytes to read from the physical memory page.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_HANGING: The W25N is hanging since it was busy for longer than 10ms.
 *  W25N_ECC_CORRECTION_UNNECESSARY: No ECC correction was necessary.
 *  W25N_ECC_CORRECTION_OK: There were 1-4 bit errors for the page which were successfully corrected.
 *  W25N_ECC_CORRECTION_ERROR: ECC correction was unsuccessful since there were more than 4 bit 
 *                             errors for the page.
 */
W25N_StatusTypeDef W25N_Read(uint8_t *p_buffer, uint16_t page_address, uint16_t column_address, uint16_t num_of_bytes);

/*
 * FUNCTION: W25N_Write
 *
 * DESCRIPTION: Write data to a single physical memory page from the given buffer.
 *
 * NOTES:
 *  - This high-level function waits until the W25N is not busy & sets the WEL before issuing the 
 *    write commands.
 *  - This high-level function checks if the write operation was successful or not after the write 
 *    is complete.
 *  - The 0th data byte in the buffer will be loaded into the page at the given column address. 
 *    The 1st data byte in the buffer will be loaded into the page at the given column address + 1, etc.
 *  - ECC will be used for each page, making each page effectively 2048 data bytes followed by 64 
 *    ECC bytes (2112 total bytes). Any sent data bytes that exceed this addressable range will be 
 *    discarded.
 *  - The write operation will not be executed if the addressed page's block is protected by the 
 *    Block Protect (TB, BP2, BP1, BP0) bits.
 *  - Only 4 partial page programs are allowed on every single page. After 4 partial page
 *    programs are completed on the same page, the page must be erased before being programmed
 *    again. This is due to successive partial page programs increasing the bit error rate.
 *  - Pages within a block have to be programmed sequentially from lower order page addresses to
 *    higher order page addresses. Programming pages out of sequence is prohibited. This is due
 *    to programming pages out of sequence increasing the bit error rate.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the buffer which contains the data bytes to write.
 *  page_address: Address of the physical memory page to write the data to.
 *  column_address: Starting byte address within the physical memory page to write the data to.
 *  num_of_bytes: Number of bytes to write to the physical memory page.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_HANGING: The W25N is hanging since it was busy for longer than 10ms.
 *  W25N_PROGRAM_OK: The program operation was successful.
 *  W25N_PROGRAM_ERROR: The program operation was unsuccessful.
 */
W25N_StatusTypeDef W25N_Write(uint8_t *p_buffer, uint16_t page_address, uint16_t column_address, uint16_t num_of_bytes);

/*
 * FUNCTION: W25N_Erase
 *
 * DESCRIPTION: Sets all memory within a specified block (64Pages, 128KBytes) to the erased state
 *              of all 1s (0xFF).
 *
 * NOTES:
 *  - This high-level function waits until the W25N is not busy & sets the WEL before issuing the 
 *    erase command.
 *  - This high-level function checks if the erase operation was successful or not after the erase 
 *    is complete.
 *  - The erase operation will not be executed if the addressed block is protected by the Block 
 *    Protect (TB, BP2, BP1, BP0) bits.
 *
 * PARAMETERS:
 *  page_address: Address of the page within the block to be erased. Using any page address within the 
 *                block will erase the entire block. For example, using any address from 0xFFC0 to 0xFFFF
 *                will erase the entire block which contains the pages from 0xFFc0 to 0xFFFF.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_HANGING: The W25N is hanging since it was busy for longer than 10ms.
 *  W25N_ERASE_OK: The erase operation was successful.
 *  W25N_ERASE_ERROR: The erase operation was unsuccessful.
 */
W25N_StatusTypeDef W25N_Erase(uint16_t page_address);

/*
 * FUNCTION: W25N_Reset_And_Init
 *
 * DESCRIPTION: Terminates current internal operations and allows the device to return to its 
 *              default power-on state and lose all the current volatile settings, then initializes
 *              the volatile settings.
 *
 * NOTES:
 *  - This high-level function waits until the W25N is not busy before issuing the reset command.
 *  - This high-level function will not reset the W25N if it is hanging. If the W25N is hanging, 
 *    the W25N will have to be power cycled. Note that data corruption may happen if there is an 
 *    ongoing internal Erase or Program operation. It is recommended by the manufacturer to ensure 
 *    the BUSY bit in Status Register is 0 before issuing the Reset command. However, a power cycle 
 *    may be required if the W25N is hanging.
 *  - This high-level function ensures the internal reset is complete by delaying for 1 ms. (tRST can be 5us - 500us)
 *  - After reset, the states of nCS, nWP, & nHOLD are set:
 *     nCS is set HIGH: this deselects the W25N
 *     nWP is set LOW: this write protects the W25N
 *     nHOLD is set HIGH: this disables the hold state, allowing normal device operations
 *  - After reset, the states of the Status Register bits are set:
 *     SR-1: SR-1 can be changed, all blocks are unlocked for writing, nWP will be used for 
 *           write protection
 *     SR-2: The OTP area is not locked, OTP access mode is not active, SR-1 is not locked, ECC
 *           is enabled, the W25N is in buffer read mode
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_HANGING: The W25N is hanging since it was busy for longer than 10ms.
 */
W25N_StatusTypeDef W25N_Reset_And_Init();

//###############################################################################################
//Public Helper Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_Page_Address_To_Block_Address
 *
 * DESCRIPTION: Returns the block address of the block which contains the specified physical 
 *              memory page.
 *
 * PARAMETERS:
 *  page_address: The physical memory page address.
 */
uint16_t W25N_Page_Address_To_Block_Address(uint16_t page_address);

#endif /* HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_ */
