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
//Define Directives
//###############################################################################################
#define W25N_SPI          hspi1

#define W25N_nCS_GPIO     GPIOB
#define W25N_nCS_PIN      GPIO_PIN_15

#define W25N_nWP_GPIO     GPIOC
#define W25N_nWP_PIN      GPIO_PIN_6

#define W25N_nHOLD_GPIO   GPIOC
#define W25N_nHOLD_PIN    GPIO_PIN_7

#define W25N_SPI_DELAY    HAL_MAX_DELAY

#define W25N_DUMMY_BYTE                         0x00

#define W25N_OPCODE_DEVICE_RESET                0xFF
#define W25N_OPCODE_READ_JEDEC_ID               0x9F
#define W25N_OPCODE_READ_STATUS_REGISTER        0x0F
#define W25N_OPCODE_WRITE_STATUS_REGISTER       0x1F
#define W25N_OPCODE_WRITE_ENABLE                0x06
#define W25N_OPCODE_WRITE_DISABLE               0x04
#define W25N_OPCODE_BAD_BLOCK_MANAGEMENT        0xA1
#define W25N_OPCODE_READ_BBM_LUT                0xA5
#define W25N_OPCODE_BLOCK_ERASE_128KB           0xD8
#define W25N_OPCODE_LOAD_PROGRAM_DATA           0x02
#define W25N_OPCODE_PROGRAM_EXECUTE             0x10
#define W25N_OPCODE_PAGE_DATA_READ              0x13
#define W25N_OPCODE_READ_DATA                   0x03

#define W25N_BBM_LUT_NUM_OF_BYTES    80 //(4 bytes per entry) * (20 entries) = 80 bytes

//###############################################################################################
//Global Variable Declarations
//###############################################################################################
extern SPI_HandleTypeDef W25N_SPI;

//###############################################################################################
//Type Definitions
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
//Driver Function Prototypes
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
 * FUNCTION: W25N_Read_Status_Register
 *
 * DESCRIPTION: This function allows the 8-bit Status Registers to be read. The bits inside
 *              the status register then are shifted out on the DO pin at the falling edge
 *              of CLK with the most significant bit (MSB).
 *
 * NOTES:
 *  - This instruction may be used at any time, even while a Program or Erase Cycle is in
 *    progress, allowing the BUSY status bit to be checked to determine when the cycle is
 *    complete and if the device can handle another instruction.
 *  - The Status Register can be read continuously.
 *
 * PARAMETERS:
 *  register_address
 *  *p_buffer: Pointer to the buffer which will contain the BBM LUT.
 */
W25N_StatusTypeDef  W25N_Read_Status_Register(uint8_t register_address, uint8_t *p_buffer);

/*
 * FUNCTION: W25N_Write_Status_Register
 *
 * DESCRIPTION: The Write Status Register Instruction allows the Status Registers to be written.
 *
 * NOTES:
 *  -
 * PARAMETERS:
 *  register_address
 *  register_value
 */
W25N_StatusTypeDef W25N_Write_Status_Register(uint8_t register_address, uint8_t register_value);

/*
 * FUNCTION: W25N_Write_Enable
 *
 * DESCRIPTION: This function will set the Write Enable Latch.
 *
 * NOTES:
 *  -    The Write Enable Latch (WEL) must be set prior to every Page Program, Quad Page Program,
 *  	 Bad Block Management instruction.
 * PARAMETERS:
 *  No parameters.
 */
W25N_StatusTypeDef W25N_Write_Enable();

/*
 * FUNCTION: W25N_Write_Disable
 *
 * DESCRIPTION: This function will reset the Write Enable Latch bit in the Status Register to 0.
 *
 * NOTES:
 *  -
 * PARAMETERS:
 *  No parameters.
 */
W25N_StatusTypeDef W25N_Write_Disable();

/*
 * FUNCTION: W25N_Bad_Block_Management
 *
 * DESCRIPTION: Adds a link between a logical block address (LBA) and a physical block address
 *              (PBA) to the internal Bad Block Management (BBM) Look Up Table (LUT). The logical
 *              block address is the address for the bad block that will be replaced by the good
 *              block indicated by the physical block address.
 *
 * NOTES:
 *  - A Write Enable command must be executed before the device will accept the BBM command.
 *  - Up to 20 links can be established in the LUT. If all 20 links have been written, the LUT-F
 *    bit in the Status Register will become a 1, and no more LBA-PBA links can be established.
 *    Therefore, prior to issuing the BBM command, the LUT-F bit should be checked to confirm if
 *    spare links are still available in the LUT.
 *  - Registering the same address in multiple PBAs is prohibited.
 *
 * PARAMETERS:
 *  logical_block_address: Address of the bad block which will be replaced by the good block.
 *  physical_block_address: Address of the good block which will replace the bad block.
 */
W25N_StatusTypeDef W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address);

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

//NOTE: CHECK IF ACTUALLY PAGE ADDRESS (ANY PAGE ADDRESS WITHIN BLOCK WE WANT TO ERASE)
//OR BLOCK ADDRESS (DATASHEET SAYS PAGE ADDRESS)
/*
 * FUNCTION: W25N_Block_Erase_128KB
 *
 * DESCRIPTION: Sets all memory within a specified block (64Pages, 128KBytes) to the erased state
 *              of all 1s (0xFF).
 *
 * NOTES:
 *  - A Write Enable command must be executed before the device will accept the Block Erase
 *    command.
 *  - After executing this function, the self-timed Block Erase cycle will commence for a maximum
 *    time duration of 10 ms. While the Block Erase cycle is in progress, the Read Status
 *    Register command may still be used to check the BUSY bit. The BUSY bit is a 1 during the
 *    Block Erase cycle and becomes a 0 when the cycle is finished and the device is ready to
 *    accept other instructions again.
 *  - After the Block Erase cycle has finished, the Write Enable Latch (WEL) bit in the Status
 *    Register is cleared to 0.
 *  - The Block Erase command will not be executed if the addressed block is protected by the
 *    Block Protect (TB, BP2, BP1, BP0) bits.
 *
 * PARAMETERS:
 *  page_address:
 */
W25N_StatusTypeDef W25N_Block_Erase_128KB(uint16_t page_address);

/*
 * FUNCTION: W25N_Load_Program_Data
 *
 * DESCRIPTION: Loads data into the W25N Data Buffer from a given C buffer. Resets the unused
 *              data bytes in the W25N Data Buffer to 0xFF.
 *
 * NOTES:
 *  - A Write Enable command must be executed before the device will accept the Load Program Data
 *    command.
 *  - The 0th data byte in the C buffer will be loaded into the W25N Data Buffer at the given
 *    column address. The 1st data byte will be loaded at the given column address + 1, etc.
 *  - ECC will be used for each page, making each page effectively have 2048 data bytes.
 *    Therefore, the W25N Data Buffer will also effectively have 2048 data bytes. Any sent data
 *    bytes that exceed this addressable range will be discarded.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the C buffer which contains the data bytes to load.
 *  column_address: W25N Data Buffer memory address for the 0th byte to load.
 *  num_of_bytes: Number of bytes to load from the C buffer into the W25N Data Buffer.
 */
W25N_StatusTypeDef W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);

/*
 * FUNCTION: W25N_Program_Execute
 *
 * DESCRIPTION: Programs the W25N Data Buffer contents into the specified physical memory page.
 *
 * NOTES:
 *  - A Write Enable command must be executed before the device will accept the Program Execute
 *    command. However, note that the Load Program Data command does not clear the Write Enable
 *    Latch (WEL) bit in the Status Register.
 *  - After executing this function, the self-timed Program Execute cycle will commence for a
 *    maximum time duration of 700 us. While the Program Execute cycle is in progress, the Read
 *    Status Register command may still be used to check the BUSY bit. The BUSY bit is a 1 during
 *    the Program Execute cycle and becomes a 0 when the cycle is finished and the device is
 *    ready to accept other instructions again.
 *  - After the Program Execute cycle has finished, the WEL bit in the Status Register is cleared
 *    to 0.
 *  - The Program Execute command will not be executed if the addressed block is protected by the
 *    Block Protect (TB, BP2, BP1, BP0) bits.
 *  - Only 4 partial page programs are allowed on every single page. After 4 partial page
 *    programs are completed on the same page, the page must be erased before being programmed
 *    again. This is due to successive partial page programs increasing the bit error rate.
 *  - Pages within a block have to be programmed sequentially from lower order page addresses to
 *    higher order page addresses. Programming pages out of sequence is prohibited. This is due
 *    to programming pages out of sequence increasing the bit error rate.
 *
 * PARAMETERS:
 *  page_address: Address of the physical memory page to program the W25N Data Buffer into.
 */
W25N_StatusTypeDef W25N_Program_Execute(uint16_t page_address);

/*
 * FUNCTION: W25N_Page_Data_Read
 *
 * DESCRIPTION: Transfers the data of the specified memory page into the W25N Data Buffer.
 *
 * NOTES:
 *  - After executing this function, the self-timed Read Page Data cycle will commence for a
 *    maximum time duration of 60 us. While the Read Page Data cycle is in progress, the Read
 *    Status Register command may still be used to check the BUSY bit. The BUSY bit is a 1 during
 *    the Read Page Data cycle and becomes a 0 when the cycle is finished and the device is ready
 *    to accept other instructions again.
 *
 * PARAMETERS:
 *  page_address: Address of the physical memory page to transfer into the W25N Data Buffer.
 */
W25N_StatusTypeDef W25N_Page_Data_Read(uint16_t page_address);

/*
 * FUNCTION: W25N_Read_Data
 *
 * DESCRIPTION: Reads one or more data bytes from the W25N Data Buffer and stores the result in a
 *              given C buffer.
 *
 * NOTES:
 *  - ECC will be used for each page, making the W25N Data Buffer effectively 2048 data bytes
 *    followed by 64 ECC bytes (2112 total bytes).
 *  - Buffer Read Mode will be used, therefore once the last data byte is output from the W25N
 *    Data Buffer, the output pin will become Hi-Z state.
 *  - The 0th data byte in the C buffer will be loaded from the W25N Data Buffer at the given
 *    column address. The 1st data byte will be loaded from the W25N Data Buffer at the given
 *    column address + 1, etc.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the C buffer which will contain the output data bytes.
 *  column_address: W25N Data Buffer memory address for the 0th byte to output.
 *  num_of_bytes: Number of bytes to output from the W25N Data Buffer into the C buffer.
 */
W25N_StatusTypeDef W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);

//###############################################################################################
//High-Level Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_One_Time_Init
 *
 * DESCRIPTION:
 *
 * NOTES:
 *  - Note1
 *  - Note2
 *  - Note3
 *
 * PARAMETERS:
 *  parameter1:
 *  parameter2:
 *  parameter3:
 */
void W25N_One_Time_Init();

//###############################################################################################
//Helper Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_SPI_Transmit_Word_16Bit
 *
 * DESCRIPTION: Transmits a 16-bit word to the W25N Flash.
 *
 * PARAMETERS:
 *  word_16bit: The 16-bit word to transmit to the W25N Flash.
 */
W25N_StatusTypeDef W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit);

#endif /* HARDWARE_PERIPHERALS_INC_W25N_DRIVER_H_ */
