/*
 * FILENAME: W25N_driver.c
 *
 * DESCRIPTION: STM32L4 driver source file for the W25N01GVZEIG NAND Flash.
 *
 * AUTHORS:
 *  - Daigh Burgess (daigh.burgess@umsats.ca)
 *  - Rodrigo Alegria (rodrigo.alegria@umsats.ca)
 *
 * CREATED ON: Oct. 30, 2022
 */

//###############################################################################################
//Include Directives
//###############################################################################################
#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "W25N_driver.h"

//###############################################################################################
//Define Directives
//###############################################################################################
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

//###############################################################################################
//Global Variable Declarations
//###############################################################################################
extern SPI_HandleTypeDef W25N_SPI;

//###############################################################################################
//Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_Device_Reset
 *
 * DESCRIPTION: Terminates current internal operations and allows the device to return to its 
 *              default power-on state and lose all the current volatile settings.
 *
 * NOTES:
 *  - This function ensures the internal reset is complete by delaying for 1 ms. (tRST can be 5us - 500us)
 *  - Data corruption may happen if there is an ongoing internal Erase or Program operation.
 *    It is recommended by the manufacturer to ensure the BUSY bit in Status Register is 0 before
 *    issuing the Reset command. THIS FUNCTION DOES NOT CHECK THE BUSY BIT BEFORE RESETTING.
 */
W25N_StatusTypeDef W25N_Device_Reset();

/*
 * FUNCTION: W25N_Read_Status_Register
 *
 * DESCRIPTION: Reads the specified 8-bit Status Register.
 *
 * NOTES:
 *  - This instruction may be used at any time, even while the BUSY status bit is 1.
 *  - The Status Register can be read continuously.
 *
 * PARAMETERS:
 *  register_address: The Status Register address. (SR-1 : 0xA0, SR-2 : 0xB0, SR-3 : 0xC0)
 *  p_buffer: Pointer to the buffer which will contain the outputted Status Register byte.
 */
W25N_StatusTypeDef W25N_Read_Status_Register(uint8_t register_address, uint8_t *p_buffer);

/*
 * FUNCTION: W25N_Write_Status_Register
 *
 * DESCRIPTION: Write to the specified 8-bit Status Register.
 *
 * NOTES:
 *  - The writable Status Register bits include:
 *      Status Register-1: SRP[1:0], TB, BP[3:0], WP-E
 *      Status Register-2: OTP-L, OTP-E, SR1-L, ECC-E, BUF
 *  - All other Status Register bits are read-only and will not be affected by the Write Status 
 *    Register instruction.
 * 
 * PARAMETERS:
 *  register_address: The Status Register address. (SR-1 : 0xA0, SR-2 : 0xB0, SR-3 : 0xC0)
 *  register_value: The value to be written to the Status Register.
 */
W25N_StatusTypeDef W25N_Write_Status_Register(uint8_t register_address, uint8_t register_value);

/*
 * FUNCTION: W25N_Write_Enable
 *
 * DESCRIPTION: Sets the Write Enable Latch (WEL) in Status Register-1 to the value 1.
 *
 * NOTES:
 *  - The WEL must be set prior to every Load Program Data, Program Execute, Block Erase, & Bad Block Management instruction.
 *  - The WEL does not need to be set prior to a Write Status Register instruction.
 */
W25N_StatusTypeDef W25N_Write_Enable();

/*
 * FUNCTION: W25N_Write_Disable
 *
 * DESCRIPTION: Resets the Write Enable Latch (WEL) in Status Register-1 to the value 0.
 *
 * NOTES:
 *  - A WEL value of 0 will prevent the Page Program, Block Erase, & Bad Block Management instructions 
 *    from executing.
 *  - The WEL is automatically reset after Power-up.
 *  - The WEL is automatically reset after completion of the Page Program, Block Erase, & Reset 
 *    instructions.
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
 *  - The parameters are BLOCK ADDRESSES, not page addresses within the specific block
 *
 * PARAMETERS:
 *  logical_block_address: Address of the bad block which will be replaced by the good block.
 *  physical_block_address: Address of the good block which will replace the bad block.
 */
W25N_StatusTypeDef W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address);

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
 *  page_address: Address of the page within the block to be erased. Using any page address within the 
 *                block will erase the entire block. For example, using any address from 0xFFC0 to 0xFFFF
 *                will erase the entire block which contains the pages from 0xFFc0 to 0xFFFF.
 */
W25N_StatusTypeDef W25N_Block_Erase_128KB(uint16_t page_address);

/*
 * FUNCTION: W25N_Load_Program_Data
 *
 * DESCRIPTION: Loads data into the W25N Data Buffer from a given buffer. Resets the unused data
 *              bytes in the W25N Data Buffer to 0xFF.
 *
 * NOTES:
 *  - A Write Enable command must be executed before the device will accept the Load Program Data
 *    command.
 *  - The 0th data byte in the buffer will be loaded into the W25N Data Buffer at the given
 *    column address. The 1st data byte will be loaded at the given column address + 1, etc.
 *  - ECC will be used for each page, making each page effectively have 2048 data bytes.
 *    Therefore, the W25N Data Buffer will also effectively have 2048 data bytes. Any sent data
 *    bytes that exceed this addressable range will be discarded.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the buffer which contains the data bytes to load.
 *  column_address: W25N Data Buffer memory address for the 0th byte to load.
 *  num_of_bytes: Number of bytes to load from the buffer into the W25N Data Buffer.
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
 *              given buffer.
 *
 * NOTES:
 *  - ECC will be used for each page, making the W25N Data Buffer effectively 2048 data bytes
 *    followed by 64 ECC bytes (2112 total bytes).
 *  - Buffer Read Mode will be used, therefore once the last data byte is output from the W25N
 *    Data Buffer, the output pin will become Hi-Z state.
 *  - The 0th data byte in the buffer will be loaded from the W25N Data Buffer at the given
 *    column address. The 1st data byte will be loaded from the W25N Data Buffer at the given
 *    column address + 1, etc.
 *
 * PARAMETERS:
 *  p_buffer: Pointer to the buffer which will contain the output data bytes.
 *  column_address: W25N Data Buffer memory address for the 0th byte to output.
 *  num_of_bytes: Number of bytes to output from the W25N Data Buffer into the buffer.
 */
W25N_StatusTypeDef W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes);

//###############################################################################################
//High-Level Driver Function Prototypes
//###############################################################################################
/*
 * FUNCTION: W25N_Wait_Until_Not_Busy
 *
 * DESCRIPTION: Waits until the BUSY status bit is 0 (not busy).
 *
 * NOTES:
 *  - Waits a maximum of 10ms. This is the maximum possible time the W25N can be busy. 
 *  - This function blocks using the HAL_Delay function.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_READY: The W25N is no longer busy.
 *  W25N_HANGING: The W25N is hanging since it was busy for longer than 10ms.
 */
W25N_StatusTypeDef W25N_Wait_Until_Not_Busy();

/*
 * FUNCTION: W25N_Check_LUT_Full
 *
 * DESCRIPTION: Checks if the Bad Block Management (BBM) Look Up Table (LUT) is full.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_LUT_HAS_ROOM: There is still room to add to the BBM LUT.
 *  W25N_LUT_FULL: The BBM LUT is full.
 */
W25N_StatusTypeDef W25N_Check_LUT_Full();

/*
 * FUNCTION: W25N_Check_ECC_Status
 *
 * DESCRIPTION: Check the ECC status of the most recent read operation.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_ECC_CORRECTION_UNNECESSARY: No ECC correction was necessary.
 *  W25N_ECC_CORRECTION_OK: There were 1-4 bit errors for the page which were successfully corrected.
 *  W25N_ECC_CORRECTION_ERROR: ECC correction was unsuccessful since there were more than 4 bit 
 *                             errors for the page.
 */
W25N_StatusTypeDef W25N_Check_ECC_Status();

/*
 * FUNCTION: W25N_Check_Program_Failure
 *
 * DESCRIPTION: Check if the most recent program operation was successful.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_PROGRAM_OK: The most recent program operation was successful.
 *  W25N_PROGRAM_ERROR: The most recent program operation was unsuccessful.
 */
W25N_StatusTypeDef W25N_Check_Program_Failure();

/*
 * FUNCTION: W25N_Check_Erase_Failure
 *
 * DESCRIPTION: Check if the most recent erase operation was successful.
 * 
 * W25N_StatusTypeDef SPECIFIC RETURNS:
 *  W25N_ERASE_OK: The most recent erase operation was successful.
 *  W25N_ERASE_ERROR: The most recent erase operation was unsuccessful.
 */
W25N_StatusTypeDef W25N_Check_Erase_Failure();

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

//###############################################################################################
//Driver Functions
//###############################################################################################
W25N_StatusTypeDef W25N_Device_Reset()
{
    W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_DEVICE_RESET;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;

    HAL_Delay(1); //The device needs max 500us to reset (no commands accepted during this time)

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Read_Status_Register(uint8_t register_address, uint8_t *p_buffer)
{
	W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_READ_STATUS_REGISTER;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_address, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, 1, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Write_Status_Register(uint8_t register_address, uint8_t register_value)
{
    W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_WRITE_STATUS_REGISTER;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_address, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &register_value, 1, W25N_SPI_DELAY);

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Write_Enable()
{
    W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_WRITE_ENABLE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Write_Disable()
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_WRITE_DISABLE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Bad_Block_Management(uint16_t logical_block_address, uint16_t physical_block_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_BAD_BLOCK_MANAGEMENT;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(logical_block_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(physical_block_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Block_Erase_128KB(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_BLOCK_ERASE_128KB;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Load_Program_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_LOAD_PROGRAM_DATA;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(column_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, p_buffer, num_of_bytes, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Program_Execute(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_PROGRAM_EXECUTE;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Page_Data_Read(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_PAGE_DATA_READ;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(page_address);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

W25N_StatusTypeDef W25N_Read_Data(uint8_t *p_buffer, uint16_t column_address, uint16_t num_of_bytes)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_READ_DATA;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_SPI_Transmit_Word_16Bit(column_address);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, num_of_bytes, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

//###############################################################################################
//Public Driver Functions
//###############################################################################################
W25N_StatusTypeDef W25N_Read_JEDEC_ID(uint8_t *p_buffer)
{
	W25N_StatusTypeDef operation_status;
	uint8_t opcode = W25N_OPCODE_READ_JEDEC_ID;
	uint8_t dummy_byte = W25N_DUMMY_BYTE;

	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

	operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;
	operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, 3, W25N_SPI_DELAY); //JEDEC ID is 3 bytes long

error:
	HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
	return operation_status;
}

W25N_StatusTypeDef W25N_Read_BBM_LUT(uint8_t *p_buffer)
{
    W25N_StatusTypeDef operation_status;
    uint8_t opcode = W25N_OPCODE_READ_BBM_LUT;
    uint8_t dummy_byte = W25N_DUMMY_BYTE;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_RESET);

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &opcode, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &dummy_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = HAL_SPI_Receive(&W25N_SPI, p_buffer, W25N_BBM_LUT_NUM_OF_BYTES, W25N_SPI_DELAY);

error:
    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET);
    return operation_status;
}

//###############################################################################################
//High-Level Driver Functions
//###############################################################################################
W25N_StatusTypeDef W25N_Wait_Until_Not_Busy()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if (!(register_contents & 0b00000001)) return W25N_READY;

    for (int i = 0; i < 10; i++) //10ms is the maximum possible busy time
    {
        HAL_Delay(1);
        operation_status = W25N_Read_Status_Register(register_address, &register_contents);
        if (operation_status != W25N_HAL_OK) goto error;
        if (!(register_contents & 0b00000001)) return W25N_READY;
    }
    operation_status = W25N_HANGING;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_LUT_Full()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b01000000) == 0b01000000) return W25N_LUT_FULL;
    else if ((register_contents & 0b01000000) == 0b00000000) return W25N_LUT_HAS_ROOM;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_ECC_Status()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b00110000) == 0b00000000) return W25N_ECC_CORRECTION_UNNECESSARY;
    else if ((register_contents & 0b00110000) == 0b00010000) return W25N_ECC_CORRECTION_OK;
    else if ((register_contents & 0b00110000) == 0b00100000) return W25N_ECC_CORRECTION_ERROR;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_Program_Failure()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b00001000) == 0b00001000) return W25N_PROGRAM_ERROR;
    else if ((register_contents & 0b00001000) == 0b00000000) return W25N_PROGRAM_OK;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Check_Erase_Failure()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_contents;
    uint8_t register_address = 0xC0;

    operation_status = W25N_Read_Status_Register(register_address, &register_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    if ((register_contents & 0b00000100) == 0b00000100) return W25N_ERASE_ERROR;
    else if ((register_contents & 0b00000100) == 0b00000000) return W25N_ERASE_OK;

    operation_status = W25N_HAL_ERROR;

error:
    return operation_status;
}

//###############################################################################################
//Public High-Level Driver Functions
//###############################################################################################
W25N_StatusTypeDef W25N_Init()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_1_contents = 0b00000010;
    uint8_t register_1_address = 0xA0;
    uint8_t register_2_contents = 0b00011000;
    uint8_t register_2_address = 0xB0;

    HAL_Delay(10); //Ensure the W25N has completed internal initialization (~5ms)

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET); //deselect the W25N
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET); //disable all write/program/erase functionality
    HAL_GPIO_WritePin(W25N_nHOLD_GPIO, W25N_nHOLD_PIN, GPIO_PIN_SET); //allow device operations (disable hold state)

    operation_status = W25N_Write_Status_Register(register_1_address, register_1_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Status_Register(register_2_address, register_2_contents);

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Establish_BBM_Link(uint16_t logical_block_address, uint16_t physical_block_address)
{
    W25N_StatusTypeDef operation_status;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_READY) goto error;

    operation_status = W25N_Check_LUT_Full();
    if (operation_status != W25N_LUT_HAS_ROOM) goto error;

    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Bad_Block_Management(logical_block_address, physical_block_address);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_READY) goto error;

    operation_status = W25N_Write_Disable();
    
error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Read(uint8_t *p_buffer, uint16_t page_address, uint16_t column_address, uint16_t num_of_bytes)
{
	W25N_StatusTypeDef operation_status;

	operation_status = W25N_Wait_Until_Not_Busy();
	if (operation_status != W25N_READY) goto error;

	operation_status = W25N_Page_Data_Read(page_address);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = W25N_Wait_Until_Not_Busy();
	if (operation_status != W25N_READY) goto error;

	operation_status = W25N_Read_Data(p_buffer, column_address, num_of_bytes);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = W25N_Check_ECC_Status();

error:
	return operation_status;
}

W25N_StatusTypeDef W25N_Write(uint8_t *p_buffer, uint16_t page_address, uint16_t column_address, uint16_t num_of_bytes)
{
	W25N_StatusTypeDef operation_status;

	operation_status = W25N_Wait_Until_Not_Busy();
	if (operation_status != W25N_READY) goto error;

	operation_status = W25N_Write_Enable();
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = W25N_Load_Program_Data(p_buffer, column_address, num_of_bytes);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = W25N_Program_Execute(page_address);
	if (operation_status != W25N_HAL_OK) goto error;

	operation_status = W25N_Wait_Until_Not_Busy();
	if (operation_status != W25N_READY) goto error;

	operation_status = W25N_Check_Program_Failure();

error:
	return operation_status;
}

W25N_StatusTypeDef W25N_Erase(uint16_t page_address)
{
    W25N_StatusTypeDef operation_status;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_READY) goto error;

    operation_status = W25N_Write_Enable();
    if (operation_status != W25N_HAL_OK) goto error;
    
    operation_status = W25N_Block_Erase_128KB(page_address);
    if (operation_status != W25N_HAL_OK) goto error;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_READY) goto error;

    operation_status = W25N_Check_Erase_Failure();

error:
    return operation_status;
}

W25N_StatusTypeDef W25N_Reset_And_Init()
{
    W25N_StatusTypeDef operation_status;
    uint8_t register_1_contents = 0b00000010;
    uint8_t register_1_address = 0xA0;
    uint8_t register_2_contents = 0b00011000;
    uint8_t register_2_address = 0xB0;

    operation_status = W25N_Wait_Until_Not_Busy();
    if (operation_status != W25N_READY) goto error;

    operation_status = W25N_Device_Reset();
    if (operation_status != W25N_HAL_OK) goto error;

    HAL_GPIO_WritePin(W25N_nCS_GPIO, W25N_nCS_PIN, GPIO_PIN_SET); //deselect the W25N
    HAL_GPIO_WritePin(W25N_nWP_GPIO, W25N_nWP_PIN, GPIO_PIN_RESET); //disable all write/program/erase functionality
    HAL_GPIO_WritePin(W25N_nHOLD_GPIO, W25N_nHOLD_PIN, GPIO_PIN_SET); //allow device operations (disable hold state)

    operation_status = W25N_Write_Status_Register(register_1_address, register_1_contents);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = W25N_Write_Status_Register(register_2_address, register_2_contents);

error:
    return operation_status;
}

//###############################################################################################
//Helper Functions
//###############################################################################################
W25N_StatusTypeDef W25N_SPI_Transmit_Word_16Bit(uint16_t word_16bit)
{
    W25N_StatusTypeDef operation_status;
    uint8_t word_16bit_high_byte = word_16bit >> 8; //value is truncated & high byte stored
    uint8_t word_16bit_low_byte = word_16bit; //value is truncated & low byte stored

    operation_status = HAL_SPI_Transmit(&W25N_SPI, &word_16bit_high_byte, 1, W25N_SPI_DELAY);
    if (operation_status != W25N_HAL_OK) goto error;
    operation_status = HAL_SPI_Transmit(&W25N_SPI, &word_16bit_low_byte, 1, W25N_SPI_DELAY);

error:
    return operation_status;
}

//###############################################################################################
//Public Helper Functions
//###############################################################################################
uint16_t W25N_Page_Address_To_Block_Address(uint16_t page_address)
{
    return page_address / W25N_PAGES_PER_BLOCK;
}
