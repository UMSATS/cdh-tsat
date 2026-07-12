/*
 * FILENAME: nand.c
 *
 * DESCRIPTION: Contains all Dhara flash interface callback implementations. Definitions are located at Middlewares/Third_Party/dhara/dhara/nand.h
 *
 * AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 * CREATED ON: Nov. 16 2025
 */


//###############################################################################################
//Include Directives
//###############################################################################################
#include <dhara/nand.h>
#include <Dhara_Wrapper.h>
#include <dhara_test.h>

#include "W25N_driver.h"

//############################################################################
//##########################    NAND.H FUNCTIONS    ##########################
//############################################################################

const struct dhara_nand my_nand = {
    .log2_page_size = NAND_LOG2_PAGE_SIZE,                  // log2 Page Size
    .log2_ppb = NAND_LOG2_PAGE_PER_BLOCK,                   // log2 Pages Per Block
    .num_blocks = NAND_NUM_AVAILABLE_BLOCKS                 // Number of available Blocks
};

int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b)
{
	uint8_t marker = 0xFF;
	uint32_t page = b << n->log2_ppb;
	W25N_StatusTypeDef status;

	// Checking spare area byte 0 for BadBlock marker
	status = W25N_Read(&marker, page, PAGESIZE, 1);

	// If read failed or BadBlock marker is found
	if ((status!=W25N_ECC_CORRECTION_UNNECESSARY&&status!=W25N_ECC_CORRECTION_OK) ||
			marker != 0xFF){
		return 1;// Bad block
	}


	return 0;  // Good block
}

void dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b)
{
	W25N_StatusTypeDef status;
	uint16_t logical_block = (uint16_t)b;
	uint16_t physical_block = NAND_NUM_AVAILABLE_BLOCKS+dharaUsedSpareCount;

	status=W25N_Check_LUT_Full();

	// If there is room and test mock is disabled, dharaTestMockBadBlock will be zero at all times except
	// during testing if DHARA_USE_MOCK_BAD_BLOCKS is 1
	if(status==W25N_LUT_HAS_ROOM&&!dharaTestMockBadBlocks){
		status=W25N_HAL_OK;

		status = W25N_Establish_BBM_Link(logical_block, physical_block);

		if(status==W25N_HAL_OK){
			dharaUsedSpareCount++;
		}else{
			//do something during error case
		}
	// If mock bad blocks are used and no error reading LUT, both LUT Full and has room is used here
	// since they are the only non error outcomes of check lut full and the previous if statement
	// already checks the status of dharaTestMockBad
	}else if(status == W25N_LUT_FULL||status == W25N_LUT_HAS_ROOM){
		uint8_t bad_marker = 0x00;
		uint32_t page = b << n->log2_ppb;

		status = W25N_Write_Spare_Area(&bad_marker, page, 0, 1);

		// Failed to write to NAND
		if (status !=W25N_PROGRAM_OK){
			//do something
		}
	}
}

int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b, dhara_error_t *err)
{
	W25N_StatusTypeDef status;
	uint32_t pageAddress=b << n->log2_ppb;;

	status=W25N_Erase(pageAddress);

	// If erase wasn't successful
    if (status != W25N_ERASE_OK) {
        *err = DHARA_E_BAD_BLOCK;
        return -1;
    }

    return 0;
}

int dhara_nand_prog(const struct dhara_nand *n, dhara_page_t p,
                    const uint8_t *data, dhara_error_t *err)
{
	W25N_StatusTypeDef status;
	uint16_t page_address = (uint16_t)p;


	status = W25N_Write((uint8_t *)data, page_address, 0, PAGESIZE);

	// Failed to write to NAND
	if (status !=W25N_PROGRAM_OK)
	{
		*err = DHARA_E_RECOVER;
		return -1;
	}

	return 0;
}

int dhara_nand_is_free(const struct dhara_nand *n, dhara_page_t p)
{

	W25N_StatusTypeDef status;
	uint16_t page_address = (uint16_t)p;

	uint8_t data[PAGESIZE];


	status = W25N_Read(data,page_address, 0, PAGESIZE);


	// If the read failed, assume not free
	if(status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK){
		return 0;
	}


	// Checking if all data is unprogrammed
	for(uint16_t i=0;i<PAGESIZE;i++){
		if(data[i]!=0xff){
			return 0;
		}

	}

	return 1;
}

int dhara_nand_read(const struct dhara_nand *n, dhara_page_t p,
                    size_t offset, size_t length,
                    uint8_t *data, dhara_error_t *err)
{
	W25N_StatusTypeDef status;


	// Checks if offset and length reach past page size
	if(offset+length>PAGESIZE){
		offset=0;
		length=PAGESIZE;
	}

	// Perform the read
	status = W25N_Read(data,
					   (uint16_t)p,          // page address
					   (uint16_t)offset,     // column address
					   (uint16_t)length);    // byte count


	// Checks if W25N_Read was successful
	if(status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK){
		return -1;
	}

	return 0;
}

int dhara_nand_copy(const struct dhara_nand *n, dhara_page_t src,
                    dhara_page_t dst, dhara_error_t *err)
{
	W25N_StatusTypeDef status;
	uint16_t page_address_src = (uint16_t)src;
	uint16_t page_address_dst = (uint16_t)dst;

	uint8_t data[PAGESIZE];


	status = W25N_Read(data,page_address_src, 0, PAGESIZE);

	// If ECC correction fails
	if (status == W25N_ECC_CORRECTION_ERROR) {
		*err = DHARA_E_TOO_BAD;
		return -1;
	}
	// If read fails
	else if (status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK){
		*err = DHARA_E_RECOVER;
		return -1;
	}


	status = W25N_Write((uint8_t *)data, page_address_dst, 0, PAGESIZE);

	// If write fails
	if (status != W25N_PROGRAM_OK)
	{
		*err = DHARA_E_RECOVER;
		return -1;
	}


	return 0; // success
}
