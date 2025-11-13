#include "../Inc/dhara.h"


//############################################################################
//##########################    NAND.H FUNCTIONS    ##########################
//############################################################################


const struct dhara_nand my_nand = {
    .log2_page_size = 11,  // Page Size
    .log2_ppb = 6,         // Pages Per Block
    .num_blocks = 1004     // Number of Blocks minus 20(For Spares to be used in BBM)
};

uint8_t usedSpareCount=0;

int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b)
{
	uint8_t marker = 0xFF;  // store read byte here
	uint32_t page = b << n->log2_ppb;  // first page of the block
	W25N_StatusTypeDef status;

	status = W25N_Read(&marker, page, 0, 1);

	if (status != W25N_HAL_OK || marker != 0xFF)// if block is bad
		return 1;

	// checking spare area byte 0 (offset = 2048)
	status = W25N_Read(&marker, page, 2048, 1);
	if (status != W25N_HAL_OK || marker != 0xFF)// if block is bad
		return 1;

	return 0;  // good block
}

void dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b)
{
	uint16_t logical_block = (uint16_t)b;
	uint16_t physical_block = n->num_blocks+usedSpareCount;

	W25N_StatusTypeDef status = W25N_Establish_BBM_Link(logical_block, physical_block);

	if (status != W25N_LUT_FULL&&status == W25N_HAL_OK)//LUT is Full
	{
		usedSpareCount++;
	}
}

int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b, dhara_error_t *err)
{
    if (W25N_Erase(b) != 0) {
        *err = DHARA_E_BAD_BLOCK;
        return -1;
    }
    return 0;
}

int dhara_nand_prog(const struct dhara_nand *n, dhara_page_t p,
                    const uint8_t *data, dhara_error_t *err)
{
	W25N_StatusTypeDef status;

	uint16_t num_bytes = (1 << n->log2_page_size);
	uint16_t page_address = (uint16_t)p;

	status = W25N_Write((uint8_t *)data, page_address, 0, num_bytes);

	if (status != W25N_HAL_OK && status != W25N_READY)
	{
		if (err) *err = DHARA_E_RECOVER;
		return -1;
	}

	return 0; // success
}

int dhara_nand_is_free(const struct dhara_nand *n, dhara_page_t p)
{

	W25N_StatusTypeDef status;

	uint16_t num_bytes = (1 << n->log2_page_size);
	uint16_t page_address = (uint16_t)p;

	uint8_t data[num_bytes];

	status = W25N_Read(data,page_address, 0, (uint16_t)num_bytes);


	// If the read failed, assume not free
	if (status == W25N_ECC_CORRECTION_ERROR) {
		return 0;
	}
	else if (status != W25N_HAL_OK && status != W25N_READY &&
			 status != W25N_ECC_CORRECTION_UNNECESSARY &&
			 status != W25N_ECC_CORRECTION_OK)
	{
		return 0;
	}


	//Checking if all data is unprogrammed
	for(uint16_t i=0;i<num_bytes;i++){
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
	size_t page_size = (1U << n->log2_page_size);

	// Sanity check
	if (offset + length > page_size)
	{
		if (err) *err = DHARA_E_BAD_BLOCK;
		return -1;
	}

	// Perform the read
	status = W25N_Read(data,
					   (uint16_t)p,          // page address
					   (uint16_t)offset,     // column address
					   (uint16_t)length);    // byte count

	//Status Check
	switch (status)
	{
		case W25N_HAL_OK:
		case W25N_READY:
		case W25N_ECC_CORRECTION_UNNECESSARY:
		case W25N_ECC_CORRECTION_OK:
			if (err) *err = DHARA_E_NONE;
			return 0;

		case W25N_ECC_CORRECTION_ERROR:
			if (err) *err = DHARA_E_TOO_BAD;
			return -1;

		case W25N_HAL_ERROR:
		case W25N_HAL_TIMEOUT:
		case W25N_HAL_BUSY:
		case W25N_HANGING:
			if (err) *err = DHARA_E_RECOVER;
			return -1;

		default:
			if (err) *err = DHARA_E_RECOVER;
			return -1;
	}
}

int dhara_nand_copy(const struct dhara_nand *n, dhara_page_t src,
                    dhara_page_t dst, dhara_error_t *err)
{
	W25N_StatusTypeDef status;

	uint16_t num_bytes = (1 << n->log2_page_size);
	uint16_t page_address_src = (uint16_t)src;
	uint16_t page_address_dst = (uint16_t)dst;

	uint8_t data[num_bytes];


	status = W25N_Read(data,page_address_src, 0, (uint16_t)num_bytes);

	if (status == W25N_ECC_CORRECTION_ERROR) {
		if (err) *err = DHARA_E_TOO_BAD;
		return -1;
	}
	else if (status != W25N_HAL_OK && status != W25N_READY &&
			 status != W25N_ECC_CORRECTION_UNNECESSARY &&
			 status != W25N_ECC_CORRECTION_OK)
	{
		if (err) *err = DHARA_E_RECOVER;
		return -1;
	}

	if (err) *err = DHARA_E_NONE;

	status = W25N_Write((uint8_t *)data, page_address_dst, 0, num_bytes);

	if (status != W25N_HAL_OK && status != W25N_READY)
	{
		if (err) *err = DHARA_E_RECOVER;
		return -1;
	}

	return 0; // success
}


//################################################################################
//##########################    FLASH INITIALIZATION    ##########################
//################################################################################

#define JOURNAL_BUFFER_SIZE 4096
uint8_t journal_buffer[JOURNAL_BUFFER_SIZE];

dhara_error_t err;

struct dhara_map my_map;


void flash_init(void){
	//initializes the W25N drivers
	W25N_Init();

	W25N_BBM_LUT_Size(&usedSpareCount);

	//Dhara initialization
	dhara_map_init(&my_map, &my_nand, journal_buffer, JOURNAL_BUFFER_SIZE);

	//Step 4: Try loading existing mapping from flash
	dhara_map_resume(&my_map, &err);

	if (err != DHARA_E_NONE) {

	}
}
