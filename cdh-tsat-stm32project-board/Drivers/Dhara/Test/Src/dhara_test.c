/*
 * Dhara_test.h
 *
 * Author
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 *  Created on: Nov 20, 2025
*/

#include <string.h>
#include <dhara/nand.h>
#include <dhara/map.h>
#include <W25N_driver.h>

#include "Dhara_Wrapper.h"
#include "Dhara_test.h"


//Stores a good blocks first page for testing
uint32_t testPage;

#define DHARA_USE_MOCK_BAD_BLOCKS 1

uint8_t dharaTestMockBadBlocks=0;

//############################################################################
//########################    NAND.C FUNCTION TEST    ########################
//############################################################################

dhara_error_t Dhara_Test_NAND_Write(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	dhara_nand_prog(&my_nand, testPage, data, &err);
	if(err!=DHARA_E_NONE) goto error;



	uint8_t readData[PAGESIZE];

	status=W25N_Read(readData, testPage, 0, PAGESIZE);

	//If Read Fails
	if (status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK) {
		err = DHARA_E_TOO_BAD;
		goto error;
	}

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_NAND_Read(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	status=W25N_Write(data, testPage, 0, PAGESIZE);

	// Failed to write to NAND
	if (status !=W25N_PROGRAM_OK){
		err = DHARA_E_RECOVER;
		goto error;
	}

	uint8_t readData[PAGESIZE];
	dhara_nand_read(&my_nand, testPage, 0, PAGESIZE, readData, &err);
	if(err!=DHARA_E_NONE) goto error;

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}

dhara_error_t Dhara_Test_NAND_Erase(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	status=W25N_Write(data, testPage, 0, PAGESIZE);

	// Failed to write to NAND
	if (status !=W25N_PROGRAM_OK){
		err = DHARA_E_RECOVER;
		goto error;
	}

	dhara_nand_erase(&my_nand, testPage/(1<<NAND_LOG2_PAGE_PER_BLOCK), &err);
	if(err!=DHARA_E_NONE) goto error;

	uint8_t readData[PAGESIZE];

	status=W25N_Read(readData, testPage, 0, PAGESIZE);

	//If Read Fails
	if (status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK) {
		err = DHARA_E_TOO_BAD;
		goto error;
	}

	for(int i=0;i<PAGESIZE;i++){
		if(readData[i]!=0xFF){
			err=DHARA_E_TOO_BAD;
			goto error;
		}
	}


error:
	return err;
}

dhara_error_t Dhara_Test_NAND_Is_Bad(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	//Fresh block shouldn't be bad
	if(dhara_nand_is_bad(&my_nand, testPage/(1<<NAND_LOG2_PAGE_PER_BLOCK))){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	uint8_t bad_marker = 0x00;

	status = W25N_Write_Spare_Area(&bad_marker, testPage, 0, 1);

	// Failed to write to NAND
	if (status !=W25N_PROGRAM_OK){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	//Fresh block shouldn't be bad
	if(!dhara_nand_is_bad(&my_nand, testPage/(1<<NAND_LOG2_PAGE_PER_BLOCK))){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}

dhara_error_t Dhara_Test_NAND_Mark_As_Bad(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	//Fresh block shouldn't be bad
	uint8_t marker = 0xFF;

	// Checking spare area byte 0 for BadBlock marker
	status = W25N_Read(&marker, testPage, PAGESIZE, 1);

	// If read failed or BadBlock marker is found
	if ((status!=W25N_ECC_CORRECTION_UNNECESSARY&&status!=W25N_ECC_CORRECTION_OK) ||
			marker != 0xFF){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	uint8_t oldSpareNumber;
	W25N_BBM_LUT_Size(&oldSpareNumber);
	dhara_nand_mark_bad(&my_nand, testPage/(1<<NAND_LOG2_PAGE_PER_BLOCK));

	status=W25N_Check_LUT_Full();

	if(!DHARA_USE_MOCK_BAD_BLOCKS||status==W25N_LUT_FULL){
		// Checking spare area byte 0 for BadBlock marker
		status = W25N_Read(&marker, testPage, PAGESIZE, 1);

		// If read failed or BadBlock marker is not found
		if ((status!=W25N_ECC_CORRECTION_UNNECESSARY&&status!=W25N_ECC_CORRECTION_OK) ||
				marker != 0x00){
			err=DHARA_E_TOO_BAD;
			goto error;
		}
	}else{
		uint8_t newSpareNumber;
		W25N_BBM_LUT_Size(&newSpareNumber);

		// If spare number stays the same then error
		if(oldSpareNumber!=newSpareNumber){
			err=DHARA_E_TOO_BAD;
						goto error;
		}
	}

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}



error:
	return err;
}

dhara_error_t Dhara_Test_NAND_Is_Free(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	if(!dhara_nand_is_free(&my_nand, testPage)){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_NAND_Copy(){
	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	status=W25N_Erase(testPage);

	if (status != W25N_ERASE_OK) {
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	status=W25N_Write(data, testPage, 0, PAGESIZE);

	// Failed to write to NAND
	if (status !=W25N_PROGRAM_OK){
		err = DHARA_E_RECOVER;
		goto error;
	}

	//copying page testPage to page testPage+1
	dhara_nand_copy(&my_nand, testPage, testPage+1, &err);
	if(err!=DHARA_E_NONE) goto error;

	uint8_t readData[PAGESIZE];

	status=W25N_Read(readData, testPage+1, 0, PAGESIZE);

	//If Read Fails
	if (status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK) {
		err = DHARA_E_TOO_BAD;
		goto error;
	}

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}


//############################################################################
//########################    NAND.C FUNCTION TEST    ########################
//############################################################################

dhara_error_t Dhara_Test_NAND(){
	dhara_error_t status=DHARA_E_NONE;

	status=Dhara_Test_NAND_Write();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_NAND_Read();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_NAND_Erase();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_NAND_Is_Bad();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_NAND_Mark_As_Bad();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_NAND_Is_Free();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_NAND_Copy();
	if(status!=DHARA_E_NONE) goto error;

error:
	return status;
}


//#############################################################################
//########################    WRAPPER FUNCTION TEST    ########################
//#############################################################################

dhara_error_t Dhara_Test_Wrapper_Clear(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	if(dhara_map_size(&my_map)!=0){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	if(dhara_map_size(&my_map)==0){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	Dhara_Clear();

	if(dhara_map_size(&my_map)!=0){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Capacity(){
	dhara_error_t err=DHARA_E_NONE;

	uint32_t result=Dhara_Capacity();
	if(result!=dhara_map_capacity(&my_map)){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Size(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	if(dhara_map_size(&my_map)!=Dhara_Size()){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	if(dhara_map_size(&my_map)!=Dhara_Size()){
		err=DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Find(){
	dhara_error_t err=DHARA_E_NONE;

	if(dhara_map_find(&my_map, 0, 0, &err)!=Dhara_Find(0, 0, &err)){
		err=DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Read(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	uint8_t readData[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		readData[i]=-1;
	}

	Dhara_Read(0, readData, PAGESIZE, &err);
	if(err!=DHARA_E_NONE) goto error;

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Write(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	Dhara_Write(0, data, PAGESIZE, &err);
	if(err!=DHARA_E_NONE) goto error;

	uint8_t readData[PAGESIZE];

	dhara_map_read(&my_map, 0, readData, &err);
	if(err!=DHARA_E_NONE) goto error;

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Copy_Page(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	Dhara_Copy_Page(0, 1, &err);
	if(err!=DHARA_E_NONE) goto error;

	W25N_StatusTypeDef status;

	uint8_t readData[PAGESIZE];

	//NOTE: There is no dhara_map function that reads pages, only sectors
	status=W25N_Read(readData, 1, 0, PAGESIZE);
	//If Read Fails
	if (status!=W25N_ECC_CORRECTION_UNNECESSARY&&
			status!=W25N_ECC_CORRECTION_OK) {
		err = DHARA_E_TOO_BAD;
		goto error;
	}

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Copy_Sector(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	Dhara_Copy_Sector(0, 1, &err);
	if(err!=DHARA_E_NONE) goto error;

	uint8_t readData[PAGESIZE];

	dhara_map_read(&my_map, 1, readData, &err);
	if(err!=DHARA_E_NONE) goto error;

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Erase(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	Dhara_Erase(0, &err);
	if(err!=DHARA_E_NONE) goto error;

	uint8_t readData[PAGESIZE];

	dhara_map_read(&my_map, 0, readData, &err);
	if(err!=DHARA_E_NONE) goto error;

	for(int i=0;i<PAGESIZE;i++){
		if(readData[i]!=0xFF){
			err=DHARA_E_TOO_BAD;
			goto error;
		}
	}

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_Force_Sync(){
	dhara_error_t err=DHARA_E_NONE;

	// Reset Dhara map
	memset(&my_map, 0, sizeof(my_map));
	err=Dhara_Init();
	if(err!=DHARA_E_NONE&&err!=DHARA_E_NOT_FOUND) goto error;
	err=DHARA_E_NONE;

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	// Needed so journal is written to NAND
	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	Dhara_Force_Sync(&err);

	dhara_map_resume(&my_map, &err);
	if(err!=DHARA_E_NONE) goto error;

error:
	return err;
}

dhara_error_t Dhara_Test_Wrapper_GC(){
	dhara_error_t err=DHARA_E_NONE;

	Dhara_GC(&err);

	return err;
}


//####################################################################
//########################    WRAPPER TEST    ########################
//####################################################################

dhara_error_t Dhara_Test_Wrapper(){
	dhara_error_t status=DHARA_E_NONE;

	status=Dhara_Test_Wrapper_Clear();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Capacity();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Size();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Find();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Read();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Write();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Copy_Page();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Copy_Sector();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Erase();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_Force_Sync();
	if(status!=DHARA_E_NONE) goto error;

	status=Dhara_Test_Wrapper_GC();
	if(status!=DHARA_E_NONE) goto error;

error:
	return status;
}


//#######################################################################
//########################    SIMULATION TEST    ########################
//#######################################################################

dhara_error_t Dhara_Test_Simulated_Power_Loss(){
	dhara_error_t err=DHARA_E_NONE;

	dhara_map_clear(&my_map);

	uint8_t data[PAGESIZE];

	for(int i=0;i<PAGESIZE;i++){
		data[i]=i;
	}

	dhara_map_write(&my_map, 0, data, &err);
	if(err!=DHARA_E_NONE) goto error;

	dhara_map_sync(&my_map, &err);
	if(err!=DHARA_E_NONE) goto error;

	// Simulate power loss by resetting Dhara map
	memset(&my_map, 0, sizeof(my_map));
	err=Dhara_Init();
	if(err!=DHARA_E_NONE&&err!=DHARA_E_NOT_FOUND) goto error;
	err=DHARA_E_NONE;

	uint8_t readData[PAGESIZE];

	dhara_map_read(&my_map, 0, readData, &err);

	if(memcmp(data,readData,PAGESIZE)!=0){
		err = DHARA_E_TOO_BAD;
		goto error;
	}


error:
	return err;
}



//#######################################################################
//########################    HELPER FUNCTION    ########################
//#######################################################################

dhara_error_t Dhara_Test_Find_Good_Block(){
	dhara_error_t err=DHARA_E_NONE;
	uint8_t pageFound=0;

	for(int i =0;i<NAND_NUM_OF_BLOCKS;i++){
		uint8_t marker = 0xFF;
		uint32_t page = i * PAGESIZE;
		W25N_StatusTypeDef status;

		// Checking spare area byte 0 for BadBlock marker
		status = W25N_Read(&marker, page, PAGESIZE, 1);

		// If read succeeds and BadBlock marker is not found(should be 0xFF)
		if ((status==W25N_ECC_CORRECTION_UNNECESSARY||status==W25N_ECC_CORRECTION_OK) &&
				marker == 0xFF){
			testPage=page;
			pageFound=1;
			break;
		}
	}

	if(!pageFound){
		err=DHARA_E_TOO_BAD;
	}

	return err;
}


//##########################################################################
//########################    FULL TEST FUNCTION    ########################
//##########################################################################

dhara_error_t Dhara_Test(){
	dharaTestMockBadBlocks=DHARA_USE_MOCK_BAD_BLOCKS;

	dhara_error_t err=DHARA_E_NONE;
	W25N_StatusTypeDef status;

	//Clears NAND blocks
	for(int i=0;i<NAND_NUM_OF_BLOCKS;i++){
		status=W25N_Erase(i*64);

		if (status != W25N_ERASE_OK) {
			err=DHARA_E_TOO_BAD;
			goto error;
		}
	}

	err=Dhara_Test_Find_Good_Block();
	if(err!=DHARA_E_NONE) goto error;

	err=Dhara_Test_NAND();
	if(err!=DHARA_E_NONE) goto error;

	err=Dhara_Test_Wrapper();
	if(err!=DHARA_E_NONE) goto error;

	err=Dhara_Test_Simulated_Power_Loss();
	if(err!=DHARA_E_NONE) goto error;

	// If no errors, clean dhara sectors.
	if(err==DHARA_E_NONE){
		Dhara_Clear();
	}


error:
	dharaTestMockBadBlocks=0;
	return err;
}

