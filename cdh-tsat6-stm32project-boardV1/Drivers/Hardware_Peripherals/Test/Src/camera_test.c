/*
 * FILENAME: camera_test.c
 *
 * DESCRIPTION: Unit test for the Camera Driver
 * 	            (see camera_driver.{c,h})
 *
 * AUTHORS:
 *  - Syed Abraham Ahmed (syed.ahmed@umsats.ca)
 *  - Amer El Eissawi (amer.eleissawi@umsats.ca)
 *
 * Created on: Dec. 6, 2022
 */

/*----------------------------------------------------------------------------------------------
Public Testing Function Definitions
-----------------------------------------------------------------------------------------------*/
void piCAM_Test_Procedure()
{
    piCAM_Boot_Up_Sequence();

    HAL_Delay(3000);
    piCAM_Status_Test();

    HAL_Delay(3000);
    piCAM_Capture_Daylight();

    piCAM_DMA_Start();
}
