/*
 * FILENAME: camera_driver_test.h
 *
 * DESCRIPTION: Unit test for the piCAM camera driver
 * 	            (see camera_driver.{c,h})
 *
 * AUTHORS:
 *  - Syed Abraham Ahmed (syed.ahmed@umsats.ca)
 *  - Amer El Eissawi (amer.eleissawi@umsats.ca)
 *
 * Created on: Dec. 6, 2022
 */

#ifndef HARDWARE_PERIPHERALS_TEST_INC_CAMERA_DRIVER_TEST_H_
#define HARDWARE_PERIPHERALS_TEST_INC_CAMERA_DRIVER_TEST_H_

/*----------------------------------------------------------------------------------------------
Public Testing Function Prototypes
-----------------------------------------------------------------------------------------------*/

/*
 * FUNCTION: piCAM_Test_Procedure
 *
 * DESCRIPTION: Tests piCAM functionality by sending a test command and receiving a test string.
 * Following captures and recieves an image from piCAM which is then processed.
 *
 * NOTES:
 * 	- Boots up piCAM
 * 	- Sends a test command to piCAM
 *  - Sends a capture command to piCAM
 * 	- Receives image from piCAM
 * 	- Processes image
 */
void piCAM_Test_Procedure();

#endif /* HARDWARE_PERIPHERALS_TEST_INC_CAMERA_DRIVER_TEST_H_ */
