/*
 * FILENAME: fetch_telem_data.h
 *
 *  AUTHORS:
 *  - Andrew Driver (andrew.driver@umsats.ca)
 *
 *  CREATED ON: May 30, 2026
 */

#ifndef INC_FETCH_TELEM_DATA_H_
#define INC_FETCH_TELEM_DATA_H_

/**
* @brief Function implementing the telemHandler thread.
* @param argument: Not used
* @retval None
*/
void StartFetchTelemData(void *argument);

#endif /* INC_FETCH_TELEM_DATA_H_ */
