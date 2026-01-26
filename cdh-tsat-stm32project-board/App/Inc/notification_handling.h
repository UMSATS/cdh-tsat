/*
 * notification_handling.h
 *
 *  Created on: Jan 25, 2026
 *      Author: jagritsharma
 */

#ifndef INC_NOTIFICATION_HANDLING_H_
#define INC_NOTIFICATION_HANDLING_H_

#include "cmsis_os.h"

extern osMessageQueueId_t notificationQueueHandle;

void StartNotification(void *argument);


#endif /* INC_NOTIFICATION_HANDLING_H_ */
