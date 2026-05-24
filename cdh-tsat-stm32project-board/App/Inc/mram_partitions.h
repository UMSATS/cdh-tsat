/*
 * mram_partitions.h
 *
 *  Created on: Jan 25, 2026
 *      Author: jagritsharma
 */

#ifndef INC_MRAM_PARTITIONS_H_
#define INC_MRAM_PARTITIONS_H_

typedef struct {
	uint16_t active_envs;
	float setpoints[16];
    float tolerance;
} PayloadState_t;

#define PAYLOAD_STATE_MRAM_ADDRESS   (0x01FFFF - sizeof(PayloadState_t))

#endif /* INC_MRAM_PARTITIONS_H_ */
