/*
 * bdot_algorithm.c
 *
 *  Closed-loop three-state attitude-control scheduler for CubeSat ADCS
 *  S1  – sample   (magnetorquers OFF, magnetometer ON) 100 ms
 *  S2  – actuate  (magnetorquers ON, magnetometer ignored) 1 200 ms
 *  S3  – decay    (magnetorquers OFF, magnetometer ignored) 100 ms
 *	
 * 	Created by: drive
 *  Author: Alexandr Yermakov
 */
#include "bdot_algorithm.h"

extern CANQueue_t can_queue;        /* Global queue defined in main.c   */
extern CAN_HandleTypeDef hcan1;

#define M_THRESHOLD 0.01f  // Or tune as needed


// TODO: Import the telemetry function 
#include "tuk/tuk.h"
#include <stdio.h>
#include "cmsis_os.h"

/* ------------------------------------------------------------------------- */
/*                              configuration                                */
/* ------------------------------------------------------------------------- */
#define S1_DURATION_MS   100u      /* 0.1 s  – matches 20 Hz data-rate window */
#define S2_DURATION_MS  1200u      /* 1.2 s  – chosen for ≤10 °/s tumble rate */
#define S3_DURATION_MS   100u      /* 0.1 s  – conservative core-decay time  */

#define ALPHA            0.20f     /* exponential filter coefficient (S1)   */

float[3] g_magnetic_field;

/* ------------------------------------------------------------------------- */
/*                              private types                                */
/* ------------------------------------------------------------------------- */
typedef enum
{
    STATE_S1_SAMPLE = 0,
    STATE_S2_ACTUATE,
    STATE_S3_DECAY
} State_t;

// typedef struct
// {
//     int16_t raw[3];
//     float   tesla[3];     /* filtered values */
// } Sample_t;

/* ------------------------------------------------------------------------- */
/*                             private variables                             */
/* ------------------------------------------------------------------------- */
static State_t   s_state          = STATE_S1_SAMPLE;
static uint32_t  s_state_entry_ms = 0u;          /* HAL_GetTick() timestamp   */
static Sample_t  s_last_sample    = {0};

/* ------------------------------------------------------------------------- */
/*                           forward declarations                            */
/* ------------------------------------------------------------------------- */
static void state_s1_sample(void);
static void state_s2_actuate(void);
static void state_s3_decay(void);
static inline void send_magnetorquer_cmd(uint8_t cmd);
bool ADCS_Bdot_Compute(float m[3]);

/* ------------------------------------------------------------------------- */
/*                              public API                                   */
/* ------------------------------------------------------------------------- */
void AttitudeControl_Init(void)
{
    /* make sure magnetorquers are OFF before we start sampling              */
    // Magnetorquer1_Off();
    // Magnetorquer2_Off();
    // Magnetorquer3_Off();


		// TODO: Add the raw magnetometer readings from ADCS
    /* take an initial measurement so we start with a meaningful value       */
    // MAG_ReadMagneticField(s_last_sample.raw);
    // MAG_ConvertToTeslas(s_last_sample.raw, s_last_sample.tesla);

	// osMessageQueueId_t mag_queue = osMessageQueueNew(QUEUE_SIZE, sizeof(CANQueueItem), NULL);
    s_state = STATE_S1_SAMPLE;
}

void AttitudeControl_Task(void)
{
    //uint32_t now = HAL_GetTick();
    //uint32_t elapsed = now - s_state_entry_ms;

    switch (s_state)
    {
        /* -------------------------------------------------  S1: SAMPLE  --- */
        case STATE_S1_SAMPLE:
            state_s1_sample();
			// TODO: needs an update for the delay of each stage
			uint32_t tick = osKernelGetTickCount() + S1_DURATION_MS;
            osDelayUntil(tick);
			s_state = STATE_S2_ACTUATE;
            break;

        /* -------------------------------------------------  S2: ACTUATE --- */
        case STATE_S2_ACTUATE:
            state_s2_actuate();
			uint32_t tick = osKernelGetTickCount() + S2_DURATION_MS;
            osDelayUntil(tick);
			s_state = STATE_S3_DECAY;
            break;

        /* -------------------------------------------------  S3: DECAY   --- */
        case STATE_S3_DECAY:
            state_s3_decay();
			uint32_t tick = osKernelGetTickCount() + S3_DURATION_MS;
            osDelayUntil(tick);
			s_state = STATE_S1_SAMPLE;
            break;

        default:
            /* should never happen */
            s_state = STATE_S1_SAMPLE;
            break;
    }
}

/* ------------------------------------------------------------------------- */
/*                             state handlers                                */
/* ------------------------------------------------------------------------- */


/* ---------- S1: sample magnetic field ------------------------------------ */
static void state_s1_sample(void)
{
    /* Ensure torquers are OFF so the reading is not contaminated.           */
	send_magnetorquer_cmd(0, 0);
	send_magnetorquer_cmd(1, 0);
	send_magnetorquer_cmd(2, 0);

    /* Raw reading */

    // MAG_ReadMagneticField(s_last_sample.raw);

    /* Convert and low-pass filter (simple 1-pole IIR)                       */
    float tmp[3];
    MAG_ConvertToTeslas(s_last_sample.raw, tmp);

    for (int i = 0; i < 3; ++i)
    {
        s_last_sample.tesla[i] = ALPHA * tmp[i] +
                                 (1.0f - ALPHA) * s_last_sample.tesla[i];
    }

    /* Debug printouts (remove in flight builds)                             */
    printf("[S1] B-field  X: %.6f  Y: %.6f  Z: %.6f (mT)\r\n",
           s_last_sample.tesla[0],
           s_last_sample.tesla[1],
           s_last_sample.tesla[2]);
}

/* ---------- S2: actuate based on last sample ----------------------------- */
static void state_s2_actuate(void)
{
    float m[3];
    if (!ADCS_Bdot_Compute(m))
        return;

    /* --------------------------------------------------  Axis 1  -------- */
    if (m[0] > M_THRESHOLD)
    {
        send_magnetorquer_cmd(0, 1); /* Forward dir  */
    }
    else if (m[0] < -M_THRESHOLD)
    {
        send_magnetorquer_cmd(0, -1); /* Reverse dir  */
    }
    else
    {
        send_magnetorquer_cmd(0, 0); /* Off          */
    }

    /* --------------------------------------------------  Axis 2  -------- */
    if (m[1] > M_THRESHOLD)
    {
        send_magnetorquer_cmd(1, 1);
    }
    else if (m[1] < -M_THRESHOLD)
    {
        send_magnetorquer_cmd(1, -1);
    }
    else
    {
        send_magnetorquer_cmd(1, 0);
    }

    /* --------------------------------------------------  Axis 3  -------- */
    if (m[2] > M_THRESHOLD)
    {
        send_magnetorquer_cmd(2, 1);
    }
    else if (m[2] < -M_THRESHOLD)
    {
        send_magnetorquer_cmd(2, -1);
    }
    else
    {
        send_magnetorquer_cmd(2, 0);
    }
}
/* ---------- S3: decay – torquers already OFF ----------------------------- */
static void state_s3_decay(void)
{
    /* Nothing to do – we simply wait for the ferromagnetic cores to reset.  */
	send_magnetorquer_cmd(0, 0);
	send_magnetorquer_cmd(1, 0);
	send_magnetorquer_cmd(2, 0);
}


/* Simple helper to push a one-byte command into the CAN queue */
static inline void send_magnetorquer_cmd(uint8_t magnetorquer_id, int8_t magnetorquer_direction)
{
	// Create a buffer for storing message body data.
    uint8_t msg_data[CAN_MAX_BODY_SIZE];
    SET_MSG_DATA(msg_data, 0, uint8_t, magnetorquer_id); // set the first byte of the message body to 0, 1 or 2
    SET_MSG_DATA(msg_data, 1, int8_t, magnetorquer_direction); // set the second byte of the message body to -1, 0, 1
    CANWrapper_Transmit(&hcan1, NODE_CDH, CMD_ADCS_SET_MAGNETORQUER_DIRECTION, msg_data);
}

#define ALPHA 0.1f
#define K 1.0f

static float B_filtered_prev[3] = {0.0f, 0.0f, 0.0f};
static uint32_t prev_time = 0;

extern float exponentialFilter(float curr, float prev, float alpha);

// Now this function just computes m and returns it
bool ADCS_Bdot_Compute(float m[3])
{
    int16_t raw_mag[3];
    float B[3];
	raw_mag = g_magnetic_field;
	// MAG_ConvertToTeslas(raw_mag, B);


    uint32_t current_time = HAL_GetTick();
    if (prev_time == 0)
    {
        prev_time = current_time;
        for (int i = 0; i < 3; ++i) {
            B_filtered_prev[i] = B[i];
            m[i] = 0.0f;
        }
        return false;
    }

    float delta_t = (current_time - prev_time) / 1000.0f;
    if (delta_t <= 0.0f) return false;

    float B_filtered[3], B_dot[3];
    for (int i = 0; i < 3; ++i)
    {
        B_filtered[i] = exponentialFilter(B[i], B_filtered_prev[i], ALPHA);
        B_dot[i] = (B_filtered[i] - B_filtered_prev[i]) / delta_t;
        m[i] = -K * B_dot[i];
        B_filtered_prev[i] = B_filtered[i];
    }

    prev_time = current_time;
    return true;
}