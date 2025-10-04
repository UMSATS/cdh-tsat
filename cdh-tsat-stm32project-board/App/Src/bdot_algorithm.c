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
 *  Co-Author: Rodrigo Alegria
 */
#include "bdot_algorithm.h"
#include "utils.h"  // For exponentialFilter and MAG_ConvertToTeslas

extern CANQueue_t can_queue;        /* Global queue defined in main.c   */
extern CAN_HandleTypeDef hcan1;

// TODO: Import the telemetry function 
#include "tuk/tuk.h"
#include <stdio.h>
#include "cmsis_os.h"
#include <string.h>  // Added for memcpy
#include "tuk/can_wrapper/telemetry_id.h"

/* ------------------------------------------------------------------------- */
/*                              configuration                                */
/* ------------------------------------------------------------------------- */
#define S1_DURATION_MS   100u      /* 0.1 s  – matches 20 Hz data-rate window */
#define S2_DURATION_MS  1200u      /* 1.2 s  – chosen for ≤10 °/s tumble rate */
#define S3_DURATION_MS   100u      /* 0.1 s  – conservative core-decay time  */

#define ALPHA            0.1f      /* exponential filter coefficient (matches Simulink)   */

float g_magnetic_field[3];

/* ------------------------------------------------------------------------- */
/*                              private types                                */
/* ------------------------------------------------------------------------- */
typedef enum
{
    STATE_S1_SAMPLE = 0,
    STATE_S2_ACTUATE,
    STATE_S3_DECAY
} State_t;

// Removed Sample_t structure - using g_magnetic_field directly

/* ------------------------------------------------------------------------- */
/*                             private variables                             */
/* ------------------------------------------------------------------------- */
static State_t   s_state          = STATE_S1_SAMPLE;
static uint32_t  s_state_entry_ms = 0u;          /* HAL_GetTick() timestamp   */

/* ------------------------------------------------------------------------- */
/*                           forward declarations                            */
/* ------------------------------------------------------------------------- */
static void state_s1_sample(void);
static void state_s2_actuate(void);
static void state_s3_decay(void);
static inline void send_magnetorquer_cmd(float dir0, float dir1, float dir2);
static inline void request_magnetic_field(void);
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

    /* Initialize B-dot algorithm state */
    is_initialized = false;
    prev_time = 0;
    for (int i = 0; i < 3; ++i) {
        B_prev[i] = 0.0f;
        B_dot_prev[i] = 0.0f;
    }
    
    /* Start in sample state */
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
            // Add proper delay for S1 state
            uint32_t tick = osKernelGetTickCount() + S1_DURATION_MS;
            osDelayUntil(tick);
            s_state = STATE_S2_ACTUATE;
            break;

        /* -------------------------------------------------  S2: ACTUATE --- */
        case STATE_S2_ACTUATE:
            state_s2_actuate();
            tick = osKernelGetTickCount() + S2_DURATION_MS;
            osDelayUntil(tick);
            s_state = STATE_S3_DECAY;
            break;

        /* -------------------------------------------------  S3: DECAY   --- */
        case STATE_S3_DECAY:
            state_s3_decay();
            tick = osKernelGetTickCount() + S3_DURATION_MS;
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
    /* Request latest magnetic field from ADCS */
    request_magnetic_field();

    /* Wait until the telemetry handler signals that new data arrived.         */
    /* Timeout after S1_DURATION_MS to avoid blocking forever.                */
    osThreadFlagsWait(0x0001, osFlagsWaitAny, S1_DURATION_MS);

    /* Debug printouts (remove in flight builds)                             */
    printf("[S1] B-field  X: %.6f  Y: %.6f  Z: %.6f (T)\r\n",
           g_magnetic_field[0],
           g_magnetic_field[1],
           g_magnetic_field[2]);
}

/* ---------- S2: actuate based on last sample ----------------------------- */
static void state_s2_actuate(void)
{
    float m[3];
    if (!ADCS_Bdot_Compute(m))
        return;

    /* TODO(PROTOTYPE): We are sending unclamped float dipole commands directly.
       Make sure to clamp to hardware-safe range and add slew-rate limiting
       before flight/production. */
    send_magnetorquer_cmd(m[0], m[1], m[2]);
}
/* ---------- S3: decay – torquers already OFF ----------------------------- */
static void state_s3_decay(void)
{
    /* Nothing to do – we simply wait for the ferromagnetic cores to reset.  */
    send_magnetorquer_cmd(0.0f, 0.0f, 0.0f);
}


/* Simple helper to push a one-byte command into the CAN queue */
static inline void send_magnetorquer_cmd(float dir0, float dir1, float dir2)
{
    uint8_t msg_data[CAN_MAX_BODY_SIZE] = {0};

    /* Pack the three float values consecutively into the CAN message body */
    memcpy(&msg_data[0], &dir0, sizeof(float));
    memcpy(&msg_data[4], &dir1, sizeof(float));
    memcpy(&msg_data[8], &dir2, sizeof(float));

    CANWrapper_Transmit(&hcan1, NODE_CDH, CMD_ADCS_SET_MAGNETORQUER_DIRECTION, msg_data);
}

/* Helper to request magnetic field telemetry from ADCS */
static inline void request_magnetic_field(void)
{
    uint8_t msg_data[CAN_MAX_BODY_SIZE] = {0};
    uint8_t key = CREATE_TELEMETRY_KEY(TEL_MAGNETIC_FIELD, 0u);
    SET_MSG_DATA(msg_data, 0, uint8_t, key);

    /* Ask ADCS subsystem to send TEL_MAGNETIC_FIELD telemetry */
    CANWrapper_Transmit(&hcan1, NODE_ADCS, CMD_COMM_GET_TELEMETRY, msg_data);
}

#define K 1.0f      /* B-dot gain (matches Simulink) */

// Static variables for B-dot calculation (matches Simulink approach)
static float B_prev[3] = {0.0f, 0.0f, 0.0f};
static float B_dot_prev[3] = {0.0f, 0.0f, 0.0f};
static uint32_t prev_time = 0;
static bool is_initialized = false;

// exponentialFilter is now available via utils.h include

/*
 * FUNCTION: ADCS_Bdot_Compute
 *
 * DESCRIPTION: Computes B-dot detumbling algorithm (matches Simulink implementation)
 *              m = -K * B_dot, where B_dot is the filtered derivative of magnetic field
 *
 * PARAMETERS:
 *  m: Output array for magnetic dipole moments [3]
 *
 * RETURNS: true if calculation successful, false if not enough data
 */
bool ADCS_Bdot_Compute(float m[3])
{
    uint32_t current_time = osKernelGetTickCount();
    
    // Initialize on first call
    if (!is_initialized) {
        prev_time = current_time;
        for (int i = 0; i < 3; ++i) {
            B_prev[i] = g_magnetic_field[i];
            B_dot_prev[i] = 0.0f;
            m[i] = 0.0f;
        }
        is_initialized = true;
        return false; // Need at least two samples
    }
    
    // Calculate time delta
    float delta_t = (current_time - prev_time) / 1000.0f; /* ticks to seconds */
    if (delta_t <= 0.0f) return false;
    
    // Current magnetic field values
    float B_curr[3] = {g_magnetic_field[0], g_magnetic_field[1], g_magnetic_field[2]};
    float B_dot_curr[3];
    
    // Calculate B-dot with filtering (same approach as Simulink)
    for (int i = 0; i < 3; ++i) {
        // Raw B-dot calculation
        float B_dot_raw = (B_curr[i] - B_prev[i]) / delta_t;
        
        // Apply exponential filter to B-dot (matches Simulink b_dot function)
        B_dot_curr[i] = (ALPHA * B_dot_raw) + ((1.0f - ALPHA) * B_dot_prev[i]);
        
        // Apply B-dot algorithm: m = -K * B_dot
        m[i] = -K * B_dot_curr[i];
        
        // Update for next iteration
        B_prev[i] = B_curr[i];
        B_dot_prev[i] = B_dot_curr[i];
    }
    
    prev_time = current_time;
    return true;
}