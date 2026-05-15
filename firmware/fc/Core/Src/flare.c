/* firmware/fc/Core/Src/flare.c */
#include "flare.h"
#include "dshot.h"
#include "pid.h"
#include <stdint.h>

static FLARE_State state;
static PID_Controller pid_roll;
static PID_Controller pid_pitch;
static PID_Controller pid_yaw;

/* ── Shared motor commands — written by FLARE_Update, read by TIM6 ISR ── */
volatile uint16_t dshot_m1 = 0;
volatile uint16_t dshot_m2 = 0;
volatile uint16_t dshot_m3 = 0;
volatile uint16_t dshot_m4 = 0;

volatile uint8_t motors_enabled = 0;

/* Clamp a float to [min, max] */
static float clamp(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

void FLARE_Init(void) {
    state.roll_sp     = 0.0f;
    state.pitch_sp    = 0.0f;
    state.yaw_rate_sp = 0.0f;
    state.throttle    = 0;
    state.armed       = 0;

    dshot_m1 = 0;
    dshot_m2 = 0;
    dshot_m3 = 0;
    dshot_m4 = 0;

    /*
     * Initial PID gains — these WILL need tuning on the bench before flight.
     * Start conservative: low P, no I, small D.
     * Args: kp, ki, kd, integral_limit, output_limit
     */
    PID_Init(&pid_roll,  1.5f, 0.0f, 0.05f, 20.0f, 200.0f);
    PID_Init(&pid_pitch, 1.5f, 0.0f, 0.05f, 20.0f, 200.0f);
    PID_Init(&pid_yaw,   2.0f, 0.0f, 0.00f, 20.0f, 200.0f);
}

void FLARE_Update(float roll, float pitch, float gx, float gy, float gz, float dt) {
    if (!state.armed || state.throttle < 48) {
        /* DSHOT 0 = disarm command. Values 1-47 are reserved special commands
         * and must never be sent to ESCs. */
        dshot_m1 = 0;
        dshot_m2 = 0;
        dshot_m3 = 0;
        dshot_m4 = 0;
        PID_Reset(&pid_roll);
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_yaw);
        return;
    }

    if (state.throttle < 250) {
        dshot_m1 = state.throttle;
        dshot_m2 = state.throttle;
        dshot_m3 = state.throttle;
        dshot_m4 = state.throttle;
        PID_Reset(&pid_roll);
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_yaw);
        return;
    }

    float out_roll  = PID_Update(&pid_roll,  state.roll_sp,     roll,  gx, dt);
    float out_pitch = PID_Update(&pid_pitch, state.pitch_sp,    pitch, gy, dt);
    float out_yaw   = PID_Update(&pid_yaw,   state.yaw_rate_sp, 0.0f,  gz, dt);

    /*
     * X-frame motor mixing:
     *
     *  M1 (Front-Left,  CCW) = throttle + roll - pitch - yaw
     *  M2 (Front-Right, CW)  = throttle - roll - pitch + yaw
     *  M3 (Rear-Right,  CCW) = throttle - roll + pitch - yaw
     *  M4 (Rear-Left,   CW)  = throttle + roll + pitch + yaw
     *
     * DSHOT value range: 48 (min throttle) to 2047 (max throttle).
     * Values below 48 are disarm/special commands — clamp to 48 minimum when armed.
     */
    float t = (float)state.throttle;

    float m1 = t + out_roll - out_pitch - out_yaw;
    float m2 = t - out_roll - out_pitch + out_yaw;
    float m3 = t - out_roll + out_pitch - out_yaw;
    float m4 = t + out_roll + out_pitch + out_yaw;

    dshot_m1 = (uint16_t)clamp(m1, 48.0f, 2047.0f);
    dshot_m2 = (uint16_t)clamp(m2, 48.0f, 2047.0f);
    dshot_m3 = (uint16_t)clamp(m3, 48.0f, 2047.0f);
    dshot_m4 = (uint16_t)clamp(m4, 48.0f, 2047.0f);
}

/* ── Arming and throttle control ─────────────────────────────────────────── */

void FLARE_SetArmed(uint8_t armed) {
    state.armed = armed;
    if (!armed) {
        /* DSHOT 0 = disarm command. Must not send 48 (min throttle) when
         * unarmed — ESC behaviour on reserved values 1-47 is undefined. */
        dshot_m1 = 0;
        dshot_m2 = 0;
        dshot_m3 = 0;
        dshot_m4 = 0;
        PID_Reset(&pid_roll);
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_yaw);
    }
}

void FLARE_SetThrottle(uint16_t throttle) {
    state.throttle = throttle;
}

/* ── Setpoint setters ────────────────────────────────────────────────────── */

void FLARE_SetRollSP   (float deg) { state.roll_sp     = deg; }
void FLARE_SetPitchSP  (float deg) { state.pitch_sp    = deg; }
void FLARE_SetYawRateSP(float dps) { state.yaw_rate_sp = dps; }