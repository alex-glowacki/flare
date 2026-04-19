#include "flare.h"
#include "dshot.h"
#include "pid.h"
#include <stdint.h>

static FLARE_State state;
static PID_Controller pid_roll;
static PID_Controller pid_pitch;
static PID_Controller pid_yaw;

/* Clamp a float to [min, max] */
static float clamp(float v, float min, float max) {
  if (v < min)
    return min;
  if (v > max)
    return max;
  return v;
}

void FLARE_Init(void) {
  state.roll_sp = 0.0f;
  state.pitch_sp = 0.0f;
  state.yaw_rate_sp = 0.0f;
  state.throttle = 0;
  state.armed = 0;

  /*
   * Initial PID gains — these WILL need tuning.
   * Start conservative: low P, no I, small D.
   * kp, ki, kd, integral_limit, output_limit
   */
  PID_Init(&pid_roll, 1.5f, 0.0f, 0.05f, 20.0f, 200.0f);
  PID_Init(&pid_pitch, 1.5f, 0.0f, 0.05f, 20.0f, 200.0f);
  PID_Init(&pid_yaw, 2.0f, 0.0f, 0.0f, 20.0f, 200.0f);
}

void FLARE_Update(float roll, float pitch, float gx, float gy, float gz,
                  float dt) {
  if (!state.armed || state.throttle < 48) {
    DSHOT_SendThrottle(0, 0, 0, 0);
    PID_Reset(&pid_roll);
    PID_Reset(&pid_pitch);
    PID_Reset(&pid_yaw);
    return;
  }

  float out_roll = PID_Update(&pid_roll, state.roll_sp, roll, gx, dt);
  float out_pitch = PID_Update(&pid_pitch, state.pitch_sp, pitch, gy, dt);
  float out_yaw = PID_Update(&pid_yaw, state.yaw_rate_sp, 0.0f, gz, dt);

  /*
   * X-frame motor mixing:
   * M1 Front-Left  = throttle + roll - pitch - yaw
   * M2 Front-Right = throttle - roll - pitch + yaw
   * M3 Rear-Right  = throttle - roll + pitch - yaw
   * M4 Rear-Left   = throttle + roll + pitch + yaw
   */
  float t = (float)state.throttle;

  float m1 = t + out_roll - out_pitch - out_yaw;
  float m2 = t - out_roll - out_pitch + out_yaw;
  float m3 = t - out_roll + out_pitch - out_yaw;
  float m4 = t + out_roll + out_pitch + out_yaw;

  uint16_t m1_cmd = (uint16_t)clamp(m1, 48.0f, 2047.0f);
  uint16_t m2_cmd = (uint16_t)clamp(m2, 48.0f, 2047.0f);
  uint16_t m3_cmd = (uint16_t)clamp(m3, 48.0f, 2047.0f);
  uint16_t m4_cmd = (uint16_t)clamp(m4, 48.0f, 2047.0f);

  DSHOT_SendThrottle(m1_cmd, m2_cmd, m3_cmd, m4_cmd);
}

/* ── Arming and throttle control ─────────────────────────────────────────── */

void FLARE_SetArmed(uint8_t armed) {
  state.armed = armed;
  if (!armed) {
    PID_Reset(&pid_roll);
    PID_Reset(&pid_pitch);
    PID_Reset(&pid_yaw);
  }
}

void FLARE_SetThrottle(uint16_t throttle) { state.throttle = throttle; }