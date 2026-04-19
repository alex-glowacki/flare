/* firmware/fc/Core/Src/imu_fusion.c */
#include "imu_fusion.h"
#include <math.h>

/* ── Scale factors ───────────────────────────────────────────────────────── */

/*
 * ACC_CONF bits[3:0] = 0x8 → ±8g range
 * 16-bit signed → 32768 counts full scale
 * 1 LSB = 8.0 / 32768.0 g
 */
#define ACC_SCALE (8.0f / 32768.0f)

/*
 * GYR_CONF bits[3:0] = 0x8 → ±2000 dps range
 * 1 LSB = 2000.0 / 32768.0 deg/s
 */
#define GYR_SCALE (2000.0f / 32768.0f)

/* ── Public API implementation ───────────────────────────────────────────── */

void IMU_Fusion_Init(IMU_Fusion_t *f) {
  f->roll = 0.0f;
  f->pitch = 0.0f;
}

void IMU_Fusion_Update(IMU_Fusion_t *f, int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz, float dt,
                       float alpha) {
  /* ── 1. Convert raw LSB → physical units ──────────────────────────────── */
  float ax_g = (float)ax * ACC_SCALE;
  float ay_g = (float)ay * ACC_SCALE;
  float az_g = (float)az * ACC_SCALE;

  float gx_dps = (float)gx * GYR_SCALE; /* roll rate  */
  float gy_dps = (float)gy * GYR_SCALE; /* pitch rate */
  (void)gz;                             /* yaw — unused, suppress warning */

  /* ── 2. Accel-derived roll and pitch (degrees) ────────────────────────── */
  /*
   * atan2f gives the angle in radians; multiply by (180/π) to convert.
   *
   * roll:  rotation around X axis — use ay and az
   * pitch: rotation around Y axis — use ax and az
   *
   * These are only accurate when the board is near-static (accel = gravity
   * only). The gyro integration below corrects for dynamic motion.
   */
  float roll_accel = atan2f(ay_g, az_g) * (180.0f / (float)M_PI);
  float pitch_accel = atan2f(-ax_g, az_g) * (180.0f / (float)M_PI);

  /* ── 3. Gyro-integrated roll and pitch (degrees) ─────────────────────── */
  /*
   * Integrate gyro rate over the timestep.
   * Gyro is accurate during motion but drifts over time (no absolute
   * reference). The complementary filter blends it with accel below.
   */
  float roll_gyro = f->roll + gx_dps * dt;
  float pitch_gyro = f->pitch + gy_dps * dt;

  /* ── 4. Complementary filter blend ───────────────────────────────────── */
  /*
   * alpha (e.g. 0.96) weights the gyro-integrated estimate.
   * (1 - alpha) weights the accel-derived estimate.
   *
   * High alpha → trusts gyro → smooth, responsive, drifts slowly.
   * Low  alpha → trusts accel → noisy, but self-correcting.
   * 0.96 at 100 Hz is a standard starting point.
   */
  f->roll = alpha * roll_gyro + (1.0f - alpha) * roll_accel;
  f->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_accel;
}