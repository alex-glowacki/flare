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
  f->yaw = 0.0f;
}

void IMU_Fusion_Update(IMU_Fusion_t *f, int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz, float mag_heading,
                       float dt, float alpha, float beta) {

  /* ── 1. Convert raw LSB → physical units ─────────────────────────────── */
  float ax_g = (float)ax * ACC_SCALE;
  float ay_g = (float)ay * ACC_SCALE;
  float az_g = (float)az * ACC_SCALE;

  float gx_dps = (float)gx * GYR_SCALE; /* roll rate  */
  float gy_dps = (float)gy * GYR_SCALE; /* pitch rate */
  float gz_dps = (float)gz * GYR_SCALE; /* yaw rate   */

  /* ── 2. Accel-derived roll and pitch (degrees) ────────────────────────── */
  /*
   * atan2f gives the angle in radians; multiply by (180/π) to convert.
   * Only accurate when near-static (accel ≈ gravity). Gyro corrects
   * for dynamic motion via the complementary filter below.
   */
  float roll_accel = atan2f(ay_g, az_g) * (180.0f / (float)M_PI);
  float pitch_accel = atan2f(-ax_g, az_g) * (180.0f / (float)M_PI);

  /* ── 3. Gyro-integrated roll, pitch, and yaw (degrees) ───────────────── */
  float roll_gyro = f->roll + gx_dps * dt;
  float pitch_gyro = f->pitch + gy_dps * dt;
  float yaw_gyro = f->yaw + gz_dps * dt;

  /* ── 4. Complementary filter — roll and pitch ────────────────────────── */
  f->roll = alpha * roll_gyro + (1.0f - alpha) * roll_accel;
  f->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_accel;

  /* ── 5. Complementary filter — yaw (gyro + magnetometer) ─────────────── */
  /*
   * Yaw cannot be corrected by the accelerometer — it has no sensitivity
   * to rotation around the gravity vector. The magnetometer provides the
   * absolute heading reference instead.
   *
   * The wrap-around problem: if gyro says 359° and mag says 1°, a naive
   * blend would average to 180° — completely wrong. We instead compute
   * the shortest angular difference between gyro-integrated yaw and the
   * mag heading, then apply the correction proportionally.
   *
   * shortest_delta is in (-180, +180]. Adding it scaled by (1-beta)
   * nudges the gyro estimate toward the mag heading by a small amount
   * each update, without the discontinuity at 0°/360°.
   */
  float delta = mag_heading - yaw_gyro;

  /* Normalise delta to (-180, +180] */
  while (delta > 180.0f)
    delta -= 360.0f;
  while (delta < -180.0f)
    delta += 360.0f;

  float yaw_fused = yaw_gyro + (1.0f - beta) * delta;

  /* Normalise fused yaw to [0, 360) */
  while (yaw_fused < 0.0f)
    yaw_fused += 360.0f;
  while (yaw_fused >= 360.0f)
    yaw_fused -= 360.0f;

  f->yaw = yaw_fused;
}