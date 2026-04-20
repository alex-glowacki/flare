/* firmware/fc/Core/Inc/imu_fusion.h */
#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Complementary filter state ─────────────────────────────────────────── */
typedef struct {
  float roll;  /* degrees, positive = right side down */
  float pitch; /* degrees, positive = nose up         */
  float yaw;   /* degrees, 0 = magnetic north, 0–360  */
} IMU_Fusion_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the fusion state to zero.
 * @param  f  Pointer to IMU_Fusion_t instance.
 */
void IMU_Fusion_Init(IMU_Fusion_t *f);

/**
 * @brief  Run one complementary filter update step.
 *
 * @param  f            Pointer to IMU_Fusion_t instance.
 * @param  ax           Raw accel X (LSB, ±8g range)
 * @param  ay           Raw accel Y (LSB, ±8g range)
 * @param  az           Raw accel Z (LSB, ±8g range)
 * @param  gx           Raw gyro X  (LSB, ±2000 dps range)
 * @param  gy           Raw gyro Y  (LSB, ±2000 dps range)
 * @param  gz           Raw gyro Z  (LSB, ±2000 dps range)
 * @param  mag_heading  Absolute heading from magnetometer in degrees [0, 360).
 *                      Pass the output of MAG_ReadHeading() here.
 * @param  dt           Time step in seconds (0.01 at 100 Hz).
 * @param  alpha        Roll/pitch filter coefficient 0–1 (0.96 recommended).
 *                      Higher = trust gyro more, lower = trust accel more.
 * @param  beta         Yaw filter coefficient 0–1 (0.90 recommended).
 *                      Higher = trust gyro more, lower = trust mag more.
 *                      Lower than alpha because mag is noisier than accel.
 */
void IMU_Fusion_Update(IMU_Fusion_t *f, int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz, float mag_heading,
                       float dt, float alpha, float beta);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_H */