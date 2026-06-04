/* firmware/fc/Core/Inc/imu_fusion.h */
#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "lpf.h"
#include <stdint.h>


typedef struct {
    /* Fused attitude angles (degrees) */
    float roll;
    float pitch;
    float yaw;

    /*
     * Filtered gyro rates (deg/s) — written each update.
     * Use these in FLARE_Update instead of re-converting raw LSB,
     * so the PID D-term receives the LPF-filtered signal.
     */
    float gx_dps;
    float gy_dps;
    float gz_dps;

    /* Low-pass filters — one per axis, gyro and accel */
    LPF_t lpf_gx;
    LPF_t lpf_gy;
    LPF_t lpf_gz;
    LPF_t lpf_ax;
    LPF_t lpf_ay;
    LPF_t lpf_az;
} IMU_Fusion_t;

/*
 * Initialise the fusion state and all LPF instances.
 *
 *   dt          — sample period in seconds (0.01 at 100Hz)
 *   fc_gyro_hz  — gyro LPF cutoff frequency (recommended: 80.0f)
 *   fc_accel_hz — accel LPF cutoff frequency (recommended: 30.0f)
 */
void IMU_Fusion_Init(IMU_Fusion_t *f, float dt, float fc_gyro_hz, float fc_accel_hz);

void IMU_Fusion_Update(IMU_Fusion_t *f, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
                       int16_t gz, float mag_heading, float dt, float alpha, float beta);

#endif /* IMU_FUSION_H */