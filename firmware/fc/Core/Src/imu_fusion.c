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

/*
 * IMU mounting offset correction.
 * Measured from LOG001.CSV post-disarm settling (t=80-88s, motors off,
 * frame stationary on flat ground). Subtract from accel-derived angles
 * so the complementary filter treats level-frame as 0°/0°.
 * Re-calibrate if the IMU is remounted.
 */
#define IMU_ROLL_OFFSET  ( 0.27f)   /* deg — roll reading when frame is level  */
#define IMU_PITCH_OFFSET (-3.80f)   /* deg — pitch reading when frame is level */

/* ── Public API implementation ───────────────────────────────────────────── */

void IMU_Fusion_Init(IMU_Fusion_t *f, float dt, float fc_gyro_hz, float fc_accel_hz) {
    f->roll  = 0.0f;
    f->pitch = 0.0f;
    f->yaw   = 0.0f;

    f->gx_dps = 0.0f;
    f->gy_dps = 0.0f;
    f->gz_dps = 0.0f;

    float alpha_gyro  = LPF_ComputeAlpha(dt, fc_gyro_hz);
    float alpha_accel = LPF_ComputeAlpha(dt, fc_accel_hz);

    LPF_Init(&f->lpf_gx, alpha_gyro);
    LPF_Init(&f->lpf_gy, alpha_gyro);
    LPF_Init(&f->lpf_gz, alpha_gyro);
    LPF_Init(&f->lpf_ax, alpha_accel);
    LPF_Init(&f->lpf_ay, alpha_accel);
    LPF_Init(&f->lpf_az, alpha_accel);
}

void IMU_Fusion_Update(IMU_Fusion_t *f, int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz, float mag_heading,
                       float dt, float alpha, float beta) {

    /* ── 1. Convert raw LSB → physical units ─────────────────────────────── */
    float ax_g = (float)ax * ACC_SCALE;
    float ay_g = (float)ay * ACC_SCALE;
    float az_g = (float)az * ACC_SCALE;

    float gx_dps = (float)gx * GYR_SCALE;
    float gy_dps = (float)gy * GYR_SCALE;
    float gz_dps = (float)gz * GYR_SCALE;

    /* ── 2. Low-pass filter — gyro rates and accel ───────────────────────── */
    /*
     * Filter gyro before feeding into integration and PID.
     * Gyro fc=80Hz: attenuates motor harmonics (~200-400Hz on 1000KV/3S)
     * while preserving attitude bandwidth. Primary D-term noise guard.
     *
     * Filter accel before atan2 angle computation.
     * Accel fc=30Hz: accel response is slow; aggressive filtering is fine
     * and reduces the noise contribution to the complementary filter
     * reference angle.
     *
     * Filtered gyro rates are stored in the struct so main.c can pass
     * them to FLARE_Update without re-converting raw LSB.
     */
    gx_dps = LPF_Update(&f->lpf_gx, gx_dps);
    gy_dps = LPF_Update(&f->lpf_gy, gy_dps);
    gz_dps = LPF_Update(&f->lpf_gz, gz_dps);

    f->gx_dps = gx_dps;
    f->gy_dps = gy_dps;
    f->gz_dps = gz_dps;

    ax_g = LPF_Update(&f->lpf_ax, ax_g);
    ay_g = LPF_Update(&f->lpf_ay, ay_g);
    az_g = LPF_Update(&f->lpf_az, az_g);

    /* ── 3. Accel-derived roll and pitch (degrees) ────────────────────────── */
    float roll_accel  = atan2f(ay_g, az_g)  * (180.0f / (float)M_PI) - IMU_ROLL_OFFSET;
    float pitch_accel = atan2f(-ax_g, az_g) * (180.0f / (float)M_PI) - IMU_PITCH_OFFSET;

    /* ── 4. Gyro-integrated roll, pitch, and yaw (degrees) ───────────────── */
    float roll_gyro  = f->roll  + gx_dps * dt;
    float pitch_gyro = f->pitch + gy_dps * dt;
    float yaw_gyro   = f->yaw   + gz_dps * dt;

    /* ── 5. Complementary filter — roll and pitch ────────────────────────── */
    f->roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_accel;
    f->pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_accel;

    /* ── 6. Complementary filter — yaw (gyro + magnetometer) ─────────────── */
    /*
     * Yaw cannot be corrected by the accelerometer — it has no sensitivity
     * to rotation around the gravity vector. The magnetometer provides the
     * absolute heading reference instead.
     *
     * shortest_delta normalises the mag-gyro difference to (-180, +180]
     * to avoid the wrap discontinuity at 0°/360°.
     */
    float delta = mag_heading - yaw_gyro;

    while (delta >  180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;

    float yaw_fused = yaw_gyro + (1.0f - beta) * delta;

    while (yaw_fused <    0.0f) yaw_fused += 360.0f;
    while (yaw_fused >= 360.0f) yaw_fused -= 360.0f;

    f->yaw = yaw_fused;
}