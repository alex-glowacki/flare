/* firmware/fc/Core/Inc/mag.h */
#ifndef MAG_H
#define MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include <stdint.h>

/* ── QMC5883L I2C address (7-bit: 0x0D, HAL uses 8-bit shifted) ─────────── */
#define QMC5883L_ADDR (0x2C << 1)

/* ── Register map ───────────────────────────────────────────────────────── */
#define QMC5883L_REG_XOUT_L 0x00
#define QMC5883L_REG_XOUT_H 0x01
#define QMC5883L_REG_YOUT_L 0x02
#define QMC5883L_REG_YOUT_H 0x03
#define QMC5883L_REG_ZOUT_L 0x04
#define QMC5883L_REG_ZOUT_H 0x05
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_CTRL1 0x09
#define QMC5883L_REG_CTRL2 0x0A
#define QMC5883L_REG_FBR 0x0B
#define QMC5883L_REG_CHIP_ID 0x0D

/* ── CTRL1 bitfield values ──────────────────────────────────────────────── */
/*
 * MODE[1:0] = 01  → Continuous measurement
 * ODR[1:0]  = 11  → 200 Hz output data rate
 * RNG[1:0]  = 01  → ±8 Gauss full scale
 * OSR[1:0]  = 00  → Over-sample ratio 512 (lowest noise)
 *
 * Packed: OSR=00 | RNG=01 | ODR=11 | MODE=01 = 0b00_01_11_01 = 0x1D
 */
#define QMC5883L_CTRL1_VAL 0x1D

/* ── STATUS register bits ───────────────────────────────────────────────── */
#define QMC5883L_STATUS_DRDY 0x01 /* Data ready */
#define QMC5883L_STATUS_OVL 0x02  /* Overflow   */

/* ── Return codes ───────────────────────────────────────────────────────── */
#define MAG_OK 0
#define MAG_ERR_ID 1        /* WHO_AM_I mismatch          */
#define MAG_ERR_I2C 2       /* HAL I2C call failed        */
#define MAG_ERR_NOT_READY 3 /* DRDY not set within timeout */

/* ── Calibration offsets (hard-iron correction) ─────────────────────────── */
/*
 * These are filled in after a calibration spin. Until then they are zero
 * and the heading will have hard-iron error. Set them via MAG_SetCalibration()
 * once you have min/max values from a rotation sweep.
 */
typedef struct {
  float offset_x; /* (max_x + min_x) / 2 */
  float offset_y; /* (max_y + min_y) / 2 */
} MAG_Cal_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the QMC5883L. Verifies chip ID, writes FBR and CTRL1.
 * @param  hi2c   Pointer to the HAL I2C handle (hi2c1).
 * @param  chip_id Output: the raw chip ID byte read from register 0x0D.
 *                 Inspect this over UART if init fails unexpectedly.
 * @return MAG_OK on success, MAG_ERR_* on failure.
 */
uint8_t MAG_Init(I2C_HandleTypeDef *hi2c, uint8_t *chip_id);

/**
 * @brief  Read raw X/Y/Z counts from the magnetometer.
 * @param  hi2c  Pointer to the HAL I2C handle.
 * @param  x     Output: raw signed X count.
 * @param  y     Output: raw signed Y count.
 * @param  z     Output: raw signed Z count.
 * @return MAG_OK, MAG_ERR_I2C, or MAG_ERR_NOT_READY.
 */
uint8_t MAG_ReadRaw(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y,
                    int16_t *z);

/**
 * @brief  Read heading in degrees [0, 360).
 *         Applies hard-iron calibration offsets if set.
 * @param  hi2c     Pointer to the HAL I2C handle.
 * @param  cal      Pointer to calibration struct (offsets may be zero).
 * @param  heading  Output: heading in degrees, 0 = magnetic north.
 * @return MAG_OK, MAG_ERR_I2C, or MAG_ERR_NOT_READY.
 */
uint8_t MAG_ReadHeading(I2C_HandleTypeDef *hi2c, const MAG_Cal_t *cal,
                        float *heading);

/**
 * @brief  Set hard-iron calibration offsets.
 * @param  cal      Pointer to MAG_Cal_t to update.
 * @param  offset_x (max_x + min_x) / 2  from a rotation sweep.
 * @param  offset_y (max_y + min_y) / 2  from a rotation sweep.
 */
void MAG_SetCalibration(MAG_Cal_t *cal, float offset_x, float offset_y);

#ifdef __cplusplus
}
#endif

#endif /* MAG_H */