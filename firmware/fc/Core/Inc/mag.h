/* firmware/fc/Core/Inc/mag.h */
#ifndef MAG_H
#define MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include <stdint.h>

/* ── QMC5883P I2C address (7-bit: 0x2C, HAL uses 8-bit shifted) ─────────── */
#define QMC5883P_ADDR (0x2C << 1)

/* ── Register map ───────────────────────────────────────────────────────── */
#define QMC5883P_REG_CHIP_ID 0x00 /* Chip ID — returns 0x80              */
#define QMC5883P_REG_XOUT_L 0x01  /* X LSB (burst: X,Y,Z = 6 bytes)     */
#define QMC5883P_REG_XOUT_H 0x02
#define QMC5883P_REG_YOUT_L 0x03
#define QMC5883P_REG_YOUT_H 0x04
#define QMC5883P_REG_ZOUT_L 0x05
#define QMC5883P_REG_ZOUT_H 0x06
#define QMC5883P_REG_STATUS 0x09 /* Bit 0 = DRDY, Bit 1 = OVL          */
#define QMC5883P_REG_CTRL 0x0A   /* Mode, ODR, RNG, OSR config          */

/* ── Chip ID ────────────────────────────────────────────────────────────── */
#define QMC5883P_CHIP_ID 0x80 /* Expected value of register 0x00     */

/* ── CTRL register (0x0A) bitfield values ───────────────────────────────── */
/*
 * MODE[1:0] = 01  → Normal mode (continuous measurement, with sleep)
 * ODR[3:2]  = 10  → 100 Hz output data rate
 * RNG[5:4]  = 00  → ±30 Gauss full scale (widest — safest default)
 * OSR1[7:6] = 00  → Over-sample ratio 1 (low power)
 *
 * Packed: OSR1=00 | RNG=00 | ODR=10 | MODE=01 = 0b00_00_10_01 = 0x09
 *
 * Note: QMC5883P has no FBR (SET/RESET period) register — do not write 0x0B.
 */
#define QMC5883P_CTRL_VAL 0x09

/* ── STATUS register bits ───────────────────────────────────────────────── */
#define QMC5883P_STATUS_DRDY 0x01 /* Data ready                          */
#define QMC5883P_STATUS_OVL 0x02  /* Overflow                            */

/* ── Return codes ───────────────────────────────────────────────────────── */
#define MAG_OK 0
#define MAG_ERR_ID 1        /* Chip ID mismatch                           */
#define MAG_ERR_I2C 2       /* HAL I2C call failed                        */
#define MAG_ERR_NOT_READY 3 /* DRDY not set within timeout                */

/* ── Calibration offsets (hard-iron correction) ─────────────────────────── */
/*
 * Filled in after a calibration spin. Until then they are zero and the
 * heading will have hard-iron error. Set via MAG_SetCalibration() once you
 * have min/max values from a 360° rotation sweep.
 */
typedef struct {
    float offset_x; /* (max_x + min_x) / 2 */
    float offset_y; /* (max_y + min_y) / 2 */
} MAG_Cal_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the QMC5883P. Verifies chip ID, writes CTRL register.
 * @param  hi2c     Pointer to the HAL I2C handle (hi2c1).
 * @param  chip_id  Output: raw chip ID byte read from register 0x00.
 *                  Inspect over UART if init fails unexpectedly.
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
uint8_t MAG_ReadRaw(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief  Read heading in degrees [0, 360).
 *         Applies hard-iron calibration offsets if set.
 * @param  hi2c     Pointer to the HAL I2C handle.
 * @param  cal      Pointer to calibration struct (offsets may be zero).
 * @param  heading  Output: heading in degrees, 0 = magnetic north.
 * @return MAG_OK, MAG_ERR_I2C, or MAG_ERR_NOT_READY.
 */
uint8_t MAG_ReadHeading(I2C_HandleTypeDef *hi2c, const MAG_Cal_t *cal, float *heading);

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