/* firmware/fc/Core/Src/mag.c */
#include "mag.h"
#include <math.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

static uint8_t MAG_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val) {
  HAL_StatusTypeDef s = HAL_I2C_Mem_Write(hi2c, QMC5883L_ADDR, reg,
                                          I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
  return (s == HAL_OK) ? MAG_OK : MAG_ERR_I2C;
}

static uint8_t MAG_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf,
                            uint8_t len) {
  HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR, reg,
                                         I2C_MEMADD_SIZE_8BIT, buf, len, 10);
  return (s == HAL_OK) ? MAG_OK : MAG_ERR_I2C;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

uint8_t MAG_Init(I2C_HandleTypeDef *hi2c, uint8_t *chip_id) {
  uint8_t ret;
  uint8_t id = 0;

  ret = MAG_ReadRegs(hi2c, QMC5883L_REG_CHIP_ID, &id, 1);
  if (ret != MAG_OK)
    return MAG_ERR_I2C;

  /* Return the raw ID for the caller to inspect over UART */
  if (chip_id != NULL)
    *chip_id = id;

  /* Validate chip ID - QMC5883L must return 0xFF */
  if (id != QMC5883L_CHIP_ID)
      return MAG_ERR_I2C;

  ret = MAG_WriteReg(hi2c, QMC5883L_REG_FBR, 0x01);
  if (ret != MAG_OK)
    return ret;

  ret = MAG_WriteReg(hi2c, QMC5883L_REG_CTRL1, QMC5883L_CTRL1_VAL);
  if (ret != MAG_OK)
    return ret;

  return MAG_OK;
}

uint8_t MAG_ReadRaw(I2C_HandleTypeDef *hi2c, int16_t *x, int16_t *y,
                    int16_t *z) {
  uint8_t ret;
  uint8_t status = 0;

  /*
   * Poll DRDY with a short timeout loop.
   * At 200 Hz, new data arrives every 5ms. 20 iterations × ~1ms = 20ms
   * maximum wait — well within our 10ms loop budget on first call,
   * and essentially instant on subsequent calls.
   */
  for (int i = 0; i < 20; i++) {
    ret = MAG_ReadRegs(hi2c, QMC5883L_REG_STATUS, &status, 1);
    if (ret != MAG_OK)
      return MAG_ERR_I2C;
    if (status & QMC5883L_STATUS_DRDY)
      break;
    HAL_Delay(1);
    if (i == 19)
      return MAG_ERR_NOT_READY;
  }

  /*
   * Burst-read all 6 data bytes starting at XOUT_L (0x00).
   * QMC5883L data format: LSB first (little-endian), 16-bit 2's complement.
   * Register order: XL, XH, YL, YH, ZL, ZH
   */
  uint8_t buf[6] = {0};
  ret = MAG_ReadRegs(hi2c, QMC5883L_REG_XOUT_L, buf, 6);
  if (ret != MAG_OK)
    return MAG_ERR_I2C;

  *x = (int16_t)(buf[0] | (buf[1] << 8));
  *y = (int16_t)(buf[2] | (buf[3] << 8));
  *z = (int16_t)(buf[4] | (buf[5] << 8));

  return MAG_OK;
}

uint8_t MAG_ReadHeading(I2C_HandleTypeDef *hi2c, const MAG_Cal_t *cal,
                        float *heading) {
  int16_t rx, ry, rz;
  uint8_t ret = MAG_ReadRaw(hi2c, &rx, &ry, &rz);
  if (ret != MAG_OK)
    return ret;

  /* Apply hard-iron correction */
  float x = (float)rx - cal->offset_x;
  float y = (float)ry - cal->offset_y;

  /*
   * atan2f(y, x) returns heading in radians from -π to +π.
   * Convert to degrees and shift to [0, 360).
   */
  float h = atan2f(y, x) * (180.0f / (float)M_PI);
  if (h < 0.0f)
    h += 360.0f;

  *heading = h;
  return MAG_OK;
}

void MAG_SetCalibration(MAG_Cal_t *cal, float offset_x, float offset_y) {
  cal->offset_x = offset_x;
  cal->offset_y = offset_y;
}