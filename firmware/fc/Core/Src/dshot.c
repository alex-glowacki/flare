/* Core/Src/dshot.c
 *
 * DSHOT300 driver — TIM4 PWM + TIM4_UP DMA burst
 *
 * Timing (@ 192MHz SYSCLK, APB1 timer clock = 192MHz, ARR = 639):
 *   Bit period  = 640 ticks = 3.333 µs   (DSHOT300 = 300 kbps)
 *   BIT_1       = 480 ticks = 2.500 µs   (75% duty)
 *   BIT_0       = 240 ticks = 1.250 µs   (37.5% duty)
 *   Reset slot  =   0 ticks              (pin low, ESC latches frame)
 *
 * Frame format (16 bits, MSB first):
 *   [15:5]  11-bit throttle value (0 = disarm, 48–2047 = throttle)
 *   [4]     telemetry request bit (0 = no telemetry)
 *   [3:0]   4-bit CRC = (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F
 *           where v = (throttle << 1) | telemetry
 *
 * DMA buffer layout:
 *   dshot_buf[bit][motor]  — 17 rows (16 data bits + 1 reset), 4 columns
 *   Columns map to CCR1–CCR4 via consecutive register addresses.
 *   DMA writes each row to TIM4->CCR1 on every timer overflow.
 */

#include "dshot.h"
#include "main.h"
#include "tim.h"
#include <string.h>

/* ── Timing constants ───────────────────────────────────────────────────── */
#define DSHOT_BIT_1 480U /* 75%   of ARR+1=640 */
#define DSHOT_BIT_0 240U /* 37.5% of ARR+1=640 */
#define DSHOT_FRAME_BITS 16U
#define DSHOT_BUF_LEN 17U /* 16 data bits + 1 reset slot */
#define DSHOT_NUM_MOTORS 4U

/*
 * DMA burst buffer.
 *
 * Layout: dshot_buf[bit_index][motor_index]
 *   bit_index  0  = MSB (bit 15)
 *   bit_index  15 = LSB (bit  0)
 *   bit_index  16 = reset slot (all zeros)
 *
 * __attribute__((aligned(4))) ensures the buffer starts on a 32-bit
 * boundary as required by the STM32H7 DMA controller.
 */
static uint32_t dshot_buf[DSHOT_BUF_LEN][DSHOT_NUM_MOTORS]
    __attribute__((aligned(4)));

/* ── Internal helpers ───────────────────────────────────────────────────── */

/**
 * @brief  Compute the 4-bit DSHOT CRC.
 *
 *         v = (throttle << 1) | telemetry
 *         CRC = (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F
 */
static uint16_t DSHOT_CalcCRC(uint16_t throttle_telem) {
  uint16_t v = throttle_telem;
  return (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F;
}

/**
 * @brief  Build a 16-bit DSHOT frame from a throttle value.
 *
 * @param  throttle  0 (disarm) or 48–2047
 * @retval 16-bit frame ready to be serialised into dshot_buf
 */
static uint16_t DSHOT_BuildFrame(uint16_t throttle) {
  /* Clamp reserved range 1–47 to 0 (disarm) */
  if (throttle > 0 && throttle < 48) {
    throttle = 0;
  }
  if (throttle > 2047) {
    throttle = 2047;
  }

  uint16_t telemetry = 0;
  uint16_t throttle_telem = (throttle << 1) | telemetry;
  uint16_t crc = DSHOT_CalcCRC(throttle_telem);

  return (throttle_telem << 4) | crc;
}

/**
 * @brief  Serialise a 16-bit DSHOT frame into one column of dshot_buf.
 *
 * @param  frame   16-bit DSHOT frame from DSHOT_BuildFrame()
 * @param  motor   Column index (0–3 → M1–M4)
 */
static void DSHOT_SerialiseFrame(uint16_t frame, uint8_t motor) {
  for (uint8_t bit = 0; bit < DSHOT_FRAME_BITS; bit++) {
    /* MSB first — bit 15 goes into row 0 */
    uint8_t row = DSHOT_FRAME_BITS - 1 - bit;
    dshot_buf[row][motor] = (frame & (1U << bit)) ? DSHOT_BIT_1 : DSHOT_BIT_0;
  }
  /* Reset slot — pin held low so ESC latches the frame */
  dshot_buf[DSHOT_FRAME_BITS][motor] = 0U;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void DSHOT_Init(void) {
  memset(dshot_buf, 0, sizeof(dshot_buf));

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /*
   * Start DMA burst on TIM4_UP.
   *
   * On each timer overflow DMA writes 4 consecutive words (CCR1–CCR4)
   * and advances its pointer, repeating for all 17 rows.
   *
   * BurstLength = total uint32_t words = 17 rows × 4 motors = 68.
   */
  HAL_TIM_DMABurst_WriteStart(&htim4, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                              (uint32_t *)dshot_buf,
                              DSHOT_BUF_LEN * DSHOT_NUM_MOTORS);
}

void DSHOT_SendThrottle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m1), 0);
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m2), 1);
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m3), 2);
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m4), 3);

  HAL_TIM_DMABurst_WriteStop(&htim4, TIM_DMA_UPDATE);
  HAL_TIM_DMABurst_WriteStart(&htim4, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                              (uint32_t *)dshot_buf,
                              DSHOT_BUF_LEN * DSHOT_NUM_MOTORS);
}