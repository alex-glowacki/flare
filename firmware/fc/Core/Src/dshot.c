/* Core/Src/dshot.c
 *
 * DSHOT300 driver — TIM4 PWM + TIM4_UP DMA, one-shot per frame
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
 *
 * Approach: Normal mode DMA, one complete 17-row frame per call.
 * DSHOT_SendThrottle() builds the frame, starts DMA, and returns.
 * The caller is responsible for calling at 100Hz (every 10ms).
 * The 17th row (reset slot = 0) ensures ESC latches the frame.
 * A clean gap exists between calls at 100Hz (10ms >> 56µs frame).
 *
 * Buffer layout: dshot_buf[row][motor], 17 rows × 4 motors = 68 words.
 * HAL_TIM_DMABurst_WriteStart BurstLength = 68 (total words to transfer).
 * On each TIM4 update event, the DMA writes 4 consecutive words (CCR1-CCR4),
 * advancing one row per timer overflow until all 17 rows are sent.
 *
 * STM32H7: buffer placed in D1 AXI SRAM (0x24000000) for DMA access.
 * D-cache flushed after each buffer write.
 */

#include "dshot.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* ── Timing constants ───────────────────────────────────────────────────── */
#define DSHOT_BIT_1 480U
#define DSHOT_BIT_0 240U
#define DSHOT_FRAME_BITS 16U
#define DSHOT_BUF_LEN 17U /* 16 data bits + 1 reset slot */
#define DSHOT_NUM_MOTORS 4U

#define DSHOT_BUF_ADDR 0x24000000UL
#define DSHOT_BUF_SIZE (DSHOT_BUF_LEN * DSHOT_NUM_MOTORS * sizeof(uint32_t))
#define DSHOT_BURST_LEN (DSHOT_BUF_LEN * DSHOT_NUM_MOTORS) /* 68 words */

static uint32_t (*const dshot_buf)[DSHOT_NUM_MOTORS] =
    (uint32_t (*)[DSHOT_NUM_MOTORS])DSHOT_BUF_ADDR;

/* ── Internal helpers ───────────────────────────────────────────────────── */

static uint16_t DSHOT_CalcCRC(uint16_t throttle_telem) {
  uint16_t v = throttle_telem;
  return (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F;
}

static uint16_t DSHOT_BuildFrame(uint16_t throttle) {
  if (throttle > 0 && throttle < 48)
    throttle = 0;
  if (throttle > 2047)
    throttle = 2047;

  uint16_t telemetry = 0;
  uint16_t throttle_telem = (throttle << 1) | telemetry;
  uint16_t crc = DSHOT_CalcCRC(throttle_telem);

  return (throttle_telem << 4) | crc;
}

static void DSHOT_SerialiseFrame(uint16_t frame, uint8_t motor) {
  for (uint8_t bit = 0; bit < DSHOT_FRAME_BITS; bit++) {
    uint8_t row = DSHOT_FRAME_BITS - 1 - bit;
    dshot_buf[row][motor] = (frame & (1U << bit)) ? DSHOT_BIT_1 : DSHOT_BIT_0;
  }
  dshot_buf[DSHOT_FRAME_BITS][motor] = 0U; /* reset slot */
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void DSHOT_Init(void) {
  memset((void *)DSHOT_BUF_ADDR, 0, DSHOT_BUF_SIZE);
  SCB_CleanDCache_by_Addr((uint32_t *)DSHOT_BUF_ADDR, DSHOT_BUF_SIZE);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void DSHOT_SendThrottle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
  /* Build all 4 motor frames into the buffer */
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m1), 0);
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m2), 1);
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m3), 2);
  DSHOT_SerialiseFrame(DSHOT_BuildFrame(m4), 3);

  SCB_CleanDCache_by_Addr((uint32_t *)DSHOT_BUF_ADDR, DSHOT_BUF_SIZE);

  /*
   * Stop any in-progress burst, then start a fresh one-shot transfer.
   *
   * BurstLength = DSHOT_BURST_LEN (68 words) — total words to transfer.
   * On each TIM4 update event (every 640 ticks = 3.333µs), the DMA
   * writes 4 consecutive words into CCR1-CCR4, advancing one row.
   * After 17 update events (17 rows), DMA stops automatically (Normal mode).
   *
   * The 10ms inter-call gap (100Hz) is >> the 56µs frame duration,
   * so there is always a clean reset gap between frames.
   */
  HAL_TIM_DMABurst_WriteStop(&htim4, TIM_DMA_UPDATE);

  HAL_TIM_DMABurst_WriteStart(&htim4, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                              (uint32_t *)DSHOT_BUF_ADDR, DSHOT_BURST_LEN);
}