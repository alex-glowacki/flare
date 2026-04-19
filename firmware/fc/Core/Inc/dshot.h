/* Core/Inc/dshot.h
 *
 * DSHOT300 driver — TIM4 PWM + TIM4_UP DMA burst
 *
 * Motor mapping:
 *   M1 → TIM4_CH1 → PD12   (Front-Left)
 *   M2 → TIM4_CH2 → PD13   (Front-Right)
 *   M3 → TIM4_CH3 → PD14   (Rear-Right)
 *   M4 → TIM4_CH4 → PD15   (Rear-Left)
 *
 * Throttle range: 0 (disarmed) or 48–2047 (armed, min–max)
 * Values 1–47 are reserved by the DSHOT protocol.
 */

#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize the DSHOT driver.
 *         Must be called once after MX_TIM4_Init() and MX_DMA_Init().
 *         Starts TIM4 PWM on all 4 channels and arms the DMA transfer.
 */
void DSHOT_Init(void);

/**
 * @brief  Send a DSHOT300 frame to all four motors.
 *
 * @param  m1  Throttle for motor 1 (Front-Left),  0 or 48–2047
 * @param  m2  Throttle for motor 2 (Front-Right), 0 or 48–2047
 * @param  m3  Throttle for motor 3 (Rear-Right),  0 or 48–2047
 * @param  m4  Throttle for motor 4 (Rear-Left),   0 or 48–2047
 */
void DSHOT_SendThrottle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif /* DSHOT_H */