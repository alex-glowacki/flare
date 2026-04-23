/* Core/Src/dshot.c
 *
 * DSHOT300 driver — TIM4 PWM + direct DMA (no HAL DMA burst API)
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
 * Approach: Direct DMA register programming, bypassing HAL DMA burst API.
 * TIM4->DCR configures burst base (CCR1) and length (4 registers).
 * On each TIM4 update event, DMA writes 4 words to TIM4->DMAR,
 * which the timer routes to CCR1→CCR2→CCR3→CCR4 automatically.
 * After 17 update events (68 words total), DMA stops (Normal mode).
 * The 17th row is the reset slot (all zeros), ensuring ESC latches the frame.
 * On DMA transfer complete, CCR1–CCR4 are set to ARR+1 (640) to hold
 * the output idle-high between frames.
 *
 * Buffer layout: dshot_buf[row][motor], 17 rows × 4 motors = 68 words.
 * Buffer placed in D1 AXI SRAM (0x24000000) for DMA access.
 * D-cache flushed after each buffer write.
 */

#include "dshot.h"
#include "main.h"
#include "tim.h"
#include <string.h>

/* ── Timing constants ───────────────────────────────────────────────────── */
#define DSHOT_BIT_1        480U
#define DSHOT_BIT_0        240U
#define DSHOT_FRAME_BITS   16U
#define DSHOT_BUF_ROWS     17U   /* 16 data bits + 1 reset slot */
#define DSHOT_NUM_MOTORS   4U
#define DSHOT_CCR_IDLE     640U  /* ARR+1 → 100% duty → idle-high between frames */

#define DSHOT_BUF_ADDR  0x24000000UL
#define DSHOT_BUF_SIZE  (DSHOT_BUF_ROWS * DSHOT_NUM_MOTORS * sizeof(uint32_t))
#define DSHOT_BURST_LEN (DSHOT_BUF_ROWS * DSHOT_NUM_MOTORS)  /* 68 words */

static uint32_t (*const dshot_buf)[DSHOT_NUM_MOTORS] =
    (uint32_t (*)[DSHOT_NUM_MOTORS])DSHOT_BUF_ADDR;

/* ── Internal helpers ───────────────────────────────────────────────────── */

static uint16_t DSHOT_BuildFrame(uint16_t throttle) {
    if (throttle > 0 && throttle < 48)
        throttle = 0;
    if (throttle > 2047)
        throttle = 2047;

    uint16_t v = (throttle << 1); /* telemetry bit = 0 */
    uint16_t crc = (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F;
    return (v << 4) | crc;
}

static void DSHOT_SerialiseFrame(uint16_t frame, uint8_t motor) {
    for (uint8_t bit = 0; bit < DSHOT_FRAME_BITS; bit++) {
        uint8_t row = DSHOT_FRAME_BITS - 1 - bit;
        dshot_buf[row][motor] = (frame & (1U << bit)) ? DSHOT_BIT_1 : DSHOT_BIT_0;
    }
    dshot_buf[DSHOT_FRAME_BITS][motor] = 0U; /* reset slot */
}

static void DSHOT_StartDMA(void) {
    DMA_Stream_TypeDef *dma = DMA1_Stream0;

    /* Wait for previous transfer to complete (TC handler disables the stream) */
    while (dma->CR & DMA_SxCR_EN) {}

    /* Clear all interrupt flags for Stream0 */
    DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 |
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    /* Set source, destination, length */
    dma->PAR  = (uint32_t)&TIM4->DMAR;
    dma->M0AR = DSHOT_BUF_ADDR;
    dma->NDTR = DSHOT_BURST_LEN;

    /* Enable TIM4 DMA update request */
    TIM4->DIER |= TIM_DIER_UDE;

    /* Enable DMA stream */
    dma->CR |= DMA_SxCR_EN;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void DSHOT_Init(void) {
    memset((void *)DSHOT_BUF_ADDR, 0, DSHOT_BUF_SIZE);
    SCB_CleanDCache_by_Addr((uint32_t *)DSHOT_BUF_ADDR, DSHOT_BUF_SIZE);

    /* Configure DMA1_Stream0: Memory→Peripheral, word width,
     * memory increment, normal mode, high priority, TC interrupt enabled */
    DMA_Stream_TypeDef *dma = DMA1_Stream0;
    dma->CR &= ~DMA_SxCR_EN;
    while (dma->CR & DMA_SxCR_EN) {}

    dma->CR = DMA_SxCR_DIR_0                   /* Memory to peripheral */
            | DMA_SxCR_MINC                    /* Memory increment */
            | (0x2UL << DMA_SxCR_MSIZE_Pos)   /* Memory word size = 32-bit */
            | (0x2UL << DMA_SxCR_PSIZE_Pos)   /* Peripheral word size = 32-bit */
            | DMA_SxCR_PL_1                    /* Priority high */
            | DMA_SxCR_TCIE;                   /* Transfer-complete interrupt enable */

    dma->FCR = 0; /* Direct mode, no FIFO */

    /* Enable DMA1_Stream0 interrupt in NVIC */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* Configure TIM4 DMA burst: base=CCR1, burst length=4 (CCR1–CCR4) */
    TIM4->DCR = (3U << TIM_DCR_DBL_Pos)    /* DBL=3 → 4 transfers per burst */
              | (13U << TIM_DCR_DBA_Pos);   /* DBA=13 → CCR1 offset in TIM4 */

    /* Pre-set CCRs idle-high so signal is high before first frame */
    TIM4->CCR1 = DSHOT_CCR_IDLE;
    TIM4->CCR2 = DSHOT_CCR_IDLE;
    TIM4->CCR3 = DSHOT_CCR_IDLE;
    TIM4->CCR4 = DSHOT_CCR_IDLE;
}

void DSHOT_SendThrottle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    DSHOT_SerialiseFrame(DSHOT_BuildFrame(m1), 0);
    DSHOT_SerialiseFrame(DSHOT_BuildFrame(m2), 1);
    DSHOT_SerialiseFrame(DSHOT_BuildFrame(m3), 2);
    DSHOT_SerialiseFrame(DSHOT_BuildFrame(m4), 3);

    SCB_CleanDCache_by_Addr((uint32_t *)DSHOT_BUF_ADDR, DSHOT_BUF_SIZE);

    DSHOT_StartDMA();
}