/* firmware/fc/Core/Src/rc.c */
/* =============================================================================
 * FLARE — RC receiver (USART2 interrupt-driven)
 *
 * Reception strategy:
 *   HAL_UART_Receive_IT() is armed for 1 byte at a time. Each RxCplt callback
 *   appends the byte to a staging buffer. When FLARE_PACKET_SIZE bytes have
 *   accumulated, the buffer is validated (magic byte + CRC-8). On success the
 *   packet is copied to the shadow buffer and the new-packet flag is set.
 *   The staging buffer is always reset after a full frame attempt regardless
 *   of outcome, so a corrupted packet does not stall reception.
 *
 * Sync recovery:
 *   If the first byte of a new frame is not FLARE_PACKET_MAGIC, the buffer is
 *   discarded immediately and the search for a valid magic byte restarts. This
 *   keeps the receiver self-synchronising after any byte-slip or noise burst.
 * ============================================================================= */

#include "rc.h"
#include "usart.h"
#include "flare_protocol.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Internal state
 * All written from the UART ISR context — declared volatile where the main
 * loop reads them, and accessed atomically (single-word flags/pointers).
 * --------------------------------------------------------------------------- */

/* Staging buffer — accumulates raw bytes from the ISR */
static uint8_t  rx_byte;                            /* single-byte DMA target  */
static uint8_t  rx_buf[FLARE_PACKET_SIZE];          /* assembles one frame     */
static uint8_t  rx_count = 0;                       /* bytes received so far   */

/* Shadow buffer — last validated packet, safe to read from main loop */
static FLARE_RC_Packet_t rc_packet;

/* Flags — written by ISR, read by main loop */
static volatile uint8_t  rc_new_packet  = 0;        /* 1 = new packet ready    */
static volatile uint32_t rc_last_rx_ms  = 0;        /* HAL_GetTick() at last good packet */
static volatile uint8_t  rc_ever_received = 0;      /* 1 once first pkt arrives */

/* ---------------------------------------------------------------------------
 * RC_Init
 * --------------------------------------------------------------------------- */
void RC_Init(void)
{
    rx_count       = 0;
    rc_new_packet  = 0;
    rc_last_rx_ms  = 0;
    rc_ever_received = 0;
    memset(&rc_packet, 0, sizeof(rc_packet));

    /* Arm first byte reception — subsequent re-arms happen in callback */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* ---------------------------------------------------------------------------
 * RC_UART_RxCpltCallback
 *
 * Called from HAL_UART_RxCpltCallback() when huart->Instance == USART2.
 * Runs in ISR context — keep it short, no HAL_Delay, no UART_Print.
 * --------------------------------------------------------------------------- */
void RC_UART_RxCpltCallback(void)
{
    /* --- Sync recovery: if this is the first byte of a frame and it is not
     *     the magic byte, discard it and wait for a valid frame start.    --- */
    if (rx_count == 0 && rx_byte != FLARE_PACKET_MAGIC) {
        /* Re-arm and keep searching */
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
        return;
    }

    /* Append byte to staging buffer */
    rx_buf[rx_count++] = rx_byte;

    /* Full frame received — validate and consume */
    if (rx_count == FLARE_PACKET_SIZE) {
        rx_count = 0;   /* reset regardless of outcome */

        const FLARE_RC_Packet_t *pkt = (const FLARE_RC_Packet_t *)rx_buf;

        /* Magic check (redundant with sync recovery but keeps validation
         * self-contained and guards against single-byte buffer corruption) */
        if (pkt->magic != FLARE_PACKET_MAGIC) {
            HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
            return;
        }

        /* CRC-8 check */
        if (pkt->checksum != flare_checksum(pkt)) {
            HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
            return;
        }

        /* Valid packet — copy to shadow buffer and signal main loop */
        memcpy(&rc_packet, pkt, sizeof(FLARE_RC_Packet_t));
        rc_new_packet    = 1;
        rc_last_rx_ms    = HAL_GetTick();
        rc_ever_received = 1;
    }

    /* Re-arm for next byte */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* ---------------------------------------------------------------------------
 * RC_GetPacket
 * --------------------------------------------------------------------------- */
uint8_t RC_GetPacket(FLARE_RC_Packet_t *out)
{
    if (!rc_new_packet) {
        return 0;
    }
    /* Copy shadow buffer and clear flag.
     * Single-byte flag write is atomic on Cortex-M7 — no critical section
     * needed here since the ISR only ever sets it, never clears it. */
    memcpy(out, &rc_packet, sizeof(FLARE_RC_Packet_t));
    rc_new_packet = 0;
    return 1;
}

/* ---------------------------------------------------------------------------
 * RC_IsHealthy
 * --------------------------------------------------------------------------- */
uint8_t RC_IsHealthy(void)
{
    if (!rc_ever_received) {
        return 0;
    }
    return (HAL_GetTick() - rc_last_rx_ms) < RC_TIMEOUT_MS;
}