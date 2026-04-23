/* firmware/fc/Core/Inc/rc.h */
/* =============================================================================
 * FLARE — RC receiver (USART2 interrupt-driven)
 *
 * Receives FLARE_RC_Packet_t frames from the quad-side Nano ESP32 over UART.
 * The ISR assembles bytes into a staging buffer; once FLARE_PACKET_SIZE bytes
 * have arrived and the packet passes magic + CRC-8 checks, it is copied into
 * a validated shadow buffer and a flag is set for the main loop to consume.
 *
 * All public functions are safe to call from the main loop context.
 * RC_UART_RxCpltCallback() must be called from HAL_UART_RxCpltCallback().
 * ============================================================================= */

#ifndef __RC_H__
#define __RC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "flare_protocol.h"

/* ---------------------------------------------------------------------------
 * Timeout — if no valid packet arrives within this many milliseconds,
 * RC_IsHealthy() returns 0 and callers should treat link as lost.
 * At 100Hz the remote sends a packet every 10ms; 250ms = 25 missed packets.
 * --------------------------------------------------------------------------- */
#define RC_TIMEOUT_MS   250U

/* ---------------------------------------------------------------------------
 * RC_Init()
 *
 * Arms the first HAL_UART_Receive_IT() call on huart2 to begin byte-by-byte
 * reception. Call once after MX_USART2_UART_Init() in main().
 * --------------------------------------------------------------------------- */
void RC_Init(void);

/* ---------------------------------------------------------------------------
 * RC_UART_RxCpltCallback()
 *
 * Must be called from HAL_UART_RxCpltCallback() when huart->Instance == USART2.
 * Assembles received bytes, validates complete packets, re-arms the interrupt.
 * --------------------------------------------------------------------------- */
void RC_UART_RxCpltCallback(void);

/* ---------------------------------------------------------------------------
 * RC_GetPacket()
 *
 * Returns 1 and copies the latest validated packet into *out if a new packet
 * has arrived since the last call. Returns 0 if no new packet is available.
 * Clears the new-packet flag on success.
 * --------------------------------------------------------------------------- */
uint8_t RC_GetPacket(FLARE_RC_Packet_t *out);

/* ---------------------------------------------------------------------------
 * RC_IsHealthy()
 *
 * Returns 1 if a valid packet has been received within RC_TIMEOUT_MS.
 * Returns 0 if the link is stale or has never received a packet.
 * --------------------------------------------------------------------------- */
uint8_t RC_IsHealthy(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC_H__ */