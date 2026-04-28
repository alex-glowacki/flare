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

#include "flare_protocol.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Timeout — if no valid packet arrives within this many milliseconds,
 * RC_IsHealthy() returns 0 and callers should treat link as lost.
 * At 100Hz the remote sends a packet every 10ms; 250ms = 25 missed packets.
 * --------------------------------------------------------------------------- */
#define RC_TIMEOUT_MS 250U

/* ---------------------------------------------------------------------------
 * Diagnostic counters — remove once the UART link is confirmed working.
 * rc_bytes_received: incremented on every raw byte received on USART2.
 * rc_init_status:    set in RC_Init() — 0=HAL_OK, 1=HAL error/busy.
 * --------------------------------------------------------------------------- */
extern uint8_t rc_rx_byte;
extern volatile uint32_t rc_bytes_received;
extern volatile uint8_t rc_init_status;
extern volatile uint32_t rc_rearm_failures;
extern volatile uint32_t rc_error_callbacks;

/* ---------------------------------------------------------------------------
 * RC_Init()
 * --------------------------------------------------------------------------- */
void RC_Init(void);

/* ---------------------------------------------------------------------------
 * RC_UART_RxCpltCallback()
 * --------------------------------------------------------------------------- */
void RC_UART_RxCpltCallback(void);

/* ---------------------------------------------------------------------------
 * RC_GetPacket()
 * --------------------------------------------------------------------------- */
uint8_t RC_GetPacket(FLARE_RC_Packet_t *out);

/* ---------------------------------------------------------------------------
 * RC_IsHealthy()
 * --------------------------------------------------------------------------- */
uint8_t RC_IsHealthy(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC_H__ */