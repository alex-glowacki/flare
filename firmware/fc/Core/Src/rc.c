/* firmware/fc/Core/Src/rc.c */
/* =============================================================================
 * FLARE — RC receiver (USART2 interrupt-driven)
 * ============================================================================= */

#include "main.h"
#include "rc.h"
#include "usart.h"
#include "flare_protocol.h"
#include <string.h>

uint8_t rc_rx_byte; /* extern visible to USART2_IRQHandler */
static uint8_t  rx_buf[FLARE_PACKET_SIZE];
static uint8_t  rx_count = 0;

static FLARE_RC_Packet_t rc_packet;

static volatile uint8_t  rc_new_packet   = 0;
static volatile uint32_t rc_last_rx_ms   = 0;
static volatile uint8_t  rc_ever_received = 0;

volatile uint32_t rc_bytes_received  = 0;
volatile uint8_t  rc_init_status     = 0xFF;
volatile uint32_t rc_rearm_failures  = 0;
volatile uint32_t rc_error_callbacks = 0;

/* ---------------------------------------------------------------------------
 * rearm_usart2_rx
 * Arms USART2 RX interrupt directly without going through HAL_UART_Receive_IT,
 * which hangs on STM32H7 due to a HAL FIFO/lock bug.
 * --------------------------------------------------------------------------- */
static void rearm_usart2_rx(void)
{
    SET_BIT(USART2->CR1, USART_CR1_RXNEIE_RXFNEIE);
}

void RC_Init(void)
{
    rx_count           = 0;
    rc_new_packet      = 0;
    rc_last_rx_ms      = 0;
    rc_ever_received   = 0;
    rc_bytes_received  = 0;
    rc_init_status     = 0xFF;
    rc_rearm_failures  = 0;
    rc_error_callbacks = 0;
    memset(&rc_packet, 0, sizeof(rc_packet));

    __HAL_UNLOCK(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                    UART_CLEAR_PEF  | UART_CLEAR_FEF);

    huart2.gState  = HAL_UART_STATE_READY;
    huart2.RxState = HAL_UART_STATE_READY;
    huart2.RxISR   = NULL;

    rearm_usart2_rx();
    rc_init_status = 0;
}

void RC_UART_RxCpltCallback(void)
{
    rc_bytes_received++;

    if (rx_count == 0 && rc_rx_byte != FLARE_PACKET_MAGIC) {
        rearm_usart2_rx();
        return;
    }

    rx_buf[rx_count++] = rc_rx_byte;

    if (rx_count == FLARE_PACKET_SIZE) {
        rx_count = 0;

        const FLARE_RC_Packet_t *pkt = (const FLARE_RC_Packet_t *)rx_buf;

        if (pkt->magic != FLARE_PACKET_MAGIC) {
            rearm_usart2_rx();
            return;
        }

        if (pkt->checksum != flare_checksum(pkt)) {
            rearm_usart2_rx();
            return;
        }

        memcpy(&rc_packet, pkt, sizeof(FLARE_RC_Packet_t));
        rc_new_packet    = 1;
        rc_last_rx_ms    = HAL_GetTick();
        rc_ever_received = 1;
    }

    rearm_usart2_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        rc_error_callbacks++;
        rx_count = 0;

        huart->gState  = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;

        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF  | UART_CLEAR_FEF);

        rearm_usart2_rx();
    }
}

uint8_t RC_GetPacket(FLARE_RC_Packet_t *out)
{
    if (!rc_new_packet) {
        return 0;
    }
    memcpy(out, &rc_packet, sizeof(FLARE_RC_Packet_t));
    rc_new_packet = 0;
    return 1;
}

uint8_t RC_IsHealthy(void)
{
    if (!rc_ever_received) {
        return 0;
    }
    return (HAL_GetTick() - rc_last_rx_ms) < RC_TIMEOUT_MS;
}