#ifndef GPS_H
#define GPS_H

#include "usart.h"
#include <stdint.h>

/* ── DMA receive buffer ──────────────────────────────────────────────────
 * M100-5883 (u-blox M10) at 115200 baud, 10Hz UBX output.
 * UBX-NAV-PVT = 100 bytes/packet × 10Hz = ~1000 bytes/s.
 * 512 bytes provides ~512ms of headroom — well above the 10ms loop tick.
 * Buffer lives in non-cacheable AXI SRAM (see MPU_Config in main.c).
 */
#define GPS_DMA_BUF_SIZE 512U

/* Maximum length of a single UBX packet (header + payload + checksum).
 * UBX-NAV-PVT payload = 92 bytes → total = 6 + 92 + 2 = 100 bytes.
 * 128 bytes gives headroom for any other NAV messages.
 */
#define GPS_UBX_MAX_LEN 128U

/* ── UBX protocol constants ──────────────────────────────────────────────
 * UBX frame format:
 *   0xB5 0x62  — sync chars
 *   CLASS      — message class (1 byte)
 *   ID         — message ID    (1 byte)
 *   LEN        — payload length, little-endian (2 bytes)
 *   PAYLOAD    — LEN bytes
 *   CK_A CK_B  — Fletcher-8 checksum over CLASS+ID+LEN+PAYLOAD
 */
#define UBX_SYNC1 0xB5U
#define UBX_SYNC2 0x62U
#define UBX_CLASS_NAV 0x01U
#define UBX_ID_NAV_PVT 0x07U /* Position, Velocity, Time — all-in-one */

/* ── GPS fix data ────────────────────────────────────────────────────────
 * UBX-NAV-PVT provides all fields below in a single 92-byte payload.
 * Units match the existing main.c telemetry print format.
 */
typedef struct {
    float latitude;       /* decimal degrees, positive = North          */
    float longitude;      /* decimal degrees, positive = East           */
    float altitude_ft;    /* feet above mean sea level (from MSL field) */
    float speed_mph;      /* ground speed in miles per hour             */
    float course_deg;     /* heading of motion, degrees true            */
    float speed_ms;       /* ground speed m/s (for flight loop use)     */
    uint8_t fix_valid;    /* 1 = valid 3D fix, 0 = no fix               */
    uint8_t fix_type;     /* UBX fixType: 0=none 2=2D 3=3D 4=GNSS      */
    uint8_t satellites;   /* number of satellites used in solution      */
    uint32_t last_fix_ms; /* HAL_GetTick() at time of last valid fix    */
} GPS_Data_t;

/* ── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the GPS driver and start USART3 DMA circular receive.
 * @param  huart  Pointer to the USART3 handle (must already be init'd at 115200).
 * @param  data   Pointer to the GPS_Data_t struct to populate.
 */
void GPS_Init(UART_HandleTypeDef *huart, GPS_Data_t *data);

/**
 * @brief  Call once per main loop iteration.
 *         Scans the DMA buffer for complete UBX-NAV-PVT packets and updates
 *         data on each valid, checksummed packet found since the last call.
 * @param  data   Same pointer passed to GPS_Init().
 */
void GPS_Update(GPS_Data_t *data);

#endif /* GPS_H */