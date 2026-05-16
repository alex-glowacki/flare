#ifndef GPS_H
#define GPS_H

#include "usart.h"
#include <stdint.h>

/* ── DMA receive buffer size ─────────────────────────────────────────────
 * The NEO-6M at 9600 baud emits ~512 bytes/s at 5Hz update rate.
 * 256 bytes provides ~213ms of headroom — well above the 10ms loop tick.
 */
#define GPS_DMA_BUF_SIZE 256U
#define GPS_NMEA_MAX_LEN 96U

/* ── GPS fix data (SAE units) ────────────────────────────────────────── */
typedef struct {
    float latitude;       /* decimal degrees, positive = North          */
    float longitude;      /* decimal degrees, positive = East           */
    float altitude_ft;    /* feet above mean sea level                  */
    float speed_mph;      /* ground speed in miles per hour             */
    float course_deg;     /* track angle, degrees true                  */
    uint8_t fix_valid;    /* 1 = valid fix, 0 = no fix                  */
    uint8_t satellites;   /* number of satellites in use                */
    uint32_t last_fix_ms; /* HAL_GetTick() at time of last valid fix    */
} GPS_Data_t;

/* ── Public API ──────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the GPS driver and start USART3 DMA circular receive.
 * @param  huart  Pointer to the USART3 handle (must already be init'd).
 * @param  data   Pointer to the GPS_Data_t struct to populate.
 */
void GPS_Init(UART_HandleTypeDef *huart, GPS_Data_t *data);

/**
 * @brief  Call once per main loop iteration.
 *         Scans the DMA buffer for new complete NMEA sentences and parses
 *         any GPRMC or GPGGA sentences found since the last call.
 * @param  data   Same pointer passed to GPS_Init().
 */
void GPS_Update(GPS_Data_t *data);

#endif /* GPS_H */