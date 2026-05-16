#include "gps.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"

/* ── Unit conversion constants ───────────────────────────────────────── */
#define KNOTS_TO_MPH    1.15078f    /* 1 knot = 1.15078 mph */
#define METERS_TO_FEET  3.28084f    /* 1 meter = 3.28084 feet */

/* ── Module-private state ────────────────────────────────────────────── */

/* DMA circular receive buffer — must live in the non-cacheable AXI SRAM
 * region (0x24000000) configured by MPU_Config() in main.c.
 * CPU reads are immediately coherent with DMA writes — no cache flush needed. */
static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE] __attribute__((section(".AXI_SRAM")));

static uint16_t gps_dma_tail = 0;
static UART_HandleTypeDef *gps_huart = NULL;

/* ── Internal helpers ────────────────────────────────────────────────── */

/**
 * @brief  Convert NMEA lat/lon field + hemisphere to signed decimal degrees.
 *         NMEA format: DDDMM.MMMMM  (degrees concatenated with decimal minutes)
 *         e.g. "4807.038" + 'N' → +48.1173°
 */
static float nmea_to_decimal(const char *field, char hemi) {
    if (field == NULL || field[0] == '\0') return 0.0f;

    float raw = strtof(field, NULL);
    int degrees = (int)(raw / 100.0f);
    float minutes = raw - (float)(degrees * 100);
    float decimal = (float)degrees + (minutes / 60.0f);

    if (hemi == 'S' || hemi == 'W') decimal = -decimal;
    return decimal;
 }

/**
 * @brief  Extract the Nth comma-delimited field from an NMEA sentence.
 *         Field index 0 = sentence ID (e.g. "$GPRMC").
 */
static void nmea_get_field(const char *sentence, uint8_t field_idx, char *out_buf, uint8_t out_len) {
    out_buf[0] = '\0';
    uint8_t current = 0;
    const char *p = sentence;

    while (*p) {
        if (*p == ',') { current++; p++; continue; }
        if (current == field_idx) {
            uint8_t i = 0;
            while (*p && *p != ',' && *p != '*' && i < (out_len - 1u)) {
                out_buf[i++] = *p++;
            }
            out_buf[i] = '\0';
            return;
        }
        p++;
    }
 }

/**
 * @brief  Validate NMEA XOR checksum.
 *         Format: $<data>*<XX>   Checksum = XOR of bytes between $ and *.
 * @return 1 if valid, 0 otherwise.
 */
static uint8_t nmea_checksum_valid(const char *sentence) {
    const char *p = sentence;
    if (*p == '$') p++;

    uint8_t calc = 0;
    while (*p && *p != '*') calc ^= (uint8_t)(*p++);
    if (*p != '*') return 0;

    p++;
    uint8_t recv = (uint8_t)strtol(p, NULL, 16);
    return (calc == recv) ? 1u : 0u;
}

/**
 * @brief  Parse $GPRMC / $GNRMC sentence.
 *
 *  Index  Content
 *  -----  -------
 *    0    $GPRMC
 *    1    HHMMSS.ss   UTC time (unused)
 *    2    A/V         A = valid, V = void
 *    3    DDMM.MMMM   latitude
 *    4    N/S
 *    5    DDDMM.MMMM  longitude
 *    6    E/W
 *    7    SSS.SS      speed over ground, knots  → converted to mph
 *    8    DDD.DD      track angle, degrees true
 *    9    DDMMYY      date (unused)
 */
static void parse_gprmc(const char *sentence, GPS_Data_t *data) {
    char field[16];

    nmea_get_field(sentence, 2, field, sizeof(field));
    if (field[0] != 'A') { data->fix_valid = 0; return; }

    char lat_str[16], lat_hemi[4];
    nmea_get_field(sentence, 3, lat_str, sizeof(lat_str));
    nmea_get_field(sentence, 4, lat_hemi, sizeof(lat_hemi));
    data->latitude = nmea_to_decimal(lat_str, lat_hemi[0]);

    char lon_str[16], lon_hemi[4];
    nmea_get_field(sentence, 5, lon_str, sizeof(lon_str));
    nmea_get_field(sentence, 6, lon_hemi, sizeof(lon_hemi));
    data->longitude = nmea_to_decimal(lon_str, lon_hemi[0]);

    /* Speed: knots → mph */
    nmea_get_field(sentence, 7, field, sizeof(field));
    data->speed_mph = strtof(field, NULL) * KNOTS_TO_MPH;

    /* Course */
    nmea_get_field(sentence, 8, field, sizeof(field));
    data->course_deg = strtof(field, NULL);

    data->fix_valid = 1;
    data->last_fix_ms = HAL_GetTick();
}

/**
 * @brief  Parse $GPGGA / $GNGGA sentence.
 *
 *  Index  Content
 *  -----  -------
 *    0    $GPGGA
 *    1    HHMMSS.ss   UTC time (unused)
 *    2    latitude
 *    3    N/S
 *    4    longitude
 *    5    E/W
 *    6    fix quality (0 = invalid)
 *    7    satellites in use
 *    8    HDOP (unused)
 *    9    altitude, metres above MSL  → converted to feet
 */
static void parse_gpgga(const char *sentence, GPS_Data_t *data) {
    char field[16];

    nmea_get_field(sentence, 6, field, sizeof(field));
    if (field[0] == '0' || field[0] == '\0') return;    /* no fix */

    nmea_get_field(sentence, 7, field, sizeof(field));
    data->satellites = (uint8_t)strtol(field, NULL, 10);

    /* Altitude: meters → feet */
    nmea_get_field(sentence, 9, field, sizeof(field));
    data->altitude_ft = strtof(field, NULL) * METERS_TO_FEET;
}

/**
 * @brief  Dispatch a complete, checksum-validated NMEA sentence.
 *         Only GPRMC/GNRMC and GPGGA/GNGGA are handled; all others dropped.
 */
static void nmea_dispatch(const char *sentence, GPS_Data_t *data) {
    if (!nmea_checksum_valid(sentence)) return;

    if (strncmp(sentence, "$GPRMC", 6) == 0 ||
        strncmp(sentence, "$GNRMC", 6) == 0) { parse_gprmc(sentence, data); }
    else if (strncmp(sentence, "$GPGGA", 6) == 0 ||
             strncmp(sentence, "$GNGGA", 6) == 0) { parse_gpgga(sentence, data); }
}

/* ── Public API ──────────────────────────────────────────────────────── */

void GPS_Init(UART_HandleTypeDef *huart, GPS_Data_t *data) {
    gps_huart = huart;
    gps_dma_tail = 0;

    memset(gps_dma_buf, 0, sizeof(gps_dma_buf));
    memset(data, 0, sizeof(GPS_Data_t));

    /*
     * Start circular DMA receive. The DMA runs continuously and wraps
     * automatically — GPS_Update() tracks the write head via NDTR.
     */
    HAL_UART_Receive_DMA(huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
}

void GPS_Update(GPS_Data_t *data) {
    if (gps_huart == NULL) return;

    /*
     * head = next byte DMA will write.
     * NDTR counts down from GPS_DMA_BUF_SIZE to 0, so:
     *   head = GPS_DMA_BUF_SIZE - NDTR
     */
    uint16_t head = GPS_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(gps_huart->hdmarx);

    if (head == gps_dma_tail) return;   /* no new bytes */

    static char line[GPS_NMEA_MAX_LEN];
    static uint8_t line_pos = 0;

    while (gps_dma_tail != head) {
        uint8_t c = gps_dma_buf[gps_dma_tail];
        gps_dma_tail = (gps_dma_tail + 1u) % GPS_DMA_BUF_SIZE;

        if (c == '$') {
            line_pos = 0;
            line[line_pos++] = (char)c;
        } else if (c == '\n') {
            if (line_pos > 0 && line_pos < GPS_NMEA_MAX_LEN) {
                line[line_pos] = '\0';
                nmea_dispatch(line, data);
            }
            line_pos = 0;
        } else if (c == '\r') {
            /* skip - NMEA line endings are \r\n */
        } else {
            if (line_pos < (GPS_NMEA_MAX_LEN - 1u)) {
                line[line_pos++] = (char)c;
            } else {
                line_pos = 0;   /* overflow - discard sentence */
            }
        }
    }
}