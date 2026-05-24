/* firmware/fc/Core/Src/gps.c
 *
 * UBX binary parser for the HGLRC M100-5883 (u-blox M10).
 * Parses UBX-NAV-PVT (class 0x01, ID 0x07) — the single all-in-one
 * position/velocity/time message. No NMEA parsing needed.
 *
 * DMA runs in circular mode on USART3 at 115200 baud.
 * GPS_Update() is called each 10ms loop tick to drain the ring buffer.
 */
#include "gps.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ── Unit conversion constants ───────────────────────────────────────── */
#define MS_TO_MPH       2.23694f    /* 1 m/s = 2.23694 mph               */
#define MM_TO_FEET      0.00328084f /* 1 mm  = 0.00328084 ft             */

/* ── UBX-NAV-PVT payload layout (92 bytes, all little-endian) ───────────
 * Only the fields we use are listed. Full spec: u-blox M10 interface manual.
 *
 *  Offset  Size  Field
 *  ------  ----  -----
 *    0      4    iTOW        ms GPS time of week (unused)
 *    4      2    year        (unused)
 *    ...
 *   20      1    fixType     0=none 1=DR 2=2D 3=3D 4=GNSS+DR 5=time only
 *   21      1    flags       bit0 = gnssFixOK (valid fix)
 *   22      1    flags2      (unused)
 *   23      1    numSV       number of satellites used
 *   24      4    lon         deg × 1e-7, signed
 *   28      4    lat         deg × 1e-7, signed
 *   32      4    height      mm above ellipsoid (unused)
 *   36      4    hMSL        mm above mean sea level → altitude_ft
 *   40      4    hAcc        mm horizontal accuracy (unused)
 *   44      4    vAcc        mm vertical accuracy (unused)
 *   48      4    velN        mm/s NED north velocity (unused)
 *   52      4    velE        mm/s NED east velocity (unused)
 *   56      4    velD        mm/s NED down velocity (unused)
 *   60      4    gSpeed      mm/s ground speed → speed_ms, speed_mph
 *   64      4    headMot     deg × 1e-5 heading of motion → course_deg
 *   68      4    sAcc        mm/s speed accuracy (unused)
 *   72      4    headAcc     deg × 1e-5 heading accuracy (unused)
 *   76      2    pDOP        (unused)
 *   78      6    reserved
 *   84      4    headVeh     deg × 1e-5 heading of vehicle (unused)
 *   88      4    magDec      deg × 1e-2 magnetic declination (unused)
 */
#define PVT_OFFSET_FIX_TYPE  20U
#define PVT_OFFSET_FLAGS     21U
#define PVT_OFFSET_NUM_SV    23U
#define PVT_OFFSET_LON       24U
#define PVT_OFFSET_LAT       28U
#define PVT_OFFSET_HMSL      36U
#define PVT_OFFSET_GSPEED    60U
#define PVT_OFFSET_HEAD_MOT  64U

#define UBX_NAV_PVT_PAYLOAD_LEN  92U

/* ── Module-private state ────────────────────────────────────────────── */

/* DMA circular receive buffer — non-cacheable AXI SRAM region
 * (0x24000000) configured by MPU_Config() in main.c.                    */
static uint8_t gps_dma_buf[GPS_DMA_BUF_SIZE] __attribute__((section(".AXI_SRAM")));

static uint16_t           gps_dma_tail = 0;
static UART_HandleTypeDef *gps_huart   = NULL;

/* ── UBX parser state machine ────────────────────────────────────────── */
typedef enum {
    UBX_STATE_SYNC1,    /* waiting for 0xB5                               */
    UBX_STATE_SYNC2,    /* waiting for 0x62                               */
    UBX_STATE_CLASS,    /* message class byte                             */
    UBX_STATE_ID,       /* message ID byte                                */
    UBX_STATE_LEN1,     /* payload length LSB                             */
    UBX_STATE_LEN2,     /* payload length MSB                             */
    UBX_STATE_PAYLOAD,  /* collecting payload bytes                       */
    UBX_STATE_CK_A,     /* first checksum byte                            */
    UBX_STATE_CK_B,     /* second checksum byte → dispatch on match       */
} UBX_State_t;

static UBX_State_t ubx_state    = UBX_STATE_SYNC1;
static uint8_t     ubx_class    = 0;
static uint8_t     ubx_id       = 0;
static uint16_t    ubx_len      = 0;
static uint16_t    ubx_pay_idx  = 0;
static uint8_t     ubx_ck_a     = 0;
static uint8_t     ubx_ck_b     = 0;
static uint8_t     ubx_payload[GPS_UBX_MAX_LEN];

/* ── Internal helpers ────────────────────────────────────────────────── */

static int32_t read_i32_le(const uint8_t *buf, uint16_t offset) {
    return (int32_t)(
        (uint32_t)buf[offset]               |
        ((uint32_t)buf[offset + 1U] << 8U)  |
        ((uint32_t)buf[offset + 2U] << 16U) |
        ((uint32_t)buf[offset + 3U] << 24U)
    );
}

static void parse_nav_pvt(const uint8_t *payload, GPS_Data_t *data) {
    uint8_t fix_type = payload[PVT_OFFSET_FIX_TYPE];
    uint8_t flags    = payload[PVT_OFFSET_FLAGS];

    /* gnssFixOK is bit 0 of flags. Require at least a 3D fix. */
    if ((flags & 0x01U) == 0U || fix_type < 3U) {
        data->fix_valid = 0;
        data->fix_type  = fix_type;
        return;
    }

    data->fix_type   = fix_type;
    data->satellites = payload[PVT_OFFSET_NUM_SV];

    /* Latitude / longitude: degrees × 1e-7 → decimal degrees */
    data->latitude  = (float)read_i32_le(payload, PVT_OFFSET_LAT) * 1e-7f;
    data->longitude = (float)read_i32_le(payload, PVT_OFFSET_LON) * 1e-7f;

    /* Altitude MSL: mm → feet */
    data->altitude_ft = (float)read_i32_le(payload, PVT_OFFSET_HMSL)
                        * MM_TO_FEET;

    /* Ground speed: mm/s → m/s and mph */
    int32_t gspeed_mms = read_i32_le(payload, PVT_OFFSET_GSPEED);
    data->speed_ms  = (float)gspeed_mms * 0.001f;
    data->speed_mph = data->speed_ms * MS_TO_MPH;

    /* Heading of motion: deg × 1e-5 → degrees */
    data->course_deg = (float)read_i32_le(payload, PVT_OFFSET_HEAD_MOT)
                       * 1e-5f;

    data->fix_valid   = 1;
    data->last_fix_ms = HAL_GetTick();
}

static void ubx_feed_byte(uint8_t c, GPS_Data_t *data) {
    switch (ubx_state) {

        case UBX_STATE_SYNC1:
            if (c == UBX_SYNC1) ubx_state = UBX_STATE_SYNC2;
            break;

        case UBX_STATE_SYNC2:
            ubx_state = (c == UBX_SYNC2) ? UBX_STATE_CLASS : UBX_STATE_SYNC1;
            break;

        case UBX_STATE_CLASS:
            ubx_class = c;
            ubx_ck_a  = c;
            ubx_ck_b  = c;
            ubx_state = UBX_STATE_ID;
            break;

        case UBX_STATE_ID:
            ubx_id    = c;
            ubx_ck_a += c; ubx_ck_b += ubx_ck_a;
            ubx_state = UBX_STATE_LEN1;
            break;

        case UBX_STATE_LEN1:
            ubx_len   = c;
            ubx_ck_a += c; ubx_ck_b += ubx_ck_a;
            ubx_state = UBX_STATE_LEN2;
            break;

        case UBX_STATE_LEN2:
            ubx_len  |= ((uint16_t)c << 8U);
            ubx_ck_a += c; ubx_ck_b += ubx_ck_a;
            ubx_pay_idx = 0;
            if (ubx_len == 0U || ubx_len > GPS_UBX_MAX_LEN) {
                ubx_state = UBX_STATE_SYNC1;
            } else {
                ubx_state = UBX_STATE_PAYLOAD;
            }
            break;

        case UBX_STATE_PAYLOAD:
            if (ubx_pay_idx < GPS_UBX_MAX_LEN) {
                ubx_payload[ubx_pay_idx] = c;
            }
            ubx_pay_idx++;
            ubx_ck_a += c; ubx_ck_b += ubx_ck_a;
            if (ubx_pay_idx >= ubx_len) {
                ubx_state = UBX_STATE_CK_A;
            }
            break;

        case UBX_STATE_CK_A:
            ubx_state = (c == ubx_ck_a) ? UBX_STATE_CK_B : UBX_STATE_SYNC1;
            break;

        case UBX_STATE_CK_B:
            if (c == ubx_ck_b) {
                if (ubx_class == UBX_CLASS_NAV &&
                    ubx_id    == UBX_ID_NAV_PVT &&
                    ubx_len   == UBX_NAV_PVT_PAYLOAD_LEN) {
                    parse_nav_pvt(ubx_payload, data);
                }
            }
            ubx_state = UBX_STATE_SYNC1;
            break;

        default:
            ubx_state = UBX_STATE_SYNC1;
            break;
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

void GPS_Init(UART_HandleTypeDef *huart, GPS_Data_t *data) {
    gps_huart    = huart;
    gps_dma_tail = 0;
    ubx_state    = UBX_STATE_SYNC1;

    memset(gps_dma_buf, 0, sizeof(gps_dma_buf));
    memset(data,        0, sizeof(GPS_Data_t));

    HAL_UART_Receive_DMA(huart, gps_dma_buf, GPS_DMA_BUF_SIZE);
}

void GPS_Update(GPS_Data_t *data) {
    if (gps_huart == NULL) return;

    uint16_t head = GPS_DMA_BUF_SIZE -
                    (uint16_t)__HAL_DMA_GET_COUNTER(gps_huart->hdmarx);

    while (gps_dma_tail != head) {
        ubx_feed_byte(gps_dma_buf[gps_dma_tail], data);
        gps_dma_tail = (gps_dma_tail + 1U) % GPS_DMA_BUF_SIZE;
    }
}