/* Core/Src/sd.c
 * FLARE SD card blackbox logger.
 *
 * Strategy:
 *   - On init: mount, scan for next available LOGxxx.CSV filename, open it,
 *     write a CSV header row.
 *   - Each SD_Log_Write(): format one CSV row with f_printf() into the FatFS
 *     internal buffer. No explicit flush — fast.
 *   - Each SD_Log_Flush(): call f_sync() to commit buffered data to the card.
 *     At 1 Hz this gives ~1s data loss on power cut — acceptable for a logger.
 *   - SD_Log_Close(): f_sync() + f_close() + f_unmount().
 */

#include "usart.h"
#include "sd.h"
#include "fatfs.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

/* ── Private state ───────────────────────────────────────────────────────── */

static FATFS   sd_fs;
static FIL     sd_file;
static uint8_t sd_initialised = 0;
static char    sd_filename[16];        /* "LOGxxx.CSV\0"   */
static char    sd_status_str[48];      /* for StatusStr()  */

/* CSV header — must match column order in SD_Log_Write() */
static const char SD_CSV_HEADER[] =
    "time_ms,"
    "roll,pitch,yaw,"
    "acc_x,acc_y,acc_z,"
    "gyr_x,gyr_y,gyr_z,"
    "throttle,m1,m2,m3,m4,"
    "armed,rc_ok,"
    "gps_fix,gps_sats,lat,lon,alt_ft,spd_mph"
    "\n";

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/*
 * Scan LOGxxx.CSV (000–999) and return the first number with no existing file.
 * Returns -1 if all 1000 slots are taken (extremely unlikely).
 */
static int SD_NextLogNumber(void)
{
    char name[16];
    for (int n = 0; n < 1000; n++) {
        snprintf(name, sizeof(name), "LOG%03d.CSV", n);
        FILINFO fno;
        if (f_stat(name, &fno) != FR_OK) return n;  /* file not found = slot free */
    }
    return -1;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

SD_Log_Result_t SD_Log_Init(void)
{
    sd_initialised = 0;
    memset(sd_status_str, 0, sizeof(sd_status_str));

    HAL_UART_Transmit(&huart1, (const uint8_t *)"[SD] calling f_mount\r\n", 22, HAL_MAX_DELAY);

    /* Mount */
    FRESULT fr = f_mount(&sd_fs, USERPath, 1);
    if (fr != FR_OK) {
        snprintf(sd_status_str, sizeof(sd_status_str),
                 "[SD] mount FAIL fr=%d", (int)fr);
        return SD_LOG_MOUNT_ERR;
    }

    HAL_UART_Transmit(&huart1, (const uint8_t *)"[SD] f_mount returned\r\n", 23, HAL_MAX_DELAY);

    /* Find next log filename */
    int log_num = SD_NextLogNumber();
    if (log_num < 0) {
        snprintf(sd_status_str, sizeof(sd_status_str),
                 "[SD] no free log slots");
        return SD_LOG_OPEN_ERR;
    }
    snprintf(sd_filename, sizeof(sd_filename), "LOG%03d.CSV", log_num);

    /* Open file — create new, fail if exists (shouldn't, but be safe) */
    fr = f_open(&sd_file, sd_filename, FA_WRITE | FA_CREATE_NEW);
    if (fr != FR_OK) {
        snprintf(sd_status_str, sizeof(sd_status_str),
                 "[SD] open FAIL fr=%d file=%s", (int)fr, sd_filename);
        return SD_LOG_OPEN_ERR;
    }

    /* Write CSV header */
    UINT bw;
    fr = f_write(&sd_file, SD_CSV_HEADER, strlen(SD_CSV_HEADER), &bw);
    if (fr != FR_OK || bw != strlen(SD_CSV_HEADER)) {
        f_close(&sd_file);
        snprintf(sd_status_str, sizeof(sd_status_str),
                 "[SD] header write FAIL fr=%d", (int)fr);
        return SD_LOG_WRITE_ERR;
    }

    f_sync(&sd_file);

    sd_initialised = 1;
    snprintf(sd_status_str, sizeof(sd_status_str),
             "[SD] OK file=%s", sd_filename);
    return SD_LOG_OK;
}

SD_Log_Result_t SD_Log_Write(
    uint32_t timestamp_ms,
    float    roll,
    float    pitch,
    float    yaw,
    int16_t  acc_x,
    int16_t  acc_y,
    int16_t  acc_z,
    int16_t  gyr_x,
    int16_t  gyr_y,
    int16_t  gyr_z,
    uint16_t throttle,
    uint16_t motor1,
    uint16_t motor2,
    uint16_t motor3,
    uint16_t motor4,
    uint8_t  armed,
    uint8_t  rc_healthy,
    uint8_t  gps_fix,
    uint8_t  gps_sats,
    float    latitude,
    float    longitude,
    float    altitude_ft,
    float    speed_mph)
{
    if (!sd_initialised) return SD_LOG_NOT_INIT;

    /* Format one CSV row — f_printf is enabled via _USE_STRFUNC=2 in ffconf.h */
    int n = f_printf(&sd_file,
        "%lu,"
        "%.2f,%.2f,%.2f,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%u,%u,%u,%u,%u,"
        "%u,%u,"
        "%u,%u,%.6f,%.6f,%.1f,%.1f"
        "\n",
        (unsigned long)timestamp_ms,
        (double)roll, (double)pitch, (double)yaw,
        (int)acc_x, (int)acc_y, (int)acc_z,
        (int)gyr_x, (int)gyr_y, (int)gyr_z,
        (unsigned)throttle,
        (unsigned)motor1, (unsigned)motor2,
        (unsigned)motor3, (unsigned)motor4,
        (unsigned)armed, (unsigned)rc_healthy,
        (unsigned)gps_fix, (unsigned)gps_sats,
        (double)latitude, (double)longitude,
        (double)altitude_ft, (double)speed_mph);

    return (n > 0) ? SD_LOG_OK : SD_LOG_WRITE_ERR;
}

SD_Log_Result_t SD_Log_Flush(void)
{
    if (!sd_initialised) return SD_LOG_NOT_INIT;
    FRESULT fr = f_sync(&sd_file);
    return (fr == FR_OK) ? SD_LOG_OK : SD_LOG_WRITE_ERR;
}

void SD_Log_Close(void)
{
    if (!sd_initialised) return;
    f_sync(&sd_file);
    f_close(&sd_file);
    f_mount(NULL, USERPath, 0);
    sd_initialised = 0;
    snprintf(sd_status_str, sizeof(sd_status_str),
             "[SD] closed file=%s", sd_filename);
}

const char *SD_Log_StatusStr(void)
{
    return sd_status_str;
}