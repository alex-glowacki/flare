/* Core/Inc/sd.h
 * FLARE SD card blackbox logger — public interface.
 */

#ifndef SD_H
#define SD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Return codes */
typedef enum {
    SD_LOG_OK = 0,
    SD_LOG_MOUNT_ERR,
    SD_LOG_OPEN_ERR,
    SD_LOG_WRITE_ERR,
    SD_LOG_NOT_INIT,
} SD_Log_Result_t;

/* Initialise FatFS, mount the card, open a new log file.
 * Call once after MX_FATFS_Init() in main.c USER CODE BEGIN 2.
 * Returns SD_LOG_OK on success. */
SD_Log_Result_t SD_Log_Init(void);

/* Write one CSV row to the log buffer.
 * Call at your desired log rate (e.g. every 10 loop iterations = 10 Hz).
 * Does NOT flush to card — call SD_Log_Flush() periodically.
 * Returns SD_LOG_OK on success. */
SD_Log_Result_t SD_Log_Write(uint32_t timestamp_ms, float roll, float pitch, float yaw,
                             int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x,
                             int16_t gyr_y, int16_t gyr_z, uint16_t throttle, uint16_t motor1,
                             uint16_t motor2, uint16_t motor3, uint16_t motor4, uint8_t armed,
                             uint8_t rc_healthy, uint8_t gps_fix, uint8_t gps_stats, float latitude,
                             float longitude, float altitude_ft, float speed_mph);

/* Flush the FatFS write buffer to the card.
 * Call once per second (every 100 loop iterations at 100 Hz).
 * Returns SD_LOG_OK on success. */
SD_Log_Result_t SD_Log_Flush(void);

/* Flush and close the log file. Call on disarm or shutdown. */
void SD_Log_Close(void);

/* Human-readable status string for serial debug - e.g. "[SD] OK file=LOG003.CSV" */
const char *SD_Log_StatusStr(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_H */