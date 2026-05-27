/* firmware/fc/Core/Inc/sd.h */
#pragma once

#include <stdint.h>

typedef enum {
    SD_LOG_OK = 0,
    SD_LOG_NOT_INIT,
    SD_LOG_MOUNT_ERR,
    SD_LOG_OPEN_ERR,
    SD_LOG_WRITE_ERR,
} SD_Log_Result_t;

SD_Log_Result_t SD_Log_Init(void);

SD_Log_Result_t SD_Log_Write(uint32_t timestamp_ms, float roll, float pitch, float yaw,
                             int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x,
                             int16_t gyr_y, int16_t gyr_z, uint16_t throttle, uint16_t motor1,
                             uint16_t motor2, uint16_t motor3, uint16_t motor4, uint8_t armed,
                             uint8_t rc_healthy, uint8_t gps_fix, uint8_t gps_sats, float latitude,
                             float longitude, float altitude_ft, float speed_mph);

SD_Log_Result_t SD_Log_Flush(void);
void SD_Log_Close(void);
const char *SD_Log_StatusStr(void);