/* FATFS/Target/user_diskio.c
 * SPI disk I/O driver for FLARE SD card blackbox logger.
 * Implements the FatFS diskio interface over SPI1 (PA5/PA6/PA7, CS=PC0).
 */

/* USER CODE BEGIN Header */
/* USER CODE END Header */

#include "stm32h7xx_hal.h"
#include <stdio.h>
#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#define SD_DBG(s) HAL_UART_Transmit(&huart1, (const uint8_t *)(s), strlen(s), HAL_MAX_DELAY)

/* Private define ------------------------------------------------------------*/

/* SD SPI timing */
#define SD_TIMEOUT_MS       500u
#define SD_DUMMY_BYTE       0xFFu

/* SD command tokens */
#define SD_CMD0             0u
#define SD_CMD1             1u
#define SD_CMD8             8u
#define SD_CMD9             9u
#define SD_CMD16            16u
#define SD_CMD17            17u
#define SD_CMD24            24u
#define SD_CMD41            41u
#define SD_CMD55            55u
#define SD_CMD58            58u

/* R1 response bits */
#define SD_R1_IDLE          0x01u
#define SD_R1_ILLEGAL_CMD   0x04u
#define SD_R1_READY         0x00u

/* Data tokens */
#define SD_TOKEN_START      0xFEu
#define SD_TOKEN_ERROR      0x00u

/* Card type flags */
#define SD_CT_UNKNOWN       0x00u
#define SD_CT_SD1           0x01u
#define SD_CT_SD2           0x02u
#define SD_CT_SDHC          0x04u

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS sd_stat = STA_NOINIT;
static uint8_t          sd_card_type = SD_CT_UNKNOWN;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status    (BYTE pdrv);
DRESULT USER_read      (BYTE pdrv, BYTE *buff,       DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write     (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
DRESULT USER_ioctl     (BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver = {
    USER_initialize,
    USER_status,
    USER_read,
#if _USE_WRITE
    USER_write,
#endif
#if _USE_IOCTL == 1
    USER_ioctl,
#endif
};

/* ── SPI helpers ─────────────────────────────────────────────────────────── */

static inline void SD_CS_Low(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static inline void SD_CS_High(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static void SD_Deselect(void)
{
    SD_CS_High();
    /* Flush RX FIFO — prevents residual bytes corrupting next BMI323 read */
    __HAL_SPI_DISABLE(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);
    uint8_t dummy = SD_DUMMY_BYTE;
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);
}

static uint8_t SD_SPI_Byte(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static uint8_t SD_WaitReady(void)
{
    uint32_t t = HAL_GetTick();
    while (SD_SPI_Byte(SD_DUMMY_BYTE) != 0xFF) {
        if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) return 0;
    }
    return 1;
}



/* ── SD command layer ────────────────────────────────────────────────────── */

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    if (!SD_WaitReady()) return 0xFF;

    uint8_t buf[6] = {
        (uint8_t)(0x40u | cmd),
        (uint8_t)(arg >> 24),
        (uint8_t)(arg >> 16),
        (uint8_t)(arg >>  8),
        (uint8_t)(arg),
        0x01u,
    };
    if (cmd == SD_CMD0) buf[5] = 0x95u;
    if (cmd == SD_CMD8) buf[5] = 0x87u;

    HAL_SPI_Transmit(&hspi1, buf, 6, HAL_MAX_DELAY);

    uint8_t r1 = 0xFF;
    for (uint8_t i = 0; i < 10; i++) {
        r1 = SD_SPI_Byte(SD_DUMMY_BYTE);
        if (!(r1 & 0x80u)) break;
    }
    return r1;
}

static uint8_t SD_RxDataBlock(uint8_t *buf, UINT len)
{
    uint32_t t = HAL_GetTick();
    uint8_t token;
    do {
        token = SD_SPI_Byte(SD_DUMMY_BYTE);
        if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) return 0;
    } while (token == 0xFF);

    if (token != SD_TOKEN_START) return 0;

    memset(buf, SD_DUMMY_BYTE, len);
    HAL_SPI_TransmitReceive(&hspi1, buf, buf, len, HAL_MAX_DELAY);

    SD_SPI_Byte(SD_DUMMY_BYTE);
    SD_SPI_Byte(SD_DUMMY_BYTE);
    return 1;
}

static uint8_t SD_TxDataBlock(const uint8_t *buf, uint8_t token)
{
    if (!SD_WaitReady()) return 0;

    SD_SPI_Byte(token);

    if (token == SD_TOKEN_START) {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 512, HAL_MAX_DELAY);
        SD_SPI_Byte(SD_DUMMY_BYTE);
        SD_SPI_Byte(SD_DUMMY_BYTE);
        uint8_t resp = SD_SPI_Byte(SD_DUMMY_BYTE) & 0x1Fu;
        if (resp != 0x05u) return 0;
    }
    return 1;
}

/* ── FatFS interface ─────────────────────────────────────────────────────── */

DSTATUS USER_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;

    sd_stat      = STA_NOINIT;
    sd_card_type = SD_CT_UNKNOWN;

    char dbg[48];

    SD_DBG("[SD] init start\r\n");

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    SD_CS_High();
    HAL_Delay(100);

    for (uint8_t i = 0; i < 10; i++) SD_SPI_Byte(SD_DUMMY_BYTE);
    SD_DBG("[SD] dummy clocks done\r\n");

    uint8_t r1 = 0xFF;
    for (uint8_t retry = 0; retry < 10; retry++) {
        SD_CS_Low();
        r1 = SD_SendCmd(SD_CMD0, 0);
        SD_Deselect();
        if (r1 == SD_R1_IDLE) break;
        HAL_Delay(10);
    }
    snprintf(dbg, sizeof(dbg), "[SD] CMD0 r1=0x%02X\r\n", r1);
    SD_DBG(dbg);
    if (r1 != SD_R1_IDLE) return sd_stat;

    SD_DBG("[SD] CMD8 start\r\n");
    SD_CS_Low();
    r1 = SD_SendCmd(SD_CMD8, 0x000001AAu);
    snprintf(dbg, sizeof(dbg), "[SD] CMD8 r1=0x%02X\r\n", r1);
    SD_DBG(dbg);

    if (r1 == SD_R1_IDLE) {
        uint8_t r7[4];
        for (uint8_t i = 0; i < 4; i++) r7[i] = SD_SPI_Byte(SD_DUMMY_BYTE);
        SD_Deselect();
        snprintf(dbg, sizeof(dbg), "[SD] R7=%02X %02X %02X %02X\r\n",
                 r7[0], r7[1], r7[2], r7[3]);
        SD_DBG(dbg);

        if (r7[2] == 0x01u && r7[3] == 0xAAu) {
            SD_DBG("[SD] ACMD41 loop start\r\n");
            uint32_t t = HAL_GetTick();
            do {
                SD_CS_Low();
                SD_SendCmd(SD_CMD55, 0);
                SD_Deselect();
                SD_CS_Low();
                r1 = SD_SendCmd(SD_CMD41, 0x40000000u);
                SD_Deselect();
                if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) {
                    SD_DBG("[SD] ACMD41 timeout\r\n");
                    return sd_stat;
                }
            } while (r1 != SD_R1_READY);
            SD_DBG("[SD] ACMD41 done\r\n");

            SD_CS_Low();
            r1 = SD_SendCmd(SD_CMD58, 0);
            uint8_t ocr[4];
            for (uint8_t i = 0; i < 4; i++) ocr[i] = SD_SPI_Byte(SD_DUMMY_BYTE);
            SD_Deselect();
            snprintf(dbg, sizeof(dbg), "[SD] OCR=%02X %02X %02X %02X\r\n",
                     ocr[0], ocr[1], ocr[2], ocr[3]);
            SD_DBG(dbg);

            sd_card_type = (r1 == SD_R1_READY)
                ? ((ocr[0] & 0x40u) ? SD_CT_SDHC : SD_CT_SD2)
                : SD_CT_UNKNOWN;
        }
    } else {
        SD_Deselect();
        SD_DBG("[SD] SD v1 path\r\n");
        uint32_t t = HAL_GetTick();
        do {
            SD_CS_Low();
            SD_SendCmd(SD_CMD55, 0);
            SD_Deselect();
            SD_CS_Low();
            r1 = SD_SendCmd(SD_CMD41, 0);
            SD_Deselect();
            if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) {
                SD_DBG("[SD] v1 ACMD41 timeout\r\n");
                return sd_stat;
            }
        } while (r1 != SD_R1_READY);

        sd_card_type = (r1 == SD_R1_READY) ? SD_CT_SD1 : SD_CT_UNKNOWN;
        if (sd_card_type == SD_CT_SD1) {
            SD_CS_Low();
            SD_SendCmd(SD_CMD16, 512);
            SD_Deselect();
        }
    }

    snprintf(dbg, sizeof(dbg), "[SD] card_type=0x%02X\r\n", sd_card_type);
    SD_DBG(dbg);

    if (sd_card_type == SD_CT_UNKNOWN) return sd_stat;

    sd_stat = 0;
    SD_DBG("[SD] init OK\r\n");
    return sd_stat;
}

DSTATUS USER_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return sd_stat;
}

DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    char dbg[48];
    snprintf(dbg, sizeof(dbg), "[SD] READ sec=%lu cnt=%u\r\n",
             (unsigned long)sector, (unsigned)count);
    SD_DBG(dbg);

    if (pdrv != 0 || sd_stat & STA_NOINIT) return RES_NOTRDY;
    if (!count) return RES_PARERR;

    if (!(sd_card_type & SD_CT_SDHC)) sector <<= 9;

    if (count == 1) {
        SD_CS_Low();
        uint8_t r1 = SD_SendCmd(SD_CMD17, sector);
        snprintf(dbg, sizeof(dbg), "[SD] CMD17 r1=0x%02X\r\n", r1);
        SD_DBG(dbg);
        DRESULT res = RES_ERROR;
        if (r1 == SD_R1_READY && SD_RxDataBlock(buff, 512)) {
            res = RES_OK;
            SD_DBG("[SD] read OK\r\n");
        } else {
            SD_DBG("[SD] read FAIL\r\n");
        }
        SD_Deselect();
        return res;
    }

    return RES_ERROR;
}

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || sd_stat & STA_NOINIT) return RES_NOTRDY;
    if (!count) return RES_PARERR;

    if (!(sd_card_type & SD_CT_SDHC)) sector <<= 9;

    if (count == 1) {
        SD_CS_Low();
        uint8_t r1 = SD_SendCmd(SD_CMD24, sector);
        DRESULT res = RES_ERROR;
        if (r1 == SD_R1_READY && SD_TxDataBlock(buff, SD_TOKEN_START)) res = RES_OK;
        SD_Deselect();
        return res;
    }

    return RES_ERROR;
}
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0 || sd_stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC:
            SD_CS_Low();
            if (SD_WaitReady()) res = RES_OK;
            SD_Deselect();
            break;

        case GET_SECTOR_COUNT: {
            SD_CS_Low();
            uint8_t r1 = SD_SendCmd(SD_CMD9, 0);
            uint8_t csd[16] = {0};
            if (r1 == SD_R1_READY && SD_RxDataBlock(csd, 16)) {
                DWORD sectors = 0;
                if ((csd[0] >> 6) == 1) {
                    uint32_t c_size = ((uint32_t)(csd[7] & 0x3Fu) << 16)
                                    | ((uint32_t) csd[8]          <<  8)
                                    |             csd[9];
                    sectors = (c_size + 1) << 10;
                } else {
                    uint8_t  n      = (csd[5] & 0x0Fu)
                                    + ((csd[10] >> 7) | ((csd[9] & 0x03u) << 1))
                                    + 2u;
                    uint32_t c_size = ((uint32_t)(csd[6] & 0x03u) << 10)
                                    | ((uint32_t) csd[7]           <<  2)
                                    | (           csd[8]           >>  6);
                    sectors = (c_size + 1u) << (n - 9u);
                }
                *(DWORD *)buff = sectors;
                res = RES_OK;
            }
            SD_Deselect();
            break;
        }

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
            break;
    }
    return res;
}
#endif /* _USE_IOCTL == 1 */