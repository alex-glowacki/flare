/* FATFS/Target/user_diskio.c
 * SPI disk I/O driver for FLARE SD card blackbox logger.
 * Implements the FatFS diskio interface over SPI1 (PA5/PA6/PA7, CS=PC0).
 */

/* USER CODE BEGIN Header */
/* USER CODE END Header */

#include "stm32h7xx_hal.h"
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

/* Private define ------------------------------------------------------------*/

/* SD SPI timing */
#define SD_TIMEOUT_MS       500u
#define SD_DUMMY_BYTE       0xFFu

/* SD command tokens */
#define SD_CMD0             0u   /* GO_IDLE_STATE        */
#define SD_CMD1             1u   /* SEND_OP_COND (MMC)   */
#define SD_CMD8             8u   /* SEND_IF_COND         */
#define SD_CMD9             9u   /* SEND_CSD             */
#define SD_CMD16            16u  /* SET_BLOCKLEN         */
#define SD_CMD17            17u  /* READ_SINGLE_BLOCK    */
#define SD_CMD24            24u  /* WRITE_BLOCK          */
#define SD_CMD41            41u  /* APP_SEND_OP_COND     */
#define SD_CMD55            55u  /* APP_CMD              */
#define SD_CMD58            58u  /* READ_OCR             */

/* R1 response bits */
#define SD_R1_IDLE          0x01u
#define SD_R1_ILLEGAL_CMD   0x04u
#define SD_R1_READY         0x00u

/* Data tokens */
#define SD_TOKEN_START      0xFEu
#define SD_TOKEN_ERROR      0x00u

/* Card type flags */
#define SD_CT_UNKNOWN       0x00u
#define SD_CT_SD1           0x01u  /* SD v1                */
#define SD_CT_SD2           0x02u  /* SD v2                */
#define SD_CT_SDHC          0x04u  /* SDHC / SDXC          */

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS sd_stat    = STA_NOINIT;
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

/*
 * Deselect card, flush the SPI RX FIFO, then clock out one dummy byte.
 *
 * The FIFO flush is required because HAL_SPI_Transmit (used for the 512-byte
 * write payload) does not drain the RX FIFO — residual bytes left there will
 * corrupt the first BMI323 read that follows on the shared SPI1 bus, causing
 * the single-sample zero dropout seen without this flush.
 */
static void SD_Deselect(void)
{
    SD_CS_High();

    /* Flush RX FIFO by toggling SPE — safe on STM32H7 when no transfer
     * is in progress (CS is already high, card is deselected). */
    __HAL_SPI_DISABLE(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);

    /* One trailing dummy byte to release MISO */
    uint8_t dummy = SD_DUMMY_BYTE;
    HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);
}

/* Send one byte, return received byte. */
static uint8_t SD_SPI_Byte(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* Wait for card to release MISO (0xFF = ready). Returns 1 on success. */
static uint8_t SD_WaitReady(void)
{
    uint32_t t = HAL_GetTick();
    while (SD_SPI_Byte(SD_DUMMY_BYTE) != 0xFF) {
        if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) return 0;
    }
    return 1;
}

/* ── SD command layer ────────────────────────────────────────────────────── */

/*
 * Send SD command and return R1 response byte.
 * Card must be selected (CS low) before calling.
 */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    if (!SD_WaitReady()) return 0xFF;

    uint8_t buf[6] = {
        (uint8_t)(0x40u | cmd),
        (uint8_t)(arg >> 24),
        (uint8_t)(arg >> 16),
        (uint8_t)(arg >>  8),
        (uint8_t)(arg),
        0x01u,   /* CRC stub — valid for CMD0 and CMD8, ignored otherwise */
    };
    if (cmd == SD_CMD0) buf[5] = 0x95u;
    if (cmd == SD_CMD8) buf[5] = 0x87u;

    HAL_SPI_Transmit(&hspi1, buf, 6, HAL_MAX_DELAY);

    /* R1 response: poll up to 10 bytes */
    uint8_t r1 = 0xFF;
    for (uint8_t i = 0; i < 10; i++) {
        r1 = SD_SPI_Byte(SD_DUMMY_BYTE);
        if (!(r1 & 0x80u)) break;
    }
    return r1;
}

/* ── Receive a data block from the card ─────────────────────────────────── */

static uint8_t SD_RxDataBlock(uint8_t *buf, UINT len)
{
    /* Wait for data token 0xFE */
    uint32_t t = HAL_GetTick();
    uint8_t token;
    do {
        token = SD_SPI_Byte(SD_DUMMY_BYTE);
        if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) return 0;
    } while (token == 0xFF);

    if (token != SD_TOKEN_START) return 0;

    /* Receive payload */
    memset(buf, SD_DUMMY_BYTE, len);
    HAL_SPI_TransmitReceive(&hspi1, buf, buf, len, HAL_MAX_DELAY);

    /* Discard two CRC bytes */
    SD_SPI_Byte(SD_DUMMY_BYTE);
    SD_SPI_Byte(SD_DUMMY_BYTE);
    return 1;
}

/* ── Transmit a data block to the card ──────────────────────────────────── */

static uint8_t SD_TxDataBlock(const uint8_t *buf, uint8_t token)
{
    if (!SD_WaitReady()) return 0;

    SD_SPI_Byte(token);

    if (token == SD_TOKEN_START) {
        /* Write payload — cast away const; HAL Tx does not modify buffer */
        HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 512, HAL_MAX_DELAY);

        /* Dummy CRC */
        SD_SPI_Byte(SD_DUMMY_BYTE);
        SD_SPI_Byte(SD_DUMMY_BYTE);

        /* Data response — lower 5 bits: 0x05 = accepted */
        uint8_t resp = SD_SPI_Byte(SD_DUMMY_BYTE) & 0x1Fu;
        if (resp != 0x05u) return 0;
    }
    return 1;
}

/* ── FatFS interface ─────────────────────────────────────────────────────── */

/**
  * @brief  Initialises the SD card over SPI.
  */
DSTATUS USER_initialize(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;

    sd_stat      = STA_NOINIT;
    sd_card_type = SD_CT_UNKNOWN;

    /* Keep BMI323 deselected for the entire SD init sequence */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    SD_CS_High();
    HAL_Delay(100);

    /* Send ≥74 dummy clocks with CS high to wake card */
    for (uint8_t i = 0; i < 10; i++) SD_SPI_Byte(SD_DUMMY_BYTE);

    /* CMD0 — reset to SPI mode, retry up to 10 times */
    uint8_t r1 = 0xFF;
    for (uint8_t retry = 0; retry < 10; retry++) {
        SD_CS_Low();
        r1 = SD_SendCmd(SD_CMD0, 0);
        SD_Deselect();
        if (r1 == SD_R1_IDLE) break;
        HAL_Delay(10);
    }
    if (r1 != SD_R1_IDLE) return sd_stat;

    /* CMD8 — probe for SD v2 (arg: VHS=1 / check pattern 0xAA) */
    SD_CS_Low();
    r1 = SD_SendCmd(SD_CMD8, 0x000001AAu);

    if (r1 == SD_R1_IDLE) {
        /* SD v2: read 32-bit R7 response */
        uint8_t r7[4];
        for (uint8_t i = 0; i < 4; i++) r7[i] = SD_SPI_Byte(SD_DUMMY_BYTE);
        SD_Deselect();

        if (r7[2] == 0x01u && r7[3] == 0xAAu) {
            /* Valid voltage/echo — init with ACMD41(HCS=1) */
            uint32_t t = HAL_GetTick();
            do {
                SD_CS_Low();
                SD_SendCmd(SD_CMD55, 0);
                SD_Deselect();
                SD_CS_Low();
                r1 = SD_SendCmd(SD_CMD41, 0x40000000u);
                SD_Deselect();
                if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) return sd_stat;
            } while (r1 != SD_R1_READY);

            /* CMD58 — read OCR to check CCS bit (SDHC vs SDSC) */
            SD_CS_Low();
            r1 = SD_SendCmd(SD_CMD58, 0);
            uint8_t ocr[4];
            for (uint8_t i = 0; i < 4; i++) ocr[i] = SD_SPI_Byte(SD_DUMMY_BYTE);
            SD_Deselect();

            sd_card_type = (r1 == SD_R1_READY)
                ? ((ocr[0] & 0x40u) ? SD_CT_SDHC : SD_CT_SD2)
                : SD_CT_UNKNOWN;
        }
    } else {
        /* SD v1 or MMC — try ACMD41 then CMD1 */
        SD_Deselect();
        uint32_t t = HAL_GetTick();
        do {
            SD_CS_Low();
            SD_SendCmd(SD_CMD55, 0);
            SD_Deselect();
            SD_CS_Low();
            r1 = SD_SendCmd(SD_CMD41, 0);
            SD_Deselect();
            if ((HAL_GetTick() - t) > SD_TIMEOUT_MS) return sd_stat;
        } while (r1 != SD_R1_READY);

        sd_card_type = (r1 == SD_R1_READY) ? SD_CT_SD1 : SD_CT_UNKNOWN;

        /* Set block length to 512 for SD v1 */
        if (sd_card_type == SD_CT_SD1) {
            SD_CS_Low();
            SD_SendCmd(SD_CMD16, 512);
            SD_Deselect();
        }
    }

    if (sd_card_type == SD_CT_UNKNOWN) return sd_stat;

    sd_stat = 0;   /* Clear STA_NOINIT — card ready */
    return sd_stat;
}

/**
  * @brief  Returns disk status.
  */
DSTATUS USER_status(BYTE pdrv)
{
    if (pdrv != 0) return STA_NOINIT;
    return sd_stat;
}

/**
  * @brief  Reads one 512-byte sector.
  */
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || sd_stat & STA_NOINIT) return RES_NOTRDY;
    if (!count) return RES_PARERR;

    /* SDHC/SDXC: sector address. SDSC: byte address. */
    if (!(sd_card_type & SD_CT_SDHC)) sector <<= 9;

    if (count == 1) {
        SD_CS_Low();
        uint8_t r1 = SD_SendCmd(SD_CMD17, sector);
        DRESULT res = RES_ERROR;
        if (r1 == SD_R1_READY && SD_RxDataBlock(buff, 512)) res = RES_OK;
        SD_Deselect();
        return res;
    }

    /* Multi-block read not needed for logger — single block only */
    return RES_ERROR;
}

/**
  * @brief  Writes one 512-byte sector.
  */
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

/**
  * @brief  I/O control — FatFS calls this for flush and geometry queries.
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0 || sd_stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC:
            /* Ensure card has finished writing */
            SD_CS_Low();
            if (SD_WaitReady()) res = RES_OK;
            SD_Deselect();
            break;

        case GET_SECTOR_COUNT: {
            /* Read CSD register to get card capacity */
            SD_CS_Low();
            uint8_t r1 = SD_SendCmd(SD_CMD9, 0);
            uint8_t csd[16] = {0};
            if (r1 == SD_R1_READY && SD_RxDataBlock(csd, 16)) {
                DWORD sectors = 0;
                if ((csd[0] >> 6) == 1) {
                    /* CSD v2 (SDHC/SDXC) */
                    uint32_t c_size = ((uint32_t)(csd[7] & 0x3Fu) << 16)
                                    | ((uint32_t) csd[8]          <<  8)
                                    |             csd[9];
                    sectors = (c_size + 1) << 10;
                } else {
                    /* CSD v1 (SDSC) */
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