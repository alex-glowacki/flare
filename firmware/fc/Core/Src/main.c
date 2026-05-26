/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body — FLARE FC (STM32H723ZGT6)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include "flare.h"
#include "flare_protocol.h"
#include "gps.h"
#include "imu_fusion.h"
#include "mag.h"
#include "rc.h"
#include "sd.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── BMI323 register addresses ──────────────────────────────────────────── */
#define BMI323_REG_CHIP_ID       0x00
#define BMI323_REG_ACC_DATA      0x03
#define BMI323_REG_GYR_DATA      0x06
#define BMI323_REG_ACC_CONF      0x20
#define BMI323_REG_GYR_CONF      0x21
#define BMI323_REG_CMD           0x7E
#define BMI323_REG_FEATURE_IO0   0x10
#define BMI323_REG_FEATURE_IO1   0x11
#define BMI323_REG_FEATURE_IO2   0x12
#define BMI323_REG_FEATURE_IO_ST 0x14
#define BMI323_REG_FEATURE_CTRL  0x40

/* ── BMI323 configuration values ────────────────────────────────────────── */
#define BMI323_CMD_SOFT_RESET    0xDEAF
#define BMI323_ACC_CONF_VAL      0x4028
#define BMI323_GYR_CONF_VAL      0x4048

/* Loop interval — 10ms = 100Hz */
#define IMU_LOOP_INTERVAL_MS     10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BMI323_POST_WRITE_DELAY()                                              \
  do {                                                                         \
    for (int _d = 0; _d < 200; _d++) {                                         \
      __NOP();                                                                 \
    }                                                                          \
  } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern volatile uint32_t dshot_send_count;
extern volatile uint32_t dshot_tc_count;

volatile uint16_t who_am_i_result   = 0;
volatile uint16_t acc_conf_readback = 0;
volatile uint16_t gyr_conf_readback = 0;
volatile uint16_t acc_conf_default  = 0;

volatile int16_t imu_acc_x = 0;
volatile int16_t imu_acc_y = 0;
volatile int16_t imu_acc_z = 0;

volatile int16_t imu_gyr_x = 0;
volatile int16_t imu_gyr_y = 0;
volatile int16_t imu_gyr_z = 0;

/* Last known-good accel samples — used to replace all-zero dropout reads */
static int16_t last_acc_x = 0;
static int16_t last_acc_y = 0;
static int16_t last_acc_z = 0;

IMU_Fusion_t imu_fusion;

MAG_Cal_t mag_cal;  /* hard-iron offsets — zero until calibrated */
float mag_heading;  /* degrees [0, 360), updated each loop       */

static FLARE_RC_Packet_t rc_pkt = {0};  /* persists across loop iterations */

GPS_Data_t gps_data;  /* latest parsed GPS fix, updated each loop */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART_Print(const char *s)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

static void SPI1_FlushRxFifo(void)
{
    __HAL_SPI_DISABLE(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);
}

static uint16_t BMI323_ReadReg(uint8_t reg)
{
    uint8_t tx[4] = {reg | 0x80, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};
    SPI1_FlushRxFifo();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 10);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    SPI1_FlushRxFifo();
    return (uint16_t)(rx[2] | (rx[3] << 8));
}

static void BMI323_WriteReg(uint8_t reg, uint16_t val)
{
    uint8_t tx[4] = {
        reg & 0x7F,
        (uint8_t)(val & 0xFF),
        (uint8_t)((val >> 8) & 0xFF),
        0x00,
    };
    uint8_t rx[4] = {0};
    SPI1_FlushRxFifo();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 10);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    SPI1_FlushRxFifo();
    BMI323_POST_WRITE_DELAY();
}

static void BMI323_BurstRead(uint8_t reg, int16_t *out, uint8_t count)
{
    uint8_t tx_len = 2 + (count * 2);
    uint8_t tx[14] = {0};
    uint8_t rx[14] = {0};
    tx[0] = reg | 0x80;
    SPI1_FlushRxFifo();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP();
    (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, tx_len, 10);
    __NOP(); __NOP();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    SPI1_FlushRxFifo();
    for (uint8_t i = 0; i < count; i++) {
        uint8_t base = 2 + (i * 2);
        out[i] = (int16_t)(rx[base] | (rx[base + 1] << 8));
    }
}

static uint8_t BMI323_InitFeatureEngine(void)
{
    BMI323_WriteReg(BMI323_REG_FEATURE_IO0, 0x0000); HAL_Delay(1);
    BMI323_WriteReg(BMI323_REG_FEATURE_IO2, 0x012C); HAL_Delay(1);
    BMI323_WriteReg(BMI323_REG_FEATURE_IO_ST, 0x0001); HAL_Delay(1);
    BMI323_WriteReg(BMI323_REG_FEATURE_CTRL, 0x0001); HAL_Delay(10);
    for (int i = 0; i < 50; i++) {
        HAL_Delay(10);
        uint16_t status = BMI323_ReadReg(BMI323_REG_FEATURE_IO1);
        uint8_t err = status & 0x0F;
        if (err == 0x01) return 0;
        if (err == 0x03) return 1;
    }
    return 1;
}

static uint8_t BMI323_Init(void)
{
    HAL_Delay(10);
    uint8_t rst_tx[3] = {BMI323_REG_CMD & 0x7F, 0xAF, 0xDE};
    uint8_t rst_rx[3] = {0};
    SPI1_FlushRxFifo();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi1, rst_tx, rst_rx, 3, 10);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_SPI_DeInit(&hspi1);
    HAL_Delay(1);
    MX_SPI1_Init();
    HAL_Delay(1);
    BMI323_ReadReg(BMI323_REG_CHIP_ID); HAL_Delay(50);
    BMI323_ReadReg(BMI323_REG_CHIP_ID); HAL_Delay(10);
    who_am_i_result = BMI323_ReadReg(BMI323_REG_CHIP_ID);
    if ((who_am_i_result & 0x00FF) != 0x43) return 1;
    acc_conf_default = BMI323_ReadReg(BMI323_REG_ACC_CONF);
    if (BMI323_InitFeatureEngine() != 0) return 2;
    BMI323_WriteReg(BMI323_REG_ACC_CONF, BMI323_ACC_CONF_VAL); HAL_Delay(10);
    acc_conf_readback = BMI323_ReadReg(BMI323_REG_ACC_CONF);
    BMI323_WriteReg(BMI323_REG_GYR_CONF, BMI323_GYR_CONF_VAL); HAL_Delay(10);
    gyr_conf_readback = BMI323_ReadReg(BMI323_REG_GYR_CONF);
    return 0;
}

static void BMI323_ReadAccel(void)
{
    int16_t buf[3];
    BMI323_BurstRead(BMI323_REG_ACC_DATA, buf, 3);
    imu_acc_x = buf[0];
    imu_acc_y = buf[1];
    imu_acc_z = buf[2];
}

static void BMI323_ReadGyro(void)
{
    int16_t buf[3];
    BMI323_BurstRead(BMI323_REG_GYR_DATA, buf, 3);
    imu_gyr_x = buf[0];
    imu_gyr_y = buf[1];
    imu_gyr_z = buf[2];
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

    char msg[128];
    uint32_t loop_count = 0;

    HAL_Delay(10);

    DSHOT_Init();
    UART_Print("[DSHOT] driver ready\r\n");

    HAL_TIM_Base_Start_IT(&htim6);
    UART_Print("[DSHOT] 1kHz timer started\r\n");

    snprintf(msg, sizeof(msg), "[DSHOT] arming loop start t=%lums\r\n",
             HAL_GetTick());
    UART_Print(msg);

    for (int i = 0; i < 800; i++) {
        HAL_Delay(10);
    }

    snprintf(msg, sizeof(msg), "[DSHOT] arming loop done  t=%lums\r\n",
             HAL_GetTick());
    UART_Print(msg);
    UART_Print("[DSHOT] ESCs armed\r\n");

    UART_Print("[FLARE] boot ok\r\n");

    uint8_t init_result = BMI323_Init();
    if (init_result == 1) {
        UART_Print("[IMU] INIT FAILED -- WHO_AM_I mismatch\r\n");
        Error_Handler();
    } else if (init_result == 2) {
        UART_Print("[IMU] INIT FAILED -- feature engine timeout\r\n");
        Error_Handler();
    }

    snprintf(msg, sizeof(msg), "[IMU] WHO_AM_I    = 0x%02X (expect 0x43)\r\n",
             who_am_i_result & 0x00FF);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "[IMU] ACC default = 0x%04X\r\n",
             acc_conf_default);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "[IMU] ACC write   = 0x%04X (expect 0x4028)\r\n",
             acc_conf_readback);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "[IMU] GYR write   = 0x%04X (expect 0x4048)\r\n",
             gyr_conf_readback);
    UART_Print(msg);

    IMU_Fusion_Init(&imu_fusion);
    UART_Print("[FUSION] complementary filter ready\r\n");

    mag_cal.offset_x = 0.0f;
    mag_cal.offset_y = 0.0f;
    uint8_t mag_chip_id = 0;
    uint8_t mag_result = MAG_Init(&hi2c1, &mag_chip_id);
    uint8_t mag_ok = (mag_result != MAG_ERR_I2C);
    snprintf(msg, sizeof(msg), "[MAG] chip ID = 0x%02X\r\n", mag_chip_id);
    UART_Print(msg);
    if (!mag_ok) {
        UART_Print("[MAG] INIT FAILED -- skipping mag reads\r\n");
    } else {
        UART_Print("[MAG] QMC5883P ready\r\n");
    }

    FLARE_Init();
    UART_Print("[FLARE] PID controller ready\r\n");

    RC_Init();
    snprintf(msg, sizeof(msg), "[RC] init status=%u (0=OK)\r\n", rc_init_status);
    UART_Print(msg);
    UART_Print("[RC] USART2 receiver ready\r\n");

    /* ── GPS init ───────────────────────────────────────────────────────── */
    GPS_Init(&huart3, &gps_data);
    UART_Print("[GPS] USART3 DMA receiver ready\r\n");
    UART_Print("[GPS] waiting for fix (may take 30-120s outdoors)...\r\n");

    /* ── IMU level calibration ───────────────────────────────────────────── */
    UART_Print("[CAL] Hold still — levelling IMU (2s)...\r\n");
    float imu_roll_offset  = 0.0f;
    float imu_pitch_offset = 0.0f;
    float cal_roll_sum  = 0.0f;
    float cal_pitch_sum = 0.0f;
    for (int cal_i = 0; cal_i < 200; cal_i++) {
        BMI323_ReadAccel();
        BMI323_ReadGyro();
        IMU_Fusion_Update(&imu_fusion,
                          imu_acc_x, imu_acc_y, imu_acc_z,
                          imu_gyr_x, imu_gyr_y, imu_gyr_z,
                          0.0f, 0.01f, 0.96f, 0.90f);
        if (cal_i >= 150) {
            cal_roll_sum  += imu_fusion.roll;
            cal_pitch_sum += imu_fusion.pitch;
        }
        HAL_Delay(10);
    }
    imu_roll_offset  = cal_roll_sum  / 50.0f;
    imu_pitch_offset = cal_pitch_sum / 50.0f;
    snprintf(msg, sizeof(msg), "[CAL] level offset: roll=%.2f  pitch=%.2f\r\n",
             imu_roll_offset, imu_pitch_offset);
    UART_Print(msg);

    UART_Print("[IMU] starting 100Hz loop\r\n");

    /* ── SD card blackbox logger init ───────────────────────────────────── */
    SD_Log_Result_t sd_log_result = SD_Log_Init();
    UART_Print(SD_Log_StatusStr());
    UART_Print("\r\n");
    if (sd_log_result != SD_LOG_OK) {
        UART_Print("[SD] WARNING: logging disabled\r\n");
    }

    /* SD init disturbs SPI1 — reinitialize BMI323 to restore its config */
    if (BMI323_Init() == 0) {
        UART_Print("[IMU] post-SD reinit OK\r\n");
    } else {
        UART_Print("[IMU] post-SD reinit FAILED\r\n");
        Error_Handler();
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        uint32_t tick_start = HAL_GetTick();

        /* Flush SPI1 RX FIFO before every IMU read — prevents residual bytes
         * from prior SD transactions corrupting the first BMI323 burst read. */
        SPI1_FlushRxFifo();

        BMI323_ReadAccel();
        BMI323_ReadGyro();

        /* Dropout guard — all-zero accel is physically impossible in normal
         * operation and indicates a corrupted SPI read caused by SD bus
         * contention. Replace with last known-good sample so the fusion
         * filter receives a continuous valid signal. */
        if (imu_acc_x == 0 && imu_acc_y == 0 && imu_acc_z == 0) {
            imu_acc_x = last_acc_x;
            imu_acc_y = last_acc_y;
            imu_acc_z = last_acc_z;
        } else {
            last_acc_x = imu_acc_x;
            last_acc_y = imu_acc_y;
            last_acc_z = imu_acc_z;
        }

        float new_heading;
        if (mag_ok && MAG_ReadHeading(&hi2c1, &mag_cal, &new_heading) == MAG_OK) {
            mag_heading = new_heading;
        }

        IMU_Fusion_Update(&imu_fusion,
                          imu_acc_x, imu_acc_y, imu_acc_z,
                          imu_gyr_x, imu_gyr_y, imu_gyr_z,
                          mag_heading, 0.01f, 0.96f, 0.90f);

        float gx_dps = imu_gyr_x / 16.384f;
        float gy_dps = imu_gyr_y / 16.384f;
        float gz_dps = imu_gyr_z / 16.384f;

        float roll_corr  = imu_fusion.roll  - imu_roll_offset;
        float pitch_corr = imu_fusion.pitch - imu_pitch_offset;

        /* ── GPS tick ───────────────────────────────────────────────────── */
        GPS_Update(&gps_data);

        /* ── RC packet consumption ──────────────────────────────────────── */
        if (RC_GetPacket(&rc_pkt)) {
            snprintf(msg, sizeof(msg),
                     "[RC] arm=%u thr=%u rol=%u pit=%u yaw=%u mode=%u crc_fail=%lu\r\n",
                     rc_pkt.armed,
                     rc_pkt.throttle, rc_pkt.roll,
                     rc_pkt.pitch,    rc_pkt.yaw,
                     rc_pkt.mode,
                     rc_crc_failures);
            UART_Print(msg);
        }

        if (RC_IsHealthy() && rc_pkt.mode != FLARE_MODE_SAFE) {
            motors_enabled = 1;

            FLARE_SetRollSP   (((float)rc_pkt.roll    - 1500.0f) * (30.0f  / 500.0f));
            FLARE_SetPitchSP  (((float)rc_pkt.pitch   - 1500.0f) * (30.0f  / 500.0f));
            FLARE_SetYawRateSP(((float)rc_pkt.yaw     - 1500.0f) * (200.0f / 500.0f));

            uint16_t dshot_thr = (uint16_t)(
                ((float)(rc_pkt.throttle - 1000) / 1000.0f)
                * (2047.0f - 48.0f) + 48.0f);
            if (dshot_thr < 48) dshot_thr = 48;

            FLARE_SetThrottle(dshot_thr);
            FLARE_SetArmed(1);
            FLARE_Update(roll_corr, pitch_corr,
                         gx_dps, gy_dps, gz_dps, 0.01f);

            snprintf(msg, sizeof(msg),
                     "[THR] rc=%u dshot=%u m1=%u m2=%u m3=%u m4=%u\r\n",
                     rc_pkt.throttle, dshot_thr,
                     dshot_m1, dshot_m2, dshot_m3, dshot_m4);
            UART_Print(msg);
        } else {
            motors_enabled = 0;
            FLARE_SetArmed(0);
            FLARE_SetThrottle(0);
            FLARE_Update(roll_corr, pitch_corr,
                         gx_dps, gy_dps, gz_dps, 0.0f);

            snprintf(msg, sizeof(msg),
                     "[SAFE] m1=%u m2=%u m3=%u m4=%u\r\n",
                     dshot_m1, dshot_m2, dshot_m3, dshot_m4);
            UART_Print(msg);
        }

        snprintf(msg, sizeof(msg),
                 "A:%6d %6d %6d  G:%6d %6d %6d"
                 "  R:%7.2f  P:%7.2f  Y:%7.2f  RC:%s\r\n",
                 imu_acc_x, imu_acc_y, imu_acc_z,
                 imu_gyr_x, imu_gyr_y, imu_gyr_z,
                 roll_corr, pitch_corr, imu_fusion.yaw,
                 RC_IsHealthy() ? "OK" : "LOST");
        UART_Print(msg);

        /* ── GPS telemetry print (once per second = every 100 loops) ────── */
        if (loop_count % 100 == 0) {
            snprintf(msg, sizeof(msg),
                     "[GPS] fix=%u sats=%u lat=%.6f lon=%.6f alt=%.1fft spd=%.1fmph\r\n",
                     gps_data.fix_valid,
                     gps_data.satellites,
                     (double)gps_data.latitude,
                     (double)gps_data.longitude,
                     (double)gps_data.altitude_ft,
                     (double)gps_data.speed_mph);
            UART_Print(msg);

            snprintf(msg, sizeof(msg),
                     "[DIAG] send=%lu tc=%lu\r\n",
                     dshot_send_count, dshot_tc_count);
            UART_Print(msg);
        }

        ++loop_count;

        /* Deadline-based delay — sleep only the remaining time in this 10ms
         * tick. Prevents long operations from pushing the next IMU read late. */
        uint32_t elapsed = HAL_GetTick() - tick_start;
        if (elapsed < IMU_LOOP_INTERVAL_MS) {
            HAL_Delay(IMU_LOOP_INTERVAL_MS - elapsed);
        }

        /* ── SD operations AFTER delay — maximally far from next IMU read ──
         * SPI1_FlushRxFifo() after each call ensures the bus is clean before
         * the next tick's BMI323_ReadAccel(). */
        if (loop_count % 20 == 0) {
            SD_Log_Write(
                HAL_GetTick(),
                roll_corr, pitch_corr, imu_fusion.yaw,
                imu_acc_x, imu_acc_y, imu_acc_z,
                imu_gyr_x, imu_gyr_y, imu_gyr_z,
                rc_pkt.throttle,
                dshot_m1, dshot_m2, dshot_m3, dshot_m4,
                rc_pkt.armed,
                RC_IsHealthy(),
                gps_data.fix_valid,
                gps_data.satellites,
                gps_data.latitude,
                gps_data.longitude,
                gps_data.altitude_ft,
                gps_data.speed_mph
            );
            SPI1_FlushRxFifo();
        }

        if (loop_count % 100 == 1) {
            SD_Log_Flush();
            SPI1_FlushRxFifo();
        }

    /* USER CODE END 3 */
    }   /* end while(1) */
}       /* end main()   */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        RC_UART_RxCpltCallback();
    }
}

/* USER CODE END 4 */

static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    /*
     * Region 0: AXI SRAM (0x24000000, 512KB) — Non-cacheable.
     *
     * The DSHOT DMA buffer lives here. Non-cacheable ensures CPU writes
     * are immediately visible to DMA without explicit cache flush calls.
     * TEX=1, C=0, B=0 = Normal memory, Non-cacheable.
     */
    MPU_InitStruct.Enable             = MPU_REGION_ENABLE;
    MPU_InitStruct.Number             = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress        = 0x24000000;
    MPU_InitStruct.Size               = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable   = 0x00;
    MPU_InitStruct.TypeExtField       = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission   = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec        = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable        = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable        = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable       = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */