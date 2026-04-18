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
#include "gpio.h"
#include "spi.h"
#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── BMI323 register addresses ──────────────────────────────────────────── */
#define BMI323_REG_CHIP_ID 0x00
#define BMI323_REG_ACC_DATA 0x03
#define BMI323_REG_GYR_DATA 0x06
#define BMI323_REG_ACC_CONF 0x20
#define BMI323_REG_GYR_CONF 0x21
#define BMI323_REG_CMD 0x7E
#define BMI323_REG_FEATURE_IO0 0x10
#define BMI323_REG_FEATURE_IO1 0x11
#define BMI323_REG_FEATURE_IO2 0x12
#define BMI323_REG_FEATURE_IO_ST 0x14
#define BMI323_REG_FEATURE_CTRL 0x40

/* ── BMI323 configuration values ────────────────────────────────────────── */
#define BMI323_CMD_SOFT_RESET 0xDEAF

/*
 * ACC_CONF = 0x4028
 *   bits [15:12] = 0x4 → Continuous mode
 *   bits [7:4]   = 0x2 → ODR 100 Hz
 *   bits [3:0]   = 0x8 → ±8g range
 *
 * GYR_CONF = 0x4048
 *   bits [15:12] = 0x4 → Continuous mode
 *   bits [7:4]   = 0x4 → ODR 100 Hz
 *   bits [3:0]   = 0x8 → ±2000 dps range
 */
#define BMI323_ACC_CONF_VAL 0x4028
#define BMI323_GYR_CONF_VAL 0x4048

/* Loop interval — 10ms = 100Hz */
#define IMU_LOOP_INTERVAL_MS 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*
 * Post-write inter-transaction delay.
 * 200 NOPs ≥ 2 µs at 96 MHz per BMI323 datasheet timing requirement.
 */
#define BMI323_POST_WRITE_DELAY()                                              \
  do {                                                                         \
    for (int _d = 0; _d < 200; _d++) {                                         \
      __NOP();                                                                 \
    }                                                                          \
  } while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint16_t who_am_i_result = 0;
volatile uint16_t acc_conf_readback = 0;
volatile uint16_t gyr_conf_readback = 0;
volatile uint16_t acc_conf_default = 0;

volatile int16_t imu_acc_x = 0;
volatile int16_t imu_acc_y = 0;
volatile int16_t imu_acc_z = 0;

volatile int16_t imu_gyr_x = 0;
volatile int16_t imu_gyr_y = 0;
volatile int16_t imu_gyr_z = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART_Print(const char *s) {
  HAL_UART_Transmit(&huart1, (const uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

static void SPI1_FlushRxFifo(void) {
  __HAL_SPI_DISABLE(&hspi1);
  __HAL_SPI_ENABLE(&hspi1);
}

/**
 * @brief  Read a 16-bit register from the BMI323 over SPI.
 *
 *         Protocol (4 bytes):
 *           TX: [addr | 0x80]  [0x00 dummy]  [0x00]  [0x00]
 *           RX: [ignored]      [ignored]     [LSB]   [MSB]
 *
 *         rx[2]=LSB, rx[3]=MSB — confirmed via hardware observation.
 *         For 1-byte registers (e.g. WHO_AM_I), rx[3] will contain
 *         garbage — mask with 0x00FF before comparing.
 */
static uint16_t BMI323_ReadReg(uint8_t reg) {
  uint8_t tx[4] = {reg | 0x80, 0x00, 0x00, 0x00};
  uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

  SPI1_FlushRxFifo();

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 10);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  SPI1_FlushRxFifo();

  return (uint16_t)(rx[2] | (rx[3] << 8));
}

/**
 * @brief  Write a 16-bit value to a BMI323 register over SPI.
 *
 *         Protocol (4 bytes — padded to match read transaction size):
 *           TX: [addr & 0x7F]  [data_LSB]  [data_MSB]  [0x00 dummy]
 *
 *         The BMI323 clocks in the first 3 bytes and ignores the dummy.
 *         4-byte HAL_SPI_TransmitReceive is used because the STM32H7 SPI
 *         peripheral drops the 3rd byte on 3-byte transactions regardless
 *         of approach (HAL_SPI_Transmit, HAL_SPI_TransmitReceive, or direct
 *         register access with TSIZE=3). Confirmed fixed at 4 bytes.
 */
static void BMI323_WriteReg(uint8_t reg, uint16_t val) {
  uint8_t tx[4] = {
      reg & 0x7F,
      (uint8_t)(val & 0xFF),        /* LSB */
      (uint8_t)((val >> 8) & 0xFF), /* MSB */
      0x00,                         /* dummy pad */
  };
  uint8_t rx[4] = {0};

  SPI1_FlushRxFifo();

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 10);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  SPI1_FlushRxFifo();
  BMI323_POST_WRITE_DELAY();
}

/**
 * @brief  Burst-read N 16-bit words from consecutive BMI323 registers.
 *
 *         Frame: [addr|0x80] [dummy] [D0_LSB D0_MSB D1_LSB D1_MSB ...]
 *         rx[2]=LSB, rx[3]=MSB per word — confirmed byte order.
 */
static void BMI323_BurstRead(uint8_t reg, int16_t *out, uint8_t count) {
  uint8_t tx_len = 2 + (count * 2);
  uint8_t tx[14] = {0};
  uint8_t rx[14] = {0};

  tx[0] = reg | 0x80;

  SPI1_FlushRxFifo();

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, tx_len, 10);
  __NOP();
  __NOP();
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  SPI1_FlushRxFifo();

  for (uint8_t i = 0; i < count; i++) {
    uint8_t base = 2 + (i * 2);
    out[i] = (int16_t)(rx[base] | (rx[base + 1] << 8));
  }
}

/**
 * @brief  Initialize the BMI323 Feature Engine.
 *
 *         Must be called before writing ACC_CONF / GYR_CONF.
 *         The BMI323 silently ignores mode bit writes to config
 *         registers until the feature engine is initialized.
 *
 * @retval 0 on success, 1 on error or timeout
 */
static uint8_t BMI323_InitFeatureEngine(void) {
  BMI323_WriteReg(BMI323_REG_FEATURE_IO0, 0x0000);
  HAL_Delay(1);

  BMI323_WriteReg(BMI323_REG_FEATURE_IO2, 0x012C);
  HAL_Delay(1);

  BMI323_WriteReg(BMI323_REG_FEATURE_IO_ST, 0x0001);
  HAL_Delay(1);

  BMI323_WriteReg(BMI323_REG_FEATURE_CTRL, 0x0001);
  HAL_Delay(10);

  /* Poll FEATURE_IO1 — wait for bit[3:0] == 0x01 (ready) */
  for (int i = 0; i < 50; i++) {
    HAL_Delay(10);
    uint16_t status = BMI323_ReadReg(BMI323_REG_FEATURE_IO1);
    uint8_t err = status & 0x0F;
    if (err == 0x01)
      return 0; /* ready */
    if (err == 0x03)
      return 1; /* feature engine error */
  }

  return 1; /* timeout */
}

/**
 * @brief  Initialize the BMI323 — soft reset, verify WHO_AM_I,
 *         init feature engine, activate accel + gyro at 100 Hz.
 *
 * @retval 0  success
 *         1  WHO_AM_I mismatch (SPI comms failure)
 *         2  feature engine timeout or error
 */
static uint8_t BMI323_Init(void) {
  HAL_Delay(10);

  /* Soft reset — BMI323 resets mid-transaction so use a short timeout.
   * The peripheral is re-initialized after the reset delay. */
  uint8_t rst_tx[3] = {BMI323_REG_CMD & 0x7F, 0xAF, 0xDE};
  uint8_t rst_rx[3] = {0};
  SPI1_FlushRxFifo();
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, rst_tx, rst_rx, 3, 10);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  HAL_Delay(50);

  /* Re-initialize SPI peripheral — clears any state from aborted reset */
  HAL_SPI_DeInit(&hspi1);
  HAL_Delay(1);
  MX_SPI1_Init();
  HAL_Delay(1);

  /* Dummy read — switches BMI323 from I2C to SPI mode post-reset */
  BMI323_ReadReg(BMI323_REG_CHIP_ID);
  HAL_Delay(50);
  BMI323_ReadReg(BMI323_REG_CHIP_ID);
  HAL_Delay(10);

  /* Verify WHO_AM_I — low byte must be 0x43, high byte is garbage for
   * 1-byte registers and is masked out */
  who_am_i_result = BMI323_ReadReg(BMI323_REG_CHIP_ID);
  if ((who_am_i_result & 0x00FF) != 0x43) {
    return 1;
  }

  acc_conf_default = BMI323_ReadReg(BMI323_REG_ACC_CONF);

  if (BMI323_InitFeatureEngine() != 0) {
    return 2;
  }

  BMI323_WriteReg(BMI323_REG_ACC_CONF, BMI323_ACC_CONF_VAL);
  HAL_Delay(10);
  acc_conf_readback = BMI323_ReadReg(BMI323_REG_ACC_CONF);

  BMI323_WriteReg(BMI323_REG_GYR_CONF, BMI323_GYR_CONF_VAL);
  HAL_Delay(10);
  gyr_conf_readback = BMI323_ReadReg(BMI323_REG_GYR_CONF);

  return 0;
}

static void BMI323_ReadAccel(void) {
  int16_t buf[3];
  BMI323_BurstRead(BMI323_REG_ACC_DATA, buf, 3);
  imu_acc_x = buf[0];
  imu_acc_y = buf[1];
  imu_acc_z = buf[2];
}

static void BMI323_ReadGyro(void) {
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
int main(void) {

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration -------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  UART_Print("[FLARE] boot ok\r\n");

  uint8_t init_result = BMI323_Init();
  if (init_result == 1) {
    UART_Print("[IMU] INIT FAILED -- WHO_AM_I mismatch\r\n");
    Error_Handler();
  } else if (init_result == 2) {
    UART_Print("[IMU] INIT FAILED -- feature engine timeout\r\n");
    Error_Handler();
  }

  char msg[96];

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

  UART_Print("[IMU] starting 100Hz loop\r\n");

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    BMI323_ReadAccel();
    BMI323_ReadGyro();

    snprintf(msg, sizeof(msg), "A:%6d %6d %6d  G:%6d %6d %6d\r\n", imu_acc_x,
             imu_acc_y, imu_acc_z, imu_gyr_x, imu_gyr_y, imu_gyr_z);
    UART_Print(msg);

    HAL_Delay(IMU_LOOP_INTERVAL_MS);
  }
  /* USER CODE END 3 */
}

/**
 * @brief  System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  Error handler — halts execution.
 * @retval None
 */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */