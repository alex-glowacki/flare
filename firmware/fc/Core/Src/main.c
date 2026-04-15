/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* BMI323 register addresses */
#define BMI323_REG_CHIP_ID 0x00
#define BMI323_REG_ACC_DATA                                                    \
  0x03 /* Accel X LSB — burst read X/Y/Z from here                           \
        */
#define BMI323_REG_GYR_DATA                                                    \
  0x06                           /* Gyro  X LSB — burst read X/Y/Z from here \
                                  */
#define BMI323_REG_ACC_CONF 0x20 /* Accel config: mode, ODR, range, BW */
#define BMI323_REG_GYR_CONF 0x21 /* Gyro  config: mode, ODR, range, BW */
#define BMI323_REG_CMD 0x7E      /* Command register - soft reset */
#define BMI323_CMD_SOFT_RESET 0xDEAF /* Soft reset command */

/*
 * ACC_CONF / GYR_CONF bit fields (combined 16-bit value written via WriteReg):
 *
 *   Bits [3:0]  ODR   — 0x08 = 100 Hz
 *   Bits [6:4]  Range — accel: 0x0 = ±2g  | gyro: 0x0 = ±125 dps
 *   Bits [11:8] Mode  — 0x4  = continuous (normal) mode
 *
 * Full value = (mode << 8) | (range << 4) | odr
 *            = (0x4 << 8) | (0x0 << 4) | 0x8
 *            = 0x0408
 *
 * Using ±2g / ±125 dps for initial bring-up — tightest range, easiest to
 * verify sensor is responding. Widen after calibration in Phase 3.
 */
#define BMI323_ACC_CONF_VAL 0x4008
#define BMI323_GYR_CONF_VAL 0x4008

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t who_am_i_result = 0;
volatile uint16_t acc_conf_readback = 0; /* ACC_CONF register readback */
volatile uint16_t gyr_conf_readback = 0; /* GYR_CONF register readback */
volatile uint16_t acc_conf_default = 0;  /* ACC_CONF before any write */

/* Raw accel counts — signed 16-bit, updated every loop iteration */
volatile int16_t imu_acc_x = 0;
volatile int16_t imu_acc_y = 0;
volatile int16_t imu_acc_z = 0;

/* Raw gyro counts — signed 16-bit, updated every loop iteration */
volatile int16_t imu_gyr_x = 0;
volatile int16_t imu_gyr_y = 0;
volatile int16_t imu_gyr_z = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Read a 16-bit register from the BMI323 over SPI.
 *         The BMI323 SPI protocol requires one dummy byte after
 *         the address byte before valid data begins.
 * @param  reg    Register address to read
 * @retval 16-bit register value (little-endian, LSB first)
 */
static uint16_t BMI323_ReadReg(uint8_t reg) {
  uint8_t tx[3] = {reg | 0x80, 0x00, 0x00};
  uint8_t rx[3];
  memset(rx, 0, sizeof(rx)); /* ensure no stale data */

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /* rx[0]=dummy, rx[1]=MSB, rx[2]=LSB */
  return (uint16_t)((rx[1] << 8) | rx[2]);
}

/**
 * @brief  Write a 16-bit value to a BMI323 register over SPI.
 *         The BMI323 accepts: [addr_byte] [LSB] [MSB]
 *         (no dummy byte on writes — dummy byte is read-only protocol)
 * @param  reg    Register address to write
 * @param  val    16-bit value to write (LSB first)
 */
static void BMI323_WriteReg(uint8_t reg, uint16_t val) {
  uint8_t tx[3] = {
      reg & 0x7F,                   /* bit7=0 → write */
      (uint8_t)((val >> 8) & 0xFF), /* MSB first */
      (uint8_t)(val & 0xFF)         /* LSB second */
  };
  uint8_t rx[3] = {0};

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Burst-read N consecutive 16-bit registers from the BMI323.
 *         Used to read accel (X/Y/Z) and gyro (X/Y/Z) data in one transaction.
 *
 *         Protocol: [addr|0x80] [dummy] [D0_LSB D0_MSB D1_LSB D1_MSB ...]
 *
 * @param  reg      Starting register address
 * @param  out      Output array of int16_t — length must be >= count
 * @param  count    Number of 16-bit words to read
 */
static void BMI323_BurstRead(uint8_t reg, int16_t *out, uint8_t count) {
  /*
   * Total SPI bytes:
   *   1 address byte + 1 dummy byte + (count * 2 data bytes)
   */
  uint8_t tx_len = 2 + (count * 2);
  uint8_t tx[14] = {0}; /* max: 2 + 6*2 = 14 for a 6-word burst */
  uint8_t rx[14] = {0};

  tx[0] = reg | 0x80; /* read flag */
  /* tx[1..n] stay 0x00 — dummy + clocking out data */

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, tx_len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /* rx[0] = addr echo (discard), rx[1] = dummy (discard)
   * rx[2], rx[3] = word 0 LSB, MSB
   * rx[4], rx[5] = word 1 LSB, MSB  ... etc */
  for (uint8_t i = 0; i < count; i++) {
    uint8_t base = 2 + (i * 2);
    out[i] = (int16_t)(rx[base] | (rx[base + 1] << 8));
  }
}

/**
 * @brief  Initialize the BMI323 — activate accel and gyro at 100Hz.
 *         Must be called after SPI is initialized and before any data reads.
 * @retval 0 on success, 1 if WHO_AM_I check fails
 */
static uint8_t BMI323_Init(void) {
  HAL_Delay(10); /* wait for BMI323 power-on reset to complete */

  /*
   * Issue a soft reset before any configuration.
   * Required to put the sensor into a known state.
   * CMD register = 0x7E, value = 0xDEAF.
   * Datasheet section 5.17: 1.5ms delay required after reset.
   */
  BMI323_WriteReg(BMI323_REG_CMD, BMI323_CMD_SOFT_RESET);
  HAL_Delay(50); /* increased from 2ms - give reset plenty of time */

  /* Dummy read to switch interface back to SPI mode after reset */
  BMI323_ReadReg(BMI323_REG_CHIP_ID);
  HAL_Delay(10); /* increased from 1ms */

  /* Verify WHO_AM_I before touching config registers */
  who_am_i_result = BMI323_ReadReg(BMI323_REG_CHIP_ID);
  if (who_am_i_result != 0x0043) {
    return 1; /* comms failure — do not proceed */
  }

  /* Read ACC_CONF before touching it - should be 0x0028 (suspend mode default)
   */
  acc_conf_default = BMI323_ReadReg(BMI323_REG_ACC_CONF);

  /*
   * On power-up, accel and gyro are in suspend mode.
   * Writing ACC_CONF and GYR_CONF with a valid mode field activates them.
   * Allow 2ms after each write for the sensor to transition.
   */
  BMI323_WriteReg(BMI323_REG_ACC_CONF, BMI323_ACC_CONF_VAL);
  HAL_Delay(10);
  acc_conf_readback =
      BMI323_ReadReg(BMI323_REG_ACC_CONF); /* verify write landed */

  BMI323_WriteReg(BMI323_REG_GYR_CONF, BMI323_GYR_CONF_VAL);
  HAL_Delay(10);
  gyr_conf_readback =
      BMI323_ReadReg(BMI323_REG_GYR_CONF); /* verify write landed */

  return 0; /* success */
}

/**
 * @brief  Read raw accel X, Y, Z from BMI323 into global imu_acc_* variables.
 *         Uses burst read for efficiency (single CS transaction).
 */
static void BMI323_ReadAccel(void) {
  int16_t buf[3];
  BMI323_BurstRead(BMI323_REG_ACC_DATA, buf, 3);
  imu_acc_x = buf[0];
  imu_acc_y = buf[1];
  imu_acc_z = buf[2];
}

/**
 * @brief  Read raw gyro X, Y, Z from BMI323 into global imu_gyr_* variables.
 *         Uses burst read for efficiency (single CS transaction).
 */
static void BMI323_ReadGyro(void) {
  int16_t buf[3];
  BMI323_BurstRead(BMI323_REG_GYR_DATA, buf, 3);
  imu_gyr_x = buf[0];
  imu_gyr_y = buf[1];
  imu_gyr_z = buf[2];
}

/**
 * @brief  Send a string via ITM SWO for Cortex-Debug console output.
 * @param  s    Null-terminated string to transmit
 */
static void ITM_Print(const char *s) {
  while (*s) {
    ITM_SendChar((uint32_t)*s++);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ITM_Print("[FLARE] boot ok\r\n");

  /* BMI323 init — activates accel + gyro, verifies WHO_AM_I */
  if (BMI323_Init() != 0) {
    ITM_Print("[IMU] INIT FAILED — WHO_AM_I mismatch\r\n");
    Error_Handler();
  }

  char msg[64];
  snprintf(msg, sizeof(msg),
           "[IMU] WHO_AM_I = 0x%04X OK — accel+gyro active\r\n",
           who_am_i_result);
  ITM_Print(msg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
