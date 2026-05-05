/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "rc.h"
#include "tim.h"
#include "dshot.h"
#include "flare.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DSHOT_CCR_IDLE  640U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t tim6_isr_count = 0;
extern volatile uint8_t dshot_dma_busy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim4_up;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers             */
/******************************************************************************/

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  uint32_t *sp = (uint32_t *)__get_MSP();
  char buf[64];
  snprintf(buf, sizeof(buf), "\r\n[FAULT] PC=0x%08lX LR=0x%08lX\r\n",
           sp[6], sp[5]);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 1000);
  /* USER CODE END HardFault_IRQn 0 */
  while (1) {}
}

void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {}
}

void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */
  while (1) {}
}

void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {}
}

void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */
  /* USER CODE END SVCall_IRQn 1 */
}

void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  /* USER CODE END DebugMonitor_IRQn 1 */
}

void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
  /* USER CODE END PendSV_IRQn 1 */
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
  if (DMA1->LISR & DMA_LISR_TCIF0) {
    DMA1->LIFCR = DMA_LIFCR_CTCIF0;
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    TIM4->DIER &= ~TIM_DIER_UDE;
    TIM4->CCR1 = DSHOT_CCR_IDLE;
    TIM4->CCR2 = DSHOT_CCR_IDLE;
    TIM4->CCR3 = DSHOT_CCR_IDLE;
    TIM4->CCR4 = DSHOT_CCR_IDLE;
    dshot_dma_busy = 0;
    return;
  }
  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_up);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)) {
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    tim6_isr_count++;
    DSHOT_SendThrottle(dshot_m1, dshot_m2, dshot_m3, dshot_m4);
    return;
  }
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  uint32_t isr = USART2->ISR;

  if (isr & USART_ISR_RXNE_RXFNE) {
    rc_rx_byte = (uint8_t)(USART2->RDR & 0xFF);
    USART2->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_FECF;
    RC_UART_RxCpltCallback();
    return;
  }

  USART2->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_FECF;

  HAL_UART_IRQHandler(&huart2);
  /* USER CODE END USART2_IRQn 0 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */