/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* ===== CONFIG YAW FUSION ===== */
#define RAD_TO_DEG              57.29577951308232f
#define YAW_ALPHA               0.98f

/* ===== OFFSET YAW ===== */
#define MAG_DECLINATION_DEG     -3.0f
#define MOUNTING_OFFSET_DEG     -73.0f
#define TOTAL_YAW_OFFSET        (MAG_DECLINATION_DEG + MOUNTING_OFFSET_DEG)

/* ===== DEBUG: CẮT 2 SỐ THẬP PHÂN (KHÔNG LÀM TRÒN) ===== */
#define TRUNC2(x)   ((float)((int32_t)((x) * 100.0f)) / 100.0f)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* ===== HANDLES TOÀN CỤC (dùng chung giữa các module) ===== */
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

/* ===== SYSTEM ===== */
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DONG_Pin GPIO_PIN_13
#define DONG_GPIO_Port GPIOC
#define TAY_Pin GPIO_PIN_5
#define TAY_GPIO_Port GPIOA
#define BAC_Pin GPIO_PIN_11
#define BAC_GPIO_Port GPIOB
#define ICM_CS_Pin GPIO_PIN_8
#define ICM_CS_GPIO_Port GPIOA
#define NAM_Pin GPIO_PIN_8
#define NAM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
