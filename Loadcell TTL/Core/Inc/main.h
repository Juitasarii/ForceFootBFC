/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  *Author: Juita Sari
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ForceFoot_DxlScan_Init(void);
void ForceFoot_DxlScan_Task(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/* Dynamixel Direction Control */
#define DXL_DIR_Pin GPIO_PIN_12
#define DXL_DIR_GPIO_Port GPIOB

/* 74HC125N 3-State Buffer Control (TX Buffer Only)
   Konfigurasi:
   - PA9  (UART1 TX) ──────► HC125N Pin 2 (1A)
   - HC125N Pin 3 (1Y) ────► U2D2 DATA (with pull-up 4.7K)
   - PA10 (UART1 RX) ─────► U2D2 DATA (direct)
   - PB12 (GPIO) ─────────► HC125N Pin 1 (1OE) - Output Enable
*/
#define HC125N_OE_Pin       GPIO_PIN_12
#define HC125N_OE_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
