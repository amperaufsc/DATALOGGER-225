/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXTRA_2_Pin GPIO_PIN_0
#define EXTRA_2_GPIO_Port GPIOA
#define BRK_PRESSURE_F_Pin GPIO_PIN_1
#define BRK_PRESSURE_F_GPIO_Port GPIOA
#define BRK_PRESSURE_T_Pin GPIO_PIN_2
#define BRK_PRESSURE_T_GPIO_Port GPIOA
#define EXTRA_3_Pin GPIO_PIN_3
#define EXTRA_3_GPIO_Port GPIOA
#define SUSP_TD_Pin GPIO_PIN_4
#define SUSP_TD_GPIO_Port GPIOA
#define SUSP_FE_Pin GPIO_PIN_5
#define SUSP_FE_GPIO_Port GPIOA
#define SUSP_FD_Pin GPIO_PIN_6
#define SUSP_FD_GPIO_Port GPIOA
#define SUSP_TE_Pin GPIO_PIN_7
#define SUSP_TE_GPIO_Port GPIOA
#define STR_ANG_Pin GPIO_PIN_0
#define STR_ANG_GPIO_Port GPIOB
#define EXTRA_1_Pin GPIO_PIN_1
#define EXTRA_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
