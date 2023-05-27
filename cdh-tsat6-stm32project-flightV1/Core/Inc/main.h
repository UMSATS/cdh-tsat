/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define WDI_Pin GPIO_PIN_0
#define WDI_GPIO_Port GPIOC
#define M_nRESET_Pin GPIO_PIN_1
#define M_nRESET_GPIO_Port GPIOC
#define CAM_TX_Pin GPIO_PIN_0
#define CAM_TX_GPIO_Port GPIOA
#define CAM_RX_Pin GPIO_PIN_1
#define CAM_RX_GPIO_Port GPIOA
#define CAM_FSH_Pin GPIO_PIN_2
#define CAM_FSH_GPIO_Port GPIOA
#define CAM_ON_Pin GPIO_PIN_3
#define CAM_ON_GPIO_Port GPIOA
#define UHF_nCS_Pin GPIO_PIN_4
#define UHF_nCS_GPIO_Port GPIOC
#define UHF_nIRQ_Pin GPIO_PIN_5
#define UHF_nIRQ_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define FLASH_nWP_Pin GPIO_PIN_12
#define FLASH_nWP_GPIO_Port GPIOB
#define FLASH_nCS_Pin GPIO_PIN_6
#define FLASH_nCS_GPIO_Port GPIOC
#define FLASH_nHOLD_Pin GPIO_PIN_7
#define FLASH_nHOLD_GPIO_Port GPIOC
#define UHF_SDN_Pin GPIO_PIN_8
#define UHF_SDN_GPIO_Port GPIOC
#define MRAM_nWP_Pin GPIO_PIN_15
#define MRAM_nWP_GPIO_Port GPIOA
#define MRAM_nCS_Pin GPIO_PIN_2
#define MRAM_nCS_GPIO_Port GPIOD
#define RELEASE_nEN_Pin GPIO_PIN_6
#define RELEASE_nEN_GPIO_Port GPIOB
#define RELEASE_Pin GPIO_PIN_7
#define RELEASE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
