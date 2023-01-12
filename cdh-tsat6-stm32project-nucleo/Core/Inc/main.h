/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_5
#define LD4_GPIO_Port GPIOA
#define RELEASE_Pin GPIO_PIN_5
#define RELEASE_GPIO_Port GPIOC
#define RELEASE_nEN_Pin GPIO_PIN_0
#define RELEASE_nEN_GPIO_Port GPIOB
#define UHF_irq_Pin GPIO_PIN_1
#define UHF_irq_GPIO_Port GPIOB
#define UHF_nCS_Pin GPIO_PIN_2
#define UHF_nCS_GPIO_Port GPIOB
#define UHF_SDN_Pin GPIO_PIN_11
#define UHF_SDN_GPIO_Port GPIOB
#define FLASH_nCS_Pin GPIO_PIN_15
#define FLASH_nCS_GPIO_Port GPIOB
#define FLASH_nWP_Pin GPIO_PIN_6
#define FLASH_nWP_GPIO_Port GPIOC
#define FLASH_nHOLD_Pin GPIO_PIN_7
#define FLASH_nHOLD_GPIO_Port GPIOC
#define MRAM_nCS_Pin GPIO_PIN_8
#define MRAM_nCS_GPIO_Port GPIOC
#define MRAM_nWP_Pin GPIO_PIN_9
#define MRAM_nWP_GPIO_Port GPIOC
#define CAM_nCS_Pin GPIO_PIN_8
#define CAM_nCS_GPIO_Port GPIOA
#define CAM_FSH_Pin GPIO_PIN_9
#define CAM_FSH_GPIO_Port GPIOA
#define CAM_ON_Pin GPIO_PIN_10
#define CAM_ON_GPIO_Port GPIOA
#define WDI_Pin GPIO_PIN_11
#define WDI_GPIO_Port GPIOA
#define M_nRESET_Pin GPIO_PIN_12
#define M_nRESET_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
