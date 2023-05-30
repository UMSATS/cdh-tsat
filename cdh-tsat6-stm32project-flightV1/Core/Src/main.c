/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "W25N_driver.h"
#include "W25N_driver_test.h"
#include "AS3001204_driver.h"
#include "AS3001204_driver_test.h"
#include "LEDs_driver.h"
#include "MAX6822_driver.h"
#include "LTC1154_driver.h"
#include "Si4464_driver.h"
#include "Si4464_driver_test.h"

#include "can.h"
#include "ax25.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// These numbers are fairly arbitarily defined. however, we can figure out
// some mathematically-sound numbers based off of the max bitstuffed length and
// going backwards from there. -NJR
#define AX25_SCRATCH_SPACE_LEN 18
#define AX25_MESSAGE_MAX_LEN 32
#define AX25_OUTPUT_MAX_LEN 63
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;

/* Definitions for blinkLED1 */
osThreadId_t blinkLED1Handle;
const osThreadAttr_t blinkLED1_attributes = {
  .name = "blinkLED1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blinkLED2 */
osThreadId_t blinkLED2Handle;
const osThreadAttr_t blinkLED2_attributes = {
  .name = "blinkLED2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blinkLED3 */
osThreadId_t blinkLED3Handle;
const osThreadAttr_t blinkLED3_attributes = {
  .name = "blinkLED3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for toggleWDI */
osThreadId_t toggleWDIHandle;
const osThreadAttr_t toggleWDI_attributes = {
  .name = "toggleWDI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commandHandler */
osThreadId_t commandHandlerHandle;
const osThreadAttr_t commandHandler_attributes = {
  .name = "commandHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for flashUnitTest */
osThreadId_t flashUnitTestHandle;
const osThreadAttr_t flashUnitTest_attributes = {
  .name = "flashUnitTest",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mramUnitTest */
osThreadId_t mramUnitTestHandle;
const osThreadAttr_t mramUnitTest_attributes = {
  .name = "mramUnitTest",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for enableRelease */
osThreadId_t enableReleaseHandle;
const osThreadAttr_t enableRelease_attributes = {
  .name = "enableRelease",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for executeRelease */
osThreadId_t executeReleaseHandle;
const osThreadAttr_t executeRelease_attributes = {
  .name = "executeRelease",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for takePicture */
osThreadId_t takePictureHandle;
const osThreadAttr_t takePicture_attributes = {
  .name = "takePicture",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendUHFBeacon */
osThreadId_t sendUHFBeaconHandle;
const osThreadAttr_t sendUHFBeacon_attributes = {
  .name = "sendUHFBeacon",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getTaskList */
osThreadId_t getTaskListHandle;
const osThreadAttr_t getTaskList_attributes = {
  .name = "getTaskList",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timeTagTask */
osThreadId_t timeTagTaskHandle;
const osThreadAttr_t timeTagTask_attributes = {
  .name = "timeTagTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canQueue */
osMessageQueueId_t canQueueHandle;
const osMessageQueueAttr_t canQueue_attributes = {
  .name = "canQueue"
};
/* Definitions for flashUnitTestQueue */
osMessageQueueId_t flashUnitTestQueueHandle;
const osMessageQueueAttr_t flashUnitTestQueue_attributes = {
  .name = "flashUnitTestQueue"
};
/* Definitions for mramUnitTestQueue */
osMessageQueueId_t mramUnitTestQueueHandle;
const osMessageQueueAttr_t mramUnitTestQueue_attributes = {
  .name = "mramUnitTestQueue"
};
/* Definitions for releaseEnableQueue */
osMessageQueueId_t releaseEnableQueueHandle;
const osMessageQueueAttr_t releaseEnableQueue_attributes = {
  .name = "releaseEnableQueue"
};
/* Definitions for releaseExecuteQueue */
osMessageQueueId_t releaseExecuteQueueHandle;
const osMessageQueueAttr_t releaseExecuteQueue_attributes = {
  .name = "releaseExecuteQueue"
};
/* Definitions for takePictureQueue */
osMessageQueueId_t takePictureQueueHandle;
const osMessageQueueAttr_t takePictureQueue_attributes = {
  .name = "takePictureQueue"
};
/* Definitions for sendUHFQueue */
osMessageQueueId_t sendUHFQueueHandle;
const osMessageQueueAttr_t sendUHFQueue_attributes = {
  .name = "sendUHFQueue"
};
/* Definitions for getTaskListQueue */
osMessageQueueId_t getTaskListQueueHandle;
const osMessageQueueAttr_t getTaskListQueue_attributes = {
  .name = "getTaskListQueue"
};
/* Definitions for timeTagQueue */
osMessageQueueId_t timeTagQueueHandle;
const osMessageQueueAttr_t timeTagQueue_attributes = {
  .name = "timeTagQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
void StartBlinkLED1(void *argument);
void StartBlinkLED2(void *argument);
void StartBlinkLED3(void *argument);
void StartToggleWDI(void *argument);
void StartCommandHandler(void *argument);
void StartFlashUnitTest(void *argument);
void StartMRAMUnitTest(void *argument);
void StartEnableRelease(void *argument);
void StartExecuteRelease(void *argument);
void StartTakePicture(void *argument);
void StartSendUHFBeacon(void *argument);
void StartGetTaskList(void *argument);
void StartTimeTagTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  MAX6822_Init();

  LEDs_Init();

  LTC1154_Init();

  HAL_StatusTypeDef can_operation_status;
  can_operation_status = CAN_Init();
  if (can_operation_status != HAL_OK) goto error;

  W25N_StatusTypeDef w25n_operation_status;
  w25n_operation_status = W25N_Init();
  if (w25n_operation_status != W25N_HAL_OK) goto error;

  HAL_StatusTypeDef as3001204_operation_status;
  as3001204_operation_status = AS3001204_Init();
  if (as3001204_operation_status != HAL_OK) goto error;

  Si4464_StatusTypeDef si4464_operation_status;
  Si4464_Reset_Device();
  Si4464_Get_CTS(); // Doesn't pass errors via return value.
  si4464_operation_status = Si4464_Init_Device();
  if (si4464_operation_status != SI4464_HAL_OK) goto error;
  si4464_operation_status = Si4464_Set_One_Prop(0x12, 0x06, 0x01);
  if (si4464_operation_status != SI4464_HAL_OK) goto error;



  //this code performs the W25N unit tests
  //this code should be completed after power cycling the W25N
  /*w25n_operation_status = Test_W25N();
  if (w25n_operation_status != W25N_HAL_OK) goto error;
  w25n_operation_status = W25N_Reset_And_Init();
  if (w25n_operation_status != W25N_HAL_OK) goto error;*/

  //this code performs the AS3001204 unit tests
  //this code should be completed after power cycling the AS3001204
  /*as3001204_operation_status = AS3001204_Test_MRAM_Driver();
  if (as3001204_operation_status != HAL_OK) goto error;
  as3001204_operation_status = AS3001204_Init();
  if (as3001204_operation_status != HAL_OK) goto error;*/

  // This code performs the Si4464 unit tests.
  // This code should be completed after power cycling the Si4464.
  si4464_operation_status = Test_Si4464();
  if (si4464_operation_status != SI4464_HAL_OK) goto error;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canQueue */
  canQueueHandle = osMessageQueueNew (100, sizeof(CANMessage_t), &canQueue_attributes);

  /* creation of flashUnitTestQueue */
  flashUnitTestQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &flashUnitTestQueue_attributes);

  /* creation of mramUnitTestQueue */
  mramUnitTestQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &mramUnitTestQueue_attributes);

  /* creation of releaseEnableQueue */
  releaseEnableQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &releaseEnableQueue_attributes);

  /* creation of releaseExecuteQueue */
  releaseExecuteQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &releaseExecuteQueue_attributes);

  /* creation of takePictureQueue */
  takePictureQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &takePictureQueue_attributes);

  /* creation of sendUHFQueue */
  sendUHFQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &sendUHFQueue_attributes);

  /* creation of getTaskListQueue */
  getTaskListQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &getTaskListQueue_attributes);

  /* creation of timeTagQueue */
  timeTagQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &timeTagQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinkLED1 */
  blinkLED1Handle = osThreadNew(StartBlinkLED1, NULL, &blinkLED1_attributes);

  /* creation of blinkLED2 */
  blinkLED2Handle = osThreadNew(StartBlinkLED2, NULL, &blinkLED2_attributes);

  /* creation of blinkLED3 */
  blinkLED3Handle = osThreadNew(StartBlinkLED3, NULL, &blinkLED3_attributes);

  /* creation of toggleWDI */
  toggleWDIHandle = osThreadNew(StartToggleWDI, NULL, &toggleWDI_attributes);

  /* creation of commandHandler */
  commandHandlerHandle = osThreadNew(StartCommandHandler, NULL, &commandHandler_attributes);

  /* creation of flashUnitTest */
  flashUnitTestHandle = osThreadNew(StartFlashUnitTest, NULL, &flashUnitTest_attributes);

  /* creation of mramUnitTest */
  mramUnitTestHandle = osThreadNew(StartMRAMUnitTest, NULL, &mramUnitTest_attributes);

  /* creation of enableRelease */
  enableReleaseHandle = osThreadNew(StartEnableRelease, NULL, &enableRelease_attributes);

  /* creation of executeRelease */
  executeReleaseHandle = osThreadNew(StartExecuteRelease, NULL, &executeRelease_attributes);

  /* creation of takePicture */
  takePictureHandle = osThreadNew(StartTakePicture, NULL, &takePicture_attributes);

  /* creation of sendUHFBeacon */
  sendUHFBeaconHandle = osThreadNew(StartSendUHFBeacon, NULL, &sendUHFBeacon_attributes);

  /* creation of getTaskList */
  getTaskListHandle = osThreadNew(StartGetTaskList, NULL, &getTaskList_attributes);

  /* creation of timeTagTask */
  timeTagTaskHandle = osThreadNew(StartTimeTagTask, NULL, &timeTagTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

error:
  exit(1);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WDI_Pin|UHF_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M_nRESET_Pin|UHF_nCS_Pin|FLASH_nCS_Pin|FLASH_nHOLD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAM_FSH_Pin|CAM_ON_Pin|MRAM_nWP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|FLASH_nWP_Pin
                          |RELEASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MRAM_nCS_GPIO_Port, MRAM_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELEASE_nEN_GPIO_Port, RELEASE_nEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : WDI_Pin M_nRESET_Pin UHF_nCS_Pin FLASH_nCS_Pin
                           FLASH_nHOLD_Pin UHF_SDN_Pin */
  GPIO_InitStruct.Pin = WDI_Pin|M_nRESET_Pin|UHF_nCS_Pin|FLASH_nCS_Pin
                          |FLASH_nHOLD_Pin|UHF_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_FSH_Pin CAM_ON_Pin MRAM_nWP_Pin */
  GPIO_InitStruct.Pin = CAM_FSH_Pin|CAM_ON_Pin|MRAM_nWP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UHF_nIRQ_Pin */
  GPIO_InitStruct.Pin = UHF_nIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UHF_nIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin FLASH_nWP_Pin
                           RELEASE_nEN_Pin RELEASE_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|FLASH_nWP_Pin
                          |RELEASE_nEN_Pin|RELEASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MRAM_nCS_Pin */
  GPIO_InitStruct.Pin = MRAM_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MRAM_nCS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    HAL_StatusTypeDef operation_status;
    operation_status = CAN_Message_Received();
    if (operation_status != HAL_OK)
    {
        //TODO: Implement error handling for CAN message receives
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkLED1 */
/**
  * @brief  Function implementing the blinkLED1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkLED1 */
void StartBlinkLED1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    LED1_Toggle();
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlinkLED2 */
/**
* @brief Function implementing the blinkLED2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkLED2 */
void StartBlinkLED2(void *argument)
{
  /* USER CODE BEGIN StartBlinkLED2 */
  /* Infinite loop */
  for(;;)
  {
    LED2_Toggle();
    osDelay(500);
  }
  /* USER CODE END StartBlinkLED2 */
}

/* USER CODE BEGIN Header_StartBlinkLED3 */
/**
* @brief Function implementing the blinkLED3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkLED3 */
void StartBlinkLED3(void *argument)
{
  /* USER CODE BEGIN StartBlinkLED3 */
  /* Infinite loop */
  for(;;)
  {
    LED3_Toggle();
    osDelay(250);
  }
  /* USER CODE END StartBlinkLED3 */
}

/* USER CODE BEGIN Header_StartToggleWDI */
/**
* @brief Function implementing the toggleWDI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartToggleWDI */
void StartToggleWDI(void *argument)
{
  /* USER CODE BEGIN StartToggleWDI */
  /* Infinite loop */
  for(;;)
  {
    MAX6822_WDI_Toggle();
    osDelay(100);
  }
  /* USER CODE END StartToggleWDI */
}

/* USER CODE BEGIN Header_StartCommandHandler */
/**
* @brief Function implementing the commandHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandHandler */
void StartCommandHandler(void *argument)
{
  /* USER CODE BEGIN StartCommandHandler */
    CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(canQueueHandle, &can_message, NULL, osWaitForever);
    switch (can_message.command) {
    case 0x41:
        osMessageQueuePut(flashUnitTestQueueHandle, &can_message, 0, 200);
        break;
    case 0x42:
        osMessageQueuePut(mramUnitTestQueueHandle, &can_message, 0, 200);
        break;
    case 0x43:
        osMessageQueuePut(releaseEnableQueueHandle, &can_message, 0, 200);
        break;
    case 0x44:
        osMessageQueuePut(releaseExecuteQueueHandle, &can_message, 0, 200);
        break;
    case 0x45:
        osMessageQueuePut(takePictureQueueHandle, &can_message, 0, 200);
        break;
    case 0x46:
        osMessageQueuePut(sendUHFQueueHandle, &can_message, 0, 200);
        break;
    case 0x47:
        osMessageQueuePut(getTaskListQueueHandle, &can_message, 0, 200);
        break;
    case 0x48:
        osMessageQueuePut(timeTagQueueHandle, &can_message, 0, 200);
        break;
    default:
        break;
    }
  }
  /* USER CODE END StartCommandHandler */
}

/* USER CODE BEGIN Header_StartFlashUnitTest */
/**
* @brief Function implementing the flashUnitTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlashUnitTest */
void StartFlashUnitTest(void *argument)
{
  /* USER CODE BEGIN StartFlashUnitTest */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(flashUnitTestQueueHandle, &can_message, NULL, osWaitForever);
  }
  /* USER CODE END StartFlashUnitTest */
}

/* USER CODE BEGIN Header_StartMRAMUnitTest */
/**
* @brief Function implementing the mramUnitTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMRAMUnitTest */
void StartMRAMUnitTest(void *argument)
{
  /* USER CODE BEGIN StartMRAMUnitTest */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(mramUnitTestQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartMRAMUnitTest */
}

/* USER CODE BEGIN Header_StartEnableRelease */
/**
* @brief Function implementing the enableRelease thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEnableRelease */
void StartEnableRelease(void *argument)
{
  /* USER CODE BEGIN StartEnableRelease */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(releaseEnableQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartEnableRelease */
}

/* USER CODE BEGIN Header_StartExecuteRelease */
/**
* @brief Function implementing the executeRelease thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartExecuteRelease */
void StartExecuteRelease(void *argument)
{
  /* USER CODE BEGIN StartExecuteRelease */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(releaseExecuteQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartExecuteRelease */
}

/* USER CODE BEGIN Header_StartTakePicture */
/**
* @brief Function implementing the takePicture thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTakePicture */
void StartTakePicture(void *argument)
{
  /* USER CODE BEGIN StartTakePicture */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(takePictureQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartTakePicture */
}

/* USER CODE BEGIN Header_StartSendUHFBeacon */
/**
* @brief Function implementing the sendUHFBeacon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendUHFBeacon */
void StartSendUHFBeacon(void *argument)
{
  /* USER CODE BEGIN StartSendUHFBeacon */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(sendUHFQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartSendUHFBeacon */
}

/* USER CODE BEGIN Header_StartGetTaskList */
/**
* @brief Function implementing the getTaskList thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetTaskList */
void StartGetTaskList(void *argument)
{
  /* USER CODE BEGIN StartGetTaskList */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(getTaskListQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartGetTaskList */
}

/* USER CODE BEGIN Header_StartTimeTagTask */
/**
* @brief Function implementing the timeTagTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTimeTagTask */
void StartTimeTagTask(void *argument)
{
  /* USER CODE BEGIN StartTimeTagTask */
    CANMessage_t can_message;
    uint8_t response_data[6] = {0,0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
      osMessageQueueGet(timeTagQueueHandle, &can_message, NULL, osWaitForever);
      CAN_Send_Default_ACK_With_Data(can_message, response_data);
  }
  /* USER CODE END StartTimeTagTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
