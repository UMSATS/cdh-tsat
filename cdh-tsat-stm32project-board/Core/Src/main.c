/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * AUTHORS:
  *  Daigh Burgess (daigh.burgess@umsats.ca)
  *   -Application Code
  *  Kyle James (kyle.james@umsats.ca)
  *   -RTC Interfacing
  *  Christina Duong (christina.duong@umsats.ca)
  *   -FreeRTOS Code
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include "W25N_driver.h"
#include "W25N_driver_test.h"
#include "AS3001204_driver.h"
#include "AS3001204_driver_test.h"
#include "LEDs_driver.h"
#include "MAX6822_driver.h"
#include "LTC1154_driver.h"
#include "can.h"
#include "telemetry.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

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
/* Definitions for canCmdHandler */
osThreadId_t canCmdHandlerHandle;
const osThreadAttr_t canCmdHandler_attributes = {
  .name = "canCmdHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for telemHandler */
osThreadId_t telemHandlerHandle;
const osThreadAttr_t telemHandler_attributes = {
  .name = "telemHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for timeTagTask */
osThreadId_t timeTagTaskHandle;
const osThreadAttr_t timeTagTask_attributes = {
  .name = "timeTagTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for stm32Reset */
osThreadId_t stm32ResetHandle;
const osThreadAttr_t stm32Reset_attributes = {
  .name = "stm32Reset",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for flashUnitTest */
osThreadId_t flashUnitTestHandle;
const osThreadAttr_t flashUnitTest_attributes = {
  .name = "flashUnitTest",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mramUnitTest */
osThreadId_t mramUnitTestHandle;
const osThreadAttr_t mramUnitTest_attributes = {
  .name = "mramUnitTest",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for deployA */
osThreadId_t deployAHandle;
const osThreadAttr_t deployA_attributes = {
  .name = "deployA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for deployB */
osThreadId_t deployBHandle;
const osThreadAttr_t deployB_attributes = {
  .name = "deployB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getTasksNum */
osThreadId_t getTasksNumHandle;
const osThreadAttr_t getTasksNum_attributes = {
  .name = "getTasksNum",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timeTagTaskInit */
osThreadId_t timeTagTaskInitHandle;
const osThreadAttr_t timeTagTaskInit_attributes = {
  .name = "timeTagTaskInit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for setRTC */
osThreadId_t setRTCHandle;
const osThreadAttr_t setRTC_attributes = {
  .name = "setRTC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getRTC */
osThreadId_t getRTCHandle;
const osThreadAttr_t getRTC_attributes = {
  .name = "getRTC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canQueue */
osMessageQueueId_t canQueueHandle;
const osMessageQueueAttr_t canQueue_attributes = {
  .name = "canQueue"
};
/* Definitions for telemQueue */
osMessageQueueId_t telemQueueHandle;
const osMessageQueueAttr_t telemQueue_attributes = {
  .name = "telemQueue"
};
/* Definitions for timeTagTaskInitQueue */
osMessageQueueId_t timeTagTaskInitQueueHandle;
const osMessageQueueAttr_t timeTagTaskInitQueue_attributes = {
  .name = "timeTagTaskInitQueue"
};
/* Definitions for setRTCQueue */
osMessageQueueId_t setRTCQueueHandle;
const osMessageQueueAttr_t setRTCQueue_attributes = {
  .name = "setRTCQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
void StartBlinkLED1(void *argument);
void StartBlinkLED2(void *argument);
void StartBlinkLED3(void *argument);
void StartToggleWDI(void *argument);
void StartCanCmdHandler(void *argument);
void StartTelemHandler(void *argument);
void StartTimeTagTask(void *argument);
void StartStm32Reset(void *argument);
void StartFlashUnitTest(void *argument);
void StartMramUnitTest(void *argument);
void StartDeployA(void *argument);
void StartDeployB(void *argument);
void StartGetTasksNum(void *argument);
void StartTimeTagTaskInit(void *argument);
void StartSetRTC(void *argument);
void StartGetRTC(void *argument);

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //disable the automatically enabled RTC alarm
  HAL_StatusTypeDef rtc_alarm_operation_status;
  rtc_alarm_operation_status = HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
  if (rtc_alarm_operation_status != HAL_OK) goto error;

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

  /* creation of telemQueue */
  telemQueueHandle = osMessageQueueNew (100, sizeof(TelemetryMessage_t), &telemQueue_attributes);

  /* creation of timeTagTaskInitQueue */
  timeTagTaskInitQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &timeTagTaskInitQueue_attributes);

  /* creation of setRTCQueue */
  setRTCQueueHandle = osMessageQueueNew (10, sizeof(CANMessage_t), &setRTCQueue_attributes);

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

  /* creation of canCmdHandler */
  canCmdHandlerHandle = osThreadNew(StartCanCmdHandler, NULL, &canCmdHandler_attributes);

  /* creation of telemHandler */
  telemHandlerHandle = osThreadNew(StartTelemHandler, NULL, &telemHandler_attributes);

  /* creation of timeTagTask */
  timeTagTaskHandle = osThreadNew(StartTimeTagTask, NULL, &timeTagTask_attributes);

  /* creation of stm32Reset */
  stm32ResetHandle = osThreadNew(StartStm32Reset, NULL, &stm32Reset_attributes);

  /* creation of flashUnitTest */
  flashUnitTestHandle = osThreadNew(StartFlashUnitTest, NULL, &flashUnitTest_attributes);

  /* creation of mramUnitTest */
  mramUnitTestHandle = osThreadNew(StartMramUnitTest, NULL, &mramUnitTest_attributes);

  /* creation of deployA */
  deployAHandle = osThreadNew(StartDeployA, NULL, &deployA_attributes);

  /* creation of deployB */
  deployBHandle = osThreadNew(StartDeployB, NULL, &deployB_attributes);

  /* creation of getTasksNum */
  getTasksNumHandle = osThreadNew(StartGetTasksNum, NULL, &getTasksNum_attributes);

  /* creation of timeTagTaskInit */
  timeTagTaskInitHandle = osThreadNew(StartTimeTagTaskInit, NULL, &timeTagTaskInit_attributes);

  /* creation of setRTC */
  setRTCHandle = osThreadNew(StartSetRTC, NULL, &setRTC_attributes);

  /* creation of getRTC */
  getRTCHandle = osThreadNew(StartGetRTC, NULL, &getRTC_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

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

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 2;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
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
  HAL_GPIO_WritePin(RELEASE_nEN_GPIO_Port, RELEASE_nEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELEASE_Pin|MRAM_nWP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UHF_nCS_Pin|FLASH_nCS_Pin|FLASH_nHOLD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UHF_SDN_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |FLASH_nWP_Pin|WDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MRAM_nCS_GPIO_Port, MRAM_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M_nRESET_GPIO_Port, M_nRESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RELEASE_nEN_Pin RELEASE_Pin MRAM_nWP_Pin */
  GPIO_InitStruct.Pin = RELEASE_nEN_Pin|RELEASE_Pin|MRAM_nWP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UHF_nCS_Pin FLASH_nCS_Pin FLASH_nHOLD_Pin */
  GPIO_InitStruct.Pin = UHF_nCS_Pin|FLASH_nCS_Pin|FLASH_nHOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UHF_nIRQ_Pin */
  GPIO_InitStruct.Pin = UHF_nIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UHF_nIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UHF_SDN_Pin LED1_Pin LED2_Pin LED3_Pin
                           FLASH_nWP_Pin WDI_Pin M_nRESET_Pin */
  GPIO_InitStruct.Pin = UHF_SDN_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |FLASH_nWP_Pin|WDI_Pin|M_nRESET_Pin;
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
//###############################################################################################
//Interrupt Service Routines
//###############################################################################################
/**
  * @brief  RTC alarm A callback
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *         the configuration information for the specified RTC.
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  osThreadFlagsSet(timeTagTaskHandle, 0x0001);
}

/**
  * @brief  CAN Rx Fifo 0 message pending callback
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
  osThreadExit();
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
  osThreadExit();
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
  osThreadExit();
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
  osThreadExit();
  /* USER CODE END StartToggleWDI */
}

/* USER CODE BEGIN Header_StartCanCmdHandler */
/**
* @brief Function implementing the canCmdHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanCmdHandler */
void StartCanCmdHandler(void *argument)
{
  /* USER CODE BEGIN StartCanCmdHandler */
  CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(canQueueHandle, &can_message, NULL, osWaitForever);
    switch (can_message.command)
    {
      case 0x40:
        osThreadFlagsSet(stm32ResetHandle, 0x0001);
        break;
      case 0x41:
        osThreadFlagsSet(flashUnitTestHandle, 0x0001);
        break;
      case 0x42:
        osThreadFlagsSet(mramUnitTestHandle, 0x0001);
        break;
      case 0x43:
        osThreadFlagsSet(deployAHandle, 0x0001);
        break;
      case 0x44:
        osThreadFlagsSet(deployBHandle, 0x0001);
        break;
      case 0x47:
        osThreadFlagsSet(getTasksNumHandle, 0x0001);
        break;
      case 0x48:
        osMessageQueuePut(timeTagTaskInitQueueHandle, &can_message, 0, 0);
        break;
      case 0x49:
        osMessageQueuePut(setRTCQueueHandle, &can_message, 0, 0);
        break;
      case 0x4A:
        osThreadFlagsSet(getRTCHandle, 0x0001);
        break;
      default:
        break;
    }
  }
  osThreadExit();
  /* USER CODE END StartCanCmdHandler */
}

/* USER CODE BEGIN Header_StartTelemHandler */
/**
* @brief Function implementing the telemHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemHandler */
void StartTelemHandler(void *argument)
{
  /* USER CODE BEGIN StartTelemHandler */
  TelemetryMessage_t telemetry_message;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(telemQueueHandle, &telemetry_message, NULL, osWaitForever);
    switch(telemetry_message.id)
    {
      //TODO: Implement telemetry handling
    }
  }
  osThreadExit();
  /* USER CODE END StartTelemHandler */
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
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from RTC alarm ISR
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    HAL_StatusTypeDef operation_status;
    CANMessage_t ack_message =
    {
      .priority = 0b0000111,
      .SenderID = 0x1,
      .DestinationID = 0x1,
      .command = 0x01,
      .data = {0x48, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}
    };

    operation_status = CAN_Transmit_Message(ack_message);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

error:
    if (operation_status != HAL_OK)
    {
      //TODO: Implement error handling for StartTimeTagTask
    }
  }
  osThreadExit();
  /* USER CODE END StartTimeTagTask */
}

/* USER CODE BEGIN Header_StartStm32Reset */
/**
* @brief Function implementing the stm32Reset thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStm32Reset */
void StartStm32Reset(void *argument)
{
  /* USER CODE BEGIN StartStm32Reset */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    MAX6822_Manual_Reset();
  }
  osThreadExit();
  /* USER CODE END StartStm32Reset */
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
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    W25N_StatusTypeDef test_result = Test_W25N();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
  /* USER CODE END StartFlashUnitTest */
}

/* USER CODE BEGIN Header_StartMramUnitTest */
/**
* @brief Function implementing the mramUnitTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMramUnitTest */
void StartMramUnitTest(void *argument)
{
  /* USER CODE BEGIN StartMramUnitTest */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    HAL_StatusTypeDef test_result = AS3001204_Test_MRAM_Driver();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
  /* USER CODE END StartMramUnitTest */
}

/* USER CODE BEGIN Header_StartDeployA */
/**
* @brief Function implementing the deployA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDeployA */
void StartDeployA(void *argument)
{
  /* USER CODE BEGIN StartDeployA */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    LTC1154_Enable();
  }
  osThreadExit();
  /* USER CODE END StartDeployA */
}

/* USER CODE BEGIN Header_StartDeployB */
/**
* @brief Function implementing the deployB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDeployB */
void StartDeployB(void *argument)
{
  /* USER CODE BEGIN StartDeployB */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    LTC1154_On();
  }
  osThreadExit();
  /* USER CODE END StartDeployB */
}

/* USER CODE BEGIN Header_StartGetTasksNum */
/**
* @brief Function implementing the getTasksNum thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetTasksNum */
void StartGetTasksNum(void *argument)
{
  /* USER CODE BEGIN StartGetTasksNum */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    uint8_t tasks_num = (uint8_t) osThreadGetCount();

    //TODO: Add CAN message transmit
  }
  osThreadExit();
  /* USER CODE END StartGetTasksNum */
}

/* USER CODE BEGIN Header_StartTimeTagTaskInit */
/**
* @brief Function implementing the timeTagTaskInit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTimeTagTaskInit */
void StartTimeTagTaskInit(void *argument)
{
  /* USER CODE BEGIN StartTimeTagTaskInit */
  CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osMessageQueueGet(timeTagTaskInitQueueHandle, &can_message, NULL, osWaitForever);

    HAL_StatusTypeDef operation_status;
    RTC_AlarmTypeDef rtc_alarm;
    uint32_t unix_timestamp = four_byte_array_to_uint32(can_message.data);
    RTC_TimeTypeDef rtc_time = unix_timestamp_to_rtc_time(unix_timestamp);
    RTC_DateTypeDef rtc_date = unix_timestamp_to_rtc_date(unix_timestamp);

    rtc_alarm.AlarmTime = rtc_time;
    rtc_alarm.AlarmMask = RTC_ALARMMASK_NONE;
    rtc_alarm.SubSeconds = 0;
    rtc_alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    rtc_alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    rtc_alarm.AlarmDateWeekDay = rtc_date.Date;
    rtc_alarm.Alarm = RTC_ALARM_A;

    operation_status = HAL_RTC_SetAlarm_IT(&hrtc, &rtc_alarm, RTC_FORMAT_BIN);

    //TODO: Implement error handling for StartTimeTagTaskInit
  }
  osThreadExit();
  /* USER CODE END StartTimeTagTaskInit */
}

/* USER CODE BEGIN Header_StartSetRTC */
/**
* @brief Function implementing the setRTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetRTC */
void StartSetRTC(void *argument)
{
  /* USER CODE BEGIN StartSetRTC */
  CANMessage_t can_message;
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osMessageQueueGet(setRTCQueueHandle, &can_message, NULL, osWaitForever);

    HAL_StatusTypeDef operation_status;
    uint32_t unix_timestamp = four_byte_array_to_uint32(can_message.data);
    RTC_TimeTypeDef rtc_time = unix_timestamp_to_rtc_time(unix_timestamp);
    RTC_DateTypeDef rtc_date = unix_timestamp_to_rtc_date(unix_timestamp);

    operation_status = HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

error:
    if (operation_status != HAL_OK)
    {
      //TODO: Implement error handling for StartSetRTC
    }
  }
  osThreadExit();
  /* USER CODE END StartSetRTC */
}

/* USER CODE BEGIN Header_StartGetRTC */
/**
* @brief Function implementing the getRTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetRTC */
void StartGetRTC(void *argument)
{
  /* USER CODE BEGIN StartGetRTC */
  /* Infinite loop */
  for(;;)
  {
    //block until thread resumed from command handler
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);

    HAL_StatusTypeDef operation_status;
    RTC_TimeTypeDef rtc_time;
    RTC_DateTypeDef rtc_date;
    uint32_t unix_timestamp;
    uint8_t response_data[6] = {0,0,0,0,0,0};

    operation_status = HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    if (operation_status != HAL_OK) goto error;
    operation_status = HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
    if (operation_status != HAL_OK) goto error;

    unix_timestamp = rtc_to_unix_timestamp(rtc_time, rtc_date);
    uint32_to_four_byte_array(unix_timestamp, response_data);

    //TODO: Add CAN message transmit

error:
    if (operation_status != HAL_OK)
    {
      //TODO: Implement error handling for StartGetRTC
    }
  }
  osThreadExit();
  /* USER CODE END StartGetRTC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
