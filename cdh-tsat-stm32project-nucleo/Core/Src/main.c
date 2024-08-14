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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include "camera_driver.h"
#include "camera_driver_test.h"
#include "W25N_driver.h"
#include "W25N_driver_test.h"
#include "AS3001204_driver.h"
#include "AS3001204_driver_test.h"
#include "LEDs_driver.h"
#include "MAX6822_driver.h"
#include "LTC1154_driver.h"
#include "can.h"
#include "SP-L2_driver.h"
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
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radioTxTask */
osThreadId_t radioTxTaskHandle;
const osThreadAttr_t radioTxTask_attributes = {
  .name = "radioTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radioRxTask */
osThreadId_t radioRxTaskHandle;
const osThreadAttr_t radioRxTask_attributes = {
  .name = "radioRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radioHandler */
osThreadId_t radioHandlerHandle;
const osThreadAttr_t radioHandler_attributes = {
  .name = "radioHandler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radioISR */
osThreadId_t radioISRHandle;
const osThreadAttr_t radioISR_attributes = {
  .name = "radioISR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for radioTxQueue */
osMessageQueueId_t radioTxQueueHandle;
const osMessageQueueAttr_t radioTxQueue_attributes = {
  .name = "radioTxQueue"
};
/* Definitions for radioRxQueue */
osMessageQueueId_t radioRxQueueHandle;
const osMessageQueueAttr_t radioRxQueue_attributes = {
  .name = "radioRxQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);
void StartRadioTxTask(void *argument);
void StartRadioRxTask(void *argument);
void StartRadioHandler(void *argument);
void StartRadioISR(void *argument);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  //###############################################################################################
  //Peripheral Initialization
  //###############################################################################################

  //this code initializes the MAX6822
  /*MAX6822_Init();*/

  //this code initializes the LEDs
  /*LEDs_Init();*/

  //this code initializes the LTC1154
  /*LTC1154_Init();*/

  //this code initializes the CAN Bus
  /*HAL_StatusTypeDef can_operation_status;
  can_operation_status = CAN_Init();
  if (can_operation_status != HAL_OK) goto error;*/

  //this code initializes the W25N
  /*W25N_StatusTypeDef w25n_operation_status;
  w25n_operation_status = W25N_Init();
  if (w25n_operation_status != W25N_HAL_OK) goto error;*/

  //this code initializes the AS3001204
  /*HAL_StatusTypeDef as3001204_operation_status;
  as3001204_operation_status = AS3001204_Init();
  if (as3001204_operation_status != HAL_OK) goto error;*/

  //this code initializes the piCAM
  /*piCAM_StatusTypeDef piCAM_operation_status;
  piCAM_operation_status = piCAM_Init();
  if (piCAM_operation_status != piCAM_HAL_OK) goto error;*/

  //###############################################################################################
  //Peripheral Unit Tests
  //###############################################################################################

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

  //this code performs the piCAM unit test
  //NOTE: piCAM_Test_Procedure(); will NOT return a HAL_StatusTypeDef as it is a void type
  // It ONLY follows this specific sequence
  // - Follows Boot Up Sequence
  // - HAL Delay of 3000ms
  // - Sends "t\0" over UART4 for test string
  // - HAL Delay of 3000ms
  // - Sends "d\0" over UART4 for image capture request, then immediately after
  //   we start DMA for specific image data.
  /*piCAM_Test_Procedure();*/

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
  /* creation of radioTxQueue */
  radioTxQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &radioTxQueue_attributes);

  /* creation of radioRxQueue */
  radioRxQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &radioRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of radioTxTask */
  radioTxTaskHandle = osThreadNew(StartRadioTxTask, NULL, &radioTxTask_attributes);

  /* creation of radioRxTask */
  radioRxTaskHandle = osThreadNew(StartRadioRxTask, NULL, &radioRxTask_attributes);

  /* creation of radioHandler */
  radioHandlerHandle = osThreadNew(StartRadioHandler, NULL, &radioHandler_attributes);

  /* creation of radioISR */
  radioISRHandle = osThreadNew(StartRadioISR, NULL, &radioISR_attributes);

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
  SpiritBaseConfiguration();
  uint8_t readyCommand = 0x62;
  uint8_t txCommand = 0x60;
  uint8_t txFIFOFlush = 0x72;
  uint8_t rxCommand = 0x61;
  uint8_t rxFlush = 0x71;
  uint8_t receivedFIFOSize = 0;
  while (1)
  {
	  uint8_t receivedFIFOSize = 0;
	  uint8_t txFIFOSize = 0;
	  uint8_t message[6] = {0}; // Test Message is "Hello!"


	  // Only Test RX or TX with this code, they use same message var.
	  /*
	   * TEST CODE FOR TX
	  message[0] = 0x48;
	  message[1] = 0x65;
	  message[2] = 0x6C;
	  message[3] = 0x6C;
	  message[4] = 0x6F;
	  message[5] = 0X21;
	  S2LP_Send_Command(readyCommand);
	  S2LP_Check_TX_FIFO_Status(&txFIFOSize);
	  S2LP_Write_TX_Fifo(6, &message);
	  S2LP_Check_TX_FIFO_Status(&txFIFOSize);
	  S2LP_Send_Command(txCommand);
	  HAL_Delay(1000);
	  S2LP_Check_TX_FIFO_Status(&txFIFOSize);
	  S2LP_Send_Command(txCommand);
	  HAL_Delay(1000);
	*/

	  /*
	   * TEST CODE FOR RX
	  S2LP_Send_Command(rxCommand);
	  // Temporary Test without IRQ seup
	  while(receivedFIFOSize == 0){
		  S2LP_Check_RX_FIFO_Status(&receivedFIFOSize);
		  HAL_Delay(3000);
	  }
	  S2LP_Read_RX_FIFO(6, &message);
	  S2LP_Send_Command(readyCommand);
	  S2LP_Send_Command(rxFlush);
	  */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD4_Pin|CAM_FSH_Pin|CAM_ON_Pin|WDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELEASE_Pin|FLASH_nWP_Pin|MRAM_nWP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELEASE_nEN_Pin|UHF_nCS_Pin|FLASH_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UHF_SDN_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLASH_nHOLD_Pin|MRAM_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M_nRESET_GPIO_Port, M_nRESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin CAM_FSH_Pin CAM_ON_Pin WDI_Pin
                           M_nRESET_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|CAM_FSH_Pin|CAM_ON_Pin|WDI_Pin
                          |M_nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELEASE_Pin FLASH_nWP_Pin FLASH_nHOLD_Pin MRAM_nCS_Pin
                           MRAM_nWP_Pin */
  GPIO_InitStruct.Pin = RELEASE_Pin|FLASH_nWP_Pin|FLASH_nHOLD_Pin|MRAM_nCS_Pin
                          |MRAM_nWP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RELEASE_nEN_Pin UHF_nCS_Pin UHF_SDN_Pin FLASH_nCS_Pin
                           LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = RELEASE_nEN_Pin|UHF_nCS_Pin|UHF_SDN_Pin|FLASH_nCS_Pin
                          |LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UHF_nIRQ_Pin */
  GPIO_InitStruct.Pin = UHF_nIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UHF_nIRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  UART Rx message pending callback
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *         the configuration information for the specified UART.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    piCAM_Receive_Check();
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

/**
 * Temporary GPIO external interrupt callback for development
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// S2-LP interrupts are set on PC0
  if(GPIO_Pin == GPIO_PIN_0)
  {
  	xTaskNotifyGive(radioISRHandle);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRadioTxTask */
/**
* @brief Function implementing the radioTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadioTxTask */
void StartRadioTxTask(void *argument)
{
  /* USER CODE BEGIN StartRadioTxTask */
	uint32_t freeBytes = 0;
  /* Infinite loop */
  for(;;)
  {
  	// Wait for handler to unblock
  	xTaskNotifyWait(0x00, 0xffffffff, &freeBytes, portMAX_DELAY);

  	// TODO: Write bytes equal to freeBytes to the FIFO

  	// Notify the handler of completion
  	xTaskNotifyGive(radioHandlerHandle);
  }
  /* USER CODE END StartRadioTxTask */
}

/* USER CODE BEGIN Header_StartRadioRxTask */
/**
* @brief Function implementing the radioRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadioRxTask */
void StartRadioRxTask(void *argument)
{
  /* USER CODE BEGIN StartRadioRxTask */
  uint32_t rxFifoLength = 0;
  /* Infinite loop */
  for(;;)
  {
  	// Wait for handler to unblock
  	xTaskNotifyWait(0x00, 0xffffffff, &rxFifoLength, portMAX_DELAY);

  	// TODO: Read bytes equal to rxFifoLength

  	// Notify the handler of completion
  	xTaskNotifyGive(radioHandlerHandle);
  }
  /* USER CODE END StartRadioRxTask */
}

/* USER CODE BEGIN Header_StartRadioHandler */
/**
* @brief Function implementing the radioHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadioHandler */
void StartRadioHandler(void *argument)
{
  /* USER CODE BEGIN StartRadioHandler */
	// Need to check this value, and declare it in the driver instead
	const uint8_t TX_FIFO_MAX_LENGTH = 32;
	const TickType_t xMaxWaitTime = pdMS_TO_TICKS(10000);
	TickType_t xStartTime;
	S2LP_StatusTypeDef status = S2LP_HAL_OK;
  S2LP_STATE state;
  uint8_t txFifoLength, rxFifoLength;
  uint32_t txTaskResult, rxTaskResult;
  /* Infinite loop */
  for(;;)
  {
  	/*
  	 * The radio should be in READY state here.
  	 * The TX and RX tasks are unblocked and move data
  	 * in/out of the radio FIFOs while the handler
  	 * waits for both to complete or timeout.
  	 */

		// Unblock the TX task
		status = S2LP_Check_TX_FIFO_Status(&txFifoLength);
		int8_t txFreeBytes = TX_FIFO_MAX_LENGTH - txFifoLength;
		xTaskNotify(radioTxTaskHandle, txFreeBytes, eSetValueWithOverwrite);

		if (status != S2LP_HAL_OK) { /*TODO*/ }

		// Unblock the RX task
		status = S2LP_Check_RX_FIFO_Status(&rxFifoLength);
		xTaskNotify(radioRxTaskHandle, rxFifoLength, eSetValueWithOverwrite);

		if (status != S2LP_HAL_OK) { /*TODO*/ }

		// Record the start time
		xStartTime = xTaskGetTickCount();

		// Wait for the tasks to complete or timeout
		/* TODO: We can't assume the TX task will complete first,
		 * using notification indexes will solve that */
		txTaskResult = ulTaskNotifyTake(pdTRUE, xMaxWaitTime);
		rxTaskResult = ulTaskNotifyTake(pdTRUE, xMaxWaitTime - (xTaskGetTickCount() - xStartTime));

		// TODO: Handle timeouts
		if (txTaskResult == 0) {}
		if (rxTaskResult == 0) {}

		/*
		 * At this point, the handler can wait for appropriate conditions**.
		 * It will then send either the TX or RX command, and wait to be
		 * notified by the ISR on completion.
		 *
		 * **TODO: Determine what these conditions are
		*/

		// If we want to transmit
		status = S2LP_Send_Command(COMMAND_TX);
		if (status == S2LP_HAL_OK)
		{
			state = S2LP_STATE_TX;
		}

		// Else if we want to receive
		status = S2LP_Send_Command(COMMAND_RX);
		if (status == S2LP_HAL_OK)
		{
			state = S2LP_STATE_RX;
		}

		// Block until radio is ready again
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		state = S2LP_STATE_READY;
  }
  /* USER CODE END StartRadioHandler */
}

/* USER CODE BEGIN Header_StartRadioISR */
/**
* @brief Function implementing the radioISR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadioISR */
void StartRadioISR(void *argument)
{
  /* USER CODE BEGIN StartRadioISR */
  /* Infinite loop */
  for(;;)
  {
  	// Wait for notification from HAL_GPIO_EXTI_CALLBACK
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Determine interrupt from status registers
    uint8_t irq_status0, irq_status1, irq_status2, irq_status3;
    S2LP_Spi_Read_Registers(0xFD, 1, &irq_status0);
    S2LP_Spi_Read_Registers(0xFC, 1, &irq_status1);
    S2LP_Spi_Read_Registers(0xFB, 1, &irq_status2);
    S2LP_Spi_Read_Registers(0xFA, 1, &irq_status3);

    /* Would this do the same thing?
     * S2LP_Spi_Read_Registers(0xFA, 4, &irq_val) */

    uint32_t irq_val =
    		irq_status0
			+ irq_status1
			+ irq_status2
			+ irq_status3;

    // Handle interrupts
    if (irq_val & IRQ_RX_DATA_READY
     || irq_val & IRQ_TX_DATA_SENT)
    {
    	xTaskNotifyGive(radioHandlerHandle);
    }

  }
  /* USER CODE END StartRadioISR */
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
