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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>
#include "stdio.h"
#include "string.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01.h"
#include "mx25r6435f.h"
#include "stm32l4s5i_iot01_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: USE
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
#define SCORE_ADD 0
#define SETTINGS_ADD 50
#define MUSIC_ADD 100
#define VALUE_LIMIT 4000

#define NOTE_0_SIZE 300
#define NOTE_1_SIZE 300
#define NOTE_2_SIZE 1000
#define NOTE_3_SIZE 300
#define NOTE_4_SIZE 250
#define NOTE_5_SIZE 500

#define NOTE_0_ADD MUSIC_ADD
#define NOTE_1_ADD NOTE_0_ADD +  NOTE_0_SIZE														 *sizeof(uint32_t)
#define NOTE_2_ADD NOTE_0_ADD + (NOTE_0_SIZE + NOTE_1_SIZE)											 *sizeof(uint32_t)
#define NOTE_3_ADD NOTE_0_ADD + (NOTE_0_SIZE + NOTE_1_SIZE + NOTE_2_SIZE)							 *sizeof(uint32_t)
#define NOTE_4_ADD NOTE_0_ADD + (NOTE_0_SIZE + NOTE_1_SIZE + NOTE_2_SIZE + NOTE_3_SIZE)				 *sizeof(uint32_t)
#define NOTE_5_ADD NOTE_0_ADD + (NOTE_0_SIZE + NOTE_1_SIZE + NOTE_2_SIZE + NOTE_3_SIZE + NOTE_4_SIZE)*sizeof(uint32_t)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// TODO: USE
uint32_t song_index = 0;
uint8_t state = 0;
uint32_t note_size[6] = {NOTE_0_SIZE, NOTE_1_SIZE, NOTE_2_SIZE, NOTE_3_SIZE, NOTE_4_SIZE, NOTE_5_SIZE};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
// TODO: USE
void gen_music(void);
void gen_sine(void);
//uint32_t* gen_notes(uint16_t size);
void gen_notes(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TODO: USE
uint32_t note_0[NOTE_0_SIZE];
uint32_t note_1[NOTE_1_SIZE];
uint32_t note_2[NOTE_2_SIZE];
uint32_t note_3[NOTE_3_SIZE];
uint32_t note_4[NOTE_4_SIZE];
uint32_t note_5[NOTE_5_SIZE];


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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //===============================================================================================================
  //===============================================================================================================
  // TODO: USE

  BSP_QSPI_Init();

  HAL_GPIO_WritePin(errorLED_GPIO_Port, errorLED_Pin, GPIO_PIN_SET);

  if(BSP_QSPI_Erase_Block(0) != QSPI_OK){
	  Error_Handler();
  }


  gen_notes();

  gen_music();

//  *note = gen_notes(MUSIC_SIZE);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);

  //  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, music, MUSIC_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 305;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 40000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 750;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(errorLED_GPIO_Port, errorLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : errorLED_Pin */
  GPIO_InitStruct.Pin = errorLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(errorLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pushbutton_Pin */
  GPIO_InitStruct.Pin = pushbutton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pushbutton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : greenLED_Pin */
  GPIO_InitStruct.Pin = greenLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(greenLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// TODO: USE
void gen_music(void){

	if(BSP_QSPI_Read(note_0, NOTE_0_ADD, sizeof(uint32_t)*(NOTE_0_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_1, NOTE_1_ADD, sizeof(uint32_t)*(NOTE_1_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_2, NOTE_2_ADD, sizeof(uint32_t)*(NOTE_2_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_3, NOTE_3_ADD, sizeof(uint32_t)*(NOTE_3_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_4, NOTE_4_ADD, sizeof(uint32_t)*(NOTE_4_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_5, NOTE_5_ADD, sizeof(uint32_t)*(NOTE_5_SIZE))!= QSPI_OK){
		Error_Handler();
	}

}

//uint32_t* gen_notes(uint16_t size){
//
////	uint32_t *arr = (uint32_t *) malloc(size*sizeof(uint32_t));
//	uint32_t *arr = malloc(size*sizeof(uint32_t));
//	float theta = 0.0;
//	float val = 0.0;
//	for (int i = 0; i < size; i++){
//		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
//		arr[i] = (uint32_t)(val + (VALUE_LIMIT/2.0));
//		theta += (2*PI)/(size);
//		if (theta >= 2*PI){
//			theta = 0.0;
//		}
//	}
//	return arr;
//}

// TODO: USE
void gen_notes(void){

	uint32_t note_0_temp[NOTE_0_SIZE];
	uint32_t note_1_temp[NOTE_1_SIZE];
	uint32_t note_2_temp[NOTE_2_SIZE];
	uint32_t note_3_temp[NOTE_3_SIZE];
	uint32_t note_4_temp[NOTE_4_SIZE];
	uint32_t note_5_temp[NOTE_5_SIZE];

	float theta = 0.0;
	float val = 0.0;
	uint32_t size = NOTE_0_SIZE;
	for (int i = 0; i < size; i++){
		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
		note_0_temp[i] = (uint32_t)(val + (VALUE_LIMIT/2.0));
		theta += (2*PI)/(size);
	}
	theta = 0.0;
	val = 0.0;
	size = NOTE_1_SIZE;
	for (int i = 0; i < size; i++){
		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
		val = (uint32_t)(val + (VALUE_LIMIT/2.0));
		note_1_temp[i] = val;
		theta += (2*PI)/(size);
	}
	theta = 0.0;
	val = 0.0;
	size = NOTE_2_SIZE;
	for (int i = 0; i < size; i++){
		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
		note_2_temp[i] = (uint32_t)(val + (VALUE_LIMIT/2.0));
		theta += (2*PI)/(size);
	}
	theta = 0.0;
	val = 0.0;
	size = NOTE_3_SIZE;
	for (int i = 0; i < size; i++){
		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
		note_3_temp[i] = (uint32_t)(val + (VALUE_LIMIT/2.0));
		theta += (2*PI)/(size);
	}
	theta = 0.0;
	val = 0.0;
	size = NOTE_4_SIZE;
	for (int i = 0; i < size; i++){
		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
		note_4_temp[i] = (uint32_t)(val + (VALUE_LIMIT/2.0));
		theta += (2*PI)/(size);
	}
	theta = 0.0;
	val = 0.0;
	size = NOTE_5_SIZE;
	for (int i = 0; i < size; i++){
		val = (VALUE_LIMIT/2.0)*arm_sin_f32(theta);
		note_5_temp[i] = (uint32_t)(val + (VALUE_LIMIT/2.0));
		theta += (2*PI)/(size);
	}

	if(BSP_QSPI_Write(note_0_temp, NOTE_0_ADD, sizeof(uint32_t)*(NOTE_0_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_1_temp, NOTE_1_ADD, sizeof(uint32_t)*(NOTE_1_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_2_temp, NOTE_2_ADD, sizeof(uint32_t)*(NOTE_2_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_3_temp, NOTE_3_ADD, sizeof(uint32_t)*(NOTE_3_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_4_temp, NOTE_4_ADD, sizeof(uint32_t)*(NOTE_4_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_5_temp, NOTE_5_ADD, sizeof(uint32_t)*(NOTE_5_SIZE))!= QSPI_OK){
		Error_Handler();
	}
}

// TODO: USE
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	state = (state + 1)%2;
	if (state == 0){
		HAL_TIM_Base_Start_IT(&htim5);
	}
	else if(state == 1){
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim5);
	}
}

/* USER CODE END 4 */

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
	// TODO: USE
	if (htim == &htim5){
			HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);	// Stop playing looped audio
			switch(song_index) {
				case 0:
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, note_0, NOTE_0_SIZE, DAC_ALIGN_12B_R);
				     break;
				case 1:
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, note_1, NOTE_1_SIZE, DAC_ALIGN_12B_R);
				     break;
				case 2:
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, note_2, NOTE_2_SIZE, DAC_ALIGN_12B_R);
					break;
				case 3:
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, note_3, NOTE_3_SIZE, DAC_ALIGN_12B_R);
					break;
				case 4:
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, note_4, NOTE_4_SIZE, DAC_ALIGN_12B_R);
					break;
				case 5:
					HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, note_5, NOTE_5_SIZE, DAC_ALIGN_12B_R);
					break;
				default:

				 }
			song_index = (song_index + 1) % 6;
		}


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
// TODO: USE
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(errorLED_GPIO_Port, errorLED_Pin, GPIO_PIN_RESET);
	__BKPT();
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
