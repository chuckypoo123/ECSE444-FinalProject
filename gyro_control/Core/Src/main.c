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
#include "stdio.h"
#include "string.h"
#include "time.h"
#include "stm32l475e_iot01_qspi.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>
//#include "stm32l475e_iot01_gyro.h"
//#include "stm32l475e_iot01_accelero.h"
//#include "stm32l475e_iot01.h"
//#include "mx25r6435f.h"
//#include "stm32l475e_iot01_qspi.h"
//#include "JOYSTICK.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALSE 0
#define TRUE 1

#define N_COLS 5
#define N_ROWS 10

#define JOYSTICK1 0

#define SCORE_ADDR (1 << 16)
#define SETTINGS_ADDR 50
#define MUSIC_ADDR 100
#define VALUE_LIMIT 4000

#define NOTE_0_SIZE 300
#define NOTE_1_SIZE 400
#define NOTE_2_SIZE 800
#define NOTE_3_SIZE 300
#define NOTE_4_SIZE 200
#define NOTE_5_SIZE 500

#define NOTE_0_ADDR MUSIC_ADDR
#define NOTE_1_ADDR NOTE_0_ADDR +  NOTE_0_SIZE														 *sizeof(uint32_t)
#define NOTE_2_ADDR NOTE_0_ADDR + (NOTE_0_SIZE + NOTE_1_SIZE)											 *sizeof(uint32_t)
#define NOTE_3_ADDR NOTE_0_ADDR + (NOTE_0_SIZE + NOTE_1_SIZE + NOTE_2_SIZE)							 *sizeof(uint32_t)
#define NOTE_4_ADDR NOTE_0_ADDR + (NOTE_0_SIZE + NOTE_1_SIZE + NOTE_2_SIZE + NOTE_3_SIZE)				 *sizeof(uint32_t)
#define NOTE_5_ADDR NOTE_0_ADDR + (NOTE_0_SIZE + NOTE_1_SIZE + NOTE_2_SIZE + NOTE_3_SIZE + NOTE_4_SIZE)*sizeof(uint32_t)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//float gyro[3];
char str_hum[100] = "";
//int16_t accel[3];

volatile uint16_t joystickXY[2]; //12 bit ADC so we use 16 bit unsigned ints
const int num_inputs = 2;
volatile int conversion_cmplt = 0; //set by the call back
enum state{
	straight, left, right
};

volatile int check_dir = 1;

uint32_t song_index = 0;
uint8_t state = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void gen_music(void);
void gen_sine(void);
//uint32_t* gen_notes(uint16_t size);
void gen_notes(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char clear_screen[6] = {0x1B, 0x5B, 0x32, 0x4A, 0x00, 0x0d};

int score = 0;
float collision = 0;

uint32_t note_0[NOTE_0_SIZE];
uint32_t note_1[NOTE_1_SIZE];
uint32_t note_2[NOTE_2_SIZE];
uint32_t note_3[NOTE_3_SIZE];
uint32_t note_4[NOTE_4_SIZE];
uint32_t note_5[NOTE_5_SIZE];

// TODO: Make variables names more descriptive
volatile uint8_t char_horz_pos = 5;
uint8_t char_vert_pos = 1;
char map[N_ROWS][2 * N_COLS +1 +2 +1] = { // +1 (last vert pipe) +2 (newline) +1 (stop char)
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
    "| | | | | |\r\n",
};

char score_string[2 * N_COLS +2] = "";



uint8_t top_row = 0;

uint8_t expected_lane = 5; // Character starts in lane 5

void update_map() {
  uint32_t rand_num;
  HAL_RNG_GenerateRandomNumber(&hrng, &rand_num);

  uint8_t past_expected_lane = expected_lane;
  if (rand_num & 1) { // Determine if expected lane changes
    if (expected_lane == 1) {
      expected_lane = 3;
    }
    else if (expected_lane == 9) {
      expected_lane = 7;
    }
    else { // Determine if going right or left
      if ((rand_num >> 2) & 1) {
        expected_lane += 2;
      }
      else {
        expected_lane -= 2;
      }
    }
  }

  for (int col = 1; col <= 2 * N_COLS - 1; col += 2) {
    if (col == past_expected_lane || col == expected_lane) {
      map[top_row][col] = ' ';
      continue;
    }

    if ((rand_num >> col) & 1) {
      map[top_row][col] = 'X';
    }
    else {
      map[top_row][col] = ' ';
    }
  }
  top_row = (top_row + 1) % N_ROWS;

  //   Detects collision
  if(map[(top_row + char_vert_pos) % N_ROWS][char_horz_pos] == 'X'){
	  collision = TRUE;
  }
}

void display_map(uint8_t start_row) {
  // First, clear console
  HAL_UART_Transmit(&huart1, (uint8_t*) clear_screen, sizeof(clear_screen), 1000);


	char real_char = map[(start_row + char_vert_pos) % N_ROWS][char_horz_pos];
	map[(start_row + char_vert_pos) % N_ROWS][char_horz_pos] = 'O';

	// Display map
	for (int row = 0; row < N_ROWS; row++) {
	  HAL_UART_Transmit(&huart1, (uint8_t*) map[(start_row + row) % N_ROWS], sizeof(map[row]), 1000);
	}

	map[(start_row + char_vert_pos) % N_ROWS][char_horz_pos] = real_char;

	sprintf(score_string, "Score:%5d", score); // TODO: Change hardcoded value
	HAL_UART_Transmit(&huart1, (uint8_t*) score_string, sizeof(score_string), 1000);
}

void end_game() {
  // Place score in top 10
  int high_scores[11];
  if (BSP_QSPI_Read(high_scores, (uint32_t) SCORE_ADDR, sizeof(high_scores)) != QSPI_OK) {
    Error_Handler();
  }

  for (int rank = 9; rank>=0; rank--) {
    if (score > high_scores[rank]) {
      high_scores[rank+1] = high_scores[rank];
    }
    else {
      high_scores[rank+1] = score;
      break;
    }
  }
  if (score > high_scores[0]) high_scores[0] = score;

  // Save top 10
  if (BSP_QSPI_Erase_Block((uint32_t) SCORE_ADDR) != QSPI_OK) {
    Error_Handler();
  }
  if (BSP_QSPI_Write(high_scores, (uint32_t) SCORE_ADDR, sizeof(high_scores)) != QSPI_OK) {
    Error_Handler();
  }

  // Display top 10
  HAL_UART_Transmit(&huart1, (uint8_t*) clear_screen, sizeof(clear_screen), 1000);
  char title[] = "High scores:\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*) title, sizeof(title), 1000);

  char ranking_line[20] = "";
  for (int rank=1; rank<=10; rank++) {
    sprintf(ranking_line, "%2d  %5d\r\n", rank, high_scores[rank-1]);
    HAL_UART_Transmit(&huart1, (uint8_t*) ranking_line, sizeof(ranking_line), 1000);
  }

  char buf[100] = "";
  sprintf(buf, "\nYour score: %d points", score);
  HAL_UART_Transmit(&huart1, (uint8_t*) buf, sizeof(buf), 1000);

  // Wait for new game to start...
  while(1) {}
}
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_QUADSPI_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_DAC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t* ) joystickXY, 2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
//  enum state mode = straight;

  BSP_QSPI_Init();

  HAL_GPIO_WritePin(errorLED_GPIO_Port, errorLED_Pin, GPIO_PIN_SET);

  if(BSP_QSPI_Erase_Block(0) != QSPI_OK){
	  Error_Handler();
  }

  gen_notes();
  gen_music();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(500);
	  update_map();
	  if (collision) end_game();
	  score++;
	  display_map(top_row);
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim5.Init.Period = 1000;
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  /*Configure GPIO pin : errorLED_Pin */
  GPIO_InitStruct.Pin = errorLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(errorLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_Pin */
  GPIO_InitStruct.Pin = PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void gen_music(void){

	if(BSP_QSPI_Read(note_0, NOTE_0_ADDR, sizeof(uint32_t)*(NOTE_0_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_1, NOTE_1_ADDR, sizeof(uint32_t)*(NOTE_1_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_2, NOTE_2_ADDR, sizeof(uint32_t)*(NOTE_2_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_3, NOTE_3_ADDR, sizeof(uint32_t)*(NOTE_3_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_4, NOTE_4_ADDR, sizeof(uint32_t)*(NOTE_4_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Read(note_5, NOTE_5_ADDR, sizeof(uint32_t)*(NOTE_5_SIZE))!= QSPI_OK){
		Error_Handler();
	}

}

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

	if(BSP_QSPI_Write(note_0_temp, NOTE_0_ADDR, sizeof(uint32_t)*(NOTE_0_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_1_temp, NOTE_1_ADDR, sizeof(uint32_t)*(NOTE_1_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_2_temp, NOTE_2_ADDR, sizeof(uint32_t)*(NOTE_2_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_3_temp, NOTE_3_ADDR, sizeof(uint32_t)*(NOTE_3_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_4_temp, NOTE_4_ADDR, sizeof(uint32_t)*(NOTE_4_SIZE))!= QSPI_OK){
		Error_Handler();
	}
	if(BSP_QSPI_Write(note_5_temp, NOTE_5_ADDR, sizeof(uint32_t)*(NOTE_5_SIZE))!= QSPI_OK){
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

	if (htim == &htim4){
//	  char buf[30];
//	  sprintf(buf, "X: %d, Y: %d\r\n\n", joystickXY[0], joystickXY[1]);
//	  HAL_UART_Transmit(&huart1, (uint8_t*) buf, sizeof(buf), 1000);
		if(check_dir == 1){
			if(joystickXY[0] < 1000){
//				mode = right;
				char_horz_pos = (char_horz_pos < 9) ? (char_horz_pos + 2) : char_horz_pos;
				check_dir = 0;
				display_map(top_row);
			}
			else if(joystickXY[0] > 3000){
				char_horz_pos = (char_horz_pos > 1) ? (char_horz_pos - 2) : char_horz_pos;
//				mode = left;
				check_dir = 0;
				display_map(top_row);
			}
//			else{
//				//char_horz_pos = char_horz_pos;
//			}
		}
		else{
			if(joystickXY[0] >= 1000 && joystickXY[0] <= 3000){
				check_dir = 1;
			}
		}
	}


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
