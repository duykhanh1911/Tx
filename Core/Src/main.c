/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_math_types_f16.h"
#include "fdacoefs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADC and DAC configuration
#define ADC_SAMPLE_RATE 96000    // Sampling frequency in Hz
#define ADC_BUFFER_SIZE 64       // Size of ADC buffer for DMA transfer
#define SINE_TABLE_SIZE 8      // Size of sine wave lookup table
#define DAC_BUFFER_SIZE ADC_BUFFER_SIZE
#define BLOCK_SIZE 32            // Block size for filter processing
#define NUM_STAGE_IIR 16
#define NUM_STD_COEFFS 5
#define IIR_ORDER (NUM_STAGE_IIR*2)            // Order of the IIR filter (32 + 1 for index 0)
#define FIR_ORDER 455
#define NUM_BLOCKS (DAC_BUFFER_SIZE / BLOCK_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

// DMA buffers for ADC and DAC
uint16_t adcBuffer[ADC_BUFFER_SIZE];
float32_t dacBuffer[DAC_BUFFER_SIZE];
// Signal processing buffers
float32_t sineTable[SINE_TABLE_SIZE];     // Carrier wave lookup table
float32_t adcValue[ADC_BUFFER_SIZE];      // ADC samples converted to float
float32_t modulatedSignalHalf[DAC_BUFFER_SIZE/2];
float32_t modulatedSignal[DAC_BUFFER_SIZE]; // Buffer for modulated signal
float32_t *modulatedSignalPtr = &modulatedSignal[0];

// IIR filter buffers and structure
float32_t filt_out_BPF[ADC_BUFFER_SIZE];  // Output buffer for filtered signal
float32_t *filt_out_BPF_ptr = &filt_out_BPF[0];

// IIR filter structure and state variables
arm_biquad_cascade_df2T_instance_f32 iir_filter_BPF;
arm_fir_instance_f32 fir_filter_BPF;
float32_t iir_State_BPF[2*NUM_STAGE_IIR];  // State buffer for IIR filter (2 states per stage)
float32_t fir_State_BPF[BLOCK_SIZE+FIR_ORDER-1];
// IIR filter coefficients for bandpass filter
// Fstop1 : 10000Hz
// Fpass1 : 10300Hz
// Fpass2 : 11700Hz
// Fstop2 : 12000Hz
// Astop1 = Astop2 = 40dB
// Apass  = 1dB
// Denominator coefficients (a) - represents the feedback part of the filter
static const float32_t a_coeffs[IIR_ORDER] = {
    1, -23.3433254617316, 270.458923271493, -2068.05077951067, 11711.9227186727,
    -52280.9402944496, 191203.373802611, -588036.150343295, 1549228.77829786,
    -3544487.26024351, 7114999.55150585, -12629096.804023, 19940196.4604793,
    -28131336.186653, 35577251.704648, -40424177.8370125, 41319828.7727523,
    -38010952.4767753, 31456289.2334764, -23387992.8902516, 15588325.8205582,
    -9283456.97370102, 4917899.62201564, -2303697.87299914, 946792.887857377,
    -337917.637907267, 103316.564961882, -26563.5229314584, 5595.49926794879,
    -929.053986421767, 114.248700906604, -9.27223577617348, 0.37350446875285
};

// Numerator coefficients (b) - represents the feedforward part of the filter
static const float32_t b_coeffs[IIR_ORDER] = {
	5.32193000539747e-22,0,-8.51508800863595e-21,0,6.38631600647696e-20,
	0,-2.98028080302258e-19,0,9.6859126098234e-19,0,
	-2.32461902635761e-18,0,4.26180154832229e-18,0,-6.08828792617471e-18,
	0,6.84932391694654e-18,0,-6.08828792617471e-18,0,4.26180154832229e-18,
	0,-2.32461902635761e-18,0,9.6859126098234e-19,0,-2.98028080302258e-19,
	0,6.38631600647696e-20,0,-8.51508800863595e-21,0,5.32193000539747e-22

};

// Combined coefficients array for biquad cascade implementation
// Format: [b0, b1, b2, a1, a2] for each biquad section
float32_t iir_coeffs[NUM_STAGE_IIR * NUM_STD_COEFFS];
float32_t coefficients[NUM_STAGE_IIR * NUM_STD_COEFFS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
float32_t dc=1800.0f;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Callback function for ADC conversion half complete
 * @param  hadc: ADC handle pointer
 * @note   Processes first half of ADC buffer:
 *         1. Converts ADC samples to float
 *         2. Modulates with carrier wave
 *         3. Applies IIR filter
 *         4. Outputs to DAC with DC offset
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
	uint32_t sineIndex=0;
	for(uint32_t i=0;i<ADC_BUFFER_SIZE/2;i++){
		adcValue[i]=((float32_t)adcBuffer[i] - 2048.0f) / 2048.0f;
	}
	for(uint32_t i=0;i<ADC_BUFFER_SIZE/2;i++){
		modulatedSignal[i]=adcValue[i]*sineTable[sineIndex];
		sineIndex=(sineIndex+1)%SINE_TABLE_SIZE;
	}
	//arm_fir_f32(&fir_filter_BPF, modulatedSignalPtr+BLOCK_SIZE,filt_out_BPF_ptr+BLOCK_SIZE , BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32(&iir_filter_BPF, modulatedSignalPtr, filt_out_BPF_ptr, BLOCK_SIZE);

	for(int i=0;i<DAC_BUFFER_SIZE/2;i++){
	    dacBuffer[i] = (uint32_t)(filt_out_BPF[i]*2047+2048);
	}
}

/**
 * @brief  Callback function for ADC conversion complete
 * @param  hadc: ADC handle pointer
 * @note   Processes second half of ADC buffer while DMA fills first half
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	uint32_t sineIndex=0;
	for(uint32_t i=ADC_BUFFER_SIZE/2;i<ADC_BUFFER_SIZE;i++){
		adcValue[i]=((float32_t)adcBuffer[i] - 2048.0f) / 2048.0f;
	}
	for(uint32_t i=ADC_BUFFER_SIZE/2;i<ADC_BUFFER_SIZE;i++){
		modulatedSignal[i]=adcValue[i]*sineTable[sineIndex];
		sineIndex=(sineIndex+1)%SINE_TABLE_SIZE;
	}
	//arm_fir_f32(&fir_filter_BPF, modulatedSignalPtr+BLOCK_SIZE,filt_out_BPF_ptr+BLOCK_SIZE , BLOCK_SIZE);
	arm_biquad_cascade_df2T_f32(&iir_filter_BPF, modulatedSignalPtr+BLOCK_SIZE, filt_out_BPF_ptr+BLOCK_SIZE, BLOCK_SIZE);
	for(int i=DAC_BUFFER_SIZE/2;i<DAC_BUFFER_SIZE;i++){
		dacBuffer[i]=(uint32_t)(filt_out_BPF[i]*2047+2048);
	}
}

/**
 * @brief  Generates sine wave lookup table for carrier signal
 * @note   Creates one complete cycle of sine wave with SINE_TABLE_SIZE points
 */
void GenerateSineWave(void){
	for(int i=0; i<SINE_TABLE_SIZE;i++){
		sineTable[i]=arm_sin_f32(2.0f*PI*i/SINE_TABLE_SIZE);
	}
}

/**
 * @brief  Initializes the IIR filter
 * @note   Converts filter coefficients to biquad cascade form
 *         Each biquad section requires 5 coefficients: b0, b1, b2, a1, a2
 *         The filter is implemented as a cascade of second-order sections
 *         Direct Form II Transposed structure is used for better numerical stability
 */
void InitializeIIRFilter(void) {
    // Convert coefficients to biquad cascade form
     for(int i = 0; i < NUM_STAGE_IIR; i++) {
        // Get coefficients for this biquad section
        float32_t b0 = b_coeffs[i*2];
        float32_t b1 = b_coeffs[i*2 + 1];
        float32_t b2 = b_coeffs[i*2 + 2];
        float32_t a1 = a_coeffs[i*2 + 1];
        float32_t a2 = a_coeffs[i*2 + 2];
        
        // Normalize coefficients by a0 (which is 1)
        iir_coeffs[i*5] = b0;      // b0
        iir_coeffs[i*5 + 1] = b1;  // b1
        iir_coeffs[i*5 + 2] = b2;  // b2
        iir_coeffs[i*5 + 3] = -a1; // -a1 (negative for Direct Form II Transposed)
        iir_coeffs[i*5 + 4] = -a2; // -a2 (negative for Direct Form II Transposed)
    }
     for (int i = 0; i < 16; i++) {
         coefficients[i * 5 + 0] =  NUM[i * 2][0] * NUM[i * 2 + 1][0];
         coefficients[i * 5 + 1] =  NUM[i * 2][0] * NUM[i * 2 + 1][1];
         coefficients[i * 5 + 2] =  NUM[i * 2][0] * NUM[i * 2 + 1][2];
         coefficients[i * 5 + 3] = -DEN[i * 2][0] * DEN[i * 2 + 1][1];
         coefficients[i * 5 + 4] = -DEN[i * 2][0] * DEN[i * 2 + 1][2];
     }
    // Initialize the IIR filter structure
    arm_biquad_cascade_df2T_init_f32(&iir_filter_BPF, NUM_STAGE_IIR, coefficients, iir_State_BPF);
    //arm_biquad_cascade_df2T_init_f32(&iir_filter_BPF, NUM_STAGE_IIR, iir_coeffs, iir_State_BPF);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  GenerateSineWave();
  //arm_fir_init_f32(&fir_filter_BPF, FIR_ORDER, &fir_Coeffs[0], &fir_State_BPF[0], BLOCK_SIZE);
  InitializeIIRFilter();
  HAL_TIM_Base_Start(&htim8);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcBuffer, ADC_BUFFER_SIZE);
  if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)&dacBuffer, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R)!=HAL_OK){
	  Error_Handler();

  };

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
