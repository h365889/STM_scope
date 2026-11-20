/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t *lut;
	uint32_t phase_step;
	float scale;
}DDS;

typedef struct
{
	uint32_t buff_size;
	uint32_t freq;
}Measure_cfg;

typedef enum
{
	MODE_SCOPE,
	MODE_TRIGGER
}Measure_mode;
typedef enum
{
	MEAS_IDLE,
	MEAS_RUNNING,
	MEAS_SENDING,
	MEAS_DONE

}Measure_state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_BUF_MAX_LEN 4096U
#define SCOPE_BUF_LEN 256U
#define LUT_SIZE 256U
#define DAC_MAX 4095U
#define DAC_F_REF 100000U
#define TIM_CLOCK 108000000U
#define TIM_PRESCALE 107U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
Measure_mode measure_mode = MODE_SCOPE;
Measure_state measure_state = MEAS_IDLE;

uint16_t adcBuf[ADC_BUF_MAX_LEN];

uint16_t lut_sine[256];
uint16_t lut_tri[256];
uint32_t phase_acc = 0;

DDS dds =
{
	.lut = lut_sine,
	.phase_step = 0,
	.scale = 1
};

Measure_cfg measure_cfg =
{
	.buff_size = 256,
	.freq = 1000
};
uint32_t TxIndex = 0;

volatile uint8_t halfReady = 0;
volatile uint8_t fullReady = 0;
volatile uint8_t usbBusy = 0;
volatile uint8_t configPending = 0;
volatile uint8_t trigger = 0;
volatile uint8_t toTrigger = 0;
volatile uint8_t toScope = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
uint8_t CDC_Transmit_FS(uint8_t* data, uint16_t len);
void Fill_LUT();
void DDS_GetTuningWord(uint16_t Hz);
void ADC_DMA_SetMode(uint32_t mode);
void CalculateTimArr(uint32_t fs);
void SwitchToScopeMode(void);
void SwitchToTriggerMode(void);
void UpdateConfig(void);
void Trigger_Start(void);
void Trigger_SendData(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Fill_LUT();
  DDS_GetTuningWord(1000);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcBuf,SCOPE_BUF_LEN);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)lut_sine, 256, DAC_ALIGN_12B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(toScope)
	  {
		  if(measure_mode != MODE_SCOPE)
		  {
			  SwitchToScopeMode();
		  }
		  toScope = 0;
	  }
	  if(toTrigger)
	  {
		  if(measure_mode != MODE_TRIGGER)
		  {
			  SwitchToTriggerMode();
		  }

		  toTrigger = 0;
	  }
	  if(configPending)
	  {
		  if(measure_mode == MODE_TRIGGER)
		  {
		  UpdateConfig();
		  }
		  configPending = 0;
	  }
	  if(measure_mode == MODE_SCOPE){

		  if (halfReady && !usbBusy)
		  {
			  if (CDC_Transmit_FS((uint8_t*)&adcBuf[0], (SCOPE_BUF_LEN / 2) * sizeof(uint16_t)) == USBD_OK)
			  {
				  halfReady = 0;
				  usbBusy = 1;
			  }
		  }

		  if (fullReady && !usbBusy)
		  {
			  if (CDC_Transmit_FS((uint8_t*)&adcBuf[SCOPE_BUF_LEN / 2], (SCOPE_BUF_LEN / 2) * sizeof(uint16_t)) == USBD_OK)
			  {
				  fullReady = 0;
				  usbBusy = 1;
			  }
		  }
	  }
	  else
	  {
		  if (trigger)
		      {
		          trigger = 0;
		          Trigger_Start();
		      }


		      if (fullReady)
		      {
		          TxIndex = 0;
		          measure_state = MEAS_SENDING;
		          fullReady = 0;
		      }

		      if (measure_state == MEAS_SENDING)
		      {
		          Trigger_SendData();
		      }
	  }

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 107;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 107;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	uint16_t DDS_GetSample(DDS *dds)
	{
		phase_acc += dds->phase_step;
		uint8_t index = phase_acc >> 24;
		float sample = (float)dds->lut[index];
		sample = 2047.5f +(sample - 2047.5f) * dds->scale;
		return (uint16_t)sample;
	}

	void DDS_GetTuningWord(uint16_t Hz)
	{
		if(Hz>DAC_F_REF/2)
		{
			dds.phase_step = 0;
		}
		uint64_t m = (Hz * (float)UINT32_MAX) / DAC_F_REF;
		dds.phase_step = (uint32_t)m;
	}

	void Fill_LUT()
	{
	    for (uint16_t i = 0; i < LUT_SIZE; i++) {
	        float phase = (float) i / LUT_SIZE;

	        lut_sine[i] = (uint16_t)((DAC_MAX / 2.0f) + (DAC_MAX / 2.0f) * sinf(2.0f * M_PI * phase));

	        if (phase < 0.5f) {
	            lut_tri[i] = (uint16_t)(2.0f * phase * DAC_MAX);
	        }
	        else
	        {
	            lut_tri[i] = (uint16_t)(2.0f * (1.0f-phase) * DAC_MAX);
	        }
	    }
	}
	void ADC_DMA_SetMode(uint32_t mode)
	{
	    HAL_DMA_DeInit(&hdma_adc1);
	    hdma_adc1.Init.Mode = mode;
	    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
	    {
	        Error_Handler();
	    }
	}

	void SwitchToTriggerMode(void)
	{
	    HAL_TIM_Base_Stop(&htim1);
	    HAL_ADC_Stop_DMA(&hadc1);
	    ADC_DMA_SetMode(DMA_NORMAL);
	    CalculateTimArr(measure_cfg.freq);
	    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcBuf,measure_cfg.buff_size);
	    fullReady = 0;
	    measure_mode = MODE_TRIGGER;
	    measure_state = MEAS_IDLE;
	}
	void SwitchToScopeMode(void)
	{
	    HAL_TIM_Base_Stop(&htim1);
	    HAL_ADC_Stop_DMA(&hadc1);
	    ADC_DMA_SetMode(DMA_CIRCULAR);
	    __HAL_TIM_SET_AUTORELOAD(&htim1, 19);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
	    halfReady = 0;
	    fullReady = 0;
	    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcBuf,SCOPE_BUF_LEN);
	    HAL_TIM_Base_Start(&htim1);
	    measure_mode = MODE_SCOPE;
	}
	void CalculateTimArr(uint32_t fs){
		 uint32_t arr = TIM_CLOCK / ((TIM_PRESCALE + 1U) * fs) - 1U;
		 __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
		 __HAL_TIM_SET_COUNTER(&htim1, 0);
	}
	void UpdateConfig(void)
	{
		 HAL_TIM_Base_Stop(&htim1);
		 HAL_ADC_Stop_DMA(&hadc1);
		 CalculateTimArr(measure_cfg.freq);
		 measure_state = MEAS_IDLE;
	}

	void Trigger_Start(void)
	{
	    if (measure_state == MEAS_IDLE)
	    {
	        __HAL_TIM_SET_COUNTER(&htim1, 0);
	        HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcBuf,measure_cfg.buff_size);
	        measure_state = MEAS_RUNNING;

	        HAL_TIM_Base_Start(&htim1);
	    }
	}

	void Trigger_SendData(void)
	{
	    if (usbBusy)
	        return;

	    uint32_t totalBytes = measure_cfg.buff_size * sizeof(uint16_t);

	    if (TxIndex < totalBytes)
	    {
	        uint32_t remaining = totalBytes - TxIndex;
	        uint16_t chunkSize = (remaining > SCOPE_BUF_LEN) ? SCOPE_BUF_LEN : remaining;
	        uint8_t *data = (uint8_t *)adcBuf;

	        if (CDC_Transmit_FS(&data[TxIndex],chunkSize) == USBD_OK)
	        {
	            usbBusy = 1;
	            TxIndex += chunkSize;
	        }
	    }
	    else
	    {
	        measure_state = MEAS_IDLE;
	        TxIndex = 0;
	    }
	}

   void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
   {
       if(hadc->Instance == ADC1)
       {
           halfReady = 1;
       }
   }

   void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
   {
       if(hadc->Instance == ADC1)
       {
    	   if(measure_mode == MODE_TRIGGER)
    	   {
    		   HAL_TIM_Base_Stop(&htim1);
    		   HAL_ADC_Stop_DMA(&hadc1);
    	   }
           fullReady = 1;
       }
   }

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
#ifdef USE_FULL_ASSERT
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
