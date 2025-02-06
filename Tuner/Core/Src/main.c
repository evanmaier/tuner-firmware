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
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <images.h>
#include "st7735.h"
#include "fonts.h"
#include "Yin.h"
#include "audio_data.h"
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_MODE 0
#define BUFFER_SIZE 256
#define SAMPLE_RATE 8000
#define THRESHOLD 0.1
#define A4 440
#define ADC_MAX 4095
#define WINDOW_SIZE 5
/* Display Parameters */
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 160
#define PADDING 10
/* Bar Parameters */
#define BAR_HEIGHT 40
#define BAR_WIDTH 50
#define CENTER_WIDTH 4
#define CENTER_OFFSET 2
/* Note Parameters */
#define NOTE_WIDTH 56
#define NOTE_HEIGHT 84
#define SHARP_WIDTH 20
#define SHARP_HEIGHT 43
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t adcData[BUFFER_SIZE*2];
uint16_t* adcBufPtr;
uint8_t dataReady;

float32_t yinBuffer[BUFFER_SIZE];
Yin yin;

uint16_t* testBufPtr;

float32_t avgWindow[WINDOW_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	adcBufPtr = &adcData[0];
	dataReady = true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcBufPtr = &adcData[BUFFER_SIZE];
	dataReady = true;
}

int closest_note(float32_t pitchInHz) {
    float32_t n = 12.0f * log2f(pitchInHz / (float32_t) A4);
    n = roundf(fmodf(n, 12.0f));
    return ((int)n + 12) % 12;
}

int deviation_cents(float32_t pitchInHz) {
	float32_t semitones = 12.0f * log2f(pitchInHz / (float32_t) A4);
	int nearest_semitone = roundf(semitones);
	float32_t deviation = (semitones - nearest_semitone) * 100;
	return (int)roundf(deviation);
}

void draw_note(const uint16_t* note, uint8_t isSharp) {
	ST7735_DrawImage((DISPLAY_WIDTH - NOTE_WIDTH)/2, DISPLAY_HEIGHT - NOTE_HEIGHT - PADDING,NOTE_WIDTH, NOTE_HEIGHT, note);
	if (isSharp) {
		ST7735_DrawImage((DISPLAY_WIDTH - NOTE_WIDTH)/2 + NOTE_WIDTH, DISPLAY_HEIGHT - NOTE_HEIGHT - PADDING, SHARP_WIDTH, SHARP_HEIGHT, image_data_Font_0x23);
	} else {
		ST7735_FillRectangleFast((DISPLAY_WIDTH - NOTE_WIDTH)/2 + NOTE_WIDTH, DISPLAY_HEIGHT - NOTE_HEIGHT - PADDING, SHARP_WIDTH, SHARP_HEIGHT, ST7735_BLACK);
	}
}

void draw_bar(int cents){
	uint16_t x = PADDING;
	if (cents < 0) {
		cents = abs(cents);
		ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH - cents, BAR_HEIGHT, ST7735_BLACK);
		x += BAR_WIDTH - cents;
		ST7735_FillRectangleFast(x, PADDING, cents, BAR_HEIGHT, ST7735_RED);
		x += abs(cents) + CENTER_WIDTH + 2*CENTER_OFFSET;
		ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_BLACK);
	} else {
		ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_BLACK);
		x += BAR_WIDTH + CENTER_WIDTH + 2*CENTER_OFFSET;
		ST7735_FillRectangleFast(x, PADDING, cents, BAR_HEIGHT, ST7735_GREEN);
		x += cents;
		ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH - cents, BAR_HEIGHT, ST7735_BLACK);
	}
}

void update_display(float32_t pitchInHz) {
	draw_bar(deviation_cents(pitchInHz));

	switch (closest_note(pitchInHz))
	{
	case 0:
		draw_note(image_data_Font_0x41, false);
		break;
	case 1:
		draw_note(image_data_Font_0x41, true);
		break;
	case 2:
		draw_note(image_data_Font_0x42, false);
		break;
	case 3:
		draw_note(image_data_Font_0x43, false);
		break;
	case 4:
		draw_note(image_data_Font_0x43, true);
		break;
	case 5:
		draw_note(image_data_Font_0x44, false);
		break;
	case 6:
		draw_note(image_data_Font_0x44, true);
		break;
	case 7:
		draw_note(image_data_Font_0x45, false);
		break;
	case 8:
		draw_note(image_data_Font_0x46, false);
		break;
	case 9:
		draw_note(image_data_Font_0x46, true);
		break;
	case 10:
		draw_note(image_data_Font_0x47, false);
		break;
	case 11:
		draw_note(image_data_Font_0x47, true);
		break;
	}
}

void normalize_data() {
	for(int i=0; i < BUFFER_SIZE; i++) {
		yinBuffer[i] = (float32_t)(2 * adcBufPtr[i] - ADC_MAX) / (float32_t) ADC_MAX;
	}
}

void normalize_test_data() {
	for(int i=0; i < BUFFER_SIZE; i++) {
		yinBuffer[i] = (float32_t)(2 * testBufPtr[i] - ADC_MAX) / (float32_t) ADC_MAX;
	}
}

float32_t avg_pitch() {
	float32_t runningSum = 0.0f;
	float32_t n = 0.0f;
	for(int i=0; i<WINDOW_SIZE; i++) {
		if (avgWindow[i] > 0) {
			runningSum += avgWindow[i];
			n++;
		}
	}
	if (n > 0) {
		return runningSum/n;
	}
	return -1;
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
	MX_SPI2_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, BUFFER_SIZE*2);
	HAL_TIM_Base_Start(&htim2);

	Yin_init(&yin, BUFFER_SIZE, SAMPLE_RATE, THRESHOLD);

	ST7735_Init();
	ST7735_FillScreenFast(ST7735_BLACK);
	ST7735_FillRectangleFast(PADDING + BAR_WIDTH + CENTER_OFFSET, PADDING, CENTER_WIDTH, BAR_HEIGHT, ST7735_YELLOW);
	/* USER CODE END 2 */

	/* Infinite loop */

	/* Test Code */
	if (TEST_MODE) {
		while (1) {
			testBufPtr = &sweepData[0];
			for(int i = 0; i < sizeof(sweepData)/sizeof(uint16_t) - BUFFER_SIZE; i++) {
			  normalize_test_data();
			  update_display(Yin_getPitch(&yin, yinBuffer));
			  testBufPtr++;
			}
		}
	}

    /* Real Code */
	float32_t pitch;
	while (1) {
		if (dataReady) {
			normalize_data();

			pitch = Yin_getPitch(&yin, yinBuffer);

			if (pitch > 0) {
				update_display(pitch);
			}

			dataReady = false;
		}
	}
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Period = 12000-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
