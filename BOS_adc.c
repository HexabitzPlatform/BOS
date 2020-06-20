/*
    BitzOS (BOS) V0.2.1 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved

    File Name     : BOS_adc.c
    Description   : Source code for BOS configuration of the ADC instances. 
		
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "BOS_adc.h"

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* Exported variables ---------------------------------------------------------*/

extern void ADC_Init(void);
static void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/

uint8_t ADC_Error_Status;

/* External functions --------------------------------------------------------*/
void ADC_Init(void)
{
  GPIO_InitTypeDef ADC_pin;

  ADC_ChannelConfTypeDef sConfig;
	/* Enable ADC clock */
	
	__HAL_RCC_DMA1_CLK_ENABLE();

	/** Configure the global features of the DMA (Direction, PeriphInc, Mode,...etc) */
  hdma_adc.Instance = DMA1_Channel1;
  hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc.Init.Mode = DMA_CIRCULAR;
  hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
  HAL_DMA_Init(&hdma_adc);
	
	__HAL_DMA2_REMAP(HAL_DMA1_CH1_ADC);

   __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc);

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
	
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	


 	__HAL_RCC_GPIOA_CLK_ENABLE();
	ADC_pin.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
	ADC_pin.Mode=GPIO_MODE_ANALOG;
	ADC_pin.Pull= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ADC_pin);
	/** initialize ADC **/
	
  if (HAL_ADC_Init(&hadc) != HAL_OK){
			Error_Handler();
  }
	/** Configure for the selected ADC regular channel to be converted.*/
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
			Error_Handler();
  }
	/** Configure for the selected ADC regular channel to be converted.*/
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
			Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.*/
  sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
			Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.*/
  sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
			Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. */
  sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
    Error_Handler();
  }
 
	
	/** initialize DMA controller **/
	
	/** Enable DMA1 controller clock **/

	
	/** Remap and link ADC with DMA **/
	
//  __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc);
//	__HAL_DMA1_REMAP(HAL_DMA1_CH1_ADC);   
	/* DMA interrupt init */
	
  /* DMA1_Ch1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);
}

void Error_Handler(void)
{
ADC_Error_Status=HAL_ADC_GetError(&hadc);
}
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
