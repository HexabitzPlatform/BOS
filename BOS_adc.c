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
extern void MX_ADC_Init(void);
static void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/


/* External functions --------------------------------------------------------*/
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	__ADC1_CLK_ENABLE();

  hdma_adc.Instance = DMA1_Channel1;
  hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc.Init.MemDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_adc.Init.Mode = DMA_CIRCULAR;
  hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
  HAL_DMA_Init(&hdma_adc);
	__HAL_DMA1_REMAP(HAL_DMA1_CH1_ADC);   
  __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc);

  /* DMA interrupt init */
  /* DMA1_Ch1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
//  /** Configure for the selected ADC regular channel to be converted. 
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted. 
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted. 
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel to be converted. 
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC_Init 2 */

//  /* USER CODE END ADC_Init 2 */

}
/** 
  * Enable DMA controller clock
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
