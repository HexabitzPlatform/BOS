/*
    BitzOS (BOS) V0.2.1 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS_adc.h
    Description   : Header file for for BOS adc. 
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_adc_H
#define BOS_adc_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

/* External definitions -------------------------------------------------------*/

extern void MX_ADC_Init(void);


/* External variables ---------------------------------------------------------*/



/* External functions ---------------------------------------------------------*/
#endif

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
