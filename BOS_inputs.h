/*
    BitzOS (BOS) V0.2.4 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved

    File Name     : BOS_inputs.h
    Description   : header file for Bitz digital and analog inputs.
	
*/
	
	
/* Define to prevent recursive inclusion -------------------------------------*/

//#ifndef BOS_INPUTS_H
//#define BOS_INPUTS_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_adc_ex.h"
#include "string.h"
/* Private and global variables ----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/	
extern BOS_Status GetPortGPIOs(uint8_t port, uint32_t *TX_Port, uint16_t *TX_Pin, uint32_t *RX_Port, uint16_t *RX_Pin);
extern void ReadTempAndVref(float *temp, float *Vref);
extern void ReadADCChannel(uint8_t Port , char * side,float *ADC_Value);
extern void ADCSelectChannel(uint8_t ADC_port, char* side);

//#define Vref_Cal ((uint16_t *)((uint32_t)0x1ffff7BA))
//#define V25  1.41
//#define Avg_Slope 4.3
//uint8_t Channel=0;
//uint16_t ADC_value[6]={0};
//uint16_t ADC_value_temp=0;
//uint16_t ADC_value_Vref=0;
//uint8_t ADC_flag=0,Rank_t=0;
/* -----------------------------------------------------------------------
	|												Exported  Functions	 														|
   ----------------------------------------------------------------------- 
*/


/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
