/**
  ******************************************************************************
  * File Name          : H01R0_DMA.c
  * Description        : This file provides code for the configuration
  *                      of the DMA instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	
/*
		MODIFIED by Hexabitz for BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"



/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/


/* Variables ---------------------------------------------------------*/

DMA_HandleTypeDef portMemDMA1;
DMA_HandleTypeDef portMemDMA2;
DMA_HandleTypeDef portMemDMA3;
DMA_HandleTypeDef portPortDMA1;
DMA_HandleTypeDef portPortDMA2;
DMA_HandleTypeDef portPortDMA3;


/* Private function prototypes -----------------------------------------------*/
void PortMemDMA1_Init(void);
void PortMemDMA2_Init(void);
void PortMemDMA3_Init(void);
void PortPortDMA1_Init(void);
void PortPortDMA2_Init(void);
void PortPortDMA3_Init(void);

/*-----------------------------------------------------------*/

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__DMA1_CLK_ENABLE();
	__DMA2_CLK_ENABLE();
	
	PortMemDMA1_Init();
	PortMemDMA2_Init();
	PortMemDMA3_Init();
	PortPortDMA1_Init();
	PortPortDMA2_Init();
	PortPortDMA3_Init();
}

/*-----------------------------------------------------------*/

/* Messaging DMA 1 (port-to-memory) initialization */
void PortMemDMA1_Init(void)
{
	/* UART RX DMA (DMA1 Ch5) */
	portMemDMA1.Instance = DMA1_Channel5;
	portMemDMA1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	portMemDMA1.Init.PeriphInc = DMA_PINC_DISABLE;
	portMemDMA1.Init.MemInc = DMA_MINC_ENABLE;
	portMemDMA1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	portMemDMA1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	portMemDMA1.Init.Mode = DMA_NORMAL; 
	portMemDMA1.Init.Priority = DMA_PRIORITY_HIGH;
	
	HAL_DMA_Init(&portMemDMA1);		
}

/*-----------------------------------------------------------*/

/* Messaging DMA 1 (port-to-memory) setup */
void PortMemDMA1_Setup(UART_HandleTypeDef* huart, uint8_t num)
{	
	/* UART RX DMA (DMA1 Ch5) */
	if (huart->Instance == USART1) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART1_RX);
	} else if (huart->Instance == USART2) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART2_RX);
	} else if (huart->Instance == USART3) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART3_RX);
	} else if (huart->Instance == USART4) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART4_RX);
	} else if (huart->Instance == USART5) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART5_RX);
	} else if (huart->Instance == USART6) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART6_RX);
	} else if (huart->Instance == USART7) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART7_RX);
	} else if (huart->Instance == USART8) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH5_USART8_RX);
	}		
	__HAL_LINKDMA(huart,hdmarx,portMemDMA1);
	
	
	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch3_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);
	
	/* Start DMA stream	*/	
	HAL_UART_Receive_DMA(huart, (uint8_t *)&cMessage[GetPort(huart)-1], num);			
}

/*-----------------------------------------------------------*/

/* Messaging DMA 2 (port-to-memory) initialization */
void PortMemDMA2_Init(void)
{
	/* UART RX DMA (DMA1 Ch6) */
	portMemDMA2.Instance = DMA1_Channel6;
	portMemDMA2.Init.Direction = DMA_PERIPH_TO_MEMORY;
	portMemDMA2.Init.PeriphInc = DMA_PINC_DISABLE;
	portMemDMA2.Init.MemInc = DMA_MINC_ENABLE;
	portMemDMA2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	portMemDMA2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	portMemDMA2.Init.Mode = DMA_NORMAL;
	portMemDMA2.Init.Priority = DMA_PRIORITY_HIGH;
	
	HAL_DMA_Init(&portMemDMA2);		
}

/*-----------------------------------------------------------*/

/* Messaging DMA 2 (port-to-memory) setup */
void PortMemDMA2_Setup(UART_HandleTypeDef* huart, uint8_t num)
{	
	/* UART RX DMA (DMA1 Ch6) */
	if (huart->Instance == USART1) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART1_RX);
	} else if (huart->Instance == USART2) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART2_RX);
	} else if (huart->Instance == USART3) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART3_RX);
	} else if (huart->Instance == USART4) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART4_RX);
	} else if (huart->Instance == USART5) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART5_RX);
	} else if (huart->Instance == USART6) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART6_RX);
	} else if (huart->Instance == USART7) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART7_RX);
	} else if (huart->Instance == USART8) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH6_USART8_RX);
	}		
	__HAL_LINKDMA(huart,hdmarx,portMemDMA2);
	
	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch3_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);
	
	/* Start DMA stream	*/	
	HAL_UART_Receive_DMA(huart, (uint8_t *)&cMessage[GetPort(huart)-1], num);			
}

/*-----------------------------------------------------------*/

/* Messaging DMA 3 (port-to-memory) initialization */
void PortMemDMA3_Init(void)
{
	/* UART RX DMA (DMA2 Ch2) */
	portMemDMA3.Instance = DMA2_Channel2;
	portMemDMA3.Init.Direction = DMA_PERIPH_TO_MEMORY;
	portMemDMA3.Init.PeriphInc = DMA_PINC_DISABLE;
	portMemDMA3.Init.MemInc = DMA_MINC_ENABLE;
	portMemDMA3.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	portMemDMA3.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	portMemDMA3.Init.Mode = DMA_NORMAL;
	portMemDMA3.Init.Priority = DMA_PRIORITY_HIGH;
	
	HAL_DMA_Init(&portMemDMA3);		
}

/*-----------------------------------------------------------*/

/* Messaging DMA 3 (port-to-memory) setup */
void PortMemDMA3_Setup(UART_HandleTypeDef* huart, uint8_t num)
{	
	/* UART RX DMA (DMA2 Ch2) */
	if (huart->Instance == USART1) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART1_RX);
	} else if (huart->Instance == USART2) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART2_RX);
	} else if (huart->Instance == USART3) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART3_RX);
	} else if (huart->Instance == USART4) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART4_RX);
	} else if (huart->Instance == USART5) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART5_RX);
	} else if (huart->Instance == USART6) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART6_RX);
	} else if (huart->Instance == USART7) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART7_RX);
	} else if (huart->Instance == USART8) {
		__HAL_DMA2_REMAP(HAL_DMA2_CH2_USART8_RX);
	}		
	__HAL_LINKDMA(huart,hdmarx,portMemDMA3);

	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
	
	/* Start DMA stream	*/	
	HAL_UART_Receive_DMA(huart, (uint8_t *)&cMessage[GetPort(huart)-1], num);			
}

/*-----------------------------------------------------------*/

/* Streaming DMA 1 (port-to-port) initialization */
void PortPortDMA1_Init(void)
{
	/* UART RX DMA (DMA1 Ch1) */
	portPortDMA1.Instance = DMA1_Channel1;
	portPortDMA1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	portPortDMA1.Init.PeriphInc = DMA_PINC_DISABLE;
	portPortDMA1.Init.MemInc = DMA_MINC_DISABLE;
	portPortDMA1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	portPortDMA1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	portPortDMA1.Init.Mode = DMA_CIRCULAR;
	portPortDMA1.Init.Priority = DMA_PRIORITY_HIGH;		
		
	HAL_DMA_Init(&portPortDMA1);	
}

/*-----------------------------------------------------------*/

/* Streaming DMA 1 (port-to-port) setup */
void PortPortDMA1_Setup(UART_HandleTypeDef* huartSrc, UART_HandleTypeDef* huartDst, uint8_t num)
{		
	/* UART RX DMA (DMA1 Ch1) */
	if (huartSrc->Instance == USART1) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART1_RX);
	} else if (huartSrc->Instance == USART2) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART2_RX);
	} else if (huartSrc->Instance == USART3) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART3_RX);
	} else if (huartSrc->Instance == USART4) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART4_RX);
	} else if (huartSrc->Instance == USART5) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART5_RX);
	} else if (huartSrc->Instance == USART6) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART6_RX);
	} else if (huartSrc->Instance == USART7) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART7_RX);
	} else if (huartSrc->Instance == USART8) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH1_USART8_RX);
	}		
	__HAL_LINKDMA(huartSrc,hdmarx,portPortDMA1);
	
	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);
	
	/* Start DMA stream	*/	
	huartSrc->State = HAL_UART_STATE_READY;
	HAL_UART_Receive_DMA(huartSrc, (uint8_t *)(&(huartDst->Instance->TDR)), num);
}

/*-----------------------------------------------------------*/

/* Streaming DMA 2 (port-to-port) initialization */
void PortPortDMA2_Init(void)
{
	/* UART RX DMA (DMA1 Ch3) */
	portPortDMA2.Instance = DMA1_Channel3;
	portPortDMA2.Init.Direction = DMA_PERIPH_TO_MEMORY;
	portPortDMA2.Init.PeriphInc = DMA_PINC_DISABLE;
	portPortDMA2.Init.MemInc = DMA_MINC_DISABLE;
	portPortDMA2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	portPortDMA2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	portPortDMA2.Init.Mode = DMA_CIRCULAR;
	portPortDMA2.Init.Priority = DMA_PRIORITY_HIGH;		
		
	HAL_DMA_Init(&portPortDMA2);	
}

/*-----------------------------------------------------------*/

/* Streaming DMA 2 (port-to-port) setup */
void PortPortDMA2_Setup(UART_HandleTypeDef* huartSrc, UART_HandleTypeDef* huartDst, uint8_t num)
{		
	/* UART RX DMA (DMA1 Ch3) */
	if (huartSrc->Instance == USART1) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART1_RX);
	} else if (huartSrc->Instance == USART2) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART2_RX);
	} else if (huartSrc->Instance == USART3) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART3_RX);
	} else if (huartSrc->Instance == USART4) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART4_RX);
	} else if (huartSrc->Instance == USART5) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART5_RX);
	} else if (huartSrc->Instance == USART6) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART6_RX);
	} else if (huartSrc->Instance == USART7) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART7_RX);
	} else if (huartSrc->Instance == USART8) {
		__HAL_DMA1_REMAP(HAL_DMA1_CH3_USART8_RX);
	}		
	__HAL_LINKDMA(huartSrc,hdmarx,portPortDMA2);
	
	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
	
	/* Start DMA stream	*/	
	huartSrc->State = HAL_UART_STATE_READY;
	HAL_UART_Receive_DMA(huartSrc, (uint8_t *)(&(huartDst->Instance->TDR)), num);
}

/*-----------------------------------------------------------*/

/* Streaming DMA 3 (port-to-port) initialization */
void PortPortDMA3_Init(void)
{
	/* UART RX DMA (DMA2 Ch3) */
	portPortDMA3.Instance = DMA2_Channel3;
	portPortDMA3.Init.Direction = DMA_PERIPH_TO_MEMORY;
	portPortDMA3.Init.PeriphInc = DMA_PINC_DISABLE;
	portPortDMA3.Init.MemInc = DMA_MINC_DISABLE;
	portPortDMA3.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	portPortDMA3.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	portPortDMA3.Init.Mode = DMA_CIRCULAR;
	portPortDMA3.Init.Priority = DMA_PRIORITY_HIGH;		
		
	HAL_DMA_Init(&portPortDMA3);	
}

/*-----------------------------------------------------------*/

/* Streaming DMA 3 (port-to-port) setup */
void PortPortDMA3_Setup(UART_HandleTypeDef* huartSrc, UART_HandleTypeDef* huartDst, uint8_t num)
{		
	/* UART RX DMA (DMA2 Ch3) */
	if (huartSrc->Instance == USART1) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART1_RX);
	} else if (huartSrc->Instance == USART2) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART2_RX);
	} else if (huartSrc->Instance == USART3) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART3_RX);
	} else if (huartSrc->Instance == USART4) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART4_RX);
	} else if (huartSrc->Instance == USART5) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART5_RX);
	} else if (huartSrc->Instance == USART6) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART6_RX);
	} else if (huartSrc->Instance == USART7) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART7_RX);
	} else if (huartSrc->Instance == USART8) {
		__HAL_DMA1_REMAP(HAL_DMA2_CH3_USART8_RX);
	}		
	__HAL_LINKDMA(huartSrc,hdmarx,portPortDMA3);
	
	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch3_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);
	
	/* Start DMA stream	*/	
	huartSrc->State = HAL_UART_STATE_READY;
	HAL_UART_Receive_DMA(huartSrc, (uint8_t *)(&(huartDst->Instance->TDR)), num);
}

/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
