/**
  ******************************************************************************
  * File Name          : H08R0_dma.h
  * Description        : This file contains all the functions prototypes for 
  *                      the DMA  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __H08R0_dma_H
#define __H08R0_dma_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
	 
	 
/* Check which DMA interrupt occured */	 
#define HAL_DMA_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)  ((((__HANDLE__)->ISR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)


/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef portMemDMA1;
extern DMA_HandleTypeDef portMemDMA2;
extern DMA_HandleTypeDef portMemDMA3;
extern DMA_HandleTypeDef portPortDMA1;
extern DMA_HandleTypeDef portPortDMA2;
extern DMA_HandleTypeDef portPortDMA3;	 

extern uint32_t DMAStream1count;
extern uint32_t DMAStream2count;
extern uint32_t DMAStream3count;
extern uint32_t DMAStream1total;
extern uint32_t DMAStream2total;
extern uint32_t DMAStream3total;

extern UART_HandleTypeDef* dmaStreamDst[3];
	 
/* External function prototypes ----------------------------------------------*/
extern void MX_DMA_Init(void);
extern void PortMemDMA1_Setup(UART_HandleTypeDef* huart, uint8_t num);
extern void PortMemDMA2_Setup(UART_HandleTypeDef* huart, uint8_t num);
extern void PortMemDMA3_Setup(UART_HandleTypeDef* huart, uint8_t num);
extern void PortPortDMA1_Setup(UART_HandleTypeDef* huartSrc, UART_HandleTypeDef* huartDst, uint8_t num);
extern void PortPortDMA2_Setup(UART_HandleTypeDef* huartSrc, UART_HandleTypeDef* huartDst, uint8_t num);
extern void PortPortDMA3_Setup(UART_HandleTypeDef* huartSrc, UART_HandleTypeDef* huartDst, uint8_t num);
extern void StopPortPortDMA1(void);
extern void StopPortPortDMA2(void);
extern void StopPortPortDMA3(void);


#ifdef __cplusplus
}
#endif

#endif /* __H08R0_dma_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
