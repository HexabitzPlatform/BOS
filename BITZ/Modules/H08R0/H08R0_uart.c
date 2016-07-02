/**
  ******************************************************************************
  * File Name          : H08R0_uart.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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


FlagStatus UartRxReady = RESET;
FlagStatus UartTxReady = RESET;
uint8_t PcPort = 0;


/* USART1 init function */
#ifdef _Usart1
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_Init(&huart1);
	#if (H01R0 || H01R1) && (_P5pol_reversed)	
		huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	#endif	
  HAL_UART_Init(&huart1);
}
#endif

/* USART2 init function */
#ifdef _Usart2
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_Init(&huart2);
	#if (H01R0 || H01R1) && (_P2pol_reversed)	
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart2.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	#endif	
  HAL_UART_Init(&huart2);
}
#endif

/* USART3 init function */
#ifdef _Usart3
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  //huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_Init(&huart3);
	#if (H01R0 || H01R1) && (_P4pol_reversed)	
		huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart3.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	#endif	
  HAL_UART_Init(&huart3);
}
#endif

/* USART4 init function */
#ifdef _Usart4
void MX_USART4_UART_Init(void)
{
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 921600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_Init(&huart4);
	#if (H01R0 || H01R1) && (_P1pol_reversed)	
		huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	#endif	
  HAL_UART_Init(&huart4);
}
#endif

/* USART5 init function */
#ifdef _Usart5
void MX_USART5_UART_Init(void)
{
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 921600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart5.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_Init(&huart5);
	#if (H01R0 || H01R1) && (_P6pol_reversed)	
		huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart5.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	#endif	
  HAL_UART_Init(&huart5);
}
#endif

/* USART6 init function */
#ifdef _Usart6
void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
//  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart6.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	HAL_UART_Init(&huart6);
	#if (H01R0 || H01R1) && (_P3pol_reversed)	
		huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart6.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	#endif	
  HAL_UART_Init(&huart6);
}
#endif

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART1)
  {
	#ifdef _Usart1
    /* Peripheral clock enable */
    __USART1_CLK_ENABLE();
  
    /* USART1 GPIO Configuration */
    GPIO_InitStruct.Pin = USART1_TX_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART1_AF;	
    HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = USART1_RX_PIN;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART1_AF;
    HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
	#endif
  }
  else if(huart->Instance==USART2)
  {
	#ifdef _Usart2
    /* Peripheral clock enable */
    __USART2_CLK_ENABLE();
  
    /* USART2 GPIO Configuration */
    GPIO_InitStruct.Pin = USART2_TX_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART2_AF;	
    HAL_GPIO_Init(USART2_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = USART2_RX_PIN;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART2_AF;
    HAL_GPIO_Init(USART2_RX_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	#endif
  }
  else if(huart->Instance==USART3)
  {
	#ifdef _Usart3
    /* Peripheral clock enable */
    __USART3_CLK_ENABLE();
  
    /* USART3 GPIO Configuration */
    GPIO_InitStruct.Pin = USART3_TX_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART3_AF;	
    HAL_GPIO_Init(USART3_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = USART3_RX_PIN;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART3_AF;
    HAL_GPIO_Init(USART3_RX_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART3_8_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_8_IRQn);
	#endif
  }
  else if(huart->Instance==USART4)
  {
	#ifdef _Usart4
    /* Peripheral clock enable */
    __USART4_CLK_ENABLE();
  
    /* USART4 GPIO Configuration */
    GPIO_InitStruct.Pin = USART4_TX_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART4_AF;	
    HAL_GPIO_Init(USART4_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = USART4_RX_PIN;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART4_AF;
    HAL_GPIO_Init(USART4_RX_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART3_8_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_8_IRQn);
	#endif
  }
  else if(huart->Instance==USART5)
  {
	#ifdef _Usart5
    /* Peripheral clock enable */
    __USART5_CLK_ENABLE();
  
    /* USART5 GPIO Configuration */
    GPIO_InitStruct.Pin = USART5_TX_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART5_AF;	
    HAL_GPIO_Init(USART5_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = USART5_RX_PIN;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART5_AF;
    HAL_GPIO_Init(USART5_RX_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART3_8_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_8_IRQn);
	#endif
  }
  else if(huart->Instance==USART6)
  {
	#ifdef _Usart6
    /* Peripheral clock enable */
    __USART6_CLK_ENABLE();
  
    /* USART6 GPIO Configuration */
    GPIO_InitStruct.Pin = USART6_TX_PIN; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART6_AF;	
    HAL_GPIO_Init(USART6_TX_PORT, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = USART6_RX_PIN;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USART6_AF;
    HAL_GPIO_Init(USART6_RX_PORT, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART3_8_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_8_IRQn);
	#endif
  }
}

/* --- Blocking (polling-based) read protected with a semaphore --- 
*/
HAL_StatusTypeDef readPxMutex(uint8_t port, char *buffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
	if (GetUart(port) != NULL) {
		/* Wait for the semaphore to be available. */
		if (osSemaphoreWait(PxRxSemaphoreHandle[port], mutexTimeout) == osOK) {
			while( result != HAL_OK && result != HAL_TIMEOUT ) {
				result = HAL_UART_Receive(GetUart(port), (uint8_t *)buffer, n, portTimeout);
			}
			/* Give back the semaphore. */
			osSemaphoreRelease(PxRxSemaphoreHandle[port]);
		}
	}
	
	return result;
}

/* --- Blocking (polling-based) write protected with a semaphore --- 
*/
HAL_StatusTypeDef writePxMutex(uint8_t port, char *buffer, uint16_t n, uint32_t mutexTimeout, uint32_t portTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
	if (GetUart(port) != NULL) {
		/*/ Wait for the semaphore to be available. */
		if (osSemaphoreWait(PxTxSemaphoreHandle[port], mutexTimeout) == osOK) {
			while( result != HAL_OK && result !=  HAL_TIMEOUT ) {
				result = HAL_UART_Transmit(GetUart(port), (uint8_t *)buffer, n, portTimeout);
			}
			/* Give back the semaphore. */
			osSemaphoreRelease(PxTxSemaphoreHandle[port]);
		}
	}
	
	return result;
}

/* --- Non-blocking (interrupt-based) read protected with a semaphore --- 
*/
HAL_StatusTypeDef readPxITMutex(uint8_t port, char *buffer, uint16_t n, uint32_t mutexTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR; 
	
	if (GetUart(port) != NULL) {
		/* Wait for the mutex to be available. */
		if (osSemaphoreWait(PxRxSemaphoreHandle[port], mutexTimeout) == osOK) {
			result = HAL_UART_Receive_IT(GetUart(port), (uint8_t *)buffer, n);
		}
	}
	
	return result;
}

/* --- Non-blocking (interrupt-based) write protected with a semaphore --- 
*/
HAL_StatusTypeDef writePxITMutex(uint8_t port, char *buffer, uint16_t n, uint32_t mutexTimeout)
{
	HAL_StatusTypeDef result = HAL_ERROR; 

	if (GetUart(port) != NULL) {	
		/* Wait for the mutex to be available. */
		if (osSemaphoreWait(PxTxSemaphoreHandle[port], mutexTimeout) == osOK) {
			result = HAL_UART_Transmit_IT(GetUart(port), (uint8_t *)buffer, n);
		}
	}
	
	return result;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
