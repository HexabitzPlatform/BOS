/**
  ******************************************************************************
  * @file    H08R0_it.c
  * @brief   Interrupt Service Routines.
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

/* External variables --------------------------------------------------------*/


/* External function prototypes ----------------------------------------------*/
extern xTaskHandle xCommandConsoleTask;
extern void NotifyMessagingTaskFromISR(uint8_t port);



/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
	
	HAL_IncTick();
  osSystickHandler();  

}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart1)		
  HAL_UART_IRQHandler(&huart1);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	switch should be performed before the interrupt exists.  That ensures the
	unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------*/

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart2)	
  HAL_UART_IRQHandler(&huart2);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	switch should be performed before the interrupt exists.  That ensures the
	unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------*/

/**
* @brief This function handles USART3 to USART8 global interrupts / USART3 wake-up interrupt through EXTI line 28.
*/
void USART3_8_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart3)
	HAL_UART_IRQHandler(&huart3);
#endif
#if defined (_Usart4)
	HAL_UART_IRQHandler(&huart4);
#endif
#if defined (_Usart5)
	HAL_UART_IRQHandler(&huart5);
#endif
#if defined (_Usart6)
	HAL_UART_IRQHandler(&huart6);
#endif

	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	switch should be performed before the interrupt exists.  That ensures the
	unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*-----------------------------------------------------------*/


/**
* @brief This function handles DMA1 channel 1 interrupt (Uplink DMA 1).
*/
void DMA1_Ch1_IRQHandler(void)
{
	/* Streaming DMA 1 */
	HAL_DMA_IRQHandler(&portPortDMA1);
	if (DMAStream1total)
		++DMAStream1count;
	if (DMAStream1count >= DMAStream1total) {
		StopPortPortDMA1();
	}
	
}

/*-----------------------------------------------------------*/

/**
* @brief This function handles DMA1 channel 2 to 3 and DMA2 channel 1 to 2 interrupts.
*/
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void)
{
	/* Messaging DMA 3 */
	if (HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_TCIF2) == SET) {
		HAL_DMA_IRQHandler(&portMemDMA3);
	/* Streaming DMA 2 */
	} else if (HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_TCIF3) == SET) {
		HAL_DMA_IRQHandler(&portPortDMA2);
		if (DMAStream2total)
			++DMAStream2count;
		if (DMAStream2count >= DMAStream2total) {
			StopPortPortDMA2();
		}
	}
}

/*-----------------------------------------------------------*/

/**
* @brief This function handles DMA1 channel 4 to 7 and DMA2 channel 3 to 5 interrupts.
*/
void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void)
{
	/* Messaging DMA 1 */
	if (HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_TCIF5) == SET) {
		HAL_DMA_IRQHandler(&portMemDMA1);
	/* Messaging DMA 2 */
	} else if (HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_TCIF6) == SET) {
		HAL_DMA_IRQHandler(&portMemDMA2);
	/* Streaming DMA 3 */
	} else if (HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_TCIF3) == SET) {
		HAL_DMA_IRQHandler(&portPortDMA3);
		if (DMAStream3total)
			++DMAStream3count;
		if (DMAStream3count >= DMAStream3total) {
			StopPortPortDMA3();
		}
	}
}

/*-----------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Give back the mutex. */
	xSemaphoreGiveFromISR( PxTxSemaphoreHandle[GetPort(huart)], &( xHigherPriorityTaskWoken ) );
	
	UartTxReady = SET;
}

/*-----------------------------------------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char cRxedChar = 0; uint8_t port = GetPort(huart);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if (portStatus[port] != STREAM) 
	{
		/* Read buffer */
		cRxedChar = huart->Instance->RDR;
		
		/* Received CLI request? */
		if( cRxedChar == '\r' )
		{
			cRxedChar = '\0';
			PcPort = port; 
			portStatus[port] = CLI;
			
			/* Activate the CLI task */
			vTaskNotifyGiveFromISR(xCommandConsoleTask, &( xHigherPriorityTaskWoken ) );		
		}
		/* Received messaging request? (any value between 1 and 50 other than \r = 0x0D) */
		else if( (cRxedChar != '\0') && (cRxedChar <= 50) )
		{
			portStatus[port] = MSG;
			messageLength[port-1] = cRxedChar;			
				
			/* Activate DMA transfer */
			PortMemDMA1_Setup(huart, cRxedChar);
			
			cRxedChar = '\0';	
		}
		/* Message has been received? */
		else if( cRxedChar == 0x75 )
		{
			/* Notify messaging tasks */
			NotifyMessagingTaskFromISR(port);		
		}
		
		/* Give back the mutex */
		xSemaphoreGiveFromISR( PxRxSemaphoreHandle[port], &( xHigherPriorityTaskWoken ) );
		
		/* Read this port again */
		if (portStatus[port] == FREE) {
			HAL_UART_Receive_IT(huart, (uint8_t *)&cRxedChar, 1);
		}
	}
	
	UartRxReady = SET;
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
