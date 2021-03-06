/**
  ******************************************************************************
  * File Name          : BOS_freertos.c
  * Description        : Code for freertos applications
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
		MODIFIED by Hexabitz for BitzOS (BOS) V0.2.4 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"



/* Variables -----------------------------------------------------------------*/

/* Used in the run time stats calculations. */
static uint32_t ulClocksPer10thOfAMilliSecond = 0UL;

/* Tasks */
TaskHandle_t defaultTaskHandle = NULL;
TaskHandle_t UserTaskHandle = NULL;
TaskHandle_t BackEndTaskHandle = NULL;
TaskHandle_t xCommandConsoleTaskHandle = NULL;

#ifdef _P1
	TaskHandle_t P1MsgTaskHandle = NULL;
#endif
#ifdef _P2
	TaskHandle_t P2MsgTaskHandle = NULL;
#endif
#ifdef _P3
	TaskHandle_t P3MsgTaskHandle = NULL;
#endif
#ifdef _P4
	TaskHandle_t P4MsgTaskHandle = NULL;
#endif
#ifdef _P5
	TaskHandle_t P5MsgTaskHandle = NULL;
#endif
#ifdef _P6
	TaskHandle_t P6MsgTaskHandle = NULL;
#endif


/* Semaphores */
SemaphoreHandle_t PxRxSemaphoreHandle[7];
SemaphoreHandle_t PxTxSemaphoreHandle[7];

/* External Variables --------------------------------------------------------*/
//extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
////extern uint8_t UARTTxBuf[3][MSG_TX_BUF_SIZE];
//extern uint8_t crcBuffer[MAX_MESSAGE_SIZE];
//extern uint8_t UARTRxBufIndex[NumOfPorts];

/* Function prototypes -------------------------------------------------------*/
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void StartDefaultTask(void * argument);
void UserTask(void * argument);
void BackEndTask(void * argument);

/* BOS exported internal functions */
extern void PxMessagingTask(void * argument);
extern void prvCLITask( void *pvParameters );
extern void CheckAttachedButtons(void);
extern void ResetAttachedButtonStates(uint8_t *deferReset);
extern BOS_Status ExecuteSnippet(void);
extern void NotifyMessagingTask(uint8_t port);

/*-----------------------------------------------------------*/

/* Init FreeRTOS */
void MX_FREERTOS_Init(void) 
{
	/* Note: CMSIS OS priority levels are -3 to +3 and FreeRTOS priority levels are 0 to 6. Use osPriorityIdle to shift CMSIS priority levels to positive */
	
  /* Create a defaultTask */
  xTaskCreate(StartDefaultTask, (const char *) "DefaultTask", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal-osPriorityIdle, &defaultTaskHandle);	

	/* Create the back-end task */
	xTaskCreate(BackEndTask, (const char *) "BackEndTask", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal-osPriorityIdle, &BackEndTaskHandle);
	
	/* Create the User task */
	xTaskCreate(UserTask, (const char *) "UserTask", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal-osPriorityIdle, &UserTaskHandle);
	
	/* Register command line commands */
	vRegisterCLICommands();
	/* Create the CLI task */
	xTaskCreate(prvCLITask, "CliTask",	(2*configMINIMAL_STACK_SIZE),	NULL,	osPriorityNormal-osPriorityIdle, &xCommandConsoleTaskHandle);		
	
	
  /* Create message parsing tasks for module ports */
#ifdef _P1
  xTaskCreate(PxMessagingTask, (const char *) "P1MsgTask", configMINIMAL_STACK_SIZE, (void *) P1, osPriorityAboveNormal-osPriorityIdle, &P1MsgTaskHandle);
#endif
#ifdef _P2
	xTaskCreate(PxMessagingTask, (const char *) "P2MsgTask", configMINIMAL_STACK_SIZE, (void *) P2, osPriorityAboveNormal-osPriorityIdle, &P2MsgTaskHandle);
#endif
#ifdef _P3
	xTaskCreate(PxMessagingTask, (const char *) "P3MsgTask", configMINIMAL_STACK_SIZE, (void *) P3, osPriorityAboveNormal-osPriorityIdle, &P3MsgTaskHandle);
#endif
#ifdef _P4
	xTaskCreate(PxMessagingTask, (const char *) "P4MsgTask", configMINIMAL_STACK_SIZE, (void *) P4, osPriorityAboveNormal-osPriorityIdle, &P4MsgTaskHandle);
#endif
#ifdef _P5
	xTaskCreate(PxMessagingTask, (const char *) "P5MsgTask", configMINIMAL_STACK_SIZE, (void *) P5, osPriorityAboveNormal-osPriorityIdle, &P5MsgTaskHandle);
#endif
#ifdef _P6
	xTaskCreate(PxMessagingTask, (const char *) "P6MsgTask", configMINIMAL_STACK_SIZE, (void *) P6, osPriorityAboveNormal-osPriorityIdle, &P6MsgTaskHandle);
#endif

	/* Create semaphores to protect module ports (FreeRTOS vSemaphoreCreateBinary didn't work) */
#ifdef _P1
	osSemaphoreDef(SemaphoreP1); PxRxSemaphoreHandle[P1] = osSemaphoreCreate(osSemaphore(SemaphoreP1), 1);
	osSemaphoreDef(SemaphoreP2); PxTxSemaphoreHandle[P1] = osSemaphoreCreate(osSemaphore(SemaphoreP2), 1);
#endif
#ifdef _P2	
	osSemaphoreDef(SemaphoreP3); PxRxSemaphoreHandle[P2] = osSemaphoreCreate(osSemaphore(SemaphoreP3), 1);
	osSemaphoreDef(SemaphoreP4); PxTxSemaphoreHandle[P2] = osSemaphoreCreate(osSemaphore(SemaphoreP4), 1);
#endif
#ifdef _P3	
	osSemaphoreDef(SemaphoreP5); PxRxSemaphoreHandle[P3] = osSemaphoreCreate(osSemaphore(SemaphoreP5), 1);
	osSemaphoreDef(SemaphoreP6); PxTxSemaphoreHandle[P3] = osSemaphoreCreate(osSemaphore(SemaphoreP6), 1);
#endif
#ifdef _P4	
	osSemaphoreDef(SemaphoreP7); PxRxSemaphoreHandle[P4] = osSemaphoreCreate(osSemaphore(SemaphoreP7), 1);
	osSemaphoreDef(SemaphoreP8); PxTxSemaphoreHandle[P4] = osSemaphoreCreate(osSemaphore(SemaphoreP8), 1);
#endif
#ifdef _P5	
	osSemaphoreDef(SemaphoreP9); PxRxSemaphoreHandle[P5] = osSemaphoreCreate(osSemaphore(SemaphoreP9), 1);
	osSemaphoreDef(SemaphoreP10); PxTxSemaphoreHandle[P5] = osSemaphoreCreate(osSemaphore(SemaphoreP10), 1);
#endif
#ifdef _P6	
	osSemaphoreDef(SemaphoreP11); PxRxSemaphoreHandle[P6] = osSemaphoreCreate(osSemaphore(SemaphoreP11), 1);
	osSemaphoreDef(SemaphoreP12); PxTxSemaphoreHandle[P6] = osSemaphoreCreate(osSemaphore(SemaphoreP12), 1);
#endif
	
}

/*-----------------------------------------------------------*/

/* StartDefaultTask function */
void StartDefaultTask(void * argument)
{
	
  /* Infinite loop */
  for(;;)
  {
		/* Switch indicator LED according to mode */
		switch (indMode)
		{
			case IND_PING :
				RTOS_IND_blink(200);
				indMode = IND_OFF;
				break;
			
			case IND_TOPOLOGY :
				RTOS_IND_blink(100);
				indMode = IND_OFF;
				break;
			
			case IND_SHORT_BLINK :
				RTOS_IND_blink(30);
				indMode = IND_OFF;
				break;
			
			default:
				break;
		}
		
		/* Read button state */
		CheckAttachedButtons();
		
		/* Execute activated Command Snippets */
		ExecuteSnippet();
		
		/* Reset button state if no delay is needed by this module */
		if(needToDelayButtonStateReset != true)	delayButtonStateReset = false;
				
		taskYIELD();
  }
	
}

/*-----------------------------------------------------------*/

void vMainConfigureTimerForRunTimeStats( void )
{
	/* How many clocks are there per tenth of a millisecond? */
	ulClocksPer10thOfAMilliSecond = configCPU_CLOCK_HZ / 10000UL;
}

/*-----------------------------------------------------------*/

uint32_t ulMainGetRunTimeCounterValue( void )
{
uint32_t ulSysTickCounts, ulTickCount, ulReturn;
const uint32_t ulSysTickReloadValue = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
volatile uint32_t * const pulCurrentSysTickCount = ( ( volatile uint32_t *) 0xe000e018 );
volatile uint32_t * const pulInterruptCTRLState = ( ( volatile uint32_t *) 0xe000ed04 );
const uint32_t ulSysTickPendingBit = 0x04000000UL;

	/* NOTE: There are potentially race conditions here.  However, it is used
	anyway to keep the examples simple, and to avoid reliance on a separate
	timer peripheral. */


	/* The SysTick is a down counter.  How many clocks have passed since it was
	last reloaded? */
	ulSysTickCounts = ulSysTickReloadValue - *pulCurrentSysTickCount;

	/* How many times has it overflowed? */
	ulTickCount = xTaskGetTickCountFromISR();

	/* Is there a SysTick interrupt pending? */
	if( ( *pulInterruptCTRLState & ulSysTickPendingBit ) != 0UL )
	{
		/* There is a SysTick interrupt pending, so the SysTick has overflowed
		but the tick count not yet incremented. */
		ulTickCount++;

		/* Read the SysTick again, as the overflow might have occurred since
		it was read last. */
		ulSysTickCounts = ulSysTickReloadValue - *pulCurrentSysTickCount;
	}

	/* Convert the tick count into tenths of a millisecond.  THIS ASSUMES
	configTICK_RATE_HZ is 1000! */
	ulReturn = ( ulTickCount * 10UL ) ;

	/* Add on the number of tenths of a millisecond that have passed since the
	tick count last got updated. */
	ulReturn += ( ulSysTickCounts / ulClocksPer10thOfAMilliSecond );

	return ulReturn;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
