/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_freertos.c
 Description   : Code for freertos applications.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Variables -----------------------------------------------------------------*/
#define SPACE 32
#define null 0
#define RUN_FOR_ONCE 1
#define CONTINUOUS_RUN 2
#define INTIAL_VALUE 3
#define NonActive 0
#define SizeOfMatrix 20
#define SizeOfMatrix2d 15
#define NumberOfParameters 8
#define COMMAND_SIZE 64
#define ParameterLocationIn2dArray ProcessingParameter[0]
#define FirstCharacterInParameter nonProcessingParameter[0]
static uint32_t Monitor_time  __attribute__((section(".mySection")));
static uint8_t currentCharacter __attribute__((section(".mySection")));
static uint8_t flag __attribute__((section(".mySection")));
uint8_t finalMatrix[COMMAND_SIZE]={0};
uint8_t nonProcessingParameter[SizeOfMatrix]={0};
uint8_t ProcessingParameter[SizeOfMatrix]={0};
uint8_t twoDMatrix[NumberOfParameters][SizeOfMatrix2d]={0};
uint8_t perviousCharacter;
uint8_t numCommandParameters;
uint8_t digitTheCommand;
uint8_t	finalMatrixIndex;
uint8_t	twoDMatrixIndex;
uint8_t	counter;
uint8_t desiredArray;
uint8_t nonProcessingParameterIndex;
uint8_t processingParameterIndex;
uint8_t Monitor_index;
uint8_t WakeupFromStopFlag;

/* Used in the run time stats calculations. */
static uint32_t ulClocksPer10thOfAMilliSecond =0UL;

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

/* Function prototypes -------------------------------------------------------*/
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void StartDefaultTask(void *argument);
void UserTask(void *argument);
void BackEndTask(void *argument);
void ExecuteMonitor(void);
/* BOS exported internal functions */
extern void PxMessagingTask(void *argument);
extern void prvCLITask(void *pvParameters);
extern void CheckAttachedButtons(void);
extern void ResetAttachedButtonStates(uint8_t *deferReset);
extern void initialValue(void);
extern BOS_Status ExecuteSnippet(void);


extern void NotifyMessagingTask(uint8_t port);

extern IWDG_HandleTypeDef hiwdg;

/*-----------------------------------------------------------*/

/* Init FreeRTOS */
void MX_FREERTOS_Init(void){
	/* Note: CMSIS OS priority levels are -3 to +3 and FreeRTOS priority levels are 0 to 6. Use osPriorityIdle to shift CMSIS priority levels to positive */

	/* Create a defaultTask */
	xTaskCreate(StartDefaultTask,(const char* ) "DefaultTask",(DEFAULT_TASK_STACK_SIZE),NULL,osPriorityNormal - osPriorityIdle,&defaultTaskHandle);
	
	/* Create the back-end task */
	xTaskCreate(BackEndTask,(const char* ) "BackEndTask",(BACKEND_TASK_STACK_SIZE),NULL,osPriorityRealtime - osPriorityIdle,&BackEndTaskHandle);
	
	/* Create the User task */
	xTaskCreate(UserTask,(const char* ) "UserTask",(USER_TASK_STACK_SIZE),NULL,osPriorityNormal - osPriorityIdle,&UserTaskHandle);
	
	/* Register command line commands */
	vRegisterCLICommands();
	/* Create the CLI task */
	xTaskCreate(prvCLITask,"CliTask",(CLI_TASK_STACK_SIZE),NULL,osPriorityNormal - osPriorityIdle,&xCommandConsoleTaskHandle);
	
	/* Create message parsing tasks for module ports */
#ifdef _P1
	xTaskCreate(PxMessagingTask,(const char* ) "P1MsgTask",(PORT_TASK_STACK_SIZE),(void* ) P1,osPriorityAboveNormal - osPriorityIdle,&P1MsgTaskHandle);
#endif
#ifdef _P2
	xTaskCreate(PxMessagingTask,(const char* ) "P2MsgTask",(PORT_TASK_STACK_SIZE),(void* ) P2,osPriorityAboveNormal - osPriorityIdle,&P2MsgTaskHandle);
#endif
#ifdef _P3
	xTaskCreate(PxMessagingTask,(const char* ) "P3MsgTask",(PORT_TASK_STACK_SIZE),(void* ) P3,osPriorityAboveNormal - osPriorityIdle,&P3MsgTaskHandle);
#endif
#ifdef _P4
	xTaskCreate(PxMessagingTask,(const char* ) "P4MsgTask",(PORT_TASK_STACK_SIZE),(void* ) P4,osPriorityAboveNormal - osPriorityIdle,&P4MsgTaskHandle);
#endif
#ifdef _P5
	xTaskCreate(PxMessagingTask,(const char* ) "P5MsgTask",(PORT_TASK_STACK_SIZE),(void* ) P5,osPriorityAboveNormal - osPriorityIdle,&P5MsgTaskHandle);
#endif
#ifdef _P6
	xTaskCreate(PxMessagingTask,(const char* ) "P6MsgTask",(PORT_TASK_STACK_SIZE),(void* ) P6,osPriorityAboveNormal - osPriorityIdle,&P6MsgTaskHandle);
#endif
	
	/* Create semaphores to protect module ports (FreeRTOS vSemaphoreCreateBinary didn't work) */
#ifdef _P1
	osSemaphoreDef(SemaphoreP1);
	PxRxSemaphoreHandle[P1] =osSemaphoreCreate(osSemaphore(SemaphoreP1),1);
	osSemaphoreDef(SemaphoreP2);
	PxTxSemaphoreHandle[P1] =osSemaphoreCreate(osSemaphore(SemaphoreP2),1);
#endif
#ifdef _P2	
	osSemaphoreDef(SemaphoreP3);
	PxRxSemaphoreHandle[P2] =osSemaphoreCreate(osSemaphore(SemaphoreP3),1);
	osSemaphoreDef(SemaphoreP4);
	PxTxSemaphoreHandle[P2] =osSemaphoreCreate(osSemaphore(SemaphoreP4),1);
#endif
#ifdef _P3	
	osSemaphoreDef(SemaphoreP5);
	PxRxSemaphoreHandle[P3] =osSemaphoreCreate(osSemaphore(SemaphoreP5),1);
	osSemaphoreDef(SemaphoreP6);
	PxTxSemaphoreHandle[P3] =osSemaphoreCreate(osSemaphore(SemaphoreP6),1);
#endif
#ifdef _P4	
	osSemaphoreDef(SemaphoreP7);
	PxRxSemaphoreHandle[P4] =osSemaphoreCreate(osSemaphore(SemaphoreP7),1);
	osSemaphoreDef(SemaphoreP8);
	PxTxSemaphoreHandle[P4] =osSemaphoreCreate(osSemaphore(SemaphoreP8),1);
#endif
#ifdef _P5	
	osSemaphoreDef(SemaphoreP9);
	PxRxSemaphoreHandle[P5] =osSemaphoreCreate(osSemaphore(SemaphoreP9),1);
	osSemaphoreDef(SemaphoreP10);
	PxTxSemaphoreHandle[P5] =osSemaphoreCreate(osSemaphore(SemaphoreP10),1);
#endif
#ifdef _P6	
	osSemaphoreDef(SemaphoreP11);
	PxRxSemaphoreHandle[P6] =osSemaphoreCreate(osSemaphore(SemaphoreP11),1);
	osSemaphoreDef(SemaphoreP12);
	PxTxSemaphoreHandle[P6] =osSemaphoreCreate(osSemaphore(SemaphoreP12),1);
#endif
	
}

/*-----------------------------------------------------------*/

/* StartDefaultTask function */
void StartDefaultTask(void *argument){
	
	/* Infinite loop */
	for(;;){
		/* Switch indicator LED according to mode */
		switch(indMode){
			case IND_PING:
				RTOS_IND_blink(200);
				indMode =IND_OFF;
				break;
				
			case IND_TOPOLOGY:
				RTOS_IND_blink(100);
				indMode =IND_OFF;
				break;
				
			case IND_SHORT_BLINK:
				RTOS_IND_blink(30);
				indMode =IND_OFF;
				break;
				
			default:
				break;
				
		}
		
		/* Read button state */
		CheckAttachedButtons();
		
		/* Execute activated Command Snippets */
		ExecuteSnippet();
		/* Execute Monitor depending on CLI Commands  */
		ExecuteMonitor();

		/* System Clock Configuration restored after wake-up from STOP1 mode */
		if (WakeupFromStopFlag) {
			SystemClock_Config();
			WakeupFromStopFlag = 0;
			IND_blink(200);
		}

		/* 50 mS timeout IWDG timer */
		HAL_IWDG_Refresh(&hiwdg);

		/* Reset button state if no delay is needed by this module */
		if(needToDelayButtonStateReset != true)
			delayButtonStateReset = false;
		
		taskYIELD();
	}
	
}

/*-----------------------------------------------------------*/

void ExecuteMonitor(void)
{


	//We have three types of parameters to process:
	//1)first parameter:It's the parameter that has no specific location in the commands.
	//I'll send the beginning of the first parameter this character->'['
	//2)second parameter:It's the parameter that has specific location in the commands.
	//I'll send the beginning of the second parameter this character->'#'
	//3)third parameter:It's the first parameter in the command,and it's contain some information about the command,
	//Such as the order of the command and its number of parameter.
	//I'll send the beginning of the third parameter this character->'='
	//How the first parameter will be processed?
	//I will send a set of numbers beginning with this parameter.
	//These numbers include the commands numbers in which this parameter is located,
	//as well as the location of this parameter in this commands.
	//How the second parameter will be processed?
	//I will send before this parameter a number,
	//this number indicates the location of this parameter in the commands.
	//How the third parameter will be processed?
	//This parameter will contain information on command as I mentioned earlier,
	//so I will send with this parameter this informations.
	//Practical example of earlier:
	//We have the next commands:   on intensity
	//                             color colorname intensity
	//These two commands will be sent in the following way:
	//=120(on [1122]intensity
	//=230(color #3colorname [1222]intensity
    //=120(on:  =  the first parameter in the command
	//          1  order of command
	//          2  Number of command's parameters
	//          0  command Place in the 2dMatrix
	//          (  means \r
	//[1122]intensity: [  It's the parameter that has no specific location in the commands
	//                 11 means if the number of command 1 means that the location of the parameter is the first place in the 2dmatrix.
    //                 22 means if the number of command 2 means that the location of the parameter is the second place in the 2dmatrix.



    if(Monitor_time == INTIAL_VALUE)
    {

#if defined(H0FR7) ||  defined(H09R0)  || defined(H15R0)
initialValue();
#endif

	Monitor_time =0;
	flag=0;
	currentCharacter=SPACE;
	for (;;)
	{
		//giving initial value to currentCharacter and perviousCharacter to avoid writing in the nonProcessingParameter matrix  in case of non-transmission from STM32CubeMonitorIDE
		                    nonProcessingParameterIndex=0;
		              do
							{
		            	    perviousCharacter= INTIAL_VALUE;
		            	    Delay_us(100);
							if(currentCharacter != perviousCharacter)
							{
								//writing characters coming from STM32CubeMonitorIDE in nonProcessingParameter matrix
								nonProcessingParameter[nonProcessingParameterIndex++]=currentCharacter;
								perviousCharacter=currentCharacter;
								currentCharacter= INTIAL_VALUE;
							}
				            }
				while(perviousCharacter != SPACE && perviousCharacter != null && flag != RUN_FOR_ONCE && flag != CONTINUOUS_RUN);


                   if(flag == NonActive)
                   {
                	   nonProcessingParameterIndex=0;
 		              //first parameter:It's the parameter that has no specific location in the commands.
 		            if(FirstCharacterInParameter == '[')
 		            {
 		            	nonProcessingParameterIndex++;

 		            	for(;;)
 		            	{

 		            		if(nonProcessingParameter[nonProcessingParameterIndex]%10 == digitTheCommand)
 		            		{
 		            			nonProcessingParameterIndex++;
 		            			ParameterLocationIn2dArray=nonProcessingParameter[nonProcessingParameterIndex]%10;
 		            			break;
 		            		}
 		            		else
 		            		{
 		            			nonProcessingParameterIndex+=2;
 		            		}
 		            	}
 		            	while(nonProcessingParameter[nonProcessingParameterIndex] != ']')
 		            	{
 		            		nonProcessingParameterIndex++;
 		            	}
 		            	nonProcessingParameterIndex++;
 		            	processingParameterIndex=1;
 		            	memcpy(&ProcessingParameter[processingParameterIndex],&nonProcessingParameter[nonProcessingParameterIndex],SizeOfMatrix-nonProcessingParameterIndex);
 		            }

 		           //second parameter:It's the parameter that has specific location in the commands.
 		            else if(FirstCharacterInParameter == '#')
 		            {
 		            	ParameterLocationIn2dArray=nonProcessingParameter[1]%10;
 		            	nonProcessingParameterIndex=2;
 		            	processingParameterIndex=1;
 		            	memcpy(&ProcessingParameter[processingParameterIndex],&nonProcessingParameter[nonProcessingParameterIndex],SizeOfMatrix-nonProcessingParameterIndex);
 		            }

 		           //third parameter:It's the first parameter in the command
 		            else if(FirstCharacterInParameter == '=')
                	   {
                	    digitTheCommand=nonProcessingParameter[1]%10;
                	    numCommandParameters=nonProcessingParameter[2]%10;
                	    ParameterLocationIn2dArray=nonProcessingParameter[3]%10;
                	    nonProcessingParameterIndex=4;
                	    processingParameterIndex=1;
                	    memcpy(&ProcessingParameter[processingParameterIndex],&nonProcessingParameter[nonProcessingParameterIndex],SizeOfMatrix-nonProcessingParameterIndex);
                	   }


               	   desiredArray=ParameterLocationIn2dArray;
               	   memset (&twoDMatrix[desiredArray][0],0, SizeOfMatrix2d);
               	   memcpy(&twoDMatrix[desiredArray][0],&ProcessingParameter[0],SizeOfMatrix2d);
		           memset (&nonProcessingParameter[0],0, SizeOfMatrix);
		           memset (&ProcessingParameter[0],0, SizeOfMatrix);
                   }


                  //mode RUN_FOR_ONCE
                   if(flag == RUN_FOR_ONCE)
		           {
                	finalMatrixIndex=0;
                	twoDMatrixIndex=1;
		          	counter=0;
		           while(counter != numCommandParameters)
		          {
		            do
		          {
		            	finalMatrix[finalMatrixIndex++]=twoDMatrix[counter][twoDMatrixIndex++];
		            	Delay_ms(1);
		          }

	            while(finalMatrix[finalMatrixIndex-1] != null && finalMatrix[finalMatrixIndex-1] != SPACE);

		                   counter++;
		                   twoDMatrixIndex=1;
		          }
		          		   flag=0;
		          		   counter=0;

  	          		 for( Monitor_index=0;Monitor_index<COMMAND_SIZE;Monitor_index++)
		          		 					  {
		          			UARTRxBuf[2][Monitor_index]=finalMatrix[Monitor_index];
		          			Delay_ms(1);
		          		 					  }
  	          		memset (&finalMatrix[0],0, COMMAND_SIZE);
		           }



                   //mode CONTINUOUS_RUN
                   if(flag == CONTINUOUS_RUN)
		           {
                	finalMatrixIndex=0;
                	twoDMatrixIndex=1;
   		          	counter=0;
   		           while(counter != numCommandParameters)
   		          {
   		            do
   		          {
   		            	finalMatrix[finalMatrixIndex++]=twoDMatrix[counter][twoDMatrixIndex++];
   		            	Delay_ms(1);
   		          }

   	            while(finalMatrix[finalMatrixIndex-1] != null && finalMatrix[finalMatrixIndex-1] != SPACE);

   		             counter++;
   		             twoDMatrixIndex=1;
   		          }
		            counter=0;
                  while(flag != NonActive)
                  {
  	          		 for( Monitor_index=0;Monitor_index<COMMAND_SIZE;Monitor_index++)
		          		 					  {
		          			UARTRxBuf[2][Monitor_index]=finalMatrix[Monitor_index];
		          			Delay_us(200);
		          		 					  }
  	          		 Delay_ms(Monitor_time);
                  }
  	          		memset (&finalMatrix[0],0, COMMAND_SIZE);
  	          	    memset (&UARTRxBuf[2][0],0, MSG_RX_BUF_SIZE);
		           }
	}
  }
}


void vMainConfigureTimerForRunTimeStats(void){
	/* How many clocks are there per tenth of a millisecond? */
	ulClocksPer10thOfAMilliSecond = configCPU_CLOCK_HZ / 10000UL;
}

/*-----------------------------------------------------------*/

uint32_t ulMainGetRunTimeCounterValue(void){
	uint32_t ulSysTickCounts, ulTickCount, ulReturn;
	const uint32_t ulSysTickReloadValue =( configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
	volatile uint32_t *const pulCurrentSysTickCount =((volatile uint32_t* )0xe000e018);
	volatile uint32_t *const pulInterruptCTRLState =((volatile uint32_t* )0xe000ed04);
	const uint32_t ulSysTickPendingBit =0x04000000UL;
	
	/* NOTE: There are potentially race conditions here.  However, it is used
	 anyway to keep the examples simple, and to avoid reliance on a separate
	 timer peripheral. */

	/* The SysTick is a down counter.  How many clocks have passed since it was
	 last reloaded? */
	ulSysTickCounts =ulSysTickReloadValue - *pulCurrentSysTickCount;
	
	/* How many times has it overflowed? */
	ulTickCount =xTaskGetTickCountFromISR();
	
	/* Is there a SysTick interrupt pending? */
	if((*pulInterruptCTRLState & ulSysTickPendingBit) != 0UL){
		/* There is a SysTick interrupt pending, so the SysTick has overflowed
		 but the tick count not yet incremented. */
		ulTickCount++;
		
		/* Read the SysTick again, as the overflow might have occurred since
		 it was read last. */
		ulSysTickCounts =ulSysTickReloadValue - *pulCurrentSysTickCount;
	}
	
	/* Convert the tick count into tenths of a millisecond.  THIS ASSUMES
	 configTICK_RATE_HZ is 1000! */
	ulReturn =(ulTickCount * 10UL);
	
	/* Add on the number of tenths of a millisecond that have passed since the
	 tick count last got updated. */
	ulReturn +=(ulSysTickCounts / ulClocksPer10thOfAMilliSecond);
	
	return ulReturn;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
