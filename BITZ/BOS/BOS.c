/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : BOS.c
    Description   : Source code for Bitz Operating System (BOS).
		
		Required MCU resources : 
		
			>> Timer 7 for micro-sec delay.
			>> DMA1 Ch5, DMA1 Ch6, DMA2 Ch2 for port-to-memory messaging.
			>> DMA1 Ch1, DMA1 Ch3, DMA2 Ch3 for port-to-port streaming.
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint8_t myID = 0;
//uint8_t myID = _module;
uint16_t myPN = modulePN;
TIM_HandleTypeDef htim7;	/* micro-second delay counter */

/* Define module PN strings [available PNs+1][5 chars] */
const char modulePNstring[4][5] = {"", "H01R0", "H01R1", "H02R0"};
	
/* Number of modules in the array */
#ifndef _N
	uint8_t N = 1;						
#else
	uint8_t N = _N;
#endif

/* Routing and topology */
uint8_t portStatus[NumOfPorts+1] = {0};
uint16_t neighbors[NumOfPorts][2] = {0};
uint16_t neighbors2[MaxNumOfPorts][2] = {0};
#ifndef _N
	uint16_t array[MaxNumOfModules][MaxNumOfPorts+1] = {{0}};			/* Array topology */
	uint8_t routeDist[MaxNumOfModules] = {0}; 
	uint8_t routePrev[MaxNumOfModules] = {0}; 
	uint8_t route[MaxNumOfModules] = {0};
#else
	uint8_t routeDist[_N] = {0}; 
	uint8_t routePrev[_N] = {0}; 
	uint8_t route[_N] = {0};
#endif

/* Dimension the buffer into which the input messages is placed. */
uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE] = {0};
uint8_t messageLength[NumOfPorts] = {0};
uint8_t messageParams[20*(MAX_MESSAGE_SIZE-5)] = {0};
char cRxedChar = 0; 
uint8_t longMessage = 0; uint16_t longMessageLastPtr = 0;
static uint8_t longMessageScratchpad[(MaxNumOfPorts+1)*MaxNumOfModules] = {0};
	
/* Messaging tasks */
extern TaskHandle_t FrontEndTaskHandle;
#ifdef _P1
extern TaskHandle_t P1MsgTaskHandle;
#endif
#ifdef _P2
extern TaskHandle_t P2MsgTaskHandle;
#endif
#ifdef _P3
extern TaskHandle_t P3MsgTaskHandle;
#endif
#ifdef _P4
extern TaskHandle_t P4MsgTaskHandle;
#endif
#ifdef _P5
extern TaskHandle_t P5MsgTaskHandle;
#endif
#ifdef _P6
extern TaskHandle_t P6MsgTaskHandle;
#endif

/* Private function prototypes -----------------------------------------------*/	

uint8_t minArr(uint8_t* arr, uint8_t* Q);
uint8_t QnotEmpty(uint8_t* Q);
void NotifyMessagingTask(uint8_t port);

/* Prototype commands functions ----------------------------------------------*/
/*
 * Implements the task-stats command.
 */
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/*
 * Implements the run-time-stats command.
 */
static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/*
 * Implements the ping command.
 */
static portBASE_TYPE pingCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/*
 * Implements the update command.
 */
static portBASE_TYPE bootloaderUpdateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/*
 * Implements the explore command.
 */
static portBASE_TYPE exploreCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );



/* Create CLI commands --------------------------------------------------------*/

/* CLI command structure : run-time-stats 
This generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t prvRunTimeStatsCommandDefinition =
{
	( const int8_t * ) "run-time-stats", /* The command string to type. */
	( const int8_t * ) "run-time-stats:\r\n Display a table showing how much processing time each FreeRTOS task has used\r\n\r\n",
	prvRunTimeStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : task-stats" 
This generates a table that gives information on each task in the system. */
static const CLI_Command_Definition_t prvTaskStatsCommandDefinition =
{
	( const int8_t * ) "task-stats", /* The command string to type. */
	( const int8_t * ) "task-stats:\r\n Display a table showing the state of each FreeRTOS task\r\n\r\n",
	prvTaskStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ping */
static const CLI_Command_Definition_t pingCommandDefinition =
{
	( const int8_t * ) "ping", /* The command string to type. */
	( const int8_t * ) "ping:\r\n Ping a module\r\n\r\n",
	pingCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : update */
static const CLI_Command_Definition_t bootloaderUpdateCommandDefinition =
{
	( const int8_t * ) "update", /* The command string to type. */
	( const int8_t * ) "update:\r\n Put the module in bootloader mode to update its firmware\r\n\r\n",
	bootloaderUpdateCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : explore */
static const CLI_Command_Definition_t exploreCommandDefinition =
{
	( const int8_t * ) "explore", /* The command string to type. */
	( const int8_t * ) "explore:\r\n Explore the array and build its topology\r\n\r\n",
	exploreCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/

/* Define long messages -------------------------------------------------------*/

static char * pcBootloaderUpdateMessage = 	\
"\n\rThis module will be forced into bootloader mode.\n\rPlease use the \"STM Flash Loader Demonstrator\" \
utility to update the firmware.\n\r\n\t*** Important ***\n\rIf this module is connected directly to PC please close this port first.\n\r";	


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/


/* PxMessagingTask function 
*/
void PxMessagingTask(void * argument)
{
uint8_t port, src, dst, temp; uint16_t code;
	
	port = (int8_t)(unsigned) argument;
	
	 /* Infinite loop */
	for( ;; )
	{
		
		/* Wait for ever until a message is received on one of the ports */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			
		if (messageLength[port-1])
		{						
			/* Read message source and destination */
			dst = cMessage[port-1][0]; 
			src = cMessage[port-1][1];	
			
			/* Read message code */
			code = ( (uint16_t) cMessage[port-1][2] << 8 ) + cMessage[port-1][3];	
			
			/* Is it a long message? Check MSB */
			if (code > 16) {
				longMessage = 1;
				code &= 0x7FFF;
			} else {
				longMessage = 0;
			}
			
			/* Check the end of message char 0x75 */
			if (cMessage[port-1][messageLength[port-1]-1] == 0x75)
			{
				/* Is it a transit message? */
				if ( ( dst && (dst != myID) && (myID != 1) ) || ( dst && (dst != myID) && (myID == 1) && (code != CODE_module_id) ) )
				{
					/* Is it a broadcast message? */
					if (dst == 0xFF) 
						BroadcastReceivedMessage(port);
					/* Forward the message to its destination */
					else			
						ForwardReceivedMessage(port);
				}
				else 
				{
							
					/* Execute required tasks */
					switch (code)
					{
						case CODE_ping :
							RTOS_IND_blink(200);
							break;
						
						case CODE_IND_toggle :
							IND_toggle();
							break;
						
						case CODE_hi :					
							/* Record your neighbor info */
							neighbors[port-1][0] = ( (uint16_t) src << 8 ) + cMessage[port-1][6];			/* Neighbor ID + Neighbor own port */
							neighbors[port-1][1] = ( (uint16_t) cMessage[port-1][4] << 8 ) + cMessage[port-1][5];		/* Neighbor PN */
							/* Send your own info */
							messageParams[1] = (uint8_t) myPN;
							messageParams[0] = (uint8_t) (myPN >> 8);	
							messageParams[2] = port;
							osDelay(2);
							/* Port, Destination = 0 (adjacent neighbor), message code, number of parameters */
							SendMessageFromPort(port, 0, CODE_hi_response, 3);
							break;
						
						case CODE_hi_response :
							/* Record your neighbor info */
							neighbors[port-1][0] = ( (uint16_t) src << 8 ) + cMessage[port-1][6];		/* Neighbor ID + Neighbor own port */
							neighbors[port-1][1] = ( (uint16_t) cMessage[port-1][4] << 8 ) + cMessage[port-1][5];		/* Neighbor PN */						
							break;
						
						case CODE_task_stats :
							break;
						
						case CODE_run_time_stats :
							break;
						
						case CODE_explore_adj :
							ExploreNeighbors(port);
							osDelay(10); temp = 0;
							/* Exploration response message */
							for (uint8_t p=1 ; p<=NumOfPorts ; p++)  
							{
								if (neighbors[p-1][0])
								{
									messageParams[temp] = p;
									memcpy(messageParams+temp+1, neighbors[p-1], (size_t)(4));
									temp += 5;		
								}
							}
							/* Port, Destination = 0 (adjacent neighbor), message code, number of parameters */
							SendMessageFromPort(port, 0, CODE_explore_adj_response, temp);
							break;
						
						case CODE_explore_adj_response :
							/* Extract the other module neighbors */
							temp = (messageLength[port-1]-5)/5;
							for (uint8_t k=0 ; k<temp ; k++)  {
								memcpy(&neighbors2[(cMessage[port-1][4+k*5])-1][0], &cMessage[port-1][5+k*5], (size_t)(4));
							}
							break;
						
						case CODE_port_dir :
							/* Reverse all ports other than the input one */
							for (uint8_t p=1 ; p<=NumOfPorts ; p++) {
								if (p != port)	SwapUartPins(GetUart(p), cMessage[port-1][4]);
							}
							break;
							
						case CODE_module_id :
							if (cMessage[port-1][4] == 0)						/* Change my own ID */
								myID = cMessage[port-1][5];
							else if (cMessage[port-1][4] == 1) {		/* Change my neighbor's ID */
								messageParams[0] = 0;											/* change own ID */
								messageParams[1] = cMessage[port-1][5];		/* The new ID */
								SendMessageFromPort(cMessage[port-1][6], 0, CODE_module_id, 3);
							}
							break;
							
						case CODE_topology :
							if (longMessage) {
								/* array is 2-byte oriented thus memcpy can copy only even number of bytes */
								/* Use a 1-byte oriented scratchpad */
								memcpy(&longMessageScratchpad[0]+longMessageLastPtr, &cMessage[port-1][4], (size_t) (messageLength[port-1]-5) );
								longMessageLastPtr += messageLength[port-1]-5;
							} else {
								memcpy(&longMessageScratchpad[0]+longMessageLastPtr, &cMessage[port-1][4], (size_t) (messageLength[port-1]-5) );
								longMessageLastPtr += messageLength[port-1]-5;
								N = (longMessageLastPtr / (MaxNumOfPorts+1)) / 2;
								/* Copy the scratchpad to array */
								memcpy(&array, &longMessageScratchpad, longMessageLastPtr);
								longMessageLastPtr = 0;
								RTOS_IND_blink(100);
							}
							break;
						
					#if defined (H01R0) || defined (H01R1)
						case CODE_H01R0_on :
							RGB_LED_on(cMessage[port-1][4]);
							break;
						
						case CODE_H01R0_off :
							RGB_LED_off();
							break;
						
						case CODE_H01R0_toggle :
							if (RGB_LED_State)
								RGB_LED_off();
							else
								RGB_LED_on(cMessage[port-1][4]);
							break;
					#endif
							
						default:
							break;
					}
					
				}

			}	
			
		}
		
		/* Reset message buffer */
		memset(cMessage[port-1], 0, (size_t) messageLength[port-1]);
		messageLength[port-1] = 0;
		/* Free the port */
		portStatus[port] = FREE;
		/* Read this port again */
		HAL_UART_Receive_IT(GetUart(port), (uint8_t *)&cRxedChar, 1);
		
		taskYIELD();
	}

}

/*-----------------------------------------------------------*/

/* TIM7 init function - 1 usec timebase 16-bit 
*/
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM7_CLK_ENABLE();

	/* Peripheral configuration */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 48;
  htim7.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim7.Init.Period = 1;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/*-----------------------------------------------------------*/

/* --- Used by FoundRoute: Find the index of the minimum module in dist that is still unvisited 
*/
uint8_t minArr(uint8_t* arr, uint8_t* Q)
{
	uint8_t smallest = 0xFF; uint8_t index = 0;

	/* Consider first element as smallest */
	if (!Q[0])						// Not visited yet
		smallest = arr[0];

	for (int i=0 ; i<N ; i++) {
		if ((arr[i] < smallest) && !Q[i]) {
			smallest = arr[i];
			index = i;
		}
	}
	
	return index;
}

/*-----------------------------------------------------------*/

/* --- Used by FoundRoute: Check if Q is empty (all modules have been visited) 
*/
uint8_t QnotEmpty(uint8_t* Q)
{		
	char temp = 1;

	for (int i=0 ; i<N ; i++) {
		temp &= Q[i];
	}	
	
	return temp;
}

/*-----------------------------------------------------------*/

/* --- Forward a received message to its destination 
*/
BOS_Status ForwardReceivedMessage(uint8_t incomingPort)
{
	BOS_Status result = BOS_OK;
	uint8_t length = 0, port = 0; uint16_t code = 0;
	
	/* Message length */
	length = messageLength[incomingPort-1];
	/* Message code */
	code = ( (uint16_t) cMessage[incomingPort-1][2] << 8 ) + cMessage[incomingPort-1][3];
	/* Message parameters */
	memcpy(messageParams, &cMessage[incomingPort-1][4], (size_t) (length-5) );
	
	/* Find best output port for destination module */
	port = FindRoute(myID, cMessage[incomingPort-1][0]); 
	
	/* Transmit the message from this port */
	SendMessageFromPort(port, cMessage[incomingPort-1][0], code, length-5);	

	return result;	
}

/*-----------------------------------------------------------*/

void NotifyMessagingTaskFromISR(uint8_t port)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	switch (port)
	{
	#ifdef _P1
		case P1 : 
			vTaskNotifyGiveFromISR(P1MsgTaskHandle, &( xHigherPriorityTaskWoken ) );	
	#endif
	#ifdef _P2
		case P2 :
			vTaskNotifyGiveFromISR(P2MsgTaskHandle, &( xHigherPriorityTaskWoken ) );	
	#endif
	#ifdef _P3
		case P3 :
			vTaskNotifyGiveFromISR(P3MsgTaskHandle, &( xHigherPriorityTaskWoken ) );	
	#endif
	#ifdef _P4
		case P4 :
			vTaskNotifyGiveFromISR(P4MsgTaskHandle, &( xHigherPriorityTaskWoken ) );	
	#endif
	#ifdef _P5
		case P5 :
			vTaskNotifyGiveFromISR(P5MsgTaskHandle, &( xHigherPriorityTaskWoken ) );	
	#endif
	#ifdef _P6
		case P6 :
			vTaskNotifyGiveFromISR(P6MsgTaskHandle, &( xHigherPriorityTaskWoken ) );	
	#endif
		default:
			;
	}		
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- BitzOS initialization. 
*/
void BOS_Init(void)
{
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM7_Init();
	
	/* Startup indicator sequence */
	IND_blink(500);
	HAL_Delay(100);
	IND_blink(100);
	HAL_Delay(100);
	IND_blink(100);	
	
	/* Initialize the module */
#ifdef  Module_Init
	Module_Init();
#endif
	
}

/*-----------------------------------------------------------*/

/* Register the commands.
*/
void vRegisterCLICommands(void)
{
	/* Register all CLI commands */
	FreeRTOS_CLIRegisterCommand( &prvTaskStatsCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &prvRunTimeStatsCommandDefinition );	
	FreeRTOS_CLIRegisterCommand( &pingCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &bootloaderUpdateCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &exploreCommandDefinition );
	
#ifdef H01R0	
	FreeRTOS_CLIRegisterCommand( &onCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &offCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &colorCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &RGBCommandDefinition );
#endif	
}

/*-----------------------------------------------------------*/

/* --- Get the UART for a given port. 
*/
UART_HandleTypeDef* GetUart(uint8_t port)
{
	switch (port)
	{
	#ifdef _P1
		case P1 : 
			return P1uart;	
	#endif
	#ifdef _P2
		case P2 :
			return P2uart;
	#endif
	#ifdef _P3
		case P3 :
			return P3uart;
	#endif
	#ifdef _P4
		case P4 :
			return P4uart;
	#endif
	#ifdef _P5
		case P5 :
			return P5uart;
	#endif
	#ifdef _P6
		case P6 :
			return P6uart;
	#endif
	#ifdef _P7
		case P7 :
			return P7uart;
	#endif
	#ifdef _P8
		case P8 :
			return P8uart;
	#endif
	#ifdef _P9
		case P9 :
			return P9uart;
	#endif
	#ifdef _P10
		case P10 :
			return P10uart;
	#endif
		default:
			return 0;
	}		
}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
#ifdef H01R0
	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART6)
		return P3;
	else if (huart->Instance == USART3)
		return P4;
	else if (huart->Instance == USART1)
		return P5;
	else if (huart->Instance == USART5)
		return P6;
#endif
//#if (HO01R2 || HO02R1)
//		if (huart->Instance == USART2)
//				return P1;
//		else if (huart->Instance == USART6)
//				return P2;
//		else if (huart->Instance == USART3)
//				return P3;
//		else if (huart->Instance == USART5)
//				return P4;
//		else if (huart->Instance == USART1)
//				return P5;
//		else if (huart->Instance == USART4)
//				return P6;
//#endif
//#if (PO01R0 || PO02R0)
//		if (huart->Instance == USART5)
//				return P1;
//		else if (huart->Instance == USART2)
//				return P2;
//		else if (huart->Instance == USART3)
//				return P3;
//		else if (huart->Instance == USART8)
//				return P4;
//		else if (huart->Instance == USART4)
//				return P5;
//#endif
//		
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Send a message to another module 
*/
BOS_Status SendMessageToModule(uint8_t dst, uint16_t code, uint16_t numberOfParams)
{
	BOS_Status result = BOS_OK;
	uint8_t port = 0; 
	
	/* Singlecast message */
	if (dst != 0xFF)
	{
		/* Find best output port for destination module */
		port = FindRoute(myID, dst); 
		
		/* Transmit the message from this port */
		SendMessageFromPort(port, dst, code, numberOfParams);	
	}
	/* Broadcast message */
	else
	{
		for( uint8_t port=1 ; port <= NumOfPorts ; port++ )
		{
			/* Transmit the message from this port */
			SendMessageFromPort(port, dst, code, numberOfParams);	
		}
	}

	return result;
}

/*-----------------------------------------------------------*/

/* --- Send a message from a specific port 
*/
BOS_Status SendMessageFromPort(uint8_t port, uint8_t dst, uint16_t code, uint16_t numberOfParams)
{
	BOS_Status result = BOS_OK; 
	uint8_t length = 0; static uint8_t totalNumberOfParams = MAX_MESSAGE_SIZE-5; static uint16_t ptrShift = 0;
	char message[MAX_MESSAGE_SIZE] = {0};
	
	/* Increase the priority of current running task */
	vTaskPrioritySet( NULL, osPriorityHigh );
	
	/* Construct the message */
	message[0] = dst;						
	message[1] = myID;
	message[3] = (uint8_t) code;
	message[2] = (uint8_t) (code >> 8);	
	
	/* Copy parameters */
	if (numberOfParams <= (MAX_MESSAGE_SIZE-5) ) {				
		memcpy((char*)&message[4], (&messageParams[0]+ptrShift), numberOfParams);
		/* Calculate message length */
		length = numberOfParams + 5;
	} else {
		/* Toggle code MSB to inform receiver of a long message */
		code |= 0x8000;		
		totalNumberOfParams = numberOfParams;
		numberOfParams = MAX_MESSAGE_SIZE-5;
		/* Break into multiple messages */
		while (totalNumberOfParams != 0)
		{		
			if ( (totalNumberOfParams/numberOfParams) >= 1) 
			{	
				/* Call this function recursively */
				SendMessageFromPort(port, dst, code, numberOfParams);
				osDelay(10);
				/* Reset messageParams buffer */
				memset( (&messageParams[0]+ptrShift), 0, numberOfParams );
				/* Update remaining number of parameters */
				totalNumberOfParams -= numberOfParams;
				ptrShift += numberOfParams;
			} 
			else 
			{
				code &= 0x7FFF;		/* Last message */
				numberOfParams = totalNumberOfParams;
				memcpy((char*)&message[4], (&messageParams[0]+ptrShift), numberOfParams);
				/* Reset messageParams buffer */
				memset( (&messageParams[0]+ptrShift), 0, numberOfParams );
				ptrShift = 0; totalNumberOfParams = 0;
				/* Calculate message length */
				length = numberOfParams + 5;
			}
		}
	}	
	
	/* 0x75 End of message */
	message[length-1] = 0x75;		
	
	/* Give back the semaphore if needed. */
	osSemaphoreRelease(PxRxSemaphoreHandle[port]);
	
	/* Transmit message length byte */
	GetUart(port)->Instance->TDR = length;

	/* Wait some time */
	Delay_us(1500);
	
	/* Transmit the message */
	writePxMutex(port, message, length, cmd50ms, 20);	

	/* Put the priority of current running task back to normal */
	vTaskPrioritySet( NULL, osPriorityNormal );

	/* Read this port again */
	HAL_UART_Receive_IT(GetUart(port), (uint8_t *)&cRxedChar, 1);
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Broadcast a received message to all connected modules 
*/
BOS_Status BroadcastReceivedMessage(uint8_t incomingPort)
{
	BOS_Status result = BOS_OK;
	uint8_t length = 0; uint16_t code = 0;
	
	/* Use broadcast plans */
	
	/* Message length */
	length = messageLength[incomingPort-1];
	/* Message code */
	memcpy(&code, &cMessage[incomingPort-1][2], 2);
	/* Message parameters */
	memcpy(messageParams, &cMessage[incomingPort-1][4], (size_t) (length-5) );
	
	/* Transmit to all broadcast ports */
	for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
	{
		if (port != incomingPort) 	/* Also check for broadcast ports from broadacast plans */	
		{
			/* Transmit the message from this port */
			SendMessageFromPort(port, 0xFF, code, length-5);	
		}	
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Explore the array and create its topology (executed only by master)
*/
BOS_Status Explore(void)
{
	BOS_Status result = BOS_OK;
	uint8_t currentID = 0, lastID = 0, temp1 = 0, temp2 = 0;
	uint16_t temp16 = 0;
	
	myID = 1; 		/* Master ID */
	
	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
	
	for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
	{
		if (port != PcPort)	SwapUartPins(GetUart(port), REVERSED);
	}
	ExploreNeighbors(PcPort);

	
	/* >>> Step 2 - Assign IDs to new modules & update the topology array */
	
	/* Step 2a - Assign IDs to new modules */
	currentID = 1;
	for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
	{
		if (neighbors[port-1][0])
		{
			/* New ID */
			messageParams[1] = ++currentID;
			N = currentID;			/* Update number of modules in the array */
			/* Inform module to change ID */
			messageParams[0] = 0;		/* change own ID */
			SendMessageFromPort(port, 0, CODE_module_id, 3);			
			/* Modify neighbors table */
			neighbors[port-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(neighbors[port-1][0]);
			osDelay(10);
		}
	}
	
	/* Step 2b - Update master topology array */
	array[0][0]	= myPN;					
	for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
	{
		if (neighbors[port-1][0])
		{
			temp16 = neighbors[port-1][0];
			temp1 = (uint8_t)(temp16>>8);										/* Neighbor ID */
			temp2 = (uint8_t)(neighbors[port-1][0]);				/* Neighbor port */
			/* Module 1 (master) */
			array[0][port] = ( temp1 << 3 ) | temp2;				/* Neighbor ID | Neighbor port */
			/* Rest of the neighbors */
			array[temp1-1][0]	= neighbors[port-1][1];				/* Neighbor PN */
			array[temp1-1][temp2] = ( myID << 3 ) | port;		/* Module 1 ID | Module 1 port */
		}
	}		
	
	
	/* >>> Step 3 - Ask each new module to explore and repeat */
	
	while (lastID != currentID)
	{
		/* Update lastID */
		lastID = currentID;
		
		/* Scan all discovered modules */
		for (uint8_t i=2 ; i<=currentID ; i++) 
		{
			/* Step 3a - Ask the module to reverse ports */
			messageParams[0] = REVERSED;
			SendMessageToModule(i, CODE_port_dir, 1);
			osDelay(10);
			
			/* Step 3b - Ask the module to explore adjacent neighbors */
			SendMessageToModule(i, CODE_explore_adj, 0);
			osDelay(100);		
		
			/* Step 3c - Assign IDs to new modules */
			for (uint8_t j=1 ; j<=MaxNumOfPorts ; j++) 
			{
				temp16 = neighbors2[j-1][0];		/* Neighbor ID */
				temp1 = (uint8_t)(temp16>>8);											
				if (temp16 != 0 && temp1 == 0)			/* UnIDed module */
				{
					/* New ID */
					messageParams[1] = ++currentID;		
					N = currentID;			/* Update number of modules in the array */
					/* Modify neighbors table */
					neighbors2[j-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(neighbors2[j-1][0]);
					/* Ask the module to ID its yet unIDed neighbors */
					messageParams[0] = 1;			/* change neighbor ID */
					messageParams[2] = j;		/* neighbor port */
					SendMessageToModule(i, CODE_module_id, 3);
					osDelay(10);
				}
			}
			
			/* Step 3d - Update master topology array */
			for (uint8_t j=1 ; j<=MaxNumOfPorts ; j++) 
			{
				if (neighbors2[j-1][0])
				{	
					temp16 = neighbors2[j-1][0];
					temp1 = (uint8_t)(temp16>>8);										/* Neighbor ID */
					temp2 = (uint8_t)(neighbors2[j-1][0]);					/* Neighbor port */		
					if (temp1 != 1)			/* Execlude the master */
					{
						/* Update module i section */
						if (array[i-1][j] == 0) {
							array[i-1][j] = ( temp1 << 3 ) | temp2;				/* Neighbor ID | Neighbor port */
						}
						/* Update module i neighbors */
						if (array[temp1-1][temp2] == 0) {
							array[temp1-1][0]	= neighbors2[j-1][1];				/* Neighbor PN */
							array[temp1-1][temp2] = ( i << 3 ) | j;				/* Module i ID | Module i port */								
						}
					}
				}
			}	
			
			/* Reset neighbors2 array */
			memset(neighbors2, 0, sizeof(neighbors2) );
			
			/* Step 3e - Ask the module to update its topology array */
			memcpy(messageParams, array, (size_t) (currentID*(MaxNumOfPorts+1)*2) );
			SendMessageToModule(i, CODE_topology, (size_t) (currentID*(MaxNumOfPorts+1)*2));
			osDelay(60);
		}
	}

	
	/* >>> Step 4 - Make sure all connected modules have been discovered */
	
//	ExploreNeighbors(PcPort);
//	/* Check for any unIDed neighbors */
//	for (uint8_t i=1 ; i<=NumOfPorts ; i++) 
//	{
//		temp16 = neighbors[i-1][0];		/* Neighbor ID */
//		temp1 = (uint8_t)(temp16>>8);											
//		if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
//			result = BOS_ERR_UnIDedModule;
//		}		
//	}
//	/* Ask other modules for any unIDed neighbors */
//	for (uint8_t i=2 ; i<=currentID ; i++) 
//	{
//		SendMessageToModule(i, CODE_explore_adj, 0);
//		osDelay(100);	
//		/* Check for any unIDed neighbors */
//		for (uint8_t j=1 ; j<=MaxNumOfPorts ; j++) 
//		{
//			temp16 = neighbors2[j-1][0];		/* Neighbor ID */
//			temp1 = (uint8_t)(temp16>>8);											
//			if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
//				result = BOS_ERR_UnIDedModule;
//			}
//		}				
//	}
	
	
	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
	
//	if (result == BOS_OK)
//	{
//		memcpy(messageParams, array, (size_t) (currentID*(MaxNumOfPorts+1)*2) );
//		SendMessageToModule(0xFF, CODE_topology, (size_t) (currentID*(MaxNumOfPorts+1)*2));		
//	}
	
	/* >>> Step 6 - Build and broadcast broadcast plans */
	

	return result;
}

/*-----------------------------------------------------------*/

/* --- Explore adjacent neighbors 
*/
BOS_Status ExploreNeighbors(uint8_t ignore)
{
	BOS_Status result = BOS_OK; 

	/* Send Hi messages to adjacent neighbors */
	for (uint8_t port=1 ; port<=NumOfPorts ; port++)  
	{
		if (port != ignore) 
		{
			/* This module info */
			messageParams[0] = (uint8_t) (myPN >> 8);
			messageParams[1] = (uint8_t) myPN;
			messageParams[2] = port;
			/* Port, Destination = 0 (adjacent neighbor), message code, number of parameters */
			SendMessageFromPort(port, 0, CODE_hi, 3);
			/* Minimum delay between two consequetive SendMessage commands (with response) */
			osDelay(10);
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
*/
void StartMicroDelay(uint16_t Delay)
{
	portENTER_CRITICAL();
	
	if (Delay)
	{
		htim7.Instance->ARR = Delay;
	
		HAL_TIM_Base_Start(&htim7);	

		while(htim7.Instance->CNT != 0)
		{
		}
		
		HAL_TIM_Base_Stop(&htim7);
	}
	
	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Swap UART pins ( NORMAL | REVERSED )--- 
*/
void SwapUartPins(UART_HandleTypeDef *huart, uint8_t direction)
{
	if (direction == REVERSED) {
		huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
		HAL_UART_Init(huart);
	} else if (direction == NORMAL) {
		huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_DISABLE;
		HAL_UART_Init(huart);		
	}
}

/*-----------------------------------------------------------*/

/* --- Find the shortest route to a module using Dijkstra's algorithm ---
 
Algorithm (from Wikipedia):

1- Assign to every node a tentative distance value: set it to zero for our initial node 
and to infinity for all other nodes.

2- Set the initial node as current. Mark all other nodes unvisited. Create a set of all 
the unvisited nodes called the unvisited set.

3- For the current node, consider all of its unvisited neighbors and calculate their tentative
distances. Compare the newly calculated tentative distance to the current assigned value and 
assign the smaller one. For example, if the current node A is marked with a distance of 6, 
and the edge connecting it with a neighbor B has length 2, then the distance to B (through A) 
will be 6 + 2 = 8. If B was previously marked with a distance greater than 8 then change it to 8. 
Otherwise, keep the current value.

4- When we are done considering all of the neighbors of the current node, mark the current 
node as visited and remove it from the unvisited set. A visited node will never be checked again.

5- If the destination node has been marked visited (when planning a route between two specific 
nodes) or if the smallest tentative distance among the nodes in the unvisited set is infinity 
(when planning a complete traversal; occurs when there is no connection between the initial 
node and remaining unvisited nodes), then stop. The algorithm has finished.

6- Otherwise, select the unvisited node that is marked with the smallest tentative distance, 
set it as the new "current node", and go back to step 3.

 */
uint8_t FindRoute(uint8_t sourceID, uint8_t desID)
{
#ifdef _N
	uint8_t Q[_N] = {0};		// All nodes initially in Q (unvisited nodes)
#else
	uint8_t Q[50] = {0};		// All nodes initially in Q (unvisited nodes)
#endif
	
	uint8_t alt = 0; uint8_t u = 0; uint8_t v = 0; uint8_t j = 0;
	
	memset(route,0,sizeof(route));
	routeDist[sourceID-1] = 0;                  // Distance from source to source
	routePrev[sourceID-1] = 0;               		// Previous node in optimal path initialization undefined
		
	/* Check adjacent neighbors first! */
	for(int col=1 ; col<=6 ; col++)
	{
		if (array[sourceID-1][col] && ((array[sourceID-1][col]>>3) == desID)) {
			routeDist[desID-1] = 1;
			route[0] = desID;
			return col;	
		}
	}						
	
	/* Initialization */
	for (int i=1 ; i<=N ; i++)   					
	{
		if (i != sourceID)            				// Where i has not yet been removed from Q (unvisited nodes)
		{
			routeDist[i-1] = 0xFF;        			// Unknown distance function from source to i
			routePrev[i-1] = 0;            			// Previous node in optimal path from source
		}                    			
	}
	
	/* Algorithm */
	while (!QnotEmpty(Q))
	{				
		u = minArr(routeDist, Q)+1;						// Source node in first case
		if (u == desID) 
		{
			goto finishedRoute;
		}
		else
			Q[u-1] = 1;													// Remove u from Q 
																								
		/* For each neighbor v where v is still in Q. */
		for (uint8_t n=1 ; n<=6 ; n++)      		// Check all module ports
		{
			if (array[u-1][n])										// There's a neighbor v at this port n
			{	
				v = (array[u-1][n]>>3);
				if (!Q[v-1])												// v is still in Q
				{
					alt = routeDist[u-1] + 1;					// Add one hop
					if (alt < routeDist[v-1])      		// A shorter path to v has been found
					{
						routeDist[v-1] = alt; 
						routePrev[v-1] = u; 
					}
				}
			}
		}
	}	
		
finishedRoute:
		
	/* Build the virtual route */	
	while (routePrev[u-1])        		// Construct the shortest path with a stack route
	{
		route[j++] = u;          				// Push the vertex onto the stack
		u = routePrev[u-1];           	// Traverse from target to source
	}
	
	/* Check which port leads to the correct module */
	for(int col=1 ; col<=6 ; col++)	
	{					
		if ( array[sourceID-1][col] && ((array[sourceID-1][col]>>3) == route[routeDist[desID-1]-1]) ) {
			return col;	
		}
	}	

	return 0;			
}

/*-----------------------------------------------------------*/

/* --- Display array topology in human-readable format through module port --- 
*/
void DisplayTopology(uint8_t port)
{
	//char* pcMessage = malloc(20);
	static char pcMessage[30];
	
	/* Print table header */
	strcpy(pcMessage, "\n\r(Module:Port)\t\t");
	writePxMutex(port, pcMessage, strlen(pcMessage), cmd50ms, HAL_MAX_DELAY);
	for (uint8_t i=1 ; i<=NumOfPorts ; i++) 
	{
		sprintf(pcMessage, "P%d\t", i);
		writePxMutex(port, pcMessage, strlen(pcMessage), cmd50ms, HAL_MAX_DELAY);
	}
	writePxMutex(port, "\n\n\r", 3, cmd50ms, HAL_MAX_DELAY);
	
	/* Print each row */
	for(uint8_t row=0 ; row<N ; row++)
	{
		sprintf(pcMessage, "Module %d:\t",row+1);
		writePxMutex(port, pcMessage, strlen(pcMessage), cmd50ms, HAL_MAX_DELAY);
		/* Module PN */
		strncpy(pcMessage, modulePNstring[(array[row][0])], 5);
		writePxMutex(port, pcMessage, 5, cmd50ms, HAL_MAX_DELAY);
		writePxMutex(port, "\t", 1, cmd50ms, HAL_MAX_DELAY);
		/* Connections */
		for(uint8_t col=1 ; col<=NumOfPorts ; col++)
		{
			if (!array[row][col])
				sprintf(pcMessage, "%d\t",0);
			else
				sprintf(pcMessage, "%d:%d\t", (array[row][col]>>3), (array[row][col]&0x07) );
			writePxMutex(port, pcMessage, strlen(pcMessage), cmd50ms, HAL_MAX_DELAY);			
		}
		writePxMutex(port, "\n\r", 2, cmd50ms, HAL_MAX_DELAY);
	}
	
	writePxMutex(port, "\n", 1, cmd50ms, HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
const int8_t *const pcTaskTableHeader = ( int8_t * ) "Task          State  Priority  Stack	#\r\n************************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcTaskTableHeader );
	vTaskList( ((char*) pcWriteBuffer) + strlen( ( char * ) pcTaskTableHeader ) );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
const int8_t * const pcStatsTableHeader = ( int8_t * ) "Task            Abs Time      % Time\r\n****************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcStatsTableHeader );
	vTaskGetRunTimeStats( ((char*) pcWriteBuffer) + strlen( ( char * ) pcStatsTableHeader ) );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE pingCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage = ( int8_t * ) "Hi from module %d\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the ping */
	sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, myID);
	RTOS_IND_blink(200);	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE bootloaderUpdateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage = ( int8_t * ) "Update firmware for module %d\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the update command */
	sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, myID);
	strcat( ( char * ) pcWriteBuffer, ( char * ) pcBootloaderUpdateMessage );
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	
	/* Address for RAM signature (STM32F09x) - Last 4 words of SRAM */
	*((unsigned long *)0x20007FF0) = 0xDEADBEEF;   

	NVIC_SystemReset();		
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE exploreCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK;
	static const int8_t *pcMessage = ( int8_t * ) "\nThe array is being explored. Please wait...\n\r";
	static const int8_t *pcMessageOK = ( int8_t * ) "\nThe array exploration succeeded. I found %d modules including myself. Below is the discovered topology:\n\r";
	static const int8_t *pcMessageErr = ( int8_t * ) "\nThe array exploration failed. Please double check connections and try again.\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the update command */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessage );
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	
	/* Call array exploration routine */
	result = Explore();
	if (result == BOS_OK) {
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, N);
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		DisplayTopology(PcPort);
	} else {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageErr );
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	}
	sprintf( ( char * ) pcWriteBuffer, " ");
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
