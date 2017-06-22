/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : BOS.c
    Description   : Source code for Bitz Operating System (BOS).
		
		Required MCU resources : 
		
			>> Timer 14 for micro-sec delay.

*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

uint16_t myPN = modulePN;
TIM_HandleTypeDef htim14;	/* micro-second delay counter */
uint8_t indMode = IND_off;

/* Define module PN strings [available PNs+1][5 chars] */
const char modulePNstring[9][5] = {"", "H01R0", "H02R0", "H04R0", "H05R0", "H07R0", "H08R0", "H09R0", "H11R0"};

/* Define BOS keywords */
const char BOSkeywords[NumOfKeywords][4] = {"me", "all"};
	
/* Number of modules in the array */
#ifndef _N
	uint8_t N = 1;
	uint8_t myID = 0;
#else
	uint8_t N = _N;
	uint8_t myID = _module;
#endif

/* Routing and topology */
uint8_t portStatus[NumOfPorts+1] = {0};
uint16_t neighbors[NumOfPorts][2] = {0};
uint16_t neighbors2[MaxNumOfPorts][2] = {0};
#ifndef _N
	uint16_t array[MaxNumOfModules][MaxNumOfPorts+1] = {{0}};			/* Array topology */
	uint16_t arrayPortsDir[MaxNumOfModules]= {0};									/* Array ports directions */
	uint8_t routeDist[MaxNumOfModules] = {0}; 
	uint8_t routePrev[MaxNumOfModules] = {0}; 
	uint8_t route[MaxNumOfModules] = {0};
	char moduleAlias[MaxNumOfModules+1][MaxLengthOfAlias+1] = {0};		/* moduleAlias[0] used to store alias for module 0 */
	uint8_t broadcastResponse[MaxNumOfModules] = {0};
#else
	uint16_t arrayPortsDir[_N]= {0};
	uint8_t routeDist[_N] = {0}; 
	uint8_t routePrev[_N] = {0}; 
	uint8_t route[_N] = {0};
	char moduleAlias[_N+1][MaxLengthOfAlias+1] = {0};
	uint8_t broadcastResponse[_N] = {0};
#endif

/* Dimension the buffer into which the input messages is placed. */
uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE] = {0};
uint8_t messageLength[NumOfPorts] = {0};
uint8_t messageParams[20*(MAX_MESSAGE_SIZE-5)] = {0};
char cRxedChar = 0; 
uint8_t longMessage = 0; uint16_t longMessageLastPtr = 0;
static uint8_t longMessageScratchpad[(MaxNumOfPorts+1)*MaxNumOfModules] = {0};
static char pcUserMessage[80];
BOS_Status responseStatus = BOS_OK; 


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

/* UARTcmd task */
extern xTaskHandle xCommandConsoleTask;

/* Define CLI command list*/
typedef struct xCOMMAND_INPUT_LIST
{
	const CLI_Command_Definition_t *pxCommandLineDefinition;
	struct xCOMMAND_INPUT_LIST *pxNext;
} 
CLI_Definition_List_Item_t;
extern CLI_Definition_List_Item_t xRegisteredCommands;

/* Private function prototypes -----------------------------------------------*/	

uint8_t minArr(uint8_t* arr, uint8_t* Q);
uint8_t QnotEmpty(uint8_t* Q);
void NotifyMessagingTaskFromISR(uint8_t port);
void NotifyMessagingTask(uint8_t port);
BOS_Status SaveEEtopology(void);
BOS_Status LoadEEtopology(void);
BOS_Status SaveEEportsDir(void);
BOS_Status LoadEEportsDir(void);
BOS_Status SaveEEalias(void);
BOS_Status LoadEEalias(void);
BOS_Status LoadEEstreams(void);
void SetupDMAStreamsFromMessage(uint8_t direction, uint32_t count, uint32_t timeout, uint8_t src1, uint8_t dst1, uint8_t src2, \
	uint8_t dst2, uint8_t src3, uint8_t dst3);
void StreamTimerCallback( TimerHandle_t xTimer );
uint8_t IsFactoryReset(void);
void EE_FormatForFactoryReset(void);
extern Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst);

/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE pingCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE bootloaderUpdateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
#ifndef _N
static portBASE_TYPE exploreCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
#endif
static portBASE_TYPE resetCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE nameCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE statusCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE infoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE scastCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

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
#ifndef _N
static const CLI_Command_Definition_t exploreCommandDefinition =
{
	( const int8_t * ) "explore", /* The command string to type. */
	( const int8_t * ) "explore:\r\n Explore the array and build its topology\r\n\r\n",
	exploreCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
#endif
/*-----------------------------------------------------------*/
/* CLI command structure : reset */
static const CLI_Command_Definition_t resetCommandDefinition =
{
	( const int8_t * ) "reset", /* The command string to type. */
	( const int8_t * ) "reset:\r\n Reset the module\r\n\r\n",
	resetCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : name */
static const CLI_Command_Definition_t nameCommandDefinition =
{
	( const int8_t * ) "name", /* The command string to type. */
	( const int8_t * ) "name:\r\n Name the module with an alias (1st par.)\r\n\r\n",
	nameCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : status */
static const CLI_Command_Definition_t statusCommandDefinition =
{
	( const int8_t * ) "status", /* The command string to type. */
	( const int8_t * ) "status:\r\n Display module status\r\n\r\n",
	statusCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : info */
static const CLI_Command_Definition_t infoCommandDefinition =
{
	( const int8_t * ) "info", /* The command string to type. */
	( const int8_t * ) "info:\r\n Display array infromation\r\n\r\n",
	infoCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : scast */
static const CLI_Command_Definition_t scastCommandDefinition =
{
	( const int8_t * ) "scast", /* The command string to type. */
	( const int8_t * ) "scast:\r\n Start a single-cast DMA stream. Source port (1st par.), source module (2nd par.), destination port (3rd par.), \
destination module (4th par.), direction ('forward', 'backward', 'bidirectional') (5th par.), transfer count (bytes) (6th par.), transfer timeout (ms) (7th par.)\r\n\r\n",
	scastCommand, /* The function to run. */
	7 /* Seven parameters are expected. */
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
	BOS_Status result = BOS_OK;
	uint8_t port, src, dst, temp; uint16_t code; uint32_t count, timeout;
	static int8_t cCLIString[ cmdMAX_INPUT_SIZE ];
	portBASE_TYPE xReturned; int8_t *pcOutputString;
	
	port = (int8_t)(unsigned) argument;
	
	 /* Infinite loop */
	for( ;; )
	{
		
		/* Wait forever until a message is received on one of the ports */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			
		if (messageLength[port-1])
		{						
			/* Read message source and destination */
			dst = cMessage[port-1][0]; 
			src = cMessage[port-1][1];	
			
			/* Read message code */
			code = ( (uint16_t) cMessage[port-1][2] << 8 ) + cMessage[port-1][3];	
			
			/* Is it a long message? Check MSB */
			if (code>>15) {
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
					/* Process BOS tasks */
					switch (code)
					{
						case CODE_unknown_message :					
							break;
						
						case CODE_ping :
							SendMessageToModule(src, CODE_ping_response, 0);
							indMode = IND_ping;
							break;

						case CODE_ping_response :
							if (!moduleAlias[myID][0])
								sprintf( ( char * ) pcUserMessage, "Hi from module %d\r\n", src);
							else
								sprintf( ( char * ) pcUserMessage, "Hi from module %d (%s)\r\n", src, moduleAlias[src]);
							writePxMutex(PcPort, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
							responseStatus = BOS_OK;								
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
							/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
							SendMessageFromPort(port, 0, 0, CODE_hi_response, 3);
							break;
						
						case CODE_hi_response :
							/* Record your neighbor info */
							neighbors[port-1][0] = ( (uint16_t) src << 8 ) + cMessage[port-1][6];		/* Neighbor ID + Neighbor own port */
							neighbors[port-1][1] = ( (uint16_t) cMessage[port-1][4] << 8 ) + cMessage[port-1][5];		/* Neighbor PN */	
							responseStatus = BOS_OK;
							break;
					#ifndef _N
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
							/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
							SendMessageFromPort(port, 0, 0, CODE_explore_adj_response, temp);
							break;
						
						case CODE_explore_adj_response :
							/* Extract the other module neighbors */
							temp = (messageLength[port-1]-5)/5;
							for (uint8_t k=0 ; k<temp ; k++)  {
								memcpy(&neighbors2[(cMessage[port-1][4+k*5])-1][0], &cMessage[port-1][5+k*5], (size_t)(4));
							}
							responseStatus = BOS_OK;
							break;
					#endif						
						case CODE_port_dir :
							/* Reverse/un-reverse ports according to command parameters */
							for (uint8_t p=1 ; p<=NumOfPorts ; p++) {
								if (p != port)	SwapUartPins(GetUart(p), cMessage[port-1][3+p]); 
							}
							/* Check the input port direction */
							SwapUartPins(GetUart(port), cMessage[port-1][4+MaxNumOfPorts]);
							break;
							
						case CODE_module_id :
							if (cMessage[port-1][4] == 0)						/* Change my own ID */
								myID = cMessage[port-1][5];
							else if (cMessage[port-1][4] == 1) {		/* Change my neighbor's ID */
								messageParams[0] = 0;											/* change own ID */
								messageParams[1] = cMessage[port-1][5];		/* The new ID */
								SendMessageFromPort(cMessage[port-1][6], 0, 0, CODE_module_id, 3);
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
								indMode = IND_topology;
							}
							break;
							
						case CODE_read_port_dir :
							temp = 0;
							/* Check my own ports */
							for (uint8_t p=1 ; p<=NumOfPorts ; p++) {
								if (GetUart(p)->AdvancedInit.Swap == UART_ADVFEATURE_SWAP_ENABLE) {
									messageParams[temp++] = p;
								}									
							}
							/* Send response */
							SendMessageToModule(src, CODE_read_port_dir_response, temp);
							break;
						
						case CODE_read_port_dir_response :
							/* Read module ports directions */
							for (uint8_t p=0 ; p<(messageLength[port-1]-5) ; p++) 
							{
								arrayPortsDir[src-1] |= (0x8000>>((cMessage[port-1][4+p])-1));								
							}
							responseStatus = BOS_OK;
							break;		

						case CODE_exp_eeprom :
						#ifndef _N
							SaveEEtopology();
						#endif
							SaveEEportsDir();
							break;
						
						case CODE_CLI_command :
							/* Obtain the address of the output buffer */
							pcOutputString = FreeRTOS_CLIGetOutputBuffer();
							/* Copy the command */
							memcpy(cCLIString, &cMessage[port-1][4], (size_t) (messageLength[port-1]-5));
							do 
							{
								/* Process the command locally */
								xReturned = FreeRTOS_CLIProcessCommand( cCLIString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );					
								/* Copy the generated string to messageParams */
								memcpy(messageParams, pcOutputString, strlen((char*) pcOutputString));
								/* Send command response */	
								SendMessageToModule(src, CODE_CLI_response, strlen((char*) pcOutputString));
								osDelay(10); 
							} 
							while( xReturned != pdFALSE );								
							/* Reset the buffer */
							memset( cCLIString, 0x00, cmdMAX_INPUT_SIZE );
							break;
							
						case CODE_CLI_response :
							/* Obtain the address of the output buffer. */
							pcOutputString = FreeRTOS_CLIGetOutputBuffer();
							/* Copy the response */
							if (longMessage) {
								memcpy(&pcOutputString[0]+longMessageLastPtr, &cMessage[port-1][4], (size_t) (messageLength[port-1]-5) );
								longMessageLastPtr += messageLength[port-1]-5;
							} else {
								memcpy(&pcOutputString[0]+longMessageLastPtr, &cMessage[port-1][4], (size_t) (messageLength[port-1]-5) );
								longMessageLastPtr = 0;
								responseStatus = BOS_OK;
								/* Wake up the UARTCmd task again */
								xTaskNotifyGive(xCommandConsoleTask);
							}							
							break;
							
						case CODE_DMA_channel :
							/* Save stream paramters in EEPROM */
							EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase], cMessage[port-1][12]);			/* Direction */
							EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+1], ( (uint16_t) cMessage[port-1][4] << 8 ) + cMessage[port-1][5]);			/* Count high word */
							EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+2], ( (uint16_t) cMessage[port-1][6] << 8 ) + cMessage[port-1][7]);			/* Count low word */
							EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+3], ( (uint16_t) cMessage[port-1][8] << 8 ) + cMessage[port-1][9]);			/* Timeout high word */
							EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+4], ( (uint16_t) cMessage[port-1][10] << 8 ) + cMessage[port-1][11]);			/* Timeout low word */
							EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+5], ( (uint16_t) cMessage[port-1][13] << 8 ) + cMessage[port-1][14]);			/* src1 | dst1 */
							if (messageLength[port-1] == 18)
								EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+6], ( (uint16_t) cMessage[port-1][15] << 8 ) + cMessage[port-1][16]);			/* src2 | dst2 */
							if (messageLength[port-1] == 20)
								EE_WriteVariable(VirtAddVarTab[_EE_DMAStreamsBase+7], ( (uint16_t) cMessage[port-1][17] << 8 ) + cMessage[port-1][18]);			/* src3 | dst3 */
							/* Reset MCU */
							NVIC_SystemReset();
							break;
						
						case CODE_DMA_scast_stream :
							count = ( (uint32_t) cMessage[port-1][4] << 24 ) + ( (uint32_t) cMessage[port-1][5] << 16 ) + ( (uint32_t) cMessage[port-1][6] << 8 ) + cMessage[port-1][7];
							timeout = ( (uint32_t) cMessage[port-1][8] << 24 ) + ( (uint32_t) cMessage[port-1][9] << 16 ) + ( (uint32_t) cMessage[port-1][10] << 8 ) + cMessage[port-1][11];
							StartScastDMAStream(cMessage[port-1][13], myID, cMessage[port-1][15], cMessage[port-1][14], cMessage[port-1][12], count, timeout);
							break;
							
						default :
							/* Process module tasks */
							result = (BOS_Status) Module_MessagingTask(code, port, src, dst);
							break;
					}
					
				}

			}	
			
		}
		
		/* Is it unknown message? */
		if (result == BOS_ERR_UnknownMessage) {
			SendMessageToModule(src, CODE_unknown_message, 0);
			result = BOS_OK;			
		}
		
		/* Reset message buffer */
		memset(cMessage[port-1], 0, (size_t) messageLength[port-1]);
		messageLength[port-1] = 0;
		if (portStatus[port] != STREAM && portStatus[port] != CLI) {
			/* Free the port */
			portStatus[port] = FREE;
			/* Read this port again */
			HAL_UART_Receive_IT(GetUart(port), (uint8_t *)&cRxedChar, 1);
		}
		
		taskYIELD();
	}

}

/*-----------------------------------------------------------*/

/*  Micro-seconds timebase init function - TIM14 (16-bit)
*/
void MX_TIM_USEC_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM14_CLK_ENABLE();

	/* Peripheral configuration */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48;
  htim14.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim14.Init.Period = 1;
  HAL_TIM_Base_Init(&htim14);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim14, &sMasterConfig);

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
	uint8_t length = 0, port = 0, src = 0; uint16_t code = 0;
	
	/* Message length */
	length = messageLength[incomingPort-1];
	/* Message source */
	src = cMessage[incomingPort-1][1];
	/* Message code */
	code = ( (uint16_t) cMessage[incomingPort-1][2] << 8 ) + cMessage[incomingPort-1][3];
	/* Message parameters */
	memcpy(messageParams, &cMessage[incomingPort-1][4], (size_t) (length-5) );
	
	/* Find best output port for destination module */
	port = FindRoute(myID, cMessage[incomingPort-1][0]); 
	
	/* Transmit the message from this port */
	SendMessageFromPort(port, src, cMessage[incomingPort-1][0], code, length-5);	

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Activate Messaging Tasks from ISR
*/
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

/* --- Activate Messaging Tasks 
*/
void NotifyMessagingTask(uint8_t port)
{
	switch (port)
	{
	#ifdef _P1
		case P1 : 
			xTaskNotifyGive(P1MsgTaskHandle);	
	#endif
	#ifdef _P2
		case P2 :
			xTaskNotifyGive(P2MsgTaskHandle);	
	#endif
	#ifdef _P3
		case P3 :
			xTaskNotifyGive(P3MsgTaskHandle);	
	#endif
	#ifdef _P4
		case P4 :
			xTaskNotifyGive(P4MsgTaskHandle);	
	#endif
	#ifdef _P5
		case P5 :
			xTaskNotifyGive(P5MsgTaskHandle);	
	#endif
	#ifdef _P6
		case P6 :
			xTaskNotifyGive(P6MsgTaskHandle);	
	#endif
		default:
			;
	}		
}

/*-----------------------------------------------------------*/

/* --- Load stored variables from emulated EEPROM 
*/
void LoadEEvars(void)
{
	/* Load array topology */
#ifndef _N
	LoadEEtopology();
#endif	
	/* Load port directions */
	LoadEEportsDir();
	
	/* Load module alias */
	LoadEEalias();
	
	/* Load DMA streams */
	LoadEEstreams();
	
	/* Load other variables */

	
}

/*-----------------------------------------------------------*/
#ifndef _N
/* --- Save array topology in EEPROM --- 
*/
BOS_Status SaveEEtopology(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0;
	
	/* Save number of modules and myID */
	temp = (uint16_t) (N<<8) + myID;
	EE_WriteVariable(VirtAddVarTab[_EE_NBase], temp);
	
	/* Save topology */
	for(uint8_t i=1 ; i<=N ; i++)
	{
		for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
		{
			if (array[i-1][0]) {
				EE_WriteVariable(VirtAddVarTab[_EE_topologyBase+add], array[i-1][j]);
				add++;
			}				
		}
	}
	
	if ((add+_EE_NBase) >= _EE_portDirBase)
		result = BOS_ERR_EEPROM;
	
	return result;
}
#endif
/*-----------------------------------------------------------*/
#ifndef _N
/* --- Load array topology stored in EEPROM --- 
*/
BOS_Status LoadEEtopology(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0;
	
	/* Load number of modules */
	EE_ReadVariable(VirtAddVarTab[_EE_NBase], &temp);
	N = (uint8_t) (temp>>8);
	if (N == 0)	N = 1;
	myID = (uint8_t) temp;
	
	/* Load topology */
	for(uint8_t i=1 ; i<=N ; i++)
	{
		for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
		{
			EE_ReadVariable(VirtAddVarTab[_EE_topologyBase+add], &array[i-1][j]);
			add++;			
		}
	}	
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Save array ports directions in EEPROM --- 
*/
BOS_Status SaveEEportsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	for(uint8_t i=1 ; i<=N ; i++)
	{
		if (arrayPortsDir[i-1])
			EE_WriteVariable(VirtAddVarTab[_EE_portDirBase+i-1], arrayPortsDir[i-1]);		
		
		if ((i+_EE_portDirBase) >= _EE_aliasBase)
			result = BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load array ports directions stored in EEPROM --- 
*/
BOS_Status LoadEEportsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	for(uint8_t i=1 ; i<=N ; i++)
	{
		EE_ReadVariable(VirtAddVarTab[_EE_portDirBase+i-1], &arrayPortsDir[i-1]);		
		
		if ((i+_EE_portDirBase) >= _EE_aliasBase)
			result = BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Save module alias in EEPROM --- 
*/
BOS_Status SaveEEalias(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0;
	
	for(uint8_t i=0 ; i<=N ; i++)
	{
		if (moduleAlias[i][0]) 
		{
			for(uint8_t j=1 ; j<=MaxLengthOfAlias ; j+=2)
			{
				temp = (uint16_t) (moduleAlias[i][j-1]<<8) + moduleAlias[i][j];
				EE_WriteVariable(VirtAddVarTab[_EE_aliasBase+add], temp);
				add++;			
			}
		}			
	}
	
	if ((add+_EE_aliasBase) >= _EE_varBase)
		result = BOS_ERR_EEPROM;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module alias stored in EEPROM --- 
*/
BOS_Status LoadEEalias(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0;
	
	for(uint8_t i=0 ; i<=N ; i++)
	{
		for(uint8_t j=1 ; j<=MaxLengthOfAlias ; j+=2)
		{
			EE_ReadVariable(VirtAddVarTab[_EE_aliasBase+add], &temp);
			moduleAlias[i][j] = (uint8_t) temp;
			moduleAlias[i][j-1] = (uint8_t) (temp>>8);
			add++;			
		}
		moduleAlias[i][MaxLengthOfAlias] = '\0';
	}
	
	if ((add+_EE_aliasBase) >= _EE_DMAStreamsBase)
		result = BOS_ERR_EEPROM;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module DMA streams --- 
*/
BOS_Status LoadEEstreams(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t temp1 = 0, temp2 = 0, status1 = 0, status2 = 0; 
	uint8_t direction = 0; uint32_t count = 0, timeout = 0;
	static uint8_t src1, dst1, src2, dst2, src3, dst3;
	
	/* Direction */
	status1 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase], &temp1);
	if (!status1) {
		direction = (uint8_t) temp1;
	}

	/* Count */
	status1 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+1], &temp1);
	status2 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+2], &temp2);
	if (!status1 && !status2) {
		count = ( (uint32_t) temp1 << 16 ) + temp2;
	}
	
	/* Timeout */
	status1 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+3], &temp1);
	status2 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+4], &temp2);
	if (!status1 && !status2) {
		timeout = ( (uint32_t) temp1 << 16 ) + temp2;
	}
	
	/* src1 | dst1 */
	status1 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+5], &temp1);
	if (!status1) {
		src1 = (uint8_t) (temp1 >> 8);
		dst1 = (uint8_t) temp1;
	}
	
	/* src2 | dst2 */
	status1 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+6], &temp1);
	if (!status1) {
		src2 = (uint8_t) (temp1 >> 8);
		dst2 = (uint8_t) temp1;	
	}

	/* src3 | dst3 */
	status1 = EE_ReadVariable(VirtAddVarTab[_EE_DMAStreamsBase+7], &temp1);
	if (!status1) {
		src3 = (uint8_t) (temp1 >> 8);
		dst3 = (uint8_t) temp1;
	}
	
	/* Activate the DMA streams */
	SetupDMAStreamsFromMessage(direction, count, timeout, src1, dst1, src2, dst2, src3, dst3);
	
	return result;
}

/*-----------------------------------------------------------*/	

/* --- Setup DMA streams upon request from another module --- 
*/
void SetupDMAStreamsFromMessage(uint8_t direction, uint32_t count, uint32_t timeout, uint8_t src1, uint8_t dst1, uint8_t src2, \
	uint8_t dst2, uint8_t src3, uint8_t dst3)
{
	TimerHandle_t xTimer = NULL;
	
	/* Start DMA streams */
	if (direction == FORWARD) 
	{							
		if (src1 && dst1) {
			PortPortDMA1_Setup(GetUart(src1), GetUart(dst1), 1); DMAStream1total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 1, StreamTimerCallback );
		}
		if (src2 && dst2) {
			PortPortDMA2_Setup(GetUart(src2), GetUart(dst2), 1); DMAStream2total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 2, StreamTimerCallback );
		}
		if (src3 && dst3) {
			PortPortDMA3_Setup(GetUart(src3), GetUart(dst3), 1); DMAStream3total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 3, StreamTimerCallback );
		}
	} 
	else if (direction == BACKWARD) 
	{
		if (src1 && dst1) {
			PortPortDMA1_Setup(GetUart(dst1), GetUart(src1), 1); DMAStream1total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 1, StreamTimerCallback );
		}
		if (src2 && dst2) {
			PortPortDMA2_Setup(GetUart(dst2), GetUart(src2), 1); DMAStream2total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 2, StreamTimerCallback );
		}
		if (src3 && dst3) {
			PortPortDMA3_Setup(GetUart(dst3), GetUart(src3), 1); DMAStream3total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 3, StreamTimerCallback );
		}
	} 
	else if (direction == BIDIRECTIONAL) 
	{
		if (src1 && dst1) {
			PortPortDMA1_Setup(GetUart(src1), GetUart(dst1), 1); DMAStream1total = count;
			PortPortDMA2_Setup(GetUart(dst1), GetUart(src1), 1); DMAStream2total = count;
			/* Create a timeout timer */
			xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 12, StreamTimerCallback );
		}
	}

	/* Start the timeout timer */
	xTimerStart( xTimer, portMAX_DELAY );

}

/*-----------------------------------------------------------*/

/* --- DMA stream timer callback --- 
*/
void StreamTimerCallback( TimerHandle_t xTimer )
{
	uint32_t tid = 0;
	
	tid = ( uint32_t ) pvTimerGetTimerID( xTimer );
	
	switch (tid)
	{
		case 1 :
			StopPortPortDMA1();
			break;
		
		case 2 :
			StopPortPortDMA2();
			break;
		
		case 3 :
			StopPortPortDMA3();
			break;
		
		case 12 :
			StopPortPortDMA1();
			StopPortPortDMA2();
			break;
		
		default:
			break;
	}	
}

/*-----------------------------------------------------------*/	

/* --- Check for factory reset condition: First port TXD is connected to last port RXD   
					or to programming port RXD   
*/
uint8_t IsFactoryReset(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_TypeDef *P1_TX_Port = 0, *P_last_RX_Port = 0, *P_prog_RX_Port = 0;
	uint16_t P1_TX_Pin = 0, P_last_RX_Pin = 0, P_prog_RX_Pin = 0;
	
	/* -- Setup GPIOs -- */
	
  /* Enable all GPIO Ports Clocks */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	
	/* UART pins */
#ifdef _Usart1
	if(GetUart(P1) == &huart1) {
		P1_TX_Port = USART1_TX_PORT;
		P1_TX_Pin = USART1_TX_PIN;
	}
#endif
#ifdef _Usart2
	if(GetUart(P1) == &huart2) {
		P1_TX_Port = USART2_TX_PORT;
		P1_TX_Pin = USART2_TX_PIN;
	}
#endif
#ifdef _Usart3
	if(GetUart(P1) == &huart3) {
		P1_TX_Port = USART3_TX_PORT;
		P1_TX_Pin = USART3_TX_PIN;
	}
#endif
#ifdef _Usart4
	if(GetUart(P1) == &huart4) {
		P1_TX_Port = USART4_TX_PORT;
		P1_TX_Pin = USART4_TX_PIN;
	}
#endif
#ifdef _Usart5
	if(GetUart(P1) == &huart5) {
		P1_TX_Port = USART5_TX_PORT;
		P1_TX_Pin = USART5_TX_PIN;
	}
#endif
#ifdef _Usart6
	if(GetUart(P1) == &huart6) {
		P1_TX_Port = USART6_TX_PORT;
		P1_TX_Pin = USART6_TX_PIN;
	}
#endif
	
	/* RXD of last port */
#ifdef _Usart1
	if(GetUart(P_LAST) == &huart1) {
		P_last_RX_Port = USART1_RX_PORT;
		P_last_RX_Pin = USART1_RX_PIN;
	}
#endif
#ifdef _Usart2
	if(GetUart(P_LAST) == &huart2) {
		P_last_RX_Port = USART2_RX_PORT;
		P_last_RX_Pin = USART2_RX_PIN;
	}
#endif
#ifdef _Usart3
	if(GetUart(P_LAST) == &huart3) {
		P_last_RX_Port = USART3_RX_PORT;
		P_last_RX_Pin = USART3_RX_PIN;
	}
#endif
#ifdef _Usart4
	if(GetUart(P_LAST) == &huart4) {
		P_last_RX_Port = USART4_RX_PORT;
		P_last_RX_Pin = USART4_RX_PIN;
	}
#endif
#ifdef _Usart5
	if(GetUart(P_LAST) == &huart5) {
		P_last_RX_Port = USART5_RX_PORT;
		P_last_RX_Pin = USART5_RX_PIN;
	}
#endif
#ifdef _Usart6
	if(GetUart(P_LAST) == &huart6) {
		P_last_RX_Port = USART6_RX_PORT;
		P_last_RX_Pin = USART6_RX_PIN;
	}
#endif	
	
	/* RXD of programming port */
#ifdef _Usart1
	if(GetUart(P_PROG) == &huart1) {
		P_prog_RX_Port = USART1_RX_PORT;
		P_prog_RX_Pin = USART1_RX_PIN;
	}
#endif
#ifdef _Usart2
	if(GetUart(P_PROG) == &huart2) {
		P_prog_RX_Port = USART2_RX_PORT;
		P_prog_RX_Pin = USART2_RX_PIN;
	}
#endif
#ifdef _Usart3
	if(GetUart(P_PROG) == &huart3) {
		P_prog_RX_Port = USART3_RX_PORT;
		P_prog_RX_Pin = USART3_RX_PIN;
	}
#endif
#ifdef _Usart4
	if(GetUart(P_PROG) == &huart4) {
		P_prog_RX_Port = USART4_RX_PORT;
		P_prog_RX_Pin = USART4_RX_PIN;
	}
#endif
#ifdef _Usart5
	if(GetUart(P_PROG) == &huart5) {
		P_prog_RX_Port = USART5_RX_PORT;
		P_prog_RX_Pin = USART5_RX_PIN;
	}
#endif
#ifdef _Usart6
	if(GetUart(P_PROG) == &huart6) {
		P_prog_RX_Port = USART6_RX_PORT;
		P_prog_RX_Pin = USART6_RX_PIN;
	}
#endif	
	
	/* TXD of first port */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = P1_TX_Pin;
	HAL_GPIO_Init(P1_TX_Port, &GPIO_InitStruct);
	
	/* RXD of last port */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;	
	GPIO_InitStruct.Pin = P_last_RX_Pin;
	HAL_GPIO_Init(P_last_RX_Port, &GPIO_InitStruct);	
	
	/* RXD of programming port */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;	
	GPIO_InitStruct.Pin = P_prog_RX_Pin;
	HAL_GPIO_Init(P_prog_RX_Port, &GPIO_InitStruct);	
	
	/* Check for factory reset conditions */
	HAL_GPIO_WritePin(P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms(1);
	if (HAL_GPIO_ReadPin(P_prog_RX_Port,P_prog_RX_Pin) == RESET)
	{
		HAL_GPIO_WritePin(P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms(1);
		if (HAL_GPIO_ReadPin(P_prog_RX_Port,P_prog_RX_Pin) == SET) {
			return 1;
		}
	}
	HAL_GPIO_WritePin(P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms(1);
	if (HAL_GPIO_ReadPin(P_last_RX_Port,P_last_RX_Pin) == RESET)
	{
		HAL_GPIO_WritePin(P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms(1);
		if (HAL_GPIO_ReadPin(P_last_RX_Port,P_last_RX_Pin) == SET) {
			return 1;
		}
	}

	
	/* Clear flag for formated EEPROM if it was already set */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	*((unsigned long *)0x20007FF0) = 0xFFFFFFFF; 
	
	return 0;
}

/*-----------------------------------------------------------*/	
 
/* --- Format emulated EEPROM for a factory reset
*/
void EE_FormatForFactoryReset(void)
{
	/* Check if EEPROM was just formated? */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	if (*((unsigned long *)0x20007FF0) == 0xBEEFDEAD)
	{
		// Do nothing
	}
	else
	{
		if (EE_Format() == HAL_OK) 
		{
			/* Set flag for formated EEPROM */
			*((unsigned long *)0x20007FF0) = 0xBEEFDEAD; 
		}
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
	/* Unlock the Flash memory */
	HAL_FLASH_Unlock();

	/* EEPROM Init */
	EE_Init();
	
	/* Check for factory reset */
	if (IsFactoryReset())
	{
		/* Format EEPROM once */
		EE_FormatForFactoryReset();
		
		/* Software reset */
		NVIC_SystemReset();
	}
	else
	{	
		/* Load stored EEPROM variables */
		LoadEEvars();
	}
	
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM_USEC_Init();
	
	/* Startup indicator sequence */
	if (myID == 0)		/* Native module */
	{
		IND_blink(500);
	}
	else							/* Non-native module */
	{
		IND_blink(500);
		HAL_Delay(100);
		IND_blink(100);
		HAL_Delay(100);
		IND_blink(100);
	}
	
	
	/* Initialize the module */
#ifdef  Module_Init
	Module_Init();
#endif

/* If no pre-defined topology, initialize ports direction */
#ifndef _N
	UpdateMyPortsDir();
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
#ifndef _N
	FreeRTOS_CLIRegisterCommand( &exploreCommandDefinition );
#endif
	FreeRTOS_CLIRegisterCommand( &resetCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &nameCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &statusCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &infoCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &scastCommandDefinition );
	
	
#ifdef H01R0	
	FreeRTOS_CLIRegisterCommand( &onCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &offCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &colorCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &RGBCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &toggleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &pulseColorCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &pulseRGBCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &sweepCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &dimCommandDefinition );
#endif	
#ifdef H09R0	
	FreeRTOS_CLIRegisterCommand( &onCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &offCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &toggleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &ledModeCommandDefinition );
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
	#ifdef _P_USB
		case P_USB :
			return P_USBuart;
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
#if (H02R0)
		if (huart->Instance == USART4)
				return P1;
		else if (huart->Instance == USART2)
				return P2;
		else if (huart->Instance == USART6)
				return P3;
		else if (huart->Instance == USART1)
				return P5;
		else if (huart->Instance == USART5)
				return P6;
#endif
#if (H04R0)
		if (huart->Instance == USART4)
				return P1;
		else if (huart->Instance == USART2)
				return P2;
		else if (huart->Instance == USART3)
				return P4;
		else if (huart->Instance == USART1)
				return P5;
		else if (huart->Instance == USART5)
				return P6;
#endif
#if (H05R0)
		if (huart->Instance == USART4)
				return P1;
		else if (huart->Instance == USART2)
				return P2;
		else if (huart->Instance == USART3)
				return P3;
		else if (huart->Instance == USART1)
				return P4;
		else if (huart->Instance == USART5)
				return P5;
#endif
#if (H07R0)
		if (huart->Instance == USART4)
				return P1;
		else if (huart->Instance == USART2)
				return P2;
		else if (huart->Instance == USART3)
				return P4;
		else if (huart->Instance == USART1)
				return P5;
#endif
#if (H08R0)
		if (huart->Instance == USART3)
				return P1;
		else if (huart->Instance == USART1)
				return P2;
		else if (huart->Instance == USART5)
				return P3;
		else if (huart->Instance == USART4)
				return P4;
		else if (huart->Instance == USART2)
				return P5;
		else if (huart->Instance == USART6)
				return P6;
#endif
#if (H09R0)
		if (huart->Instance == USART5)
				return P1;
		else if (huart->Instance == USART2)
				return P2;
		else if (huart->Instance == USART6)
				return P3;
		else if (huart->Instance == USART3)
				return P4;
		else if (huart->Instance == USART1)
				return P5;
#endif
#if (H11R0)
		if (huart->Instance == USART2)
				return P1;
		else if (huart->Instance == USART6)
				return P2;
		else if (huart->Instance == USART4)
				return P3;
		else if (huart->Instance == USART3)
				return P4;
		else if (huart->Instance == USART1)
				return P5;
		else if (huart->Instance == USART5)
				return P6;
#endif
		
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
		SendMessageFromPort(port, myID, dst, code, numberOfParams);	
	}
	/* Broadcast message */
	else
	{
		for( uint8_t port=1 ; port <= NumOfPorts ; port++ )
		{
			/* Transmit the message from this port */
			SendMessageFromPort(port, myID, dst, code, numberOfParams);	
		}
	}

	return result;
}

/*-----------------------------------------------------------*/

/* --- Send a message from a specific port 
*/
BOS_Status SendMessageFromPort(uint8_t port, uint8_t src, uint8_t dst, uint16_t code, uint16_t numberOfParams)
{
	BOS_Status result = BOS_OK; 
	uint8_t length = 0; static uint16_t totalNumberOfParams = 0; static uint16_t ptrShift = 0;
	char message[MAX_MESSAGE_SIZE] = {0};
	
	/* Increase the priority of current running task */
	vTaskPrioritySet( NULL, osPriorityHigh );
	
	if (!src)	src = myID;
	
	/* Construct the message */
	message[0] = dst;						
	message[1] = src;
	message[3] = (uint8_t) code;
	message[2] = (uint8_t) (code >> 8);	
	
	/* Copy parameters */
	if (numberOfParams <= (MAX_MESSAGE_SIZE-5) ) {				
		memcpy((char*)&message[4], (&messageParams[0]+ptrShift), numberOfParams);
		/* Calculate message length */
		length = numberOfParams + 5;
		/* Reset messageParams buffer */
		memset( (&messageParams[0]+ptrShift), 0, numberOfParams );
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
				SendMessageFromPort(port, src, dst, code, numberOfParams);
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
	
	/* In case response is expected */
	responseStatus = BOS_ERR_NoResponse;

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
	uint8_t length = 0, src = 0; uint16_t code = 0;
	
	/* Use broadcast plans */
	
	/* Message length */
	length = messageLength[incomingPort-1];
	/* Message source */
	src = cMessage[incomingPort-1][1];
	/* Message code */
	code = ( (uint16_t) cMessage[incomingPort-1][2] << 8 ) + cMessage[incomingPort-1][3];
	/* Message parameters */
	memcpy(messageParams, &cMessage[incomingPort-1][4], (size_t) (length-5) );
	
	/* Transmit to all broadcast ports */
	for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
	{
		if (port != incomingPort) 	/* Also check for broadcast ports from broadacast plans */	
		{
			/* Transmit the message from this port */
			SendMessageFromPort(port, src, 0xFF, code, length-5);	
		}	
	}
	
	return result;
}

/*-----------------------------------------------------------*/
#ifndef _N
/* --- Explore the array and create its topology (executed only by master)
*/
BOS_Status Explore(void)
{
	BOS_Status result = BOS_OK;
	uint8_t currentID = 0, lastID = 0, temp1 = 0, temp2 = 0;
	uint16_t temp16 = 0;
	
	myID = 1; 		/* Master ID */
	
	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
	
	for (uint8_t port=1 ; port<=NumOfPorts ; port++) {
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
			SendMessageFromPort(port, 0, 0, CODE_module_id, 3);			
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
			for (uint8_t p=1 ; p<=MaxNumOfPorts ; p++) {
				messageParams[p-1] = REVERSED;
			}
			messageParams[MaxNumOfPorts] = NORMAL;		/* Make sure the inport is not reversed */
			SendMessageToModule(i, CODE_port_dir, MaxNumOfPorts+1);
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
	
	ExploreNeighbors(PcPort);
	/* Check for any unIDed neighbors */
	for (uint8_t i=1 ; i<=NumOfPorts ; i++) 
	{
		temp16 = neighbors[i-1][0];		/* Neighbor ID */
		temp1 = (uint8_t)(temp16>>8);											
		if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
			result = BOS_ERR_UnIDedModule;
		}		
	}
	/* Ask other modules for any unIDed neighbors */
	for (uint8_t i=2 ; i<=currentID ; i++) 
	{
		SendMessageToModule(i, CODE_explore_adj, 0);
		osDelay(100);	
		/* Check for any unIDed neighbors */
		for (uint8_t j=1 ; j<=MaxNumOfPorts ; j++) 
		{
			temp16 = neighbors2[j-1][0];		/* Neighbor ID */
			temp1 = (uint8_t)(temp16>>8);											
			if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
				result = BOS_ERR_UnIDedModule;
			}
		}				
	}
	
	
	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
	
	if (result == BOS_OK)
	{	
		/* Step 5a - Virtually reset the state of master ports to Normal */
		for (uint8_t port=1 ; port<=NumOfPorts ; port++) {
			arrayPortsDir[0] &= (~(0x8000>>(port-1)));		/* Set bit to zero */
		}
		
		/* Step 5b - Update other modules ports starting from the last one */
		for (uint8_t i=currentID ; i>=2 ; i--) 
		{
			for (uint8_t p=1 ; p<=MaxNumOfPorts ; p++) 
			{		
				if (!array[i-1][p])	{
					/* If empty port leave normal */
					messageParams[p-1] = NORMAL;
					arrayPortsDir[i-1] &= (~(0x8000>>(p-1)));		/* Set bit to zero */
				} else {
					/* If not empty, check neighbor */			
					temp16 = array[i-1][p];
					temp1 = (uint8_t)(temp16>>3);										/* Neighbor ID */
					temp2 = (uint8_t)(temp16 & 0x0007);							/* Neighbor port */	
					/* Check neighbor port direction */
					if ( !(arrayPortsDir[temp1-1] & (0x8000>>(temp2-1))) ) {
						/* Neighbor port is normal */
						messageParams[p-1] = REVERSED;
						arrayPortsDir[i-1] |= (0x8000>>(p-1));		/* Set bit to one */
					} else {
						/* Neighbor port is reversed */
						messageParams[p-1] = NORMAL;
						arrayPortsDir[i-1] &= (~(0x8000>>(p-1)));		/* Set bit to zero */						
					}				
				}
			}
			
			/* Step 5c - Check if an inport is reversed */
			/* Find out the inport to this module from master */
			FindRoute(1, i);
			temp1 = route[NumberOfHops(i)-1];				/* previous module = route[Number of hops - 1] */
			temp2 = FindRoute(i, temp1);
			/* Is the inport reversed? */
			if ( (temp1 == i) || (messageParams[temp2-1] == REVERSED) )
				messageParams[MaxNumOfPorts] = REVERSED;		/* Make sure the inport is reversed */
			
			/* Step 5d - Update module ports directions */
			SendMessageToModule(i, CODE_port_dir, MaxNumOfPorts+1);
			osDelay(10);			
		}			
	
		/* Step 5e - Update master ports > all normal */
		for (uint8_t port=1 ; port<=NumOfPorts ; port++) {
			if (port != PcPort)	SwapUartPins(GetUart(port), NORMAL);
		}
	}
	
			
	/* >>> Step 6 - Test new port directions by pinging all modules */
	
	if (result == BOS_OK) 
	{		
		for (uint8_t i=2 ; i<=N ; i++) 
		{
			SendMessageToModule(i, CODE_ping, 0);
			//osDelay(100*NumberOfHops(i));	
			osDelay(100);
			if (responseStatus == BOS_OK)
				result = BOS_OK;
			else if (responseStatus == BOS_ERR_NoResponse)
				result = BOS_ERR_NoResponse;
		}
	}
	
	
	/* >>> Step 7 - Build and broadcast broadcast plans */
	
	
	/* >>> Step 8 - Save all (topology, port directions and broadcast plans) in EEPROM */
	
	if (result == BOS_OK)
	{
		/* Save data in the master */
		SaveEEtopology();
		SaveEEportsDir();
		/* Ask other modules to save their data too */
		//SendMessageToModule(0xFF, CODE_exp_eeprom, 0);
		for (uint8_t i=2 ; i<=N ; i++) 
		{
			SendMessageToModule(i, CODE_exp_eeprom, 0);	
			osDelay(50);
		}
	}	

	return result;
}
#endif
/*-----------------------------------------------------------*/
#ifndef _N
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
			/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
			SendMessageFromPort(port, 0, 0, CODE_hi, 3);
			/* Minimum delay between two consequetive SendMessage commands (with response) */
			osDelay(10);
		}
	}
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
*/
void StartMicroDelay(uint16_t Delay)
{
	portENTER_CRITICAL();
	
	if (Delay)
	{
		htim14.Instance->ARR = Delay;
	
		HAL_TIM_Base_Start(&htim14);	

		while(htim14.Instance->CNT != 0)
		{
		}
		
		HAL_TIM_Base_Stop(&htim14);
	}
	
	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Swap UART pins ( NORMAL | REVERSED )--- 
*/
void SwapUartPins(UART_HandleTypeDef *huart, uint8_t direction)
{
	if (huart != NULL) {
		if (direction == REVERSED) {
			arrayPortsDir[myID-1] |= (0x8000>>(GetPort(huart)-1));		/* Set bit to one */
			huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
			huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
			HAL_UART_Init(huart);
		} else if (direction == NORMAL) {
			arrayPortsDir[myID-1] &= (~(0x8000>>(GetPort(huart)-1)));		/* Set bit to zero */
			huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
			huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_DISABLE;
			HAL_UART_Init(huart);		
		}
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
	/* Print table header */
	sprintf(pcUserMessage, "\n\r(Module:Port)\t\t");
	writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
	for (uint8_t i=1 ; i<=NumOfPorts ; i++) 
	{
		sprintf(pcUserMessage, "P%d\t", i);
		writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
	}
	writePxMutex(port, "\n\n\r", 3, cmd50ms, HAL_MAX_DELAY);
	
	/* Print each row */
	for(uint8_t row=0 ; row<N ; row++)
	{
		sprintf(pcUserMessage, "Module %d:\t",row+1);
		writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
		/* Module PN */
		strncpy(pcUserMessage, modulePNstring[(array[row][0])], 5);
		writePxMutex(port, pcUserMessage, 5, cmd50ms, HAL_MAX_DELAY);
		writePxMutex(port, "\t", 1, cmd50ms, HAL_MAX_DELAY);
		/* Connections */
		for(uint8_t col=1 ; col<=NumOfPorts ; col++)
		{
			if (!array[row][col])
				sprintf(pcUserMessage, "%d\t",0);
			else
				sprintf(pcUserMessage, "%d:%d\t", (array[row][col]>>3), (array[row][col]&0x07) );
			writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);			
		}
		writePxMutex(port, "\n\r", 2, cmd50ms, HAL_MAX_DELAY);
	}
	
	writePxMutex(port, "\n", 1, cmd50ms, HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/

/* --- Display ports directions in human-readable format through module port --- 
*/
void DisplayPortsDir(uint8_t port)
{
	sprintf(pcUserMessage, "\n\rThese ports are reversed:");
	writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
	
	for (uint8_t i=1 ; i<=N ; i++) 
	{
		for (uint8_t p=1 ; p<=MaxNumOfPorts ; p++) 
		{		
			if ( (arrayPortsDir[i-1] & (0x8000>>(p-1))) ) 			/* Port is reversed */
			{
				sprintf(pcUserMessage, "\n\rModule %d : P%d", i, p);
				writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
			}	
		}
	}
	
	sprintf(pcUserMessage, "\n\n\rAll other ports are normal\n\r");
	writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/

/* --- Display a description of current module status (Firmware, Ports, P2P DMAs) --- 
*/
void DisplayModuleStatus(uint8_t port)
{
	int8_t *pcOutputString;
	uint16_t temp = 0;
	
	/* Obtain the address of the output buffer. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();
	
	strcpy( (char *) pcOutputString, "");
	
	sprintf(pcUserMessage, "\n\r*** Module %d Status ***\n", myID);
	strcat( (char *) pcOutputString, pcUserMessage);
	sprintf(pcUserMessage, "\n\rConnected via port: P%d\n\r", PcPort);
	strcat( (char *) pcOutputString, pcUserMessage);
	
	/* Firmware */
	sprintf(pcUserMessage, "\n\rFirmware version: %s", _firmVersion);
	strcat( (char *) pcOutputString, pcUserMessage);
	sprintf(pcUserMessage, "\n\rFirmware date:    %s", _firmDate);
	strcat( (char *) pcOutputString, pcUserMessage);
	sprintf(pcUserMessage, "\n\rFirmware time:    %s\n\r", _firmTime);
	strcat( (char *) pcOutputString, pcUserMessage);	
	
	/* Ports */
	sprintf(pcUserMessage, "\n\rPorts Status:\n\n\r");
	strcat( (char *) pcOutputString, pcUserMessage);
	for(uint8_t i=1 ; i<=NumOfPorts ; i++)
	{
		sprintf(pcUserMessage, "P%d: ", i);
		strcat( (char *) pcOutputString, pcUserMessage);
		switch (portStatus[i])
		{
				case FREE : 
						sprintf(pcUserMessage, "Free\n\r"); break;
				case MSG :
						sprintf(pcUserMessage, "Receiving messages\n\r"); break;
				case STREAM :
						sprintf(pcUserMessage, "Streaming\n\r"); break;
				case CLI :
						sprintf(pcUserMessage, "Receiving user commands\n\r"); break;
				default:
						break;
		}		
		strcat( (char *) pcOutputString, pcUserMessage);
	}	

	/* P2P DMAs */
	sprintf(pcUserMessage, "\n\rPort-to-port DMAs Status:\n\r");
	strcat( (char *) pcOutputString, pcUserMessage);	
	if (portPortDMA1.Instance->CNDTR == 0) {
			sprintf(pcUserMessage, "\n\rPort-to-port DMA 1 is free");
	} else {
			sprintf(pcUserMessage, "\n\rPort-to-port DMA 1 is streaming from P%d to P%d", GetPort(portPortDMA1.Parent), GetPort(dmaStreamDst[0]));
	}
	strcat( (char *) pcOutputString, pcUserMessage);
	if (portPortDMA2.Instance->CNDTR == 0) {
			sprintf(pcUserMessage, "\n\rPort-to-port DMA 2 is free");
	} else {
			sprintf(pcUserMessage, "\n\rPort-to-port DMA 2 is streaming from P%d to P%d", GetPort(portPortDMA2.Parent), GetPort(dmaStreamDst[1]));
	}
	strcat( (char *) pcOutputString, pcUserMessage);
	if (portPortDMA3.Instance->CNDTR == 0) {
			sprintf(pcUserMessage, "\n\rPort-to-port DMA 3 is free");
	} else {
			sprintf(pcUserMessage, "\n\rPort-to-port DMA 3 is streaming from P%d to P%d", GetPort(portPortDMA3.Parent), GetPort(dmaStreamDst[2]));
	}
	strcat( (char *) pcOutputString, pcUserMessage);
	strcat( (char *) pcOutputString, "\n\r");
	
	/* Ports direction */
	strcat( (char *) pcOutputString, "\n\rThese ports are reversed: ");
	temp = strlen( (char *) pcOutputString);
	for (uint8_t p=1 ; p<=NumOfPorts ; p++) 
	{		
		if ( (arrayPortsDir[myID-1] & (0x8000>>(p-1))) ) 			/* Port is reversed */
		{
			sprintf(pcUserMessage, "P%d ", p);
			strcat( (char *) pcOutputString, pcUserMessage);
		}	
	}
	if (temp == strlen( (char *) pcOutputString)) {				/* All ports are normal */
		strcat( (char *) pcOutputString, "None");
	}
	strcat( (char *) pcOutputString, "\n\r");
	
	/* Display output */
	if (port)
		writePxMutex(port, (char *) pcOutputString, strlen( (char *) pcOutputString), cmd50ms, HAL_MAX_DELAY);
	
}

/*-----------------------------------------------------------*/

/* --- Extract module ID from it's alias, ID string or keyword --- 
*/
uint8_t GetID(char* string)
{
	uint8_t id = 0;
	
	if(!strcmp(string, "me"))							/* Check keywords */
		return myID;
	else if(!strcmp(string, "all"))							
		return 0xFF;				/* BOS_BROADCAST */
	else if (string[0] == '#') {					/* Check IDs */
		id = atol(string+1);
		if (id > 0 && id <= N)
			return id;
		else if (id == myID)
			return myID;
		else
			return 101;				/* BOS_ERR_WrongID */
	} else {															/* Check alias */
		for (int8_t i=N ; i>=0 ; i--) {
			if(!strcmp(string, moduleAlias[i]) && (*string != 0))
				return i;	
		}
		return 100;			/* BOS_ERR_WrongName */
	}
	
}

/*-----------------------------------------------------------*/

/* --- Name a module with an alias --- 
*/
BOS_Status NameModule(uint8_t module, char* alias)
{
	BOS_Status result = BOS_OK; 
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;

	/* Check alias with keywords */
	for(int i=0 ; i<NumOfKeywords ; i++)
	{
		if (!strcmp(alias, BOSkeywords[i]))	
			return BOS_ERR_Keyword;
	}
	
	/* Check alias with other aliases */
	for(int i=1 ; i<N ; i++)
	{
		if (!strcmp(alias, moduleAlias[i]))	
			return BOS_ERR_ExistingAlias;
	}
	
	/* Check alias with BOS and module commands */
	for( pxCommand = &xRegisteredCommands; pxCommand != NULL; pxCommand = pxCommand->pxNext )
	{
		pcRegisteredCommandString = pxCommand->pxCommandLineDefinition->pcCommand;
		xCommandStringLength = strlen( ( const char * ) pcRegisteredCommandString );
		
		if( !strncmp(alias, (const char *) pcRegisteredCommandString, xCommandStringLength ) ) {
			return BOS_ERR_ExistingCmd;
		}
	}
	
	/* Alias is unique */
	strcpy(moduleAlias[module], alias);
	
	/* Share new alias with other modules */
	
	
	/* Save new alias to Flash */
	result = SaveEEalias();
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Read Ports directions when a pre-defined topology file is used --- 
*/
BOS_Status ReadPortsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	/* Ask all other modules for their ports directions */
	for (uint8_t i=1 ; i<=N ; i++) 
	{
		if (i != myID) {
			SendMessageToModule(i, CODE_read_port_dir, 0);
			osDelay(50);
			if (responseStatus != BOS_OK)	{
				result = BOS_ERR_NoResponse;
			} 	
		} else {
			/* Check my own ports */
			for (uint8_t p=1 ; p<=NumOfPorts ; p++) {
				if (GetUart(p)->AdvancedInit.Swap == UART_ADVFEATURE_SWAP_ENABLE) {
					arrayPortsDir[myID-1] |= (0x8000>>(p-1));		/* Set bit to 1 */
				}									
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/
#ifndef _N
/* --- Update module port directions based on what is stored in eeprom --- 
*/
BOS_Status UpdateMyPortsDir(void)
{
	BOS_Status result = BOS_OK;
	
	/* Check port direction */
	for (uint8_t p=1 ; p<=NumOfPorts ; p++) 
	{
		if ( !(arrayPortsDir[myID-1] & (0x8000>>(p-1))) ) {
			/* Port is normal */
			SwapUartPins(GetUart(p), NORMAL);
		} else {
			/* Port is reversed */
			SwapUartPins(GetUart(p), REVERSED);					
		}	
	}		
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Start a single-cast DMA stream across the array. Transfer ends after (count) bytes are transferred 
			or timeout (ms), whichever comes first. --- 
*/
BOS_Status StartScastDMAStream(uint8_t srcP, uint8_t srcM, uint8_t dstP, uint8_t dstM, uint8_t direction, uint32_t count, uint32_t timeout)
{
	BOS_Status result = BOS_OK;
	TimerHandle_t xTimer = NULL;
	uint8_t port = 0, temp1 = 0, temp2 = 0;
	
	/* Is the source a different module? */
	if (srcM != myID) {
		/* Forward this task to the source module */
		messageParams[0] = (uint8_t) (count >> 24);			/* Count */
		messageParams[1] = (uint8_t) (count >> 16);
		messageParams[2] = (uint8_t) (count >> 8);
		messageParams[3] = (uint8_t) count;
		messageParams[4] = (uint8_t) (timeout >> 24);		/* Timeout */
		messageParams[5] = (uint8_t) (timeout >> 16);
		messageParams[6] = (uint8_t) (timeout >> 8);
		messageParams[7] = (uint8_t) timeout;
		messageParams[8] = direction;										/* Stream direction */
		messageParams[9] = srcP;												/* Source port */
		messageParams[10] = dstM;												/* destination module */
		messageParams[11] = dstP;												/* destination port */
		SendMessageToModule(srcM, CODE_DMA_scast_stream, 12);		
		
		return result;
	}
	
	/* Inform participating modules */
	for(uint8_t i=0 ; i<sizeof(route) ; i++)
	{
		FindRoute(srcM, dstM);
		/* Message other modules */
		if (route[i]) 
		{
			/* Find out the inport and outport to this module from previous one */
			if (route[i+1]) {
				temp1 = FindRoute(route[i], route[i+1]);
			} else {
				temp1 = FindRoute(route[i], srcM);
			}
			FindRoute(srcM, dstM);
			if (route[i] == dstM) {
				temp2 = dstP;
			} else {
				temp2 = FindRoute(route[i], route[i-1]);
			}
			/* Message parameters*/
			messageParams[0] = (uint8_t) (count >> 24);			/* Count */
			messageParams[1] = (uint8_t) (count >> 16);
			messageParams[2] = (uint8_t) (count >> 8);
			messageParams[3] = (uint8_t) count;
			messageParams[4] = (uint8_t) (timeout >> 24);		/* Timeout */
			messageParams[5] = (uint8_t) (timeout >> 16);
			messageParams[6] = (uint8_t) (timeout >> 8);
			messageParams[7] = (uint8_t) timeout;
			messageParams[8] = direction;										/* Stream direction */
			messageParams[9] = temp1;												/* Source port */
			messageParams[10] = temp2;											/* destination port */
			FindRoute(srcM, dstM);
			SendMessageToModule(route[i], CODE_DMA_channel, 11);
			osDelay(10);
		}
	}
	
	if (srcM == dstM)
		port = dstP;
	else
		port = FindRoute(srcM, dstM);
	
	/* Setup my own DMA stream */
	if (direction == FORWARD) {
		PortPortDMA1_Setup(GetUart(srcP), GetUart(port), 1); DMAStream1total = count;
		/* Create a timeout timer */
		xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 1, StreamTimerCallback );
	} else if (direction == BACKWARD) {
		PortPortDMA1_Setup(GetUart(port), GetUart(srcP), 1); DMAStream1total = count;
		/* Create a timeout timer */
		xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 1, StreamTimerCallback );
	} else if (direction == BIDIRECTIONAL) {
		PortPortDMA1_Setup(GetUart(srcP), GetUart(port), 1); DMAStream1total = count;
		PortPortDMA2_Setup(GetUart(port), GetUart(srcP), 1); DMAStream2total = count;
		/* Create a timeout timer */
		xTimer = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 12, StreamTimerCallback );
	}
	
	/* Start the timeout timer */
	xTimerStart( xTimer, portMAX_DELAY );
	
	return result;
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
	static const int8_t *pcMessage1 = ( int8_t * ) "Hi from module %d\r\n";
	static const int8_t *pcMessage2 = ( int8_t * ) "Hi from module %d (%s)\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the ping */
	if (!moduleAlias[myID][0])
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage1, myID);
	else
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage2, myID, moduleAlias[myID]);
	
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
#ifndef _N
static portBASE_TYPE exploreCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK;
	static const int8_t *pcMessage = ( int8_t * ) "\nThe array is being explored. Please wait...\n\r";
	static const int8_t *pcMessageOK = ( int8_t * ) "\nThe array exploration succeeded. I found %d modules including myself. Here is the discovered topology:\n\r";
	static const int8_t *pcMessageErr = ( int8_t * ) "\nThe array exploration failed. Please double check connections, reset the modules and try again.\n\r";
	
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
		DisplayPortsDir(PcPort);
	} else {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageErr );
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	}
	sprintf( ( char * ) pcWriteBuffer, " ");
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
#endif

/*-----------------------------------------------------------*/

static portBASE_TYPE resetCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	NVIC_SystemReset();	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE nameCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1; 
	static portBASE_TYPE xParameterStringLength1;
	
	static const int8_t *pcMessageOK = ( int8_t * ) "Module %d is named %s\n\r";
	static const int8_t *pcMessageKey = ( int8_t * ) "%s is a reserved BOS keyword! Please use a different alias\n\r";
	static const int8_t *pcMessageAlias = ( int8_t * ) "%s is already used! Please use a different alias\n\r";
	static const int8_t *pcMessageCmd = ( int8_t * ) "%s is an existing CLI command! Please use a different alias\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

	/* Check alias length */
	if (xParameterStringLength1 > MaxLengthOfAlias) {
		pcParameterString1[MaxLengthOfAlias] = '\0';
	}
	
	/* Name the module */
	result = NameModule(myID, (char*) pcParameterString1);
		
	/* Respond to the update command */
	if (result == BOS_OK)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, myID, pcParameterString1);
	else if (result == BOS_ERR_Keyword)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageKey, pcParameterString1);
	else if (result == BOS_ERR_ExistingAlias)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageAlias, pcParameterString1);	
	else if (result == BOS_ERR_ExistingCmd)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageCmd, pcParameterString1);	
	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE statusCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the status command */
	DisplayModuleStatus(0);
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE infoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Read Ports directions when a pre-defined topology file is used */
	if (N > 1)
		result = ReadPortsDir();
	
	/* Respond to the info command */
	sprintf( ( char * ) pcWriteBuffer, "\n\rNumber of modules: %d\n", N);
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	sprintf( ( char * ) pcWriteBuffer, "\n\rArray topology:\n");
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	DisplayTopology(PcPort);
	DisplayPortsDir(PcPort);
	if (result == BOS_ERR_NoResponse) {
		sprintf( ( char * ) pcWriteBuffer, "Could not read ports direction for some modules! Please try again\n\r");
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);		
	}
	sprintf( ( char * ) pcWriteBuffer, " ");
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE scastCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4; 
	static int8_t *pcParameterString5, *pcParameterString6, *pcParameterString7; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0, xParameterStringLength3 = 0; 
	portBASE_TYPE xParameterStringLength4 = 0, xParameterStringLength5 = 0, xParameterStringLength6 = 0;
	portBASE_TYPE xParameterStringLength7 = 0;
	uint8_t direction = 0, srcP = 0, dstP = 0, srcM = 0, dstM = 0; uint32_t count = 0, timeout = 0;
	char par1[MaxLengthOfAlias+1] = {0}, par2[MaxLengthOfAlias+1] = {0}, par3[MaxLengthOfAlias+1] = {0};
	
	static const int8_t *pcMessage = ( int8_t * ) "Activating a %s single-cast DMA stream from P%d in module %s to P%d in module %s. The stream will deactivate after %d bytes or %d ms\n\r";
	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (pcParameterString1[0] == 'P' || pcParameterString1[0] == 'p') {
		srcP = ( uint8_t ) atol( ( char * ) pcParameterString1+1 );
	}

	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	strncpy(par1, ( char * ) pcParameterString2, xParameterStringLength2);
	srcM = GetID(par1);
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	if (pcParameterString3[0] == 'P' || pcParameterString3[0] == 'p') {
		dstP = ( uint8_t ) atol( ( char * ) pcParameterString3+1 );
	}

	/* Obtain the 4th parameter string. */
	pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
	strncpy(par2, ( char * ) pcParameterString4, xParameterStringLength4);
	dstM = GetID(par2);
	
	/* Obtain the 5th parameter string. */
	pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);
	/* Read the color value. */
	if (!strncmp((const char *)pcParameterString5, "FORWARD", xParameterStringLength5) || !strncmp((const char *)pcParameterString5, "forward", xParameterStringLength5))
		direction = FORWARD;
	else if (!strncmp((const char *)pcParameterString5, "BACKWARD", xParameterStringLength5) || !strncmp(( const char *)pcParameterString5, "backward", xParameterStringLength5))
		direction = BACKWARD;
	else if (!strncmp((const char *)pcParameterString5, "BIDIRECTIONAL", xParameterStringLength5) || !strncmp((const char *)pcParameterString5, "bidirectional", xParameterStringLength5))
		direction = BIDIRECTIONAL;
	strncpy(par3, ( char * ) pcParameterString5, xParameterStringLength5);
	
	/* Obtain the 6th parameter string. */
	pcParameterString6 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 6, &xParameterStringLength6);
	count = ( uint32_t ) atol( ( char * ) pcParameterString6 );

	/* Obtain the 7th parameter string. */
	pcParameterString7 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 7, &xParameterStringLength7);
	timeout = ( uint32_t ) atol( ( char * ) pcParameterString7 );
	
	result = StartScastDMAStream(srcP, srcM, dstP, dstM, direction, count, timeout);
	
	/* Respond to the command */
	if (result == BOS_OK) 
	{	
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, par3, srcP, par1, dstP, par2, count, timeout);
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
