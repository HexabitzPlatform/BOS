/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H09R0.c
    Description   : Source code for module H09R0.
										Solid state relay (AQH3213A) 
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,5,6 for module ports.
			>> PB0 for AQH3213A.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

uint8_t SSR_State = 0, SSRindMode = 0;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/	
void SSRTimerCallback( TimerHandle_t xTimer );

/* Create CLI commands --------------------------------------------------------*/

portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE offCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE toggleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE ledModeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : on */
const CLI_Command_Definition_t onCommandDefinition =
{
	( const int8_t * ) "on", /* The command string to type. */
	( const int8_t * ) "(H09R0) on:\r\n Turn solid state relay on with a timeout (ms) (1st par.)\r\n\r\n",
	onCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : off */
const CLI_Command_Definition_t offCommandDefinition =
{
	( const int8_t * ) "off", /* The command string to type. */
	( const int8_t * ) "(H09R0) off:\r\n Turn solid state relay off\r\n\r\n",
	offCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : toggle */
const CLI_Command_Definition_t toggleCommandDefinition =
{
	( const int8_t * ) "toggle", /* The command string to type. */
	( const int8_t * ) "(H09R0) toggle:\r\n Toggle solid state relay\r\n\r\n",
	toggleCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ledMode */
const CLI_Command_Definition_t ledModeCommandDefinition =
{
	( const int8_t * ) "ledMode", /* The command string to type. */
	( const int8_t * ) "(H09R0) ledMode:\r\n Set solid state relay indicator LED mode ('on' or 'off') (1st par.)\r\n\r\n",
	ledModeCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H09R0 message processing task. 
*/
H09R0_Status H09R0_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	H09R0_Status result = H09R0_OK;
	
	switch (code)
	{

		default:
			result = H09R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Solid State Relay timer callback --- 
*/
void SSRTimerCallback( TimerHandle_t xTimer )
{
	SSR_off();
}

/*-----------------------------------------------------------*/	


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- H09R0 module initialization --- 
*/
void H09R0_Init(void)
{	
	
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	
	/* SSR GPIO */
	SSR_Init();
  
}

/*-----------------------------------------------------------*/

/* --- Turn on the solid state relay ---
*/
H09R0_Status SSR_on(uint32_t timeout)
{	
	H09R0_Status result = H09R0_OK;	
	TimerHandle_t xTimer = NULL;
	
	HAL_GPIO_WritePin(_SSR_PORT,_SSR_PIN,GPIO_PIN_SET);
	
	/* Indicator LED */
	if (SSRindMode) {
		IND_on();
	}
	
	/* Timeout */
	if (timeout != portMAX_DELAY) {
		/* Create a timeout timer */
		xTimer = xTimerCreate( "SSRTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) 1, SSRTimerCallback );	
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	
	/* Update SSR state */
	SSR_State = 1;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Turn off the solid state relay ---
*/
H09R0_Status SSR_off(void)
{	
	H09R0_Status result = H09R0_OK;	
	
	HAL_GPIO_WritePin(_SSR_PORT,_SSR_PIN,GPIO_PIN_RESET);
	
	/* Indicator LED */
	if (SSRindMode) {
		IND_off();
	}	

	/* Update SSR state */
	SSR_State = 0;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Toggle the solid state relay ---
*/
H09R0_Status SSR_toggle(void)
{	
	H09R0_Status result = H09R0_OK;	
	
	if (SSR_State) 
		result = SSR_off();
	else 
		result = SSR_on(portMAX_DELAY);
	
	return result;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H09R0_Status result = H09R0_OK;
	
	int8_t *pcParameterString1; portBASE_TYPE xParameterStringLength1 = 0; 
	uint32_t timeout = 0;
	static const int8_t *pcOKMessage = ( int8_t * ) "Solid state relay is turned on with timeout %d ms\r\n";
	static const int8_t *pcOKMessageInf = ( int8_t * ) "Solid state relay is turned on without timeout\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,		/* The command string itself. */
									1,						/* Return the first parameter. */
									&xParameterStringLength1	/* Store the parameter string length. */
								);
	timeout = ( uint32_t ) atol( ( char * ) pcParameterString1 );
	
	result = SSR_on(timeout);	
	
	/* Respond to the command */
	if (result == H09R0_OK) {
		if (timeout != portMAX_DELAY) {
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOKMessage, timeout);
		} else {
			strcpy( ( char * ) pcWriteBuffer, ( char * ) pcOKMessageInf);
		}
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE offCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H09R0_Status result = H09R0_OK;
	
	static const int8_t *pcMessage = ( int8_t * ) "Solid state relay is turned off\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	result = SSR_off();
	
	/* Respond to the command */
	if (result == H09R0_OK) {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessage);
	}
		
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE toggleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H09R0_Status result = H09R0_OK;
	
	static const int8_t *pcOK1Message = ( int8_t * ) "Solid state relay is turned on\r\n";
	static const int8_t *pcOK0Message = ( int8_t * ) "Solid state relay is turned off\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	result = SSR_toggle();	
	
	/* Respond to the command */
	if (result == H09R0_OK) {
		if (SSR_State) {
			strcpy( ( char * ) pcWriteBuffer, ( char * ) pcOK1Message);
		} else {
			strcpy( ( char * ) pcWriteBuffer, ( char * ) pcOK0Message);
		}
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE ledModeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	int8_t *pcParameterString1; portBASE_TYPE xParameterStringLength1 = 0; 
	
	static const int8_t *pcOK1Message = ( int8_t * ) "Solid state relay indicator LED is enabled\r\n";
	static const int8_t *pcOK0Message = ( int8_t * ) "Solid state relay indicator LED is disabled\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,		/* The command string itself. */
									1,						/* Return the first parameter. */
									&xParameterStringLength1	/* Store the parameter string length. */
								);
	if (!strcmp( ( char * ) pcParameterString1, "on") || !strcmp( ( char * ) pcParameterString1, "ON"))
		SSRindMode = 1;
	else if (!strcmp( ( char * ) pcParameterString1, "off") || !strcmp( ( char * ) pcParameterString1, "OFF"))
		SSRindMode = 0;
	
	/* Respond to the command */
	if (SSRindMode) {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcOK1Message);
	} else {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcOK0Message);
	}

	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
