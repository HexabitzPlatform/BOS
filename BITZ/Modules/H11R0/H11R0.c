/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H11R0.c
    Description   : Source code for module H11R0.
										USB 2.0 - UART (FT230XQ)
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,5,6 for module ports.
			>> USART 4 for FT230XQ.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/

//portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );


///* CLI command structure : off */
//const CLI_Command_Definition_t offCommandDefinition =
//{
//	( const int8_t * ) "off", /* The command string to type. */
//	( const int8_t * ) "(H01R0) off:\r\n Turn RGB LED off\r\n\r\n",
//	offCommand, /* The function to run. */
//	0 /* No parameters are expected. */
//};
/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H11R0 message processing task. 
*/
H11R0_Status H11R0_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	H11R0_Status result = H11R0_OK;
	
	switch (code)
	{
//		case CODE_H01R0_on :
//			break;
		
		default:
			result = H11R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/



/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- H11R0 module initialization. 
*/
void H11R0_Init(void)
{
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	
	/* USB port */
  MX_USART4_UART_Init();
	
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

//portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
//{
//	H01R0_Status result = H01R0_OK;
//	
//	int8_t *pcParameterString1; portBASE_TYPE xParameterStringLength1 = 0; 
//	uint8_t intensity = 0;
//	static const int8_t *pcOKMessage = ( int8_t * ) "RGB LED is on at intensity %d%%\r\n";
//	static const int8_t *pcWrongIntensityMessage = ( int8_t * ) "Wrong intensity!\n\r";
//	
//	/* Remove compile time warnings about unused parameters, and check the
//	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
//	write buffer length is adequate, so does not check for buffer overflows. */
//	( void ) xWriteBufferLen;
//	configASSERT( pcWriteBuffer );
//	
//	/* Obtain the 1st parameter string. */
//	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter
//								(
//									pcCommandString,		/* The command string itself. */
//									1,						/* Return the first parameter. */
//									&xParameterStringLength1	/* Store the parameter string length. */
//								);
//	intensity = ( uint8_t ) atol( ( char * ) pcParameterString1 );
//	
//	result = RGB_LED_on(intensity);	
//	
//	/* Respond to the command */
//	if (result == H01R0_OK)
//		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOKMessage, intensity);
//	else if (result == H01R0_ERR_WrongIntensity)
//		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongIntensityMessage);
//	
//	/* There is no more data to return after this single string, so return
//	pdFALSE. */
//	return pdFALSE;
//}

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
