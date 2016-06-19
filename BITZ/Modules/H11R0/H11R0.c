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



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
