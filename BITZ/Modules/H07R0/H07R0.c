/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H07R0.c
    Description   : Source code for module H07R0.
										Analog inputs 0 - 5V/10V (AD628ARZ) 
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4 for module ports.
			>> ADC_IN8 for AD628ARZ.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/




/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H07R0 message processing task. 
*/
H07R0_Status H07R0_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	H07R0_Status result = H07R0_OK;
	
	switch (code)
	{

		default:
			result = H07R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/



/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- H07R0 module initialization. 
*/
void H07R0_Init(void)
{	
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
	
	/* ADC */
	//MX_ADC_Init();
  
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
