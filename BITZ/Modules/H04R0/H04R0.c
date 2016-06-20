/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H04R0.c
    Description   : Source code for module H04R0.
										Sound speaker (Knowles) with audio amp (TS4990IST) and headset port 
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4,5 for module ports.
			>> DAC_OUT 1 for TS4990IST.
			>> PB0 for TS4990IST STDBY.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/




/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H04R0 message processing task. 
*/
H04R0_Status H04R0_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	H04R0_Status result = H04R0_OK;
	
	switch (code)
	{

		default:
			result = H04R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/



/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- H04R0 module initialization. 
*/
void H04R0_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
	
	/* DAC */
	MX_DAC_Init();
	
	/* GPIO */
	GPIO_InitStruct.Pin = _STDBY_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(_STDBY_PORT, &GPIO_InitStruct);
  
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
