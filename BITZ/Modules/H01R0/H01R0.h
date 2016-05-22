/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
		
    File Name     : H01R0.c
    Description   : Header file for module H01R0.
										RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3)
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H01R0_H
#define H01R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H01R0

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1
	
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT	GPIOB
#define	USART3_RX_PORT	GPIOB
#define	USART3_AF				GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	GPIOB
#define	USART5_RX_PORT	GPIOB
#define	USART5_AF				GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT	GPIOA
#define	USART6_RX_PORT	GPIOA
#define	USART6_AF				GPIO_AF5_USART6

/* Module-specific Definitions */
#ifdef H01R0
	#define _RGB_LED_PORT			GPIOB
	#define _RGB_LED_RED			GPIO_PIN_2
	#define _RGB_LED_GREEN		GPIO_PIN_0
	#define _RGB_LED_BLUE			GPIO_PIN_1
#endif

/* H01R0_Status Type Definition */  
typedef enum 
{
  H01R0_OK = 0x00,
  H01R0_ERR_WrongColor = 0x01,
	H01R0_ERR_WrongIntensity = 0x02,
} H01R0_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_11

/* Color Enumerations */
enum BasicColors{BLACK=1, WHITE, RED, BLUE, YELLOW, CYAN, MAGENTA, GREEN};

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);

extern uint8_t RGB_LED_State;
extern uint8_t RGB_LED_Intensity_Old;

/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/

#define	CODE_H01R0_on							20
#define	CODE_H01R0_off						21
#define	CODE_H01R0_toggle					22

	
/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/
#ifdef H01R0
	extern void H01R0_Init(void);
	extern H01R0_Status RGB_LED_setRGB(uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity);
#endif
#ifdef H01R1
	extern void H01R1_Init(void);
	extern void startPWM_RED(uint16_t period, uint16_t width);
#endif
extern H01R0_Status RGB_LED_setColor(uint8_t color, uint8_t intensity);
extern H01R0_Status RGB_LED_on(uint8_t intensity);
extern H01R0_Status RGB_LED_off(void);

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/
portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE offCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE colorCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE RGBCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
extern const CLI_Command_Definition_t onCommandDefinition;
extern const CLI_Command_Definition_t offCommandDefinition;
extern const CLI_Command_Definition_t colorCommandDefinition;
extern const CLI_Command_Definition_t RGBCommandDefinition;





#endif /* H01R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
