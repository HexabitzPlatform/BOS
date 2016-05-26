/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
		
    File Name     : BOS.c
    Description   : Header file for Bitz Operating System (BOS).
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_H
#define BOS_H


/* Module part number */
/*  Uncomment the line below according to the module used in your application.
		Tip: To avoid modifying this file each time you need to switch between these
    modules, you can define the module in your toolchain compiler preprocessor. 
*/
#if !defined (H01R0) && !defined (H01R1) && !defined (H02R0) && !defined (H03R0) && !defined (H04R0) &&   \
    !defined (H05R0) && !defined (H06R0) && !defined (H07R0) && !defined (H08R0)                             
//  #define H01R0  /* RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3) (GPIOs) */  
//  #define H01R1  /* RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3) (All LEDs are timer channels) */  
//  #define H02R0  /* RGB LED */  
//  #define H03R0  /* RGB LED */  
//  #define H04R0  /* RGB LED */  
//  #define H05R0  /* RGB LED */  	                                        
#endif

/* Enumerations */
enum PortNames{PC, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10};
enum PortStatus{FREE, MSG, STREAM, CLI};
enum UartDirection{NORMAL, REVERSED};
enum modulePartNumbers{_H01R0=1, _H01R1, _H02R0};


/* Includes ------------------------------------------------------------------*/

/* STM HAL */
#include "stm32f0xx_hal.h" 

/* RTOS */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
	 
/* RTOS CLI */
#include "FreeRTOS_CLI.h"
#include "BOS_CLI.h"

/* C STD Library */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>	 

/* Include a predefined topology here */
//#include "topology_ex.h"


/* Module includes and initialization */
#ifdef H01R0
	#include "H01R0.h"
	#include "H01R0_uart.h"	
	#include "H01R0_gpio.h"	
	#include "H01R0_dma.h"		
	#define	Module_Init		H01R0_Init
#endif
#ifdef H01R1
	#include "H01R0.h"
	#define	Module_Init		H01R1_Init
#endif

/* Number of ports */
#if defined (H01R0) || defined (H01R1) || defined (H02R0)
	#define	NumOfPorts		6
#endif

/* Firmware */
#define	_firmVersion		"FOR001"
#define _firmDate				__DATE__
#define _firmTime				__TIME__


/* BOS_Status Type Definition */  
typedef enum 
{
  BOS_OK       	= 0x00,
  BOS_ERR_UnIDedModule    = 0x05,
} BOS_Status;

/* BOS Parameters */ 
#define MAX_MESSAGE_SIZE			50
#define	MaxNumOfModules				50
#define MaxNumOfPorts					10
#define	MSG_ACK								0x65


/* Delay macros */
#define	Delay_us(t)			StartMicroDelay(t)		/* RTOS safe */
#define	Delay_ms(t)			HAL_Delay(t)					/* Non-RTOS safe */
#define	Delay_s(t)			HAL_Delay(1000*t)			/* Non-RTOS safe */


/* Port-UART mapping */
#if defined (H01R0) || defined (H01R1)
	#define P1uart &huart4	
	#define P2uart &huart2
	#define P3uart &huart6
	#define P4uart &huart3
	#define P5uart &huart1
	#define P6uart &huart5
#endif
//#if (HO01R2 || HO02R1)
//	#ifndef P1uart	
//		#define P1uart &huart2
//	#endif
//	#ifndef P2uart	
//		#define P2uart &huart6
//	#endif
//	#ifndef P3uart
//		#define P3uart &huart3
//	#endif
//	#ifndef P4uart	
//		#define P4uart &huart5
//	#endif
//	#ifndef P5uart	
//		#define P5uart &huart1
//	#endif
//	#ifndef P6uart	
//		#define P6uart &huart4
//	#endif
//#endif


/* External variables ---------------------------------------------------------*/
extern char cRxedChar;
extern uint8_t myID;
extern uint16_t myPN;
extern uint8_t N;
extern const char modulePNstring[4][5];
extern uint8_t portStatus[NumOfPorts+1];
extern uint16_t neighbors[NumOfPorts][2];
extern uint8_t messageParams[20*(MAX_MESSAGE_SIZE-5)];
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE];
extern uint8_t messageLength[NumOfPorts];
extern SemaphoreHandle_t PxRxSemaphoreHandle[7];
extern SemaphoreHandle_t PxTxSemaphoreHandle[7];
#ifndef _N
extern uint16_t array[50][MaxNumOfPorts+1];
#endif
extern uint8_t routeDist[]; 
extern uint8_t routePrev[]; 
extern uint8_t route[];

/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/
#define	CODE_ping									1
#define	CODE_IND_toggle						2

#define	CODE_hi										4
#define	CODE_hi_response					5

#define	CODE_task_stats						6
#define	CODE_run_time_stats				7

#define	CODE_explore_adj					10
#define	CODE_explore_adj_response	11
#define	CODE_port_dir							12
#define	CODE_module_id						13
#define	CODE_topology							14
#define	CODE_broadcast_plan				15


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* Indicator LED */
#define IND_toggle()		HAL_GPIO_TogglePin(_IND_LED_PORT,_IND_LED_PIN)		
#define IND_on()				HAL_GPIO_WritePin(_IND_LED_PORT,_IND_LED_PIN,GPIO_PIN_SET)		
#define IND_off()				HAL_GPIO_WritePin(_IND_LED_PORT,_IND_LED_PIN,GPIO_PIN_RESET)		
#define IND_blink(t)				IND_on();	HAL_Delay(t); IND_off()		/* Use before starting the scheduler */
#define RTOS_IND_blink(t)		IND_on();	osDelay(t); IND_off()			/* Use after starting the scheduler */

extern void BOS_Init(void);
extern UART_HandleTypeDef* GetUart(uint8_t port);
extern uint8_t GetPort(UART_HandleTypeDef *huart);
extern void vRegisterCLICommands(void);
extern BOS_Status SendMessageToModule(uint8_t dst, uint16_t code, uint16_t numberOfParams);
extern BOS_Status SendMessageFromPort(uint8_t port, uint8_t dst, uint16_t code, uint16_t numberOfParams);
extern BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
extern BOS_Status BroadcastReceivedMessage(uint8_t IncomingPort);
extern void StartMicroDelay(uint16_t Delay);
extern BOS_Status Explore(void);
extern BOS_Status ExploreNeighbors(uint8_t ignore);
extern void SwapUartPins(UART_HandleTypeDef *huart, uint8_t direction);
extern uint8_t FindRoute(uint8_t sourceID, uint8_t desID);
extern void DisplayTopology(uint8_t port);


#endif /* BOS_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
