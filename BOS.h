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
#if !defined (H01R0) && !defined (H02R0) && !defined (H03R0) && !defined (H04R0) &&   \
    !defined (H05R0) && !defined (H06R0) && !defined (H07R0) && !defined (H08R0)                             
//  #define H01R0  /* RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3) (GPIOs) */  
//  #define H01R1  /* RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3) (All LEDs are timer channels) */  
//  #define H02R0  /* RGB LED */  
//  #define H03R0  /* RGB LED */  
//  #define H04R0  /* RGB LED */  
//  #define H05R0  /* RGB LED */  	                                        
#endif

/* Enumerations */
enum PortNames{PC, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P_USB};
enum PortStatus{FREE, MSG, STREAM, CLI};
enum UartDirection{NORMAL, REVERSED};
enum modulePartNumbers{_H01R0=1, _H01R1, _H02R0, _H04R0, _H05R0, _H07R0, _H08R0, _H09R0, _H11R0};
enum IndMode{IND_off, IND_ping, IND_topology};
enum DMAStreamDirection{FORWARD, BACKWARD, BIDIRECTIONAL};


/* Includes ------------------------------------------------------------------*/

/* STM HAL */
#include "stm32f0xx_hal.h" 

/* RTOS */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"	 

/* BOS */
#include "BOS_CLI.h"
#include "BOS_eeprom.h"

/* C STD Library */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>	 

/* Module includes and initialization */
#ifdef H01R0
	#include "H01R0.h"
	#include "H01R0_uart.h"	
	#include "H01R0_gpio.h"	
	#include "H01R0_dma.h"		
	#define	Module_Init							H01R0_Init
	#define	Module_MessagingTask		H01R0_MessagingTask
	#define Module_Status						H01R0_Status
#endif
#ifdef H02R0
	#include "H02R0.h"
	#include "H02R0_uart.h"	
	#include "H02R0_gpio.h"	
	#include "H02R0_dma.h"		
	#define	Module_Init							H02R0_Init
	#define	Module_MessagingTask		H02R0_MessagingTask
	#define Module_Status						H02R0_Status
#endif
#ifdef H04R0
	#include "H04R0.h"
	#include "H04R0_uart.h"	
	#include "H04R0_gpio.h"	
	#include "H04R0_dma.h"		
	#include "H04R0_dac.h"		
	#define	Module_Init							H04R0_Init
	#define	Module_MessagingTask		H04R0_MessagingTask
	#define Module_Status						H04R0_Status
#endif
#ifdef H05R0
	#include "H05R0.h"
	#include "H05R0_uart.h"	
	#include "H05R0_gpio.h"	
	#include "H05R0_dma.h"		
	#include "H05R0_spi.h"		
	#define	Module_Init							H05R0_Init
	#define	Module_MessagingTask		H05R0_MessagingTask
	#define Module_Status						H05R0_Status
#endif
#ifdef H07R0
	#include "H07R0.h"
	#include "H07R0_uart.h"	
	#include "H07R0_gpio.h"	
	#include "H07R0_dma.h"		
	#include "H07R0_adc.h"		
	#define	Module_Init							H07R0_Init
	#define	Module_MessagingTask		H07R0_MessagingTask
	#define Module_Status						H07R0_Status
#endif
#ifdef H08R0
	#include "H08R0.h"
	#include "H08R0_uart.h"	
	#include "H08R0_gpio.h"	
	#include "H08R0_dma.h"			
	#include "H08R0_spi.h"
	#define	Module_Init							H08R0_Init
	#define	Module_MessagingTask		H08R0_MessagingTask
	#define Module_Status						H08R0_Status
#endif
#ifdef H09R0
	#include "H09R0.h"
	#include "H09R0_uart.h"	
	#include "H09R0_gpio.h"	
	#include "H09R0_dma.h"			
	#define	Module_Init							H09R0_Init
	#define	Module_MessagingTask		H09R0_MessagingTask
	#define Module_Status						H09R0_Status
#endif
#ifdef H11R0
	#include "H11R0.h"
	#include "H11R0_uart.h"	
	#include "H11R0_gpio.h"	
	#include "H11R0_dma.h"		
	#define	Module_Init							H11R0_Init
	#define	Module_MessagingTask		H11R0_MessagingTask
	#define Module_Status						H11R0_Status
#endif

#define P_LAST 								NumOfPorts

/* Firmware */
#define	_firmVersion		"0.0.0"
#define _firmDate				__DATE__
#define _firmTime				__TIME__


/* BOS_Status Type Definition */  
typedef enum 
{
  BOS_OK = 0,
	BOS_ERR_UnknownMessage = 1,
  BOS_ERR_NoResponse = 2,
  BOS_ERR_UnIDedModule = 5,
	BOS_ERR_Keyword = 6,
	BOS_ERR_ExistingAlias = 7,
	BOS_ERR_ExistingCmd = 8,
	BOS_ERR_EEPROM = 10,
	BOS_ERR_WrongName = 100,
	BOS_ERR_WrongID = 101,
	BOS_BROADCAST = 255,
} BOS_Status;

/* BOS Parameters */ 
#define MAX_MESSAGE_SIZE			50
#define cmdMAX_INPUT_SIZE			50
#define	MaxNumOfModules				25
#define MaxNumOfPorts					10
#define MaxLengthOfAlias			10
#define NumOfKeywords					2

/* EEPROM virtual addresses - Consider MaxNumOfModules is 25 */
#define _EE_NBase						1	
#define _EE_topologyBase		2	
#define _EE_portDirBase			277	
#define _EE_aliasBase				303	
#define _EE_DMAStreamsBase	434
#define _EE_varBase					442

#if MaxNumOfModules > 25
 #warning "Only data for 25 modules will be stored in EEPROM."
#endif

/* Delay macros */
#define	Delay_us(t)			StartMicroDelay(t)		/* RTOS safe */
#define	Delay_ms(t)			HAL_Delay(t)					/* Non-RTOS safe */
#define	Delay_s(t)			HAL_Delay(1000*t)			/* Non-RTOS safe */

/* Serial Wire Interface */
#define SWDIO_PIN			GPIO_PIN_13
#define	SWDIO_PORT		GPIOA
#define	SWCLK_PIN			GPIO_PIN_14
#define	SWCLK_PORT		GPIOA


/* External variables ---------------------------------------------------------*/
extern char cRxedChar;
extern uint8_t myID;
extern uint16_t myPN;
extern uint8_t indMode;
extern uint8_t N;
extern const char modulePNstring[9][5];
extern const char BOSkeywords[NumOfKeywords][4];
extern uint8_t portStatus[NumOfPorts+1];
extern uint16_t neighbors[NumOfPorts][2];
extern uint8_t messageParams[20*(MAX_MESSAGE_SIZE-5)];
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE];
extern uint8_t messageLength[NumOfPorts];
extern SemaphoreHandle_t PxRxSemaphoreHandle[7];
extern SemaphoreHandle_t PxTxSemaphoreHandle[7];
static char pcUserMessage[80];
extern BOS_Status responseStatus;
#ifndef _N
	extern uint16_t array[MaxNumOfModules][MaxNumOfPorts+1];
	extern char moduleAlias[MaxNumOfModules+1][MaxLengthOfAlias+1];
	extern uint8_t broadcastResponse[MaxNumOfModules];
#else
	extern char moduleAlias[_N+1][MaxLengthOfAlias+1];
	extern uint8_t broadcastResponse[_N];
#endif
extern uint8_t routeDist[]; 
extern uint8_t routePrev[]; 
extern uint8_t route[];

/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/
#define	CODE_unknown_message			0
#define	CODE_ping									1
#define	CODE_ping_response				2

#define	CODE_IND_toggle						5

#define	CODE_hi										10
#define	CODE_hi_response					11
#define	CODE_explore_adj					12
#define	CODE_explore_adj_response	13
#define	CODE_port_dir							14
#define	CODE_module_id						15
#define	CODE_topology							16
#define	CODE_broadcast_plan				17
#define	CODE_read_port_dir				18
#define	CODE_read_port_dir_response		19
#define	CODE_exp_eeprom	 					20
#define	CODE_CLI_command 					21
#define	CODE_CLI_response  				22
#define	CODE_DMA_channel  				23
#define	CODE_DMA_scast_stream  		24


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

#define	NumberOfHops(i)		routeDist[i-1]

extern void BOS_Init(void);
extern UART_HandleTypeDef* GetUart(uint8_t port);
extern uint8_t GetPort(UART_HandleTypeDef *huart);
extern void vRegisterCLICommands(void);
extern BOS_Status SendMessageToModule(uint8_t dst, uint16_t code, uint16_t numberOfParams);
extern BOS_Status SendMessageFromPort(uint8_t port, uint8_t src, uint8_t dst, uint16_t code, uint16_t numberOfParams);
extern BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
extern BOS_Status BroadcastReceivedMessage(uint8_t IncomingPort);
extern void StartMicroDelay(uint16_t Delay);
extern BOS_Status Explore(void);
extern BOS_Status ExploreNeighbors(uint8_t ignore);
extern void SwapUartPins(UART_HandleTypeDef *huart, uint8_t direction);
extern uint8_t FindRoute(uint8_t sourceID, uint8_t desID);
extern void DisplayTopology(uint8_t port);
extern void DisplayPortsDir(uint8_t port);
extern void DisplayModuleStatus(uint8_t port);
extern uint8_t GetID(char* string);
extern BOS_Status NameModule(uint8_t module, char* alias);
extern BOS_Status ReadPortsDir(void);
extern BOS_Status UpdateMyPortsDir(void);
extern BOS_Status StartScastDMAStream(uint8_t srcP, uint8_t srcM, uint8_t dstP, uint8_t dstM, uint8_t direction, uint32_t count, uint32_t timeout);


#endif /* BOS_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
