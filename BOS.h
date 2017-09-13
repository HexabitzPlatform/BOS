/*
    BitzOS (BOS) V0.1.1 - Copyright (C) 2017 Hexabitz
    All rights reserved
		
    File Name     : BOS.h
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
    !defined (H05R0) && !defined (H06R0) && !defined (H07R0) && !defined (H08R0) && !defined (H12R0)                            
//  #define H01R0  /* RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3) (GPIOs) */  
//  #define H01R1  /* RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3) (All LEDs are timer channels) */  
//  #define H02R0  /* RGB LED */  
//  #define H03R0  /* RGB LED */  
//  #define H04R0  /* RGB LED */  
//  #define H05R0  /* RGB LED */  	                                        
#endif

/* Enumerations */
enum PortNames_e{PC, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P_USB};
enum ButtonNames_e{B1=1, B2, B3, B4, B5, B6, B7, B8, B9, B10};
enum PortStatus_e{FREE, MSG, STREAM, CLI, PORTBUTTON};
enum UartDirection_e{NORMAL, REVERSED};
enum modulePartNumbers_e{_H01R0=1, _H02R0, _H04R0, _H05R0, _H07R0, _H08R0, _H09R0, _H11R2, _H12R0};
enum IndMode_e{IND_OFF, IND_PING, IND_TOPOLOGY};
enum DMAStreamDirection_e{FORWARD, BACKWARD, BIDIRECTIONAL};
enum buttonType_e{NONE=0, MOMENTARY_NO, MOMENTARY_NC, ONOFF_NO, ONOFF_NC};		/* NO: Naturally Open, NC: Naturally CLosed */
enum buttonState_e{OFF=1, ON, OPEN, CLOSED, CLICKED, DBL_CLICKED, PRESSED, RELEASED, PRESSED_FOR_X1_SEC, PRESSED_FOR_X2_SEC,\
										 PRESSED_FOR_X3_SEC, RELEASED_FOR_Y1_SEC, RELEASED_FOR_Y2_SEC, RELEASED_FOR_Y3_SEC};

/* Includes ------------------------------------------------------------------*/

/* STM HAL */
#include "stm32f0xx_hal.h" 
										 
/* Project Header File */
#include "project.h" 

/* RTOS */
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"	 

/* BOS */
#include "BOS_eeprom.h"

/* C STD Library */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>	 
#include <limits.h>	

/* Module includes and initialization */
#ifdef H01R0
	#include "H01R0.h"
#endif
#ifdef H02R0
	#include "H02R0.h"	
#endif
#ifdef H04R0
	#include "H04R0.h"	
#endif
#ifdef H05R0
	#include "H05R0.h"	
#endif
#ifdef H07R0
	#include "H07R0.h"	
#endif
#ifdef H08R0
	#include "H08R0.h"
#endif
#ifdef H09R0
	#include "H09R0.h"		
#endif
#ifdef H11R0
	#include "H11R0.h"	
#endif
#ifdef H12R0
	#include "H12R0.h"	
#endif


#define P_LAST 								NumOfPorts

/* Firmware */
#define	_firmMajor			0
#define	_firmMinor			1
#define	_firmPatch			1
#define _firmDate				__DATE__
#define _firmTime				__TIME__


/* BOS_Status Type Definition */  
typedef enum 
{
  BOS_OK = 0,
	BOS_ERR_UnknownMessage = 1,
  BOS_ERR_NoResponse = 2,
	BOS_ERR_MSG_Reflection = 3,
  BOS_ERR_UnIDedModule = 5,
	BOS_ERR_Keyword = 6,
	BOS_ERR_ExistingAlias = 7,
	BOS_ERR_ExistingCmd = 8,
	BOS_ERR_EEPROM = 10,
	BOS_ERR_BUTTON_NOT_DEFINED = 11,
	BOS_ERR_BUTTON_PRESS_EVENT_FULL = 12,
	BOS_ERR_BUTTON_RELEASE_EVENT_FULL = 13,
	BOS_ERR_SNIP_MEM_FULL = 14,
	BOS_ERR_WrongName = 100,
	BOS_ERR_WrongID = 101,
	BOS_ERR_WrongParam = 102,
	BOS_ERR_WrongValue = 103,
	BOS_MEM_ERASED = 250,
	BOS_BROADCAST = 255,
	BOS_ERROR = 255
} BOS_Status;

/* Button Configuration Struct Type Definition */  
typedef struct
{
	uint16_t debounce;
	uint16_t singleClickTime;
	uint8_t minInterClickTime;
	uint8_t maxInterClickTime;
} 
buttonsConfig_t;

/* BOS Struct Type Definition */  
typedef struct
{
	buttonsConfig_t buttons;
	uint8_t response;
	uint32_t clibaudrate;
} 
BOS_t;

/* Button Struct Type Definition */  
typedef struct
{
	uint8_t state;
	uint8_t type;
	uint8_t pressedX1Sec;
	uint8_t pressedX2Sec;
	uint8_t pressedX3Sec;
	uint8_t releasedY1Sec;
	uint8_t releasedY2Sec;
	uint8_t releasedY3Sec;
	uint8_t events;
} 
button_t;

/* Button Events Definition */ 
#define	BUTTON_EVENT_CLICKED									0x01
#define	BUTTON_EVENT_DBL_CLICKED							0x02
#define	BUTTON_EVENT_PRESSED_FOR_X1_SEC				0x04
#define	BUTTON_EVENT_PRESSED_FOR_X2_SEC				0x05
#define	BUTTON_EVENT_PRESSED_FOR_X3_SEC				0x10
#define	BUTTON_EVENT_RELEASED_FOR_Y1_SEC			0x20
#define	BUTTON_EVENT_RELEASED_FOR_Y2_SEC			0x40
#define	BUTTON_EVENT_RELEASED_FOR_Y3_SEC			0x80



/* BOS Parameters */ 
#define MAX_MESSAGE_SIZE					50
#define cmdMAX_INPUT_SIZE					50
#define	MaxNumOfModules						25
#define MaxNumOfPorts							10
#define MaxLengthOfAlias					10
#define NumOfKeywords							2
#define NumOfParamsHelpStrings		6
#define DEF_BUTTON_DEBOUNCE						30				// Button debounce time in ms
#define DEF_BUTTON_CLICK							50				// Button single click minimum time in ms
#define DEF_BUTTON_MIN_INTER_CLICK		5					// Button min inter-click time (in ms) for double clicks (uint8_t size)
#define DEF_BUTTON_MAX_INTER_CLICK		250				// Button max inter-click time (in ms) for double clicks (uint8_t size)
#define BOS_RESPONSE_ALL							0x60			// Send response messages for both Messaging and CLI
#define BOS_RESPONSE_MSG							0x40			// Send response messages for Messaging only (no CLI)
#define BOS_RESPONSE_NONE							0x00			// Do not send any response messages
#define DEF_ARRAY_BAUDRATE						921600
#define DEF_CLI_BAUDRATE							921600
#define CLI_BAUDRATE_1								115200

/* Command Snippets */
#define SNIPPETS_BUF_SIZE							1000
#define SNIPPET_CONDITION							0x80			// Conditional statement - condition delimiter
#define SNIPPET_CONDITION_CMDS				0x82			// Conditional statement - command delimiter
#define SNIPPET_END										0xF0			// End of Snippet delimiter


/* Delay macros */
#define	Delay_us(t)							StartMicroDelay(t)		/* RTOS safe (16 bits) - Use before and after starting the scheduler */
#define	Delay_ms_no_rtos(t)			StartMilliDelay(t)		/* RTOS safe (16 bits) - Use before and after starting the scheduler */
#define	Delay_ms(t)							HAL_Delay(t)					/* Non-RTOS safe (32 bits) - Use only after starting the scheduler */
#define	Delay_s(t)							HAL_Delay(1000*t)			/* Non-RTOS safe (32 bits) - Use only after starting the scheduler */

/* Serial Wire Interface */
#define SWDIO_PIN			GPIO_PIN_13
#define	SWDIO_PORT		GPIOA
#define	SWCLK_PIN			GPIO_PIN_14
#define	SWCLK_PORT		GPIOA


/* External variables ---------------------------------------------------------*/
extern char cRxedChar;
extern uint8_t myID, bcastID;
extern uint16_t myPN;
extern uint8_t indMode;
extern uint8_t N;
extern const char modulePNstring[10][5];
extern const char BOSkeywords[NumOfKeywords][4];
extern uint8_t portStatus[NumOfPorts+1];
extern uint16_t neighbors[NumOfPorts][2];
extern uint8_t messageParams[20*(MAX_MESSAGE_SIZE-5)];
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE];
extern uint8_t messageLength[NumOfPorts];
extern SemaphoreHandle_t PxRxSemaphoreHandle[7];
extern SemaphoreHandle_t PxTxSemaphoreHandle[7];
static char pcUserMessage[80];
extern const char * pcParamsHelpString[];
extern BOS_Status responseStatus;
#ifndef _N
	extern char moduleAlias[MaxNumOfModules+1][MaxLengthOfAlias+1];
	extern uint8_t broadcastResponse[MaxNumOfModules];
#else
	extern char moduleAlias[_N+1][MaxLengthOfAlias+1];
	extern uint8_t broadcastResponse[_N];
#endif
extern uint8_t routeDist[]; 
extern uint8_t routePrev[]; 
extern uint8_t route[];
extern button_t button[NumOfPorts+1];
extern BOS_t BOS;
extern uint8_t PcPort;
extern uint8_t deferButtonReset;
extern uint8_t BOS_initialized;

/* Exported internal functions ---------------------------------------------------------*/
void StringToLowerCase(char *string);
extern BOS_Status UpdateBaudrate(uint8_t port, uint32_t baudrate);

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
#define IND_ON()				HAL_GPIO_WritePin(_IND_LED_PORT,_IND_LED_PIN,GPIO_PIN_SET)		
#define IND_OFF()				HAL_GPIO_WritePin(_IND_LED_PORT,_IND_LED_PIN,GPIO_PIN_RESET)		
#define IND_blink(t)				IND_ON();	HAL_Delay(t); IND_OFF()		/* Use after starting the scheduler */
#define RTOS_IND_blink(t)		IND_ON();	osDelay(t); IND_OFF()			/* Use after starting the scheduler */

#define	NumberOfHops(i)		routeDist[i-1]

extern void BOS_Init(void);
extern UART_HandleTypeDef* GetUart(uint8_t port);
extern uint8_t GetPort(UART_HandleTypeDef *huart);
extern void vRegisterCLICommands(void);
extern BOS_Status SendMessageToModule(uint8_t dst, uint16_t code, uint16_t numberOfParams);
extern BOS_Status SendMessageFromPort(uint8_t port, uint8_t src, uint8_t dst, uint16_t code, uint16_t numberOfParams);
extern BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
extern BOS_Status BroadcastReceivedMessage(uint8_t IncomingPort);
extern BOS_Status BroadcastMessage(uint8_t incomingPort, uint8_t src, uint16_t code, uint16_t numberOfParams);
extern void StartMicroDelay(uint16_t Delay);
extern void StartMilliDelay(uint16_t Delay);
extern BOS_Status Explore(void);
extern BOS_Status ExploreNeighbors(uint8_t ignore);
extern BOS_Status FindBroadcastRoutes(uint8_t src);
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
extern BOS_Status AddPortButton(uint8_t buttonType, uint8_t port);
extern BOS_Status RemovePortButton(uint8_t port);
extern BOS_Status SetButtonEvents(uint8_t port, uint8_t clicked, uint8_t dbl_clicked, uint8_t pressed_x1sec, uint8_t pressed_x2sec, uint8_t pressed_x3sec,\
													uint8_t released_y1sec, uint8_t released_y2sec, uint8_t released_y3sec);




#endif /* BOS_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
