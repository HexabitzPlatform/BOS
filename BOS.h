/*
    BitzOS (BOS) V0.2.1 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : BOS.h
    Description   : Header file for Bitz Operating System (BOS).
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_H
#define BOS_H

/* Includes ------------------------------------------------------------------*/

#include "BOS_MsgCodes.h" 

/* STM HAL */
#include "stm32f0xx_hal.h" 

/* Firmware */
#define	_firmMajor			0
#define	_firmMinor			2
#define	_firmPatch			1
#define _firmDate				__DATE__
#define _firmTime				__TIME__

/* Enumerations */
enum PortNames_e{PC, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, PUSB, P_RS485};
enum ButtonNames_e{B1=1, B2, B3, B4, B5, B6, B7, B8, B9, B10};
enum PortStatus_e{FREE, MSG, STREAM, CLI, PORTBUTTON, OVERRUN, CUSTOM};
enum UartDirection_e{NORMAL, REVERSED};
enum modulePartNumbers_e{_H01R0=1, _P01R0, _H23R0, _H23R1, _H07R3, _H08R6, _P08R6, _H09R0, _H1BR6, _H12R0, _H13R7, _H0FR1, _H0FR6, _H1AR2, _H0AR9, _H1DR1, _H1DR5, _H0BR4, _H18R0, _H26R0};
enum IndMode_e{IND_OFF, IND_PING, IND_TOPOLOGY, IND_SHORT_BLINK};
enum DMAStreamDirection_e{FORWARD, BACKWARD, BIDIRECTIONAL};
enum buttonType_e{NONE=0, MOMENTARY_NO, MOMENTARY_NC, ONOFF_NO, ONOFF_NC};		/* NO: Naturally Open, NC: Naturally CLosed */
enum buttonState_e{OFF=1, ON, OPEN, CLOSED, CLICKED, DBL_CLICKED, PRESSED, RELEASED, PRESSED_FOR_X1_SEC, PRESSED_FOR_X2_SEC,\
										 PRESSED_FOR_X3_SEC, RELEASED_FOR_Y1_SEC, RELEASED_FOR_Y2_SEC, RELEASED_FOR_Y3_SEC};
enum bootStatus_e{POWER_ON_BOOT, RESET_BOOT};

/* Color Enumerations */
enum BasicColors{BLACK=1, WHITE, RED, BLUE, YELLOW, CYAN, MAGENTA, GREEN};


/* RGB LED Mode Enumerations */
enum RGBLedMode{RGB_PULSE_RGB=1, RGB_PULSE_COLOR, RGB_SWEEP_BASIC, RGB_SWEEP_FINE, RGB_DIM_UP, RGB_DIM_UP_WAIT, RGB_DIM_DOWN, RGB_DIM_DOWN_WAIT,\
	RGB_DIM_UP_DOWN, RGB_DIM_DOWN_UP, RGB_DIM_UP_DOWN_WAIT, RGB_DIM_DOWN_UP_WAIT};


/* RTC Enums */
enum rtc_ampm_e{RTC_AM = 1, RTC_PM};
enum rtc_daylight_e{DAYLIGHT_SUB1H = -1, DAYLIGHT_NONE = 0, DAYLIGHT_ADD1H = 1};
enum rtc_months_e{JANUARY = 1, FEBRUARY, MARCH, APRIL, MAY, JUNE, JULY, AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER};
enum rtc_weekdays_e{MONDAY = 1, TUESDAY, WEDNESDAY, THURSDAY, FRIDAY, SATURDAY, SUNDAY};  
/* Type definitions */
typedef enum { FMT_UINT8 = 1, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL } varFormat_t;
typedef enum { TRACE_NONE = 0, TRACE_MESSAGE, TRACE_RESPONSE, TRACE_BOTH } traceOptions_t;


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
	BOS_ERR_REMOTE_READ_TIMEOUT = 15,
	BOS_ERR_REMOTE_READ_NO_VAR = 16,
	BOS_ERR_REMOTE_WRITE_TIMEOUT = 17,
	BOS_ERR_REMOTE_WRITE_MEM_FULL = 18,
	BOS_ERR_REMOTE_WRITE_INDEX = 19,
	BOS_ERR_LOCAL_FORMAT_UPDATED = 20,
	BOS_ERR_REMOTE_WRITE_ADDRESS = 21,
	BOS_ERR_REMOTE_WRITE_FLASH = 22,
	BOS_ERR_PORT_BUSY = 23,
	BOS_ERR_WrongName = 100,
	BOS_ERR_WrongGroup = 101,
	BOS_ERR_WrongID = 102,
	BOS_ERR_WrongParam = 103,
	BOS_ERR_WrongValue = 104,
	BOS_ERR_MSG_DOES_NOT_FIT = 105,
	BOS_MEM_ERASED = 250,
	BOS_MEM_FULL = 251,
	BOS_MULTICAST = 254,
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

/* Time/Date Struct Type Definition */  
typedef struct
{
	uint16_t msec;
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t ampm;
} 
BOS_time_t;
typedef struct
{
	uint8_t weekday;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} 
BOS_date_t;

/* BOS Struct Type Definition */  
typedef struct
{
	buttonsConfig_t buttons;
	uint8_t response;
	traceOptions_t trace;
	uint32_t clibaudrate;
	uint8_t daylightsaving;
	uint8_t hourformat;
	BOS_time_t time;						// Not saved with BOS parameters
	BOS_date_t date;						// Not saved with BOS parameters
	uint8_t overrun;
	uint8_t disableCLI;
} 
BOS_t;

/* Module Parameter Struct Type Definition */  
typedef struct
{
	void *paramPtr;
	varFormat_t paramFormat;
	char *paramName;
} 
module_param_t;
extern module_param_t modParam[];

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

/* Snippet Conditionals Struct Type Definition */  
typedef struct
{
	uint8_t conditionType;
	uint8_t mathOperator;
	uint8_t buffer1[4];
	uint8_t buffer2[4];
} 
snippetConditions_t;

/* Snippet Struct Type Definition */  
typedef struct
{
	snippetConditions_t cond;
	char *cmd;
	uint8_t state;
} 
snippet_t;

/* Button Events Definition */ 
#define	BUTTON_EVENT_CLICKED									0x01
#define	BUTTON_EVENT_DBL_CLICKED							0x02
#define	BUTTON_EVENT_PRESSED_FOR_X1_SEC				0x04
#define	BUTTON_EVENT_PRESSED_FOR_X2_SEC				0x08
#define	BUTTON_EVENT_PRESSED_FOR_X3_SEC				0x10
#define	BUTTON_EVENT_RELEASED_FOR_Y1_SEC			0x20
#define	BUTTON_EVENT_RELEASED_FOR_Y2_SEC			0x40
#define	BUTTON_EVENT_RELEASED_FOR_Y3_SEC			0x80
#define	BUTTON_EVENT_MODE_CLEAR								0
#define	BUTTON_EVENT_MODE_OR									1

/* BOS Defiitions */
#define BOS_RESPONSE_ALL							0x60			// Send response messages for both Messaging and CLI
#define BOS_RESPONSE_MSG							0x20			// Send response messages for Messaging only (no CLI)
#define BOS_RESPONSE_CLI							0x40			// Send response messages for CLI only (no messages)
#define BOS_RESPONSE_NONE							0x00			// Do not send any response messages
#define REMOTE_MEMORY_ADD             0
#define REMOTE_BOS_PARAM              1
#define REMOTE_MODULE_PARAM           2
#define REMOTE_BOS_VAR                3

/* Math Operators */
#define MATH_EQUAL										1
#define MATH_GREATER									2
#define MATH_SMALLER									3
#define MATH_GREATER_EQUAL						4
#define MATH_SMALLER_EQUAL						5
#define MATH_NOT_EQUAL								6
#define NUM_MATH_OPERATORS						6

/* Command Snippets */
#define MAX_SNIPPETS									5					// Max number of accepted Snippets
#define SNIPPET_CONDITION							1					// Snippet state machine codes
#define SNIPPET_COMMANDS							2					
#define SNIPPET_ACTIVATE							3					
#define SNIP_COND_BUTTON_EVENT				1					// Snippet command types
#define SNIP_COND_MODULE_EVENT				2
#define SNIP_COND_MODULE_PARAM_CONST	3
#define SNIP_COND_MODULE_PARAM_PARAM	4


/* BOS Parameters and constants */ 
#define	NUM_OF_MODULE_PN							21
#define P_LAST 												NumOfPorts
#define MAX_MESSAGE_SIZE							56
#define MAX_PARAMS_PER_MESSAGE				(MAX_MESSAGE_SIZE-10)		// H + Z + length + Dst + Src + 1 x Options + 2 x Code + CRC + 1 x reserved = 10
#define cmdMAX_INPUT_SIZE							50
#define	MaxNumOfModules								25
#define	MaxNumOfGroups								10
#define MaxNumOfPorts									10
#define MaxLengthOfAlias							9
#define MAX_BOS_VARS									30
#define NumOfKeywords									4
#define NumOfParamsHelpStrings				7
#define DEF_BUTTON_DEBOUNCE						30				// Button debounce time in ms
#define DEF_BUTTON_CLICK							50				// Button single click minimum time in ms
#define DEF_BUTTON_MIN_INTER_CLICK		5					// Button min inter-click time (in ms) for double clicks (uint8_t size)
#define DEF_BUTTON_MAX_INTER_CLICK		250				// Button max inter-click time (in ms) for double clicks (uint8_t size)
#define DEF_ARRAY_BAUDRATE						921600
#define DEF_CLI_BAUDRATE							921600
#define CLI_BAUDRATE_1								115200
//#define MSG_RX_BUF_SIZE								(250)			// 2 Mbps UART at 1 KHz parsing rate
#define MSG_RX_BUF_SIZE								(64)			// 1 Mbps UART at 0.5 KHz parsing rate
#define MSG_TX_BUF_SIZE								(250)			// 2 Mbps UART at 1 KHz parsing rate



/* Delay macros */
#define	Delay_us(t)							StartMicroDelay(t)		/* RTOS safe blocking delay (16 bits) - Use before and after starting the scheduler */
#define	Delay_ms_no_rtos(t)			StartMilliDelay(t)		/* RTOS safe blocking delay (16 bits) - Use before and after starting the scheduler */
#define	Delay_ms(t)							HAL_Delay(t)					/* Non-RTOS safe (32 bits) - Use only after starting the scheduler */
#define	Delay_s(t)							HAL_Delay(1000*t)			/* Non-RTOS safe (32 bits) - Use only after starting the scheduler */

/* Misc macros */
#define	InGroup(module, group)	( (groupModules[module-1] >> group) & 0x0001 )

/* Serial Wire Interface */
#define SWDIO_PIN			GPIO_PIN_13
#define	SWDIO_PORT		GPIOA
#define	SWCLK_PIN			GPIO_PIN_14
#define	SWCLK_PORT		GPIOA

/* MCU UUID */
#define MCU_F0_UUID_BASE					0x1FFFF7AC
#define MCU_F0_FLASH_SIZE_BASE		0x1FFFF7CC


/* Interrupt Priorities - 0 (highest) to 3 in F0 MCUs */
#define	MSG_DMA_INT_PRIORITY			0
#define	STREAM_DMA_INT_PRIORITY		1



/* Includes ------------------------------------------------------------------*/
										 
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
#include "BOS_utils.h"

/* C STD Library */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>	 
#include <limits.h>	

/* Module includes and initialization */
#if defined(H01R0) || defined(P01R0)
	#include "H01R0.h"
#endif
#if defined(H23R1) || defined(H23R0)
	#include "H23Rx.h"	
#endif
#ifdef H07R3
	#include "H07R3.h"	
#endif
#if defined(H08R6) || defined(P08R6)
	#include "H08R6.h"	
#endif
#ifdef H1BR6
	#include "H1BR6.h"	
#endif
#ifdef H12R0
	#include "H12R0.h"	
#endif
#ifdef H13R7
	#include "H13R7.h"
#endif
#if defined(H0FR1) || defined(H0FR6)
	#include "H0FR6.h"		
#endif
#ifdef H1AR2
	#include "H1AR0.h"	
#endif
#ifdef H09R0
	#include "H09R0.h"	
#endif
#ifdef H0AR9
	#include "H0AR9.h"	
#endif
#ifdef H0BR4
	#include "H0BR4.h"	
#endif
#ifdef H18R0
	#include "H18R0.h"	
#endif
#ifdef H1DR1
	#include "H1DR1.h"	
#endif
#ifdef H1DR5
	#include "H1DR5.h"	
#endif
#ifdef H26R0
	#include "H26R0.h"	
#endif

/* More BOS header files - must be defined after module headers */
#include "BOS_DMA.h"


/* External variables ---------------------------------------------------------*/
extern char cRxedChar;
extern uint8_t myID, bcastID;
extern uint16_t myPN;
extern uint8_t indMode;
extern uint8_t N;
extern const char modulePNstring[NUM_OF_MODULE_PN][6];
extern uint8_t portStatus[NumOfPorts+1];
extern uint16_t neighbors[NumOfPorts][2];
extern uint8_t messageParams[MAX_PARAMS_PER_MESSAGE];
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE];
extern uint8_t messageLength[NumOfPorts];
extern SemaphoreHandle_t PxRxSemaphoreHandle[7];
extern SemaphoreHandle_t PxTxSemaphoreHandle[7];
static char pcUserMessage[80];
extern const char * pcParamsHelpString[];
extern BOS_Status responseStatus;
extern char groupAlias[MaxNumOfGroups][MaxLengthOfAlias+1];
#ifndef __N
	extern char moduleAlias[MaxNumOfModules+1][MaxLengthOfAlias+1];
	extern uint8_t broadcastResponse[MaxNumOfModules];
	extern uint16_t groupModules[MaxNumOfModules];
#else
	extern char moduleAlias[__N+1][MaxLengthOfAlias+1];
	extern uint8_t broadcastResponse[__N];
	extern uint16_t groupModules[__N];
#endif
extern uint8_t routeDist[]; 
extern uint8_t routePrev[]; 
extern uint8_t route[];
extern button_t button[NumOfPorts+1];
extern bool delayButtonStateReset, needToDelayButtonStateReset;
extern BOS_t BOS;
extern uint8_t PcPort, bootStatus;
extern uint8_t BOS_initialized;
extern uint32_t BOS_var_reg[MAX_BOS_VARS];
extern snippet_t snippets[MAX_SNIPPETS];
extern uint8_t numOfBosCommands;
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];

/* Exported internal functions ---------------------------------------------------------*/

extern void StringToLowerCase(char *string);
extern BOS_Status UpdateBaudrate(uint8_t port, uint32_t baudrate);
extern BOS_Status BroadcastMessage(uint8_t src, uint8_t dstGroup, uint16_t code, uint16_t numberOfParams);
extern void SystemClock_Config(void);
extern void MX_FREERTOS_Init(void);
extern void SystemClock_Config(void);



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
extern BOS_Status SendMessageToGroup(char* group, uint16_t code, uint16_t numberOfParams);
extern BOS_Status SendMessageFromPort(uint8_t port, uint8_t src, uint8_t dst, uint16_t code, uint16_t numberOfParams);
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
extern int16_t GetID(char* string);
extern BOS_Status NameModule(uint8_t module, char* alias);
extern BOS_Status AddModuleToGroup(uint8_t module, char* group);
extern BOS_Status ReadPortsDir(void);
extern BOS_Status UpdateMyPortsDir(void);
extern BOS_Status StartScastDMAStream(uint8_t srcP, uint8_t srcM, uint8_t dstP, uint8_t dstM, uint8_t direction, uint32_t count, uint32_t timeout, bool stored);
extern BOS_Status AddPortButton(uint8_t buttonType, uint8_t port);
extern BOS_Status RemovePortButton(uint8_t port);
extern BOS_Status SetButtonEvents(uint8_t port, uint8_t clicked, uint8_t dbl_clicked, uint8_t pressed_x1sec, uint8_t pressed_x2sec, uint8_t pressed_x3sec,\
													uint8_t released_y1sec, uint8_t released_y2sec, uint8_t released_y3sec, uint8_t mode);
extern uint32_t *ReadRemoteVar(uint8_t module, uint32_t remoteAddress, varFormat_t *remoteFormat, uint32_t timeout);
extern uint32_t *ReadRemoteMemory(uint8_t module, uint32_t remoteAddress, varFormat_t requestedFormat, uint32_t timeout);
extern uint32_t *ReadRemoteParam(uint8_t module, char* paramString, varFormat_t *remoteFormat, uint32_t timeout);
extern BOS_Status WriteRemote(uint8_t module, uint32_t localAddress, uint32_t remoteAddress, varFormat_t format, uint32_t timeout);
extern BOS_Status WriteRemoteForce(uint8_t module, uint32_t localAddress, uint32_t remoteAddress, varFormat_t format, uint32_t timeout);
extern uint8_t AddBOSvar(varFormat_t format, uint32_t address);
extern BOS_Status BOS_CalendarConfig(uint8_t month, uint8_t day, uint16_t year, uint8_t weekday, uint8_t seconds, \
															uint8_t minutes, uint8_t hours, uint8_t AMPM, int8_t daylightsaving);
extern void GetTimeDate(void);
extern char *GetDateString(void);
extern char *GetTimeString(void);
extern BOS_Status Bridge(uint8_t port1, uint8_t port2);
extern BOS_Status Unbridge(uint8_t port1, uint8_t port2);
extern BOS_Status printfp(uint8_t port, char* str);

#endif /* BOS_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
