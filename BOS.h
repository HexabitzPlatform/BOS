/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS.h
 Description   : Header file for Bitz Operating System (BOS).

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_H
#define BOS_H

/* Includes ------------------------------------------------------------------*/

#include "BOS_MsgCodes.h" 
#include <stdbool.h>

/* STM HAL */
#if defined(STM32G0B1xx)
#include "stm32g0xx_hal.h"
#elif defined(H41R6)
#include "stm32f4xx_hal.h"
#endif

/* Firmware */
#define	_firmMajor			0
#define	_firmMinor			2
#define	_firmPatch			6
#define _firmDate			__DATE__
#define _firmTime			__TIME__


/* *************************************************************************/
/* Enumerations Definitions ************************************************/
/* *************************************************************************/
/* Available ports on the module */
enum PortNames_e {
	PC,     /* Port for the controller (PC) */
	P1, P2, P3, P4, P5, P6, P7, P8, P9, P10,  /* General ports */
	PUSB,   /* USB port */
	P_RS485 /* RS485 communication port */
};

/* Button names on the module */
enum ButtonNames_e {
	B1 =1, B2, B3, B4, B5, B6, B7, B8, B9, B10
};

/* Status of a port */
enum PortStatus_e {
	FREE,       /* Port is available */
	MSG,        /* Port is used for messaging */
	STREAM,     /* Port is used for streaming data */
	CLI,        /* Port is in Command Line Interface mode */
	PORTBUTTON, /* Port is used as a button input */
	OVERRUN,    /* Port has encountered data overrun */
	CUSTOM,     /* Port is in a custom mode */
	H_Status,   /* 'H’=72 is message start delimiters */
	Z_Status    /* ‘Z’=90 is message start delimiters */
};

/* UART data direction */
enum UartDirection_e {
	NORMAL,   /* Normal UART transmission direction */
	REVERSED  /* Reversed UART transmission direction */
};

/* Different module part numbers */
enum ModulePartNumbers_e {
	_H01R0 =1, _P01R0, _H23R0, _H23R1, _H23R3, _H07R3, _H08R6,
	_P08R6, _H09R0,_H09R9, _H1BR6, _H12R0, _H13R7, _H0FR1,
	_H0FR6, _H0FR7, _H1AR2, _H0AR9, _H1DR1, _H1DR5, _H0BR4,
	_H18R0, _H26R0, _H15R0, _H10R4, _H2AR3,_H41R6,_H3BR6,
	_H18R1,_H1FR5,_H3BR2,_H21R2,_H17R1,_H15R8,_H2BR0,_H05R0,
	_H3BR7,_H2BR1,_H07R8,_H08R7,_H16R6,_P08R7,_H19R0
};

/* LED indicator modes */
enum IndMode_e {
	IND_OFF,        /* Indicator off */
	IND_PING,       /* Indicator blinks when pinged */
	IND_TOPOLOGY,   /* Indicator used for topology identification */
	IND_SHORT_BLINK /* Short blink mode */
};

/* DMA stream direction */
enum DMAStreamDirection_e {
	FORWARD,      /* Data moves forward */
	BACKWARD,     /* Data moves backward */
	BIDIRECTIONAL /* Data moves in both directions */
};

/* Button types */
enum ButtonType_e {
	NONE = 0,      /* No button */
	MOMENTARY_NO,  /* Momentary button, normally open */
	MOMENTARY_NC,  /* Momentary button, normally closed */
	ONOFF_NO,      /* On/Off button, normally open */
	ONOFF_NC       /* On/Off button, normally closed */
};

/* Button states */
enum ButtonState_e {
	OFF =1, ON, OPEN, CLOSED, CLICKED, DBL_CLICKED, PRESSED, RELEASED,
	PRESSED_FOR_X1_SEC, PRESSED_FOR_X2_SEC, PRESSED_FOR_X3_SEC,
	RELEASED_FOR_Y1_SEC, RELEASED_FOR_Y2_SEC, RELEASED_FOR_Y3_SEC
};

/* Boot statuses */
enum BootStatus_e {
	POWER_ON_BOOT, /* Booting from power-on */
	RESET_BOOT     /* Booting from reset */
};

/* Basic colors */
enum BasicColors {
	BLACK =1, WHITE, RED, BLUE, YELLOW, CYAN, MAGENTA, GREEN,AQUA,PURPLE,LIGHTBLUE,ORANGE,INDIGO,
};

/* RGB LED operating modes */
enum RGBLedMode {
	RGB_PULSE_RGB = 1,    /* Pulsing RGB colors */
	RGB_PULSE_COLOR,      /* Pulsing a single color */
	RGB_SWEEP_BASIC,      /* Sweeping through basic colors */
	RGB_SWEEP_FINE,       /* Smooth color sweeping */
	RGB_DIM_UP,           /* Gradually increasing brightness */
	RGB_DIM_UP_WAIT,      /* Gradually increasing brightness with wait time */
	RGB_DIM_DOWN,         /* Gradually decreasing brightness */
	RGB_DIM_DOWN_WAIT,    /* Gradually decreasing brightness with wait time */
	RGB_DIM_UP_DOWN,      /* Brightness up then down */
	RGB_DIM_DOWN_UP,      /* Brightness down then up */
	RGB_DIM_UP_DOWN_WAIT, /* Brightness up-down with wait */
	RGB_DIM_DOWN_UP_WAIT  /* Brightness down-up with wait */
};

/* RTC time periods (AM/PM) */
enum TimePeriod_e {
	RTC_AM = 1, /* AM (Before Noon) */
	RTC_PM      /* PM (After Noon) */
};

/* Daylight saving adjustments */
enum Daylight_e {
	DAYLIGHT_SUB1H = -1, /* Subtract 1 hour for daylight saving */
	DAYLIGHT_NONE = 0,   /* No daylight saving adjustment */
	DAYLIGHT_ADD1H = 1   /* Add 1 hour for daylight saving */
};

/* Months of the year */
enum Months_e {
	JANUARY =1, FEBRUARY, MARCH, APRIL, MAY, JUNE, JULY, AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER
};

/* Days of the week */
enum Weekdays_e {
	MONDAY =1, TUESDAY, WEDNESDAY, THURSDAY, FRIDAY, SATURDAY, SUNDAY
};

/***************************************************************************/
/* Typedef Definitions *****************************************************/
/***************************************************************************/

/* Typedef Enumeration Definitions *****************************************/
/* Variable data formats */
typedef enum {
	FMT_UINT8 = 1,  /* Unsigned 8-bit integer */
	FMT_INT8,       /* Signed 8-bit integer */
	FMT_UINT16,     /* Unsigned 16-bit integer */
	FMT_INT16,      /* Signed 16-bit integer */
	FMT_UINT32,     /* Unsigned 32-bit integer */
	FMT_INT32,      /* Signed 32-bit integer */
	FMT_FLOAT,      /* Floating-point number */
	FMT_BOOL        /* Boolean value (true/false) */
} varFormat_t;

//typedef enum {
//	TRACE_NONE =0, TRACE_MESSAGE, TRACE_RESPONSE, TRACE_BOTH
//} traceOptions_t;

/* Number of attempts Type Definition */
//typedef enum {
//	once =1, twice=2, three_time=3, forever =0x8000
//} trial_t;

/* BOS system status and error codes */
typedef enum {
	BOS_OK = 0,                          /* Operation successful */
	BOS_ERR_UnknownMessage = 1,          /* Unknown message received */
	BOS_ERR_NoResponse = 2,              /* No response from module */
	BOS_ERR_MSG_Reflection = 3,          /* Message reflection detected */
	BOS_ERR_UnIDedModule = 5,            /* Unidentified module */
	BOS_ERR_Keyword = 6,                 /* Invalid keyword */
	BOS_ERR_ExistingAlias = 7,           /* Alias already exists */
	BOS_ERR_ExistingCmd = 8,             /* Command already exists */
	BOS_ERR_EEPROM = 10,                 /* EEPROM error */
	BOS_ERR_BUTTON_NOT_DEFINED = 11,     /* Button not defined */
	BOS_ERR_BUTTON_PRESS_EVENT_FULL = 12,/* Button press event memory full */
	BOS_ERR_BUTTON_RELEASE_EVENT_FULL = 13, /* Button release event memory full */
	BOS_ERR_SNIP_MEM_FULL = 14,          /* Snippet memory full */
	BOS_ERR_REMOTE_READ_TIMEOUT = 15,    /* Timeout during remote read */
	BOS_ERR_REMOTE_READ_NO_VAR = 16,     /* No variable found during remote read */
	BOS_ERR_REMOTE_WRITE_TIMEOUT = 17,   /* Timeout during remote write */
	BOS_ERR_REMOTE_WRITE_MEM_FULL = 18,  /* Remote write memory full */
	BOS_ERR_REMOTE_WRITE_INDEX = 19,     /* Invalid remote write index */
	BOS_ERR_LOCAL_FORMAT_UPDATED = 20,   /* Local format updated */
	BOS_ERR_REMOTE_WRITE_ADDRESS = 21,   /* Invalid remote write address */
	BOS_ERR_REMOTE_WRITE_FLASH = 22,     /* Remote write flash error */
	BOS_ERR_PORT_BUSY = 23,              /* Communication port busy */
	BOS_ERR_TIMEOUT = 24,                /* Operation timeout */
	BOS_ERR_WrongName = 100,             /* Incorrect name */
	BOS_ERR_WrongGroup = 101,            /* Incorrect group */
	BOS_ERR_WrongID = 102,               /* Incorrect ID */
	BOS_ERR_WrongParam = 103,            /* Incorrect parameter */
	BOS_ERR_WrongValue = 104,            /* Incorrect value */
	BOS_ERR_MSG_DOES_NOT_FIT = 105,      /* Message does not fit */
	BOS_MEM_ERASED = 250,                /* Memory erased */
	BOS_MEM_FULL = 251,                  /* Memory full */
	BOS_MULTICAST = 254,                 /* Multicast message */
	BOS_BROADCAST = 255,                 /* Broadcast message */
	BOS_ERROR = 255                      /* Generic error */
} BOS_Status;

/* Wake-up pins from standby mode */
typedef enum {
	PA0_PIN = 0, /* Pin PA0 */
	PA2_PIN,     /* Pin PA2 */
	PB5_PIN,     /* Pin PB5 */
	PC13_PIN,    /* Pin PC13 */
	NRST_PIN     /* Reset pin */
} WakeupPins_t;

/* Typedef Structure Definitions *******************************************/
/* Button configuration settings */
typedef struct {
	uint16_t debounce;         /* Debounce time in milliseconds */
	uint16_t singleClickTime;  /* Maximum time for a single click */
	uint8_t minInterClickTime; /* Minimum time between consecutive clicks */
	uint8_t maxInterClickTime; /* Maximum time between consecutive clicks */
} buttonsConfig_t;

/* Button properties */
typedef struct {
	uint8_t state;         /* Current button state */
	uint8_t type;          /* Type of button */
	uint8_t pressedX1Sec;  /* Button pressed for X1 seconds */
	uint8_t pressedX2Sec;  /* Button pressed for X2 seconds */
	uint8_t pressedX3Sec;  /* Button pressed for X3 seconds */
	uint8_t releasedY1Sec; /* Button released for Y1 seconds */
	uint8_t releasedY2Sec; /* Button released for Y2 seconds */
	uint8_t releasedY3Sec; /* Button released for Y3 seconds */
	uint8_t events;        /* Event status */
} button_t;

/* Time representation */
typedef struct {
	uint16_t msec;   /* Milliseconds */
	uint8_t seconds; /* Seconds */
	uint8_t minutes; /* Minutes */
	uint8_t hours;   /* Hours */
	uint8_t ampm;    /* AM/PM indicator */
} BOS_time_t;

/* Date representation */
typedef struct {
	uint8_t weekday; /* Day of the week (Monday = 1, Sunday = 7) */
	uint8_t day;     /* Day of the month */
	uint8_t month;   /* Month (1 = January, 12 = December) */
	uint16_t year;   /* Year */
} BOS_date_t;

/* BOS system configuration structure */
typedef struct {
	buttonsConfig_t buttons; /* Button configuration */
	uint32_t clibaudrate;    /* CLI baud rate */
	uint8_t daylightsaving;  /* Daylight saving mode */
	uint8_t hourformat;      /* Hour format (12h/24h) */
	BOS_time_t time;         /* Current system time (not saved) */
	BOS_date_t date;         /* Current system date (not saved) */
	uint8_t disableCLI;      /* Disable command-line interface */
} BOS_t;

/* BOS Struct Type Definition */
//typedef struct {
//	uint8_t response;
//	bool trace;
//	uint8_t overrun;
//	bool received_Acknowledgment;
//	bool Acknowledgment;
//	trial_t trial;
//} BOSMessaging_t;

/* BOS message option byte structure */
typedef struct {
	uint8_t ExtendedOptions     : 1; /* If set, additional option byte follows */
	uint8_t ExtendedMessageCode : 1; /* If set, message codes are 16-bit */
	uint8_t Trace               : 1; /* If set, message trace (ping) is enabled */
	uint8_t Acknowledgment      : 1; /* Message acknowledgment flag */
	uint8_t Reserved            : 1; /* Reserved for future use */
	uint8_t Response            : 2; /* Response type */
	uint8_t LongMessage         : 1; /* If set, message continues in next packet */
} BOSOptionByte_t;

/* Module parameters */
typedef struct {
	void *paramPtr;          /* Pointer to parameter data */
	varFormat_t paramFormat; /* Format of the parameter */
	char *paramName;         /* Name of the parameter */
} module_param_t;

/* Snippet conditionals */
typedef struct {
	uint8_t conditionType; /* Type of condition */
	uint8_t mathOperator;  /* Mathematical operator */
	uint8_t buffer1[4];    /* First condition buffer */
	uint8_t buffer2[4];    /* Second condition buffer */
} snippetConditions_t;

/* Snippet properties */
typedef struct {
	snippetConditions_t cond; /* Snippet conditionals */
	char *cmd;     /* Command string */
	uint8_t state; /* Snippet state */
} snippet_t;

/* Receiving default values for H1DR5 module */
typedef struct {
	uint8_t Local_mac_addr[6];  /* Local MAC address */
	uint8_t Remote_mac_addr[6]; /* Remote MAC address */
	uint8_t Local_IP[4];        /* Local IP address */
	uint8_t Remote_IP[4];       /* Remote IP address */
	uint8_t ip_mask[4];         /* Subnet mask */
	uint8_t ip_dest[4];         /* Destination IP */
	uint8_t Local_PORT;         /* Local port number */
	uint8_t Remote_PORT;        /* Remote port number */
} receive_defalt_value;

/* Remote data buffer used to store data from message codes */
typedef struct {
	bool     Databool;     /* Boolean data */
	int8_t   Data8;        /* Signed 8-bit integer */
	uint8_t  DataU8[3];    /* Unsigned 8-bit array */
	int16_t  Data16;       /* Signed 16-bit integer */
	uint16_t DataU16[3];   /* Unsigned 16-bit array */
	int32_t  Data32;       /* Signed 32-bit integer */
	uint32_t DataU32;      /* Unsigned 32-bit integer */
	float    DataFloat[4]; /* Floating point array */
} RemoteDataBuffer_t;

/***************************************************************************/
/* Macro Definitions *******************************************************/
/***************************************************************************/

/* Button Events Definition */
#define BUTTON_EVENT_CLICKED                 0x01
#define BUTTON_EVENT_DBL_CLICKED             0x02
#define BUTTON_EVENT_PRESSED_FOR_X1_SEC      0x04
#define BUTTON_EVENT_PRESSED_FOR_X2_SEC      0x08
#define BUTTON_EVENT_PRESSED_FOR_X3_SEC      0x10
#define BUTTON_EVENT_RELEASED_FOR_Y1_SEC     0x20
#define BUTTON_EVENT_RELEASED_FOR_Y2_SEC     0x40
#define BUTTON_EVENT_RELEASED_FOR_Y3_SEC     0x80
#define BUTTON_EVENT_MODE_CLEAR              0
#define BUTTON_EVENT_MODE_OR                 1

/* BOS Response Definitions */
#define BOS_RESPONSE_ALL                     0x03 /* Send response messages for both Messaging and CLI */
#define BOS_RESPONSE_MSG                     0x01 /* Send response messages for Messaging only */
#define BOS_RESPONSE_CLI                     0x02 /* Send response messages for CLI only */
#define BOS_RESPONSE_NONE                    0x00 /* Do not send any response messages */

/* Remote Memory Types */
#define REMOTE_MEMORY_ADD                    0
#define REMOTE_BOS_PARAM                     1
#define REMOTE_MODULE_PARAM                  2
#define REMOTE_BOS_VAR                       3

/* Math Operators */
#define MATH_EQUAL                           1
#define MATH_GREATER                         2
#define MATH_SMALLER                         3
#define MATH_GREATER_EQUAL                   4
#define MATH_SMALLER_EQUAL                   5
#define MATH_NOT_EQUAL                       6
#define NUM_MATH_OPERATORS                   6

/* Command Snippets */
#define MAX_SNIPPETS                         5 /* Max number of accepted Snippets */
#define SNIPPET_CONDITION                    1 /* Snippet state machine codes */
#define SNIPPET_COMMANDS                     2
#define SNIPPET_ACTIVATE                     3

/* Snippet Command Types */
#define SNIP_COND_BUTTON_EVENT               1
#define SNIP_COND_MODULE_EVENT               2
#define SNIP_COND_MODULE_PARAM_CONST         3
#define SNIP_COND_MODULE_PARAM_PARAM         4

/* BOS Parameters and Constants */
#define NUM_OF_MODULE_PN                     46 /* Number of Modules */
#define P_LAST                               NumOfPorts
#define MAX_MESSAGE_SIZE                     56 /* Max Number of Bytes in One Message */
#define MAX_PARAMS_PER_MESSAGE               (MAX_MESSAGE_SIZE - 10) /* Calculated max params per message */
#define cmdMAX_INPUT_SIZE                    50
#define MaxNumOfModules                      26 /* Max Number of Modules in one Array */
#define MaxNumOfGroups                       10
#define MaxNumOfPorts                        10 /* Max number of ports in one module */
#define MaxLengthOfAlias                     9
#define MAX_BOS_VARS                         30 /* Max number of BOS Variables */
#define NumOfKeywords                        4
#define NumOfParamsHelpStrings               7

/* Default Button Timings */
#define DEF_BUTTON_DEBOUNCE                  30 /* Button debounce time in ms */
#define DEF_BUTTON_CLICK                     50 /* Button single click minimum time in ms */
#define DEF_BUTTON_MIN_INTER_CLICK           5  /* Min inter-click time (ms) for double clicks */
#define DEF_BUTTON_MAX_INTER_CLICK           250 /* Max inter-click time (ms) for double clicks */

/* Default Baud Rates */
#define DEF_ARRAY_BAUDRATE                   921600 /* Default baud rate for all modules */
#define DEF_CLI_BAUDRATE                     921600 /* Default baud rate for CLI */
#define CLI_BAUDRATE_1                       115200

/* Message Buffer Sizes */
#define MSG_RX_BUF_SIZE                      192 /* 1 Mbps UART at 0.5 KHz parsing rate */
#define MSG_TX_BUF_SIZE                      250 /* 2 Mbps UART at 1 KHz parsing rate */

/* Delay Macros */
#define Delay_us(t)                          StartMicroDelay(t) /* RTOS-safe microsecond delay */
#define Delay_ms_no_rtos(t)                  StartMilliDelay(t) /* RTOS-safe millisecond delay */
#define Delay_ms(t)                          HAL_Delay(t)       /* Non-RTOS safe millisecond delay */
#define Delay_s(t)                           HAL_Delay(1000 * t) /* Non-RTOS safe second delay */

/* Miscellaneous Macros */
#define InGroup(module, group)               ((groupModules[module - 1] >> group) & 0x0001)

/* Serial Wire Interface (SWI) */
#define SWDIO_PIN                            GPIO_PIN_13
#define SWDIO_PORT                           GPIOA
#define SWCLK_PIN                            GPIO_PIN_14
#define SWCLK_PORT                           GPIOA

/* MCU Unique Identifiers */
#define MCU_F0_UUID_BASE                     0x1FFFF7AC
#define MCU_F0_FLASH_SIZE_BASE               0x1FFFF7CC

/* Interrupt Priorities */
#define MSG_DMA_INT_PRIORITY                 0 /* Highest priority */
#define STREAM_DMA_INT_PRIORITY              1

/* Message properties */
#define MSG_COUNT 		5 /* TODO: messages count should be increased, but there's no enough memory now */
#define MSG_MAX_SIZE 	56

/***************************************************************************/
/* Includes ****************************************************************/
/***************************************************************************/

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
#include "BOS_utils.h"
#include "BOS_messaging.h"

/* Emulated EEPROM from ST */
#include "eeprom_emul.h"
#include "flash_interface.h"

/* C STD Library */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>	 
#include <limits.h>	

/* Module includes and initialization */
#if defined(H01R0)
#include "H01R0.h"
#endif
#if defined(H23R1) || defined(H23R0)
	#include "H23Rx.h"	
#endif
#if defined(H23R3)
	#include "H23R3.h"
#endif
#ifdef H07R3
	#include "H07R3.h"	
#endif
#if defined(H08R6)
	#include "H08R6.h"	
#endif
#ifdef H09R9
    #include "H09R9.h"
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
#ifdef H0FR1
	#include "H0FR1.h"
#endif
#ifdef H0FR6
	#include "H0FR6.h"
#endif
#ifdef H0FR7
	#include "H0FR7.h"
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
#ifdef H15R0
	#include "H15R0.h"	
#endif
#ifdef H10R4
    #include "H10R4.h"
#endif
#ifdef H2AR3
    #include "H2AR3.h"
#endif
#ifdef H41R6
    #include "H41R6.h"
#endif
#ifdef H3BR6
    #include "H3BR6.h"
#endif
#ifdef H3BR7
    #include "H3BR7.h"
#endif
#ifdef H18R1
    #include "H18R1.h"
#endif
#ifdef H1FR5
    #include "H1FR5.h"
#endif
#ifdef H3BR2
    #include "H3BR2.h"
#endif
#ifdef H21R2
    #include "H21R2.h"
#endif
#ifdef H17R1
    #include "H17R1.h"
#endif
#ifdef H15R8
    #include "H15R8.h"
#endif
#ifdef H2BR0
    #include "H2BR0.h"
#endif
#ifdef H2BR1
    #include "H2BR1.h"
#endif
#ifdef H05R0
    #include "H05R0.h"
#endif
#ifdef H07R8
    #include "H07R8.h"
#endif
#ifdef H08R7
    #include "H08R7.h"
#endif
#ifdef H16R6
    #include "H16R6.h"
#endif
#ifdef P08R7
    #include "P08R7.h"
#endif
#ifdef P01R0
    #include "P01R0.h"
#endif
#ifdef H19R0
    #include "H19R0.h"
#endif
/* More BOS header files - must be defined after module headers */
#include "BOS_DMA.h"

/***************************************************************************/
/* External variables ******************************************************/
/***************************************************************************/

extern bool delayButtonStateReset;
extern bool needToDelayButtonStateReset;
extern bool ACK_FLAG;
extern bool rejected_FLAG;

extern char *pcBootloaderUpdateMessage;
extern char *pcRemoteBootloaderUpdateMessage;
extern char *pcRemoteBootloaderUpdateViaPortMessage;
extern char *pcRemoteBootloaderUpdateWarningMessage;
extern const char *pcParamsHelpString[];
extern const char modulePNstring[NUM_OF_MODULE_PN][6];
extern char groupAlias[MaxNumOfGroups][MaxLengthOfAlias + 1];
extern char cRxedChar;
static char pcUserMessage[80];

extern uint8_t myID;
extern uint8_t bcastID;
extern uint8_t indMode;
extern uint8_t N;
extern uint8_t numOfBosCommands;
extern uint8_t CLI_Data ;
extern uint8_t port_DMA;
extern uint8_t PcPort;
extern uint8_t bootStatus;
extern uint8_t BOS_initialized;
extern uint8_t routeDist[];
extern uint8_t routePrev[];
extern uint8_t route[];
extern uint8_t messageParams[MAX_PARAMS_PER_MESSAGE];
extern uint8_t messageLength[NumOfPorts];
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE];
extern uint8_t portStatus[NumOfPorts + 1];

/* Flags for CLI Task */
extern uint8_t Activate_CLI_For_First_Time_Flag;
extern uint8_t Read_In_CLI_Task_Flag;

/* Messages circular buffer variables */
extern uint8_t MSG_Buffer_Index_Start[NumOfPorts];
extern uint8_t MSG_Buffer_Index_End[NumOfPorts];
extern uint8_t MSG_Buffer[NumOfPorts][MSG_COUNT][MSG_MAX_SIZE];
extern uint8_t Process_Message_Buffer[MSG_COUNT];
extern uint8_t Process_Message_Buffer_Index_Start;
extern uint8_t Process_Message_Buffer_Index_End;
extern uint8_t index_input[6] ;
extern uint8_t index_process[6] ;
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];

extern uint16_t myPN;
extern volatile uint16_t neighbors[NumOfPorts][2];

extern uint32_t BOS_var_reg[MAX_BOS_VARS];
extern volatile uint32_t MBmessageParams[9];
extern volatile uint32_t* index_dma[6];

extern snippet_t snippets[MAX_SNIPPETS];
extern button_t button[NumOfPorts + 1];
extern BOS_t BOS;
extern BOS_Status responseStatus;
extern BOSOptionByte_t OptionByte;
extern BOSOptionByte_t UserOptionByte;
extern RemoteDataBuffer_t RemoteDataBuffer;
extern module_param_t modParam[];
//extern BOSMessaging_t BOSMessaging;

#ifndef __N
extern uint16_t array[MaxNumOfModules][MaxNumOfPorts + 1]; /* Array topology */
extern uint8_t routeDist[MaxNumOfModules];
extern uint8_t routePrev[MaxNumOfModules];
extern char moduleAlias[MaxNumOfModules + 1][MaxLengthOfAlias + 1]; /* moduleAlias[0] used to store alias for module 0 */
extern uint8_t broadcastResponse[MaxNumOfModules];
extern uint16_t groupModules[MaxNumOfModules]; /* Group 0 (LSB) to Group 15 (MSB) */
#else
extern	uint8_t routeDist[__N];
extern	uint8_t routePrev[__N];
extern	char moduleAlias[__N+1][MaxLengthOfAlias+1];
extern	uint8_t broadcastResponse[__N];
extern	uint16_t groupModules[__N];									/* Group 0 (LSB) to Group 15 (MSB) */
#endif

/*Output_Port_Array[__N]:
This array stores all solutions (output ports) to send messages
between modules based on the topology file using FindRoute() function,
so we can read these output ports when needed instead of figuring out the correct port every time.
*/
#ifdef __N
extern uint8_t Output_Port_Array[__N];
#endif

/*..............User Data from external ports (like USB, Ethernet, BLE ...)..........*/
#ifdef __USER_DATA_BUFFER
#define USER_RX_BUF_SIZE  512
extern uint8_t UserBufferData[USER_RX_BUF_SIZE];
extern uint8_t UserData;
extern uint8_t indexInputUserDataBuffer;
extern uint8_t indexProcessUserDataBuffer;
extern volatile uint32_t* DMACountUserDataBuffer;
extern uint8_t GetUserDataCount(void);
extern BOS_Status GetUserDataByte(uint8_t* pData);
#endif

/* FreeRTOS semaphore handles */
extern SemaphoreHandle_t PxRxSemaphoreHandle[7];
extern SemaphoreHandle_t PxTxSemaphoreHandle[7];

 /*
  *New private function [inside SendMessageFromPort() ] for sending BOS Messages.
  *instead of writePxDMAMutex (the previous function)
  */

 extern HAL_StatusTypeDef Send_BOS_Message(uint8_t port, uint8_t* buffer, uint16_t n, uint32_t mutexTimeout,uint8_t dst);

 /***************************************************************************/
 /*************************** BOS General Functions *************************/
 /***************************************************************************/

 /* ======================= System Configuration API ======================= */
 extern void SystemClock_Config(void);

 /* ====================== FreeRTOS Initialization API ====================== */
 extern void MX_FREERTOS_Init(void);

 /* ========================== Indicator LED APIs =========================== */
#define IND_toggle()			HAL_GPIO_TogglePin(_IND_LED_PORT,_IND_LED_PIN)
#define IND_ON()				HAL_GPIO_WritePin(_IND_LED_PORT,_IND_LED_PIN,GPIO_PIN_SET)
#define IND_OFF()				HAL_GPIO_WritePin(_IND_LED_PORT,_IND_LED_PIN,GPIO_PIN_RESET)
#define IND_blink(t)			IND_ON();	HAL_Delay(t); IND_OFF()	/* Use after starting the scheduler */
#define RTOS_IND_blink(t)		IND_ON();	osDelay(t); IND_OFF()	/* Use after starting the scheduler */

 /* ============================== Delay APIs ============================== */
 extern void StartMicroDelay(uint16_t Delay);
 extern void StartMilliDelay(uint16_t Delay);

 /* ======================== BOS Initialization APIs ======================= */
 extern void BOS_Init(void);
 extern void Module_Init(void);

 /* ======================== BOS Port Handling APIs ======================== */
 extern UART_HandleTypeDef* GetUart(uint8_t port);
 extern uint8_t GetPort(UART_HandleTypeDef *huart);
 extern BOS_Status UpdateBaudrate(uint8_t port, uint32_t baudrate);
 extern void SwapUartPins(UART_HandleTypeDef *huart, uint8_t direction);
 extern BOS_Status ReadPortsDir(void);
 extern BOS_Status UpdateMyPortsDir(void);

 /* ================== Module Identification & Naming APIs ================= */
 extern int16_t GetID(char *string);
 extern BOS_Status NameModule(uint8_t module, char *alias);
 extern BOS_Status AddModuleToGroup(uint8_t module, char *group);

 /* =========================== Exploration APIs =========================== */
 extern BOS_Status Explore(void);
 extern BOS_Status ExploreNeighbors(uint8_t ignore);
 extern BOS_Status FindBroadcastRoutes(uint8_t src);
 extern uint8_t FindRoute(uint8_t sourceID, uint8_t desID);
 extern void DisplayTopology(uint8_t port);
 extern void DisplayPortsDir(uint8_t port);
 extern void DisplayModuleStatus(uint8_t port);
 #define NumberOfHops(i) routeDist[i-1]

 /* ============================ Messaging APIs ============================ */
 extern BOS_Status SendLargeMessageToModule(uint8_t dst, uint16_t code, uint8_t *pParameters, uint16_t numberOfParams);
 extern BOS_Status SendMessageToModule(uint8_t dst, uint16_t code, uint16_t numberOfParams);
 extern BOS_Status SendMessageToGroup(char *group, uint16_t code, uint16_t numberOfParams);
 extern BOS_Status SendMessageFromPort(uint8_t port, uint8_t src, uint8_t dst, uint16_t code, uint16_t numberOfParams);
 extern BOS_Status BroadcastMessage(uint8_t src, uint8_t dstGroup, uint16_t code, uint16_t numberOfParams);
 extern BOS_Status ReadDataFromSensorModule(uint8_t disModuleID, uint16_t Code, uint32_t *pDataReceived, uint16_t timeout);

 /* ========================= Data Streaming APIs ========================== */
 extern BOS_Status StartScastDMAStream(uint8_t srcP, uint8_t srcM, uint8_t dstP, uint8_t dstM, uint8_t direction, uint32_t count, uint32_t timeout, bool stored);

 /* ========================= Button Handling APIs ========================= */
 extern BOS_Status AddPortButton(uint8_t buttonType, uint8_t port);
 extern BOS_Status RemovePortButton(uint8_t port);
 extern BOS_Status SetButtonEvents(uint8_t port, uint8_t clicked, uint8_t dbl_clicked, uint8_t pressed_x1sec, uint8_t pressed_x2sec, uint8_t pressed_x3sec, uint8_t released_y1sec, uint8_t released_y2sec, uint8_t released_y3sec, uint8_t mode);

 /* ===================== Remote Variable Handling APIs ==================== */
 extern uint32_t* ReadRemoteVar(uint8_t module, uint32_t remoteAddress, varFormat_t *remoteFormat, uint32_t timeout);
 extern uint32_t* ReadRemoteMemory(uint8_t module, uint32_t remoteAddress, varFormat_t requestedFormat, uint32_t timeout);
 extern uint32_t* ReadRemoteParam(uint8_t module, char *paramString, varFormat_t *remoteFormat, uint32_t timeout);
 extern BOS_Status WriteRemote(uint8_t dstModuleID, uint32_t localVarAddress, uint32_t BOSVarAddress, varFormat_t format, uint32_t timeout);
 extern uint8_t AddBOSvar(varFormat_t format, uint32_t address);
 extern BOS_Status WriteToMBModule(uint8_t dst, uint8_t rank, float var1, float var2, float var3);
 extern BOS_Status ReadFromMBModule(uint8_t dst, uint8_t rank, uint32_t timeout);

 /* ====================== Date & Time Handling APIs ====================== */
 extern BOS_Status BOS_CalendarConfig(uint8_t month, uint8_t day, uint16_t year, uint8_t weekday, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t AMPM, int8_t daylightsaving);
 extern void GetTimeDate(void);
 extern char* GetDateString(void);
 extern char* GetTimeString(void);

 /* ========================= Port Bridging APIs ========================== */
 extern BOS_Status Bridge(uint8_t port1, uint8_t port2);
 extern BOS_Status Unbridge(uint8_t port1, uint8_t port2);

 /* ============================== CLI APIs =============================== */
 extern void vRegisterCLICommands(void);
 extern void StringToLowerCase(char *string);

 /* =========================== Bootloader APIs =========================== */
 extern void SetupPortForRemoteBootloaderUpdate(uint8_t port);

 /* ============================== Print APIs ============================= */
 extern BOS_Status printfp(uint8_t port, char *str);

 /* ======================== Power Management APIs ======================== */
 extern BOS_Status EnableStopModebyUARTx(uint8_t port);
 extern BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t WakeupPins);
 extern BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t WakeupPins);

#endif /* BOS_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
