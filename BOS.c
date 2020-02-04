/*
    BitzOS (BOS) V0.2.0 - Copyright (C) 2017-2019 Hexabitz
    All rights reserved

    File Name     : BOS.c
    Description   : Source code for Bitz Operating System (BOS).
		
		Required MCU resources : 
		
			>> Timer 14 for micro-sec delay.
			>> Timer 15 for milli-sec delay.

*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private and global variables ---------------------------------------------------------*/

BOS_t BOS; 
BOS_t BOS_default = { .clibaudrate = DEF_CLI_BAUDRATE, .response = BOS_RESPONSE_ALL, .trace = TRACE_BOTH, .buttons.debounce = DEF_BUTTON_DEBOUNCE, .buttons.singleClickTime = DEF_BUTTON_CLICK, 
											.buttons.minInterClickTime = DEF_BUTTON_MIN_INTER_CLICK, .buttons.maxInterClickTime = DEF_BUTTON_MAX_INTER_CLICK, .daylightsaving = DAYLIGHT_NONE, .hourformat = 24 };
uint16_t myPN = modulePN;
TIM_HandleTypeDef htim14;	/* micro-second delay counter */
TIM_HandleTypeDef htim15;	/* milli-second delay counter */
uint8_t indMode = IND_OFF;

/* Define module PN strings [available PNs+1][5 chars] */
const char modulePNstring[NUM_OF_MODULE_PN][6] = {"", "H01R0", "P01R0", "H23R0", "H23R1", "H07R3", "H08R6", "P08R6", "H09R0", "H1BR6", "H12R0", "H13R7", "H0FR1", "H0FR6", "H1AR2", "H0AR9", "H1DR1", "H1DR5", "H0BR4", "H18R0", "H26R0"};

/* Define BOS keywords */
static const char BOSkeywords[NumOfKeywords][4] = {"me", "all", "if", "for"};

static const char *monthStringAbreviated[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static const char *weekdayString[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
uint16_t Dolp,lock;
/* Number of modules in the array */
#ifndef _N
	uint8_t N = 1;
	uint8_t myID = 0;
#else
	uint8_t N = _N;
	uint8_t myID = _module;
#endif

/* Routing and topology */
bool Topology_ok;
uint16_t iteration=0,Nid,longMessagecount,numoflongmsg,totalofrcvmsg;
uint8_t mcount,rcount,Topology_count,Ping_count,Hi_count,in;
uint8_t portStatus[NumOfPorts+1] = {0};
uint16_t neighbors[NumOfPorts][2] = {0};
uint16_t neighbors2[NumOfPorts][2] = {0};
uint16_t bcastRoutes[MaxNumOfModules] = {0};				/* P1 is LSB */
bool AddBcastPayload = false;
uint8_t dstGroupID = BOS_BROADCAST;
char groupAlias[MaxNumOfGroups][MaxLengthOfAlias+1] = {0};
#ifndef _N
	uint16_t array[MaxNumOfModules][MaxNumOfPorts+1] = {{0}};			/* Array topology */
	uint16_t arrayPortsDir[MaxNumOfModules]= {0};									/* Array ports directions */
	uint8_t routeDist[MaxNumOfModules] = {0}; 
	uint8_t routePrev[MaxNumOfModules] = {0}; 
	uint8_t route[MaxNumOfModules] = {0};
	char moduleAlias[MaxNumOfModules+1][MaxLengthOfAlias+1] = {0};		/* moduleAlias[0] used to store alias for module 0 */
	uint8_t broadcastResponse[MaxNumOfModules] = {0};
	uint16_t groupModules[MaxNumOfModules] = {0};			/* Group 0 (LSB) to Group 15 (MSB) */
#else
	uint16_t arrayPortsDir[_N]= {0};
	uint8_t routeDist[_N] = {0}; 
	uint8_t routePrev[_N] = {0}; 
	uint8_t route[_N] = {0};
	char moduleAlias[_N+1][MaxLengthOfAlias+1] = {0};
	uint8_t broadcastResponse[_N] = {0};
	uint16_t groupModules[_N] = {0};									/* Group 0 (LSB) to Group 15 (MSB) */
#endif

/* Buffers and communication */
uint8_t num=0;	// COUNTER FOR SENDEING MESSAGE IN EXPLORE
uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE] = {0};		// Buffer for messages received and ready to be parsed 
char message[MAX_MESSAGE_SIZE] = {0};										// Buffer to construct a message to be sent
uint8_t messageLength[NumOfPorts] = {0};
uint8_t messageParams[MAX_PARAMS_PER_MESSAGE] = {0};
char cRxedChar = 0; 
uint8_t longMessage = 0; uint16_t longMessageLastPtr = 0;
static uint8_t longMessageScratchpad[(MaxNumOfPorts+1)*MaxNumOfModules] = {0};
static char pcUserMessage[80];
BOS_Status responseStatus = BOS_OK; 
uint8_t bcastID = 0;			// Counter for unique broadcast ID
uint8_t PcPort = 0;
uint8_t BOS_initialized = 0;
uint32_t BOS_var_reg[MAX_BOS_VARS];			// BOS variables register: Bits 31-16: variable RAM address shift from SRAM_BASE, Bits 15-8: status. Bits 7-0: format.
uint64_t remoteBuffer = 0;
varFormat_t remoteVarFormat = FMT_UINT8;
uint8_t CLI_LOW_Baudrate_Flag = 0; 			//Flage for Lower CLI baudrate is set

/* Buttons */
button_t button[NumOfPorts+1] = {0};
uint32_t pressCounter[NumOfPorts+1] = {0};
uint32_t releaseCounter[NumOfPorts+1] = {0};
uint8_t dblCounter[NumOfPorts+1] = {0};
bool needToDelayButtonStateReset = false, delayButtonStateReset = false;

/* Messaging tasks */
extern TaskHandle_t UserTaskHandle;
#ifdef _P1
extern TaskHandle_t P1MsgTaskHandle;
#endif
#ifdef _P2
extern TaskHandle_t P2MsgTaskHandle;
#endif
#ifdef _P3
extern TaskHandle_t P3MsgTaskHandle;
#endif
#ifdef _P4
extern TaskHandle_t P4MsgTaskHandle;
#endif
#ifdef _P5
extern TaskHandle_t P5MsgTaskHandle;
#endif
#ifdef _P6
extern TaskHandle_t P6MsgTaskHandle;
#endif

/* UARTcmd task */
extern TaskHandle_t xCommandConsoleTaskHandle;

/* Define CLI command list*/
typedef struct xCOMMAND_INPUT_LIST
{
	const CLI_Command_Definition_t *pxCommandLineDefinition;
	struct xCOMMAND_INPUT_LIST *pxNext;
} 
CLI_Definition_List_Item_t;
extern CLI_Definition_List_Item_t xRegisteredCommands;
uint8_t numOfBosCommands;

/* Variables exported internally */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
extern uint8_t crcBuffer[MAX_MESSAGE_SIZE];

/* RTC */
RTC_HandleTypeDef RtcHandle;
uint8_t bootStatus = POWER_ON_BOOT;

/* Private function prototypes -----------------------------------------------*/	

uint8_t minArr(uint8_t* arr, uint8_t* Q);
uint8_t QnotEmpty(uint8_t* Q);
void NotifyMessagingTask(uint8_t port);
//BOS_Status SaveEEtopology(void);								
//BOS_Status LoadEEtopology(void);
uint8_t SaveToRO(void);
#ifndef _N
uint8_t ClearROtopology(void);
#endif
uint8_t LoadROsnippets(void);
uint8_t LoadROtopology(void);
BOS_Status SaveEEportsDir(void);
BOS_Status ClearEEportsDir(void);
BOS_Status LoadEEportsDir(void);
BOS_Status SaveEEalias(void);
BOS_Status LoadEEalias(void);
BOS_Status SaveEEgroup(void);
BOS_Status LoadEEgroup(void);
BOS_Status LoadEEstreams(void);
BOS_Status SaveEEstreams(uint8_t direction, uint32_t count, uint32_t timeout, uint8_t src1, uint8_t dst1, uint8_t src2, \
	uint8_t dst2, uint8_t src3, uint8_t dst3);
BOS_Status LoadEEparams(void);
BOS_Status SaveEEparams(void);
BOS_Status LoadEEbuttons(void);
BOS_Status SetupDMAStreams(uint8_t direction, uint32_t count, uint32_t timeout, uint8_t src, uint8_t dst);
void StreamTimerCallback( TimerHandle_t xTimerStream );
uint8_t IsFactoryReset(void);
void EE_FormatForFactoryReset(void);
BOS_Status GetPortGPIOs(uint8_t port, uint32_t *TX_Port, uint16_t *TX_Pin, uint32_t *RX_Port, uint16_t *RX_Pin);
BOS_Status CheckForTimedButtonPress(uint8_t port);
BOS_Status CheckForTimedButtonRelease(uint8_t port);
void buttonPressedCallback(uint8_t port);
void buttonReleasedCallback(uint8_t port);
void buttonClickedCallback(uint8_t port);
void buttonDblClickedCallback(uint8_t port);
void buttonPressedForXCallback(uint8_t port, uint8_t eventType);
void buttonReleasedForYCallback(uint8_t port, uint8_t eventType);
BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
BOS_Status BroadcastReceivedMessage(uint8_t dstType, uint8_t IncomingPort);
BOS_Status WriteToRemote(uint8_t module, uint32_t localAddress, uint32_t remoteAddress, varFormat_t format, uint32_t timeout, uint8_t force);
BOS_Status RTC_Init(void);
BOS_Status RTC_CalendarConfig(void);
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);

/* Module exported internal functions */
extern uint8_t IsModuleParameter(char* name);
extern void Module_Init(void);
extern void RegisterModuleCLICommands(void);
extern Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift);

extern bool ParseSnippetCommand(char *snippetBuffer, int8_t *cliBuffer);

const char * pcParamsHelpString[NumOfParamsHelpStrings] = {"\r\nBOS.response: all, message, cli, none\r\n",
"\r\nBOS.trace: all, message, response, none\r\n",
"BOS.clibaudrate: CLI baudrate. Default is 921600. This affects all ports. If you change this value, \
you must connect to a CLI port on each startup to restore other array ports into default baudrate\r\n",
																															 "BOS.debounce: 1 ... 65536 msec\r\n",
																															 "BOS.singleclicktime: 1 ... 65536 msec\r\n",
																															 "BOS.mininterclicktime: 1 ... 255 msec\r\n",
																															 "BOS.maxinterclicktime: 1 ... 255 msec\r\n"};

/* Create CLI commands --------------------------------------------------------*/

static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE pingCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE bootloaderUpdateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
#ifndef _N
static portBASE_TYPE exploreCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
#endif
static portBASE_TYPE resetCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE nameCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE groupCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE statusCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE infoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE scastCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE addbuttonCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE removebuttonCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE setCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE getCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE defaultCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE timeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE dateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE setBaudrateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE uuidCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE idcodeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE flashsizeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE snipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE actSnipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE pauseSnipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE delSnipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE bridgeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE unbridgeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : run-time-stats 
This generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t prvRunTimeStatsCommandDefinition =
{
	( const int8_t * ) "run-time-stats", /* The command string to type. */
	( const int8_t * ) "run-time-stats:\r\n Display a table showing how much processing time each FreeRTOS task has used\r\n\r\n",
	prvRunTimeStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : task-stats" 
This generates a table that gives information on each task in the system. */
static const CLI_Command_Definition_t prvTaskStatsCommandDefinition =
{
	( const int8_t * ) "task-stats", /* The command string to type. */
	( const int8_t * ) "task-stats:\r\n Display a table showing the state of each FreeRTOS task\r\n\r\n",
	prvTaskStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ping */
static const CLI_Command_Definition_t pingCommandDefinition =
{
	( const int8_t * ) "ping", /* The command string to type. */
	( const int8_t * ) "ping:\r\n Ping a module\r\n\r\n",
	pingCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : update */
static const CLI_Command_Definition_t bootloaderUpdateCommandDefinition =
{
	( const int8_t * ) "update", /* The command string to type. */
	( const int8_t * ) "update:\r\n Put the module in factory bootloader mode to update its firmware.\r\n\n\
- Use '#n.update' to update remote module n. Make sure the programming port is the incoming port of this module.\r\n\
- Use 'update via #n px' to use the ST Flash Loader tool with module n, port x to update a neighbor module. This is useful if \
target module is not part of the topology or if programming port is not in the shortest path to target module.\r\n\r\n",
	bootloaderUpdateCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : explore */
#ifndef _N
static const CLI_Command_Definition_t exploreCommandDefinition =
{
	( const int8_t * ) "explore", /* The command string to type. */
	( const int8_t * ) "explore:\r\n Explore the array and build its topology\r\n\r\n",
	exploreCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
#endif
/*-----------------------------------------------------------*/
/* CLI command structure : reset */
static const CLI_Command_Definition_t resetCommandDefinition =
{
	( const int8_t * ) "reset", /* The command string to type. */
	( const int8_t * ) "reset:\r\n Reset the module\r\n\r\n",
	resetCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : name */
static const CLI_Command_Definition_t nameCommandDefinition =
{
	( const int8_t * ) "name", /* The command string to type. */
	( const int8_t * ) "name:\r\n Name the module with an alias (1st par.)\r\n\r\n",
	nameCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : group */
static const CLI_Command_Definition_t groupCommandDefinition =
{
	( const int8_t * ) "group", /* The command string to type. */
	( const int8_t * ) "group:\r\n Group multiple modules (2nd+ par.) into a new or existing group (1st par.)\r\n\r\n",
	groupCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : status */
static const CLI_Command_Definition_t statusCommandDefinition =
{
	( const int8_t * ) "status", /* The command string to type. */
	( const int8_t * ) "status:\r\n Display module status\r\n\r\n",
	statusCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : info */
static const CLI_Command_Definition_t infoCommandDefinition =
{
	( const int8_t * ) "info", /* The command string to type. */
	( const int8_t * ) "info:\r\n Display array infromation\r\n\r\n",
	infoCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : scast */
static const CLI_Command_Definition_t scastCommandDefinition =
{
	( const int8_t * ) "scast", /* The command string to type. */
	( const int8_t * ) "scast:\r\n Start a single-cast DMA stream. Source port (1st par.), source module (2nd par.), destination port (3rd par.), \
destination module (4th par.), direction ('forward', 'backward', 'bidirectional') (5th par.), transfer count (bytes) (6th par.), transfer timeout (ms) (7th par.)\r\n\r\n",
	scastCommand, /* The function to run. */
	7 /* Seven parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : addbutton */
static const CLI_Command_Definition_t addbuttonCommandDefinition =
{
	( const int8_t * ) "add-button", /* The command string to type. */
	( const int8_t * ) "add-button:\r\n Define a button at one of the array ports. Button type ('momentary-no', 'momentary-nc', 'onoff-no', 'onoff-nc')(1st par.), Button port (2nd par.)\r\n\r\n",
	addbuttonCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : addbutton */
static const CLI_Command_Definition_t removebuttonCommandDefinition =
{
	( const int8_t * ) "remove-button", /* The command string to type. */
	( const int8_t * ) "remove-button:\r\n Remove a button that was previously defined at this port (1st par.)\r\n\r\n",
	removebuttonCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : set */
static const CLI_Command_Definition_t setCommandDefinition =
{
	( const int8_t * ) "set", /* The command string to type. */
	( const int8_t * ) "set:\r\n Set a parameter (1st par.) with a given value (2nd par.)\r\n\r\n",
	setCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : get */
static const CLI_Command_Definition_t getCommandDefinition =
{
	( const int8_t * ) "get", /* The command string to type. */
	( const int8_t * ) "get:\r\n Get the current value of a parameter (1st par.)\r\n\r\n",
	getCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : default */
static const CLI_Command_Definition_t defaultCommandDefinition =
{
	( const int8_t * ) "default", /* The command string to type. */
	( const int8_t * ) "default:\r\n Type 'default params' to set all parameters to default values\r\n Type 'default array' to remove current topology\r\n\r\n",
	defaultCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : time */
static const CLI_Command_Definition_t timeCommandDefinition =
{
	( const int8_t * ) "time", /* The command string to type. */
	( const int8_t * ) "time:\r\n Display current time in HH:MM:SS-msec format\r\n\r\n",
	timeCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : date */
static const CLI_Command_Definition_t dateCommandDefinition =
{
	( const int8_t * ) "date", /* The command string to type. */
	( const int8_t * ) "date:\r\n Display current date in Weekday MM/DD/YYYY format\r\n\r\n",
	dateCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : set-baudrate */
const CLI_Command_Definition_t setBaudrateCommandDefinition =
{
	( const int8_t * ) "set-baudrate", /* The command string to type. */
	( const int8_t * ) "set-baudrate:\r\n Set UART baudrate\r\n\t(1st parameter): P1 to P6\r\n\t(2nd parameter): baudrate\r\n\r\n",
	setBaudrateCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : uuid */
static const CLI_Command_Definition_t uuidCommandDefinition =
{
	( const int8_t * ) "uuid", /* The command string to type. */
	( const int8_t * ) "uuid:\r\n Display MCU unique UID\r\n\r\n",
	uuidCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : idcode */
static const CLI_Command_Definition_t idcodeCommandDefinition =
{
	( const int8_t * ) "idcode", /* The command string to type. */
	( const int8_t * ) "idcode:\r\n Display MCU IDCODE (DEV_ID and REV_ID)\r\n\r\n",
	idcodeCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : flash-size */
static const CLI_Command_Definition_t flashsizeCommandDefinition =
{
	( const int8_t * ) "flash-size", /* The command string to type. */
	( const int8_t * ) "flash-size:\r\n Display MCU Flash size in Kbytes\r\n\r\n",
	flashsizeCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : snip */
static const CLI_Command_Definition_t snipCommandDefinition =
{
	( const int8_t * ) "snip", /* The command string to type. */
	( const int8_t * ) "snip:\r\n Display a list of stored Command Snippets to edit or delete\r\n\r\n",
	snipCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : act-snip */
static const CLI_Command_Definition_t actSnipCommandDefinition =
{
	( const int8_t * ) "act-snip", /* The command string to type. */
	( const int8_t * ) "act-snip:\r\n Activate a Command Snippet\r\n\r\n",
	actSnipCommand, /* The function to run. */
	1 /* One parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : pause-snip */
static const CLI_Command_Definition_t pauseSnipCommandDefinition =
{
	( const int8_t * ) "pause-snip", /* The command string to type. */
	( const int8_t * ) "pause-snip:\r\n Pause a Command Snippet\r\n\r\n",
	pauseSnipCommand, /* The function to run. */
	1 /* One parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : del-snip */
static const CLI_Command_Definition_t delSnipCommandDefinition =
{
	( const int8_t * ) "del-snip", /* The command string to type. */
	( const int8_t * ) "del-snip:\r\n Delete a Command Snippet\r\n\r\n",
	delSnipCommand, /* The function to run. */
	1 /* One parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : bridge */
static const CLI_Command_Definition_t bridgeCommandDefinition =
{
	( const int8_t * ) "bridge", /* The command string to type. */
	( const int8_t * ) "bridge:\r\n Bridge two array ports\r\n\r\n",
	bridgeCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : Unbridge */
static const CLI_Command_Definition_t unbridgeCommandDefinition =
{
	( const int8_t * ) "unbridge", /* The command string to type. */
	( const int8_t * ) "unbridge:\r\n Un-bridge two array ports\r\n\r\n",
	unbridgeCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/


/* Define long messages -------------------------------------------------------*/

static char * pcBootloaderUpdateMessage = 	\
"\n\rThis module will be forced into bootloader mode.\n\rPlease use the \"STM Flash Loader Demonstrator\" \
utility to update the firmware.\n\r\n\t*** Important ***\n\rIf this module is connected directly to PC please close this port first.\n\r";	

static char * pcRemoteBootloaderUpdateMessage = "\n\rModule %d will be forced into bootloader mode.";	
static char * pcRemoteBootloaderUpdateViaPortMessage = "\n\rRemote update via module %d, port P%d will be triggered.";	

static char * pcRemoteBootloaderUpdateWarningMessage = 	\
"\n\rPlease use the \"STM Flash Loader Demonstrator\" utility to update the firmware.\
\n\r\n\t*** Important ***\n\r- If this module is connected directly to PC please close this port first.\n\r\
- You must power cycle the entire array after the update is finished.\n\r";	

/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/


/* PxMessagingTask function 
*/
void PxMessagingTask(void * argument)
{
	BOS_Status result = BOS_OK; HAL_StatusTypeDef status = HAL_OK;
	uint8_t port, src, dst, temp, i, p, shift, numOfParams; uint16_t code;
	uint32_t count, timeout, temp32;
	bool extendCode = false, extendOptions = false; 
	static int8_t cCLIString[ cmdMAX_INPUT_SIZE ];
	portBASE_TYPE xReturned; int8_t *pcOutputString;
	static uint8_t bcastLastID;
	
	port = (int8_t)(unsigned) argument;
	
	 /* Infinite loop */
	for( ;; )
	{
		
		/* Wait forever until a message is received on one of the ports */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(port>6){port=6;}
		if (messageLength[port-1])
		{						
			/* Long message? Read Options Byte MSB */
			if (cMessage[port-1][2]>>7) {
				longMessage = 1;
			} else {
				longMessage = 0;
			}
			
			/* Read message source and destination */
			dst = cMessage[port-1][0]; 
			src = cMessage[port-1][1];	
			
			/* Reset array index shift */
			shift = 0;
			
			/* Read message options */
			if (cMessage[port-1][2] & 0x01) {						// 1st bit (LSB) Extended options - TODO handle extended options case
				extendOptions = true;	
				(void) extendOptions;		// remove warning		
				++shift;				
			} 
			extendCode = (cMessage[port-1][2]>>1)&0x01;									// 2nd bit Extended code
			BOS.trace = (traceOptions_t)((cMessage[port-1][2]>>2)&0x03);	// 3rd-4th bits Trace 
																																	// 5th bit Reserved
			BOS.response = (cMessage[port-1][2])&0x60;									// 6th-7th bits Response mode
																																	// 8th bit (MSB) Long message
			
			/* Read message code - LSB first */
			if (extendCode == true) {		
				code = ( ( (uint16_t) cMessage[port-1][4+shift] << 8 ) + cMessage[port-1][3+shift] );	
				++shift;
			} else {
				code = cMessage[port-1][3+shift];
			}
	
			/* Is it a transit message? Check for the case when module is being IDed */
			if ( ( dst && (dst < BOS_MULTICAST) && (dst != myID) && (myID != 1) ) || 
					 ( dst && (dst < BOS_MULTICAST) && (dst != myID) && (myID == 1) && (code != CODE_MODULE_ID) ) )
			{
				/* Forward the message to its destination */		
				ForwardReceivedMessage(port);
				if (BOS.trace)
					indMode = IND_SHORT_BLINK;
					
					/* Special messages that require local action */
					if (code == CODE_UPDATE) {		// Remote bootloader update
						Delay_ms(100); remoteBootloaderUpdate(src, dst, port, 0);								
					} else if (code == CODE_UPDATE_VIA_PORT) {		// Remote 'via port' bootloader update
						Delay_ms(100); remoteBootloaderUpdate(src, dst, port, cMessage[port-1][shift]);								
					}
			}
			/* Either broadcast or multicast local message */
			else 
			{				
				/* Is it a broadcast message with unique ID? */
				if (dst == BOS_BROADCAST && cMessage[port-1][messageLength[port-1]-1] != bcastLastID) 
				{
					bcastID = bcastLastID = cMessage[port-1][messageLength[port-1]-1];			// Store bcastID 		
					BroadcastReceivedMessage(BOS_BROADCAST, port);
					cMessage[port-1][messageLength[port-1]-1] = 0;								// Reset bcastID location 
				} 
				/* Reflection of last broadcast message! */
				else if (dst == BOS_BROADCAST && cMessage[port-1][messageLength[port-1]-1] == bcastLastID) 
				{
					result = BOS_ERR_MSG_Reflection;
				}
								
				/* Is it a multicast message with unique ID? */
				if (dst == BOS_MULTICAST && cMessage[port-1][messageLength[port-1]-1] != bcastLastID) 
				{
					bcastID = bcastLastID = cMessage[port-1][messageLength[port-1]-1];			// Store bcastID 		
					BroadcastReceivedMessage(BOS_MULTICAST, port);
					cMessage[port-1][messageLength[port-1]-1] = 0;								// Reset bcastID location 
					temp = cMessage[port-1][messageLength[port-1]-2];							// Number of members in this multicast group - TODO breaks when message is 14 length and padded
					/* Am I part of this multicast group? */
					result = BOS_ERR_WrongID;
					for(i=0 ; i<temp ; i++)
					{
						if (myID == cMessage[port-1][messageLength[port-1]-2-temp+i]) {
							result = BOS_OK;
							break;
						}
					}
				} 
				/* Reflection of last multi-cast message! */
				else if (dst == BOS_MULTICAST && cMessage[port-1][messageLength[port-1]-1] == bcastLastID) 
				{
					result = BOS_ERR_MSG_Reflection;
				}
				
				/* Set shift index to the start of message payload (parameters) */
				shift += 4;
				
				/* Message payload size */
				numOfParams = messageLength[port-1] - shift;
				
				/* Process BOS Messages payload */
				if (result == BOS_OK)
				{
					switch (code)
					{
						case CODE_UNKNOWN_MESSAGE :					
							break;
						
						case CODE_PING :
							indMode = IND_PING;	osDelay(10);
							if (BOS.response == BOS_RESPONSE_ALL || BOS.response == BOS_RESPONSE_MSG)
								SendMessageToModule(src, CODE_PING_RESPONSE, 0);	
							break;

						case CODE_PING_RESPONSE :
							if (!moduleAlias[myID][0])
								sprintf( ( char * ) pcUserMessage, "Hi from module %d\r\n", src);
							else
								sprintf( ( char * ) pcUserMessage, "Hi from module %d (%s)\r\n", src, moduleAlias[src]);
							writePxMutex(PcPort, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
							responseStatus = BOS_OK;								
							break;
							
						case CODE_IND_ON :
							IND_ON();
							break;
						
						case CODE_IND_OFF :
							IND_OFF();
							break;
						
						case CODE_IND_TOGGLE :
							IND_toggle();
							break;
						
						case CODE_HI :					
							/* Record your neighbor info */
							neighbors[port-1][0] = ( (uint16_t) src << 8 ) + cMessage[port-1][2+shift];			/* Neighbor ID + Neighbor own port */
							neighbors[port-1][1] = ( (uint16_t) cMessage[port-1][shift] << 8 ) + cMessage[port-1][1+shift];		/* Neighbor PN */
							/* Send your own info */
							messageParams[1] = (uint8_t) myPN;
							messageParams[0] = (uint8_t) (myPN >> 8);	
							messageParams[2] = port;
							osDelay(2);
							/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
							SendMessageFromPort(port, 0, 0, CODE_HI_RESPONSE, 3);
							break;
						
						case CODE_HI_RESPONSE :
							/* Record your neighbor info */
							neighbors[port-1][0] = ( (uint16_t) src << 8 ) + cMessage[port-1][2+shift];		/* Neighbor ID + Neighbor own port */
							neighbors[port-1][1] = ( (uint16_t) cMessage[port-1][shift] << 8 ) + cMessage[port-1][1+shift];		/* Neighbor PN */	
							responseStatus = BOS_OK;
							break;
					#ifndef _N
						case CODE_EXPLORE_ADJ :
							ExploreNeighbors(port);	indMode = IND_TOPOLOGY;
							osDelay(10); temp = 0;
							/* Exploration response message */
							for (uint8_t p=1 ; p<=NumOfPorts ; p++)  
							{
								if (neighbors[p-1][0])
								{
									messageParams[temp] = p;
									memcpy(messageParams+temp+1, neighbors[p-1], (size_t)(4));
									temp += 5;		
								}
							}
							SendMessageToModule(src, CODE_EXPLORE_ADJ_RESPONSE, temp);
							osDelay(10);
							break;
						
						case CODE_EXPLORE_ADJ_RESPONSE :
							/* Extract the other module neighbors */
							temp = numOfParams/5;
							for (uint8_t k=0 ; k<temp ; k++)  {
								memcpy(&neighbors2[(cMessage[port-1][shift+k*5])-1][0], &cMessage[port-1][1+shift+k*5], (size_t)(4));
							}
							responseStatus = BOS_OK;
							break;
					#endif						
						case CODE_PORT_DIRECTION :
							/* Reverse/un-reverse ports according to command parameters */
							for (uint8_t p=1 ; p<=NumOfPorts ; p++) {
								if (p != port)	SwapUartPins(GetUart(p), cMessage[port-1][shift+p-1]); 
							}
							/* Check the input port direction */
							SwapUartPins(GetUart(port), cMessage[port-1][shift+MaxNumOfPorts]);
							break;
							
						case CODE_PORT_DIRECTION_FINAL:
							
							for (port=1 ; port<=NumOfPorts ; port++) 
							{
								if(array[myID-1][port])
								{
									Nid=array[myID-1][port]>>3;
									if(Nid>myID)
									{
										SwapUartPins(GetUart(port),NORMAL);
									}
									else 
									{
										SwapUartPins(GetUart(port),REVERSED);
									}
								}
								else 
								{
									SwapUartPins(GetUart(port),NORMAL);
								}
							}
							break;
							
						case CODE_MODULE_ID :
								myID = cMessage[port-1][shift];
						break;
						
						case CODE_NEIGHBORS_ID:
								/* Change my neighbor's ID */
								messageParams[0] = cMessage[port-1][shift];		/* The new ID */
								SendMessageFromPort(cMessage[port-1][1+shift], 0, 0, CODE_MODULE_ID, 2);
						break;
							
						case CODE_TOPOLOGY :
								totalofrcvmsg = cMessage[port-1][shift+1];
								mcount = totalofrcvmsg / (MAX_PARAMS_PER_MESSAGE-2); 
								if((totalofrcvmsg % (MAX_PARAMS_PER_MESSAGE-2))!=0){mcount=mcount+1;}
							
								numoflongmsg=cMessage[port-1][shift];
								longMessageLastPtr = (MAX_PARAMS_PER_MESSAGE-1) * (numoflongmsg-1);
								
								if(rcount & (0x01 << (numoflongmsg-1))){
									rcount=0,mcount=0;
								}
								else {
									rcount |= (0x01 << (numoflongmsg-1));
									memcpy(&longMessageScratchpad[0]+longMessageLastPtr, &cMessage[port-1][shift+2],  (size_t) numOfParams );	
								}
								
								if(rcount == (0xff >> (8 - mcount)))
								{
									N = (totalofrcvmsg / (MaxNumOfPorts+1)) / 2;
									memcpy(&array, &longMessageScratchpad, totalofrcvmsg);
									longMessageLastPtr = 0;
									totalofrcvmsg=0 , numoflongmsg=0 , rcount=0 , mcount=0;
									
									SendMessageToModule(1, CODE_TOPOLOGY_RESPONSE,0);
								}
//								if (longMessage) {
//									/* array is 2-byte oriented thus memcpy can copy only even number of bytes TODO test maybe broken */
//									/* Use a 1-byte oriented scratchpad */
//									numoflongmsg=cMessage[port-1][shift];
//									memcpy(&longMessageScratchpad[0]+longMessageLastPtr, &cMessage[port-1][shift+1], (size_t) numOfParams );	
//									longMessageLastPtr += numOfParams;
//								} else {
//									numoflongmsg=cMessage[port-1][shift];
//									memcpy(&longMessageScratchpad[0]+longMessageLastPtr, &cMessage[port-1][shift+1], (size_t) numOfParams );
//									longMessageLastPtr += numOfParams;
//									N = (longMessageLastPtr / (MaxNumOfPorts+1)) / 2;
//									/* Copy the scratchpad to array */
//									memcpy(&array, &longMessageScratchpad, longMessageLastPtr);
//									longMessageLastPtr = 0;

//									SendMessageToModule(1, CODE_TOPOLOGY_RESPONSE,0);
//									//indMode = IND_TOPOLOGY;
//								}	
							break;
						case CODE_TOPOLOGY_RESPONSE:
							Topology_ok=1;
						break;
						
						case CODE_READ_PORT_DIR :
							temp = 0;
							/* Check my own ports */
								for (p=1 ; p<=NumOfPorts ; p++) {
								if (GetUart(p)->AdvancedInit.Swap == UART_ADVFEATURE_SWAP_ENABLE) {
									messageParams[temp++] = p;
								}									
							}
							/* Send response */
							SendMessageToModule(src, CODE_READ_PORT_DIR_RESPONSE, temp);
							break;
						
						case CODE_READ_PORT_DIR_RESPONSE :
							/* Read module ports directions */
							for (p=0 ; p<numOfParams ; p++) 
							{
								arrayPortsDir[src-1] |= (0x8000>>((cMessage[port-1][shift+p])-1));								
							}
							responseStatus = BOS_OK;
							break;		

							case CODE_BAUDRATE :
								/* Change baudrate of specified ports */
								temp = temp32 = 0;
								temp32 = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];		
								if (cMessage[port-1][4+shift] == 0xFF)					// All ports
								{
									for (p=1 ; p<=NumOfPorts ; p++) 
									{
										UpdateBaudrate(p, temp32); 
									}																	
								}
								else
								{
									for (p=0 ; p<numOfParams ; p++) 
									{
										temp = cMessage[port-1][4+shift+p];
										if (temp>0 && temp<=NumOfPorts)	{
											UpdateBaudrate(temp, temp32); 
										}
									}
								}
								break;
								
						case CODE_EXP_EEPROM :
								SaveToRO();
							SaveEEportsDir();
							indMode = IND_PING;
							break;
						
						case CODE_DEF_ARRAY :					
							/* Clear the topology */
							ClearEEportsDir();
							#ifndef _N
							ClearROtopology();
							#endif
							osDelay(100);
							indMode = IND_TOPOLOGY;
							break;
						
						case CODE_CLI_COMMAND :
							/* Obtain the address of the output buffer */
							pcOutputString = FreeRTOS_CLIGetOutputBuffer();
							/* Copy the command */
							if (dst == BOS_BROADCAST)
								memcpy(cCLIString, &cMessage[port-1][shift], (size_t) (numOfParams-1));					// remove bcastID
							else if (dst == BOS_MULTICAST)
								memcpy(cCLIString, &cMessage[port-1][shift], (size_t) (numOfParams-temp-2));		// remove bcastID + groupm members + group count
							else
								memcpy(cCLIString, &cMessage[port-1][shift], (size_t) numOfParams);
							do 
							{
								/* Pass the inport to CLI command parsers temporarily through PcPort */
								temp = PcPort; PcPort = port;
								/* Process the command locally */
								xReturned = FreeRTOS_CLIProcessCommand( cCLIString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );	
								/* Restore back PcPort */
								PcPort = temp;
								/* Respond to the CLI command */
								if (BOS.response == BOS_RESPONSE_ALL)
								{
									/* Copy the generated string to messageParams */
									memcpy(messageParams, pcOutputString, strlen((char*) pcOutputString));
									/* Send command response */	
									SendMessageToModule(src, CODE_CLI_RESPONSE, strlen((char*) pcOutputString));
									osDelay(10); 
								}
							} 
							while( xReturned != pdFALSE );								
							/* Reset the buffer */
							memset( cCLIString, 0x00, cmdMAX_INPUT_SIZE );
							break;
							
						case CODE_CLI_RESPONSE :
							/* Obtain the address of the output buffer and clear the buffer. */
							pcOutputString = FreeRTOS_CLIGetOutputBuffer();
							memset( pcOutputString, 0x00, strlen((char*)pcOutputString) );
							/* Copy the response */
							if (longMessage) {
								memcpy(&pcOutputString[0]+longMessageLastPtr, &cMessage[port-1][shift], (size_t) numOfParams );
								longMessageLastPtr += numOfParams;
							} else {
								memcpy(&pcOutputString[0]+longMessageLastPtr, &cMessage[port-1][shift], (size_t) numOfParams );
								longMessageLastPtr = 0;
								responseStatus = BOS_OK;
								/* Wake up the CliTask again */
								xTaskNotify( ( xCommandConsoleTaskHandle ), 0, eNoAction );			// Notify the task without modifying its notification value
							}							
							break;
							
							case CODE_UPDATE :
								/* Trigger ST factory bootloader update */
								/* Address for RAM signature (STM32F09x) - Last 4 words of SRAM */
								*((unsigned long *)0x20007FF0) = 0xDEADBEEF;   
								indMode = IND_PING;
								osDelay(10);
								NVIC_SystemReset();												
								break;
							
							case CODE_UPDATE_VIA_PORT :
								/* I'm the last module before target. First, ask the target to jump to factory bootloader */
								SendMessageFromPort(cMessage[port-1][shift], 0, 0, CODE_UPDATE, 0);
								osDelay(100);
								/* Then, setup myself for remote 'via port' update */
								remoteBootloaderUpdate(src, myID, port, cMessage[port-1][shift]);
								break;
								
						case CODE_DMA_CHANNEL :
							/* Read EEPROM storage flag */
							temp = cMessage[port-1][11+shift];
							if (numOfParams == 15)	temp = cMessage[port-1][13+shift];							
							if (numOfParams == 17)	temp = cMessage[port-1][15+shift];
								count = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
								timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];									
									
							/* Activate the stream */
							if (temp == false)
							{
								count = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
								timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];									
								if (cMessage[port-1][9+shift] && cMessage[port-1][10+shift])
									SetupDMAStreams(cMessage[port-1][8+shift], count, timeout, cMessage[port-1][9+shift], cMessage[port-1][10+shift]);
								if (cMessage[port-1][11+shift] && cMessage[port-1][12+shift])
									SetupDMAStreams(cMessage[port-1][8+shift], count, timeout, cMessage[port-1][11+shift], cMessage[port-1][12+shift]);
								if (cMessage[port-1][13+shift] && cMessage[port-1][14+shift])
									SetupDMAStreams(cMessage[port-1][8+shift], count, timeout, cMessage[port-1][13+shift], cMessage[port-1][14+shift]);
							}
							/* Save stream paramters in EEPROM */
							else
							{
								EE_WriteVariable(_EE_DMA_STREAM_BASE, cMessage[port-1][8+shift]);			/* Direction */
								EE_WriteVariable(_EE_DMA_STREAM_BASE+1, ( (uint16_t) cMessage[port-1][shift] << 8 ) + cMessage[port-1][1+shift]);			/* Count high half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE+2, ( (uint16_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift]);			/* Count low half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE+3, ( (uint16_t) cMessage[port-1][4+shift] << 8 ) + cMessage[port-1][5+shift]);			/* Timeout high half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE+4, ( (uint16_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift]);			/* Timeout low half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE+5, ( (uint16_t) cMessage[port-1][9+shift] << 8 ) + cMessage[port-1][10+shift]);			/* src1 | dst1 */
								if (numOfParams == 19)
									EE_WriteVariable(_EE_DMA_STREAM_BASE+6, ( (uint16_t) cMessage[port-1][11+shift] << 8 ) + cMessage[port-1][12+shift]);			/* src2 | dst2 */
								if (numOfParams == 21)
									EE_WriteVariable(_EE_DMA_STREAM_BASE+7, ( (uint16_t) cMessage[port-1][13+shift] << 8 ) + cMessage[port-1][14+shift]);			/* src3 | dst3 */
								/* Reset MCU */
								NVIC_SystemReset();
							}
							break;
						
						case CODE_DMA_SCAST_STREAM :
							count = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
							timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];
							StartScastDMAStream(cMessage[port-1][9+shift], myID, cMessage[port-1][11+shift], cMessage[port-1][10+shift], cMessage[port-1][8+shift], count, timeout, cMessage[port-1][12+shift]);
							break;								
						
						
							case CODE_READ_REMOTE :	
							 if	(cMessage[port-1][shift]==REMOTE_MEMORY_ADD)											// request for a memory address
							{
									// Get requested address
									temp32 = ( (uint32_t) cMessage[port-1][2+shift] << 24 ) + ( (uint32_t) cMessage[port-1][3+shift] << 16 ) + ( (uint32_t) cMessage[port-1][4+shift] << 8 ) + cMessage[port-1][5+shift];				
									// Get variable according to requested format
									switch (cMessage[port-1][1+shift])											// requested format
                  {
										case FMT_BOOL:
                  	case FMT_UINT8: 
											messageParams[0] = *(__IO uint8_t *)temp32; 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 1); break;
                  	case FMT_INT8: 
											messageParams[0] = *(__IO int8_t *)temp32; 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 1); break;
                  	case FMT_UINT16: 
											messageParams[0] = (uint8_t)((*(__IO uint16_t *)temp32)>>0); messageParams[1] = (uint8_t)((*(__IO uint16_t *)temp32)>>8);  
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 2); break;
                  	case FMT_INT16: 
											messageParams[0] = (uint8_t)((*(__IO int16_t *)temp32)>>0); messageParams[1] = (uint8_t)((*(__IO int16_t *)temp32)>>8); 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 2); break;
                  	case FMT_UINT32: 
											messageParams[0] = (uint8_t)((*(__IO uint32_t *)temp32)>>0); messageParams[1] = (uint8_t)((*(__IO uint32_t *)temp32)>>8); 
											messageParams[2] = (uint8_t)((*(__IO uint32_t *)temp32)>>16); messageParams[3] = (uint8_t)((*(__IO uint32_t *)temp32)>>24); 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 4); break;
                  	case FMT_INT32: 
											messageParams[0] = (uint8_t)((*(__IO int32_t *)temp32)>>0); messageParams[1] = (uint8_t)((*(__IO int32_t *)temp32)>>8); 
											messageParams[2] = (uint8_t)((*(__IO int32_t *)temp32)>>16); messageParams[3] = (uint8_t)((*(__IO int32_t *)temp32)>>24);
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 4); break;										
                  	case FMT_FLOAT:
											messageParams[0] = *(__IO uint8_t *)(temp32+0); messageParams[1] = *(__IO uint8_t *)(temp32+1); 
											messageParams[2] = *(__IO uint8_t *)(temp32+2); messageParams[3] = *(__IO uint8_t *)(temp32+3); 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 8); break;	// You cannot bitwise floats	
                  	default:
                  		break;
                  }		
								}	
								else if(cMessage[port-1][shift]==REMOTE_MODULE_PARAM)			// request for a Module param
								{
									cMessage[port-1][messageLength[port-1]-1] = 0;		 // adding string termination
									temp=IsModuleParameter((char *)&cMessage[port-1][1+shift]);          // extrating module parameter
									if (temp == 0) {																					// Parameter does not exist
									SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 1);							
								} else {
										// Parameter exists. Get its pointer
										temp32 = (uint32_t) modParam[temp-1].paramPtr;
										messageParams[0] = modParam[temp-1].paramFormat;
										// Send parameter according to its format
									switch (messageParams[0])											// requested format
									{
										case FMT_BOOL:
										case FMT_UINT8: 
											messageParams[1] = *(__IO uint8_t *)temp32; 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 2); break;
										case FMT_INT8: 
											messageParams[1] = *(__IO int8_t *)temp32; 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 2); break;
										case FMT_UINT16: 
											messageParams[1] = (uint8_t)((*(__IO uint16_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO uint16_t *)temp32)>>8); 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 3); break;
										case FMT_INT16: 
											messageParams[1] = (uint8_t)((*(__IO int16_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO int16_t *)temp32)>>8); 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 3); break;
										case FMT_UINT32: 
											messageParams[1] = (uint8_t)((*(__IO uint32_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO uint32_t *)temp32)>>8); 
											messageParams[3] = (uint8_t)((*(__IO uint32_t *)temp32)>>16); messageParams[4] = (uint8_t)((*(__IO uint32_t *)temp32)>>24); 
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 5); break;
										case FMT_INT32: 
											messageParams[1] = (uint8_t)((*(__IO int32_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO int32_t *)temp32)>>8); 
											messageParams[3] = (uint8_t)((*(__IO int32_t *)temp32)>>16); messageParams[4] = (uint8_t)((*(__IO int32_t *)temp32)>>24);
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 5); break;										
										case FMT_FLOAT:
											messageParams[1] = *(__IO uint8_t *)(temp32+0); messageParams[2] = *(__IO uint8_t *)(temp32+1);  
											messageParams[3] = *(__IO uint8_t *)(temp32+2); messageParams[4] = *(__IO uint8_t *)(temp32+3);  			// You cannot bitwise floats	
											SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 9); break;			
										default:
											break;
									}											
								}
							}
								else if(cMessage[port-1][shift]>=REMOTE_BOS_VAR)			// request for a BOS var
							{
									messageParams[0] = BOS_var_reg[cMessage[port-1][shift]-REMOTE_BOS_VAR-1]&0x000F;					// send variable format (lower 4 bits)
									if (messageParams[0] == 0) {																					// Variable does not exist
										SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 1);							
									} else {
										// Variable exists. Get its memory address
										temp32 = (BOS_var_reg[cMessage[port-1][shift]-REMOTE_BOS_VAR-1]>>16) + SRAM_BASE;
										// Send variable according to its format
										switch (messageParams[0])											// requested format
										{
											case FMT_BOOL:
											case FMT_UINT8: 
												messageParams[1] = *(__IO uint8_t *)temp32; 
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 2); break;
											case FMT_INT8: 
												messageParams[1] = *(__IO int8_t *)temp32; 
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 2); break;
											case FMT_UINT16: 
												messageParams[1] = (uint8_t)((*(__IO uint16_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO uint16_t *)temp32)>>8); 
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 3); break;
											case FMT_INT16: 
												messageParams[1] = (uint8_t)((*(__IO int16_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO int16_t *)temp32)>>8); 
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 3); break;
											case FMT_UINT32: 
												messageParams[1] = (uint8_t)((*(__IO uint32_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO uint32_t *)temp32)>>8); 
												messageParams[3] = (uint8_t)((*(__IO uint32_t *)temp32)>>16); messageParams[4] = (uint8_t)((*(__IO uint32_t *)temp32)>>24); 
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 5); break;
											case FMT_INT32: 
												messageParams[1] = (uint8_t)((*(__IO int32_t *)temp32)>>0); messageParams[2] = (uint8_t)((*(__IO int32_t *)temp32)>>8); 
												messageParams[3] = (uint8_t)((*(__IO int32_t *)temp32)>>16); messageParams[4] = (uint8_t)((*(__IO int32_t *)temp32)>>24);
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 5); break;										
											case FMT_FLOAT:
												messageParams[1] = *(__IO uint8_t *)(temp32+0); messageParams[2] = *(__IO uint8_t *)(temp32+1); 
												messageParams[3] = *(__IO uint8_t *)(temp32+2); messageParams[4] = *(__IO uint8_t *)(temp32+3);  			// You cannot bitwise floats	
												SendMessageToModule(src, CODE_READ_REMOTE_RESPONSE, 9); break;			
											default:
												break;
										}											
									}
								}

								break;			
							
						case CODE_READ_REMOTE_RESPONSE :
							if (remoteBuffer == REMOTE_BOS_VAR || remoteBuffer == REMOTE_MODULE_PARAM)				// We requested a BOS variable or module param
							{
								// Read variable according to its format
								remoteVarFormat = (varFormat_t) cMessage[port-1][shift];
								switch (cMessage[port-1][shift])											// Remote format
								{																									// Note that cMessage[port-1][5+shift] can be unaligned. That's why we cannot use simple memory access
									case 0:																					// This variable does not exist
										responseStatus = BOS_ERR_REMOTE_READ_NO_VAR; break;
									case FMT_BOOL:
									case FMT_UINT8: 
										remoteBuffer = cMessage[port-1][1+shift]; break;
									case FMT_INT8:
										remoteBuffer = (int8_t)cMessage[port-1][1+shift]; break;
									case FMT_UINT16: 
										remoteBuffer = ((uint16_t)cMessage[port-1][1+shift]<<0) + ((uint16_t)cMessage[port-1][2+shift]<<8); break;
									case FMT_INT16: 
										remoteBuffer = ((int16_t)cMessage[port-1][1+shift]<<0) + ((int16_t)cMessage[port-1][2+shift]<<8); break;
									case FMT_UINT32: 
										remoteBuffer = ((uint32_t)cMessage[port-1][1+shift]<<0) + ((uint32_t)cMessage[port-1][2+shift]<<8) + ((uint32_t)cMessage[port-1][3+shift]<<16) + ((uint32_t)cMessage[port-1][4+shift]<<24); break;
									case FMT_INT32:
										remoteBuffer = ((int32_t)cMessage[port-1][1+shift]<<0) + ((int32_t)cMessage[port-1][2+shift]<<8) + ((int32_t)cMessage[port-1][3+shift]<<16) + ((int32_t)cMessage[port-1][4+shift]<<24); break;									
									case FMT_FLOAT:
										remoteBuffer = ((uint32_t)cMessage[port-1][1+shift]<<0) + ((uint32_t)cMessage[port-1][2+shift]<<8) + ((uint32_t)cMessage[port-1][3+shift]<<16) + ((uint32_t)cMessage[port-1][4+shift]<<24); break;
									default:
										break;
								}										
							}
							else if (remoteBuffer == REMOTE_MEMORY_ADD)										// We requested a memory location
							{
								// Read variable according to requested format
								switch (remoteBuffer)															// Requested format
								{																									// Note that cMessage[port-1][shift] can be unaligned. That's why we cannot use simple memory access
									case FMT_BOOL:
									case FMT_UINT8: 
										remoteBuffer = cMessage[port-1][shift]; break;
									case FMT_INT8:
										remoteBuffer = (int8_t)cMessage[port-1][shift]; break;
									case FMT_UINT16: 
										remoteBuffer = ((uint16_t)cMessage[port-1][shift]<<0) + ((uint16_t)cMessage[port-1][1+shift]<<8); break;
									case FMT_INT16: 
										remoteBuffer = ((int16_t)cMessage[port-1][shift]<<0) + ((int16_t)cMessage[port-1][1+shift]<<8); break;
									case FMT_UINT32: 
										remoteBuffer = ((uint32_t)cMessage[port-1][shift]<<0) + ((uint32_t)cMessage[port-1][1+shift]<<8) + ((uint32_t)cMessage[port-1][2+shift]<<16) + ((uint32_t)cMessage[port-1][3+shift]<<24); break;
									case FMT_INT32:
										remoteBuffer = ((int32_t)cMessage[port-1][shift]<<0) + ((int32_t)cMessage[port-1][1+shift]<<8) + ((int32_t)cMessage[port-1][2+shift]<<16) + ((int32_t)cMessage[port-1][3+shift]<<24); break;									
									case FMT_FLOAT:
										remoteBuffer = ((uint32_t)cMessage[port-1][shift]<<0) + ((uint32_t)cMessage[port-1][1+shift]<<8) + ((uint32_t)cMessage[port-1][2+shift]<<16) + ((uint32_t)cMessage[port-1][3+shift]<<24); break;
									default:
										break;
								}															
							}
							else
							{
							}
							// Remote read status
							if (responseStatus != BOS_ERR_REMOTE_READ_NO_VAR)	responseStatus = BOS_OK;
							break;	

							
						case CODE_WRITE_REMOTE :
						case CODE_WRITE_REMOTE_FORCE :
							
							responseStatus = BOS_OK;		// Initialize response
							if(cMessage[port-1][shift])			// request for a BOS var
							{
								// Check variable index is within the limit of MAX_BOS_VARS
								if(cMessage[port-1][shift] <= MAX_BOS_VARS)
								{
									temp32 = (BOS_var_reg[cMessage[port-1][shift]-1]>>16) + SRAM_BASE;				// Get var memory addres
									// Modify the variable or create a new one if it does not exist
									switch (cMessage[port-1][1+shift])											// requested format
									{
										case FMT_BOOL:
										case FMT_UINT8: 
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(uint8_t));							// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)			// Write remote value
												*(__IO uint8_t *)temp32 = cMessage[port-1][2+shift];					
											break;
											
										case FMT_INT8: 
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(int8_t));							// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)			// Write remote value
												*(__IO int8_t *)temp32 = (int8_t)cMessage[port-1][2+shift];		
											break;
											
										case FMT_UINT16: 
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(uint16_t));						// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)			// Write remote value
												*(__IO uint16_t *)temp32 = ((uint16_t)cMessage[port-1][2+shift]<<0) + ((uint16_t)cMessage[port-1][3+shift]<<8);					
											break;
											
										case FMT_INT16: 
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(int16_t));							// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)			// Write remote value
												*(__IO int16_t *)temp32 = ((int16_t)cMessage[port-1][2+shift]<<0) + ((int16_t)cMessage[port-1][3+shift]<<8);					
											break;
											
										case FMT_UINT32: 
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(uint32_t));						// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)			// Write remote value
												*(__IO uint32_t *)temp32 = ((uint32_t)cMessage[port-1][2+shift]<<0) + ((uint32_t)cMessage[port-1][3+shift]<<8) + ((uint32_t)cMessage[port-1][4+shift]<<16) + ((uint32_t)cMessage[port-1][5+shift]<<24);					
											break;
											
										case FMT_INT32: 
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(int32_t));							// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)			// Write remote value
												*(__IO int32_t *)temp32 = ((int32_t)cMessage[port-1][2+shift]<<0) + ((int32_t)cMessage[port-1][3+shift]<<8) + ((int32_t)cMessage[port-1][4+shift]<<16) + ((int32_t)cMessage[port-1][5+shift]<<24);					
											break;
											
										case FMT_FLOAT:
											if ((BOS_var_reg[cMessage[port-1][shift]-1]&0x000F) == 0) {		// Variable does not exist																															
												temp32 = (uint32_t)malloc(sizeof(float));								// Create a new one
												if (temp32 != 0) {
													BOS_var_reg[cMessage[port-1][shift]-1] = ((temp32-SRAM_BASE)<<16) + cMessage[port-1][1+shift];
												} else {																								// Cannot alocate memory
													responseStatus = BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if (responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) {			// Write remote value
												remoteBuffer = ((uint32_t)cMessage[port-1][2+shift]<<0) + ((uint32_t)cMessage[port-1][3+shift]<<8) + ((uint32_t)cMessage[port-1][4+shift]<<16) + ((uint32_t)cMessage[port-1][5+shift]<<24);
												*(float *)temp32 = *(float *)&remoteBuffer;																		
											}
											break;												
													
										default:
											break;
									}			
									
									/* Update local format if needed - Todo give local warning later */
									if ( (BOS_var_reg[cMessage[port-1][shift]-1] & 0x000F) != cMessage[port-1][1+shift] ) {		
										BOS_var_reg[cMessage[port-1][shift]-1] &= (0xFFF0+cMessage[port-1][1+shift]);
										responseStatus = BOS_ERR_LOCAL_FORMAT_UPDATED;
									}								
								}		
								else
								{		
									responseStatus = BOS_ERR_REMOTE_WRITE_INDEX;		// BOS var index out of range
								}	
							}
							else												// request for a memory address
							{
								// Get the requested address
								temp32 = ( (uint32_t) cMessage[port-1][2+shift] << 24 ) + ( (uint32_t) cMessage[port-1][3+shift] << 16 ) + ( (uint32_t) cMessage[port-1][4+shift] << 8 ) + cMessage[port-1][5+shift];				
								// Write data to Flash or SRAM based on requested format
								if ( temp32 >= SRAM_BASE && temp32 < (SRAM_BASE+SRAM_SIZE) )			// SRAM
								{
									switch (cMessage[port-1][1+shift])															// Requested format
									{																									
										case FMT_BOOL:
										case FMT_UINT8: 
											*(__IO uint8_t *)temp32 = cMessage[port-1][6+shift]; break;
										case FMT_INT8:
											*(__IO int8_t *)temp32 = (int8_t)cMessage[port-1][6+shift]; break;
										case FMT_UINT16: 
											*(__IO uint16_t *)temp32 = ((uint16_t)cMessage[port-1][6+shift]<<0) + ((uint16_t)cMessage[port-1][7+shift]<<8);	break;
										case FMT_INT16: 
											*(__IO int16_t *)temp32 = ((int16_t)cMessage[port-1][6+shift]<<0) + ((int16_t)cMessage[port-1][7+shift]<<8);	break;
										case FMT_UINT32: 
											*(__IO uint32_t *)temp32 = ((uint32_t)cMessage[port-1][6+shift]<<0) + ((uint32_t)cMessage[port-1][7+shift]<<8) + ((uint32_t)cMessage[port-1][8+shift]<<16) + ((uint32_t)cMessage[port-1][9+shift]<<24); break;
										case FMT_INT32:
											*(__IO int32_t *)temp32 = ((int32_t)cMessage[port-1][6+shift]<<0) + ((int32_t)cMessage[port-1][7+shift]<<8) + ((int32_t)cMessage[port-1][8+shift]<<16) + ((int32_t)cMessage[port-1][9+shift]<<24); break; 									
										case FMT_FLOAT:
											remoteBuffer = ((uint32_t)cMessage[port-1][6+shift]<<0) + ((uint32_t)cMessage[port-1][7+shift]<<8) + ((uint32_t)cMessage[port-1][8+shift]<<16) + ((uint32_t)cMessage[port-1][9+shift]<<24);
											*(float *)temp32 = *(float *)&remoteBuffer;	break;
										default:
											break;
									}												
								}
								else if ( temp32 >= FLASH_BASE && temp32 < (FLASH_BASE+FLASH_SIZE) )			// Flash
								{								
									HAL_FLASH_Unlock();
									/* Erase page if force write is requested */
									if (code == CODE_WRITE_REMOTE_FORCE)
									{
										FLASH_EraseInitTypeDef erase; uint32_t eraseError;
										erase.TypeErase = FLASH_TYPEERASE_PAGES;
										erase.PageAddress = temp32;
										erase.NbPages = 1;
										status = HAL_FLASHEx_Erase(&erase, &eraseError);
										if (status != HAL_OK || eraseError != 0xFFFFFFFF) responseStatus = BOS_ERR_REMOTE_WRITE_FLASH;							
									}
									/* Write new value */
									if (responseStatus == BOS_OK)
									{
										switch (cMessage[port-1][1+shift])															// Requested format
										{																									
											case FMT_BOOL:
											case FMT_UINT8: 
											case FMT_INT8:
												if (*(__IO uint16_t *)temp32 != 0xFFFF) {
													responseStatus = BOS_ERR_REMOTE_WRITE_FLASH; break;
												} else {
													remoteBuffer = cMessage[port-1][6+shift]; status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, temp32, remoteBuffer); break;
												}
											case FMT_UINT16: 
											case FMT_INT16:
												if (*(__IO uint16_t *)temp32 != 0xFFFF) {
													responseStatus = BOS_ERR_REMOTE_WRITE_FLASH; break;
												} else {
													remoteBuffer = ((uint16_t)cMessage[port-1][6+shift]<<0) + ((uint16_t)cMessage[port-1][7+shift]<<8);
													status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, temp32, remoteBuffer); break;
												}
											case FMT_UINT32: 
											case FMT_INT32:
												if (*(__IO uint32_t *)temp32 != 0xFFFFFFFF) {
													responseStatus = BOS_ERR_REMOTE_WRITE_FLASH; break;
												} else {
													remoteBuffer = ((uint32_t)cMessage[port-1][6+shift]<<0) + ((uint32_t)cMessage[port-1][7+shift]<<8) + ((uint32_t)cMessage[port-1][8+shift]<<16) + ((uint32_t)cMessage[port-1][9+shift]<<24); 
													status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, temp32, remoteBuffer); break;
												}				
											case FMT_FLOAT:
												if (*(__IO uint32_t *)temp32 != 0xFFFFFFFF) {
													responseStatus = BOS_ERR_REMOTE_WRITE_FLASH; break;
												} else {
													remoteBuffer = ((uint32_t)cMessage[port-1][6+shift]<<0) + ((uint32_t)cMessage[port-1][7+shift]<<8) + ((uint32_t)cMessage[port-1][8+shift]<<16) + ((uint32_t)cMessage[port-1][9+shift]<<24);
													status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, temp32, remoteBuffer); break;
												}
											default:
												break;
										}		
									}										
									HAL_FLASH_Lock();
									if (status != HAL_OK)	responseStatus = BOS_ERR_REMOTE_WRITE_FLASH;
								}	
								else	
									responseStatus = BOS_ERR_REMOTE_WRITE_ADDRESS;
							}
							
							/* Send confirmation back */
							if (BOS.response == BOS_RESPONSE_ALL || BOS.response == BOS_RESPONSE_MSG) {
								messageParams[0] = responseStatus;
								SendMessageToModule(src, CODE_WRITE_REMOTE_RESPONSE, 1);											
							}
							break;	

							
						case CODE_WRITE_REMOTE_RESPONSE :
							responseStatus = (BOS_Status) cMessage[port-1][shift];
							break;	
						
						case CODE_PORT_FORWARD :
							writePxMutex(cMessage[port-1][shift], (char *)&cMessage[port-1][shift+1], numOfParams-1, 10, 10);
							break;
						
						default :
							/* Process module messages */
							result = (BOS_Status) Module_MessagingTask(code, port, src, dst, shift);
							break;
					}
				}
			}	
		}
		
		/* Is it unknown message? */
		if (result == BOS_ERR_UnknownMessage) {
			SendMessageToModule(src, CODE_UNKNOWN_MESSAGE, 0);
			result = BOS_OK;			
		}
		
		/* Reset message buffer */
		memset(cMessage[port-1], 0, (size_t) messageLength[port-1]);
		messageLength[port-1] = 0;
		if (portStatus[port] != STREAM && portStatus[port] != CLI && portStatus[port] != PORTBUTTON) {
			/* Free the port */
			portStatus[port] = FREE;
		}
		
		taskYIELD();
	}

}

/*-----------------------------------------------------------*/

/*  Micro-seconds timebase init function - TIM14 (16-bit)
*/
void TIM_USEC_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM14_CLK_ENABLE();

	/* Peripheral configuration */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = HAL_RCC_GetPCLK1Freq()/1000000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0xFFFF;
  HAL_TIM_Base_Init(&htim14);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim14, &sMasterConfig);
	
	HAL_TIM_Base_Start(&htim14);
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*  Milli-seconds timebase init function - TIM15 (16-bit)
*/
void TIM_MSEC_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM15_CLK_ENABLE();

	/* Peripheral configuration */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = HAL_RCC_GetPCLK1Freq()/1000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0xFFFF;
  HAL_TIM_Base_Init(&htim15);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);
	
	HAL_TIM_Base_Start(&htim15);
}

/*-----------------------------------------------------------*/

/* --- Used by FoundRoute: Find the index of the minimum module in dist that is still unvisited 
*/
uint8_t minArr(uint8_t* arr, uint8_t* Q)
{
	uint8_t smallest = 0xFF; uint8_t index = 0;

	/* Consider first element as smallest */
	if (!Q[0])						// Not visited yet
		smallest = arr[0];

	for (int i=0 ; i<N ; i++) {
		if ((arr[i] < smallest) && !Q[i]) {
			smallest = arr[i];
			index = i;
		}
	}
	
	return index;
}

/*-----------------------------------------------------------*/

/* --- Used by FoundRoute: Check if Q is empty (all modules have been visited) 
*/
uint8_t QnotEmpty(uint8_t* Q)
{		
	char temp = 1;

	for (int i=0 ; i<N ; i++) {
		temp &= Q[i];
	}	
	
	return temp;
}

/*-----------------------------------------------------------*/

/* --- Forward a received message to its destination 
*/
BOS_Status ForwardReceivedMessage(uint8_t incomingPort)
{
	BOS_Status result = BOS_OK;
	uint8_t port, dst;

	/* Single-cast. Do not add broadcast ID */
	AddBcastPayload = false; 	

	dst = cMessage[incomingPort-1][0];
	
	/* Find best output port for destination module */
	port = FindRoute(myID, dst); 
	
	/* Forward the message. Set src and code to 0 to inform the API to copy the exact message received on incomingPort 
			which is passed thru numberOfParams and to use port as output port */
	SendMessageFromPort(port, 0, dst, 0, incomingPort);
	
	return result;	
}

/*-----------------------------------------------------------*/

/* --- Broadcast a received message to all connected modules - TODO update with new protocol
*/
BOS_Status BroadcastReceivedMessage(uint8_t dstGroup, uint8_t incomingPort)
{
	BOS_Status result = BOS_OK;
	
	/* Broadcast ID and groups are already in the payload. Don't add new ones */
	AddBcastPayload = false; dstGroupID = dstGroup;	
	
	/* Forward the message with a broadcast flag. Set src and code to 0 to inform the API to copy the exact message received on 
		incomingPort which is passed thru numberOfParams. Src will be updated with original source inside the function */
	if (dstGroup == BOS_BROADCAST)
		SendMessageFromPort(0, 0, BOS_BROADCAST, 0, incomingPort);
	else
		SendMessageFromPort(0, 0, BOS_MULTICAST, 0, incomingPort);
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Activate Messaging Tasks
*/
void NotifyMessagingTask(uint8_t port)
{
	switch (port)
	{
	#ifdef _P1
		case P1 : 
			xTaskNotifyGive(P1MsgTaskHandle);	break;
	#endif
	#ifdef _P2
		case P2 :
			xTaskNotifyGive(P2MsgTaskHandle);	break;
	#endif
	#ifdef _P3
		case P3 :
			xTaskNotifyGive(P3MsgTaskHandle);	break;
	#endif
	#ifdef _P4
		case P4 :
			xTaskNotifyGive(P4MsgTaskHandle);	break;
	#endif
	#ifdef _P5
		case P5 :
			xTaskNotifyGive(P5MsgTaskHandle);	break;
	#endif
	#ifdef _P6
		case P6 :
			xTaskNotifyGive(P6MsgTaskHandle);	break;
	#endif
		default: break;
	}		
}

/*-----------------------------------------------------------*/

/* --- Load stored variables from emulated EEPROM 
*/
void LoadEEvars(void)
{
	/* Load array topology */
#ifndef _N
	LoadROtopology();
#endif	
	/* Load port directions */
	LoadEEportsDir();
	
	/* Load module alias */
	LoadEEalias();

	/* Load group modules */
	LoadEEgroup();
	
	/* Load DMA streams */
	LoadEEstreams();
	
	/* Load parameters. If not found, load defaults */
	LoadEEparams();	
	
	/* Load buttons */
	LoadEEbuttons();	
	
	// Load Command Snippets
	LoadROsnippets();
}

/*-----------------------------------------------------------*/

/* --- Save array topology and Command Snippets in Flash RO --- 
*/
uint8_t SaveToRO(void)
{
	BOS_Status result = BOS_OK; 
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 2, temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t)+1] = {0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {			
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}	
	
	/* Save number of modules and myID */
	if (myID)
	{
		temp = (uint16_t) (N<<8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS, temp);
		FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}			
	
	/* Save topology */
		for(uint8_t i=1 ; i<=N ; i++)
		{
			for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
			{
				if (array[i-1][0]) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS+add, array[i-1][j]);
					add += 2;
					FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					}		
				}				
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s=0 ; s<numOfRecordedSnippets ; s++) 
	{
		if (snippets[s].cond.conditionType) 
		{
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy( (uint8_t *)&snipBuffer[1], (uint8_t *)&snippets[s], sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j=0 ; j<(sizeof(snippet_t)/2) ; j++)
			{		
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)&snipBuffer[j*2]);
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}				
			}			
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j=0 ; j<((strlen(snippets[s].cmd)+1)/2) ; j++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd, *(uint16_t *)(snippets[s].cmd+j*2));
				FlashStatus = FLASH_WaitForLastOperation((uint32_t)HAL_FLASH_TIMEOUT_VALUE); 
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}				
			}				
		}	
	}
	
	HAL_FLASH_Lock();
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Clear array topology in SRAM and Flash RO --- 
*/
uint8_t ClearROtopology(void)
{
	// Clear the array 
	memset(array, 0, sizeof(array));
	N = 1; myID = 0;
	
	return SaveToRO();
}

/*-----------------------------------------------------------*/

/* --- Load Command Snippets stored in Flash RO --- 
*/
uint8_t LoadROsnippets(void)
{
	uint8_t i = 0;
	int currentAdd = RO_MID_ADDRESS;
	char *snipBuffer = (char *) malloc(cmdMAX_INPUT_SIZE);
	if (snipBuffer == NULL)	return BOS_MEM_FULL;
	
	// Exit if no recorded Snippets
	if (*(uint8_t *)currentAdd != 0xFE)	return BOS_ERROR;
	
	/* Load Snippets */
	for(uint8_t s=0 ; s<MAX_SNIPPETS ; s++)
	{
		// Load conditions starting at RO_MID_ADDRESS
		for(i=0 ; i<sizeof(snippet_t) ; i++)
			snipBuffer[i] = (*(__IO uint8_t*)(currentAdd++)); 
		memcpy( (uint8_t *)&snippets[s], (uint8_t *)&snipBuffer[1], sizeof(snippet_t));
		memset(snipBuffer, 0, sizeof(snippet_t)); i = 0;
		// Load commands until you get next 0xFE
		while (*(uint8_t *)currentAdd != 0xFE && *(uint8_t *)currentAdd != 0xFF && i<cmdMAX_INPUT_SIZE)
		{
			snipBuffer[i] = *(uint8_t *)currentAdd;
			++currentAdd; ++i;
		}
		if (snipBuffer[i-1] != 0)		++i;	// String termination char was not recorded, then add one
		// Allocate buffer for the Snippet commands
		snippets[s].cmd = (char *) malloc(i);
		if (snippets[s].cmd == NULL) {
			memset(&snippets[s], 0, sizeof(snippet_t) );
			free(snipBuffer);			
			return BOS_ERR_SNIP_MEM_FULL;
		} else {	
			// Copy the command
			memcpy(snippets[s].cmd, snipBuffer, i);
			++numOfRecordedSnippets;		// Record a successful Snippet
			memset(snipBuffer, 0, i);		
		}
		// Exit if no more Snippets
		if (*(uint8_t *)currentAdd != 0xFE)	break;
	}	
	
	free(snipBuffer);
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- Load array topology stored in Flash RO --- 
*/
uint8_t LoadROtopology(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 2, temp = 0;
	
	/* Load number of modules */
	temp = (*(__IO uint16_t*)(RO_START_ADDRESS));
	
	if (temp == 0xFFFF)				// Memory has been erased
	{
		N = 1;
		myID = 0;
		return BOS_MEM_ERASED;
	}
	else
	{		
		N = (uint8_t) (temp>>8);
		if (N == 0)	N = 1;
		myID = (uint8_t) temp;
		
		/* Load topology */
		for(uint8_t i=1 ; i<=N ; i++)
		{
			for(uint8_t j=0 ; j<=MaxNumOfPorts ; j++)
			{
				array[i-1][j] = (*(__IO uint16_t*)(RO_START_ADDRESS+add));
				add += 2;			
			}
		}	
	}
	
	return result;
}
/*-----------------------------------------------------------*/

/* --- Save array ports directions in EEPROM --- 
*/
BOS_Status SaveEEportsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	for(uint8_t i=1 ; i<=N ; i++)
	{
		if (arrayPortsDir[i-1])
			EE_WriteVariable(_EE_PORT_DIR_BASE+i-1, arrayPortsDir[i-1]);		
		
		if ((i+_EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result = BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Clear array ports directions in EEPROM --- 
*/
BOS_Status ClearEEportsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	memset(arrayPortsDir, 0, sizeof(arrayPortsDir));
	
	for(uint8_t i=1 ; i<=N ; i++)
	{
		if (arrayPortsDir[i-1])
			EE_WriteVariable(_EE_PORT_DIR_BASE+i-1, arrayPortsDir[i-1]);		
		
		if ((i+_EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result = BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load array ports directions stored in EEPROM --- 
*/
BOS_Status LoadEEportsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	for(uint8_t i=1 ; i<=N ; i++)
	{
		EE_ReadVariable(_EE_PORT_DIR_BASE+i-1, &arrayPortsDir[i-1]);		
		
		if ((i+_EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result = BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Save module alias in EEPROM --- 
*/
BOS_Status SaveEEalias(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0;
	
	for(uint8_t i=0 ; i<=N ; i++)				// N+1 module aliases
	{
		if (moduleAlias[i][0]) 				
		{
			for(uint8_t j=1 ; j<=MaxLengthOfAlias ; j+=2)
			{
				temp = (uint16_t) (moduleAlias[i][j-1]<<8) + moduleAlias[i][j];
				EE_WriteVariable(_EE_ALIAS_BASE+add, temp);
				add++;			
			}
		}			
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Save module groups in EEPROM --- 
*/
BOS_Status SaveEEgroup(void)
{
	BOS_Status result = BOS_OK;
	uint16_t add = 0, temp = 0; uint8_t i=0;
	
	/* Save group members */
	for(i=0 ; i<N ; i++)			// N modules
	{
		if (groupModules[i]) 
		{
			EE_WriteVariable(_EE_GROUP_MODULES_BASE+add, groupModules[i]);
			add++;			
		}			
	}

	/* Save group alias */
	for(i=0 ; i<MaxNumOfGroups ; i++)		// MaxNumOfGroups group aliases
	{
		if (groupAlias[i][0]) 				
		{
			for(uint8_t j=1 ; j<=MaxLengthOfAlias ; j+=2)
			{
				temp = (uint16_t) (groupAlias[i][j-1]<<8) + groupAlias[i][j];
				EE_WriteVariable(_EE_GROUP_ALIAS_BASE+add, temp);
				add++;			
			}
		}			
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module alias stored in EEPROM --- 
*/
BOS_Status LoadEEalias(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0;
	
	for(uint8_t i=0 ; i<=N ; i++)				// N+1 module aliases
	{
		for(uint8_t j=1 ; j<=MaxLengthOfAlias ; j+=2)
		{
			EE_ReadVariable(_EE_ALIAS_BASE+add, &temp);
			moduleAlias[i][j] = (uint8_t) temp;
			moduleAlias[i][j-1] = (uint8_t) (temp>>8);
			add++;			
		}
		moduleAlias[i][MaxLengthOfAlias] = '\0';
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module groups stored in EEPROM --- 
*/
BOS_Status LoadEEgroup(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t add = 0, temp = 0; uint8_t i=0;
	
	/* Load group members */
	for(i=0 ; i<N ; i++)			// N modules
	{
		EE_ReadVariable(_EE_GROUP_MODULES_BASE+add, &groupModules[i]);
		add++;
	}

	/* Load group alias */
	for(i=0 ; i<MaxNumOfGroups ; i++)		// MaxNumOfGroups group aliases
	{
		for(uint8_t j=1 ; j<=MaxLengthOfAlias ; j+=2)
		{
			EE_ReadVariable(_EE_GROUP_ALIAS_BASE+add, &temp);
			groupAlias[i][j] = (uint8_t) temp;
			groupAlias[i][j-1] = (uint8_t) (temp>>8);
			add++;			
		}
		groupAlias[i][MaxLengthOfAlias] = '\0';
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module DMA streams --- 
*/
BOS_Status LoadEEstreams(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t temp1 = 0, temp2 = 0, status1 = 0, status2 = 0; 
	uint8_t direction = 0; uint32_t count = 0, timeout = 0;
	static uint8_t src1, dst1, src2, dst2, src3, dst3;
	
	/* Direction */
	status1 = EE_ReadVariable(_EE_DMA_STREAM_BASE, &temp1);
	if (!status1) {
		direction = (uint8_t) temp1;
	}

	/* Count */
	status1 = EE_ReadVariable(_EE_DMA_STREAM_BASE+1, &temp1);
	status2 = EE_ReadVariable(_EE_DMA_STREAM_BASE+2, &temp2);
	if (!status1 && !status2) {
		count = ( (uint32_t) temp1 << 16 ) + temp2;
	}
	
	/* Timeout */
	status1 = EE_ReadVariable(_EE_DMA_STREAM_BASE+3, &temp1);
	status2 = EE_ReadVariable(_EE_DMA_STREAM_BASE+4, &temp2);
	if (!status1 && !status2) {
		timeout = ( (uint32_t) temp1 << 16 ) + temp2;
	}
	
	/* src1 | dst1 */
	status1 = EE_ReadVariable(_EE_DMA_STREAM_BASE+5, &temp1);
	if (!status1) {
		src1 = (uint8_t) (temp1 >> 8);
		dst1 = (uint8_t) temp1;
	}
	
	/* src2 | dst2 */
	status1 = EE_ReadVariable(_EE_DMA_STREAM_BASE+6, &temp1);
	if (!status1) {
		src2 = (uint8_t) (temp1 >> 8);
		dst2 = (uint8_t) temp1;	
	}

	/* src3 | dst3 */
	status1 = EE_ReadVariable(_EE_DMA_STREAM_BASE+7, &temp1);
	if (!status1) {
		src3 = (uint8_t) (temp1 >> 8);
		dst3 = (uint8_t) temp1;
	}
	
	/* Activate the DMA streams */
	if (src1 && dst1)
		SetupDMAStreams(direction, count, timeout, src1, dst1);
	if (src2 && dst2)
		SetupDMAStreams(direction, count, timeout, src2, dst2);
	if (src3 && dst3)
		SetupDMAStreams(direction, count, timeout, src3, dst3);
	
	return result;
}

/*-----------------------------------------------------------*/	

/* --- Save DMA streams to emulated EEPROM. --- 
*/
BOS_Status SaveEEstreams(uint8_t direction, uint32_t count, uint32_t timeout, uint8_t src1, uint8_t dst1, uint8_t src2, \
	uint8_t dst2, uint8_t src3, uint8_t dst3)
{
	BOS_Status result = BOS_OK; 
	
	EE_WriteVariable(_EE_DMA_STREAM_BASE, direction);			/* Direction */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+1, ( (uint16_t) (count >> 8)));				/* Count high half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+2, ( (uint16_t) count));								/* Count low half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+3, ( (uint16_t) (timeout >> 8)));			/* Timeout high half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+4, ( (uint16_t) timeout));							/* Timeout low half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+5, ( (uint16_t) (src1 << 8) ) + (uint16_t) dst1);			/* src1 | dst1 */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+6, ( (uint16_t) (src2 << 8) ) + (uint16_t) dst2);			/* src1 | dst1 */
	EE_WriteVariable(_EE_DMA_STREAM_BASE+7, ( (uint16_t) (src3 << 8) ) + (uint16_t) dst3);			/* src1 | dst1 */
	
	return result;
}

/*-----------------------------------------------------------*/	

/* --- Load module parameters from emulated EEPROM. If erased, loade defualts --- 
*/
BOS_Status LoadEEparams(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t temp1, temp2, status1, status2; 
	
	/* Read params base - BOS response and BOS trace */
	status1 = EE_ReadVariable(_EE_PARAMS_BASE, &temp1);
	/* Found the variable (EEPROM is not cleared) */
	if (!status1) {
		BOS.response = (uint8_t)temp1;
		BOS.trace = (traceOptions_t)(temp1>>8);
	/* Couldn't find the variable, load default config */
	} else {
		BOS.response = BOS_default.response;
		BOS.trace = BOS_default.trace;
	}
		
	/* Read Button debounce */
	status1 = EE_ReadVariable(_EE_PARAMS_DEBOUNCE, &temp1);
	if (!status1) 
		BOS.buttons.debounce = temp1;
	else
		BOS.buttons.debounce = BOS_default.buttons.debounce;

	/* Read Button single click time */
	status1 = EE_ReadVariable(_EE_PARAMS_SINGLE_CLICK, &temp1);
	if (!status1) 
		BOS.buttons.singleClickTime = temp1;
	else
		BOS.buttons.singleClickTime = BOS_default.buttons.singleClickTime;	

	/* Read Button double click time (min and max inter-click) */
	status1 = EE_ReadVariable(_EE_PARAMS_DBL_CLICK, &temp1);
	if (!status1) {
		BOS.buttons.minInterClickTime = (uint8_t)temp1;
		BOS.buttons.maxInterClickTime = (uint8_t)(temp1>>8);
	} else {
		BOS.buttons.minInterClickTime = BOS_default.buttons.minInterClickTime;	
		BOS.buttons.maxInterClickTime = BOS_default.buttons.maxInterClickTime;	
	}
	
	/* Read CLI baudrate */
	status1 = EE_ReadVariable(_EE_CLI_BAUD, &temp1);
	status2 = EE_ReadVariable(_EE_CLI_BAUD+1, &temp2);
	if (!status1 && !status2) 
	{
		BOS.clibaudrate = (uint32_t)temp1 | (((uint32_t)temp2)<<16);
	}
	else if(CLI_LOW_Baudrate_Flag)
		BOS.clibaudrate = CLI_BAUDRATE_1;
	else
		BOS.clibaudrate = BOS_default.clibaudrate;
	
	/* Read RTC hourformat and daylightsaving */
	status1 = EE_ReadVariable(_EE_PARAMS_RTC, &temp1);
	if (!status1) {
		BOS.daylightsaving = (int8_t)temp1;
		BOS.hourformat = (uint8_t)(temp1>>8);
	} else {
		BOS.hourformat = 24;
		BOS.daylightsaving = DAYLIGHT_NONE;
	}		
	
	return result;
}

/*-----------------------------------------------------------*/	

/* --- Save module parameters to emulated EEPROM. --- 
*/
BOS_Status SaveEEparams(void)
{
	BOS_Status result = BOS_OK; 
	
	/* Save params base - BOS response & BOS trace */
	EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<5) | (uint16_t)BOS.response);
		
	/* Save Button debounce */
	EE_WriteVariable(_EE_PARAMS_DEBOUNCE, BOS.buttons.debounce);

	/* Save Button single click time */
	EE_WriteVariable(_EE_PARAMS_SINGLE_CLICK, BOS.buttons.singleClickTime);

	/* Save Button double click time (min and max inter-click) */
	EE_WriteVariable(_EE_PARAMS_DBL_CLICK, ((uint16_t)BOS.buttons.maxInterClickTime<<8) | (uint16_t)BOS.daylightsaving);

	/* Save CLI baudrate */
	EE_WriteVariable(_EE_CLI_BAUD, (uint16_t)BOS.clibaudrate);
	EE_WriteVariable(_EE_CLI_BAUD+1, (uint16_t)(BOS.clibaudrate>>16));
	
	/* Save RTC hourformat and daylightsaving */
	EE_WriteVariable(_EE_PARAMS_RTC, ((uint16_t)BOS.hourformat<<8) | (uint16_t)BOS.buttons.minInterClickTime);
	
	return result;
}

/*-----------------------------------------------------------*/	

/* --- Load button definitions and events from EEPROM --- 
*/
BOS_Status LoadEEbuttons(void)
{
	BOS_Status result = BOS_OK; 
	uint16_t temp16 = 0, status1 = 0; 
	uint8_t temp8 = 0;
	
	for(uint8_t i=0 ; i<=NumOfPorts ; i++)
	{
		status1 = EE_ReadVariable(_EE_BUTTON_BASE+4*(i), &temp16);
		
		if(!status1)																												// This variable exists
		{
			temp8 = (uint8_t)(temp16 >> 8);
			if ( ((temp8 >> 4) == i+1) && ((temp8 & 0x0F) != NONE) )					// This is same port and button type is not none
			{
				button[i+1].type = temp8 & 0x0F;
				button[i+1].events = (uint8_t)temp16;
				EE_ReadVariable(_EE_BUTTON_BASE+4*(i)+1, &temp16);
				button[i+1].pressedX1Sec = (uint8_t)(temp16 >> 8);
				button[i+1].releasedY1Sec = (uint8_t)temp16;
				EE_ReadVariable(_EE_BUTTON_BASE+4*(i)+2, &temp16);
				button[i+1].pressedX2Sec = (uint8_t)(temp16 >> 8);
				button[i+1].releasedY2Sec = (uint8_t)temp16;
				EE_ReadVariable(_EE_BUTTON_BASE+4*(i)+3, &temp16);
				button[i+1].pressedX3Sec = (uint8_t)(temp16 >> 8);
				button[i+1].releasedY3Sec = (uint8_t)temp16;
				/* Setup the button and its events */
				AddPortButton(button[i+1].type, i+1);
				SetButtonEvents(i+1, (button[i+1].events & BUTTON_EVENT_CLICKED), ((button[i+1].events & BUTTON_EVENT_DBL_CLICKED)>>1), button[i+1].pressedX1Sec,\
												button[i+1].pressedX2Sec, button[i+1].pressedX3Sec, button[i+1].releasedY1Sec, button[i+1].releasedY2Sec, button[i+1].releasedY3Sec, BUTTON_EVENT_MODE_CLEAR);
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/	

/* --- Setup DMA streams upon request from another module --- 
*/
BOS_Status SetupDMAStreams(uint8_t direction, uint32_t count, uint32_t timeout, uint8_t src, uint8_t dst)
{
	TimerHandle_t xTimerStream = NULL; 
	
	if (src == dst) {		// Streaming inside the module
		portStatus[src] = STREAM;
		return BOS_ERR_WrongParam;
	} else if (!src || !dst) 
		return BOS_ERR_WrongParam;
	
	/* Start DMA streams */
	if (direction == FORWARD) 
	{									
		if (StartDMAstream(GetUart(src), GetUart(dst), 1) == BOS_ERR_PORT_BUSY)	return BOS_ERR_PORT_BUSY; 
		/* Create a timeout timer */
		xTimerStream = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * )&src, StreamTimerCallback );
		dmaStreamTotal[src-1] = count;
	} 
	else if (direction == BACKWARD) 
	{
		if (StartDMAstream(GetUart(dst), GetUart(src), 1) == BOS_ERR_PORT_BUSY)	return BOS_ERR_PORT_BUSY; 
		/* Create a timeout timer */
		xTimerStream = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * )&dst, StreamTimerCallback );
		dmaStreamTotal[src-1] = count;
	} 
	else if (direction == BIDIRECTIONAL) 
	{
		if (StartDMAstream(GetUart(src), GetUart(dst), 1) == BOS_ERR_PORT_BUSY)	return BOS_ERR_PORT_BUSY;
		/* Create a timeout timer */
		xTimerStream = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * )&src, StreamTimerCallback );
		dmaStreamTotal[src-1] = count;
		if (StartDMAstream(GetUart(dst), GetUart(src), 1) == BOS_ERR_PORT_BUSY)	return BOS_ERR_PORT_BUSY; 
		/* Create a timeout timer */
		xTimerStream = xTimerCreate( "StreamTimer", pdMS_TO_TICKS(timeout), pdFALSE, ( void * )&dst, StreamTimerCallback );
		dmaStreamTotal[dst-1] = count;
	}
	else
		return BOS_ERR_WrongParam;

	
	/* Start the timeout timer */
	if (xTimerStream != NULL)
		xTimerStart( xTimerStream, portMAX_DELAY );
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- DMA stream timer callback --- 
*/
void StreamTimerCallback( TimerHandle_t xTimerStream )
{
	uint32_t tid = 0;
	
	tid = ( uint32_t ) pvTimerGetTimerID( xTimerStream );
	
	StopStreamDMA(tid);
	
	SwitchStreamDMAToMsg(tid);
}

/*-----------------------------------------------------------*/	

/* --- Check for factory reset condition: 
				- P1 TXD is connected to last port RXD    
*/
uint8_t IsFactoryReset(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t P1_TX_Port, P1_RX_Port, P_last_TX_Port, P_last_RX_Port;
	uint16_t P1_TX_Pin, P1_RX_Pin, P_last_TX_Pin, P_last_RX_Pin;
	
	/* -- Setup GPIOs -- */
	
  /* Enable all GPIO Ports Clocks */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	
	/* Get GPIOs */
	GetPortGPIOs(P1, &P1_TX_Port, &P1_TX_Pin, &P1_RX_Port, &P1_RX_Pin);
	GetPortGPIOs(P_LAST, &P_last_TX_Port, &P_last_TX_Pin, &P_last_RX_Port, &P_last_RX_Pin);
	
	/* TXD of first port */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = P1_TX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef *)P1_TX_Port, &GPIO_InitStruct);
	
	/* RXD of last port */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;	
	GPIO_InitStruct.Pin = P_last_RX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef *)P_last_RX_Port, &GPIO_InitStruct);	

	
	/* Check for factory reset conditions */
	HAL_GPIO_WritePin((GPIO_TypeDef *)P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms_no_rtos(5);
	if (HAL_GPIO_ReadPin((GPIO_TypeDef *)P_last_RX_Port,P_last_RX_Pin) == RESET)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *)P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms_no_rtos(5);
		if (HAL_GPIO_ReadPin((GPIO_TypeDef *)P_last_RX_Port,P_last_RX_Pin) == SET) {
			return 1;
		}
	}

	/* Clear flag for formated EEPROM if it was already set */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	*((unsigned long *)0x20007FF0) = 0xFFFFFFFF; 
	
	return 0;
}

/*-----------------------------------------------------------*/	

/* --- Check if booting into lower CLI baudrate:
				- Connect P1 TXD and P2 RXD to boot CLI at 115200
*/
uint8_t IsLowerCLIbaud(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t P1_TX_Port, P1_RX_Port, P2_TX_Port, P2_RX_Port;
	uint16_t P1_TX_Pin, P1_RX_Pin, P2_TX_Pin, P2_RX_Pin;
	
	/* -- Setup GPIOs -- */
	
	/* Get GPIOs */
	GetPortGPIOs(P1, &P1_TX_Port, &P1_TX_Pin, &P1_RX_Port, &P1_RX_Pin);
	GetPortGPIOs(P2, &P2_TX_Port, &P2_TX_Pin, &P2_RX_Port, &P2_RX_Pin);
	
	/* P1 TXD */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = P1_TX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef *)P1_TX_Port, &GPIO_InitStruct);
	
	/* P2 RXD */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;	
	GPIO_InitStruct.Pin = P2_RX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef *)P2_RX_Port, &GPIO_InitStruct);	
	
	/* Check for lower CLI baudrate conditions */
	HAL_GPIO_WritePin((GPIO_TypeDef *)P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms_no_rtos(5);		
	if (HAL_GPIO_ReadPin((GPIO_TypeDef *)P2_RX_Port,P2_RX_Pin) == RESET)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *)P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms_no_rtos(5);		
		if (HAL_GPIO_ReadPin((GPIO_TypeDef *)P2_RX_Port,P2_RX_Pin) == SET) 
		{
			return 1;
		}
	}

	return 0;
}

/*-----------------------------------------------------------*/	

/* --- Format emulated EEPROM for a factory reset
*/
void EE_FormatForFactoryReset(void)
{
	/* Check if EEPROM was just formated? */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	if (*((unsigned long *)0x20007FF0) == 0xBEEFDEAD)
	{
		// Do nothing
	}
	else
	{
		if (EE_Format() == HAL_OK) 
		{
			/* Set flag for formated EEPROM */
			*((unsigned long *)0x20007FF0) = 0xBEEFDEAD; 
		}
	}
	
}

/*-----------------------------------------------------------*/	

/* --- Port buttons state parser
*/
void CheckAttachedButtons(void)
{
	uint32_t TX_Port, RX_Port; 
	uint16_t TX_Pin, RX_Pin;
	uint8_t connected = GPIO_PIN_RESET, state = 0;
	static uint8_t clicked;
	
	for(uint8_t i=1 ; i<=NumOfPorts ; i++)
	{
		if (button[i].type)			// Only check defined butons
		{
			/* 1. Reset button state */
			if (delayButtonStateReset == false)	button[i].state = NONE;		

			/* 2. Get button GPIOs */
			GetPortGPIOs(i, &TX_Port, &TX_Pin, &RX_Port, &RX_Pin);
			
			/* 3. Check if port pins are connected */
			HAL_GPIO_WritePin((GPIO_TypeDef *)TX_Port, TX_Pin, GPIO_PIN_RESET); Delay_us(10);
			if (HAL_GPIO_ReadPin((GPIO_TypeDef *)RX_Port, RX_Pin) == GPIO_PIN_RESET) 
			{
				HAL_GPIO_WritePin((GPIO_TypeDef *)TX_Port, TX_Pin, GPIO_PIN_SET); Delay_us(10);
				connected = HAL_GPIO_ReadPin((GPIO_TypeDef *)RX_Port, RX_Pin); 
			}		
			HAL_GPIO_WritePin((GPIO_TypeDef *)TX_Port, TX_Pin, GPIO_PIN_RESET);
			
			/* 4. Determine button state based on port reading and button type */
			switch (button[i].type)
      {
      	case MOMENTARY_NO:
					if (connected == GPIO_PIN_SET)	
						state = CLOSED;
					else if (connected == GPIO_PIN_RESET)
						state = OPEN;			
      		break;
				
      	case MOMENTARY_NC:
					if (connected == GPIO_PIN_SET)	
						state = CLOSED;
					else if (connected == GPIO_PIN_RESET) 
						state = OPEN;	
      		break;
				
      	case ONOFF_NO:
					if (connected == GPIO_PIN_SET)	
						state = ON;
					else if (connected == GPIO_PIN_RESET) 
						state = OFF;
      		break;
				
      	case ONOFF_NC:
					if (connected == GPIO_PIN_SET)	
						state = OFF;
					else if (connected == GPIO_PIN_RESET) 
						state = ON;
      		break;
				
      	default:
      		break;
      }
			
			/* 5. Debounce this state and update button struct if needed */		
			
			/* 5.A. Possible change of state 1: OPEN > CLOSED or OFF >> ON */
			if (state == CLOSED || state == ON)												
			{
				if (pressCounter[i] < 0xFFFF)	
					++pressCounter[i];																			// Advance the debounce counter
				else	
					pressCounter[i] = 0;																		// Reset debounce counter					
			}
			
			/* 5.B. Possible change of state 2: CLOSED > OPEN or ON >> OFF */
			if (state == OPEN || state == OFF)												
			{
				if (releaseCounter[i] < 0xFFFF)
					++releaseCounter[i];																		// Advance the debounce counter
				else	
					releaseCounter[i] = 0;																	// Reset debounce counter		
				
				if (clicked == 2 && dblCounter[i] <= BOS.buttons.maxInterClickTime)				// Advance the inter-click counter		
					++dblCounter[i];			
				else if (dblCounter[i] > BOS.buttons.maxInterClickTime)	{
					clicked = 0;
					dblCounter[i] = 0;																			// Reset the inter-click counter
				}					
			}
			
			/* Analyze state */
			
			/* 5.C. On press: Record a click if pressed less than 1 second */
			if (pressCounter[i] < BOS.buttons.debounce) 									
			{
				// This is noise. Ignore it
			} 
			else 
			{
				if (pressCounter[i] == BOS.buttons.debounce)
				{
					button[i].state = PRESSED;															// Record a PRESSED event. This event is always reset on next tick.
					++pressCounter[i];
				}
				
				if (releaseCounter[i] > BOS.buttons.debounce)							// Reset releaseCounter if needed - to avoid masking pressCounter on NO switches
					releaseCounter[i] = 0;					
				
				if (pressCounter[i] > BOS.buttons.singleClickTime && pressCounter[i] < 500)	
				{
					if (clicked == 0)
						clicked = 1;																					// Record a possible single click 
					else if (clicked == 2) {
						if (dblCounter[i] > BOS.buttons.minInterClickTime && dblCounter[i] < BOS.buttons.maxInterClickTime) {
							clicked = 3;																				// Record a possible double click 
							dblCounter[i] = 0;																	// Reset the inter-click counter
						}
					}						
				}								
				else if (pressCounter[i] >= 500 && pressCounter[i] < 0xFFFF)	
				{
					if (clicked)	clicked = 0;															// Cannot be a click
					// Process PRESSED_FOR_X_SEC events
					CheckForTimedButtonPress(i);
				}	
			}
			
			/* 5.D. On release: Record a click if pressed less than 1 second */
			if (releaseCounter[i] < BOS.buttons.debounce) 							
			{
				// This is noise. Ignore it
			} 	
			else 
			{
				if (releaseCounter[i] == BOS.buttons.debounce)
				{
					button[i].state = RELEASED;															// Record a RELEASED event. This event is always reset on next tick.
					++releaseCounter[i];
				}
				
				if (pressCounter[i] > BOS.buttons.debounce)								// Reset pressCounter if needed - to avoid masking releaseCounter on NC switches
					pressCounter[i] = 0;				
				
				if (releaseCounter[i] > BOS.buttons.singleClickTime && releaseCounter[i] < 500)	
				{
					if (clicked == 1)
					{
						button[i].state = CLICKED;														// Record a single button click event
						clicked = 2;																					// Prepare for a double click
					}
					else if (clicked == 3)
					{
						button[i].state = DBL_CLICKED;												// Record a double button click event
						clicked = 0;																					// Prepare for a single click					
					}
				}					
				else if (releaseCounter[i] >= 500 && releaseCounter[i] < 0xFFFF)	
				{
					// Process RELEASED_FOR_Y_SEC events
					CheckForTimedButtonRelease(i);
				}	
			}	
			
			/* 6. Run button callbacks if needed */
			switch (button[i].state)
      {
      	case PRESSED :
					buttonPressedCallback(i);
					button[i].state = NONE;
      		break;
				
      	case RELEASED :
					buttonReleasedCallback(i);
					button[i].state = NONE;
      		break;
				
      	case CLICKED :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_CLICKED)) 
					{
						delayButtonStateReset = true;
						buttonClickedCallback(i);
					}
      		break;
				
      	case DBL_CLICKED :				
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_DBL_CLICKED)) 
					{
						delayButtonStateReset = true;
						buttonDblClickedCallback(i);
					}
      		break;
					
      	case PRESSED_FOR_X1_SEC :		
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_PRESSED_FOR_X1_SEC)) 
					{				
						delayButtonStateReset = true;
						buttonPressedForXCallback(i, PRESSED_FOR_X1_SEC-8);
					}
					break;
				case PRESSED_FOR_X2_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_PRESSED_FOR_X2_SEC)) 
					{
						delayButtonStateReset = true;
						buttonPressedForXCallback(i, PRESSED_FOR_X2_SEC-8);
					}
					break;
				case PRESSED_FOR_X3_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_PRESSED_FOR_X3_SEC)) 
					{
						delayButtonStateReset = true;
						buttonPressedForXCallback(i, PRESSED_FOR_X3_SEC-8);
					}
					break;
				
      	case RELEASED_FOR_Y1_SEC :	
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_RELEASED_FOR_Y1_SEC)) 
					{
						delayButtonStateReset = true;
						buttonReleasedForYCallback(i, RELEASED_FOR_Y1_SEC-11);
					}
					break;		
					
				case RELEASED_FOR_Y2_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_RELEASED_FOR_Y2_SEC)) 
					{	
						delayButtonStateReset = true;
						buttonReleasedForYCallback(i, RELEASED_FOR_Y2_SEC-11);
					}
					break;			
					
				case RELEASED_FOR_Y3_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_RELEASED_FOR_Y3_SEC)) 
					{	
						delayButtonStateReset = true;
						buttonReleasedForYCallback(i, RELEASED_FOR_Y3_SEC-11);
					}
					break;
				
      	default:
      		break;
      }	
		}					// Done checking this button
	}						// Done checking all buttons
	
}

/*-----------------------------------------------------------*/	

/* --- Check for timed press button events
*/
BOS_Status CheckForTimedButtonPress(uint8_t port)
{
	BOS_Status result = BOS_OK;
	uint32_t t1 = button[port].pressedX1Sec, t2 = button[port].pressedX2Sec, t3 = button[port].pressedX3Sec;
	
	/* Convert to ms */
	t1 *= 1000; t2 *= 1000; t3 *= 1000;
	
	if (pressCounter[port] == t1)	
	{	
		button[port].state = PRESSED_FOR_X1_SEC;
	}
	else if (pressCounter[port] == t2)	
	{	
		button[port].state = PRESSED_FOR_X2_SEC;
	}		
	else if (pressCounter[port] == t3)	
	{	
		button[port].state = PRESSED_FOR_X2_SEC;
	}	

	return result;	
}

/*-----------------------------------------------------------*/	

/* --- Check for timed release button events
*/
BOS_Status CheckForTimedButtonRelease(uint8_t port)
{
	BOS_Status result = BOS_OK;
	uint32_t t1 = button[port].releasedY1Sec, t2 = button[port].releasedY2Sec, t3 = button[port].releasedY3Sec;

	/* Convert to ms */
	t1 *= 1000; t2 *= 1000; t3 *= 1000;
	
	if (releaseCounter[port] == t1)	
	{	
		button[port].state = RELEASED_FOR_Y1_SEC;
	}
	else if (releaseCounter[port] == t2)	
	{	
		button[port].state = RELEASED_FOR_Y2_SEC;
	}		
	else if (releaseCounter[port] == t3)	
	{	
		button[port].state = RELEASED_FOR_Y2_SEC;
	}	

	return result;	
}

/*-----------------------------------------------------------*/	

/* --- Reset state of attached buttons to avoid recurring callbacks
*/
void ResetAttachedButtonStates(uint8_t *deferReset)
{
	if (!*deferReset)
	{
		for(uint8_t i=1 ; i<=NumOfPorts ; i++)
		{
			if(button[i].state != NONE)
				button[i].state = NONE;
		}
	}	
	//*deferReset = 0;
}

/*-----------------------------------------------------------*/	

/* --- Get GPIO pins and ports of this array port
*/
BOS_Status GetPortGPIOs(uint8_t port, uint32_t *TX_Port, uint16_t *TX_Pin, uint32_t *RX_Port, uint16_t *RX_Pin)
{
	BOS_Status result = BOS_OK;
	
	/* Get port UART */
	UART_HandleTypeDef* huart = GetUart(port);
	
	if (huart == &huart1) 
	{	
#ifdef _Usart1		
		*TX_Port = (uint32_t)USART1_TX_PORT;
		*TX_Pin = USART1_TX_PIN;
		*RX_Port = (uint32_t)USART1_RX_PORT;
		*RX_Pin = USART1_RX_PIN;
#endif
	} 
#ifdef _Usart2	
	else if (huart == &huart2) 
	{	
		*TX_Port = (uint32_t)USART2_TX_PORT;
		*TX_Pin = USART2_TX_PIN;
		*RX_Port = (uint32_t)USART2_RX_PORT;
		*RX_Pin = USART2_RX_PIN;
	} 
#endif
#ifdef _Usart3	
	else if (huart == &huart3) 
	{	
		*TX_Port = (uint32_t)USART3_TX_PORT;
		*TX_Pin = USART3_TX_PIN;
		*RX_Port = (uint32_t)USART3_RX_PORT;
		*RX_Pin = USART3_RX_PIN;
	} 
#endif
#ifdef _Usart4	
	else if (huart == &huart4) 
	{	
		*TX_Port = (uint32_t)USART4_TX_PORT;
		*TX_Pin = USART4_TX_PIN;
		*RX_Port = (uint32_t)USART4_RX_PORT;
		*RX_Pin = USART4_RX_PIN;
	} 
#endif
#ifdef _Usart5	
	else if (huart == &huart5) 
	{	
		*TX_Port = (uint32_t)USART5_TX_PORT;
		*TX_Pin = USART5_TX_PIN;
		*RX_Port = (uint32_t)USART5_RX_PORT;
		*RX_Pin = USART5_RX_PIN;
	} 
#endif
#ifdef _Usart6	
	else if (huart == &huart6) 
	{	
		*TX_Port = (uint32_t)USART6_TX_PORT;
		*TX_Pin = USART6_TX_PIN;
		*RX_Port = (uint32_t)USART6_RX_PORT;
		*RX_Pin = USART6_RX_PIN;
	} 
#endif
#ifdef _Usart7
	else if (huart == &huart7) 
	{		
		*TX_Port = (uint32_t)USART7_TX_PORT;
		*TX_Pin = USART7_TX_PIN;
		*RX_Port = (uint32_t)USART7_RX_PORT;
		*RX_Pin = USART7_RX_PIN;
	} 
#endif
#ifdef _Usart8	
	else if (huart == &huart8) 
	{	
		*TX_Port = (uint32_t)USART8_TX_PORT;
		*TX_Pin = USART8_TX_PIN;
		*RX_Port = (uint32_t)USART8_RX_PORT;
		*RX_Pin = USART8_RX_PIN;
	} 
#endif
	else
		result = BOS_ERROR;	
	
	return result;	
}

/*-----------------------------------------------------------*/	

/* --- Button press callback. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonPressedCallback(uint8_t port)
{	
}

/*-----------------------------------------------------------*/	

/* --- Button release callback. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonReleasedCallback(uint8_t port)
{	
}

/*-----------------------------------------------------------*/	

/* --- Button single click callback. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonClickedCallback(uint8_t port)
{	
}

/*-----------------------------------------------------------*/	

/* --- Button double click callback. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonDblClickedCallback(uint8_t port)
{	
}

/*-----------------------------------------------------------*/	

/* --- Button pressed_for_x callbacks. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonPressedForXCallback(uint8_t port, uint8_t eventType)
{	
}

/*-----------------------------------------------------------*/	

/* --- Button released_for_y callbacks. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonReleasedForYCallback(uint8_t port, uint8_t eventType)
{	
}

/*-----------------------------------------------------------*/	

/* --- Read a value from a remote module. 
			 This API returns a void pointer to the remote value. Cast this pointer to match the appropriate format.
			 If the returned value is NULL, then remote variable does not exist or remote module is not responsive.
					module: Remote module ID. 
					localAddress: Local memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
					remoteAddress: Remote memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
					format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
					timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
					force: Put 1 to force full-page erase before writing to Flash.
*/
BOS_Status WriteToRemote(uint8_t module, uint32_t localAddress, uint32_t remoteAddress, varFormat_t format, uint32_t timeout, uint8_t force)
{
	uint8_t response; uint16_t code;
	
	/* Check whether response is enabled or disabled */
	response = BOS.response;
	if (timeout)
		BOS.response = BOS_RESPONSE_MSG;
	else
		BOS.response = BOS_RESPONSE_NONE;
	
	/* Check if a force write is needed */
	if (force)
		code = CODE_WRITE_REMOTE_FORCE;
	else
		code = CODE_WRITE_REMOTE;
	
	/* Writing to a BOS var */
	if (remoteAddress < FLASH_BASE)
	{
		messageParams[0] = remoteAddress;			// Send BOS variable index
		messageParams[1] = format;						// Send local format
		/* Send variable value based on local format */
		switch (format)											
		{
			case FMT_BOOL:
			case FMT_UINT8: 
				messageParams[2] = *(__IO uint8_t *)localAddress; 
				SendMessageToModule(module, CODE_WRITE_REMOTE, 3); break;
			case FMT_INT8: 
				messageParams[2] = *(__IO int8_t *)localAddress; 
				SendMessageToModule(module, CODE_WRITE_REMOTE, 3); break;
			case FMT_UINT16: 
				messageParams[2] = (uint8_t)((*(__IO uint16_t *)localAddress)>>0); messageParams[3] = (uint8_t)((*(__IO uint16_t *)localAddress)>>8); 
				SendMessageToModule(module, CODE_WRITE_REMOTE, 4); break;
			case FMT_INT16: 
				messageParams[2] = (uint8_t)((*(__IO int16_t *)localAddress)>>0); messageParams[3] = (uint8_t)((*(__IO int16_t *)localAddress)>>8); 
				SendMessageToModule(module, CODE_WRITE_REMOTE, 4); break;
			case FMT_UINT32: 
				messageParams[2] = (uint8_t)((*(__IO uint32_t *)localAddress)>>0); messageParams[3] = (uint8_t)((*(__IO uint32_t *)localAddress)>>8); 
				messageParams[4] = (uint8_t)((*(__IO uint32_t *)localAddress)>>16); messageParams[5] = (uint8_t)((*(__IO uint32_t *)localAddress)>>24); 
				SendMessageToModule(module, CODE_WRITE_REMOTE, 6); break;
			case FMT_INT32: 
				messageParams[2] = (uint8_t)((*(__IO int32_t *)localAddress)>>0); messageParams[3] = (uint8_t)((*(__IO int32_t *)localAddress)>>8); 
				messageParams[4] = (uint8_t)((*(__IO int32_t *)localAddress)>>16); messageParams[5] = (uint8_t)((*(__IO int32_t *)localAddress)>>24);
				SendMessageToModule(module, CODE_WRITE_REMOTE, 6); break;										
			case FMT_FLOAT:
				messageParams[2] = *(__IO uint8_t *)(localAddress+0); messageParams[3] = *(__IO uint8_t *)(localAddress+1); messageParams[4] = *(__IO uint8_t *)(localAddress+2); 
				messageParams[5] = *(__IO uint8_t *)(localAddress+3); messageParams[6] = *(__IO uint8_t *)(localAddress+4); messageParams[7] = *(__IO uint8_t *)(localAddress+5); 
				messageParams[8] = *(__IO uint8_t *)(localAddress+6); messageParams[9] = *(__IO uint8_t *)(localAddress+7); 			// You cannot bitwise floats	
				SendMessageToModule(module, CODE_WRITE_REMOTE, 10); break;			
			default:
				break;
		}					
	}
	/* Writing to a memory address */
	else
	{
		messageParams[0] = 0;													
		messageParams[1] = format;						// Local format
		messageParams[2] = (uint8_t)(remoteAddress>>24); messageParams[3] = (uint8_t)(remoteAddress>>16); // Remote address
		messageParams[4] = (uint8_t)(remoteAddress>>8); messageParams[5] = (uint8_t)remoteAddress; 				
		/* Send variable value based on local format */
		switch (format)											
		{
			case FMT_BOOL:
			case FMT_UINT8: 
				messageParams[6] = *(__IO uint8_t *)localAddress; 
				SendMessageToModule(module, code, 7); break;
			case FMT_INT8: 
				messageParams[6] = *(__IO int8_t *)localAddress; 
				SendMessageToModule(module, code, 7); break;
			case FMT_UINT16: 
				messageParams[6] = (uint8_t)((*(__IO uint16_t *)localAddress)>>0); messageParams[7] = (uint8_t)((*(__IO uint16_t *)localAddress)>>8); 
				SendMessageToModule(module, code, 8); break;
			case FMT_INT16: 
				messageParams[6] = (uint8_t)((*(__IO int16_t *)localAddress)>>0); messageParams[7] = (uint8_t)((*(__IO int16_t *)localAddress)>>8); 
				SendMessageToModule(module, code, 8); break;
			case FMT_UINT32: 
				messageParams[6] = (uint8_t)((*(__IO uint32_t *)localAddress)>>0); messageParams[7] = (uint8_t)((*(__IO uint32_t *)localAddress)>>8); 
				messageParams[8] = (uint8_t)((*(__IO uint32_t *)localAddress)>>16); messageParams[9] = (uint8_t)((*(__IO uint32_t *)localAddress)>>24); 
				SendMessageToModule(module, code, 10); break;
			case FMT_INT32: 
				messageParams[6] = (uint8_t)((*(__IO int32_t *)localAddress)>>0); messageParams[7] = (uint8_t)((*(__IO int32_t *)localAddress)>>8); 
				messageParams[8] = (uint8_t)((*(__IO int32_t *)localAddress)>>16); messageParams[9] = (uint8_t)((*(__IO int32_t *)localAddress)>>24);
				SendMessageToModule(module, code, 10); break;										
			case FMT_FLOAT:
				messageParams[6] = *(__IO uint8_t *)(localAddress+0); messageParams[7] = *(__IO uint8_t *)(localAddress+1); messageParams[8] = *(__IO uint8_t *)(localAddress+2); 
				messageParams[9] = *(__IO uint8_t *)(localAddress+3); messageParams[10] = *(__IO uint8_t *)(localAddress+4); messageParams[11] = *(__IO uint8_t *)(localAddress+5); 
				messageParams[12] = *(__IO uint8_t *)(localAddress+6); messageParams[13] = *(__IO uint8_t *)(localAddress+7); 			// You cannot bitwise floats	
				SendMessageToModule(module, code, 14); break;			
			default:
				break;
		}			
	}
	
	/* Restore response settings to default */
	BOS.response = response;

	/* If confirmation is requested, wait for it until timeout */
	if (timeout)
	{
		uint32_t t0 = HAL_GetTick();
		while ( (responseStatus != BOS_OK) && ((HAL_GetTick()-t0) < timeout) ) { };	
		return responseStatus;
	}
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- Initialize and config the internal real-time clock (RTC) and boot status.
*/
BOS_Status RTC_Init(void)
{
	/* RTC clock enable */
  __HAL_RCC_RTC_ENABLE();
	
	/* Configure the RTC 
		f_ckspre = f_rtcclk / ((PREDIV_S+1) * (PREDIV_A+1))
			- f_rtcclk is HSE 8 MHz / 32 = 250 kHz
			- f_ckspre should be 1 Hz 
			- PREDIV_A should be as high as possible to minimize power consumption
					>> Choose PREDIV_A = 124 and PREDIV_S = 1999
	*/
	RtcHandle.Instance = RTC; 
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = 124;
  RtcHandle.Init.SynchPrediv = 1999;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
		
	if (HAL_RTC_Init(&RtcHandle) != HAL_OK)	return BOS_ERROR;

  /* Check if Data stored in BackUp register1: No Need to reconfigure RTC */
  /* Read the Back Up Register 1 Data */
  if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) != 0x32F2)
  {
    /* Configure RTC Calendar */
    RTC_CalendarConfig();
  }
  else
  {
    /* Check if the Power On Reset flag is set */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
			bootStatus = POWER_ON_BOOT;
    }
    /* Check if Pin Reset flag is set */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
			bootStatus = RESET_BOOT;
    }
  }
  /* Clear source Reset Flag */
  __HAL_RCC_CLEAR_RESET_FLAGS();	
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- First time-configuration of the internal real-time clock.
*/
BOS_Status RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;	
	uint8_t month, day, year, seconds, minutes, hours; 
	char comDate[] = __DATE__, comTime[] = __TIME__;
	
	/* Get compile date */
  year = atoi(comDate + 9);		// only last 2 digits
  *(comDate + 6) = 0;
  day = atoi(comDate + 4);
  *(comDate + 3) = 0;
  for (uint8_t i = 0; i < 12; i++)
  {
    if (!strcmp(comDate, monthStringAbreviated[i]))	
			month = i + 1;
  }

	/* Get compile time */
	seconds = atoi(comTime + 6);
	*(comDate + 5) = 0;
	minutes = atoi(comTime + 3);
	*(comDate + 2) = 0;
	hours = atoi(comTime);
	
  /* Set Date */
  sdatestructure.Year = year;
  sdatestructure.Month = month;
  sdatestructure.Date = day;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;		// Todo - Calculate weekday later
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;

  /* Set Time */
  stimestructure.Hours = hours;
  stimestructure.Minutes = minutes;
  stimestructure.Seconds = seconds;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;	BOS.hourformat = 24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
	
  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;

  /* Writes a data in a RTC Backup data Register1 */
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2);
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
*/
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport)
{
	uint8_t myOutport = 0, lastModule = 0; int8_t *pcOutputString;
	
	/* 1. Get route to destination module */	
	myOutport = FindRoute(myID, dst);
	if (outport && dst == myID) {												/* This is a 'via port' update and I'm the last module */
		myOutport = outport;
		lastModule = myID;
	} else if (outport == 0) {													/* This is a remote update */		
		if (NumberOfHops(dst) == 1)
			lastModule = myID;
		else
			lastModule = route[NumberOfHops(dst)-1];				/* previous module = route[Number of hops - 1] */
	}
	
	/* 2. If this is the source of the message, show status on the CLI */
	if (src == myID)
	{	
		/* Obtain the address of the output buffer.  Note there is no mutual
		exclusion on this buffer as it is assumed only one command console
		interface will be used at any one time. */
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		
		if (outport == 0)		// This is a remote module update
			sprintf( ( char * ) pcOutputString, pcRemoteBootloaderUpdateMessage, dst);
		else								// This is a 'via port' remote update
			sprintf( ( char * ) pcOutputString, pcRemoteBootloaderUpdateViaPortMessage, dst, outport);
			
		strcat(( char * ) pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport, ( char * ) pcOutputString, strlen(( char * )pcOutputString), cmd50ms);
		Delay_ms(100);
	}
	
	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);
	
	/* 4. If this is last module before destination, swap my outport */
	if (lastModule == myID) {
		SwapUartPins(GetUart(myOutport), REVERSED);
	}	
	
	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL, 0xFFFFFFFF, 0xFFFFFFFF, false);	
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
				- Set baudrate to 57600
				- Enable even parity
				- Set datasize to 9 bits
*/
void SetupPortForRemoteBootloaderUpdate(uint8_t port)
{
	UART_HandleTypeDef *huart = GetUart(port);

	huart->Init.BaudRate = 57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);	
	
	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/*-----------------------------------------------------------*/

/* --- Check if this string is a local module parameter or event. Returns parameter index+1
*/
uint8_t IsModuleParameter(char* name)
{
	for(uint8_t i=0; i<NUM_MODULE_PARAMS ;i++)
  {
		if (!strcmp(name, (const char *)(modParam[i].paramName)))
			return i+1;
  }
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Check if this string is a math operator and return its enum
*/
uint8_t IsMathOperator(char* string)
{
	for(uint8_t i=0; i<NUM_MATH_OPERATORS ;i++)
  {
		if (!strcmp(string, "="))
			return MATH_EQUAL;
		else if (!strcmp(string, ">"))
			return MATH_GREATER;
		else if (!strcmp(string, "<"))
			return MATH_SMALLER;
		else if (!strcmp(string, ">="))
			return MATH_GREATER_EQUAL;
		else if (!strcmp(string, "<="))
			return MATH_SMALLER_EQUAL;
		else if (!strcmp(string, "!="))
			return MATH_NOT_EQUAL;
  }
	return 0;
}


/*-----------------------------------------------------------*/

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	
	__HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	

	__SYSCFG_CLK_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- BitzOS initialization. 
*/
void BOS_Init(void)
{
	/* Initialize and configure RTC */
	RTC_Init();
	GetTimeDate();

	/* EEPROM Init */
	EE_Init();
	
  /* Initialize all configured peripherals */
  GPIO_Init();
	DMA_Init();
	TIM_USEC_Init();
	CRC_Init();
	TIM_MSEC_Init();
	
	/* Check for factory reset */
	if (IsFactoryReset())
	{
		/* Format EEPROM once */
		EE_FormatForFactoryReset();
		
		/* Software reset */
		NVIC_SystemReset();
	}
	
	/* Check if booting at lower CLI baudrate */
	if (IsLowerCLIbaud())
	{
		CLI_LOW_Baudrate_Flag = 1;
		/* Initialize the module */
		Delay_ms_no_rtos(50);					// Give other modules time to finish factory reset and baudrate check
		Module_Init();	
		
		BOS.clibaudrate = CLI_BAUDRATE_1;
		/* Update all ports to lower baudrate */
		for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
		{	
			UpdateBaudrate(port, BOS.clibaudrate);
		}
	}
	else
	{
		/* Initialize the module with default baudrate */
		Delay_ms_no_rtos(50);					// Give other modules time to finish factory reset and baudrate check
		Module_Init();				
	}
	
	/* Load stored EEPROM variables */
	LoadEEvars();
	
/* If no pre-defined topology, initialize ports direction */
#ifndef _N
	UpdateMyPortsDir();
#endif	
	
	/* Start backend messaging DMAs */
	SetupMessagingRxDMAs();

	/* Startup indicator sequence */
	if (myID == 0)		/* Native module */
	{
		IND_ON();	Delay_ms_no_rtos(500); IND_OFF();
	}
	else							/* Non-native module */
	{
		IND_ON();	Delay_ms_no_rtos(500); IND_OFF();
		Delay_ms_no_rtos(100);
		IND_ON();	Delay_ms_no_rtos(100); IND_OFF();
		Delay_ms_no_rtos(100);
		IND_ON();	Delay_ms_no_rtos(100); IND_OFF();
	}
	
	/* Reset UART overrun errors in case other modules were already transmitting on startup */
	ResetUartORE();

	BOS_initialized = 1;
}

/*-----------------------------------------------------------*/

/* Register the commands.
*/
void vRegisterCLICommands(void)
{
	/* Register all BOS CLI commands */
	FreeRTOS_CLIRegisterCommand( &prvTaskStatsCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &prvRunTimeStatsCommandDefinition );	
	FreeRTOS_CLIRegisterCommand( &pingCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &bootloaderUpdateCommandDefinition );
#ifndef _N
	FreeRTOS_CLIRegisterCommand( &exploreCommandDefinition );
#endif
	FreeRTOS_CLIRegisterCommand( &resetCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &nameCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &groupCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &statusCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &infoCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &scastCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &addbuttonCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &removebuttonCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &setCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &getCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &defaultCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &timeCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &dateCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &setBaudrateCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &uuidCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &idcodeCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &flashsizeCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &snipCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &actSnipCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &pauseSnipCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &delSnipCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &bridgeCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &unbridgeCommandDefinition);
	
	numOfBosCommands = 28;			// Add "help" command
#ifndef _N	
	numOfBosCommands = 29;
#endif

	/* Register module CLI commands */	
	RegisterModuleCLICommands();
}

/*-----------------------------------------------------------*/

/* --- Get the UART for a given port. 
*/
UART_HandleTypeDef* GetUart(uint8_t port)
{
	switch (port)
	{
	#ifdef _P1
		case P1 : 
			return P1uart;	
	#endif
	#ifdef _P2
		case P2 :
			return P2uart;
	#endif
	#ifdef _P3
		case P3 :
			return P3uart;
	#endif
	#ifdef _P4
		case P4 :
			return P4uart;
	#endif
	#ifdef _P5
		case P5 :
			return P5uart;
	#endif
	#ifdef _P6
		case P6 :
			return P6uart;
	#endif
	#ifdef _P7
		case P7 :
			return P7uart;
	#endif
	#ifdef _P8
		case P8 :
			return P8uart;
	#endif
	#ifdef _P9
		case P9 :
			return P9uart;
	#endif
	#ifdef _P10
		case P10 :
			return P10uart;
	#endif
		default:
			return 0;
	}		
}

/*-----------------------------------------------------------*/

/* --- Broadcast a message to all connected modules
*/
BOS_Status BroadcastMessage(uint8_t src, uint8_t dstGroup, uint16_t code, uint16_t numberOfParams)
{
	/* Set a flag to populate broadcast ID and groups */
	AddBcastPayload = true; dstGroupID = dstGroup;
	
	/* Send the message out with a broadcast flag */
	if (dstGroup == BOS_BROADCAST)
		SendMessageFromPort(0, src, BOS_BROADCAST, code, numberOfParams);
	else
		SendMessageFromPort(0, src, BOS_MULTICAST, code, numberOfParams);

	/* Reset messageParams buffer */
	memset( messageParams, 0, numberOfParams );
	AddBcastPayload = false;
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- Send a message to a group of modules. If current module is part of the group it will be exempted 
*/
BOS_Status SendMessageToGroup(char* group, uint16_t code, uint16_t numberOfParams)
{
	BOS_Status result = BOS_OK;
	uint8_t i = 0; 
	
	/* Search for group alias*/

	for(i=0 ; i<MaxNumOfGroups ; i++)
	{
		/* This group exists */
		if (!strcmp(group, groupAlias[i]))	
		{
			/* Multicast the message to this group */
			result = BroadcastMessage(myID, i, code, numberOfParams);
			
			return result;
		}
	}	
	
	/* This group does not exist */
	return BOS_ERR_WrongGroup;
}

/*-----------------------------------------------------------*/

/* --- Send a message to another module 
*/
BOS_Status SendMessageToModule(uint8_t dst, uint16_t code, uint16_t numberOfParams)
{
	BOS_Status result = BOS_OK;
	uint8_t port = 0; 
	
	/* Singlecast message */
	if (dst != BOS_BROADCAST)
	{
		/* Find best output port for destination module */
		port = FindRoute(myID, dst); 
		
		/* Transmit the message from this port */
		SendMessageFromPort(port, myID, dst, code, numberOfParams);	
		
		/* Reset messageParams buffer */
		memset( messageParams, 0, numberOfParams );
	}
	/* Broadcast message */
	else
	{
		BroadcastMessage(myID, BOS_BROADCAST, code, numberOfParams);
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Send a message from a specific port 
	 Note: The messageParams buffer does not get erased here to enable reuse for other transmissions.
				Make sure you manually erase the buffer when you're done with it. 

				#		port			src				dst							case
				====================================================================================
				1		0					!0				!0							Broadcast or multi-cast message.
				2		0					0					!0							Broadcast or multi-cast message forwarded from another port (which is passed to the API thru numberOfParams).
				3		0					!0				0								Not allowed.
				4		0					0					0								Not allowed.
				5		!0				!0				!0							Single-cast message.
        6   !0        0					!0							Either single-cast message with myID as source module OR (if code == 0)
																								 single-cast message forwarded from another port (which is passed to the API thru numberOfParams).
        7   !0        !0				0								Not allowed.
        8   !0        0					0								Message sent to adjacent neighbor (e.g., if ID is unknown) with myID as source module.
*/
BOS_Status SendMessageFromPort(uint8_t port, uint8_t src, uint8_t dst, uint16_t code, uint16_t numberOfParams)
{
	BOS_Status result = BOS_OK; 
	uint8_t length = 0, shift = 0; static uint16_t totalNumberOfParams = 0; static uint16_t ptrShift = 0; static uint16_t totalParams=0;
	bool extendOptions = false, extendCode = false;
	UBaseType_t TaskPriority;
	
	/* Sanity check broadcast/multi-cast and not allowed cases */
	if ((port == 0 && dst == 0) ||																												// cases 3 & 4
			(port == 0 && dst != BOS_BROADCAST && dst != BOS_MULTICAST) || 										// cases 1 & 2
			(port != 0 && src != 0 && dst == 0)) {																						// case 7
		return BOS_ERR_WrongParam; 
	}
	
	/* Increase the priority of current running task */
	TaskPriority = uxTaskPriorityGet( NULL );
	vTaskPrioritySet( NULL, osPriorityHigh-osPriorityIdle );
	
	/* HZ Delimiter */
	message[0] = 'H';						
	message[1] = 'Z';

	/* Should I copy message buffer from another port or construct from scratch? */
	if ((port == 0 && src == 0 && (dst == BOS_BROADCAST || dst == BOS_MULTICAST)) || code == 0)					// case 2 and part of case 6
	{
		/* Get message length from the incoming port */
		length = messageLength[numberOfParams-1];

		/* Copy message buffer from the incoming port as is */
		memcpy(&message[3], &cMessage[numberOfParams-1][0], (size_t) length);
	}
	/* Construct message from scratch - case 5 */
	else
	{		
		/* Sending to adjacent neighbors - case 2, case 8 and part of case 6 */
		if (src == 0)		src = myID;
				
		/* Extended code flag? */
		if (code > 0xFF)	extendCode = true;
		
		/* TODO implement extended options */		

		
		/* Construct the message */
		
		/* Header */
		message[2] = length;	
		message[3] = dst;						
		message[4] = src;
		
		/* Options */
		/* Long Message (8th-MSB) Response (7th - 6th) : Reserved (5th) : Trace (4th-3rd) : Extended Code (2nd) : Extended Options (1st-LSB) */
		message[5] |= ((BOS.response) | (BOS.trace<<2) | (extendCode<<1) | (extendOptions));
		if (extendOptions == true) {
			++shift;
		}
		
		/* Code - LSB first */
		message[6+shift] = (uint8_t) code;
		if (extendCode == true) {
			++shift;
			message[6+shift] = (uint8_t) (code >> 8);		
		}
		
		/* Parameters */
		
		if (numberOfParams <= MAX_PARAMS_PER_MESSAGE ) {	
			if(longMessagecount>0)
			{
				message[7+shift] = longMessagecount;
				message[8+shift] = totalParams;
				memcpy((char*)&message[9+shift], (&messageParams[0]+ptrShift), numberOfParams);
				/* Calculate message length */
				length = numberOfParams + shift + 4 + 2;
			}
			else
			{
				memcpy((char*)&message[7+shift], (&messageParams[0]+ptrShift), numberOfParams);
				/* Calculate message length */
				length = numberOfParams + shift + 4;
			}
		} else {
			/* Long message: Set Options byte 8th bit */
			message[5] |= 0x80;		
			totalNumberOfParams = numberOfParams-2;				// Leave one byte for long msg counter
			totalParams = numberOfParams-2;
			numberOfParams = MAX_PARAMS_PER_MESSAGE-1;
			/* Break into multiple messages */
			while (totalNumberOfParams != 0)
			{		
				if ( (totalNumberOfParams/numberOfParams) >= 1) 
				{	
					longMessagecount++;
					/* Call this function recursively */
					SendMessageFromPort(port, src, dst, code, numberOfParams);
					osDelay(500);
					/* Increase the priority of current running task */
					TaskPriority = uxTaskPriorityGet( NULL );
					vTaskPrioritySet( NULL, osPriorityHigh-osPriorityIdle );
					/* Update remaining number of parameters */
					totalNumberOfParams -= (numberOfParams-1);
					ptrShift += (numberOfParams);
				} 
				else 
				{
					longMessagecount++;
					message[5] &= 0x7F;		/* Last message. Reset long message flag */
					numberOfParams = totalNumberOfParams-1;
					message[7+shift] = longMessagecount;
					message[8+shift] = totalParams;
					memcpy((char*)&message[9+shift], (&messageParams[0]+ptrShift), numberOfParams);
					ptrShift = 0; totalNumberOfParams = 0; longMessagecount = 0;
					/* Calculate message length */
					length = numberOfParams + shift + 4 + 2;					
				}
			}
		}	

		/* Check if brodcast payload (bcast ID and groups) should be appended to message payload */
		/* TODO - handle the edge case of brodcast/multi-cast long message. bcastID should go into each message but the groups only in the last one */
		
		if(AddBcastPayload == true)
		{
			uint8_t groupMembers = 0;
		
			/* Add group members if it's a multicast */
			if (dstGroupID < BOS_BROADCAST)
			{
				/* Extract and add group member IDs to the Message */
				for(uint16_t i=1 ; i<=N ; i++)						// N modules
				{
					if (InGroup(i, dstGroupID))
					{
						++groupMembers;							// Add this member
						if ((numberOfParams+groupMembers+1) < MAX_PARAMS_PER_MESSAGE)
							message[7+shift+numberOfParams+groupMembers-1] = i;
						else
							return BOS_ERR_MSG_DOES_NOT_FIT;
					}
				}
				/* Add number of members */
				message[7+shift+numberOfParams+groupMembers] = groupMembers;
			}

			/* Add unique broadcast ID */
			if ( (dstGroupID == BOS_BROADCAST) && ((numberOfParams+1) < MAX_PARAMS_PER_MESSAGE) )
				message[7+shift+numberOfParams] = ++bcastID;
			else if (dstGroupID == BOS_BROADCAST)
				return BOS_ERR_MSG_DOES_NOT_FIT;
			else if ( (dstGroupID < BOS_BROADCAST) && ((numberOfParams+groupMembers+2) < MAX_PARAMS_PER_MESSAGE) )		// Multicast
				message[7+shift+numberOfParams+groupMembers+1] = ++bcastID;
			else if (dstGroupID < BOS_BROADCAST)																																		// Multicast
				return BOS_ERR_MSG_DOES_NOT_FIT;
			
			/* Calculate new message length */
			if (dstGroupID == BOS_BROADCAST)
				length += 1;		// + bcastID
			else
				length += groupMembers + 2;		// + bcastID + number of group member + group members IDs 
		}
	}
		
	/* Copy message length */
	message[2] = length;
	
	/* End of message - Calculate CRC8 */	
	memcpy(crcBuffer, &message[0], length + shift + 3);	
	message[length+shift+3] = HAL_CRC_Calculate(&hcrc, (uint32_t *)&crcBuffer, (length + shift + 3)/4);
	if ((length + shift + 3)%4 != 0) 							// Non-word-aligned packet
		message[length+shift+3] = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&crcBuffer[((length + shift + 3)/4)*4], 1);

	memset(crcBuffer, 0, sizeof(crcBuffer));
	//if(! message[length+3]){message[length+3]=1;}  /*Making sure CRC Value Is not Zero*/
	
	/* Transmit the message - single-cast */
	if (dst != BOS_BROADCAST && dst != BOS_MULTICAST) 
	{
		writePxDMAMutex(port, message, length+4, cmd50ms);
		num++;
	}
	/* Transmit the message - multi-cast or broadcast */
	else
	{
		if (code == 0 && src == 0) {					// Forwarded broadcast or multicast. Update with original source.
			src = message[4];
		} 
		
		/* Get broadcast routes */
		FindBroadcastRoutes(src);
		
		/* Send to all my broadcast ports */
		for (uint8_t p=1 ; p<=NumOfPorts ; p++) 
		{
			if ( (bcastRoutes[myID-1] >> (p-1)) & 0x01 ) 		
			{
				/* Transmit the message from this port */
				writePxDMAMutex(p, message, length+4, cmd50ms);
				Delay_ms(1);
			}	
		}
	}

	/* Put the priority of current running task back to its default state */
	vTaskPrioritySet( NULL, TaskPriority );
	
	/* Reset responseStatus in case response is expected - TODO should be tailored for each port */
	responseStatus = BOS_ERR_NoResponse;
	
	return result;
}

/*-----------------------------------------------------------*/

#ifndef _N
/* --- Explore the array and create its topology (executed only by master)
*/
BOS_Status Explore(void)
{
	BOS_Status result = BOS_OK;
	uint8_t currentID = 0, lastID = 0, temp1 = 0, temp2 = 0, i = 0, j = 0, ports = 0, portn = 0, porta=0, Hi_count=0;
	uint16_t temp16 = 0;
	
	myID = 1; 		/* Master ID */
	
	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
	
	for (uint8_t ports=1 ; ports<=NumOfPorts ; ports++) {
		if (ports != PcPort)	SwapUartPins(GetUart(ports), REVERSED);
	}
	
	while(Hi_count<5)
	{
		ExploreNeighbors(PcPort); indMode = IND_TOPOLOGY;
		Hi_count++;
	}
	Hi_count=0;
	
	/* >>> Step 2 - Assign IDs to new modules & update the topology array */
	
	/* Step 2a - Assign IDs to new modules */
	currentID = 1;
	for (portn=1 ; portn<=NumOfPorts ; portn++) 
	{
		if (neighbors[portn-1][0])
		{
			/* New ID */
			messageParams[0] = ++currentID;
			N = currentID;			/* Update number of modules in the array */
			/* Inform module to change ID */
			SendMessageFromPort(portn, 0, 0, CODE_MODULE_ID, 1);			
			/* Modify neighbors table */
			neighbors[portn-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(neighbors[portn-1][0]);
			osDelay(10);
		}
	}
	
	/* Step 2b - Update master topology array */
	array[0][0]	= myPN;					
	for (porta=1 ; porta<=NumOfPorts ; porta++) 
	{
		if (neighbors[porta-1][0])
		{
			temp16 = neighbors[porta-1][0];
			temp1 = (uint8_t)(temp16>>8);										/* Neighbor ID */
			temp2 = (uint8_t)(neighbors[porta-1][0]);				/* Neighbor port */
			/* Module 1 (master) */
			array[0][porta] = ( temp1 << 3 ) | temp2;				/* Neighbor ID | Neighbor port */
			/* Rest of the neighbors */
			array[temp1-1][0]	= neighbors[porta-1][1];				/* Neighbor PN */
			array[temp1-1][temp2] = ( myID << 3 ) | porta;		/* Module 1 ID | Module 1 port */
		}
	}		
	
	/* Step 2c - Ask neighbors to update their topology array */
	for (i=2 ; i<=currentID ; i++) 
	{
		while(Topology_ok==0 && Topology_count<10)
		{
			memcpy(messageParams, array, (size_t) (currentID*(MaxNumOfPorts+1)*2) );
			SendMessageToModule(i, CODE_TOPOLOGY, (size_t) (currentID*(MaxNumOfPorts+1)*2));
			Topology_count++;
			osDelay(100);
		}
		Topology_ok=0;
		Topology_count=0;
	}
	
	
	/* >>> Step 3 - Ask each new module to explore and repeat */
	
	while (lastID != currentID)
	{
		/* Update lastID */
		lastID = currentID;
		
		/* Scan all discovered modules */
		for (i=2 ; i<=currentID ; i++) 
		{
			/* Step 3a - Ask the module to reverse ports */
			for (uint8_t p=1 ; p<=MaxNumOfPorts ; p++) {
				messageParams[p-1] = REVERSED;
			}
			messageParams[MaxNumOfPorts] = NORMAL;		/* Make sure the inport is not reversed */
			SendMessageToModule(i, CODE_PORT_DIRECTION, MaxNumOfPorts+1);
			osDelay(10);
			
			/* Step 3b - Ask the module to explore adjacent neighbors */
			SendMessageToModule(i, CODE_EXPLORE_ADJ, 0);
			osDelay(500);		
		
			/* Step 3c - Assign IDs to new modules */
			for (j=1 ; j<=MaxNumOfPorts ; j++) 
			{
				temp16 = neighbors2[j-1][0];		/* Neighbor ID */
				temp1 = (uint8_t)(temp16>>8);											
				if (temp16 != 0 && temp1 == 0)			/* UnIDed module */
				{
					/* New ID */
					messageParams[0] = ++currentID;		
					N = currentID;			/* Update number of modules in the array */
					/* Modify neighbors table */
					neighbors2[j-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(neighbors2[j-1][0]);
					/* Ask the module to ID its yet unIDed neighbors */
					messageParams[1] = j;		/* neighbor port */
					SendMessageToModule(i, CODE_NEIGHBORS_ID, 2);
					osDelay(10);
				}
			}
			
			/* Step 3d - Update master topology array */
			for (j=1 ; j<=MaxNumOfPorts ; j++) 
			{
				if (neighbors2[j-1][0])
				{	
					temp16 = neighbors2[j-1][0];
					temp1 = (uint8_t)(temp16>>8);										/* Neighbor ID */
					temp2 = (uint8_t)(neighbors2[j-1][0]);					/* Neighbor port */		
					if (temp1 != 1)			/* Execlude the master */
					{
						/* Update module i section */
						if (array[i-1][j] == 0) {
							array[i-1][j] = ( temp1 << 3 ) | temp2;				/* Neighbor ID | Neighbor port */
						}
						/* Update module i neighbors */
						if (array[temp1-1][temp2] == 0) {
							array[temp1-1][0]	= neighbors2[j-1][1];				/* Neighbor PN */
							array[temp1-1][temp2] = ( i << 3 ) | j;				/* Module i ID | Module i port */								
						}
					}
				}
			}	
			
			/* Reset neighbors2 array */
			memset(neighbors2, 0, sizeof(neighbors2) );
		}
	}
	/* Step 3e - Ask all discovered modules to update their topology array */
	for (j=2 ; j<=currentID ; j++) 
	{
		while(Topology_ok==0 && Topology_count<50)
		{
			memcpy(messageParams, array, (size_t) (currentID*(MaxNumOfPorts+1)*2) );
			SendMessageToModule(j, CODE_TOPOLOGY, (size_t) (currentID*(MaxNumOfPorts+1)*2));
			Topology_count++;
			osDelay(100);
		}
		Topology_ok=0;
		Topology_count=0;
	}

	/* >>> Step 4 - Make sure all connected modules have been discovered */
	
	ExploreNeighbors(PcPort);
	/* Check for any unIDed neighbors */
	for (i=1 ; i<=NumOfPorts ; i++) 
	{
		temp16 = neighbors[i-1][0];		/* Neighbor ID */
		temp1 = (uint8_t)(temp16>>8);											
		if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
			result = BOS_ERR_UnIDedModule;
		}		
	}
	/* Ask other modules for any unIDed neighbors */
	for (i=2 ; i<=currentID ; i++) 
	{
		SendMessageToModule(i, CODE_EXPLORE_ADJ, 0);
		osDelay(200);	
		/* Check for any unIDed neighbors */
		for (j=1 ; j<=MaxNumOfPorts ; j++) 
		{
			temp16 = neighbors2[j-1][0];		/* Neighbor ID */
			temp1 = (uint8_t)(temp16>>8);											
			if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
				result = BOS_ERR_UnIDedModule;
			}
		}				
	}
	
//	if(result==BOS_ERR_UnIDedModule)
//	{
//		iteration++;
////		memset(array, 0, sizeof(array) );
////		memset(neighbors, 0, sizeof(neighbors2) );
////		memset(neighbors2, 0, sizeof(array) );
//		//printf
//		goto again;
//	}
	
	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
	
	if (result == BOS_OK)
	{	
//		/* Step 5a - Virtually reset the state of master ports to Normal */
//		for (port=1 ; port<=NumOfPorts ; port++) {
//			arrayPortsDir[0] &= (~(0x8000>>(port-1)));		/* Set bit to zero */
//		}
//		
//		/* Step 5b - Update other modules ports starting from the last one */
//		for (i=currentID ; i>=2 ; i--) 
//		{
//			for (p=1 ; p<=MaxNumOfPorts ; p++) 
//			{		
//				if (!array[i-1][p])	{
//					/* If empty port leave normal */
//					messageParams[p-1] = NORMAL;
//					arrayPortsDir[i-1] &= (~(0x8000>>(p-1)));		/* Set bit to zero */
//				} else {
//					/* If not empty, check neighbor */			
//					temp16 = array[i-1][p];
//					temp1 = (uint8_t)(temp16>>3);										/* Neighbor ID */
//					temp2 = (uint8_t)(temp16 & 0x0007);							/* Neighbor port */	
//					/* Check neighbor port direction */
//					if ( !(arrayPortsDir[temp1-1] & (0x8000>>(temp2-1))) ) {
//						/* Neighbor port is normal */
//						messageParams[p-1] = REVERSED;
//						arrayPortsDir[i-1] |= (0x8000>>(p-1));		/* Set bit to one */
//					} else {
//						/* Neighbor port is reversed */
//						messageParams[p-1] = NORMAL;
//						arrayPortsDir[i-1] &= (~(0x8000>>(p-1)));		/* Set bit to zero */						
//					}				
//				}
//			}
//			
//			/* Step 5c - Check if an inport is reversed */
//			/* Find out the inport to this module from master */
//			FindRoute(1, i);
//			temp1 = route[NumberOfHops(i)-1];			0	/* previous module = route[Number of hops - 1] */
//			temp2 = FindRoute(i, temp1);
//			/* Is the inport reversed? */
//			if ( (temp1 == i) || (messageParams[temp2-1] == REVERSED) )
//				messageParams[MaxNumOfPorts] = REVERSED;		/* Make sure the inport is reversed */
//			
//			/* Step 5d - Update module ports directions */
//			SendMessageToModule(i, CODE_PORT_DIRECTION, MaxNumOfPorts+1);
//			osDelay(10);			
//		}


		SendMessageToModule(BOS_BROADCAST, CODE_PORT_DIRECTION_FINAL, MaxNumOfPorts+1);
		osDelay(500);

		/* Step 5e - Update master ports > all normal */
		for (ports=1 ; ports<=NumOfPorts ; ports++) {
			if (ports != PcPort)	SwapUartPins(GetUart(ports), NORMAL);
		}
	}
	
			
	/* >>> Step 6 - Test new port directions by pinging all modules */
	
	if (result == BOS_OK) 
	{		
		osDelay(100);
		BOS.response = BOS_RESPONSE_MSG;		// Enable response for pings
		for (i=2 ; i<=N ; i++) 
		{
			while(Ping_count<50)
			{
				SendMessageToModule(i, CODE_PING, 0);
				osDelay(500*NumberOfHops(i));
				Ping_count++;				
				//osDelay(1000);
				if (responseStatus == BOS_OK){
					result = BOS_OK;
					Ping_count=51;
				}
				else if (responseStatus == BOS_ERR_NoResponse){
					result = BOS_ERR_NoResponse;
				}
			}
			Ping_count=0;
		}
	}
	
//	if(result==BOS_ERR_NoResponse)
//	{
//		iteration++;
////		memset(array, 0, sizeof(array) );
////		memset(neighbors, 0, sizeof(neighbors2) );
////		memset(neighbors2, 0, sizeof(array) );
//		goto again;
//	}
	
	
	/* >>> Step 7 - Save all (topology and port directions) in RO/EEPROM */
	
//	if (result == BOS_OK)
//	{
//		/* Save data in the master */
//		SaveToRO();
//		SaveEEportsDir();
//		osDelay(100);
//		/* Ask other modules to save their data too */
//		SendMessageToModule(BOS_BROADCAST, CODE_EXP_EEPROM, 0);
//	}	

	return result;
}
#endif
/*-----------------------------------------------------------*/
#ifndef _N
/* --- Explore adjacent neighbors 
*/
BOS_Status ExploreNeighbors(uint8_t ignore)
{
	BOS_Status result = BOS_OK; 

	/* Send Hi messages to adjacent neighbors */
	for (uint8_t port=1 ; port<=NumOfPorts ; port++)  
	{
		if (port != ignore) 
		{
			/* This module info */
			messageParams[0] = (uint8_t) (myPN >> 8);
			messageParams[1] = (uint8_t) myPN;
			messageParams[2] = port;
			/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
			SendMessageFromPort(port, 0, 0, CODE_HI, 3);
			/* Minimum delay between two consequetive SendMessage commands (with response) */
			osDelay(10);
		}
	}
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Find array broadcast routes starting from a given module 
				(Takes about 50 usec)
*/
BOS_Status FindBroadcastRoutes(uint8_t src)
{
	BOS_Status result = BOS_OK; 
	uint8_t p = 0, m = 0, level = 0, untaged = 0; 
	uint8_t  modules[N];			// Todo: Optimize to make bit-wise
	
	/* 1. Initialize modules list and broadcast routes */
	
	for(m=0 ; m<N ; m++)
	{	
		modules[m] = 0;
		bcastRoutes[m] = 0;
	}
	modules[src-1] = ++level;					// Tag the source
	
	/* 2. Source module should send to all neighbors */
	
	++level;													// Move one level
	
	for(p=1 ; p<=NumOfPorts ; p++)
	{
		if (array[src-1][p]) 
		{
			bcastRoutes[src-1] |= (0x01 << (p-1));
			modules[(array[src-1][p] >> 3)-1] = level;			// Tag this module as already broadcasted-to 
		}
	}
	
	/* 3. Starting from source neighbors, check all other modules we haven't broadcasted-to yet, one by one */
	
	do
	{	
		untaged = 0;																			// Reset the untaged counter
		++level;																					// Move one level
		
		for(m=0 ; m<N ; m++)															// Scan all modules in the list
		{
			if (modules[m] == (level-1))										// This module is already broadcasted-to from the previous level 
			{			
				for(p=1 ; p<=NumOfPorts ; p++)								// Check all neighbors if they're not already broadcasted-to
				{
					if (array[m][p] && (modules[(array[m][p] >> 3)-1] == 0)) 			// Found an untaged module
					{
						bcastRoutes[m] |= (0x01 << (p-1));
						modules[(array[m][p] >> 3)-1] = level;		// Tag this module as already broadcasted-to 
						++untaged;
					}
				}			
			}
		}
	} 
	while (untaged);

	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
*/
void StartMicroDelay(uint16_t Delay)
{
	uint32_t t0=0;

	portENTER_CRITICAL();
	
	if (Delay)
	{
		t0 = htim14.Instance->CNT;

		while(htim14.Instance->CNT - t0 <= Delay) {};
	}
	
	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Load and start milli-second delay counter --- 
*/
void StartMilliDelay(uint16_t Delay)
{
	uint32_t t0=0;
	
	portENTER_CRITICAL();
	
	if (Delay)
	{
		t0 = htim15.Instance->CNT;

		while(htim15.Instance->CNT - t0 <= Delay) {};
	}
	
	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Swap UART pins ( NORMAL | REVERSED )--- 
*/
void SwapUartPins(UART_HandleTypeDef *huart, uint8_t direction)
{
	if (huart != NULL) {
		if (direction == REVERSED) {
			arrayPortsDir[myID-1] |= (0x8000>>(GetPort(huart)-1));		/* Set bit to one */
			huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
			huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
			HAL_UART_Init(huart);
		} else if (direction == NORMAL) {
			arrayPortsDir[myID-1] &= (~(0x8000>>(GetPort(huart)-1)));		/* Set bit to zero */
			huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
			huart->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_DISABLE;
			HAL_UART_Init(huart);		
		}
	}
}

/*-----------------------------------------------------------*/

/* --- Find the shortest route to a module using Dijkstra's algorithm ---
 
Algorithm (from Wikipedia):

1- Assign to every node a tentative distance value: set it to zero for our initial node 
and to infinity for all other nodes.

2- Set the initial node as current. Mark all other nodes unvisited. Create a set of all 
the unvisited nodes called the unvisited set.

3- For the current node, consider all of its unvisited neighbors and calculate their tentative
distances. Compare the newly calculated tentative distance to the current assigned value and 
assign the smaller one. For example, if the current node A is marked with a distance of 6, 
and the edge connecting it with a neighbor B has length 2, then the distance to B (through A) 
will be 6 + 2 = 8. If B was previously marked with a distance greater than 8 then change it to 8. 
Otherwise, keep the current value.

4- When we are done considering all of the neighbors of the current node, mark the current 
node as visited and remove it from the unvisited set. A visited node will never be checked again.

5- If the destination node has been marked visited (when planning a route between two specific 
nodes) or if the smallest tentative distance among the nodes in the unvisited set is infinity 
(when planning a complete traversal; occurs when there is no connection between the initial 
node and remaining unvisited nodes), then stop. The algorithm has finished.

6- Otherwise, select the unvisited node that is marked with the smallest tentative distance, 
set it as the new "current node", and go back to step 3.

 */
uint8_t FindRoute(uint8_t sourceID, uint8_t desID)
{
#ifdef _N
	uint8_t Q[_N] = {0};		// All nodes initially in Q (unvisited nodes)
#else
	uint8_t Q[50] = {0};		// All nodes initially in Q (unvisited nodes)
#endif
	
	uint8_t alt = 0; uint8_t u = 0; uint8_t v = 0; uint8_t j = 0;
	
	memset(route,0,sizeof(route));
	routeDist[sourceID-1] = 0;                  // Distance from source to source
	routePrev[sourceID-1] = 0;               		// Previous node in optimal path initialization undefined
		
	/* Check adjacent neighbors first! */
	for(int col=1 ; col<=6 ; col++)
	{
		if (array[sourceID-1][col] && ((array[sourceID-1][col]>>3) == desID)) {
			routeDist[desID-1] = 1;
			route[0] = desID;
			return col;	
		}
	}						
	
	/* Initialization */
	for (int i=1 ; i<=N ; i++)   					
	{
		if (i != sourceID)            				// Where i has not yet been removed from Q (unvisited nodes)
		{
			routeDist[i-1] = 0xFF;        			// Unknown distance function from source to i
			routePrev[i-1] = 0;            			// Previous node in optimal path from source
		}                    			
	}
	
	/* Algorithm */
	while (!QnotEmpty(Q))
	{				
		u = minArr(routeDist, Q)+1;						// Source node in first case
		if (u == desID) 
		{
			goto finishedRoute;
		}
		else
			Q[u-1] = 1;													// Remove u from Q 
																								
		/* For each neighbor v where v is still in Q. */
		for (uint8_t n=1 ; n<=6 ; n++)      		// Check all module ports
		{
			if (array[u-1][n])										// There's a neighbor v at this port n
			{	
				v = (array[u-1][n]>>3);
				if (!Q[v-1])												// v is still in Q
				{
					alt = routeDist[u-1] + 1;					// Add one hop
					if (alt < routeDist[v-1])      		// A shorter path to v has been found
					{
						routeDist[v-1] = alt; 
						routePrev[v-1] = u; 
					}
				}
			}
		}
	}	
		
finishedRoute:
		
	/* Build the virtual route */	
	while (routePrev[u-1])        		// Construct the shortest path with a stack route
	{
		route[j++] = u;          				// Push the vertex onto the stack
		u = routePrev[u-1];           	// Traverse from target to source
	}
	
	/* Check which port leads to the correct module */
	for(int col=1 ; col<=6 ; col++)	
	{					
		if ( array[sourceID-1][col] && ((array[sourceID-1][col]>>3) == route[routeDist[desID-1]-1]) ) {
			return col;	
		}
	}	

	return 0;			
}

/*-----------------------------------------------------------*/

/* --- Display array topology in human-readable format through module port --- 
*/
void DisplayTopology(uint8_t port)
{
	/* Print table header */
	sprintf(pcUserMessage, "\n\r(Module:Port)\t\t");
	writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
	for (uint8_t i=1 ; i<=NumOfPorts ; i++) 
	{
		sprintf(pcUserMessage, "P%d\t", i);
		writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
	}
	writePxMutex(port, "\n\n\r", 3, cmd50ms, HAL_MAX_DELAY);
	
	/* Print each row */
	for(uint8_t row=0 ; row<N ; row++)
	{
		sprintf(pcUserMessage, "Module %d:\t",row+1);
		writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
		/* Module PN */
		strncpy(pcUserMessage, modulePNstring[(array[row][0])], 5);
		writePxMutex(port, pcUserMessage, 5, cmd50ms, HAL_MAX_DELAY);
		writePxMutex(port, "\t", 1, cmd50ms, HAL_MAX_DELAY);
		/* Connections */
		for(uint8_t col=1 ; col<=NumOfPorts ; col++)
		{
			if (!array[row][col])
				sprintf(pcUserMessage, "%d\t",0);
			else
				sprintf(pcUserMessage, "%d:%d\t", (array[row][col]>>3), (array[row][col]&0x07) );
			writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);			
		}
		writePxMutex(port, "\n\r", 2, cmd50ms, HAL_MAX_DELAY);
	}
	
	writePxMutex(port, "\n", 1, cmd50ms, HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/

/* --- Display ports directions in human-readable format through module port --- 
*/
void DisplayPortsDir(uint8_t port)
{
	sprintf(pcUserMessage, "\n\rThese ports are reversed:");
	writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
	
	for (uint8_t i=1 ; i<=N ; i++) 
	{
		for (uint8_t p=1 ; p<=MaxNumOfPorts ; p++) 
		{		
			if ( (arrayPortsDir[i-1] & (0x8000>>(p-1))) ) 			/* Port is reversed */
			{
				sprintf(pcUserMessage, "\n\rModule %d : P%d", i, p);
				writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
			}	
		}
	}
	
	sprintf(pcUserMessage, "\n\n\rAll other ports are normal\n\r");
	writePxMutex(port, pcUserMessage, strlen(pcUserMessage), cmd50ms, HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/

/* --- Display a description of current module status (Firmware, Ports, P2P DMAs) --- 
*/
void DisplayModuleStatus(uint8_t port)
{
	int8_t *pcOutputString;
	uint16_t temp = 0;
	
	/* Obtain the address of the output buffer. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();
	
	strcpy( (char *) pcOutputString, "");
	
	sprintf(pcUserMessage, "\n\r*** Module %d Status ***\n", myID);
	strcat( (char *) pcOutputString, pcUserMessage);
	sprintf(pcUserMessage, "\n\rConnected via port: P%d\n\r", PcPort);
	strcat( (char *) pcOutputString, pcUserMessage);
	
	/* Firmware */
	sprintf(pcUserMessage, "\n\rFirmware version: %d.%d.%d", _firmMajor, _firmMinor, _firmPatch);
	strcat( (char *) pcOutputString, pcUserMessage);
	sprintf(pcUserMessage, "\n\rFirmware date:    %s", _firmDate);
	strcat( (char *) pcOutputString, pcUserMessage);
	sprintf(pcUserMessage, "\n\rFirmware time:    %s\n\r", _firmTime);
	strcat( (char *) pcOutputString, pcUserMessage);	
	
	/* Ports */
	sprintf(pcUserMessage, "\n\rPorts Status:\n\n\r");
	strcat( (char *) pcOutputString, pcUserMessage);
	for(uint8_t i=1 ; i<=NumOfPorts ; i++)
	{
		sprintf(pcUserMessage, "P%d: ", i);
		strcat( (char *) pcOutputString, pcUserMessage);
		switch (portStatus[i])
		{
				case FREE : 
						sprintf(pcUserMessage, "Free\n\r"); break;
				case MSG :
						sprintf(pcUserMessage, "Receiving messages\n\r"); break;
				case STREAM :
						sprintf(pcUserMessage, "Streaming\n\r"); break;
				case CLI :
						sprintf(pcUserMessage, "Receiving user commands\n\r"); break;
				case PORTBUTTON :
						sprintf(pcUserMessage, "Connected to a button/switch\n\r"); break;
				default:
						break;
		}		
		strcat( (char *) pcOutputString, pcUserMessage);
	}	

	/* P2P DMAs */
	sprintf(pcUserMessage, "\n\rDMA Streams Status:\n\r");
	strcat( (char *) pcOutputString, pcUserMessage);	
	for (char i=1 ; i<=6 ; i++) {
		if (streamDMA[i-1].Instance == 0) {
				sprintf(pcUserMessage, "\n\rStreaming DMA %d is free", i);
				strcat( (char *) pcOutputString, pcUserMessage);
		} else {
				sprintf(pcUserMessage, "\n\rStreaming DMA %d is streaming from P%d to P%d", i, GetPort(streamDMA[i-1].Parent), GetPort(dmaStreamDst[i-1]));
				strcat( (char *) pcOutputString, pcUserMessage);
		}
	}
	strcat( (char *) pcOutputString, "\n\r");
	
	/* Ports direction */
	strcat( (char *) pcOutputString, "\n\rThese ports are reversed: ");
	temp = strlen( (char *) pcOutputString);
	for (uint8_t p=1 ; p<=NumOfPorts ; p++) 
	{		
		if ( (arrayPortsDir[myID-1] & (0x8000>>(p-1))) ) 			/* Port is reversed */
		{
			sprintf(pcUserMessage, "P%d ", p);
			strcat( (char *) pcOutputString, pcUserMessage);
		}	
	}
	if (temp == strlen( (char *) pcOutputString)) {				/* All ports are normal */
		strcat( (char *) pcOutputString, "None");
	}
	strcat( (char *) pcOutputString, "\n\r");
	
	/* Display output */
	if (port)
		writePxMutex(port, (char *) pcOutputString, strlen( (char *) pcOutputString), cmd50ms, HAL_MAX_DELAY);
	
}

/*-----------------------------------------------------------*/

/* --- Extract module ID from it's alias, ID string or keyword --- 
*/
int16_t GetID(char* string)
{
	uint8_t id = 0, i = 0;
	
	if(!strcmp(string, "me"))							/* Check keywords */
		return myID;
	else if(!strcmp(string, "all"))							
		return BOS_BROADCAST;				
	else if (string[0] == '#') 						/* Check IDs */
	{					
		id = atol(string+1);
		if (id > 0 && id <= N)
			return id;
		else if (id == myID)
			return myID;
		else
			return BOS_ERR_WrongID;				
	} 
	else 																	/* Check alias */
	{															
		/* Check module alias */
		for (i=0 ; i<N ; i++) {
			if(!strcmp(string, moduleAlias[i]) && (*string != 0))	return (i);	
		}
		
		/* Check group alias */
		for(i=0 ; i<MaxNumOfGroups ; i++) {
			if (!strcmp(string, groupAlias[i]))	return (BOS_MULTICAST|(i<<8));
		}			
		
		return BOS_ERR_WrongName;			
	}
	
}

/*-----------------------------------------------------------*/

/* --- Name a module with an alias --- 
*/
BOS_Status NameModule(uint8_t module, char* alias)
{
	BOS_Status result = BOS_OK; int i = 0;
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;

	/* 1. Check module alias with keywords */
	for(i=0 ; i<NumOfKeywords ; i++)
	{
		if (!strcmp(alias, BOSkeywords[i]))	
			return BOS_ERR_Keyword;
	}
	
	/* 2. Check module alias with other module aliases */
	for(i=1 ; i<N ; i++)
	{
		if (!strcmp(alias, moduleAlias[i]))	
			return BOS_ERR_ExistingAlias;
	}

	/* 3. Check module alias with group aliases */
	for(i=0 ; i<MaxNumOfGroups ; i++)
	{
		if (!strcmp(alias, groupAlias[i]))	
			return BOS_ERR_ExistingAlias;
	}
	
	/* 4. Check alias with BOS and module commands */
	for( pxCommand = &xRegisteredCommands; pxCommand != NULL; pxCommand = pxCommand->pxNext )
	{
		pcRegisteredCommandString = pxCommand->pxCommandLineDefinition->pcCommand;
		xCommandStringLength = strlen( ( const char * ) pcRegisteredCommandString );
		
		if( !strncmp(alias, (const char *) pcRegisteredCommandString, xCommandStringLength ) ) {
			return BOS_ERR_ExistingCmd;
		}
	}
	
	/* 5. Module alias is unique */
	strcpy(moduleAlias[module], alias);
	
	/* 6. Share new module alias with other modules */
	
	
	/* 7. Save new alias to emulated EEPROM */
	result = SaveEEalias();
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Add a module to this group --- 
*/
BOS_Status AddModuleToGroup(uint8_t module, char* group)
{
	BOS_Status result = BOS_OK; int i = 0, j = 0;
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;

	/* Check alias with other group aliases */
	
	for(i=0 ; i<MaxNumOfGroups ; i++)
	{
		/* This group already exists */
		if (!strcmp(group, groupAlias[i]))	
		{
			/* 1. Add this module to the group */
			groupModules[module-1] |= (0x0001<<i);	

			/* 2. Save group to emulated EEPROM -- Should call this manually */
			//result = SaveEEgroup();			
			
			return result;
		}
	}
	
	/* This is a new group - Verify alias and create the group */
	
	/* 1. Check group alias with keywords */
	for(j=0 ; j<NumOfKeywords ; j++)
	{
		if (!strcmp(group, BOSkeywords[j]))	
			return BOS_ERR_Keyword;
	}	

	/* 2. Check group alias with module aliases */
	for(j=1 ; j<N ; j++)
	{
		if (!strcmp(group, moduleAlias[j]))	
			return BOS_ERR_ExistingAlias;
	}		
	
	/* 3. Check group alias with BOS and module commands */
	for( pxCommand = &xRegisteredCommands; pxCommand != NULL; pxCommand = pxCommand->pxNext )
	{
		pcRegisteredCommandString = pxCommand->pxCommandLineDefinition->pcCommand;
		xCommandStringLength = strlen( ( const char * ) pcRegisteredCommandString );
		
		if( !strncmp(group, (const char *) pcRegisteredCommandString, xCommandStringLength ) ) {
			return BOS_ERR_ExistingCmd;
		}
	}			
	
	/* 4. Group alias is unique - copy to first empty location */
	for(i=0 ; i<MaxNumOfGroups ; i++)
	{
		if (!groupAlias[i][0]) {	
			strcpy(groupAlias[i], group);	
			break;
		}
	}		
	
	/* 5. Add this module to the new group */
	groupModules[module-1] |= (0x0001<<i);
	
	/* 6. Share new group with other modules */


	/* 7. Save new group to emulated EEPROM - Should call this manually */
	//result = SaveEEgroup();			
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Read Ports directions when a pre-defined topology file is used --- 
*/
BOS_Status ReadPortsDir(void)
{
	BOS_Status result = BOS_OK; 
	
	/* Ask all other modules for their ports directions */
	for (uint8_t i=1 ; i<=N ; i++) 
	{
		if (i != myID) {
			SendMessageToModule(i, CODE_READ_PORT_DIR, 0);
			Delay_ms_no_rtos(50);
			if (responseStatus != BOS_OK)	{
				result = BOS_ERR_NoResponse;
			} 	
		} else {
			/* Check my own ports */
			for (uint8_t p=1 ; p<=NumOfPorts ; p++) {
				if (GetUart(p)->AdvancedInit.Swap == UART_ADVFEATURE_SWAP_ENABLE) {
					arrayPortsDir[myID-1] |= (0x8000>>(p-1));		/* Set bit to 1 */
				}									
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/
#ifndef _N
/* --- Update module port directions based on what is stored in eeprom --- 
*/
BOS_Status UpdateMyPortsDir(void)
{
	BOS_Status result = BOS_OK;
	
	/* Check port direction */
	for (uint8_t p=1 ; p<=NumOfPorts ; p++) 
	{
		if ( !(arrayPortsDir[myID-1] & (0x8000>>(p-1))) ) {
			/* Port is normal */
			SwapUartPins(GetUart(p), NORMAL);
		} else {
			/* Port is reversed */
			SwapUartPins(GetUart(p), REVERSED);					
		}	
	}		
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Start a single-cast DMA stream across the array. Transfer ends after (count) bytes are transferred 
			or timeout (ms), whichever comes first. If stored = true, the stream is stored in emulated eeprom --- 
*/
BOS_Status StartScastDMAStream(uint8_t srcP, uint8_t srcM, uint8_t dstP, uint8_t dstM, uint8_t direction, uint32_t count, uint32_t timeout, bool stored)
{
	BOS_Status result = BOS_OK;
	uint8_t port = 0, temp1 = 0, temp2 = 0;
	
	/* Is the source a different module? */
	if (srcM != myID) {
		/* Forward this task to the source module */
		messageParams[0] = (uint8_t) (count >> 24);			/* Count */
		messageParams[1] = (uint8_t) (count >> 16);
		messageParams[2] = (uint8_t) (count >> 8);
		messageParams[3] = (uint8_t) count;
		messageParams[4] = (uint8_t) (timeout >> 24);		/* Timeout */
		messageParams[5] = (uint8_t) (timeout >> 16);
		messageParams[6] = (uint8_t) (timeout >> 8);
		messageParams[7] = (uint8_t) timeout;
		messageParams[8] = direction;										/* Stream direction */
		messageParams[9] = srcP;												/* Source port */
		messageParams[10] = dstM;												/* destination module */
		messageParams[11] = dstP;												/* destination port */
		messageParams[12] = stored;											/* EEPROM storage */
		SendMessageToModule(srcM, CODE_DMA_SCAST_STREAM, 13);		
		
		return result;
	}
	
	/* Inform participating modules */
	for(uint8_t i=0 ; i<sizeof(route) ; i++)
	{
		FindRoute(srcM, dstM);
		/* Message other modules */
		if (route[i]) 
		{
			/* Find out the inport and outport to this module from previous one */
			if (route[i+1]) {
				temp1 = FindRoute(route[i], route[i+1]);
			} else {
				temp1 = FindRoute(route[i], srcM);
			}
			FindRoute(srcM, dstM);
			if (route[i] == dstM) {
				temp2 = dstP;
			} else {
				temp2 = FindRoute(route[i], route[i-1]);
			}
			/* Message parameters*/
			messageParams[0] = (uint8_t) (count >> 24);			/* Count */
			messageParams[1] = (uint8_t) (count >> 16);
			messageParams[2] = (uint8_t) (count >> 8);
			messageParams[3] = (uint8_t) count;
			messageParams[4] = (uint8_t) (timeout >> 24);		/* Timeout */
			messageParams[5] = (uint8_t) (timeout >> 16);
			messageParams[6] = (uint8_t) (timeout >> 8);
			messageParams[7] = (uint8_t) timeout;
			messageParams[8] = direction;										/* Stream direction */
			messageParams[9] = temp1;												/* Source port */
			messageParams[10] = temp2;											/* destination port */
			messageParams[11] = stored;											/* EEPROM storage */
			FindRoute(srcM, dstM);
			SendMessageToModule(route[i], CODE_DMA_CHANNEL, 12);
			osDelay(10);
		}
	}
	
	if (srcM == dstM)
		port = dstP;
	else
		port = FindRoute(srcM, dstM);
	
	/* Setup my own DMA stream */
	SetupDMAStreams(direction, count, timeout, srcP, port);
	
	// Store my own streams to EEPROM
	if (stored) {		
		SaveEEstreams(direction, count, timeout, srcP, port, 0, 0, 0, 0);
	}
	
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Define a new button attached to one of array ports
					buttonType: MOMENTARY_NO, MOMENTARY_NC, ONOFF_NO, ONOFF_NC
					port: array port (P1 - Px)
*/
BOS_Status AddPortButton(uint8_t buttonType, uint8_t port)
{
	BOS_Status result = BOS_OK;
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t TX_Port, RX_Port; 
	uint16_t TX_Pin, RX_Pin, temp16, res;
	uint8_t temp8 = 0;
	
	/* 1. Stop communication at this port (only if the scheduler is running) - TODO update*/
	if (BOS_initialized) {
		osSemaphoreRelease(PxRxSemaphoreHandle[port]);		/* Give back the semaphore if it was taken */
		osSemaphoreRelease(PxTxSemaphoreHandle[port]);
	}
	portStatus[port] = PORTBUTTON;	
	
	/* 2. Deinitialize UART (only if module is initialized) */
	if (BOS_initialized) {
		HAL_UART_DeInit(GetUart(port));
	}
	
	/* 3. Initialize GPIOs */
	GetPortGPIOs(port, &TX_Port, &TX_Pin, &RX_Port, &RX_Pin);		
	/* Ouput (TXD) */
	GPIO_InitStruct.Pin = TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init((GPIO_TypeDef *)TX_Port, &GPIO_InitStruct);
	/* Input (RXD) */
	GPIO_InitStruct.Pin = RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init((GPIO_TypeDef *)RX_Port, &GPIO_InitStruct);

	/* 4. Update button struct */
	button[port].type = buttonType;	
	
	/* 5. Add to EEPROM if not already there */
	res = EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1), &temp16);
	if(!res)																														// This variable exists
	{
		temp8 = (uint8_t)(temp16 >> 8);
		if ( ((temp8 >> 4) == port) && ((temp8 & 0x0F) == buttonType) )		// This is same port and same type, do not update
			return BOS_OK;
		else 																															// Update the variable
		{																														
			temp16 = ((uint16_t)port << 12) | ((uint16_t)buttonType << 8);
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), temp16);
			/* Reset times */
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, 0);
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, 0);
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, 0);
		}
	}
	else																																// Variable does not exist. Create a new one
	{
		temp16 = ((uint16_t)port << 12) | ((uint16_t)buttonType << 8);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), temp16);		
		/* Reset times */
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, 0);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, 0);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, 0);
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Undefine a button attached to one of array ports and restore the port to default state
					port: array port (P1 - Px)
*/
BOS_Status RemovePortButton(uint8_t port)
{
	BOS_Status result = BOS_OK;
	uint16_t res, temp16;
	
	/* 1. Remove from button struct */
	button[port].type = NONE;
	button[port].state = NONE;
	button[port].events = 0;
	button[port].pressedX1Sec = 0; button[port].pressedX2Sec = 0; button[port].pressedX3Sec = 0;
	button[port].releasedY1Sec = 0; button[port].releasedY2Sec = 0; button[port].releasedY3Sec = 0;
	
	/* 2. Remove from EEPROM if it's already there */
	res = EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1), &temp16);
	if(!res)																														// This variable exists, reset all to zeros
	{
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), 0);
		/* Reset times */
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, 0);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, 0);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, 0);		
	}
	
	/* 3. Initialize UART at this port */
	UART_HandleTypeDef* huart = GetUart(port);
	
	if (huart->Instance == USART1) 
	{	
#ifdef _Usart1		
		MX_USART1_UART_Init();
#endif
	} 
	else if (huart->Instance == USART2) 
	{	
#ifdef _Usart2	
		MX_USART2_UART_Init();
#endif
	} 
	else if (huart->Instance == USART3) 
	{	
#ifdef _Usart3	
		MX_USART3_UART_Init();
#endif
	} 
	else if (huart->Instance == USART4) 
	{	
#ifdef _Usart4	
		MX_USART4_UART_Init();
#endif
	} 
	else if (huart->Instance == USART5) 
	{	
#ifdef _Usart5	
		MX_USART5_UART_Init();
#endif
	} 
	else if (huart->Instance == USART6) 
	{	
#ifdef _Usart6	
		MX_USART6_UART_Init();
#endif
	} 
	else if (huart->Instance == USART7) 
	{	
#ifdef _Usart7	
		MX_USART7_UART_Init();
#endif
	} 
	else if (huart->Instance == USART8) 
	{	
#ifdef _Usart8	
		MX_USART8_UART_Init();
#endif
	} 
	else
		result = BOS_ERROR;			
	
	/* 4. Start scanning this port */
	portStatus[port] = FREE;
	/* Read this port again */
	HAL_UART_Receive_IT(huart, (uint8_t *)&cRxedChar, 1);	
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Setup button events and callbacks
					port: array port (P1 - Px) where the button is attached 
					clicked: Single click event (1: Enable, 0: Disable)
					dbl_clicked: Double click event (1: Enable, 0: Disable)
					pressed_x1sec, pressed_x1sec, pressed_x1sec: Press time for events X1, X2 and X3 in seconds. Use 0 to disable the event. 
					released_x1sec, released_x1sec, released_x1sec: Release time for events Y1, Y2 and Y3 in seconds. Use 0 to disable the event. 
					mode: BUTTON_EVENT_MODE_CLEAR to clear events marked with 0, BUTTON_EVENT_MODE_OR to OR events marked with 1 with existing events.
*/
BOS_Status SetButtonEvents(uint8_t port, uint8_t clicked, uint8_t dbl_clicked, uint8_t pressed_x1sec, uint8_t pressed_x2sec, uint8_t pressed_x3sec,\
													uint8_t released_y1sec, uint8_t released_y2sec, uint8_t released_y3sec, uint8_t mode)
{
	BOS_Status result = BOS_OK;	
	uint16_t res, temp16; uint8_t temp8;
	
	if (button[port].type == NONE)
		return BOS_ERR_BUTTON_NOT_DEFINED;
	
	button[port].pressedX1Sec = pressed_x1sec; button[port].pressedX2Sec = pressed_x2sec; button[port].pressedX3Sec = pressed_x3sec;
	button[port].releasedY1Sec = released_y1sec; button[port].releasedY2Sec = released_y2sec; button[port].releasedY3Sec = released_y3sec;
	
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && clicked)) {				
		button[port].events |= BUTTON_EVENT_CLICKED;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !clicked) {
		button[port].events &= ~BUTTON_EVENT_CLICKED;		
	}
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && dbl_clicked)) {		
		button[port].events |= BUTTON_EVENT_DBL_CLICKED;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !dbl_clicked) {
		button[port].events &= ~BUTTON_EVENT_DBL_CLICKED;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && pressed_x1sec)) {			
		button[port].events |= BUTTON_EVENT_PRESSED_FOR_X1_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !pressed_x1sec) {
		button[port].events &= ~BUTTON_EVENT_PRESSED_FOR_X1_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && pressed_x2sec)) {		
		button[port].events |= BUTTON_EVENT_PRESSED_FOR_X2_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !pressed_x2sec) {
		button[port].events &= ~BUTTON_EVENT_PRESSED_FOR_X2_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && pressed_x3sec)) {		
		button[port].events |= BUTTON_EVENT_PRESSED_FOR_X3_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !pressed_x3sec) {
		button[port].events &= ~BUTTON_EVENT_PRESSED_FOR_X3_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && released_y1sec)) {		
		button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y1_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !released_y1sec) {
		button[port].events &= ~BUTTON_EVENT_RELEASED_FOR_Y1_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && released_y2sec)) {		
		button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y2_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !released_y2sec) {
		button[port].events &= ~BUTTON_EVENT_RELEASED_FOR_Y2_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && released_y3sec)) {		
		button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y3_SEC;	
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !released_y3sec) {
		button[port].events &= ~BUTTON_EVENT_RELEASED_FOR_Y3_SEC;		
	}
	
	/* Add to EEPROM */
	res = EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1), &temp16);
	if(!res)																														// This variable exists
	{
		temp8 = (uint8_t)(temp16 >> 8);																		// Keep upper byte
		/* Store event flags */
		if ((uint8_t)(temp16) != button[port].events) {										// Update only if different
			temp16 = ((uint16_t)temp8 << 8) | (uint16_t)button[port].events;
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), temp16);
		}
		
		/* Store times - only if different */
		EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1)+1, &temp16);
		if ( temp16 != (((uint16_t)pressed_x1sec << 8) | (uint16_t) released_y1sec) )
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, ((uint16_t)pressed_x1sec << 8) | (uint16_t) released_y1sec);
		
		EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1)+2, &temp16);
		if ( temp16 != (((uint16_t)pressed_x2sec << 8) | (uint16_t) released_y2sec) )
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, ((uint16_t)pressed_x2sec << 8) | (uint16_t) released_y2sec);
		
		EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1)+3, &temp16);
		if ( temp16 != (((uint16_t)pressed_x3sec << 8) | (uint16_t) released_y3sec) )
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, ((uint16_t)pressed_x3sec << 8) | (uint16_t) released_y3sec);
	}	// TODO - var does not exist after adding button!
	else																																// Variable does not exist. Return error
		return BOS_ERR_BUTTON_NOT_DEFINED;	
		
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Read a variable from a remote module. 
			 This API returns a pointer to the remote value. Cast this pointer to match the appropriate format.
			 If the returned value is NULL, then remote variable does not exist or remote module is not responsive.
					module: Remote module ID. 
					remoteAddress: Remote value memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to read BOS variables with unknown addresses.
					remoteFormat (output): Pointer to format of remote BOS variable. 					
					timeout: Read timeout in msec.
*/
uint32_t *ReadRemoteVar(uint8_t module, uint32_t remoteAddress, varFormat_t *remoteFormat, uint32_t timeout)
{
	/* Reset local buffer */
	remoteBuffer = REMOTE_BOS_VAR;
	
	/* Send the Message */
	messageParams[0] = remoteAddress + REMOTE_BOS_VAR;			// Send BOS variable index
	SendMessageToModule(module, CODE_READ_REMOTE, 1);
	
	/* Wait until read is complete */
	uint32_t t0 = HAL_GetTick();
	while ( (responseStatus != BOS_OK) && ((HAL_GetTick()-t0) < timeout) ) { };
	
	/* Return the read value address */
	if (responseStatus == BOS_OK) 
	{
		/* Return the remote var format */
		*remoteFormat = remoteVarFormat;
		
		return ((uint32_t *)&remoteBuffer);		
	}
	else 
		return NULL;
}

/*-----------------------------------------------------------*/

/* --- Read a memory address from a remote module. 
			 This API returns a pointer to the remote value. Cast this pointer to match the appropriate format.
			 If the returned value is NULL, then remote variable does not exist or remote module is not responsive.
					module: Remote module ID. 
					remoteAddress: Remote value memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to read BOS variables.
					requestedFormat (input): Requested format of remote memory location (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
					timeout: Read timeout in msec.
*/
uint32_t *ReadRemoteMemory(uint8_t module, uint32_t remoteAddress, varFormat_t requestedFormat, uint32_t timeout)
{
	/* Reset local buffer */
	remoteBuffer = REMOTE_MEMORY_ADD;
	
	/* Send the Message */
	messageParams[0] = REMOTE_MEMORY_ADD;
	messageParams[1] = requestedFormat;						// Requested format
	messageParams[2] = (uint8_t)(remoteAddress>>24); messageParams[3] = (uint8_t)(remoteAddress>>16); // Remote address
	messageParams[4] = (uint8_t)(remoteAddress>>8); messageParams[5] = (uint8_t)remoteAddress; 		
	SendMessageToModule(module, CODE_READ_REMOTE, 6);
	remoteBuffer = requestedFormat;								// Set a flag that we requested a memory location
	
	/* Wait until read is complete */
	uint32_t t0 = HAL_GetTick();
	while ( (responseStatus != BOS_OK) && ((HAL_GetTick()-t0) < timeout) ) { };
	
	/* Return the read value address */
	if (responseStatus == BOS_OK)
		return ((uint32_t *)&remoteBuffer);	
	else 
		return NULL;
}

/*-----------------------------------------------------------*/

/* --- Read a parameter from a remote module. 
			 This API returns a pointer to the remote parameter. Cast this pointer to match the appropriate format.
			 If the returned parameter is NULL, then remote parameter does not exist or remote module is not responsive.
					module: Remote module ID. 
					paramString: Remote parameter string address (RAM or Flash). Use the 1 to MAX_BOS_VARS to read BOS parameters with unknown addresses.
					remoteFormat (output): Pointer to format of remote BOS variable. 					
					timeout: Read timeout in msec.
*/
uint32_t *ReadRemoteParam(uint8_t module, char* paramString, varFormat_t *remoteFormat, uint32_t timeout)
{
	/* Reset local buffer */
	remoteBuffer = REMOTE_MODULE_PARAM;
	
	/* Send the Message */
	messageParams[0] = REMOTE_MODULE_PARAM;
	memcpy(&messageParams[1], paramString, strlen(paramString));   // copy BOS parameter index to location
	SendMessageToModule(module, CODE_READ_REMOTE, strlen(paramString)+1);
	
	/* Wait until read is complete */
	uint32_t t0 = HAL_GetTick();
	while ( (responseStatus != BOS_OK) && ((HAL_GetTick()-t0) < timeout) ) { };
	
	/* Return the read value address */
	if (responseStatus == BOS_OK) 
	{
		/* Return the remote var format */
		*remoteFormat = remoteVarFormat;
		
		return ((uint32_t *)&remoteBuffer);		
	}
	else 
		return NULL;
}

/*-----------------------------------------------------------*/

/* --- Write a value to a remote module. 
					module: Remote module ID. 
					localAddress: Local memory address (RAM or Flash). 
					remoteAddress: Remote memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
					format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
					timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
*/
BOS_Status WriteRemote(uint8_t module, uint32_t localAddress, uint32_t remoteAddress, varFormat_t format, uint32_t timeout)
{
	return WriteToRemote(module, localAddress, remoteAddress, format, timeout, 0);
}

/*-----------------------------------------------------------*/

/* --- Write a value to a remote module and force full-page erase when writing to Flash. 
					module: Remote module ID. 
					localAddress: Local memory address (RAM or Flash). 
					remoteAddress: Remote memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
					format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
					timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
*/
BOS_Status WriteRemoteForce(uint8_t module, uint32_t localAddress, uint32_t remoteAddress, varFormat_t format, uint32_t timeout)
{
	return WriteToRemote(module, localAddress, remoteAddress, format, timeout, 1);
}

/*-----------------------------------------------------------*/

/* --- Assign an index to a new BOS variable. BOS variables must be global or static to ensure we don't refernce a stack address.
*/
uint8_t AddBOSvar(varFormat_t format, uint32_t address)
{
	for( uint8_t v=0 ; v<MAX_BOS_VARS ; v++)
  {
		if((BOS_var_reg[v]&0x000F) == 0)		// Index not assigned yet
		{
			BOS_var_reg[v] = format + ((address-SRAM_BASE)<<16);
			return (v+1);
		}
  }
	
	return 0;			// Memory full
}

/*-----------------------------------------------------------*/

/* --- BOS internal real-time clock and calendar configuration.
*/
BOS_Status BOS_CalendarConfig(uint8_t month, uint8_t day, uint16_t year, uint8_t weekday, uint8_t seconds, \
															uint8_t minutes, uint8_t hours, uint8_t AMPM, int8_t daylightsaving)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;	
	
  /* Set Date */
  sdatestructure.Year = year-2000;
  sdatestructure.Month = month;
  sdatestructure.Date = day;
  sdatestructure.WeekDay = weekday;		// Todo - Calculate weekday later
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;

  /* Set Time */
  stimestructure.Hours = hours;
  stimestructure.Minutes = minutes;
  stimestructure.Seconds = seconds; 
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;		// Todo - Use this to make sure user does not change daylight settings again
	
//	if (daylightsaving == DAYLIGHT_NONE) 											// Todo
//		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	else if (daylightsaving == DAYLIGHT_ADD1H)
//		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_ADD1H;
//	else if (daylightsaving == DAYLIGHT_SUB1H)
//		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_SUB1H;
	
	if (hours > 12)	BOS.hourformat = 24;
	
	if (AMPM == RTC_AM) {
		stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
		BOS.hourformat = 12;
	} else if (AMPM == RTC_PM) {
		stimestructure.TimeFormat = RTC_HOURFORMAT12_PM;
		BOS.hourformat = 12;
	} else
		BOS.hourformat = 24;
	
  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
		return BOS_ERROR;
	
	/* Save RTC hourformat and daylightsaving to EEPROM */
	EE_WriteVariable(_EE_PARAMS_RTC, ((uint16_t)BOS.hourformat<<8) | (uint16_t)BOS.buttons.minInterClickTime);

  /* Writes a data in a RTC Backup data Register1 */
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2);
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* --- Get current RTC time and date.
*/
void GetTimeDate(void)
{
	RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
	
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
	
	BOS.time.ampm = (stimestructureget.TimeFormat >> 7) + 1;
	BOS.time.msec = stimestructureget.SubSeconds / 2;
	BOS.time.seconds = stimestructureget.Seconds;
	BOS.time.minutes = stimestructureget.Minutes;
	BOS.time.hours = stimestructureget.Hours;
	BOS.date.day = sdatestructureget.Date;
	BOS.date.month = sdatestructureget.Month;
	BOS.date.weekday = sdatestructureget.WeekDay;
	BOS.date.year = sdatestructureget.Year + 2000;
}

/*-----------------------------------------------------------*/

/* --- Make a data string with format weekday / month / date / year 
*/
char *GetDateString(void)
{
  static const char formatDateStr[] = "%s %02d/%02d/%04d";
  char *buffer = malloc(30 * sizeof(int8_t));
  memset (buffer, 0x00, 30 * sizeof(int8_t));
  sprintf(buffer, formatDateStr, weekdayString[BOS.date.weekday-1], BOS.date.month, BOS.date.day, BOS.date.year);
  return buffer;
}

/*-----------------------------------------------------------*/

/* --- Make a time string with format hour / minute / second
*/
char *GetTimeString(void)
{
  static const char formatTimeStr[] = "%02d:%02d:%02d";
  char *buffer = malloc(10 * sizeof(int8_t));
  memset (buffer, 0x00, 10 * sizeof(int8_t));
  sprintf(buffer, formatTimeStr, BOS.time.hours, BOS.time.minutes, BOS.time.seconds);
  return buffer;
}

/*-----------------------------------------------------------*/

/* --- Bridge two array/communication ports together
*/
BOS_Status Bridge(uint8_t port1, uint8_t port2)
{
	// Link the ports together with an infinite DMA stream
	return StartScastDMAStream(port1, myID, port2, myID, BIDIRECTIONAL, 0xFFFFFFFF, 0xFFFFFFFF, true);
}

/*-----------------------------------------------------------*/

/* --- Un-bridge two array/communication ports
*/
BOS_Status Unbridge(uint8_t port1, uint8_t port2)
{		
	// Remove the stream from EEPROM
	SaveEEstreams(0, 0, 0, 0, 0, 0, 0, 0, 0);
	
	// Stop the DMA streams and enable messaging back on these ports
	if(streamDMA[port1-1].Instance != 0 && streamDMA[port2-1].Instance != 0) 
			{SwitchStreamDMAToMsg(port1);SwitchStreamDMAToMsg(port2);return BOS_OK;}
	else if (streamDMA[port1-1].Instance != 0)
			{SwitchStreamDMAToMsg(port1);return BOS_OK;}
	else if (streamDMA[port2-1].Instance != 0)
			{SwitchStreamDMAToMsg(port2);return BOS_OK;}	
	else {return BOS_ERR_WrongValue;}
}

/*-----------------------------------------------------------*/

/* --- Print formatted text to one of the module ports
*/
BOS_Status printfp(uint8_t port, char* str)
{		
	if (writePxMutex(port, str, strlen(str), 1, 1) == HAL_OK)
		return BOS_OK;
	else 
		return BOS_ERROR;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
const int8_t *const pcTaskTableHeader = ( int8_t * ) "Task          State  Priority  Stack	#\r\n************************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcTaskTableHeader );
	vTaskList( ((char*) pcWriteBuffer) + strlen( ( char * ) pcTaskTableHeader ) );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
const int8_t * const pcStatsTableHeader = ( int8_t * ) "Task            Abs Time      % Time\r\n****************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcStatsTableHeader );
	vTaskGetRunTimeStats( ((char*) pcWriteBuffer) + strlen( ( char * ) pcStatsTableHeader ) );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE pingCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage1 = ( int8_t * ) "Hi from module %d\r\n";
	static const int8_t *pcMessage2 = ( int8_t * ) "Hi from module %d (%s)\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the ping */
	if (!moduleAlias[myID][0])
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage1, myID);
	else
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage2, myID, moduleAlias[myID]);
	
	RTOS_IND_blink(200);	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE bootloaderUpdateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage = ( int8_t * ) "Update firmware for module %d\n\r";
	static const int8_t *pcMessageWrongValue = ( int8_t * ) "Wrong value!\n\r";	
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3; 
	static portBASE_TYPE xParameterStringLength1, xParameterStringLength2, xParameterStringLength3;
	uint8_t module, port; BOS_Status result = BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	
	/* Local update */
	if (pcParameterString1 == NULL)
	{
		/* Respond to the update command */
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, myID);
		strcat( ( char * ) pcWriteBuffer, ( char * ) pcBootloaderUpdateMessage );
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		
		/* Address for RAM signature (STM32F09x) - Last 4 words of SRAM */
		*((unsigned long *)0x20007FF0) = 0xDEADBEEF;   
		indMode = IND_PING;
		osDelay(10);
		NVIC_SystemReset();						
	}	
	else 
	{
		/* This is a 'via port' remote update command */	
		if (!strncmp((const char *)pcParameterString1, "via", xParameterStringLength1)) 
		{
			pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
			pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
			
			/* Parse the module */
			if (pcParameterString2[0] == '#') {
				module = ( uint8_t ) atol( ( char * ) pcParameterString2+1 );
			} 
			else
				result = BOS_ERR_WrongValue;				
			
			/* Parse the port */
			if (pcParameterString3[0] == 'p') {
				port = ( uint8_t ) atol( ( char * ) pcParameterString3+1 );
			} 
			else
				result = BOS_ERR_WrongValue;		
			
			/* I'm the source of the command and the target is > 1 hop away */
			if (module != myID)
			{
				/* Deactivate responses */
				BOS.response = BOS_RESPONSE_NONE;							
				
				/* Forward the command */
				messageParams[0] = port; SendMessageToModule(module, CODE_UPDATE_VIA_PORT, 1);
				osDelay(100);			
				/* Execute locally */
				remoteBootloaderUpdate(myID, module, PcPort, port);	
			}
			/* I'm the source of the command and my neighbor is the target */
			else
			{
				/* Ask the target to jump to factory bootloader */
				SendMessageFromPort(port, 0, 0, CODE_UPDATE, 0);
				osDelay(100);
				/* Then, setup myself for remote 'via port' update */
				remoteBootloaderUpdate(myID, myID, PcPort, port);							
			}		
		}
		else
			result = BOS_ERR_WrongValue;		
	}
	
	/* Respond to user */
	if (result == BOS_ERR_WrongValue) {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongValue );			
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
#ifndef _N
static portBASE_TYPE exploreCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK;
	static const int8_t *pcMessage = ( int8_t * ) "\nThe array is being explored. Please wait...\n\r";
	static const int8_t *pcMessageOK = ( int8_t * ) "\nThe array exploration succeeded. I found %d modules including myself. Here is the discovered topology:\n\r";
	static const int8_t *pcMessageErr = ( int8_t * ) "\nThe array exploration failed. Please double check connections, reset the modules and try again.\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the update command */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessage );
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	
	/* Call array exploration routine */
	result = Explore();
	if (result == BOS_OK) {
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, N);
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		DisplayTopology(PcPort);
		DisplayPortsDir(PcPort);
	} else {
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageErr );
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	}
	sprintf( ( char * ) pcWriteBuffer, " ");
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
#endif

/*-----------------------------------------------------------*/

static portBASE_TYPE resetCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	NVIC_SystemReset();	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE nameCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1; 
	static portBASE_TYPE xParameterStringLength1;
	
	static const int8_t *pcMessageOK = ( int8_t * ) "Module %d is named %s\n\r";
	static const int8_t *pcMessageKey = ( int8_t * ) "%s is a reserved BOS keyword! Please use a different alias\n\r";
	static const int8_t *pcMessageAlias = ( int8_t * ) "%s is already used! Please use a different alias\n\r";
	static const int8_t *pcMessageCmd = ( int8_t * ) "%s is an existing CLI command! Please use a different alias\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

	/* Check alias length */
	if (xParameterStringLength1 > MaxLengthOfAlias) {
		pcParameterString1[MaxLengthOfAlias] = '\0';
	}
	
	/* Name the module */
	result = NameModule(myID, (char*) pcParameterString1);
		
	/* Respond to the update command */
	if (result == BOS_OK)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, myID, pcParameterString1);
	else if (result == BOS_ERR_Keyword)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageKey, pcParameterString1);
	else if (result == BOS_ERR_ExistingAlias)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageAlias, pcParameterString1);	
	else if (result == BOS_ERR_ExistingCmd)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageCmd, pcParameterString1);	
	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE groupCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1, *pcParameterString, count; 
	static portBASE_TYPE xParameterStringLength1, xParameterStringLength;
	char module[MaxLengthOfAlias+30] = {0}; int16_t modID = 0, type = 0; char alias[MaxLengthOfAlias+1] = {0};
	
	static const int8_t *pcMessageWrongModule = ( int8_t * ) "%s is a wrong module ID or alias\n\r";
	static const int8_t *pcMessageOKnew = ( int8_t * ) "] added to new group %s\n\r";
	static const int8_t *pcMessageOKexist = ( int8_t * ) "] added to existing group %s\n\r";
	static const int8_t *pcMessageKey = ( int8_t * ) "%s is a reserved BOS keyword! Please use a different alias\n\r";
	static const int8_t *pcMessageAlias = ( int8_t * ) "%s is already used! Please use a different alias\n\r";
	static const int8_t *pcMessageCmd = ( int8_t * ) "%s is an existing CLI command! Please use a different alias\n\r";
	static const int8_t *pcMessageNoModules = ( int8_t * ) "Please enter some module IDs to add to %s\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string - Group name */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	strncpy( alias, ( char * ) pcParameterString1, xParameterStringLength1);
	
	/* Is it new or existing group? */
	type = 1;
	for(uint8_t i=0 ; i<MaxNumOfGroups ; i++)
	{
		/* This group already exists */
		if (!strcmp(alias, groupAlias[i]))	
		{
			type = 0; break;
		}
	}	
	
	/* Extract modules and add them to group */
	count = 2;
	strcpy( ( char * ) pcWriteBuffer, "Modules [");
	pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, count, &xParameterStringLength);
	while (pcParameterString != NULL)
	{
		strncpy(module, ( char * ) pcParameterString, xParameterStringLength); module[xParameterStringLength] = '\0';
		modID = GetID(module);
		
		if (modID < 0)	break;
		
		result = AddModuleToGroup(modID, alias);
		
		if (result != BOS_OK)	break;
		
		if (count > 2)
			strcat( ( char * ) pcWriteBuffer, ", "); 
		
		strcat( ( char * ) pcWriteBuffer, module);
		
		/* Extract next module */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, ++count, &xParameterStringLength);	
	}
		
	/* Respond to the update command */
	if (modID < 0) 
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongModule, module);
	else if (count == 2)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageNoModules, alias);
	else if (result == BOS_OK && type) {
		sprintf( module, ( char * ) pcMessageOKnew, alias); 
		strcat( ( char * ) pcWriteBuffer, module);
	} else if (result == BOS_OK && !type) {
		sprintf( module, ( char * ) pcMessageOKexist, alias);
		strcat( ( char * ) pcWriteBuffer, module);
	} else if (result == BOS_ERR_Keyword)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageKey, alias);
	else if (result == BOS_ERR_ExistingAlias)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageAlias, alias);	
	else if (result == BOS_ERR_ExistingCmd)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageCmd, alias);	
	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE statusCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the status command */
	DisplayModuleStatus(0);
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE infoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Read Ports directions when a pre-defined topology file is used */
	if (N > 1)
		result = ReadPortsDir();
	
	/* Respond to the info command */
	sprintf( ( char * ) pcWriteBuffer, "\n\rNumber of modules: %d\n", N);
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	sprintf( ( char * ) pcWriteBuffer, "\n\rArray topology:\n");
	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	DisplayTopology(PcPort);
	DisplayPortsDir(PcPort);
	if (result == BOS_ERR_NoResponse) {
		sprintf( ( char * ) pcWriteBuffer, "Could not read ports direction for some modules! Please try again\n\r");
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);		
	}
	sprintf( ( char * ) pcWriteBuffer, " ");
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE scastCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4; 
	static int8_t *pcParameterString5, *pcParameterString6, *pcParameterString7; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0, xParameterStringLength3 = 0; 
	portBASE_TYPE xParameterStringLength4 = 0, xParameterStringLength5 = 0, xParameterStringLength6 = 0;
	portBASE_TYPE xParameterStringLength7 = 0;
	uint8_t direction = 0, srcP = 0, dstP = 0, srcM = 0, dstM = 0; uint32_t count = 0, timeout = 0;
	char par1[MaxLengthOfAlias+1] = {0}, par2[MaxLengthOfAlias+1] = {0}, par3[MaxLengthOfAlias+1] = {0};
	
	static const int8_t *pcMessage = ( int8_t * ) "Activating a %s single-cast DMA stream from P%d in module %s to P%d in module %s. The stream will deactivate after %d bytes or %d ms\n\r";
	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (pcParameterString1[0] == 'P') {
		srcP = ( uint8_t ) atol( ( char * ) pcParameterString1+1 );
	}

	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	strncpy(par1, ( char * ) pcParameterString2, xParameterStringLength2);
	srcM = (uint8_t) GetID(par1);
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	if (pcParameterString3[0] == 'p') {
		dstP = ( uint8_t ) atol( ( char * ) pcParameterString3+1 );
	}

	/* Obtain the 4th parameter string. */
	pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
	strncpy(par2, ( char * ) pcParameterString4, xParameterStringLength4);
	dstM = (uint8_t) GetID(par2);
	
	/* Obtain the 5th parameter string. */
	pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);
	/* Read the color value. */
	if (!strncmp((const char *)pcParameterString5, "forward", xParameterStringLength5))
		direction = FORWARD;
	else if (!strncmp(( const char *)pcParameterString5, "backward", xParameterStringLength5))
		direction = BACKWARD;
	else if (!strncmp((const char *)pcParameterString5, "bidirectional", xParameterStringLength5))
		direction = BIDIRECTIONAL;
	strncpy(par3, ( char * ) pcParameterString5, xParameterStringLength5);
	
	/* Obtain the 6th parameter string. */
	pcParameterString6 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 6, &xParameterStringLength6);
	count = ( uint32_t ) atol( ( char * ) pcParameterString6 );

	/* Obtain the 7th parameter string. */
	pcParameterString7 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 7, &xParameterStringLength7);
	timeout = ( uint32_t ) atol( ( char * ) pcParameterString7 );
	
	result = StartScastDMAStream(srcP, srcM, dstP, dstM, direction, count, timeout, false);
	
	/* Respond to the command */
	if (result == BOS_OK) 
	{	
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, par3, srcP, par1, dstP, par2, count, timeout);
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE addbuttonCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1, *pcParameterString2; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0; 
	uint8_t port = 0, type = 0;
	
	static const int8_t *pcMessage = ( int8_t * ) "A new %s button named B%d was defined at port P%d\n\r";
	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "momentary-no", xParameterStringLength1)) {
		type = MOMENTARY_NO;
	} else if (!strncmp((const char *)pcParameterString1, "momentary-nc", xParameterStringLength1)) {
		type = MOMENTARY_NC;
	} else if (!strncmp((const char *)pcParameterString1, "onoff-no", xParameterStringLength1)) {
		type = ONOFF_NO;
	} else if (!strncmp((const char *)pcParameterString1, "onoff-nc", xParameterStringLength1)) {
		type = ONOFF_NC;
	}
		
	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	if (pcParameterString2[0] == 'p') {
		port = ( uint8_t ) atol( ( char * ) pcParameterString2+1 );
	}
	
	result = AddPortButton(type, port);
	
	/* Respond to the command */
	if (result == BOS_OK) 
	{	
		pcParameterString1[xParameterStringLength1] = 0;		// Get rid of the remaining parameters
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, pcParameterString1, port, port);
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE removebuttonCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1; 
	portBASE_TYPE xParameterStringLength1 = 0; 
	uint8_t port = 0;
	
	static const int8_t *pcMessage = ( int8_t * ) "Button B%d was removed from port P%d\n\r";
	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (pcParameterString1[0] == 'p') {
		port = ( uint8_t ) atol( ( char * ) pcParameterString1+1 );
	}
	
	result = RemovePortButton(port);
	
	/* Respond to the command */
	if (result == BOS_OK) 
	{	
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, port, port);
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE setCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4, *pcParameterString5; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0, xParameterStringLength3 = 0;
	portBASE_TYPE xParameterStringLength4 = 0, xParameterStringLength5 = 0;
	uint16_t temp16 = 0; uint32_t temp2 = 0; uint8_t extraMessage = 0, temp81, temp82, temp83, temp84;
	
	static const int8_t *pcMessageOK = ( int8_t * ) "%s was set to %s\n\r";	
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\n\r";	
	static const int8_t *pcMessageWrongValue = ( int8_t * ) "Wrong value!\n\r";	
	static const int8_t *pcMessageCLI1 = ( int8_t * ) "\nYou must restart to enable the new baudrate.\n\r";
	static const int8_t *pcMessageCLI2 = ( int8_t * ) "This affects all ports. If you change this value from default, \
you must connect to a CLI port on each startup to restore other array ports into default baudrate\n\r";
	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "bos.", 4)) 
	{
		/* Obtain the 2nd parameter string: the value to set */
		pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	
		if (!strncmp((const char *)pcParameterString1+4, "response", xParameterStringLength1-4)) 
		{
			if (!strncmp((const char *)pcParameterString2, "all", xParameterStringLength2)) {
				BOS.response = BOS_RESPONSE_ALL;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
			} else if (!strncmp((const char *)pcParameterString2, "message", xParameterStringLength2)) {
				BOS.response = BOS_RESPONSE_MSG;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
		  } else if (!strncmp((const char *)pcParameterString2, "cli", xParameterStringLength2)) {
				BOS.response = BOS_RESPONSE_CLI;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
		  } else if (!strncmp((const char *)pcParameterString2, "none", xParameterStringLength2)) {
				BOS.response = BOS_RESPONSE_NONE;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
			} else
				result = BOS_ERR_WrongValue;
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "trace", xParameterStringLength1-4)) 
		{
			if (!strncmp((const char *)pcParameterString2, "all", xParameterStringLength2)) {
				BOS.trace = TRACE_BOTH;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
			} else if (!strncmp((const char *)pcParameterString2, "message", xParameterStringLength2)) {
				BOS.trace = TRACE_MESSAGE;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
			} else if (!strncmp((const char *)pcParameterString2, "response", xParameterStringLength2)) {
				BOS.trace = TRACE_RESPONSE;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
		  } else if (!strncmp((const char *)pcParameterString2, "none", xParameterStringLength2)) {
				BOS.trace = TRACE_NONE;
				EE_WriteVariable(_EE_PARAMS_BASE, ((uint16_t)BOS.trace<<8) | (uint16_t)BOS.response);
			} else
				result = BOS_ERR_WrongValue;
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "clibaudrate", xParameterStringLength1-4)) 
		{
			temp2 = atoi((const char *)pcParameterString2);
			if (temp2 <= DEF_CLI_BAUDRATE) {
				BOS.clibaudrate = temp2;
				EE_WriteVariable(_EE_CLI_BAUD, (uint16_t)BOS.clibaudrate);
				EE_WriteVariable(_EE_CLI_BAUD+1, (uint16_t)(BOS.clibaudrate>>16));
				extraMessage = 1;
			} else
				result = BOS_ERR_WrongValue;			
		}
		else if (!strncmp((const char *)pcParameterString1+4, "debounce", xParameterStringLength1-4)) 
		{
			temp16 = atoi((const char *)pcParameterString2);
			if (temp16 >= 1 && temp16 <= USHRT_MAX) {
				BOS.buttons.debounce = temp16;
				EE_WriteVariable(_EE_PARAMS_DEBOUNCE, temp16);
			} else
				result = BOS_ERR_WrongValue;
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "singleclicktime", xParameterStringLength1-4)) 
		{
			temp16 = atoi((const char *)pcParameterString2);
			if (temp16 >= 1 && temp16 <= USHRT_MAX) {
				BOS.buttons.singleClickTime = temp16;
				EE_WriteVariable(_EE_PARAMS_SINGLE_CLICK, temp16);
			} else
				result = BOS_ERR_WrongValue;			
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "mininterclicktime", xParameterStringLength1-4)) 
		{
			temp16 = atoi((const char *)pcParameterString2);
			if (temp16 >= 1 && temp16 <= UCHAR_MAX) {
				BOS.buttons.minInterClickTime = temp16;
				EE_WriteVariable(_EE_PARAMS_DBL_CLICK, ((uint16_t)BOS.buttons.maxInterClickTime<<8) | (uint16_t)BOS.buttons.minInterClickTime);
			} else
				result = BOS_ERR_WrongValue;			
		} 		
		else if (!strncmp((const char *)pcParameterString1+4, "maxinterclicktime", xParameterStringLength1-4)) 
		{
			temp16 = atoi((const char *)pcParameterString2);
			if (temp16 >= 1 && temp16 <= UCHAR_MAX) {
				BOS.buttons.maxInterClickTime = temp16;
				EE_WriteVariable(_EE_PARAMS_DBL_CLICK, ((uint16_t)BOS.buttons.maxInterClickTime<<8) | (uint16_t)BOS.buttons.minInterClickTime);
			} else
				result = BOS_ERR_WrongValue;					
		} 
		else	
			result = BOS_ERR_WrongParam;
	}
	else if (!strncmp((const char *)pcParameterString1, "time", 4))
	{
		pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
		pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
		pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
		pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);
		temp81 = atoi((const char *)pcParameterString2);		// Hours
		temp82 = atoi((const char *)pcParameterString3);		// Minutes
		temp83 = atoi((const char *)pcParameterString4);		// Seconds
		
		if (pcParameterString5 != NULL) {
			if (!strncmp((const char *)pcParameterString5, "am", 2))
				temp84 = RTC_AM;
			else if (!strncmp((const char *)pcParameterString5, "pm", 2))
				temp84 = RTC_PM;
			else
				result = BOS_ERR_WrongValue;
		}
		
		if (result == BOS_OK) 
		{
			if (temp81 > 23 || temp82 > 59 || temp83 > 59)
				result = BOS_ERR_WrongValue;
			else {
				GetTimeDate();				
				result = BOS_CalendarConfig(BOS.date.month, BOS.date.day, BOS.date.year, BOS.date.weekday, temp83, temp82, temp81, temp84, BOS.daylightsaving);
			}
		}
	}
	else if (!strncmp((const char *)pcParameterString1, "date", 4))
	{
		pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
		pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
		pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
		pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);
		temp83 = atoi((const char *)pcParameterString4);		// day
		temp16 = atoi((const char *)pcParameterString5);		// year	
		
		if (!strncmp((const char *)pcParameterString2, "monday", 6))
			temp81 = MONDAY;
		else if (!strncmp((const char *)pcParameterString2, "tuesday", 7))
			temp81 = TUESDAY;
		else if (!strncmp((const char *)pcParameterString2, "wednesday", 9))
			temp81 = WEDNESDAY;
		else if (!strncmp((const char *)pcParameterString2, "thursday", 8))
			temp81 = THURSDAY;
		else if (!strncmp((const char *)pcParameterString2, "friday", 6))
			temp81 = FRIDAY;
		else if (!strncmp((const char *)pcParameterString2, "saturday", 8))
			temp81 = SATURDAY;
		else if (!strncmp((const char *)pcParameterString2, "sunday", 6))
			temp81 = SUNDAY;
		else
			result = BOS_ERR_WrongValue;		
		
		if (!strncmp((const char *)pcParameterString3, "january", 7) || !strncmp((const char *)pcParameterString3, "1 ", 2))
			temp82 = JANUARY;
		else if (!strncmp((const char *)pcParameterString3, "february", 8) || !strncmp((const char *)pcParameterString3, "2 ", 2))
			temp82 = FEBRUARY;
		else if (!strncmp((const char *)pcParameterString3, "march", 5) || !strncmp((const char *)pcParameterString3, "3 ", 2))
			temp82 = MARCH;
		else if (!strncmp((const char *)pcParameterString3, "april", 5) || !strncmp((const char *)pcParameterString3, "4 ", 2))
			temp82 = APRIL;
		else if (!strncmp((const char *)pcParameterString3, "may", 3) || !strncmp((const char *)pcParameterString3, "5 ", 2))
			temp82 = MAY;
		else if (!strncmp((const char *)pcParameterString3, "june", 4) || !strncmp((const char *)pcParameterString3, "6 ", 2))
			temp82 = JUNE;
		else if (!strncmp((const char *)pcParameterString3, "july", 4) || !strncmp((const char *)pcParameterString3, "7 ", 2))
			temp82 = JULY;
		else if (!strncmp((const char *)pcParameterString3, "august", 5) || !strncmp((const char *)pcParameterString3, "8 ", 2))
			temp82 = AUGUST;
		else if (!strncmp((const char *)pcParameterString3, "september", 9) || !strncmp((const char *)pcParameterString3, "9 ", 2))
			temp82 = SEPTEMBER;
		else if (!strncmp((const char *)pcParameterString3, "october", 7) || !strncmp((const char *)pcParameterString3, "10", 2))
			temp82 = OCTOBER;
		else if (!strncmp((const char *)pcParameterString3, "november", 8) || !strncmp((const char *)pcParameterString3, "11", 2))
			temp82 = NOVEMBER;
		else if (!strncmp((const char *)pcParameterString3, "december", 8) || !strncmp((const char *)pcParameterString3, "12", 2))
			temp82 = DECEMBER;
		else
			result = BOS_ERR_WrongValue;	
		
		if (result == BOS_OK) 
		{
			if (temp83 < 1 || temp83 > 31 || temp16 < 2000 || temp16 > 2100)
				result = BOS_ERR_WrongValue;
			else {
				GetTimeDate();
				result = BOS_CalendarConfig(temp82, temp83, temp16, temp81, BOS.time.seconds, BOS.time.minutes, BOS.time.hours, BOS.time.ampm, BOS.daylightsaving);
			}
		}
	}
	else	
		result = BOS_ERR_WrongParam;
		
	/* Respond to the command */
	if (result == BOS_OK) 
	{
		pcParameterString1[xParameterStringLength1] = 0;		// Get rid of the remaining parameters
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, pcParameterString1, pcParameterString2);
		if (extraMessage == 1) {
			strcat(( char * ) pcWriteBuffer, ( char * ) pcMessageCLI1);
			strcat(( char * ) pcWriteBuffer, ( char * ) pcMessageCLI2);
		}
	}
	else if (result == BOS_ERR_WrongParam)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	else if (result == BOS_ERR_WrongValue)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongValue );
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE getCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1, *pcParameterString2; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;
	uint8_t temp8 = 0, i = 0, j = 0;
	
	static const int8_t *pcMessageOK = ( int8_t * ) "%s\n\r";	
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\n\r";		
	static const int8_t *pcMessageWrongValue = ( int8_t * ) "%s is set to a wrong value!\n\r";	
	static const int8_t *pcMessageGroupDoesNotExist = ( int8_t * ) "%s group does not exist!\n\r";	
	static const int8_t *pcMessageGroupExists = ( int8_t * ) "Group %s members:\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "bos.", 4)) 
	{
		if (!strncmp((const char *)pcParameterString1+4, "response", xParameterStringLength1-4)) 
		{
			if (BOS.response == BOS_RESPONSE_ALL)
				sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "all");
			else if (BOS.response == BOS_RESPONSE_MSG)
				sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "msg");
			else if (BOS.response == BOS_RESPONSE_NONE)
				sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "none");
			else
				result = BOS_ERR_WrongValue;
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "trace", xParameterStringLength1-4)) 
		{
			if (BOS.trace == TRACE_BOTH)
				sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "all");
			else if (BOS.trace == TRACE_MESSAGE)
				sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "msg");
			else if (BOS.trace == TRACE_NONE)
				sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "none");
			else
				result = BOS_ERR_WrongValue;
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "clibaudrate", xParameterStringLength1-4)) 
		{
			sprintf( ( char * ) pcWriteBuffer, "%d\n\r", BOS.clibaudrate);
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "debounce", xParameterStringLength1-4)) 
		{
			sprintf( ( char * ) pcWriteBuffer, "%d\n\r", BOS.buttons.debounce);
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "singleclicktime", xParameterStringLength1-4)) 
		{
			sprintf( ( char * ) pcWriteBuffer, "%d\n\r", BOS.buttons.singleClickTime);
		} 
		else if (!strncmp((const char *)pcParameterString1+4, "mininterclicktime", xParameterStringLength1-4)) 
		{
			sprintf( ( char * ) pcWriteBuffer, "%d\n\r", BOS.buttons.minInterClickTime);
		} 		
		else if (!strncmp((const char *)pcParameterString1+4, "maxinterclicktime", xParameterStringLength1-4)) 
		{
			sprintf( ( char * ) pcWriteBuffer, "%d\n\r", BOS.buttons.maxInterClickTime);
		} 
		else	
			result = BOS_ERR_WrongParam;
	}
	else if (!strncmp((const char *)pcParameterString1, "group", 5))
	{
		pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
		temp8 = 0;
		/* Check group exists */
		for(i=0 ; i<MaxNumOfGroups ; i++)
		{
			if (!strcmp( ( char * ) pcParameterString2, groupAlias[i]))	
			{
				temp8 = 1; break;
			}
		}	
		/* Group does not exist*/
		if (!temp8)
		{
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageGroupDoesNotExist, ( char * ) pcParameterString2 );
			return pdFALSE;
		}
		else
		{
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageGroupExists, ( char * ) pcParameterString2 );
			/* Extract group members */
			for(j=1 ; j<=N ; j++)						// N modules
			{
				if (InGroup(j, i))
				{
					sprintf( ( char * ) pcWriteBuffer, "%s#%d\n\r", ( char * ) pcWriteBuffer, j );
				}
			}		
		}	
	}
	else	
		result = BOS_ERR_WrongParam;
		
	/* Respond to the command */
	if (result == BOS_ERR_WrongParam)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	else if (result == BOS_ERR_WrongValue)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongValue, pcParameterString1);
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE defaultCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK; 
	static int8_t *pcParameterString1; 
	portBASE_TYPE xParameterStringLength1 = 0;
	
	static const int8_t *pcMessageOKParams = ( int8_t * ) "All parameters set to default values\n\r";	
	static const int8_t *pcMessageOKArray = ( int8_t * ) "Current array topology was removed. Please reboot all modules\n\r";	
	static const int8_t *pcMessageWrongValue = ( int8_t * ) "Wrong value!\n\r";	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp((const char *)pcParameterString1, "params", xParameterStringLength1)) 
	{
		memcpy(&BOS, &BOS_default, sizeof(BOS_default));
		SaveEEparams();
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageOKParams );
	}
	else if (!strncmp((const char *)pcParameterString1, "array", xParameterStringLength1)) 
	{
		/* Broadcast the default array Message */
		SendMessageToModule(BOS_BROADCAST, CODE_DEF_ARRAY, 0);
		indMode = IND_TOPOLOGY; osDelay(100);
		/* Clear the topology */
		ClearEEportsDir();
		#ifndef _N
		ClearROtopology();
		#endif
		osDelay(100);
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageOKArray );
	}
	else
		result = BOS_ERR_WrongValue;

	/* Respond to the command */
	if (result == BOS_ERR_WrongValue)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongValue );
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE timeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage24 = ( int8_t * ) "Current time is %02d:%02d:%02d-%03d\n\r";	
	static const int8_t *pcMessage12 = ( int8_t * ) "Current time is %02d:%02d:%02d-%03d %s\n\r";	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	GetTimeDate();
	/* Respond to the command */
	if (BOS.hourformat == 24)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage24, BOS.time.hours, BOS.time.minutes, BOS.time.seconds, BOS.time.msec );
	else if (BOS.hourformat == 12)
	{
		if (BOS.time.ampm == RTC_AM)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage12, BOS.time.hours, BOS.time.minutes, BOS.time.seconds, BOS.time.msec, "AM" );
		else if (BOS.time.ampm == RTC_PM)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage12, BOS.time.hours, BOS.time.minutes, BOS.time.seconds, BOS.time.msec, "PM" );
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE dateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessageDate = ( int8_t * ) "Current date is %s\n\r";	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	GetTimeDate();
	/* Respond to the command */
	sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDate, GetDateString() );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE setBaudrateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	BOS_Status result = BOS_OK;

	int8_t *pcParameterString1;
	int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0;
	portBASE_TYPE xParameterStringLength2 = 0;
	static const int8_t *pcMessageOK = ( int8_t * ) "Baudrate for port P%d was set to %d\r\n";
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

	uint8_t port;
	uint32_t baudrate;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* 1st parameter for port name: P1 to P6 */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (pcParameterString1[0] == 'p') {
		port = ( uint8_t ) atol( ( char * ) pcParameterString1+1 );
	}
	else
	{
		result = BOS_ERR_WrongValue;
	}
  /* 2nd parameter for baudrate */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	baudrate = ( uint32_t ) atol( ( char * ) pcParameterString2 );

	/* Respond to the command */
	if (BOS_ERR_WrongValue == result)
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}
  else
  {
		UpdateBaudrate(port, baudrate);
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, baudrate, port);
  }

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE uuidCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessageUUID = ( int8_t * ) "MCU UUID is\n\r";	
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Respond to the command */
	sprintf( ( char * ) pcWriteBuffer, "%s", ( char * ) pcMessageUUID );
	for(uint8_t i=0 ; i<3 ; i++)
  {
#if defined  (STM32F0)
		sprintf( ( char * ) pcWriteBuffer, "%s%08X", ( char * ) pcWriteBuffer, *(uint32_t *) (MCU_F0_UUID_BASE+i*4) );
#endif
  }
	strcat(( char * ) pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE idcodeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageDEVID = ( int8_t * ) "MCU DEV_ID is %s\n\r";
	static const int8_t *pcMessageREVID = ( int8_t * ) "%sMCU REV_ID is %d.0\n\r";
	uint16_t dev = 0;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Respond to the command */
	dev = HAL_GetDEVID();
	switch (dev)
  {
  	case 0x444 :
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDEVID, "STM32F03x" );
  		break;
  	case 0x445 :
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDEVID, "STM32F04x" );
  		break;
  	case 0x440 :
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDEVID, "STM32F05x" );
  		break;
  	case 0x448 :
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDEVID, "STM32F07x" );
  		break;
  	case 0x442 :
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDEVID, "STM32F09x" );
  		break;
  	default:
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageDEVID, "UNKNOWN" );
  		break;
  }
	sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageREVID, ( char * ) pcWriteBuffer, HAL_GetREVID()>>12 );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE flashsizeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageFLASH = ( int8_t * ) "MCU Flash size is %d Kbytes\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Respond to the command */
	sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageFLASH, (*(uint32_t *) (MCU_F0_FLASH_SIZE_BASE)) & 0x0000FFFF );


	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE snipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageSnipWelcome = ( int8_t * ) "The following Command Snippets are stored in memory:\n\n\r";
	static const int8_t *pcMessageSnipAction = ( int8_t * ) "To delete a Snippet, type: del-snip x\n\rTo activate a Snippet, type: \
act-snip x\n\rTo pause a Snippet, type: pause-snip x\n\n\rwhere x is the Snippet number from the list\n\r";
	static const int8_t *pcMessageSnipStart = ( int8_t * ) "[%02d] %s\n\r";
	static const int8_t *pcMessageSnipButtonEventClicked = ( int8_t * ) "%sif b%d.clicked";
	static const int8_t *pcMessageSnipButtonEventDblClicked = ( int8_t * ) "%sif b%d.double clicked";
	static const int8_t *pcMessageSnipButtonEventPressed = ( int8_t * ) "%sif b%d.pressed for %d";
	static const int8_t *pcMessageSnipButtonEventReleased = ( int8_t * ) "%sif b%d.released for %d";	
	static const int8_t *pcMessageCmds = ( int8_t * ) "%s\n\r\t%s";
	static const int8_t *pcMessageEnd = ( int8_t * ) "\n\rend if\n\n\r";
	char status[2][7] = {"Paused", "Active"};
	static int8_t commands[ cmdMAX_INPUT_SIZE ];
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Respond to the command */
	writePxMutex(PcPort, (char*) pcMessageSnipWelcome, strlen((char*) pcMessageSnipWelcome), cmd50ms, HAL_MAX_DELAY);
	
	/* Go through all stored Snippets */
	uint8_t count = 1;
	for(uint8_t s=0 ; s<numOfRecordedSnippets ; s++)
  {
		if (snippets[s].cond.conditionType)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageSnipStart, count, status[snippets[s].state]);
		
		// Parse conditions
		switch (snippets[s].cond.conditionType)
		{
			case SNIP_COND_BUTTON_EVENT:

				switch (snippets[s].cond.buffer1[1])
        {
        	case CLICKED:
						sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageSnipButtonEventClicked, ( char * ) pcWriteBuffer, snippets[s].cond.buffer1[0], snippets[s].cmd);				
        		break;
        	case DBL_CLICKED:
						sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageSnipButtonEventDblClicked, ( char * ) pcWriteBuffer, snippets[s].cond.buffer1[0], snippets[s].cmd);				
        		break;
					case PRESSED_FOR_X1_SEC:
					case PRESSED_FOR_X2_SEC:
					case PRESSED_FOR_X3_SEC:
						sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageSnipButtonEventPressed, ( char * ) pcWriteBuffer, snippets[s].cond.buffer1[0], snippets[s].cond.buffer1[2], snippets[s].cmd);				
        		break;
					case RELEASED_FOR_Y1_SEC:
					case RELEASED_FOR_Y2_SEC:
					case RELEASED_FOR_Y3_SEC:
						sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageSnipButtonEventReleased, ( char * ) pcWriteBuffer, snippets[s].cond.buffer1[0], snippets[s].cond.buffer1[2], snippets[s].cmd);				
        		break;						
        	default:
        		break;
        }
				
				break;
				
			case SNIP_COND_MODULE_PARAM_CONST:
				break;
			
			default:
				break;
		}
		
		// Parse commands
		while (ParseSnippetCommand(snippets[s].cmd, (int8_t *) &commands) != false)
		{
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageCmds, pcWriteBuffer, commands );
			memset( &commands, 0x00, strlen((char*) commands) );
		}
		
		// Finish and write the buffer
		strcat( ( char * ) pcWriteBuffer, ( char * ) pcMessageEnd);
		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		
		++count;
	}

	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageSnipAction );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE actSnipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageOK = ( int8_t * ) "Snippet was activated. Type snip to view updated list\n\r";
	static const int8_t *pcMessageWrong = ( int8_t * ) "The Snippet number was not found\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	BOS_Status result = BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* 1st parameter for Snippet index */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	uint8_t index = ( uint8_t ) atoi( ( char * ) pcParameterString1 );
	
	if (!index || index > numOfRecordedSnippets)	result = BOS_ERROR;
	
	/* Respond to the command */
	if (result == BOS_OK) {
		snippets[index-1].state = true;
		SaveToRO();
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK );
	}
	else
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrong );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE pauseSnipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageOK = ( int8_t * ) "Snippet was paused. Type snip to view updated list\n\r";
	static const int8_t *pcMessageWrong = ( int8_t * ) "The Snippet number was not found\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	BOS_Status result = BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* 1st parameter for Snippet index */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	uint8_t index = ( uint8_t ) atoi( ( char * ) pcParameterString1 );
	
	if (!index || index > numOfRecordedSnippets)	result = BOS_ERROR;
	
	/* Respond to the command */
	if (result == BOS_OK) {
		snippets[index-1].state = false;
		SaveToRO();
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK );
	}
	else
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrong );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE delSnipCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageOK = ( int8_t * ) "Snippet was deleted. Type snip to view updated list\n\r";
	static const int8_t *pcMessageWrong = ( int8_t * ) "The Snippet number was not found\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	BOS_Status result = BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* 1st parameter for Snippet index */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	uint8_t index = ( uint8_t ) atoi( ( char * ) pcParameterString1 );
	
	if (!index || index > numOfRecordedSnippets)	result = BOS_ERROR;
	
	if (result == BOS_OK)
	{
		// Delete the Snippet
		snippets[index-1].cond.conditionType = 0;
		snippets[index-1].cond.mathOperator = 0;
		memset(snippets[index-1].cond.buffer1, 0, 4);
		snippets[index-1].state = false;
		free(snippets[index-1].cmd);
		snippets[index-1].cmd = NULL;
		
		// Reorder remaining Snippets to avoid empty indices
		for(uint8_t s=index ; s<numOfRecordedSnippets ; s++) {
			if (snippets[s].cond.conditionType) {
				memcpy( &snippets[s-1], &snippets[s], sizeof(snippet_t) );
				memset( &snippets[s], 0, sizeof(snippet_t) );
			}
		}		
		--numOfRecordedSnippets;
		
		// Write updated list to RO
		SaveToRO();
	}
	
	/* Respond to the command */
	if (result == BOS_OK)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK );
	else
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrong );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE bridgeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageOK = ( int8_t * ) "P%d and P%d are bridged together\n\r";
	static const int8_t *pcMessageWrong = ( int8_t * ) "Wrong syntax\n\r";
	static const int8_t *pcMessageFail = ( int8_t * ) "Port bridging failed\n\r";
	int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;
	BOS_Status result = BOS_OK;
	uint8_t port1, port2;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (pcParameterString1[0] == 'p') {
		port1 = ( uint8_t ) atol( ( char * ) pcParameterString1+1 );
	} else {
		result = BOS_ERR_WrongParam;
	}
	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	if (pcParameterString2[0] == 'p') {
		port2 = ( uint8_t ) atol( ( char * ) pcParameterString2+1 );
	} else {
		result = BOS_ERR_WrongParam;
	}
	
	/* Build the bridge */
	if (result == BOS_OK) 
		result = Bridge(port1, port2);
	else
		result = BOS_ERR_WrongParam;
	
	/* Return CLI output */
	if (result == BOS_OK) 
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, port1, port2 );
	else if (result == BOS_ERR_WrongParam) 
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrong );	
	else 
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageFail );	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
	
/*-----------------------------------------------------------*/


static portBASE_TYPE unbridgeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	static const int8_t *pcMessageOK = ( int8_t * ) "P%d and P%d are un-bridged\n\r";
	static const int8_t *pcMessageWrong = ( int8_t * ) "Wrong syntax\n\r";
	static const int8_t *pcMessageFail = ( int8_t * ) "Port unbridging failed\n\r";
	int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;
	BOS_Status result = BOS_OK;
	uint8_t port1, port2;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (pcParameterString1[0] == 'p') {
		port1 = ( uint8_t ) atol( ( char * ) pcParameterString1+1 );
	} else {
		result = BOS_ERR_WrongParam;
	}
	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	if (pcParameterString2[0] == 'p') {
		port2 = ( uint8_t ) atol( ( char * ) pcParameterString2+1 );
	} else {
		result = BOS_ERR_WrongParam;
	}
	
	/* Build the bridge */
	if (result == BOS_OK) 
		result = Unbridge(port1, port2);
	else
		result = BOS_ERR_WrongParam;
	
	/* Return CLI output */
	if (result == BOS_OK) 
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, port1, port2 );
	else if (result == BOS_ERR_WrongParam) 
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrong );	
	else 
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageFail );	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
	
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
