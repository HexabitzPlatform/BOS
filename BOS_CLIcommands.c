/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : BOS_CLIcommands.c
 Description   : Source code for Bitz CLI commands APIs.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private and Golbal variables ----------------------------------------------*/
extern BOS_t BOS;
extern BOS_t BOS_default;
extern uint8_t numOfRecordedSnippets;

/* Define BOS keywords */
const char mathStr[NUM_MATH_OPERATORS][3] ={"==", ">", "<", ">=", "<=", "!="};

/* Define long messages -------------------------------------------------------*/

/* Exported functions */
extern uint8_t SaveToRO(void);
extern BOS_Status SaveEEparams(void);
extern BOS_Status ClearEEportsDir(void);
#ifndef __N
extern uint8_t ClearROtopology(void);
#endif
extern void RegisterModuleCLICommands(void);
extern bool ParseSnippetCommand(char *snippetBuffer,int8_t *cliBuffer);
extern void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE prvTaskStatsCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE prvRunTimeStatsCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE pingCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE bootloaderUpdateCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
#ifndef __N
static portBASE_TYPE exploreCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
#endif
static portBASE_TYPE resetCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE nameCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE groupCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE statusCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE infoCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE scastCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE addbuttonCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE removebuttonCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE setCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE getCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE defaultCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE timeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE dateCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE setBaudrateCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE uuidCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE idcodeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE flashsizeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE snipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE actSnipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE pauseSnipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE delSnipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE bridgeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE unbridgeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE testportCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE ADCReadCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE ReadTempCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE ReadVrefCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
static portBASE_TYPE GetReadPrecentageCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);

/* CLI command structure : run-time-stats 
 This generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t prvRunTimeStatsCommandDefinition ={
	(const int8_t* )"run-time-stats", /* The command string to type. */
	(const int8_t* )"run-time-stats:\r\n Display a table showing how much processing time each FreeRTOS task has used\r\n\r\n", prvRunTimeStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : task-stats" 
 This generates a table that gives information on each task in the system. */
static const CLI_Command_Definition_t prvTaskStatsCommandDefinition ={
	(const int8_t* )"task-stats", /* The command string to type. */
	(const int8_t* )"task-stats:\r\n Display a table showing the state of each FreeRTOS task\r\n\r\n", prvTaskStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ping */
static const CLI_Command_Definition_t pingCommandDefinition ={
	(const int8_t* )"ping", /* The command string to type. */
	(const int8_t* )"ping:\r\n Ping a module\r\n\r\n", pingCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : update */
static const CLI_Command_Definition_t bootloaderUpdateCommandDefinition ={
	(const int8_t* )"update", /* The command string to type. */
	(const int8_t* )"update:\r\n Put the module in factory bootloader mode to update its firmware.\r\n\n\
						  - Use '#n.update' to update remote module n. Make sure the programming port is the incoming port of this module.\r\n\
						  - Use 'update via #n px' to use the ST Flash Loader tool with module n, port x to update a neighbor module. This is useful if \
							target module is not part of the topology or if programming port is not in the shortest path to target module.\r\n\r\n", bootloaderUpdateCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : explore */
#ifndef __N
static const CLI_Command_Definition_t exploreCommandDefinition ={
	(const int8_t* )"explore", /* The command string to type. */
	(const int8_t* )"explore:\r\n Explore the array and build its topology\r\n\r\n", exploreCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
#endif
/*-----------------------------------------------------------*/
/* CLI command structure : reset */
static const CLI_Command_Definition_t resetCommandDefinition ={
	(const int8_t* )"reset", /* The command string to type. */
	(const int8_t* )"reset:\r\n Reset the module\r\n\r\n", resetCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : name */
static const CLI_Command_Definition_t nameCommandDefinition ={
	(const int8_t* )"name", /* The command string to type. */
	(const int8_t* )"name:\r\n Name the module with an alias (1st par.)\r\n\r\n", nameCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : group */
static const CLI_Command_Definition_t groupCommandDefinition ={
	(const int8_t* )"group", /* The command string to type. */
	(const int8_t* )"group:\r\n Group multiple modules (2nd+ par.) into a new or existing group (1st par.)\r\n\r\n", groupCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : status */
static const CLI_Command_Definition_t statusCommandDefinition ={
	(const int8_t* )"status", /* The command string to type. */
	(const int8_t* )"status:\r\n Display module status\r\n\r\n", statusCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : info */
static const CLI_Command_Definition_t infoCommandDefinition ={
	(const int8_t* )"info", /* The command string to type. */
	(const int8_t* )"info:\r\n Display array information\r\n\r\n", infoCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : scast */
static const CLI_Command_Definition_t scastCommandDefinition ={
	(const int8_t* )"scast", /* The command string to type. */
	(const int8_t* )"scast:\r\n Start a single-cast DMA stream. Source port (1st par.), source module (2nd par.), destination port (3rd par.), \
                     destination module (4th par.), direction ('forward', 'backward', 'bidirectional') (5th par.), transfer count (bytes) (6th par.), transfer timeout (ms) (7th par.)\r\n\r\n", scastCommand, /* The function to run. */
    7 /* Seven parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : add button */
static const CLI_Command_Definition_t addbuttonCommandDefinition ={
	(const int8_t* )"add-button", /* The command string to type. */
	(const int8_t* )"add-button:\r\n Define a button at one of the array ports. Button type ('momentary-no', 'momentary-nc', 'onoff-no', 'onoff-nc')(1st par.), Button port (2nd par.)\r\n\r\n", addbuttonCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : remove button */
static const CLI_Command_Definition_t removebuttonCommandDefinition ={
	(const int8_t* )"remove-button", /* The command string to type. */
	(const int8_t* )"remove-button:\r\n Remove a button that was previously defined at this port (1st par.)\r\n\r\n", removebuttonCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : set */
static const CLI_Command_Definition_t setCommandDefinition ={
	(const int8_t* )"set", /* The command string to type. */
	(const int8_t* )"set:\r\n Set a parameter (1st par.) with a given value (2nd par.)\r\n\r\n", setCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : get */
static const CLI_Command_Definition_t getCommandDefinition ={
	(const int8_t* )"get", /* The command string to type. */
	(const int8_t* )"get:\r\n Get the current value of a parameter (1st par.)\r\n\r\n", getCommand, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : default */
static const CLI_Command_Definition_t defaultCommandDefinition ={
	(const int8_t* )"default", /* The command string to type. */
	(const int8_t* )"default:\r\n Type 'default params' to set all parameters to default values\r\n Type 'default array' to remove current topology\r\n\r\n", defaultCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : time */
static const CLI_Command_Definition_t timeCommandDefinition ={
	(const int8_t* )"time", /* The command string to type. */
	(const int8_t* )"time:\r\n Display current time in HH:MM:SS-msec format\r\n\r\n", timeCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : date */
static const CLI_Command_Definition_t dateCommandDefinition ={
	(const int8_t* )"date", /* The command string to type. */
	(const int8_t* )"date:\r\n Display current date in Weekday MM/DD/YYYY format\r\n\r\n", dateCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : set-baudrate */
const CLI_Command_Definition_t setBaudrateCommandDefinition ={
	(const int8_t* )"set-baudrate", /* The command string to type. */
	(const int8_t* )"set-baudrate:\r\n Set UART baudrate\r\n\t(1st parameter): P1 to P6\r\n\t(2nd parameter): baudrate\r\n\r\n", setBaudrateCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : uuid */
static const CLI_Command_Definition_t uuidCommandDefinition ={
	(const int8_t* )"uuid", /* The command string to type. */
	(const int8_t* )"uuid:\r\n Display MCU unique UID\r\n\r\n", uuidCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : idcode */
static const CLI_Command_Definition_t idcodeCommandDefinition ={
	(const int8_t* )"idcode", /* The command string to type. */
	(const int8_t* )"idcode:\r\n Display MCU IDCODE (DEV_ID and REV_ID)\r\n\r\n", idcodeCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : flash-size */
static const CLI_Command_Definition_t flashsizeCommandDefinition ={
	(const int8_t* )"flash-size", /* The command string to type. */
	(const int8_t* )"flash-size:\r\n Display MCU Flash size in Kbytes\r\n\r\n", flashsizeCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : snip */
static const CLI_Command_Definition_t snipCommandDefinition ={
	(const int8_t* )"snip", /* The command string to type. */
	(const int8_t* )"snip:\r\n Display a list of stored Command Snippets to edit or delete\r\n\r\n", snipCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : act-snip */
static const CLI_Command_Definition_t actSnipCommandDefinition ={
	(const int8_t* )"act-snip", /* The command string to type. */
	(const int8_t* )"act-snip:\r\n Activate a Command Snippet\r\n\r\n", actSnipCommand, /* The function to run. */
	1 /* One parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : pause-snip */
static const CLI_Command_Definition_t pauseSnipCommandDefinition ={
	(const int8_t* )"pause-snip", /* The command string to type. */
	(const int8_t* )"pause-snip:\r\n Pause a Command Snippet\r\n\r\n", pauseSnipCommand, /* The function to run. */
	1 /* One parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : del-snip */
static const CLI_Command_Definition_t delSnipCommandDefinition ={
	(const int8_t* )"del-snip", /* The command string to type. */
	(const int8_t* )"del-snip:\r\n Delete a Command Snippet\r\n\r\n", delSnipCommand, /* The function to run. */
	1 /* One parameters is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : bridge */
static const CLI_Command_Definition_t bridgeCommandDefinition ={
	(const int8_t* )"bridge", /* The command string to type. */
	(const int8_t* )"bridge:\r\n Bridge two array ports\r\n\r\n", bridgeCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : Unbridge */
static const CLI_Command_Definition_t unbridgeCommandDefinition ={
	(const int8_t* )"unbridge", /* The command string to type. */
	(const int8_t* )"unbridge:\r\n Un-bridge two array ports\r\n\r\n", unbridgeCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : test port */
static const CLI_Command_Definition_t testportCommandDefinition ={
	(const int8_t* )"test-port", /* The command string to type. */
	(const int8_t* )"test-port:\r\n test port functionality. you can choose either to test one specific port or to test all ports. please type Px where x stands for the number of port or type <all> to test all port\r\n\r\n", testportCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
/*-----------------------------------------------------------*/

/* CLI command structure : Read ADC value */
static const CLI_Command_Definition_t ADCReadCommandDefinition ={
	(const int8_t* )"read-adc", /* The command string to type. */
	(const int8_t* )"read-adc:\r\n Read ADC Value from Port 2 or Port 3 and choose the side whereas top or bottom\r\n\r\n", ADCReadCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : Read internal temperature value */
static const CLI_Command_Definition_t ReadTempDefinition ={
	(const int8_t* )"read-temp", /* The command string to type. */
	(const int8_t* )"read-temp:\r\n Read internal temperature\r\n\r\n", ReadTempCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : Read internal voltage reference value */
static const CLI_Command_Definition_t ReadVrefDefinition ={
	(const int8_t* )"read-vref", /* The command string to type. */
	(const int8_t* )"read-vref:\r\n Read internal reference Voltage\r\n\r\n", ReadVrefCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : Read ADC Percentage value */
static const CLI_Command_Definition_t GetReadPercentageDefinition ={
	(const int8_t* )"read-adc-percentage", /* The command string to type. */
	(const int8_t* )"read-adc-percentage:\r\n Get percentage value from port 2 or port 3\r\n\r\n", GetReadPrecentageCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* --- Register user CLI Commands 
 This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak void RegisterUserCLICommands(void){
	
}

/*-----------------------------------------------------------*/

/* Register the commands.
 */
void vRegisterCLICommands(void){
	/* Register all BOS CLI commands */
	FreeRTOS_CLIRegisterCommand(&prvTaskStatsCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&prvRunTimeStatsCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&pingCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&bootloaderUpdateCommandDefinition);
#ifndef __N
	FreeRTOS_CLIRegisterCommand(&exploreCommandDefinition);
#endif
	FreeRTOS_CLIRegisterCommand(&resetCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&nameCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&groupCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&statusCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&infoCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&scastCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&addbuttonCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&removebuttonCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&setCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&getCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&defaultCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&timeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&dateCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&setBaudrateCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&uuidCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&idcodeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&flashsizeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&snipCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&actSnipCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&pauseSnipCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&delSnipCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&bridgeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&unbridgeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&testportCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&ADCReadCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&ReadTempDefinition);
	FreeRTOS_CLIRegisterCommand(&ReadVrefDefinition);
	FreeRTOS_CLIRegisterCommand(&GetReadPercentageDefinition);
	numOfBosCommands =34;			// Add "help" command
#ifndef __N
	numOfBosCommands =35;
#endif
	
	/* Register module CLI commands */
	RegisterModuleCLICommands();
	
	/* Register user CLI commands */
	RegisterUserCLICommands();
}

/* -----------------------------------------------------------------------
 |															Commands																 	|
 -----------------------------------------------------------------------
 */

static portBASE_TYPE prvTaskStatsCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	const int8_t *const pcTaskTableHeader =(int8_t* )"Task State Priority Stack #\r\n************************************************\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Generate a table of task stats. */
	strcpy((char* )pcWriteBuffer,(char* )pcTaskTableHeader);
	vTaskList(((char* )pcWriteBuffer) + strlen((char* )pcTaskTableHeader));
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvRunTimeStatsCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	const int8_t *const pcStatsTableHeader =(int8_t* )"Task Abs Time % Time\r\n****************************************\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Generate a table of task stats. */
	strcpy((char* )pcWriteBuffer,(char* )pcStatsTableHeader);
	vTaskGetRunTimeStats(((char* )pcWriteBuffer) + strlen((char* )pcStatsTableHeader));
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE pingCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessage1 =(int8_t* )"Hi from module %d\r\n";
	static const int8_t *pcMessage2 =(int8_t* )"Hi from module %d (%s)\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Respond to the ping */
	if(!moduleAlias[myID][0])
		sprintf((char* )pcWriteBuffer,(char* )pcMessage1,myID);
	else
		sprintf((char* )pcWriteBuffer,(char* )pcMessage2,myID,moduleAlias[myID]);
	
	RTOS_IND_blink(200);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/


static portBASE_TYPE bootloaderUpdateCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessage =(int8_t* )"Update firmware for module %d\n\r";
	static const int8_t *pcMessageWrongValue =(int8_t* )"Wrong value!\n\r";
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3;
	static portBASE_TYPE xParameterStringLength1, xParameterStringLength2, xParameterStringLength3;
	uint8_t module, port;
	BOS_Status result =BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	
	/* Local update */
	if(pcParameterString1 == NULL){
		/* Respond to the update command */
		sprintf((char* )pcWriteBuffer,(char* )pcMessage,myID);
		strcat((char* )pcWriteBuffer,(char* )pcBootloaderUpdateMessage);
		writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),cmd50ms,HAL_MAX_DELAY);
		#ifndef STM32G0B1xx
		/* Address for RAM signature (STM32F09x) - Last 4 words of SRAM */
		*((unsigned long* )0x20007FF0) =0xDEADBEEF;
		#else
		/* Address for RAM signature (STM32G0Bx) - Last 4 words of SRAM */
		*((unsigned long* )0x20023FF0) =0xDEADBEEF; //Boundary address[0x20000000 - 0x20023FFF]
		#endif
		indMode =IND_PING;
		osDelay(10);
		NVIC_SystemReset();
	}
	else{
		/* This is a 'via port' remote update command */
		if(!strncmp((const char* )pcParameterString1,"via",xParameterStringLength1)){
			pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
			pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,3,&xParameterStringLength3);
			
			/* Parse the module */
			if(pcParameterString2[0] == '#'){
				module =(uint8_t )atol((char* )pcParameterString2 + 1);
			}
			else
				result =BOS_ERR_WrongValue;
			
			/* Parse the port */
			if(pcParameterString3[0] == 'p'){
				port =(uint8_t )atol((char* )pcParameterString3 + 1);
			}
			else
				result =BOS_ERR_WrongValue;
			
			/* I'm the source of the command and the target is > 1 hop away */
			if(module != myID){
				/* Deactivate responses */
				BOSMessaging.response = BOS_RESPONSE_NONE;
				
				/* Forward the command */
				messageParams[0] =port;
				SendMessageToModule(module,CODE_UPDATE_VIA_PORT,1);
				osDelay(100);
				/* Execute locally */
				remoteBootloaderUpdate(myID,module,PcPort,port);
			}
			/* I'm the source of the command and my neighbor is the target */
			else{
				/* Ask the target to jump to factory bootloader */
				SendMessageFromPort(port,0,0,CODE_UPDATE,0);
				osDelay(100);
				/* Then, setup myself for remote 'via port' update */
				remoteBootloaderUpdate(myID,myID,PcPort,port);
			}
		}
		else
			result =BOS_ERR_WrongValue;
	}
	
	/* Respond to user */
	if(result == BOS_ERR_WrongValue){
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongValue);
	}
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
#ifndef __N
static portBASE_TYPE exploreCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
//	BOS_Status result = BOS_OK;
//	static const int8_t *pcMessage = ( int8_t * ) "\nThe array is being explored. Please wait...\n\r";
//	static const int8_t *pcMessageOK = ( int8_t * ) "\nThe array exploration succeeded. I found %d modules including myself. Here is the discovered topology:\n\r";
//	static const int8_t *pcMessageErr = ( int8_t * ) "\nThe array exploration failed. Please double check connections, reset the modules and try again.\n\r";
//	
//	/* Remove compile time warnings about unused parameters, and check the
//	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
//	write buffer length is adequate, so does not check for buffer overflows. */
//	( void ) pcCommandString;
//	( void ) xWriteBufferLen;
//	configASSERT( pcWriteBuffer );
	
//	/* Respond to the update command */
//	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessage );
//	writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
//	
//	/* Call array exploration routine */
//	result = Explore();
//	if (result == BOS_OK) {
//		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, N);
//		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
//		DisplayTopology(PcPort);
//		DisplayPortsDir(PcPort);
//	} else {
//		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageErr );
//		writePxMutex(PcPort, (char*) pcWriteBuffer, strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
//	}
//	sprintf( ( char * ) pcWriteBuffer, " ");
//	
//	/* There is no more data to return after this single string, so return
//	pdFALSE. */
//	return pdFALSE;
	return 0;
}
#endif

/*-----------------------------------------------------------*/

static portBASE_TYPE resetCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	NVIC_SystemReset();
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE nameCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1;
	static portBASE_TYPE xParameterStringLength1;
	
	static const int8_t *pcMessageOK =(int8_t* )"Module %d is named %s\n\r";
	static const int8_t *pcMessageKey =(int8_t* )"%s is a reserved BOS keyword! Please use a different alias\n\r";
	static const int8_t *pcMessageAlias =(int8_t* )"%s is already used! Please use a different alias\n\r";
	static const int8_t *pcMessageCmd =(int8_t* )"%s is an existing CLI command! Please use a different alias\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	
	/* Check alias length */
	if(xParameterStringLength1 > MaxLengthOfAlias){
		pcParameterString1[MaxLengthOfAlias] ='\0';
	}
	
	/* Name the module */
	result =NameModule(myID,(char* )pcParameterString1);
	
	/* Respond to the update command */
	if(result == BOS_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,myID,pcParameterString1);
	else if(result == BOS_ERR_Keyword)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageKey,pcParameterString1);
	else if(result == BOS_ERR_ExistingAlias)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageAlias,pcParameterString1);
	else if(result == BOS_ERR_ExistingCmd)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageCmd,pcParameterString1);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE groupCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1, *pcParameterString, count;
	static portBASE_TYPE xParameterStringLength1, xParameterStringLength;
	char module[MaxLengthOfAlias + 30] ={0};
	int16_t modID =0, type =0;
	char alias[MaxLengthOfAlias + 1] ={0};
	
	static const int8_t *pcMessageWrongModule =(int8_t* )"%s is a wrong module ID or alias\n\r";
	static const int8_t *pcMessageOKnew =(int8_t* )"] added to new group %s\n\r";
	static const int8_t *pcMessageOKexist =(int8_t* )"] added to existing group %s\n\r";
	static const int8_t *pcMessageKey =(int8_t* )"%s is a reserved BOS keyword! Please use a different alias\n\r";
	static const int8_t *pcMessageAlias =(int8_t* )"%s is already used! Please use a different alias\n\r";
	static const int8_t *pcMessageCmd =(int8_t* )"%s is an existing CLI command! Please use a different alias\n\r";
	static const int8_t *pcMessageNoModules =(int8_t* )"Please enter some module IDs to add to %s\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string - Group name */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	strncpy(alias,(char* )pcParameterString1,xParameterStringLength1);
	
	/* Is it new or existing group? */
	type =1;
	for(uint8_t i =0; i < MaxNumOfGroups; i++){
		/* This group already exists */
		if(!strcmp(alias,groupAlias[i])){
			type =0;
			break;
		}
	}
	
	/* Extract modules and add them to group */
	count =2;
	strcpy((char* )pcWriteBuffer,"Modules [");
	pcParameterString =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,count,&xParameterStringLength);
	while(pcParameterString != NULL){
		strncpy(module,(char* )pcParameterString,xParameterStringLength);
		module[xParameterStringLength] ='\0';
		modID =GetID(module);
		
		if(modID < 0)
			break;
		
		result =AddModuleToGroup(modID,alias);
		
		if(result != BOS_OK)
			break;
		
		if(count > 2)
			strcat((char* )pcWriteBuffer,", ");
		
		strcat((char* )pcWriteBuffer,module);
		
		/* Extract next module */
		pcParameterString =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,++count,&xParameterStringLength);
	}
	
	/* Respond to the update command */
	if(modID < 0)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageWrongModule,module);
	else if(count == 2)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageNoModules,alias);
	else if(result == BOS_OK && type){
		sprintf(module,(char* )pcMessageOKnew,alias);
		strcat((char* )pcWriteBuffer,module);
	}
	else if(result == BOS_OK && !type){
		sprintf(module,(char* )pcMessageOKexist,alias);
		strcat((char* )pcWriteBuffer,module);
	}
	else if(result == BOS_ERR_Keyword)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageKey,alias);
	else if(result == BOS_ERR_ExistingAlias)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageAlias,alias);
	else if(result == BOS_ERR_ExistingCmd)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageCmd,alias);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE statusCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Respond to the status command */
	DisplayModuleStatus(0);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE infoCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )pcCommandString;
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Read Ports directions when a pre-defined topology file is used */
	if(N > 1)
		result =ReadPortsDir();
	
	/* Respond to the info command */
	sprintf((char* )pcWriteBuffer,"\n\rNumber of modules: %d\n",N);
	writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),
	cmd50ms,HAL_MAX_DELAY);
	sprintf((char* )pcWriteBuffer,"\n\rArray topology:\n");
	writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),
	cmd50ms,HAL_MAX_DELAY);
	DisplayTopology(PcPort);
	DisplayPortsDir(PcPort);
	if(result == BOS_ERR_NoResponse){
		sprintf((char* )pcWriteBuffer,"Could not read ports direction for some modules! Please try again\n\r");
		writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),cmd50ms,HAL_MAX_DELAY);
	}
	sprintf((char* )pcWriteBuffer," ");
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE scastCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4;
	static int8_t *pcParameterString5, *pcParameterString6, *pcParameterString7;
	portBASE_TYPE xParameterStringLength1 =0, xParameterStringLength2 =0, xParameterStringLength3 =0;
	portBASE_TYPE xParameterStringLength4 =0, xParameterStringLength5 =0, xParameterStringLength6 =0;
	portBASE_TYPE xParameterStringLength7 =0;
	uint8_t direction =0, srcP =0, dstP =0, srcM =0, dstM =0;
	uint32_t count =0, timeout =0;
	char par1[MaxLengthOfAlias + 1] ={0}, par2[MaxLengthOfAlias + 1] ={0}, par3[MaxLengthOfAlias + 1] ={0};
	
	static const int8_t *pcMessage =(int8_t* )"Activating a %s single-cast DMA stream from P%d in module %s to P%d in module %s. The stream will deactivate after %d bytes or %d ms\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(pcParameterString1[0] == 'P'){
		srcP =(uint8_t )atol((char* )pcParameterString1 + 1);
	}
	
	/* Obtain the 2nd parameter string. */
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	strncpy(par1,(char* )pcParameterString2,xParameterStringLength2);
	srcM =(uint8_t )GetID(par1);
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,3,&xParameterStringLength3);
	if(pcParameterString3[0] == 'p'){
		dstP =(uint8_t )atol((char* )pcParameterString3 + 1);
	}
	
	/* Obtain the 4th parameter string. */
	pcParameterString4 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,4,&xParameterStringLength4);
	strncpy(par2,(char* )pcParameterString4,xParameterStringLength4);
	dstM =(uint8_t )GetID(par2);
	
	/* Obtain the 5th parameter string. */
	pcParameterString5 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,5,&xParameterStringLength5);
	/* Read the color value. */
	if(!strncmp((const char* )pcParameterString5,"forward",xParameterStringLength5))
		direction =FORWARD;
	else if(!strncmp((const char* )pcParameterString5,"backward",xParameterStringLength5))
		direction =BACKWARD;
	else if(!strncmp((const char* )pcParameterString5,"bidirectional",xParameterStringLength5))
		direction =BIDIRECTIONAL;
	strncpy(par3,(char* )pcParameterString5,xParameterStringLength5);
	
	/* Obtain the 6th parameter string. */
	pcParameterString6 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,6,&xParameterStringLength6);
	count =(uint32_t )atol((char* )pcParameterString6);
	
	/* Obtain the 7th parameter string. */
	pcParameterString7 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,7,&xParameterStringLength7);
	timeout =(uint32_t )atol((char* )pcParameterString7);
	
	result =StartScastDMAStream(srcP,srcM,dstP,dstM,direction,count,timeout,false);
	
	/* Respond to the command */
	if(result == BOS_OK){
		sprintf((char* )pcWriteBuffer,(char* )pcMessage,par3,srcP,par1,dstP,par2,count,timeout);
	}
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE addbuttonCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0, xParameterStringLength2 =0;
	uint8_t port =0, type =0;
	
	static const int8_t *pcMessage =(int8_t* )"A new %s button named B%d was defined at port P%d\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(!strncmp((const char* )pcParameterString1,"momentary-no",xParameterStringLength1)){
		type =MOMENTARY_NO;
	}
	else if(!strncmp((const char* )pcParameterString1,"momentary-nc",xParameterStringLength1)){
		type =MOMENTARY_NC;
	}
	else if(!strncmp((const char* )pcParameterString1,"onoff-no",xParameterStringLength1)){
		type =ONOFF_NO;
	}
	else if(!strncmp((const char* )pcParameterString1,"onoff-nc",xParameterStringLength1)){
		type =ONOFF_NC;
	}
	
	/* Obtain the 2nd parameter string. */
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	if(pcParameterString2[0] == 'p'){
		port =(uint8_t )atol((char* )pcParameterString2 + 1);
	}
	
	result =AddPortButton(type,port);
	
	/* Respond to the command */
	if(result == BOS_OK){
		pcParameterString1[xParameterStringLength1] =0;			// Get rid of the remaining parameters
		sprintf((char* )pcWriteBuffer,(char* )pcMessage,pcParameterString1,port,port);
	}
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE removebuttonCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	uint8_t port =0;
	
	static const int8_t *pcMessage =(int8_t* )"Button B%d was removed from port P%d\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(pcParameterString1[0] == 'p'){
		port =(uint8_t )atol((char* )pcParameterString1 + 1);
	}
	
	result =RemovePortButton(port);
	
	/* Respond to the command */
	if(result == BOS_OK){
		sprintf((char* )pcWriteBuffer,(char* )pcMessage,port,port);
	}
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE setCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4, *pcParameterString5;
	portBASE_TYPE xParameterStringLength1 =0, xParameterStringLength2 =0, xParameterStringLength3 =0;
	portBASE_TYPE xParameterStringLength4 =0, xParameterStringLength5 =0;
	uint16_t temp16 =0;
	uint32_t temp2 =0;
	uint8_t extraMessage =0, temp81, temp82, temp83, temp84;
	
	static const int8_t *pcMessageOK =(int8_t* )"%s was set to %s\n\r";
	static const int8_t *pcMessageWrongParam =(int8_t* )"Wrong parameter!\n\r";
	static const int8_t *pcMessageWrongValue =(int8_t* )"Wrong value!\n\r";
	static const int8_t *pcMessageCLI1 =(int8_t* )"\nYou must restart to enable the new baudrate.\n\r";
	static const int8_t *pcMessageCLI2 =(int8_t* )"This affects all ports. If you change this value from default, \
												   you must connect to a CLI port on each startup to restore other array ports into default baudrate\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(!strncmp((const char* )pcParameterString1,"bos.",4)){
		/* Obtain the 2nd parameter string: the value to set */
		pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
		
		if(!strncmp((const char* )pcParameterString1 + 4,"response",xParameterStringLength1 - 4)){
			if(!strncmp((const char* )pcParameterString2,"all",xParameterStringLength2)){
				BOSMessaging.response = BOS_RESPONSE_ALL;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else if(!strncmp((const char* )pcParameterString2,"message",xParameterStringLength2)){
				BOSMessaging.response = BOS_RESPONSE_MSG;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else if(!strncmp((const char* )pcParameterString2,"cli",xParameterStringLength2)){
				BOSMessaging.response = BOS_RESPONSE_CLI;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else if(!strncmp((const char* )pcParameterString2,"none",xParameterStringLength2)){
				BOSMessaging.response = BOS_RESPONSE_NONE;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"trace",xParameterStringLength1 - 4)){
			if(!strncmp((const char* )pcParameterString2,"all",xParameterStringLength2)){
				BOSMessaging.trace =TRACE_BOTH;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else if(!strncmp((const char* )pcParameterString2,"message",xParameterStringLength2)){
				BOSMessaging.trace =TRACE_MESSAGE;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else if(!strncmp((const char* )pcParameterString2,"response",xParameterStringLength2)){
				BOSMessaging.trace =TRACE_RESPONSE;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else if(!strncmp((const char* )pcParameterString2,"none",xParameterStringLength2)){
				BOSMessaging.trace =TRACE_NONE;
				EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 8) | (uint16_t )BOSMessaging.response);
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"clibaudrate",xParameterStringLength1 - 4)){
			temp2 =atoi((const char* )pcParameterString2);
			if(temp2 <= DEF_CLI_BAUDRATE){
				BOS.clibaudrate =temp2;
				EE_WriteVariable(_EE_CLI_BAUD,(uint16_t )BOS.clibaudrate);
				EE_WriteVariable(_EE_CLI_BAUD + 1,(uint16_t )(BOS.clibaudrate >> 16));
				extraMessage =1;
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"debounce",xParameterStringLength1 - 4)){
			temp16 =atoi((const char* )pcParameterString2);
			if(temp16 >= 1 && temp16 <= USHRT_MAX){
				BOS.buttons.debounce =temp16;
				EE_WriteVariable(_EE_PARAMS_DEBOUNCE,temp16);
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"singleclicktime",xParameterStringLength1 - 4)){
			temp16 =atoi((const char* )pcParameterString2);
			if(temp16 >= 1 && temp16 <= USHRT_MAX){
				BOS.buttons.singleClickTime =temp16;
				EE_WriteVariable(_EE_PARAMS_SINGLE_CLICK,temp16);
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"mininterclicktime",xParameterStringLength1 - 4)){
			temp16 =atoi((const char* )pcParameterString2);
			if(temp16 >= 1 && temp16 <= UCHAR_MAX){
				BOS.buttons.minInterClickTime =temp16;
				EE_WriteVariable(_EE_PARAMS_DBL_CLICK,((uint16_t )BOS.buttons.maxInterClickTime << 8) | (uint16_t )BOS.buttons.minInterClickTime);
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"maxinterclicktime",xParameterStringLength1 - 4)){
			temp16 =atoi((const char* )pcParameterString2);
			if(temp16 >= 1 && temp16 <= UCHAR_MAX){
				BOS.buttons.maxInterClickTime =temp16;
				EE_WriteVariable(_EE_PARAMS_DBL_CLICK,((uint16_t )BOS.buttons.maxInterClickTime << 8) | (uint16_t )BOS.buttons.minInterClickTime);
			}
			else
				result =BOS_ERR_WrongValue;
		}
		else
			result =BOS_ERR_WrongParam;
	}
	else if(!strncmp((const char* )pcParameterString1,"time",4)){
		pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
		pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,3,&xParameterStringLength3);
		pcParameterString4 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,4,&xParameterStringLength4);
		pcParameterString5 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,5,&xParameterStringLength5);
		temp81 =atoi((const char* )pcParameterString2);		// Hours
		temp82 =atoi((const char* )pcParameterString3);		// Minutes
		temp83 =atoi((const char* )pcParameterString4);		// Seconds
		
		if(pcParameterString5 != NULL){
			if(!strncmp((const char* )pcParameterString5,"am",2))
				temp84 =RTC_AM;
			else if(!strncmp((const char* )pcParameterString5,"pm",2))
				temp84 =RTC_PM;
			else
				result =BOS_ERR_WrongValue;
		}
		
		if(result == BOS_OK){
			if(temp81 > 23 || temp82 > 59 || temp83 > 59)
				result =BOS_ERR_WrongValue;
			else{
				GetTimeDate();
				result =BOS_CalendarConfig(BOS.date.month,BOS.date.day,BOS.date.year,BOS.date.weekday,temp83,temp82,temp81,temp84,BOS.daylightsaving);
			}
		}
	}
	else if(!strncmp((const char* )pcParameterString1,"date",4)){
		pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
		pcParameterString3 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,3,&xParameterStringLength3);
		pcParameterString4 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,4,&xParameterStringLength4);
		pcParameterString5 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,5,&xParameterStringLength5);
		temp83 =atoi((const char* )pcParameterString4);		// day
		temp16 =atoi((const char* )pcParameterString5);		// year
		
		if(!strncmp((const char* )pcParameterString2,"monday",6))
			temp81 =MONDAY;
		else if(!strncmp((const char* )pcParameterString2,"tuesday",7))
			temp81 =TUESDAY;
		else if(!strncmp((const char* )pcParameterString2,"wednesday",9))
			temp81 =WEDNESDAY;
		else if(!strncmp((const char* )pcParameterString2,"thursday",8))
			temp81 =THURSDAY;
		else if(!strncmp((const char* )pcParameterString2,"friday",6))
			temp81 =FRIDAY;
		else if(!strncmp((const char* )pcParameterString2,"saturday",8))
			temp81 =SATURDAY;
		else if(!strncmp((const char* )pcParameterString2,"sunday",6))
			temp81 =SUNDAY;
		else
			result =BOS_ERR_WrongValue;
		
		if(!strncmp((const char* )pcParameterString3,"january",7) || !strncmp((const char* )pcParameterString3,"1 ",2))
			temp82 =JANUARY;
		else if(!strncmp((const char* )pcParameterString3,"february",8) || !strncmp((const char* )pcParameterString3,"2 ",2))
			temp82 =FEBRUARY;
		else if(!strncmp((const char* )pcParameterString3,"march",5) || !strncmp((const char* )pcParameterString3,"3 ",2))
			temp82 =MARCH;
		else if(!strncmp((const char* )pcParameterString3,"april",5) || !strncmp((const char* )pcParameterString3,"4 ",2))
			temp82 =APRIL;
		else if(!strncmp((const char* )pcParameterString3,"may",3) || !strncmp((const char* )pcParameterString3,"5 ",2))
			temp82 =MAY;
		else if(!strncmp((const char* )pcParameterString3,"june",4) || !strncmp((const char* )pcParameterString3,"6 ",2))
			temp82 =JUNE;
		else if(!strncmp((const char* )pcParameterString3,"july",4) || !strncmp((const char* )pcParameterString3,"7 ",2))
			temp82 =JULY;
		else if(!strncmp((const char* )pcParameterString3,"august",5) || !strncmp((const char* )pcParameterString3,"8 ",2))
			temp82 =AUGUST;
		else if(!strncmp((const char* )pcParameterString3,"september",9) || !strncmp((const char* )pcParameterString3,"9 ",2))
			temp82 =SEPTEMBER;
		else if(!strncmp((const char* )pcParameterString3,"october",7) || !strncmp((const char* )pcParameterString3,"10",2))
			temp82 =OCTOBER;
		else if(!strncmp((const char* )pcParameterString3,"november",8) || !strncmp((const char* )pcParameterString3,"11",2))
			temp82 =NOVEMBER;
		else if(!strncmp((const char* )pcParameterString3,"december",8) || !strncmp((const char* )pcParameterString3,"12",2))
			temp82 =DECEMBER;
		else
			result =BOS_ERR_WrongValue;
		
		if(result == BOS_OK){
			if(temp83 < 1 || temp83 > 31 || temp16 < 2000 || temp16 > 2100)
				result =BOS_ERR_WrongValue;
			else{
				GetTimeDate();
				result =BOS_CalendarConfig(temp82,temp83,temp16,temp81,BOS.time.seconds,BOS.time.minutes,BOS.time.hours,BOS.time.ampm,BOS.daylightsaving);
			}
		}
	}
	else
		result =BOS_ERR_WrongParam;
	
	/* Respond to the command */
	if(result == BOS_OK){
		pcParameterString1[xParameterStringLength1] =0;		// Get rid of the remaining parameters
		sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,pcParameterString1,pcParameterString2);
		if(extraMessage == 1){
			strcat((char* )pcWriteBuffer,(char* )pcMessageCLI1);
			strcat((char* )pcWriteBuffer,(char* )pcMessageCLI2);
		}
	}
	else if(result == BOS_ERR_WrongParam)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongParam);
	else if(result == BOS_ERR_WrongValue)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongValue);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE getCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0, xParameterStringLength2 =0;
	uint8_t temp8 =0, i =0, j =0;
	
	static const int8_t *pcMessageOK =(int8_t* )"%s\n\r";
	static const int8_t *pcMessageWrongParam =(int8_t* )"Wrong parameter!\n\r";
	static const int8_t *pcMessageWrongValue =(int8_t* )"%s is set to a wrong value!\n\r";
	static const int8_t *pcMessageGroupDoesNotExist =(int8_t* )"%s group does not exist!\n\r";
	static const int8_t *pcMessageGroupExists =(int8_t* )"Group %s members:\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(!strncmp((const char* )pcParameterString1,"bos.",4)){
		if(!strncmp((const char* )pcParameterString1 + 4,"response",xParameterStringLength1 - 4)){
			if(BOSMessaging.response == BOS_RESPONSE_ALL)
				sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,"all");
			else if(BOSMessaging.response == BOS_RESPONSE_MSG)
				sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,"msg");
			else if(BOSMessaging.response == BOS_RESPONSE_NONE)
				sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,"none");
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"trace",xParameterStringLength1 - 4)){
			if(BOSMessaging.trace == TRACE_BOTH)
				sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,"all");
			else if(BOSMessaging.trace == TRACE_MESSAGE)
				sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,"msg");
			else if(BOSMessaging.trace == TRACE_NONE)
				sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,"none");
			else
				result =BOS_ERR_WrongValue;
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"clibaudrate",xParameterStringLength1 - 4)){
			sprintf((char* )pcWriteBuffer,"%d\n\r",BOS.clibaudrate);
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"debounce",xParameterStringLength1 - 4)){
			sprintf((char* )pcWriteBuffer,"%d\n\r",BOS.buttons.debounce);
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"singleclicktime",xParameterStringLength1 - 4)){
			sprintf((char* )pcWriteBuffer,"%d\n\r",BOS.buttons.singleClickTime);
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"mininterclicktime",xParameterStringLength1 - 4)){
			sprintf((char* )pcWriteBuffer,"%d\n\r",BOS.buttons.minInterClickTime);
		}
		else if(!strncmp((const char* )pcParameterString1 + 4,"maxinterclicktime",xParameterStringLength1 - 4)){
			sprintf((char* )pcWriteBuffer,"%d\n\r",BOS.buttons.maxInterClickTime);
		}
		else
			result =BOS_ERR_WrongParam;
	}
	else if(!strncmp((const char* )pcParameterString1,"group",5)){
		pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
		temp8 =0;
		/* Check group exists */
		for(i =0; i < MaxNumOfGroups; i++){
			if(!strcmp((char* )pcParameterString2,groupAlias[i])){
				temp8 =1;
				break;
			}
		}
		/* Group does not exist*/
		if(!temp8){
			sprintf((char* )pcWriteBuffer,(char* )pcMessageGroupDoesNotExist,(char* )pcParameterString2);
			return pdFALSE;
		}
		else{
			sprintf((char* )pcWriteBuffer,(char* )pcMessageGroupExists,(char* )pcParameterString2);
			/* Extract group members */
			for(j =1; j <= N; j++)						// N modules
			    {
				if(InGroup(j,i)){
					sprintf((char* )pcWriteBuffer,"%s#%d\n\r",(char* )pcWriteBuffer,j);
				}
			}
		}
	}
	else
		result =BOS_ERR_WrongParam;
	
	/* Respond to the command */
	if(result == BOS_ERR_WrongParam)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongParam);
	else if(result == BOS_ERR_WrongValue)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageWrongValue,pcParameterString1);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE defaultCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	
	static const int8_t *pcMessageOKParams =(int8_t* )"All parameters set to default values\n\r";
	static const int8_t *pcMessageOKArray =(int8_t* )"Current array topology was removed. Please reboot all modules\n\r";
	static const int8_t *pcMessageWrongValue =(int8_t* )"Wrong value!\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(!strncmp((const char* )pcParameterString1,"params",xParameterStringLength1)){
		memcpy(&BOS,&BOS_default,sizeof(BOS_default));
		SaveEEparams();
		strcpy((char* )pcWriteBuffer,(char* )pcMessageOKParams);
	}
	else if(!strncmp((const char* )pcParameterString1,"array",xParameterStringLength1)){
		/* Broadcast the default array Message */
		SendMessageToModule(BOS_BROADCAST,CODE_DEF_ARRAY,0);
		indMode =IND_TOPOLOGY;
		osDelay(100);
		/* Clear the topology */
		ClearEEportsDir();
#ifndef __N
		ClearROtopology();
#endif
		osDelay(100);
		strcpy((char* )pcWriteBuffer,(char* )pcMessageOKArray);
	}
	else
		result =BOS_ERR_WrongValue;
	
	/* Respond to the command */
	if(result == BOS_ERR_WrongValue)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongValue);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE timeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessage24 =(int8_t* )"Current time is %02d:%02d:%02d-%03d\n\r";
	static const int8_t *pcMessage12 =(int8_t* )"Current time is %02d:%02d:%02d-%03d %s\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	GetTimeDate();
	/* Respond to the command */
	if(BOS.hourformat == 24)
		sprintf((char* )pcWriteBuffer,(char* )pcMessage24,BOS.time.hours,BOS.time.minutes,BOS.time.seconds,BOS.time.msec);
	else if(BOS.hourformat == 12){
		if(BOS.time.ampm == RTC_AM)
			sprintf((char* )pcWriteBuffer,(char* )pcMessage12,BOS.time.hours,BOS.time.minutes,BOS.time.seconds,BOS.time.msec,"AM");
		else if(BOS.time.ampm == RTC_PM)
			sprintf((char* )pcWriteBuffer,(char* )pcMessage12,BOS.time.hours,BOS.time.minutes,BOS.time.seconds,BOS.time.msec,"PM");
	}
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE dateCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageDate =(int8_t* )"Current date is %s\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	GetTimeDate();
	/* Respond to the command */
	sprintf((char* )pcWriteBuffer,(char* )pcMessageDate,GetDateString());
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE setBaudrateCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	BOS_Status result =BOS_OK;
	int8_t *pcParameterString1;
	int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	static const int8_t *pcMessageOK =(int8_t* )"Baudrate for port P%d was set to %d\r\n";
	static const int8_t *pcMessageWrongParam =(int8_t* )"Wrong parameter!\r\n";
	
	uint8_t port;
	uint32_t baudrate;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* 1st parameter for port name: P1 to P6 */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(pcParameterString1[0] == 'p'){
		port =(uint8_t )atol((char* )pcParameterString1 + 1);
	}
	else{
		result =BOS_ERR_WrongValue;
	}
	/* 2nd parameter for baudrate */
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	baudrate =(uint32_t )atol((char* )pcParameterString2);
	
	/* Respond to the command */
	if(BOS_ERR_WrongValue == result){
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrongParam);
	}
	else{
		UpdateBaudrate(port,baudrate);
		sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,baudrate,port);
	}
	
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE uuidCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageUUID =(int8_t* )"MCU UUID is\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Respond to the command */
	sprintf((char* )pcWriteBuffer,"%s",(char* )pcMessageUUID);
	for(uint8_t i =0; i < 3; i++){
#if defined  (STM32F0)
		sprintf((char* )pcWriteBuffer,"%s%08X",(char* )pcWriteBuffer,*(uint32_t* )(MCU_F0_UUID_BASE + i * 4));
#endif
	}
	strcat((char* )pcWriteBuffer,"\r\n");
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE idcodeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageDEVID =(int8_t* )"MCU DEV_ID is %s\n\r";
	static const int8_t *pcMessageREVID =(int8_t* )"%sMCU REV_ID is %d.0\n\r";
	uint16_t dev =0;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Respond to the command */
	dev =HAL_GetDEVID();
	switch(dev){
		case 0x444:
			sprintf((char* )pcWriteBuffer,(char* )pcMessageDEVID,"STM32F03x");
			break;
		case 0x445:
			sprintf((char* )pcWriteBuffer,(char* )pcMessageDEVID,"STM32F04x");
			break;
		case 0x440:
			sprintf((char* )pcWriteBuffer,(char* )pcMessageDEVID,"STM32F05x");
			break;
		case 0x448:
			sprintf((char* )pcWriteBuffer,(char* )pcMessageDEVID,"STM32F07x");
			break;
		case 0x442:
			sprintf((char* )pcWriteBuffer,(char* )pcMessageDEVID,"STM32F09x");
			break;
		default:
			sprintf((char* )pcWriteBuffer,(char* )pcMessageDEVID,"UNKNOWN");
			break;
	}
	sprintf((char* )pcWriteBuffer,(char* )pcMessageREVID,(char* )pcWriteBuffer,HAL_GetREVID() >> 12);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE flashsizeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageFLASH =(int8_t* )"MCU Flash size is %d Kbytes\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Respond to the command */
	sprintf((char* )pcWriteBuffer,(char* )pcMessageFLASH,(*(uint32_t* )(MCU_F0_FLASH_SIZE_BASE)) & 0x0000FFFF);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE snipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageSnipWelcome =(int8_t* )"The following Command Snippets are stored in memory:\n\n\r";
	static const int8_t *pcMessageSnipAction =(int8_t* )"To delete a Snippet, type: del-snip x\n\rTo activate a Snippet, type: \
														act-snip x\n\rTo pause a Snippet, type: pause-snip x\n\n\rwhere x is the Snippet number from the list\n\r";
	static const int8_t *pcMessageSnipStart =(int8_t* )"[%02d] %s\n\r";
	static const int8_t *pcMessageSnipButtonEventClicked =(int8_t* )"%sif b%d.clicked";
	static const int8_t *pcMessageSnipButtonEventDblClicked =(int8_t* )"%sif b%d.double clicked";
	static const int8_t *pcMessageSnipButtonEventPressed =(int8_t* )"%sif b%d.pressed for %d";
	static const int8_t *pcMessageSnipButtonEventReleased =(int8_t* )"%sif b%d.released for %d";
	static const int8_t *pcMessageSnipModuleParamConst =(int8_t* )"%sif %s %s %.1f";
	static const int8_t *pcMessageCmds =(int8_t* )"%s\n\r\t%s";
	static const int8_t *pcMessageEnd =(int8_t* )"\n\rend if\n\n\r";
	char status[2][7] ={"Paused", "Active"};
	static int8_t commands[cmdMAX_INPUT_SIZE];
	float flt1;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Respond to the command */
	writePxMutex(PcPort,(char* )pcMessageSnipWelcome,strlen((char* )pcMessageSnipWelcome),cmd50ms,HAL_MAX_DELAY);
	
	/* Go through all stored Snippets */
	uint8_t count =1;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType)
			sprintf((char* )pcWriteBuffer,(char* )pcMessageSnipStart,count,status[snippets[s].state]);
		
		// Parse conditions
		switch(snippets[s].cond.conditionType){
			case SNIP_COND_BUTTON_EVENT:

				switch(snippets[s].cond.buffer1[1]){
					case CLICKED:
						sprintf((char* )pcWriteBuffer,(char* )pcMessageSnipButtonEventClicked,(char* )pcWriteBuffer,snippets[s].cond.buffer1[0],snippets[s].cmd);
						break;
					case DBL_CLICKED:
						sprintf((char* )pcWriteBuffer,(char* )pcMessageSnipButtonEventDblClicked,(char* )pcWriteBuffer,snippets[s].cond.buffer1[0],snippets[s].cmd);
						break;
					case PRESSED_FOR_X1_SEC:
					case PRESSED_FOR_X2_SEC:
					case PRESSED_FOR_X3_SEC:
						sprintf((char* )pcWriteBuffer,(char* )pcMessageSnipButtonEventPressed,(char* )pcWriteBuffer,snippets[s].cond.buffer1[0],snippets[s].cond.buffer1[2],snippets[s].cmd);
						break;
					case RELEASED_FOR_Y1_SEC:
					case RELEASED_FOR_Y2_SEC:
					case RELEASED_FOR_Y3_SEC:
						sprintf((char* )pcWriteBuffer,(char* )pcMessageSnipButtonEventReleased,(char* )pcWriteBuffer,snippets[s].cond.buffer1[0],snippets[s].cond.buffer1[2],snippets[s].cmd);
						break;
					default:
						break;
				}
				
				break;
				
			case SNIP_COND_MODULE_PARAM_CONST:
				// Get the module parameter, math operator and constant values.
				memcpy((uint8_t* )&flt1,&snippets[s].cond.buffer2,sizeof(float));	// This buffer can be misaligned and cause hardfault on F0
				sprintf((char* )pcWriteBuffer,(char* )pcMessageSnipModuleParamConst,(char* )pcWriteBuffer,modParam[snippets[s].cond.buffer1[1] - 1].paramName,mathStr[snippets[s].cond.mathOperator - 1],flt1);
				break;
				
			default:
				break;
		}
		
		// Parse commands
		while(ParseSnippetCommand(snippets[s].cmd,(int8_t* )&commands) != false){
			sprintf((char* )pcWriteBuffer,(char* )pcMessageCmds,pcWriteBuffer,commands);
			memset(&commands,0x00,strlen((char* )commands));
		}
		
		// Finish and write the buffer
		strcat((char* )pcWriteBuffer,(char* )pcMessageEnd);
		writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),cmd50ms,HAL_MAX_DELAY);
		
		++count;
	}
	
	strcpy((char* )pcWriteBuffer,(char* )pcMessageSnipAction);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE actSnipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageOK =(int8_t* )"Snippet was activated. Type snip to view updated list\n\r";
	static const int8_t *pcMessageWrong =(int8_t* )"The Snippet number was not found\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	BOS_Status result =BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* 1st parameter for Snippet index */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	uint8_t index =(uint8_t )atoi((char* )pcParameterString1);
	
	if(!index || index > numOfRecordedSnippets)
		result =BOS_ERROR;
	
	/* Respond to the command */
	if(result == BOS_OK){
		snippets[index - 1].state = true;
		SaveToRO();
		strcpy((char* )pcWriteBuffer,(char* )pcMessageOK);
	}
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE pauseSnipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageOK =(int8_t* )"Snippet was paused. Type snip to view updated list\n\r";
	static const int8_t *pcMessageWrong =(int8_t* )"The Snippet number was not found\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	BOS_Status result =BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* 1st parameter for Snippet index */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	uint8_t index =(uint8_t )atoi((char* )pcParameterString1);
	
	if(!index || index > numOfRecordedSnippets)
		result =BOS_ERROR;
	
	/* Respond to the command */
	if(result == BOS_OK){
		snippets[index - 1].state = false;
		SaveToRO();
		strcpy((char* )pcWriteBuffer,(char* )pcMessageOK);
	}
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE delSnipCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageOK =(int8_t* )"Snippet was deleted. Type snip to view updated list\n\r";
	static const int8_t *pcMessageWrong =(int8_t* )"The Snippet number was not found\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	BOS_Status result =BOS_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* 1st parameter for Snippet index */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	uint8_t index =(uint8_t )atoi((char* )pcParameterString1);
	
	if(!index || index > numOfRecordedSnippets)
		result =BOS_ERROR;
	
	if(result == BOS_OK){
		// Delete the Snippet
		snippets[index - 1].cond.conditionType =0;
		snippets[index - 1].cond.mathOperator =0;
		memset(snippets[index - 1].cond.buffer1,0,4);
		snippets[index - 1].state = false;
		free(snippets[index - 1].cmd);
		snippets[index - 1].cmd = NULL;
		
		// Reorder remaining Snippets to avoid empty indices
		for(uint8_t s =index; s < numOfRecordedSnippets; s++){
			if(snippets[s].cond.conditionType){
				memcpy(&snippets[s - 1],&snippets[s],sizeof(snippet_t));
				memset(&snippets[s],0,sizeof(snippet_t));
			}
		}
		--numOfRecordedSnippets;
		
		// Write updated list to RO
		SaveToRO();
	}
	
	/* Respond to the command */
	if(result == BOS_OK)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageOK);
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE bridgeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageOK =(int8_t* )"P%d and P%d are bridged together\n\r";
	static const int8_t *pcMessageWrong =(int8_t* )"Wrong syntax\n\r";
	static const int8_t *pcMessageFail =(int8_t* )"Port bridging failed\n\r";
	int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0, xParameterStringLength2 =0;
	BOS_Status result =BOS_OK;
	uint8_t port1, port2;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(pcParameterString1[0] == 'p'){
		port1 =(uint8_t )atol((char* )pcParameterString1 + 1);
	}
	else{
		result =BOS_ERR_WrongParam;
	}
	/* Obtain the 2nd parameter string. */
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	if(pcParameterString2[0] == 'p'){
		port2 =(uint8_t )atol((char* )pcParameterString2 + 1);
	}
	else{
		result =BOS_ERR_WrongParam;
	}
	
	/* Build the bridge */
	if(result == BOS_OK)
		result =Bridge(port1,port2);
	else
		result =BOS_ERR_WrongParam;
	
	/* Return CLI output */
	if(result == BOS_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,port1,port2);
	else if(result == BOS_ERR_WrongParam)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageFail);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE unbridgeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageOK =(int8_t* )"P%d and P%d are un-bridged\n\r";
	static const int8_t *pcMessageWrong =(int8_t* )"Wrong syntax\n\r";
	static const int8_t *pcMessageFail =(int8_t* )"Port unbridging failed\n\r";
	int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0, xParameterStringLength2 =0;
	BOS_Status result =BOS_OK;
	uint8_t port1, port2;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(pcParameterString1[0] == 'p'){
		port1 =(uint8_t )atol((char* )pcParameterString1 + 1);
	}
	else{
		result =BOS_ERR_WrongParam;
	}
	/* Obtain the 2nd parameter string. */
	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	if(pcParameterString2[0] == 'p'){
		port2 =(uint8_t )atol((char* )pcParameterString2 + 1);
	}
	else{
		result =BOS_ERR_WrongParam;
	}
	
	/* Build the bridge */
	if(result == BOS_OK)
		result =Unbridge(port1,port2);
	else
		result =BOS_ERR_WrongParam;
	
	/* Return CLI output */
	if(result == BOS_OK)
		sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,port1,port2);
	else if(result == BOS_ERR_WrongParam)
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageFail);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE testportCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageOK =(int8_t* )"P%d is working correctly\n\r";
	static const int8_t *pcMessageWrong =(int8_t* )"Wrong syntax\n\r";
	static const int8_t *pcMessageFail =(int8_t* )"P%d test failed\n\r";
	static const int8_t *pcMessageWrong1 =(int8_t* )"the port number is wrong\n\r";
	static const int8_t *pcMessageWait =(int8_t* )"Please shorten the next port and press any key to continue testing the next one\n\r\n\r";
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	BOS_Status result =BOS_OK;
	uint8_t portt, ports;
	extern uint8_t UARTRxBufIndex[NumOfPorts];
	char WriteVaule[1] ="H";
	char ReadValue[1];
	int LastEnter =0;
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	if(strcmp((char* )pcParameterString1,"all") == 0){
		if(LastEnter == 0)
			LastEnter =UARTRxBufIndex[PcPort - 1];
		for(ports =1; ports <= NumOfPorts; ports++){
			if(PcPort != ports){
				WriteVaule[0] =rand();
				writePxMutex(ports,WriteVaule,1,10,100);
#ifndef H41R6
				ReadValue[0] =(GetUart(ports)->Instance->RDR);
#else
				ReadValue[0] =(GetUart(ports)->Instance->DR);
#endif
				if(WriteVaule[0] == ReadValue[0])
					result =BOS_OK;
				else
					result =BOS_ERR_Keyword;
				
				if(result == BOS_OK){
					sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,ports);
					writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),10,100);
				}
				else if(result == BOS_ERR_Keyword){
					sprintf((char* )pcWriteBuffer,(char* )pcMessageFail,ports);
					writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),10,100);
				}
				strcpy((char* )pcWriteBuffer,(char* )pcMessageWait);
				writePxMutex(PcPort,(char* )pcWriteBuffer,strlen((char* )pcWriteBuffer),10,100);
				while(UARTRxBuf[PcPort - 1][LastEnter + 1] == 0){
					Delay_ms(1);
				}
				LastEnter++;
			}
		}
	}
	else if(pcParameterString1[0] == 'p'){
		portt =(uint8_t )atol((char* )pcParameterString1 + 1);
		if(portt > 0 && portt <= NumOfPorts){
			if(result == BOS_OK){
				WriteVaule[0] =rand();
				writePxMutex(portt,WriteVaule,1,cmd50ms,100);
#ifndef H41R6
				ReadValue[0] =(GetUart(ports)->Instance->RDR);
#else
				ReadValue[0] =(GetUart(ports)->Instance->DR);
#endif
			}
			if(WriteVaule[0] == ReadValue[0])
				result =BOS_OK;
			else
				result =BOS_ERR_Keyword;
		}
		else{
			result =BOS_ERR_WrongID;
		}
		if(result == BOS_OK)
			sprintf((char* )pcWriteBuffer,(char* )pcMessageOK,portt);
		else if(result == BOS_ERR_WrongID)
			strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong1);
		else if(result == BOS_ERR_Keyword)
			sprintf((char* )pcWriteBuffer,(char* )pcMessageFail,portt);
	}
	else{
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	}
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static portBASE_TYPE ADCReadCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageWrong =(int8_t* )"Wrong Parameter\n\r";	//wrong parameter was entered it's not top nor bottom
	static const int8_t *pcMessageWrong1 =(int8_t* )"Wrong Port number \n\r"; //wrong port number was entered
	int8_t *pcParameterString1;
	int8_t *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 =0;
	portBASE_TYPE xParameterStringLength2 =0;
	BOS_Status result =BOS_OK;
	uint8_t ADCports;
	float ADC_Value_CLI =0;
	float *ADC_Value_CLII;
	char *ADC_Side;
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	
	/* Obtain the 2nd parameter string. */

	pcParameterString2 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,2,&xParameterStringLength2);
	
	if(*pcParameterString1 == '2' || *pcParameterString1 == '3'){
		ADCports =(uint8_t )atol((char* )pcParameterString1);
		
		if(strcmp((char* )pcParameterString2,"top") == 0 || strcmp((char* )pcParameterString2,"bottom") == 0){
			
			if(strcmp((char* )pcParameterString2,"top") == 0)
				ADC_Side ="top";
			else if(strcmp((char* )pcParameterString2,"top") == 0)
				ADC_Side ="bottom";
			
			ADCSelectChannel(ADCports,ADC_Side);
			ReadADCChannel(ADCports,ADC_Side,&ADC_Value_CLI);
			
			strcpy(pcWriteBuffer,(char* )&ADC_Value_CLI);
			
			sprintf(pcWriteBuffer,"ADC_Value=%u \r\n",(uint16_t )ADC_Value_CLI);
			
		}
		else
			strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
		
	}
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong1);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE ReadTempCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	
	float ADC_Value_TEMP =0, ADC_Value_Vref =0;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	ReadTempAndVref(&ADC_Value_TEMP,&ADC_Value_Vref);
	
	strcpy(pcWriteBuffer,(char* )&ADC_Value_TEMP);
	
	sprintf(pcWriteBuffer,"internal temperature is %.2fC \r\n",ADC_Value_TEMP);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE ReadVrefCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	float ADC_Value_TEMP =0, ADC_Value_Vref =0;
	
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	ReadTempAndVref(&ADC_Value_TEMP,&ADC_Value_Vref);
	
	strcpy(pcWriteBuffer,(char* )&ADC_Value_TEMP);
	
	sprintf(pcWriteBuffer,"internal reference voltage is=%.2fV \r\n",ADC_Value_Vref);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE GetReadPrecentageCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	static const int8_t *pcMessageWrong =(int8_t* )"Wrong Port number \n\r"; //wrong port number was entered
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 =0;
	BOS_Status result =BOS_OK;
	uint8_t ADCports;
	float ADC_Value_CLI =0;
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void )xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	
	/* Obtain the 1st parameter string. */

	pcParameterString1 =(int8_t* )FreeRTOS_CLIGetParameter(pcCommandString,1,&xParameterStringLength1);
	
	if(*pcParameterString1 == '2' || *pcParameterString1 == '3'){
		ADCports =(uint8_t )atol((char* )pcParameterString1);
		GetReadPrecentage(ADCports,&ADC_Value_CLI);
		
		sprintf(pcWriteBuffer,"ADC value percentage is=%.2f%% %\r\n",ADC_Value_CLI);
	}
	else
		strcpy((char* )pcWriteBuffer,(char* )pcMessageWrong);
	
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
	
}
/*-----------------------------------------------------------*/
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
