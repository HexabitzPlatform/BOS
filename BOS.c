/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : BOS.c
 Description   : Source code for Bitz Operating System (BOS).

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*Output_Port_Array[__N]:
This array stores all solutions (output ports) to send messages
between modules based on the topology file using FindRoute() function,
so we can read these output ports when needed instead of figuring out the correct port every time.
*/
#ifdef __N
uint8_t Output_Port_Array[__N] = {0};
#endif



/*Flag for CLI Task:
 *
 * Activate_CLI_For_First_Time_Flag:
 * Default value: 0
 * Its Value after receiving '\r' for the first time (setting a port as PcPort): 1
 *
 * Read_In_CLI_Task_Flag:
 * Default value: 0
 * Its value each time a byte is received: 1
 */
uint8_t Activate_CLI_For_First_Time_Flag = 0;
uint8_t Read_In_CLI_Task_Flag = 0;

//The new messages circular buffer:
uint8_t MSG_Buffer_Index_Start[NumOfPorts] = {0};
uint8_t MSG_Buffer_Index_End[NumOfPorts] = {0};
uint8_t MSG_Buffer[NumOfPorts][MSG_COUNT][MSG_MAX_SIZE] = {0};


//Processing message circular buffer:
uint8_t Process_Message_Buffer[MSG_COUNT] = {0};
uint8_t Process_Message_Buffer_Index_Start = 0;
uint8_t Process_Message_Buffer_Index_End = 0;

uint8_t index_input[6]={0};
uint8_t index_process[6]={0};
volatile uint32_t* index_dma[6] ;
uint8_t CLI_Data = 0;
uint8_t port_DMA =0;
/*
 *New private function [inside SendMessageFromPort() ] for sending BOS Messages.
 *instead of writePxDMAMutex (the previous function)
 */

HAL_StatusTypeDef Send_BOS_Message(uint8_t port, uint8_t* buffer, uint16_t n, uint32_t mutexTimeout)
{
	HAL_StatusTypeDef result =HAL_ERROR;

	if(GetUart(port) != NULL){
		/* Wait for the mutex to be available. */
		if(osSemaphoreWait(PxTxSemaphoreHandle[port],mutexTimeout) == osOK){
			for(uint8_t i=0;i<n;i++)
			{
				result =HAL_UART_Transmit_IT(GetUart(port),buffer,1);
				buffer++;
				//Delay_us(500);
				Delay_ms(2);
			}
		}
	}
	Delay_ms(10);// Delay Between Sending Two Messages.
	return result;
}

/*..............User Data from external ports (like USB, Ethernet, BLE ...)..........*/
#ifdef __USER_DATA_BUFFER
uint8_t UserBufferData[USER_RX_BUF_SIZE]={0};
uint8_t UserData=0;
uint8_t indexInputUserDataBuffer = 0;
uint8_t indexProcessUserDataBuffer = 0;
volatile uint32_t* DMACountUserDataBuffer = NULL;


uint8_t GetUserDataCount(void)
{
	indexInputUserDataBuffer = USER_RX_BUF_SIZE - (uint8_t)(*DMACountUserDataBuffer);

	if(indexInputUserDataBuffer== indexProcessUserDataBuffer)
	{
		return 0;
	}

	else
	{
		if(indexInputUserDataBuffer > indexProcessUserDataBuffer)
		{
			return (indexInputUserDataBuffer - indexProcessUserDataBuffer);
		}
		else
		{
			return (indexInputUserDataBuffer - indexProcessUserDataBuffer + USER_RX_BUF_SIZE);
		}
	}
}


BOS_Status GetUserDataByte(uint8_t* pData)
{

	if(GetUserDataCount() != 0)
	{
		if(pData == NULL)
		{
			return BOS_ERROR;
		}

		*pData =  UserBufferData[indexProcessUserDataBuffer];
		indexProcessUserDataBuffer++;
		if(indexProcessUserDataBuffer == USER_RX_BUF_SIZE)
		{
			indexProcessUserDataBuffer = 0;
		}
		return BOS_OK;
	}

	else
	{
		return BOS_ERROR;
	}

}
#endif

/*...................................................................................*/


/* Private and global variables ---------------------------------------------------------*/
BOSMessaging_t BOSMessaging;
BOS_t BOS;
BOS_t BOS_default ={.clibaudrate = DEF_CLI_BAUDRATE,  .buttons.debounce =
DEF_BUTTON_DEBOUNCE, .buttons.singleClickTime = DEF_BUTTON_CLICK, .buttons.minInterClickTime = DEF_BUTTON_MIN_INTER_CLICK, .buttons.maxInterClickTime = DEF_BUTTON_MAX_INTER_CLICK, .daylightsaving =DAYLIGHT_NONE, .hourformat =24, .disableCLI = false};
BOSMessaging_t BOSMessging_default={ .response =
	BOS_RESPONSE_ALL, .trace =TRACE_NONE,.Acknowledgment=false,.trial=once,.received_Acknowledgment=false,
};
uint16_t myPN = modulePN;
uint8_t indMode =IND_OFF;

/* Define module PN strings [available PNs+1][5 chars] */
const char modulePNstring[NUM_OF_MODULE_PN][6] ={"",  "H01R0", "P01R0", "H23R0", "H23R1", "H23R3", "H07R3", "H08R6", "P08R6", "H09R0","H09R9", "H1BR6", "H12R0", "H13R7", "H0FR1", "H0FR6", "H0FR7","H1AR2","H0AR9","H1DR1", "H1DR5", "H0BR4", "H18R0", "H26R0", "H15R0", "H10R4", "H2AR3", "H41R6","H3BR6","H18R1"};

/* Define BOS keywords */
static const char BOSkeywords[NumOfKeywords][4] ={"me", "all", "if", "for"};

const char *monthStringAbreviated[] ={"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

static const char *weekdayString[] ={"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
bool ACK_FLAG=0,rejected_FLAG=0;
//static const char mathStr[NUM_MATH_OPERATORS][3] = {"==", ">", "<", ">=", "<=", "!="};

/* Define long messages -------------------------------------------------------*/
char *pcBootloaderUpdateMessage ="\n\rThis module will be forced into bootloader mode.\n\rPlease use the \"STM Flash Loader Demonstrator\" \
								  utility to update the firmware.\n\r\n\t*** Important ***\n\rIf this module is connected directly to PC please close this port first.\n\r";

char *pcRemoteBootloaderUpdateMessage ="\n\rModule %d will be forced into bootloader mode.";
char *pcRemoteBootloaderUpdateViaPortMessage ="\n\rRemote update via module %d, port P%d will be triggered.";

char *pcRemoteBootloaderUpdateWarningMessage ="\n\rPlease use the \"STM Flash Loader Demonstrator\" utility to update the firmware.\
											   \n\r\n\t*** Important ***\n\r- If this module is connected directly to PC please close this port first.\n\r\
											   - You must power cycle the entire array after the update is finished.\n\r";

/* Define CLI command list*/
typedef struct xCOMMAND_INPUT_LIST {
	const CLI_Command_Definition_t *pxCommandLineDefinition;
	struct xCOMMAND_INPUT_LIST *pxNext;
} CLI_Definition_List_Item_t;
extern CLI_Definition_List_Item_t xRegisteredCommands;
uint8_t numOfBosCommands;

/* Number of modules in the array */
#ifndef __N
uint8_t N =1;
uint8_t myID =0;
#else
	uint8_t N = __N;
	uint8_t myID = _module;
#endif

/* Routing and topology ....................................................................... */
uint8_t portStatus[NumOfPorts + 1] ={0};
uint16_t neighbors[NumOfPorts][2] ={0};
uint16_t neighbors2[NumOfPorts][2] ={0};
uint16_t bcastRoutes[MaxNumOfModules] ={0}; /* P1 is LSB */
bool AddBcastPayload = false;
uint8_t dstGroupID =BOS_BROADCAST;
char groupAlias[MaxNumOfGroups][MaxLengthOfAlias + 1] ={0};
#ifndef __N
uint16_t array[MaxNumOfModules][MaxNumOfPorts + 1] ={{0}}; /* Array topology */
uint8_t routeDist[MaxNumOfModules] ={0};
uint8_t routePrev[MaxNumOfModules] ={0};
char moduleAlias[MaxNumOfModules + 1][MaxLengthOfAlias + 1] ={0}; /* moduleAlias[0] used to store alias for module 0 */
uint8_t broadcastResponse[MaxNumOfModules] ={0};
uint16_t groupModules[MaxNumOfModules] ={0}; /* Group 0 (LSB) to Group 15 (MSB) */
#else
	uint8_t routeDist[__N] = {0};
	uint8_t routePrev[__N] = {0};
	char moduleAlias[__N+1][MaxLengthOfAlias+1] = {0};
	uint8_t broadcastResponse[__N] = {0};
	uint16_t groupModules[__N] = {0};									/* Group 0 (LSB) to Group 15 (MSB) */
#endif
/* ............................................................................................. */

/* Buffers and communication.................................................................... */
uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE] ={0};	// Buffer for received messages and ready to be parsed
char message[MAX_MESSAGE_SIZE] ={0};					// Buffer to construct a message to be sent
uint8_t messageLength[NumOfPorts] ={0};
uint8_t messageParams[MAX_PARAMS_PER_MESSAGE] ={0};
char cRxedChar =0;
uint8_t longMessage =0;
uint16_t longMessageLastPtr =0;
static char pcUserMessage[80];
BOS_Status responseStatus =BOS_OK;
uint8_t bcastID =0;			// Counter for unique broadcast ID
uint8_t PcPort =0;
uint8_t BOS_initialized =0;
uint32_t BOS_var_reg[MAX_BOS_VARS];	// BOS variables register: Bits 31-16: variable RAM address shift from SRAM_BASE, Bits 15-8: status. Bits 7-0: format.
uint64_t remoteBuffer =0;
varFormat_t remoteVarFormat =FMT_UINT8;
uint8_t CLI_LOW_Baudrate_Flag =0; 		//Flag for Lower CLI baudrate is set
/* ............................................................................................. */

/* Messaging tasks.............................................................................. */
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
/* ............................................................................................. */

/* UARTcmd task */
extern TaskHandle_t xCommandConsoleTaskHandle;

/* Variables exported internally */
extern uint8_t numOfRecordedSnippets;
extern uint8_t crcBuffer[MAX_MESSAGE_SIZE];

/* Private function prototypes -----------------------------------------------*/

/* Explore related APIs */
uint8_t minArr(uint8_t *arr,uint8_t *Q);
uint8_t QnotEmpty(uint8_t *Q);
void NotifyMessagingTask(uint8_t port);
uint8_t SaveToRO(void);
#ifndef __N
uint8_t ClearROtopology(void);
#endif
/*--------------------------------------------------------------*/

/* Load form EEPROM related APIs */
BOS_Status LoadROsnippets(void);
BOS_Status LoadROtopology(void);
BOS_Status LoadEEportsDir(void);
BOS_Status LoadEEalias(void);
BOS_Status LoadEEgroup(void);
BOS_Status LoadEEstreams(void);
BOS_Status LoadEEbuttons(void);
BOS_Status LoadEEparams(void);
/*--------------------------------------------------------------*/

/* Save to EEPROM related APIs */
BOS_Status SaveEEportsDir(void);
BOS_Status SaveEEalias(void);
BOS_Status SaveEEgroup(void);
BOS_Status SaveEEstreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src1,uint8_t dst1,uint8_t src2,uint8_t dst2,uint8_t src3,uint8_t dst3);
BOS_Status SaveEEparams(void);

/*--------------------------------------------------------------*/
BOS_Status ClearEEportsDir(void);
BOS_Status SetupDMAStreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src,uint8_t dst);
//void StreamTimerCallback( TimerHandle_t xTimerStream );
uint8_t IsFactoryReset(void);
void EE_FormatForFactoryReset(void);
BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin);

BOS_Status WriteToRemote(uint8_t module,uint32_t localAddress,uint32_t remoteAddress,varFormat_t format,uint32_t timeout,uint8_t force);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
BOS_Status User_MessagingParser(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Module exported internal functions */
extern uint8_t IsModuleParameter(char *name);
extern void Module_Peripheral_Init(void);
extern void TIM_USEC_Init(void);
extern void TIM_MSEC_Init(void);
extern BOS_Status RTC_Init(void);
extern Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

extern bool ParseSnippetCommand(char *snippetBuffer,int8_t *cliBuffer);

const char *pcParamsHelpString[NumOfParamsHelpStrings] ={"\r\nBOS.response: all, message, cli, none\r\n", "\r\nBOS.trace: all, message, response, none\r\n", "BOS.clibaudrate: CLI baudrate. Default is 921600. This affects all ports. If you change this value, \
           you must connect to a CLI port on each startup to restore other array ports into default baudrate\r\n", "BOS.debounce: 1 ............ 65536 msec\r\n", "BOS.singleclicktime: 1 ..... 65536 msec\r\n", "BOS.mininterclicktime: 1 ... 255 msec\r\n", "BOS.maxinterclicktime: 1 ... 255 msec\r\n"};

/* ............................................................................................. */

/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 -----------------------------------------------------------------------
 */

/* --- Load stored variables,Ports directions,Module's name , Group...etc. from emulated EEPROM ---------------------------------------*/

// --- Load stored variables from emulated EEPROM
void LoadEEvars(void){
	/* Load array topology */
#ifndef __N
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

/* --- Load array topology stored in Flash RO --- */
BOS_Status LoadROtopology(void){
	BOS_Status result =BOS_OK;
	uint16_t add =2, temp =0;
	
	/* Load number of modules */
	temp =(*(__IO uint16_t* )(RO_START_ADDRESS));
	
	if(temp == 0xFFFF)				// Memory has been erased
	{
		N =1;
		myID =0;
		return BOS_MEM_ERASED;
	}
	else{
		N =(uint8_t )(temp >> 8);
		if(N == 0)
			N =1;
		myID =(uint8_t )temp;
		
		/* Load topology */
		for(uint8_t i =1; i <= N; i++){
			for(volatile uint8_t j =0; j <= MaxNumOfPorts; j++){
				array[i - 1][j] =(*(__IO uint16_t* )(RO_START_ADDRESS + add));
				add +=2;
			}
		}
	}
	
	return result;
}

/* --- Load array ports directions stored in EEPROM --- */

BOS_Status LoadEEportsDir(void){
	BOS_Status result =BOS_OK;
	
	for(uint8_t i =1; i <= N; i++){
		EE_ReadVariable(_EE_PORT_DIR_BASE + i - 1,&arrayPortsDir[i - 1]);
		
		if((i + _EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result =BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module alias stored in EEPROM --- */
BOS_Status LoadEEalias(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	
	for(uint8_t i =0; i <= N; i++)				// N+1 module aliases
	    {
		for(uint8_t j =1; j <= MaxLengthOfAlias; j +=2){
			EE_ReadVariable(_EE_ALIAS_BASE + add,&temp);
			moduleAlias[i][j] =(uint8_t )temp;
			moduleAlias[i][j - 1] =(uint8_t )(temp >> 8);
			add++;
		}
		moduleAlias[i][MaxLengthOfAlias] ='\0';
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module groups stored in EEPROM --- */
BOS_Status LoadEEgroup(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	uint8_t i =0;
	
	/* Load group members */
	for(i =0; i < N; i++)			// N modules
	    {
		EE_ReadVariable(_EE_GROUP_MODULES_BASE + add,&groupModules[i]);
		add++;
	}
	
	/* Load group alias */
	for(i =0; i < MaxNumOfGroups; i++)		// MaxNumOfGroups group aliases
	    {
		for(uint8_t j =1; j <= MaxLengthOfAlias; j +=2){
			EE_ReadVariable(_EE_GROUP_ALIAS_BASE + add,&temp);
			groupAlias[i][j] =(uint8_t )temp;
			groupAlias[i][j - 1] =(uint8_t )(temp >> 8);
			add++;
		}
		groupAlias[i][MaxLengthOfAlias] ='\0';
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load module DMA streams --- */

BOS_Status LoadEEstreams(void){
	BOS_Status result =BOS_OK;
	uint16_t temp1 =0, temp2 =0, status1 =0, status2 =0;
	uint8_t direction =0;
	uint32_t count =0, timeout =0;
	static uint8_t src1, dst1, src2, dst2, src3, dst3;
	
	/* Direction */
	status1 =EE_ReadVariable(_EE_DMA_STREAM_BASE,&temp1);
	if(!status1){
		direction =(uint8_t )temp1;
	}
	
	/* Count */
	status1 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 1,&temp1);
	status2 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 2,&temp2);
	if(!status1 && !status2){
		count =((uint32_t )temp1 << 16) + temp2;
	}
	
	/* Timeout */
	status1 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 3,&temp1);
	status2 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 4,&temp2);
	if(!status1 && !status2){
		timeout =((uint32_t )temp1 << 16) + temp2;
	}
	
	/* src1 | dst1 */
	status1 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 5,&temp1);
	if(!status1){
		src1 =(uint8_t )(temp1 >> 8);
		dst1 =(uint8_t )temp1;
	}
	
	/* src2 | dst2 */
	status1 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 6,&temp1);
	if(!status1){
		src2 =(uint8_t )(temp1 >> 8);
		dst2 =(uint8_t )temp1;
	}
	
	/* src3 | dst3 */
	status1 =EE_ReadVariable(_EE_DMA_STREAM_BASE + 7,&temp1);
	if(!status1){
		src3 =(uint8_t )(temp1 >> 8);
		dst3 =(uint8_t )temp1;
	}
	
	/* Activate the DMA streams */
	if(src1 && dst1)
		SetupDMAStreams(direction,count,timeout,src1,dst1);
	if(src2 && dst2)
		SetupDMAStreams(direction,count,timeout,src2,dst2);
	if(src3 && dst3)
		SetupDMAStreams(direction,count,timeout,src3,dst3);
	
	return result;
}

// --- Load module parameters from emulated EEPROM. If erased, load defaults --- */

BOS_Status LoadEEparams(void){
	BOS_Status result =BOS_OK;
	uint16_t temp1, temp2, status1, status2;
	
	/* Read params base - BOS response and BOS trace */
	status1 =EE_ReadVariable(_EE_PARAMS_BASE,&temp1);
	/* Found the variable (EEPROM is not cleared) */
	if(!status1){
		BOSMessaging.response =(uint8_t )temp1;
		BOSMessaging.trace =(traceOptions_t )(temp1 >> 8);
		/* Couldn't find the variable, load default config */
	}
	else{
		BOSMessaging.response =BOSMessging_default.response;
		BOSMessaging.trace =BOSMessging_default.trace;

	}
	/* Read params base - BOS response and BOS trace */
	status1 =EE_ReadVariable(_EE_PARAMS_Messaging,&temp1);

	if(!status1){
		BOSMessaging.Acknowledgment =(bool )(temp1 >>15);
		BOSMessaging.trial =(uint16_t)(temp1 >> 1);
		/* Couldn't find the variable, load default config */
	}
	else{
		BOSMessaging.Acknowledgment=BOSMessging_default.Acknowledgment;
		BOSMessaging.trial=BOSMessging_default.trial;
	}
	/* Read Button debounce */
	status1 =EE_ReadVariable(_EE_PARAMS_DEBOUNCE,&temp1);
	if(!status1)
		BOS.buttons.debounce =temp1;
	else
		BOS.buttons.debounce =BOS_default.buttons.debounce;
	
	/* Read Button single click time */
	status1 =EE_ReadVariable(_EE_PARAMS_SINGLE_CLICK,&temp1);
	if(!status1)
		BOS.buttons.singleClickTime =temp1;
	else
		BOS.buttons.singleClickTime =BOS_default.buttons.singleClickTime;
	
	/* Read Button double click time (min and max inter-click) */
	status1 =EE_ReadVariable(_EE_PARAMS_DBL_CLICK,&temp1);
	if(!status1){
		BOS.buttons.minInterClickTime =(uint8_t )temp1;
		BOS.buttons.maxInterClickTime =(uint8_t )(temp1 >> 8);
	}
	else{
		BOS.buttons.minInterClickTime =BOS_default.buttons.minInterClickTime;
		BOS.buttons.maxInterClickTime =BOS_default.buttons.maxInterClickTime;
	}
	
	/* Read CLI baudrate */
	status1 =EE_ReadVariable(_EE_CLI_BAUD,&temp1);
	status2 =EE_ReadVariable(_EE_CLI_BAUD + 1,&temp2);
	if(!status1 && !status2){
		BOS.clibaudrate =(uint32_t )temp1 | (((uint32_t )temp2) << 16);
	}
	else if(CLI_LOW_Baudrate_Flag)
		BOS.clibaudrate = CLI_BAUDRATE_1;
	else
		BOS.clibaudrate =BOS_default.clibaudrate;
	
	/* Read RTC hourformat and daylightsaving */
	status1 =EE_ReadVariable(_EE_PARAMS_RTC,&temp1);
	if(!status1){
		BOS.daylightsaving =(int8_t )temp1;
		BOS.hourformat =(uint8_t )(temp1 >> 8);
	}
	else{
		BOS.hourformat =24;
		BOS.daylightsaving =DAYLIGHT_NONE;
	}
	
	/* Read disableCLI */
	status1 =EE_ReadVariable(_EE_PARAMS_DISABLE_CLI,&temp1);
	/* Found the variable (EEPROM is not cleared) */
	if(!status1){
		BOS.disableCLI =(uint8_t )temp1;
		/* Couldn't find the variable, load default config */
	}
	else{
		BOS.disableCLI =BOS_default.disableCLI;
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Load button definitions and events from EEPROM --- */
BOS_Status LoadEEbuttons(void){
	BOS_Status result =BOS_OK;
	uint16_t temp16 =0, status1 =0;
	uint8_t temp8 =0;
	
	for(uint8_t i =0; i <= NumOfPorts; i++){
		status1 =EE_ReadVariable(_EE_BUTTON_BASE + 4 * (i),&temp16);
		
		if(!status1)									// This variable exists
		{
			temp8 =(uint8_t )(temp16 >> 8);
			if(((temp8 >> 4) == i + 1) && ((temp8 & 0x0F) != NONE))									// This is same port and button type is not none
			{
				button[i + 1].type =temp8 & 0x0F;
				button[i + 1].events =(uint8_t )temp16;
				EE_ReadVariable(_EE_BUTTON_BASE + 4 * (i) + 1,&temp16);
				button[i + 1].pressedX1Sec =(uint8_t )(temp16 >> 8);
				button[i + 1].releasedY1Sec =(uint8_t )temp16;
				EE_ReadVariable(_EE_BUTTON_BASE + 4 * (i) + 2,&temp16);
				button[i + 1].pressedX2Sec =(uint8_t )(temp16 >> 8);
				button[i + 1].releasedY2Sec =(uint8_t )temp16;
				EE_ReadVariable(_EE_BUTTON_BASE + 4 * (i) + 3,&temp16);
				button[i + 1].pressedX3Sec =(uint8_t )(temp16 >> 8);
				button[i + 1].releasedY3Sec =(uint8_t )temp16;
				/* Setup the button and its events */
				AddPortButton(button[i + 1].type,i + 1);
				SetButtonEvents(i + 1,(button[i + 1].events & BUTTON_EVENT_CLICKED),((button[i + 1].events & BUTTON_EVENT_DBL_CLICKED) >> 1),button[i + 1].pressedX1Sec,button[i + 1].pressedX2Sec,button[i + 1].pressedX3Sec,button[i + 1].releasedY1Sec,button[i + 1].releasedY2Sec,button[i + 1].releasedY3Sec,BUTTON_EVENT_MODE_CLEAR);
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/
/* --- Load Command Snippets stored in Flash RO  ---*/
BOS_Status LoadROsnippets(void){
	uint8_t i =0;
	int currentAdd = RO_MID_ADDRESS;
	char *snipBuffer =(char* )malloc(cmdMAX_INPUT_SIZE);
	if(snipBuffer == NULL)
		return BOS_MEM_FULL;
	
	// Exit if no recorded Snippets
	if(*(uint8_t* )currentAdd != 0xFE)
		return BOS_ERROR;
	
	/* Load Snippets */
	for(uint8_t s =0; s < MAX_SNIPPETS; s++){
		// Load conditions starting at RO_MID_ADDRESS
		for(i =0; i < sizeof(snippet_t); i++)
			snipBuffer[i] =(*(__IO uint8_t* )(currentAdd++));
		memcpy((uint8_t* )&snippets[s],(uint8_t* )&snipBuffer[1],sizeof(snippet_t));
		memset(snipBuffer,0,sizeof(snippet_t));
		i =0;
		// Load commands until you get next 0xFE
		currentAdd=currentAdd+20;
		while(*(uint8_t* )currentAdd != 0xFE && *(uint8_t* )currentAdd != 0xFF && i < cmdMAX_INPUT_SIZE){
			snipBuffer[i] =*(uint8_t* )currentAdd;
			++currentAdd;
			++i;
		}
		if(snipBuffer[i - 1] != 0)
			++i;	// String termination char was not recorded, then add one
		// Allocate buffer for the Snippet commands
		snippets[s].cmd =(char* )malloc(i);
		if(snippets[s].cmd == NULL){
			memset(&snippets[s],0,sizeof(snippet_t));
			free(snipBuffer);
			return BOS_ERR_SNIP_MEM_FULL;
		}
		else{
			// Copy the command
			memcpy(snippets[s].cmd,snipBuffer,i);
			++numOfRecordedSnippets;		// Record a successful Snippet
			memset(snipBuffer,0,i);
		}
		// Exit if no more Snippets
		if(*(uint8_t* )currentAdd != 0xFE)
			break;
	}
	
	free(snipBuffer);
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/

/* --- Save Ports directions,Module's name, Group...etc. from emulated EEPROM ---------------------------------------*/

/* Save to EEPROM related APIs */

/* --- Save array ports directions in EEPROM ---*/

BOS_Status SaveEEportsDir(void){
	BOS_Status result =BOS_OK;
	
	for(uint8_t i =1; i <= N; i++){
		if(arrayPortsDir[i - 1])
			EE_WriteVariable(_EE_PORT_DIR_BASE + i - 1,arrayPortsDir[i - 1]);
		
		if((i + _EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result =BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/
// --- Save module alias in EEPROM ---
BOS_Status SaveEEalias(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	
	for(uint8_t i =0; i <= N; i++)				// N+1 module aliases
	    {
		if(moduleAlias[i][0]){
			for(uint8_t j =1; j <= MaxLengthOfAlias; j +=2){
				temp =(uint16_t )(moduleAlias[i][j - 1] << 8) + moduleAlias[i][j];
				EE_WriteVariable(_EE_ALIAS_BASE + add,temp);
				add++;
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Save module groups in EEPROM ---*/

BOS_Status SaveEEgroup(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	uint8_t i =0;
	
	/* Save group members */
	for(i =0; i < N; i++)			// N modules
	    {
		if(groupModules[i]){
			EE_WriteVariable(_EE_GROUP_MODULES_BASE + add,groupModules[i]);
			add++;
		}
	}
	
	/* Save group alias */
	for(i =0; i < MaxNumOfGroups; i++)		// MaxNumOfGroups group aliases
	    {
		if(groupAlias[i][0]){
			for(uint8_t j =1; j <= MaxLengthOfAlias; j +=2){
				temp =(uint16_t )(groupAlias[i][j - 1] << 8) + groupAlias[i][j];
				EE_WriteVariable(_EE_GROUP_ALIAS_BASE + add,temp);
				add++;
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Save DMA streams to emulated EEPROM.*/
BOS_Status SaveEEstreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src1,uint8_t dst1,uint8_t src2,uint8_t dst2,uint8_t src3,uint8_t dst3){
	BOS_Status result =BOS_OK;
	
	EE_WriteVariable(_EE_DMA_STREAM_BASE,direction); /* Direction */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 1,((uint16_t )(count >> 8))); /* Count high half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 2,((uint16_t )count)); /* Count low half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 3,((uint16_t )(timeout >> 8))); /* Timeout high half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 4,((uint16_t )timeout)); /* Timeout low half-word */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 5,((uint16_t )(src1 << 8)) + (uint16_t )dst1); /* src1 | dst1 */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 6,((uint16_t )(src2 << 8)) + (uint16_t )dst2); /* src1 | dst1 */
	EE_WriteVariable(_EE_DMA_STREAM_BASE + 7,((uint16_t )(src3 << 8)) + (uint16_t )dst3); /* src1 | dst1 */
	
	return result;
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* --- Save module parameters to emulated EEPROM. ---*/

BOS_Status SaveEEparams(void){
	BOS_Status result =BOS_OK;
	
	/* Save params base - BOS response & BOS trace */
	EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )BOSMessaging.trace << 5) | (uint16_t )BOSMessaging.response);
	
	EE_WriteVariable(_EE_PARAMS_Messaging,((uint16_t )BOSMessaging.Acknowledgment << 15) | (uint16_t )BOSMessaging.trial);

	/* Save Button debounce */
	EE_WriteVariable(_EE_PARAMS_DEBOUNCE,BOS.buttons.debounce);
	
	/* Save Button single click time */
	EE_WriteVariable(_EE_PARAMS_SINGLE_CLICK,BOS.buttons.singleClickTime);
	
	/* Save Button double click time (min and max inter-click) */
	EE_WriteVariable(_EE_PARAMS_DBL_CLICK,((uint16_t )BOS.buttons.maxInterClickTime << 8) | (uint16_t )BOS.daylightsaving);
	
	/* Save CLI baudrate */
	EE_WriteVariable(_EE_CLI_BAUD,(uint16_t )BOS.clibaudrate);
	EE_WriteVariable(_EE_CLI_BAUD + 1,(uint16_t )(BOS.clibaudrate >> 16));
	
	/* Save RTC hour format and daylight saving */
	EE_WriteVariable(_EE_PARAMS_RTC,((uint16_t )BOS.hourformat << 8) | (uint16_t )BOS.buttons.minInterClickTime);
	
	/* Save disableCLI */
	EE_WriteVariable(_EE_PARAMS_DISABLE_CLI,(uint16_t )BOS.disableCLI);
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Clear array ports directions in EEPROM --- */

BOS_Status ClearEEportsDir(void){
	BOS_Status result =BOS_OK;
	
	memset(arrayPortsDir,0,sizeof(arrayPortsDir));
	
	for(uint8_t i =1; i <= N; i++){
		if(arrayPortsDir[i - 1])
			EE_WriteVariable(_EE_PORT_DIR_BASE + i - 1,arrayPortsDir[i - 1]);
		
		if((i + _EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result =BOS_ERR_EEPROM;
	}
	
	return result;
}

/*-----------------------------------------------------------*/
//TODO change loction of the API
// --- Format emulated EEPROM for a factory reset
void EE_FormatForFactoryReset(void){
	/* Check if EEPROM was just formated? */
	/* Flag address (STM32F09x) - Last 4 words of SRAM */
	if(*((unsigned long* )0x20007FF0) == 0xBEEFDEAD){
		// Do nothing
	}
	else{
		if(EE_Format() == HAL_OK){
			/* Set flag for formated EEPROM */
			*((unsigned long* )0x20007FF0) =0xBEEFDEAD;
		}
	}
	
}

/*----------------------------------------------------------------*/

/* --- Check if booting into lower CLI baudrate:
 - Connect P1 TXD and P2 RXD to boot CLI at 115200
 */
uint8_t IsLowerCLIbaud(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t P1_TX_Port, P1_RX_Port, P2_TX_Port, P2_RX_Port;
	uint16_t P1_TX_Pin, P1_RX_Pin, P2_TX_Pin, P2_RX_Pin;
	
	/* -- Setup GPIOs -- */

	/* Get GPIOs */
	GetPortGPIOs(P1,&P1_TX_Port,&P1_TX_Pin,&P1_RX_Port,&P1_RX_Pin);
	GetPortGPIOs(P2,&P2_TX_Port,&P2_TX_Pin,&P2_RX_Port,&P2_RX_Pin);
	
	/* P1 TXD */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin =P1_TX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef* )P1_TX_Port,&GPIO_InitStruct);
	
	/* P2 RXD */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Pin =P2_RX_Pin;
	HAL_GPIO_Init((GPIO_TypeDef* )P2_RX_Port,&GPIO_InitStruct);
	
	/* Check for lower CLI baudrate conditions */
	HAL_GPIO_WritePin((GPIO_TypeDef* )P1_TX_Port,P1_TX_Pin,GPIO_PIN_RESET);
	Delay_ms_no_rtos(5);
	if(HAL_GPIO_ReadPin((GPIO_TypeDef* )P2_RX_Port,P2_RX_Pin) == RESET){
		HAL_GPIO_WritePin((GPIO_TypeDef* )P1_TX_Port,P1_TX_Pin,GPIO_PIN_SET);
		Delay_ms_no_rtos(5);
		if(HAL_GPIO_ReadPin((GPIO_TypeDef* )P2_RX_Port,P2_RX_Pin) == SET){
			return 1;
		}
	}
	
	return 0;
}

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/* --- Check if this string is a local module parameter or event. Returns parameter index+1
 */
uint8_t IsModuleParameter(char *name){
	for(uint8_t i =0; i < NUM_MODULE_PARAMS; i++){
		if(!strcmp(name,(const char* )(modParam[i].paramName)))
			return i + 1;
	}
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Check if this string is a math operator and return its enum
 */
uint8_t IsMathOperator(char *string){
	for(uint8_t i =0; i < NUM_MATH_OPERATORS; i++){
		if(!strcmp(string,"="))
			return MATH_EQUAL;
		else if(!strcmp(string,">"))
			return MATH_GREATER;
		else if(!strcmp(string,"<"))
			return MATH_SMALLER;
		else if(!strcmp(string,">="))
			return MATH_GREATER_EQUAL;
		else if(!strcmp(string,"<="))
			return MATH_SMALLER_EQUAL;
		else if(!strcmp(string,"!="))
			return MATH_NOT_EQUAL;
	}
	return 0;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |						BOS related APIS	  							  |
 -------------------------------------------------------------------------
 */

/* --- BitzOS initialization. 
 */
void BOS_Init(void){

/*
 *Storing Values inside Output_Port_Array[] using FindRoute() Function
*/
#ifdef __N
	for(uint8_t i = 1;i <= __N;i++)
	{
		if(myID == i) Output_Port_Array[i-1] = 0;
		else Output_Port_Array[i-1] = FindRoute(myID, i);
	}
#endif

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
	if(IsFactoryReset()){
		/* Format EEPROM once */
		EE_FormatForFactoryReset();
		
		/* Software reset */
		NVIC_SystemReset();
	}
	
	/* Check if booting at lower CLI baudrate */
	if(IsLowerCLIbaud()){
		CLI_LOW_Baudrate_Flag =1;
		/* Initialize the module */
		Delay_ms_no_rtos(50);		// Give other modules time to finish factory reset and baudrate check
		Module_Peripheral_Init();
		
		BOS.clibaudrate = CLI_BAUDRATE_1;
		/* Update all ports to lower baudrate */
		for(uint8_t port =1; port <= NumOfPorts; port++){
			UpdateBaudrate(port,BOS.clibaudrate);
		}
	}
	else{
		/* Initialize the module with default baudrate */
		Delay_ms_no_rtos(50);		// Give other modules time to finish factory reset and baudrate check
		Module_Peripheral_Init();
	}
	
	/* Load stored EEPROM variables */
	LoadEEvars();
	
	/* If no pre-defined topology, initialize ports direction */
#ifndef __N
	UpdateMyPortsDir();
#endif	
	
	/* Start backend messaging DMAs */
	SetupMessagingRxDMAs();
	
	/* Startup indicator sequence */
	if(myID == 0) /* Native module */
	{
		IND_ON();
		Delay_ms_no_rtos(500);IND_OFF();
	}
	else /* Non-native module */
	{
		IND_ON();
		Delay_ms_no_rtos(500);
		IND_OFF();
		Delay_ms_no_rtos(100);
		IND_ON();
		Delay_ms_no_rtos(100);
		IND_OFF();
	}
	
	/* Reset UART overrun errors in case other modules were already transmitting on startup */
	ResetUartORE();
	
	BOS_initialized =1;
}
void Module_Init(void){

	/* Reset all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize BitzOS */
	BOS_Init();

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();
}
/*-----------------------------------------------------------*/
//#ifndef _N
///* --- Explore the array and create its topology (executed only by master)
//*/
//BOS_Status Explore(void)
//{
//	BOS_Status result = BOS_OK;
//	uint8_t currentID = 0, lastID = 0, temp1 = 0, temp2 = 0, i = 0, j = 0, p = 0, port = 0;
//	uint16_t temp16 = 0;
//	
//	myID = 1; 		/* Master ID */
//	
//	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
//	
//	for (uint8_t port=1 ; port<=NumOfPorts ; port++) {
//		if (port != PcPort)	SwapUartPins(GetUart(port), REVERSED);
//	}
//	ExploreNeighbors(PcPort); indMode = IND_TOPOLOGY;
//	
//	/* >>> Step 2 - Assign IDs to new modules & update the topology array */
//	
//	/* Step 2a - Assign IDs to new modules */
//	currentID = 1;
//	for (port=1 ; port<=NumOfPorts ; port++) 
//	{
//		if (neighbors[port-1][0])
//		{
//			/* New ID */
//			messageParams[1] = ++currentID;
//			N = currentID;			/* Update number of modules in the array */
//			/* Inform module to change ID */
//			messageParams[0] = 0;		/* change own ID */
//			SendMessageFromPort(port, 0, 0, CODE_MODULE_ID, 3);			
//			/* Modify neighbors table */
//			neighbors[port-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(neighbors[port-1][0]);
//			osDelay(10);
//		}
//	}
//	
//	/* Step 2b - Update master topology array */
//	array[0][0]	= myPN;					
//	for (port=1 ; port<=NumOfPorts ; port++) 
//	{
//		if (neighbors[port-1][0])
//		{
//			temp16 = neighbors[port-1][0];
//			temp1 = (uint8_t)(temp16>>8);										/* Neighbor ID */
//			temp2 = (uint8_t)(neighbors[port-1][0]);				/* Neighbor port */
//			/* Module 1 (master) */
//			array[0][port] = ( temp1 << 3 ) | temp2;				/* Neighbor ID | Neighbor port */
//			/* Rest of the neighbors */
//			array[temp1-1][0]	= neighbors[port-1][1];				/* Neighbor PN */
//			array[temp1-1][temp2] = ( myID << 3 ) | port;		/* Module 1 ID | Module 1 port */
//		}
//	}		
//	
//	/* Step 2c - Ask neighbors to update their topology array */
//	for (i=2 ; i<=currentID ; i++) 
//	{
//		memcpy(messageParams, array, (size_t) (currentID*(MaxNumOfPorts+1)*2) );
//		SendMessageToModule(i, CODE_TOPOLOGY, (size_t) (currentID*(MaxNumOfPorts+1)*2));
//		osDelay(60);
//	}
//	
//	
//	/* >>> Step 3 - Ask each new module to explore and repeat */
//	
//	while (lastID != currentID)
//	{
//		/* Update lastID */
//		lastID = currentID;
//		
//		/* Scan all discovered modules */
//		for (i=2 ; i<=currentID ; i++) 
//		{
//			/* Step 3a - Ask the module to reverse ports */
//			for (uint8_t p=1 ; p<=MaxNumOfPorts ; p++) {
//				messageParams[p-1] = REVERSED;
//			}
//			messageParams[MaxNumOfPorts] = NORMAL;		/* Make sure the inport is not reversed */
//			SendMessageToModule(i, CODE_PORT_DIRECTION, MaxNumOfPorts+1);
//			osDelay(10);
//			
//			/* Step 3b - Ask the module to explore adjacent neighbors */
//			SendMessageToModule(i, CODE_EXPLORE_ADJ, 0);
//			osDelay(100);		
//		
//			/* Step 3c - Assign IDs to new modules */
//			for (j=1 ; j<=MaxNumOfPorts ; j++) 
//			{
//				temp16 = neighbors2[j-1][0];		/* Neighbor ID */
//				temp1 = (uint8_t)(temp16>>8);											
//				if (temp16 != 0 && temp1 == 0)			/* UnIDed module */
//				{
//					/* New ID */
//					messageParams[1] = ++currentID;		
//					N = currentID;			/* Update number of modules in the array */
//					/* Modify neighbors table */
//					neighbors2[j-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(neighbors2[j-1][0]);
//					/* Ask the module to ID its yet unIDed neighbors */
//					messageParams[0] = 1;			/* change neighbor ID */
//					messageParams[2] = j;		/* neighbor port */
//					SendMessageToModule(i, CODE_MODULE_ID, 3);
//					osDelay(10);
//				}
//			}
//			
//			/* Step 3d - Update master topology array */
//			for (j=1 ; j<=MaxNumOfPorts ; j++) 
//			{
//				if (neighbors2[j-1][0])
//				{	
//					temp16 = neighbors2[j-1][0];
//					temp1 = (uint8_t)(temp16>>8);										/* Neighbor ID */
//					temp2 = (uint8_t)(neighbors2[j-1][0]);					/* Neighbor port */		
//					if (temp1 != 1)			/* Execlude the master */
//					{
//						/* Update module i section */
//						if (array[i-1][j] == 0) {
//							array[i-1][j] = ( temp1 << 3 ) | temp2;				/* Neighbor ID | Neighbor port */
//						}
//						/* Update module i neighbors */
//						if (array[temp1-1][temp2] == 0) {
//							array[temp1-1][0]	= neighbors2[j-1][1];				/* Neighbor PN */
//							array[temp1-1][temp2] = ( i << 3 ) | j;				/* Module i ID | Module i port */								
//						}
//					}
//				}
//			}	
//			
//			/* Reset neighbors2 array */
//			memset(neighbors2, 0, sizeof(neighbors2) );
//			
//			/* Step 3e - Ask all discovered modules to update their topology array */
//			for (j=2 ; j<=currentID ; j++) 
//			{
//				memcpy(messageParams, array, (size_t) (currentID*(MaxNumOfPorts+1)*2) );
//				SendMessageToModule(j, CODE_TOPOLOGY, (size_t) (currentID*(MaxNumOfPorts+1)*2));
//				osDelay(60);
//			}
//		}
//	}
//	
//	/* >>> Step 4 - Make sure all connected modules have been discovered */
//	
//	ExploreNeighbors(PcPort);
//	/* Check for any unIDed neighbors */
//	for (i=1 ; i<=NumOfPorts ; i++) 
//	{
//		temp16 = neighbors[i-1][0];		/* Neighbor ID */
//		temp1 = (uint8_t)(temp16>>8);											
//		if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
//			result = BOS_ERR_UnIDedModule;
//		}		
//	}
//	/* Ask other modules for any unIDed neighbors */
//	for (i=2 ; i<=currentID ; i++) 
//	{
//		SendMessageToModule(i, CODE_EXPLORE_ADJ, 0);
//		osDelay(100);	
//		/* Check for any unIDed neighbors */
//		for (j=1 ; j<=MaxNumOfPorts ; j++) 
//		{
//			temp16 = neighbors2[j-1][0];		/* Neighbor ID */
//			temp1 = (uint8_t)(temp16>>8);											
//			if (temp16 != 0 && temp1 == 0) {		/* UnIDed module */
//				result = BOS_ERR_UnIDedModule;
//			}
//		}				
//	}
//	
//	
//	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
//	
//	if (result == BOS_OK)
//	{	
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
//			temp1 = route[NumberOfHops(i)-1];				/* previous module = route[Number of hops - 1] */
//			temp2 = FindRoute(i, temp1);
//			/* Is the inport reversed? */
//			if ( (temp1 == i) || (messageParams[temp2-1] == REVERSED) )
//				messageParams[MaxNumOfPorts] = REVERSED;		/* Make sure the inport is reversed */
//			
//			/* Step 5d - Update module ports directions */
//			SendMessageToModule(i, CODE_PORT_DIRECTION, MaxNumOfPorts+1);
//			osDelay(10);			
//		}			
//	
//		/* Step 5e - Update master ports > all normal */
//		for (port=1 ; port<=NumOfPorts ; port++) {
//			if (port != PcPort)	SwapUartPins(GetUart(port), NORMAL);
//		}
//	}
//	
//			
//	/* >>> Step 6 - Test new port directions by pinging all modules */
//	
//	if (result == BOS_OK) 
//	{		
//		osDelay(100);
//		BOS.response = BOS_RESPONSE_MSG;		// Enable response for pings
//		for (i=2 ; i<=N ; i++) 
//		{
//			SendMessageToModule(i, CODE_PING, 0);
//			osDelay(300*NumberOfHops(i));	
//			//osDelay(100);
//			if (responseStatus == BOS_OK)
//				result = BOS_OK;
//			else if (responseStatus == BOS_ERR_NoResponse)
//				result = BOS_ERR_NoResponse;
//		}
//	}
//	
//	/* >>> Step 7 - Save all (topology and port directions) in RO/EEPROM */
//	
//	if (result == BOS_OK)
//	{
//		/* Save data in the master */
//		SaveToRO();
//		SaveEEportsDir();
//		osDelay(100);
//		/* Ask other modules to save their data too */
//		SendMessageToModule(BOS_BROADCAST, CODE_EXP_EEPROM, 0);
//	}	
//	return result;
//}
//#endif
/*-----------------------------------------------------------*/
#ifndef __N
/* --- Explore adjacent neighbors 
 */
BOS_Status ExploreNeighbors(uint8_t ignore){
	BOS_Status result =BOS_OK;
	
	/* Send Hi messages to adjacent neighbors */
	for(uint8_t port =1; port <= NumOfPorts; port++){
		if(port != ignore){
			/* This module info */
			messageParams[0] =(uint8_t )(myPN >> 8);
			messageParams[1] =(uint8_t )myPN;
			messageParams[2] =port;
			/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
			SendMessageFromPort(port,0,0,CODE_HI,3);
			/* Minimum delay between two consequetive SendMessage commands (with response) */
			osDelay(10);
		}
	}
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Find array broadcast routes starting from a given module (Takes about 50 usec) */
BOS_Status FindBroadcastRoutes(uint8_t src){
	BOS_Status result =BOS_OK;
	uint8_t p =0, m =0, level =0, untaged =0;
	uint8_t modules[N];			// Todo: Optimize to make bit-wise
	
	/* 1. Initialize modules list and broadcast routes */

	for(m =0; m < N; m++){
		modules[m] =0;
		bcastRoutes[m] =0;
	}
	modules[src - 1] =++level;					// Tag the source
	
	/* 2. Source module should send to all neighbors */

	++level;												// Move one level
	
	for(p =1; p <= 6; p++){
		if(array[src - 1][p]){
			bcastRoutes[src - 1] |=(0x01 << (p - 1));
			modules[(array[src - 1][p] >> 3) - 1] =level;												// Tag this module as already broadcasted-to
		}
	}
	
	/* 3. Starting from source neighbors, check all other modules we haven't broadcasted-to yet, one by one */

	do{
		untaged =0;								// Reset the untaged counter
		++level;											// Move one level
		
		for(m =0; m < N; m++)					// Scan all modules in the list
		    {
			if(modules[m] == (level - 1))					// This module is already broadcasted-to from the previous level
			{
				for(p =1; p <= 6; p++)					// Check all neighbors if they're not already broadcasted-to
				    {
					if(array[m][p] && (modules[(array[m][p] >> 3) - 1] == 0)) // Found an untaged module
					{
						bcastRoutes[m] |=(0x01 << (p - 1));
						modules[(array[m][p] >> 3) - 1] =level; // Tag this module as already broadcasted-to
						++untaged;
					}
				}
			}
		}
	} while(untaged);
	
	return result;
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
uint8_t FindRoute(uint8_t sourceID,uint8_t desID){
#ifdef __N
	uint8_t Q[__N] = {0};		// All nodes initially in Q (unvisited nodes)
#else
	uint8_t Q[50] ={0};			// All nodes initially in Q (unvisited nodes)
#endif
	
	uint8_t alt =0;
	uint8_t u =0;
	uint8_t v =0;
	uint8_t j =0;
	
	memset(route,0,sizeof(route));
	routeDist[sourceID - 1] =0;               // Distance from source to source
	routePrev[sourceID - 1] =0; // Previous node in optimal path initialization undefined
	
	/* Check adjacent neighbors first! */
	for(int col =1; col <= 6; col++){
		if(array[sourceID - 1][col] && ((array[sourceID - 1][col] >> 3) == desID)){
			routeDist[desID - 1] =1;
			route[0] =desID;
			return col;
		}
	}
	
	/* Initialization */
	for(int i =1; i <= N; i++){
		if(i != sourceID) // Where i has not yet been removed from Q (unvisited nodes)
		{
			routeDist[i - 1] =0xFF; // Unknown distance function from source to i
			routePrev[i - 1] =0;   // Previous node in optimal path from source
		}
	}
	
	/* Algorithm */
	while(!QnotEmpty(Q)){
		u =minArr(routeDist,Q) + 1;				// Source node in first case
		if(u == desID){
			goto finishedRoute;
		}
		else
			Q[u - 1] =1;									// Remove u from Q
			
		/* For each neighbor v where v is still in Q. */
		for(uint8_t n =1; n <= 6; n++)      		// Check all module ports
		    {
			if(array[u - 1][n])		// There's a neighbor v at this port n
			{
				v =(array[u - 1][n] >> 3);
				if(!Q[v - 1])								// v is still in Q
				{
					alt =routeDist[u - 1] + 1;					// Add one hop
					if(alt < routeDist[v - 1]) // A shorter path to v has been found
					{
						routeDist[v - 1] =alt;
						routePrev[v - 1] =u;
					}
				}
			}
		}
	}
	
	finishedRoute:

	/* Build the virtual route */
	while(routePrev[u - 1])   // Construct the shortest path with a stack route
	{
		route[j++] =u;          			// Push the vertex onto the stack
		u =routePrev[u - 1];           		// Traverse from target to source
	}
	
	/* Check which port leads to the correct module */
	for(int col =1; col <= 6; col++){
		if(array[sourceID - 1][col] && ((array[sourceID - 1][col] >> 3) == route[routeDist[desID - 1] - 1])){
			return col;
		}
	}
	
	return 0;
}

/* --- Used by FoundRoute: Find the index of the minimum module in dist that is still unvisited
 */
uint8_t minArr(uint8_t *arr,uint8_t *Q){
	uint8_t smallest =0xFF;
	uint8_t index =0;
	
	/* Consider first element as smallest */
	if(!Q[0])						// Not visited yet
		smallest =arr[0];
	
	for(int i =0; i < N; i++){
		if((arr[i] < smallest) && !Q[i]){
			smallest =arr[i];
			index =i;
		}
	}
	
	return index;
}

/*-----------------------------------------------------------*/

/* --- Used by FoundRoute: Check if Q is empty (all modules have been visited)
 */
uint8_t QnotEmpty(uint8_t *Q){
	char temp =1;
	
	for(int i =0; i < N; i++){
		temp &=Q[i];
	}
	
	return temp;
}
/*-----------------------------------------------------------*/

/* --- Display array topology in human-readable format through module port --- 
 */
void DisplayTopology(uint8_t port){
	/* Print table header */
	sprintf(pcUserMessage,"\n\r(Module:Port)\t\t");
	writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
	HAL_MAX_DELAY);
	for(uint8_t i =1; i <= NumOfPorts; i++){
		sprintf(pcUserMessage,"P%d\t",i);
		writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
		HAL_MAX_DELAY);
	}
	writePxMutex(port,"\n\n\r",3,cmd50ms,HAL_MAX_DELAY);
	
	/* Print each row */
	for(uint8_t row =0; row < N; row++){
		sprintf(pcUserMessage,"Module %d:\t",row + 1);
		writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
		HAL_MAX_DELAY);
		/* Module PN */
		strncpy(pcUserMessage,modulePNstring[(array[row][0])],5);
		writePxMutex(port,pcUserMessage,5,cmd50ms,HAL_MAX_DELAY);
		writePxMutex(port,"\t",1,cmd50ms,HAL_MAX_DELAY);
		/* Connections */
		for(uint8_t col =1; col <= NumOfPorts; col++){
			if(!array[row][col])
				sprintf(pcUserMessage,"%d\t",0);
			else
				sprintf(pcUserMessage,"%d:%d\t",(array[row][col] >> 3),(array[row][col] & 0x07));
			writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
			HAL_MAX_DELAY);
		}
		writePxMutex(port,"\n\r",2,cmd50ms,HAL_MAX_DELAY);
	}
	
	writePxMutex(port,"\n",1,cmd50ms,HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/

/* --- Display ports directions in human-readable format through module port --- 
 */
void DisplayPortsDir(uint8_t port){
	sprintf(pcUserMessage,"\n\rThese ports are reversed:");
	writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
	HAL_MAX_DELAY);
	
	for(uint8_t i =1; i <= N; i++){
		for(uint8_t p =1; p <= MaxNumOfPorts; p++){
			if((arrayPortsDir[i - 1] & (0x8000 >> (p - 1)))) /* Port is reversed */
			{
				sprintf(pcUserMessage,"\n\rModule %d : P%d",i,p);
				writePxMutex(port,pcUserMessage,strlen(pcUserMessage),
				cmd50ms,HAL_MAX_DELAY);
			}
		}
	}
	
	sprintf(pcUserMessage,"\n\n\rAll other ports are normal\n\r");
	writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
	HAL_MAX_DELAY);
}

/*-----------------------------------------------------------*/

/* --- Display a description of current module status (Firmware, Ports, P2P DMAs) --- 
 */
void DisplayModuleStatus(uint8_t port){
	int8_t *pcOutputString;
	uint16_t temp =0;
	
	/* Obtain the address of the output buffer. */
	pcOutputString =FreeRTOS_CLIGetOutputBuffer();
	
	strcpy((char* )pcOutputString,"");
	
	sprintf(pcUserMessage,"\n\r*** Module %d Status ***\n",myID);
	strcat((char* )pcOutputString,pcUserMessage);
	sprintf(pcUserMessage,"\n\rConnected via port: P%d\n\r",PcPort);
	strcat((char* )pcOutputString,pcUserMessage);
	
	/* Firmware */
	sprintf(pcUserMessage,"\n\rFirmware version: %d.%d.%d",_firmMajor,
	_firmMinor,_firmPatch);
	strcat((char* )pcOutputString,pcUserMessage);
	sprintf(pcUserMessage,"\n\rFirmware date:    %s",_firmDate);
	strcat((char* )pcOutputString,pcUserMessage);
	sprintf(pcUserMessage,"\n\rFirmware time:    %s\n\r",_firmTime);
	strcat((char* )pcOutputString,pcUserMessage);
	
	/* Ports */
	sprintf(pcUserMessage,"\n\rPorts Status:\n\n\r");
	strcat((char* )pcOutputString,pcUserMessage);
	for(uint8_t i =1; i <= NumOfPorts; i++){
		sprintf(pcUserMessage,"P%d: ",i);
		strcat((char* )pcOutputString,pcUserMessage);
		switch(portStatus[i]){
			case FREE:
				sprintf(pcUserMessage,"Free\n\r");
				break;
			case MSG:
				sprintf(pcUserMessage,"Receiving messages\n\r");
				break;
			case STREAM:
				sprintf(pcUserMessage,"Streaming\n\r");
				break;
			case CLI:
				sprintf(pcUserMessage,"Receiving user commands\n\r");
				break;
			case PORTBUTTON:
				sprintf(pcUserMessage,"Connected to a button/switch\n\r");
				break;
			default:
				break;
		}
		strcat((char* )pcOutputString,pcUserMessage);
	}
	
	/* P2P DMAs */
	sprintf(pcUserMessage,"\n\rDMA Streams Status:\n\r");
	strcat((char* )pcOutputString,pcUserMessage);
	for(char i =1; i <= 6; i++){
		if(streamDMA[i - 1].Instance == 0){
			sprintf(pcUserMessage,"\n\rStreaming DMA %d is free",i);
			strcat((char* )pcOutputString,pcUserMessage);
		}
		else{
			sprintf(pcUserMessage,"\n\rStreaming DMA %d is streaming from P%d to P%d",i,GetPort(streamDMA[i - 1].Parent),GetPort(dmaStreamDst[i - 1]));
			strcat((char* )pcOutputString,pcUserMessage);
		}
	}
	strcat((char* )pcOutputString,"\n\r");
	
	/* Ports direction */
	strcat((char* )pcOutputString,"\n\rThese ports are reversed: ");
	temp =strlen((char* )pcOutputString);
	for(uint8_t p =1; p <= NumOfPorts; p++){
		if((arrayPortsDir[myID - 1] & (0x8000 >> (p - 1)))) /* Port is reversed */
		{
			sprintf(pcUserMessage,"P%d ",p);
			strcat((char* )pcOutputString,pcUserMessage);
		}
	}
	if(temp == strlen((char* )pcOutputString)){ /* All ports are normal */
		strcat((char* )pcOutputString,"None");
	}
	strcat((char* )pcOutputString,"\n\r");
	
	/* Display output */
	if(port)
		writePxMutex(port,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms,HAL_MAX_DELAY);
	
}

/*-----------------------------------------------------------*/

/* --- Extract module ID from it's alias, ID string or keyword --- 
 */
int16_t GetID(char *string){
	uint8_t id =0, i =0;
	
	if(!strcmp(string,"me")) /* Check keywords */
		return myID;
	else if(!strcmp(string,"all"))
		return BOS_BROADCAST;
	else if(string[0] == '#') /* Check IDs */
	{
		id =atol(string + 1);
		if(id > 0 && id <= N)
			return id;
		else if(id == myID)
			return myID;
		else
			return BOS_ERR_WrongID;
	}
	else /* Check alias */
	{
		/* Check module alias */
		for(i =0; i < N; i++){
			if(!strcmp(string,moduleAlias[i]) && (*string != 0))
				return (i);
		}
		
		/* Check group alias */
		for(i =0; i < MaxNumOfGroups; i++){
			if(!strcmp(string,groupAlias[i]))
				return (BOS_MULTICAST | (i << 8));
		}
		
		return BOS_ERR_WrongName;
	}
	
}

/*-----------------------------------------------------------*/

/* --- Name a module with an alias --- 
 */
BOS_Status NameModule(uint8_t module,char *alias){
	BOS_Status result =BOS_OK;
	int i =0;
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;
	
	/* 1. Check module alias with keywords */
	for(i =0; i < NumOfKeywords; i++){
		if(!strcmp(alias,BOSkeywords[i]))
			return BOS_ERR_Keyword;
	}
	
	/* 2. Check module alias with other module aliases */
	for(i =1; i < N; i++){
		if(!strcmp(alias,moduleAlias[i]))
			return BOS_ERR_ExistingAlias;
	}
	
	/* 3. Check module alias with group aliases */
	for(i =0; i < MaxNumOfGroups; i++){
		if(!strcmp(alias,groupAlias[i]))
			return BOS_ERR_ExistingAlias;
	}
	
	/* 4. Check alias with BOS and module commands */
	for(pxCommand =&xRegisteredCommands; pxCommand != NULL; pxCommand =pxCommand->pxNext){
		pcRegisteredCommandString =pxCommand->pxCommandLineDefinition->pcCommand;
		xCommandStringLength =strlen((const char* )pcRegisteredCommandString);
		
		if(!strncmp(alias,(const char* )pcRegisteredCommandString,xCommandStringLength)){
			return BOS_ERR_ExistingCmd;
		}
	}
	
	/* 5. Module alias is unique */
	strcpy(moduleAlias[module],alias);
	
	/* 6. Share new module alias with other modules */

	/* 7. Save new alias to emulated EEPROM */
	result =SaveEEalias();
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Add a module to this group --- 
 */
BOS_Status AddModuleToGroup(uint8_t module,char *group){
	BOS_Status result =BOS_OK;
	int i =0, j =0;
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;
	
	/* Check alias with other group aliases */

	for(i =0; i < MaxNumOfGroups; i++){
		/* This group already exists */
		if(!strcmp(group,groupAlias[i])){
			/* 1. Add this module to the group */
			groupModules[module - 1] |=(0x0001 << i);
			
			/* 2. Save group to emulated EEPROM -- Should call this manually */
			//result = SaveEEgroup();			
			return result;
		}
	}
	
	/* This is a new group - Verify alias and create the group */

	/* 1. Check group alias with keywords */
	for(j =0; j < NumOfKeywords; j++){
		if(!strcmp(group,BOSkeywords[j]))
			return BOS_ERR_Keyword;
	}
	
	/* 2. Check group alias with module aliases */
	for(j =1; j < N; j++){
		if(!strcmp(group,moduleAlias[j]))
			return BOS_ERR_ExistingAlias;
	}
	
	/* 3. Check group alias with BOS and module commands */
	for(pxCommand =&xRegisteredCommands; pxCommand != NULL; pxCommand =pxCommand->pxNext){
		pcRegisteredCommandString =pxCommand->pxCommandLineDefinition->pcCommand;
		xCommandStringLength =strlen((const char* )pcRegisteredCommandString);
		
		if(!strncmp(group,(const char* )pcRegisteredCommandString,xCommandStringLength)){
			return BOS_ERR_ExistingCmd;
		}
	}
	
	/* 4. Group alias is unique - copy to first empty location */
	for(i =0; i < MaxNumOfGroups; i++){
		if(!groupAlias[i][0]){
			strcpy(groupAlias[i],group);
			break;
		}
	}
	
	/* 5. Add this module to the new group */
	groupModules[module - 1] |=(0x0001 << i);
	
	/* 6. Share new group with other modules */

	/* 7. Save new group to emulated EEPROM - Should call this manually */
	//result = SaveEEgroup();			
	return result;
}



/*-----------------------------------------------------------*/

/* --- Write a value to a remote module.
 module: Remote module ID.
 localAddress: Local memory address (RAM or Flash).
 remoteAddress: Remote memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
 format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
 timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
 force: Put 1 to force full-page erase before writing to Flash.
 */
BOS_Status WriteToRemote(uint8_t module,uint32_t localAddress,uint32_t remoteAddress,varFormat_t format,uint32_t timeout,uint8_t force){
	
	uint8_t response;
	uint16_t code;
	
	/* Check whether response is enabled or disabled */
	response =BOSMessaging.response;
	if(timeout)
		BOSMessaging.response = BOS_RESPONSE_MSG;
	else
		BOSMessaging.response = BOS_RESPONSE_NONE;
	
	/* Check if a force write is needed */
	if(force)
		code = CODE_WRITE_REMOTE_FORCE;
	else
		code = CODE_WRITE_REMOTE;
	
	/* Writing to a BOS var */
	if(remoteAddress < FLASH_BASE){
		messageParams[0] =remoteAddress;			// Send BOS variable index
		messageParams[1] =format;						// Send local format
		/* Send variable value based on local format */
		switch(format){
			case FMT_BOOL:
			case FMT_UINT8:
				messageParams[2] =*(__IO uint8_t* )localAddress;
				SendMessageToModule(module,CODE_WRITE_REMOTE,3);
				break;
			case FMT_INT8:
				messageParams[2] =*(__IO int8_t* )localAddress;
				SendMessageToModule(module,CODE_WRITE_REMOTE,3);
				break;
			case FMT_UINT16:
				messageParams[2] =(uint8_t )((*(__IO uint16_t* )localAddress) >> 0);
				messageParams[3] =(uint8_t )((*(__IO uint16_t* )localAddress) >> 8);
				SendMessageToModule(module,CODE_WRITE_REMOTE,4);
				break;
			case FMT_INT16:
				messageParams[2] =(uint8_t )((*(__IO int16_t* )localAddress) >> 0);
				messageParams[3] =(uint8_t )((*(__IO int16_t* )localAddress) >> 8);
				SendMessageToModule(module,CODE_WRITE_REMOTE,4);
				break;
			case FMT_UINT32:
				messageParams[2] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 0);
				messageParams[3] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 8);
				messageParams[4] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 16);
				messageParams[5] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 24);
				SendMessageToModule(module,CODE_WRITE_REMOTE,6);
				break;
			case FMT_INT32:
				messageParams[2] =(uint8_t )((*(__IO int32_t* )localAddress) >> 0);
				messageParams[3] =(uint8_t )((*(__IO int32_t* )localAddress) >> 8);
				messageParams[4] =(uint8_t )((*(__IO int32_t* )localAddress) >> 16);
				messageParams[5] =(uint8_t )((*(__IO int32_t* )localAddress) >> 24);
				SendMessageToModule(module,CODE_WRITE_REMOTE,6);
				break;
			case FMT_FLOAT:
				messageParams[2] =*(__IO uint8_t* )(localAddress + 0);
				messageParams[3] =*(__IO uint8_t* )(localAddress + 1);
				messageParams[4] =*(__IO uint8_t* )(localAddress + 2);
				messageParams[5] =*(__IO uint8_t* )(localAddress + 3);
				messageParams[6] =*(__IO uint8_t* )(localAddress + 4);
				messageParams[7] =*(__IO uint8_t* )(localAddress + 5);
				messageParams[8] =*(__IO uint8_t* )(localAddress + 6);
				messageParams[9] =*(__IO uint8_t* )(localAddress + 7); // You cannot bitwise floats
				SendMessageToModule(module,CODE_WRITE_REMOTE,10);
				break;
			default:
				break;
		}
	}
	/* Writing to a memory address */
	else{
		messageParams[0] =0;
		messageParams[1] =format;							// Local format
		messageParams[2] =(uint8_t )(remoteAddress >> 24);
		messageParams[3] =(uint8_t )(remoteAddress >> 16); // Remote address
		messageParams[4] =(uint8_t )(remoteAddress >> 8);
		messageParams[5] =(uint8_t )remoteAddress;
		/* Send variable value based on local format */
		switch(format){
			case FMT_BOOL:
			case FMT_UINT8:
				messageParams[6] =*(__IO uint8_t* )localAddress;
				SendMessageToModule(module,code,7);
				break;
			case FMT_INT8:
				messageParams[6] =*(__IO int8_t* )localAddress;
				SendMessageToModule(module,code,7);
				break;
			case FMT_UINT16:
				messageParams[6] =(uint8_t )((*(__IO uint16_t* )localAddress) >> 0);
				messageParams[7] =(uint8_t )((*(__IO uint16_t* )localAddress) >> 8);
				SendMessageToModule(module,code,8);
				break;
			case FMT_INT16:
				messageParams[6] =(uint8_t )((*(__IO int16_t* )localAddress) >> 0);
				messageParams[7] =(uint8_t )((*(__IO int16_t* )localAddress) >> 8);
				SendMessageToModule(module,code,8);
				break;
			case FMT_UINT32:
				messageParams[6] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 0);
				messageParams[7] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 8);
				messageParams[8] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 16);
				messageParams[9] =(uint8_t )((*(__IO uint32_t* )localAddress) >> 24);
				SendMessageToModule(module,code,10);
				break;
			case FMT_INT32:
				messageParams[6] =(uint8_t )((*(__IO int32_t* )localAddress) >> 0);
				messageParams[7] =(uint8_t )((*(__IO int32_t* )localAddress) >> 8);
				messageParams[8] =(uint8_t )((*(__IO int32_t* )localAddress) >> 16);
				messageParams[9] =(uint8_t )((*(__IO int32_t* )localAddress) >> 24);
				SendMessageToModule(module,code,10);
				break;
			case FMT_FLOAT:
				messageParams[6] =*(__IO uint8_t* )(localAddress + 0);
				messageParams[7] =*(__IO uint8_t* )(localAddress + 1);
				messageParams[8] =*(__IO uint8_t* )(localAddress + 2);
				messageParams[9] =*(__IO uint8_t* )(localAddress + 3);
				messageParams[10] =*(__IO uint8_t* )(localAddress + 4);
				messageParams[11] =*(__IO uint8_t* )(localAddress + 5);
				messageParams[12] =*(__IO uint8_t* )(localAddress + 6);
				messageParams[13] =*(__IO uint8_t* )(localAddress + 7); // You cannot bitwise floats
				SendMessageToModule(module,code,14);
				break;
			default:
				break;
		}
	}
	
	/* Restore response settings to default */
	BOSMessaging.response =response;
	
	/* If confirmation is requested, wait for it until timeout */
	if(timeout){
		uint32_t t0 =HAL_GetTick();
		while((responseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){};
		return responseStatus;
	}
	
	return BOS_OK;
}
/* --- Read a variable from a remote module. 
 This API returns a pointer to the remote value. Cast this pointer to match the appropriate format.
 If the returned value is NULL, then remote variable does not exist or remote module is not responsive.
 module: Remote module ID.
 remoteAddress: Remote value memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to read BOS variables with unknown addresses.
 remoteFormat (output): Pointer to format of remote BOS variable.
 timeout: Read timeout in msec.
 */
uint32_t* ReadRemoteVar(uint8_t module,uint32_t remoteAddress,varFormat_t *remoteFormat,uint32_t timeout){
	/* Reset local buffer */
	remoteBuffer = REMOTE_BOS_VAR;
	
	/* Send the Message */
	messageParams[0] =remoteAddress + REMOTE_BOS_VAR; // Send BOS variable index
	SendMessageToModule(module,CODE_READ_REMOTE,1);
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	while((responseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){
	};
	
	/* Return the read value address */
	if(responseStatus == BOS_OK){
		/* Return the remote var format */
		*remoteFormat =remoteVarFormat;
		
		return ((uint32_t* )&remoteBuffer);
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
uint32_t* ReadRemoteMemory(uint8_t module,uint32_t remoteAddress,varFormat_t requestedFormat,uint32_t timeout){
	/* Reset local buffer */
	remoteBuffer = REMOTE_MEMORY_ADD;
	
	/* Send the Message */
	messageParams[0] = REMOTE_MEMORY_ADD;
	messageParams[1] =requestedFormat;						// Requested format
	messageParams[2] =(uint8_t )(remoteAddress >> 24);
	messageParams[3] =(uint8_t )(remoteAddress >> 16); // Remote address
	messageParams[4] =(uint8_t )(remoteAddress >> 8);
	messageParams[5] =(uint8_t )remoteAddress;
	SendMessageToModule(module,CODE_READ_REMOTE,6);
	remoteBuffer =requestedFormat;	// Set a flag that we requested a memory location
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	while((responseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){
	};
	
	/* Return the read value address */
	if(responseStatus == BOS_OK)
		return ((uint32_t* )&remoteBuffer);
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
uint32_t* ReadRemoteParam(uint8_t module,char *paramString,varFormat_t *remoteFormat,uint32_t timeout){
	/* Reset local buffer */
	remoteBuffer = REMOTE_MODULE_PARAM;
	
	/* Send the Message */
	messageParams[0] = REMOTE_MODULE_PARAM;
	memcpy(&messageParams[1],paramString,strlen(paramString)); // copy BOS parameter index to location
	SendMessageToModule(module,CODE_READ_REMOTE,strlen(paramString) + 1);
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	while((responseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){
	};
	
	/* Return the read value address */
	if(responseStatus == BOS_OK){
		/* Return the remote var format */
		*remoteFormat =remoteVarFormat;
		
		return ((uint32_t* )&remoteBuffer);
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
BOS_Status WriteRemote(uint8_t module,uint32_t localAddress,uint32_t remoteAddress,varFormat_t format,uint32_t timeout){
	return WriteToRemote(module,localAddress,remoteAddress,format,timeout,0);
}

/*-----------------------------------------------------------*/

/* --- Write a value to a remote module and force full-page erase when writing to Flash. 
 module: Remote module ID.
 localAddress: Local memory address (RAM or Flash).
 remoteAddress: Remote memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
 format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
 timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
 */
BOS_Status WriteRemoteForce(uint8_t module,uint32_t localAddress,uint32_t remoteAddress,varFormat_t format,uint32_t timeout){
	return WriteToRemote(module,localAddress,remoteAddress,format,timeout,1);
}

/*-----------------------------------------------------------*/

/* --- Assign an index to a new BOS variable. BOS variables must be global or static to ensure we don't refernce a stack address.
 */
uint8_t AddBOSvar(varFormat_t format,uint32_t address){
	for(uint8_t v =0; v < MAX_BOS_VARS; v++){
		if((BOS_var_reg[v] & 0x000F) == 0)		// Index not assigned yet
		{
			BOS_var_reg[v] =format + ((address - SRAM_BASE) << 16);
			return (v + 1);
		}
	}
	
	return 0;			// Memory full
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*  Read three variables From ModBus module , by sending a request
 to MB module , which responed and send the variables and store it
 in MBmessageParams.
 dst : When creating topology , you have to consider :
 MB module ID is 1.
 BOS module ID is 2.
 rank : represents a set of 3 variables.
 ModBus module has nine variables.
 0 is var1 , var2 and var3 in mb-module.
 1 is var4 , var5 and var6 in mb-module.
 2 is var7 , var8 and var9 in mb-module.

 */
BOS_Status ReadFromMBModule(uint8_t dst,uint8_t rank,uint32_t timeout){
	messageParams[0] =rank;
	messageParams[1] =0;        // the size of message buffer
	messageParams[2] =0;        //  of the MB module is 21 byte
	messageParams[3] =0;        // so , 13 bytes' messageParams '
	messageParams[4] =0;        // + 8 bytes ' message frame setting'
	messageParams[5] =0;        // = 21 bytes.
	messageParams[6] =0;
	messageParams[7] =0;
	messageParams[8] =0;
	messageParams[9] =0;
	messageParams[10] =0;
	messageParams[11] =0;
	messageParams[12] =0;
	SendMessageToModule(dst,CODE_READ_REMOTE,13);
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	//while ( (responseStatus != BOS_OK) && ((HAL_GetTick()-t0) < timeout) ) { };
	while(((HAL_GetTick() - t0) < timeout)){
	};
	/* Return the read value address */
	if(responseStatus == BOS_OK){
		
		return BOS_OK;
	}
	else
		return BOS_ERROR;
	
}

/*-----------------------------------------------------------*/
/*  Write three variables to ModBus module:
 dst : When creating topology , you have to consider :
 MB module ID is 1.
 BOS module ID is 2.
 rank : represents a set of 3 variables.
 ModBus module has nine variables.
 0 is var1 , var2 and var3 in mb-module.
 1 is var4 , var5 and var6 in mb-module.
 2 is var7 , var8 and var9 in mb-module.

 */
BOS_Status WriteToMBModule(uint8_t dst,uint8_t rank,float var1,float var2,float var3){
	BOS_Status result =BOS_OK;
	
	if(rank <= 3){
		messageParams[0] =rank;
		messageParams[1] =(uint8_t )((*(uint32_t* )&var1) >> 0);   // first var
		messageParams[2] =(uint8_t )((*(uint32_t* )&var1) >> 8);
		messageParams[3] =(uint8_t )((*(uint32_t* )&var1) >> 16);
		messageParams[4] =(uint8_t )((*(uint32_t* )&var1) >> 24);
		
		messageParams[5] =(uint8_t )((*(uint32_t* )&var2) >> 0);  // second var
		messageParams[6] =(uint8_t )((*(uint32_t* )&var2) >> 8);
		messageParams[7] =(uint8_t )((*(uint32_t* )&var2) >> 16);
		messageParams[8] =(uint8_t )((*(uint32_t* )&var2) >> 24);
		
		messageParams[9] =(uint8_t )((*(uint32_t* )&var3) >> 0);   // third var
		messageParams[10] =(uint8_t )((*(uint32_t* )&var3) >> 8);
		messageParams[11] =(uint8_t )((*(uint32_t* )&var3) >> 16);
		messageParams[12] =(uint8_t )((*(uint32_t* )&var3) >> 24);
		
		SendMessageToModule(dst,CODE_WRITE_REMOTE,13);
	}
	else
		result =BOS_ERR_WrongParam;
	
	return result;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* --- Make a data string with format weekday / month / date / year */
char* GetDateString(void){
	static const char formatDateStr[] ="%s %02d/%02d/%04d";
	char *buffer =malloc(30 * sizeof(int8_t));
	memset(buffer,0x00,30 * sizeof(int8_t));
	sprintf(buffer,formatDateStr,weekdayString[BOS.date.weekday - 1],BOS.date.month,BOS.date.day,BOS.date.year);
	return buffer;
}

/*-----------------------------------------------------------*/

/* --- Make a time string with format hour / minute / second*/
char* GetTimeString(void){
	static const char formatTimeStr[] ="%02d:%02d:%02d";
	char *buffer =malloc(10 * sizeof(int8_t));
	memset(buffer,0x00,10 * sizeof(int8_t));
	sprintf(buffer,formatTimeStr,BOS.time.hours,BOS.time.minutes,BOS.time.seconds);
	return buffer;
}

/*-----------------------------------------------------------*/

/* --- Bridge two array/communication ports together */
BOS_Status Bridge(uint8_t port1,uint8_t port2){
	// Link the ports together with an infinite DMA stream
	return StartScastDMAStream(port1,myID,port2,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,true);
}

/*-----------------------------------------------------------*/

/* --- Un-bridge two array/communication ports  */
BOS_Status Unbridge(uint8_t port1,uint8_t port2){
	// Remove the stream from EEPROM
	SaveEEstreams(0,0,0,0,0,0,0,0,0);
	
	// Stop the DMA streams and enable messaging back on these ports
	if(streamDMA[port1 - 1].Instance != 0 && streamDMA[port2 - 1].Instance != 0){
		SwitchStreamDMAToMsg(port1);
		SwitchStreamDMAToMsg(port2);
		return BOS_OK;
	}
	else if(streamDMA[port1 - 1].Instance != 0){
		SwitchStreamDMAToMsg(port1);
		return BOS_OK;
	}
	else if(streamDMA[port2 - 1].Instance != 0){
		SwitchStreamDMAToMsg(port2);
		return BOS_OK;
	}
	else{
		return BOS_ERR_WrongValue;
	}
}

/*-----------------------------------------------------------*/

/* --- Print formatted text to one of the module ports */
BOS_Status printfp(uint8_t port,char *str){
	if(writePxMutex(port,str,strlen(str),1,1) == HAL_OK)
		return BOS_OK;
	else
		return BOS_ERROR;
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
