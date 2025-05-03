/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS.c
 Description   : Source code for Bitz Operating System (BOS).

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/***************************************************************************/
/* Exported variables ******************************************************/
/***************************************************************************/

bool ACKMessageFlag=0;
bool RejectedMessageFlag=0;
bool AddBcastPayload = false;

char GroupAlias[MAX_NUM_OF_GROUPS][MAX_LENGTH_OF_ALIAS + 1] ={0};
char Message[MAX_MESSAGE_SIZE] ={0};	/* Buffer to construct a message to be sent */
//char cRxedChar =0;

/* Define module PN strings [available PNs+1][5 chars] */
const char ModulePNstring[NUM_OF_MODULE_PN][PN_NUM_OF_CHARACTERS] ={"", "H01R0", "P01R0", "H23R0", "H23R1", "H23R3", "H07R3", "H08R6",
	"P08R6", "H09R0", "H09R9", "H1BR6", "H12R0", "H13R7", "H0FR1", "H0FR6", "H0FR7", "H1AR2", "H0AR9", "H1DR1",
	"H1DR5", "H0BR4", "H18R0", "H26R0", "H15R0", "H10R4", "H2AR3", "H41R6", "H3BR6", "H18R1", "H1FR5", "H3BR2",
	"H21R2", "H17R1", "H15R8", "H2BR0", "H05R0", "H3BR7", "H2BR1", "H07R8", "H08R7", "H16R6", "P08R7", "H19R0"};
static const char BOSkeywords[NUM_OF_KEYWORDS][4] ={"me", "all", "if", "for"};
static const char *WeekdayString[] ={"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
const char *MonthStringAbreviated[] ={"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
const char *pcParamsHelpString[NUM_OF_PARAMS_HELP_STRINGS] ={"\r\nBOS.response: all, message, cli, none\r\n", "\r\nBOS.trace: all, message, response, none\r\n",
	        "BOS.clibaudrate: CLI baudrate. Default is 921600. This affects all ports. If you change this value, \
           you must connect to a CLI port on each startup to restore other Array ports into default baudrate\r\n", "BOS.debounce: 1 ............ 65536 msec\r\n",
           "BOS.singleclicktime: 1 ..... 65536 msec\r\n", "BOS.mininterclicktime: 1 ... 255 msec\r\n", "BOS.maxinterclicktime: 1 ... 255 msec\r\n"};
const char *pcBootloaderUpdateMessage ="\n\rThis module will be forced into bootloader mode.\n\rPlease use the \"STM Flash Loader Demonstrator\" \
			utility to update the firmware.\n\r\n\t*** Important ***\n\rIf this module is connected directly to PC please close this port first.\n\r";
const char *pcRemoteBootloaderUpdateMessage ="\n\rModule %d will be forced into bootloader mode.";
const char *pcRemoteBootloaderUpdateWarningMessage ="\n\rPlease use the \"STM Flash Loader Demonstrator\" utility to update the firmware.\
			\n\r\n\t*** Important ***\n\r- If this module is connected directly to PC please close this port first.\n\r\
			- You must power cycle the entire Array after the update is finished.\n\r";
char *pcRemoteBootloaderUpdateViaPortMessage ="\n\rRemote update via module %d, port P%d will be triggered.";

uint8_t pcPort =0;
uint8_t bcastID =0;			/* Counter for unique broadcast ID */
uint8_t IndicatorMode =IND_OFF;
uint8_t bosInitialized =0;
uint8_t NumOfBosCommands = 0;
uint8_t dstGroupID =BOS_BROADCAST;
uint8_t NumOfRecordedSnippets =0;
uint8_t cliDataInputFlag = 0;
uint8_t MessageLength[NUM_OF_PORTS] ={0};
uint8_t PortStatus[NUM_OF_PORTS + 1] ={0};
uint8_t cMessage[NUM_OF_PORTS][MAX_MESSAGE_SIZE] ={0};	/* Buffer for received messages and ready to be parsed */
uint8_t MessageParams[MAX_PARAMS_PER_MESSAGE] ={0};
uint8_t StreamBuffer[STREAM_BUF_SIZE] = {0};

uint16_t myPN = MODULE_PN;
uint16_t Neighbors[NUM_OF_PORTS][2] ={0};
uint16_t Neighbors2[NUM_OF_PORTS][2] ={0};
uint16_t bcastRoutes[MAX_NUM_OF_MODULES] ={0}; /* P1 is LSB */

/* BOS variables register: Bits 31-16:
 * variable RAM address shift from SRAM_BASE, Bits 15-8: status.
 * Bits 7-0: format. */
uint32_t bosVarRegister[MAX_BOS_VARS];
volatile uint32_t* dmaIndex[NUM_OF_PORTS] ;

uint64_t RemoteBuffer =0;
uint8_t RequestFormat = 0;
/*OutputPortArray[__N]:
This Array stores all solutions (output ports) to send messages
between modules based on the topology file using FindRoute() function,
so we can read these output ports when needed instead of figuring out the correct port every time.
*/
#ifdef __N
uint8_t OutputPortArray[__N] = {0};
#endif

/* User Data from external ports (USB, Ethernet, BLE ...) ******************/
#ifdef __USER_DATA_BUFFER
uint8_t UserBufferData[USER_RX_BUF_SIZE]={0};
uint8_t UserData=0;
uint8_t IndexInputUserDataBuffer = 0;
uint8_t IndexProcessUserDataBuffer = 0;
volatile uint32_t* dmaCountUserDataBuffer = NULL;
#endif

#ifndef __N
uint16_t Array[MAX_NUM_OF_MODULES][MAX_NUM_OF_PORTS + 1] ={{0}}; /* Array topology */
uint8_t RouteDist[MAX_NUM_OF_MODULES] ={0};
uint8_t RoutePrev[MAX_NUM_OF_MODULES] ={0};
char ModuleAlias[MAX_NUM_OF_MODULES + 1][MAX_LENGTH_OF_ALIAS + 1] ={0}; /* ModuleAlias[0] used to store alias for module 0 */
uint8_t BroadcastResponse[MAX_NUM_OF_MODULES] ={0};
uint16_t GroupModules[MAX_NUM_OF_MODULES] ={0}; /* Group 0 (LSB) to Group 15 (MSB) */
uint16_t ArrayPortsDir[MAX_NUM_OF_MODULES]; /* Array ports directions */
uint8_t N =1;
uint8_t myID =0;
#else
	uint8_t RouteDist[__N] = {0};
	uint8_t RoutePrev[__N] = {0};
	char ModuleAlias[__N+1][MAX_LENGTH_OF_ALIAS+1] = {0};
	uint8_t BroadcastResponse[__N] = {0};
	uint16_t GroupModules[__N] = {0};
	uint16_t ArrayPortsDir[__N ] = {0};
	uint8_t N = __N;
	uint8_t myID = _module;
#endif

BOS_t BOS;
BOSOptionByte_t OptionByte ={0};
BOS_Status ResponseStatus =BOS_OK;
VariableFormat_t RemoteVarFormat =FMT_UINT8;
BOSOptionByte_t UserOptionByte ={.Trace = false, .Acknowledgment = false, .Response = BOS_RESPONSE_NONE};
BOS_t BOS_default ={.cliBaudrate = DEF_CLI_BAUDRATE, .Buttons.Debounce =DEF_BUTTON_DEBOUNCE,
	.Buttons.SingleClickTime = DEF_BUTTON_CLICK,.Buttons.minInterClickTime = DEF_BUTTON_MIN_INTER_CLICK,
	.Buttons.maxInterClickTime = DEF_BUTTON_MAX_INTER_CLICK,.DaylightSaving =DAYLIGHT_NONE, .HourFormat =24,
	.DisableCLI = false};

/* Exported internally: CLI command list ***********************************/
typedef struct xCOMMAND_INPUT_LIST {
	const CLI_Command_Definition_t *pxCommandLineDefinition;
	struct xCOMMAND_INPUT_LIST *pxNext;
} CLI_Definition_List_Item_t;
extern CLI_Definition_List_Item_t xRegisteredCommands;

/* Local Variables *********************************************************/
static char pcUserMessage[PC_USER_MESSAGE_SIZE];
uint8_t ExtraPcPort = 0;
uint8_t cliLowBaudrateFlag =0; 	/* Flag for Lower CLI baudrate is set */

/***************************************************************************/
/* Exported Functions ******************************************************/
/***************************************************************************/

/* Module exported internal functions **************************************/
extern uint8_t SaveTopologyToRO(void);
//extern uint8_t IsFactoryReset(void);
//extern BOS_Status GetPortGPIOs(uint8_t port,uint32_t *TX_Port,uint16_t *TX_Pin,uint32_t *RX_Port,uint16_t *RX_Pin);
extern BOS_Status SetButtonEvents(uint8_t port, ButtonState_e buttonState, uint8_t mode);
extern BOS_Status AddPortButton(ButtonType_e buttonType, uint8_t port);
extern BOS_Status RTC_Init(void);
extern void Module_Peripheral_Init(void);
extern void TIM_USEC_Init(void);
extern void TIM_MSEC_Init(void);
extern void MX_IWDG_Init(void);

/* BOS exported internal functions *****************************************/
extern BOS_Status SetupDMAStreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src,uint8_t dst);

/***************************************************************************/
/* Private function prototypes *********************************************/
/***************************************************************************/
void EE_FormatForFactoryReset(void);
uint8_t IsModuleParameter(char *name);
BOS_Status ClearEEportsDir(void);
BOS_Status WriteToRemote(uint8_t module,uint32_t localVarAddress,uint32_t remoteVarAddress,VariableFormat_t format,uint32_t timeout/*,uint8_t force*/);

/* Find Route related APIs ****************************************************/
uint8_t minArr(uint8_t *arr,uint8_t *Q);
uint8_t QnotEmpty(uint8_t *Q);

/* Load form EEPROM related APIs *******************************************/
BOS_Status LoadROsnippets(void);
BOS_Status LoadROtopology(void);
BOS_Status LoadEEportsDir(void);
BOS_Status LoadEEalias(void);
BOS_Status LoadEEgroup(void);
BOS_Status LoadEEstreams(void);
BOS_Status LoadEEbuttons(void);
BOS_Status LoadEEparams(void);

/* Save to EEPROM related APIs *********************************************/
BOS_Status SaveEEportsDir(void);
BOS_Status SaveEEalias(void);
BOS_Status SaveEEgroup(void);
BOS_Status SaveEEstreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src1,uint8_t dst1,uint8_t src2,uint8_t dst2,uint8_t src3,uint8_t dst3);
BOS_Status SaveEEparams(void);

/***************************************************************************/
/*****************************  Private Functions **************************/
/***************************************************************************/

/* Load stored variables,Ports directions,Module's name ...etc
 * from emulated EEPROM and RO Flash
 */
void LoadEEvars(void){
	/* Load Array topology */
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
	
	/* Load Command Snippets */
	LoadROsnippets();
}

/***************************************************************************/
/* Load Array topology stored in Flash RO */
BOS_Status LoadROtopology(void){
	BOS_Status result =BOS_OK;
	uint16_t add =8, temp =0;
	
	/* Load number of modules */
	temp =(*(__IO uint16_t* )(TOPOLOGY_START_ADDRESS));
	
	if(temp == MEMORY_ERASED){   /* if memory has been erased */
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
			for(volatile uint8_t j =0; j <= MAX_NUM_OF_PORTS; j++){
				Array[i - 1][j] =(*(__IO uint16_t* )(TOPOLOGY_START_ADDRESS + add));
				add +=8;
			}
		}
	}
	
	return result;
}

/***************************************************************************/
/* Load Array ports directions stored in Emulated EEPROM */
BOS_Status LoadEEportsDir(void){
	BOS_Status result =BOS_OK;
	
	for(uint8_t i =1; i <= N; i++){
		EE_ReadVariable(_EE_PORT_DIR_BASE + i - 1,&ArrayPortsDir[i - 1]);
		
		if((i + _EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result =BOS_ERR_EEPROM;
	}
	
	return result;
}

/***************************************************************************/
/* Load module alias stored in Emulated EEPROM */
BOS_Status LoadEEalias(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	
	for(uint8_t i =0; i <= N; i++){ // N+1 module aliases
		for(uint8_t j =1; j <= MAX_LENGTH_OF_ALIAS; j +=2){
			EE_ReadVariable(_EE_ALIAS_BASE + add,&temp);
			ModuleAlias[i][j] =(uint8_t )temp;
			ModuleAlias[i][j - 1] =(uint8_t )(temp >> 8);
			add++;
		}
		ModuleAlias[i][MAX_LENGTH_OF_ALIAS] ='\0';
	}
	
	return result;
}

/***************************************************************************/
/* Load module groups stored in Emulated EEPROM */
BOS_Status LoadEEgroup(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	uint8_t i =0;
	
	/* Load group members */
	for(i =0; i < N; i++){ // N modules
		EE_ReadVariable(_EE_GROUP_MODULES_BASE + add,&GroupModules[i]);
		add++;
	}
	
	/* Load group alias */
	for(i =0; i < MAX_NUM_OF_GROUPS; i++){
		for(uint8_t j =1; j <= MAX_LENGTH_OF_ALIAS; j +=2){
			EE_ReadVariable(_EE_GROUP_ALIAS_BASE + add,&temp);
			GroupAlias[i][j] =(uint8_t )temp;
			GroupAlias[i][j - 1] =(uint8_t )(temp >> 8);
			add++;
		}
		GroupAlias[i][MAX_LENGTH_OF_ALIAS] ='\0';
	}
	
	return result;
}

/***************************************************************************/
/* Load module DMA streams stored in Emulated EEPROM */
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

/***************************************************************************/
/* Load module parameters stored in Emulated EEPROM */
BOS_Status LoadEEparams(void){
	BOS_Status result =BOS_OK;
	uint16_t temp1, temp2, status1, status2;
	
	/* Read params base - BOS response and BOS trace */
	status1 =EE_ReadVariable(_EE_PARAMS_BASE,&temp1);
	/* Found the variable (EEPROM is not cleared) */
	if(!status1){
		OptionByte.Response =(uint8_t )temp1;
		OptionByte.Trace =(temp1 >> 8);
		/* Couldn't find the variable, load default config */
	}
	else{
		OptionByte.Response =UserOptionByte.Response;
		OptionByte.Trace =UserOptionByte.Trace;

	}
	/* Read params base - BOS response and BOS trace */
	status1 =EE_ReadVariable(_EE_PARAMS_Messaging,&temp1);

	if(!status1){
		OptionByte.Acknowledgment =(bool )(temp1 >> 15);
		/* Couldn't find the variable, load default config */
	}
	else
		OptionByte.Acknowledgment =UserOptionByte.Acknowledgment;

	/* Read Button debounce */
	status1 =EE_ReadVariable(_EE_PARAMS_DEBOUNCE,&temp1);
	if(!status1)
		BOS.Buttons.Debounce =temp1;
	else
		BOS.Buttons.Debounce =BOS_default.Buttons.Debounce;
	
	/* Read Button single click time */
	status1 =EE_ReadVariable(_EE_PARAMS_SINGLE_CLICK,&temp1);
	if(!status1)
		BOS.Buttons.SingleClickTime =temp1;
	else
		BOS.Buttons.SingleClickTime =BOS_default.Buttons.SingleClickTime;
	
	/* Read Button double click time (min and max inter-click) */
	status1 =EE_ReadVariable(_EE_PARAMS_DBL_CLICK,&temp1);
	if(!status1){
		BOS.Buttons.minInterClickTime =(uint8_t )temp1;
		BOS.Buttons.maxInterClickTime =(uint8_t )(temp1 >> 8);
	}
	else{
		BOS.Buttons.minInterClickTime =BOS_default.Buttons.minInterClickTime;
		BOS.Buttons.maxInterClickTime =BOS_default.Buttons.maxInterClickTime;
	}
	
	/* Read CLI baudrate */
	status1 =EE_ReadVariable(_EE_CLI_BAUD,&temp1);
	status2 =EE_ReadVariable(_EE_CLI_BAUD + 1,&temp2);
	if(!status1 && !status2){
		BOS.cliBaudrate =(uint32_t )temp1 | (((uint32_t )temp2) << 16);
	}
	else if(cliLowBaudrateFlag)
		BOS.cliBaudrate = CLI_BAUDRATE_1;
	else
		BOS.cliBaudrate =BOS_default.cliBaudrate;
	
	/* Read RTC hourformat and daylightsaving */
	status1 =EE_ReadVariable(_EE_PARAMS_RTC,&temp1);
	if(!status1){
		BOS.DaylightSaving =(int8_t )temp1;
		BOS.HourFormat =(uint8_t )(temp1 >> 8);
	}
	else{
		BOS.HourFormat =24;
		BOS.DaylightSaving =DAYLIGHT_NONE;
	}
	
	/* Read disableCLI */
	status1 =EE_ReadVariable(_EE_PARAMS_DISABLE_CLI,&temp1);
	/* Found the variable (EEPROM is not cleared) */
	if(!status1){
		BOS.DisableCLI =(uint8_t )temp1;
		/* Couldn't find the variable, load default config */
	}
	else{
		BOS.DisableCLI =BOS_default.DisableCLI;
	}
	
	return result;
}

/***************************************************************************/
/* Load button definitions and events stored in Emulated EEPROM */
BOS_Status LoadEEbuttons(void){
	BOS_Status result =BOS_OK;
	uint16_t temp16 =0, status1 =0;
	uint8_t temp8 =0;
	
	for(uint8_t i =0; i <= NUM_OF_PORTS; i++){
		status1 =EE_ReadVariable(_EE_BUTTON_BASE + 4 * (i),&temp16);
		
		/* This variable exists */
		if(!status1){
			temp8 =(uint8_t )(temp16 >> 8);
			if(((temp8 >> 4) == i + 1) && ((temp8 & 0x0F) != NONE)){
				Button[i + 1].Type =temp8 & 0x0F;
				Button[i + 1].Event =(uint8_t )temp16;

				/* Setup the button and its events */
				AddPortButton(Button[i + 1].Type,i + 1);
				SetButtonEvents(i + 1,(Button[i + 1].Event & BUTTON_EVENT_CLICKED),BUTTON_EVENT_MODE_CLEAR);
			}
		}
	}
	
	return result;
}

/***************************************************************************/
/* Load Command Snippets stored in Flash RO */
BOS_Status LoadROsnippets(void){
	uint8_t i =0;
	int currentAdd = SNIPPETS_START_ADDRESS;
	char *snipBuffer =(char* )malloc(cmdMAX_INPUT_SIZE);
	if(snipBuffer == NULL)
		return BOS_MEM_FULL;
	
	/* Exit if no recorded Snippets */
	if(*(uint8_t* )currentAdd != 0xFE)
		return BOS_ERROR;
	
	/* Load Snippets */
	for(uint8_t s =0; s < MAX_SNIPPETS; s++){
		/* Load conditions starting at RO_MID_ADDRESS */
		for(i =0; i < sizeof(Snippet_t); i++)
			snipBuffer[i] =(*(__IO uint8_t* )(currentAdd++));
		memcpy((uint8_t* )&Snippets[s],(uint8_t* )&snipBuffer[1],sizeof(Snippet_t));
		memset(snipBuffer,0,sizeof(Snippet_t));
		i =0;
		/* Load commands until you get next 0xFE */
		currentAdd =currentAdd + 20;
		while(*(uint8_t* )currentAdd != 0xFE && *(uint8_t* )currentAdd != 0xFF && i < cmdMAX_INPUT_SIZE){
			snipBuffer[i] =*(uint8_t* )currentAdd;
			++currentAdd;
			++i;
		}
		if(snipBuffer[i - 1] != 0)
			++i; /* String termination char was not recorded, then add one */

		/* Allocate buffer for the Snippet commands */
		Snippets[s].CMD =(char* )malloc(i);
		if(Snippets[s].CMD == NULL){
			memset(&Snippets[s],0,sizeof(Snippet_t));
			free(snipBuffer);
			return BOS_ERR_SNIP_MEM_FULL;
		}
		else{
			/*- Copy the command */
			memcpy(Snippets[s].CMD,snipBuffer,i);
			++NumOfRecordedSnippets; /* Record a successful Snippet */
			memset(snipBuffer,0,i);
		}
		/* Exit if no more Snippets */
		if(*(uint8_t* )currentAdd != 0xFE)
			break;
	}
	
	free(snipBuffer);
	return BOS_OK;
}

/***************************************************************************/
/* Save Array ports directions to Emulated EEPROM */
BOS_Status SaveEEportsDir(void){
	BOS_Status result =BOS_OK;
	
	for(uint8_t i =1; i <= N; i++){
		if(ArrayPortsDir[i - 1])
			EE_WriteVariable(_EE_PORT_DIR_BASE + i - 1,ArrayPortsDir[i - 1]);
		
		if((i + _EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result =BOS_ERR_EEPROM;
	}
	
	return result;
}

/***************************************************************************/
/* Save module alias to Emulated EEPROM */
BOS_Status SaveEEalias(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	
	/* N+1 module aliases */
	for(uint8_t i =0; i <= N; i++){
		if(ModuleAlias[i][0]){
			for(uint8_t j =1; j <= MAX_LENGTH_OF_ALIAS; j +=2){
				temp =(uint16_t )(ModuleAlias[i][j - 1] << 8) + ModuleAlias[i][j];
				EE_WriteVariable(_EE_ALIAS_BASE + add,temp);
				add++;
			}
		}
	}
	
	return result;
}

/***************************************************************************/
/* Save module groups to Emulated EEPROM */
BOS_Status SaveEEgroup(void){
	BOS_Status result =BOS_OK;
	uint16_t add =0, temp =0;
	uint8_t i =0;
	
	/* Save group members
	 * N modules */
	for(i =0; i < N; i++){
		if(GroupModules[i]){
			EE_WriteVariable(_EE_GROUP_MODULES_BASE + add,GroupModules[i]);
			add++;
		}
	}
	
	/* Save group alias */
	for(i =0; i < MAX_NUM_OF_GROUPS; i++){
		if(GroupAlias[i][0]){
			for(uint8_t j =1; j <= MAX_LENGTH_OF_ALIAS; j +=2){
				temp =(uint16_t )(GroupAlias[i][j - 1] << 8) + GroupAlias[i][j];
				EE_WriteVariable(_EE_GROUP_ALIAS_BASE + add,temp);
				add++;
			}
		}
	}
	
	return result;
}

/***************************************************************************/
/* Save DMA streams to Emulated EEPROM*/
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

/***************************************************************************/
/* --- Save module parameters to Emulated EEPROM */
BOS_Status SaveEEparams(void){
	BOS_Status result =BOS_OK;
	
	/* Save params base - BOS response & BOS trace */
//	EE_WriteVariable(_EE_PARAMS_BASE,((uint16_t )OptionByte.Trace << 5) | (uint16_t )OptionByte.Response);
//	EE_WriteVariable(_EE_PARAMS_Messaging,((uint16_t )OptionByte.Acknowledgment << 15) | (uint16_t )OptionByte.trial);

	/* Save Button debounce */
	EE_WriteVariable(_EE_PARAMS_DEBOUNCE,BOS.Buttons.Debounce);
	
	/* Save Button single click time */
	EE_WriteVariable(_EE_PARAMS_SINGLE_CLICK,BOS.Buttons.SingleClickTime);
	
	/* Save Button double click time (min and max inter-click) */
	EE_WriteVariable(_EE_PARAMS_DBL_CLICK,((uint16_t )BOS.Buttons.maxInterClickTime << 8) | (uint16_t )BOS.DaylightSaving);
	
	/* Save CLI baudrate */
	EE_WriteVariable(_EE_CLI_BAUD,(uint16_t )BOS.cliBaudrate);
	EE_WriteVariable(_EE_CLI_BAUD + 1,(uint16_t )(BOS.cliBaudrate >> 16));
	
	/* Save RTC hour format and daylight saving */
	EE_WriteVariable(_EE_PARAMS_RTC,((uint16_t )BOS.HourFormat << 8) | (uint16_t )BOS.Buttons.minInterClickTime);
	
	/* Save disableCLI */
	EE_WriteVariable(_EE_PARAMS_DISABLE_CLI,(uint16_t )BOS.DisableCLI);
	
	return result;
}

/***************************************************************************/
/* Clear Array ports directions in Emulated EEPROM */
BOS_Status ClearEEportsDir(void){
	BOS_Status result =BOS_OK;
	
	memset(ArrayPortsDir,0,sizeof(ArrayPortsDir));
	
	for(uint8_t i =1; i <= N; i++){
		if(ArrayPortsDir[i - 1])
			EE_WriteVariable(_EE_PORT_DIR_BASE + i - 1,ArrayPortsDir[i - 1]);
		
		if((i + _EE_PORT_DIR_BASE) >= _EE_ALIAS_BASE)
			result =BOS_ERR_EEPROM;
	}
	
	return result;
}

/***************************************************************************/
/* Check if booting into lower CLI baudrate:
 * Connect P1 TXD and P2 RXD to boot CLI at 115200
 */
uint8_t IsLowerCLIbaud(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t P1_TX_Port, P1_RX_Port, P2_TX_Port, P2_RX_Port;
	uint16_t P1_TX_Pin, P1_RX_Pin, P2_TX_Pin, P2_RX_Pin;

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

/***************************************************************************/
/* Check if this string is a local module parameter or event
 * Returns parameter index+1
 */
uint8_t IsModuleParameter(char *name){
	for(uint8_t i =0; i < NUM_MODULE_PARAMS; i++){
		if(!strcmp(name,(const char* )(ModuleParam[i].ParamName)))
			return i + 1;
	}
	return 0;
}

/***************************************************************************/
/* Check if this string is a math operator and return its enum */
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

/***************************************************************************/
/*************************** BOS General Functions *************************/
/***************************************************************************/

/* BitzOS initialization */
void BOS_Init(void){

	/* Storing Values inside OutputPortArray[] using FindRoute() Function */
#ifdef __N
	for(uint8_t i = 1;i <= __N;i++)
	{
		if(myID == i) OutputPortArray[i-1] = 0;
		else OutputPortArray[i-1] = FindRoute(myID, i);
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
		cliLowBaudrateFlag =1;
		/* Initialize the module */
		/* Give other modules time to finish factory reset and baudrate check */
		Delay_ms_no_rtos(50);
		Module_Peripheral_Init();
		
		BOS.cliBaudrate = CLI_BAUDRATE_1;
		/* Update all ports to lower baudrate */
		for(uint8_t port =1; port <= NUM_OF_PORTS; port++){
			UpdateBaudrate(port,BOS.cliBaudrate);
		}
	}
	else{
		Delay_ms_no_rtos(50);
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
	/* Native module */
	if(myID == 0){
		IND_ON();
		Delay_ms_no_rtos(500);IND_OFF();
	}
	/* Non-native module */
	else{
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
	
	/* initialize IWDG timer lastly in order to avoid reset */
//	MX_IWDG_Init();
	bosInitialized =1;
}

/***************************************************************************/
/* */
void Module_Init(void){

	/* Initialize HAL library */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize BitzOS */
	BOS_Init();

#ifdef SEGGER_SYSTEM_VIEW_ENABLE
	/* SEGGER System view Start-up Functions */
	/* Configure and initialize SystemView */
	SEGGER_SYSVIEW_Conf();
	/* Start SystemView */
	SEGGER_SYSVIEW_Start();
#endif

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();
}

/***************************************************************************/
#ifdef __USER_DATA_BUFFER
uint8_t GetUserDataCount(void)
{
	IndexInputUserDataBuffer = USER_RX_BUF_SIZE - (uint8_t)(*dmaCountUserDataBuffer);

	if(IndexInputUserDataBuffer== IndexProcessUserDataBuffer)
	{
		return 0;
	}

	else
	{
		if(IndexInputUserDataBuffer > IndexProcessUserDataBuffer)
		{
			return (IndexInputUserDataBuffer - IndexProcessUserDataBuffer);
		}
		else
		{
			return (IndexInputUserDataBuffer - IndexProcessUserDataBuffer + USER_RX_BUF_SIZE);
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

		*pData =  UserBufferData[IndexProcessUserDataBuffer];
		IndexProcessUserDataBuffer++;
		if(IndexProcessUserDataBuffer == USER_RX_BUF_SIZE)
		{
			IndexProcessUserDataBuffer = 0;
		}
		return BOS_OK;
	}

	else
	{
		return BOS_ERROR;
	}

}
#endif

/***************************************************************************/
#ifndef __N
/* Explore the Array and create its topology (executed only by master) */
BOS_Status Explore(void)
{
	BOS_Status result = BOS_OK;
	uint8_t currentID = 0, lastID = 0, temp1 = 0, temp2 = 0, i = 0, j = 0, p = 0, port = 0;
	uint16_t temp16 = 0;
	myID = 1; 		/* Master ID */

	/* Step 1: ******************************************************************/
	/* Reverse master ports and explore adjacent Neighbors **********************/
	/* **************************************************************************/

	pcPort = ExtraPcPort;
	for (uint8_t port=1 ; port<=NUM_OF_PORTS ; port++) {
		if (port != pcPort)	SwapUartPins(GetUart(port), REVERSED);
	}
	ExploreNeighbors(pcPort); IndicatorMode = IND_TOPOLOGY;
	osDelay(50);

	/* Step 2: ******************************************************************/
	/* Assign IDs to new modules & update the topology Array ********************/
	/* **************************************************************************/

	/* Step 2a - Assign IDs to new modules **************************************/
	currentID = 1;
	for (port=1 ; port<=NUM_OF_PORTS ; port++)
	{
		if (Neighbors[port-1][0])
		{
			/* New ID */
			MessageParams[1] = ++currentID;
			N = currentID;			/* Update number of modules in the Array */
			/* Inform module to change ID */
			MessageParams[0] = 0;	/* change own ID */
			SendMessageFromPort(port, 0, 0, CODE_MODULE_ID, 3);
			/* Modify Neighbors table */
			Neighbors[port-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(Neighbors[port-1][0]);
			osDelay(50);
		}
	}

	/* Step 2b - Update master topology Array ***********************************/
	Array[0][0]	= myPN;
	for (port=1 ; port<=NUM_OF_PORTS ; port++)
	{
		if (Neighbors[port-1][0])
		{
			temp16 = Neighbors[port-1][0];
			temp1 = (uint8_t)(temp16>>8);					/* Neighbor ID */
			temp2 = (uint8_t)(Neighbors[port-1][0]);		/* Neighbor port */
			/* Module 1 (master) */
			Array[0][port] = ( temp1 << 3 ) | temp2;		/* Neighbor ID | Neighbor port */
			/* Rest of the Neighbors */
			Array[temp1-1][0]	= Neighbors[port-1][1];		/* Neighbor PN */
			Array[temp1-1][temp2] = ( myID << 3 ) | port;	/* Module 1 ID | Module 1 port */
		}
	}

	/* Step 2c - Ask Neighbors to update their topology Array *******************/
	for (i=2 ; i<=currentID ; i++)
	{
//		memcpy(MessageParams, Array, (size_t) (currentID*(MAX_NUM_OF_PORTS+1)*2) );
//		SendMessageToModule(i, CODE_TOPOLOGY, (size_t) (currentID*(MAX_NUM_OF_PORTS+1)*2));
		SendLargeMessageToModule(i, CODE_TOPOLOGY, (uint8_t *) Array, (currentID*(MAX_NUM_OF_PORTS+1)*2));

		osDelay(10);
	}

	/* Step 3: ******************************************************************/
	/* Ask each new module to explore and repeat ********************************/
	/* **************************************************************************/

	while (lastID != currentID)
	{
		/* Scan all discovered modules */
		for (i=2 ; i<=currentID ; i++)
		{
			/* Step 3a - Ask the module to reverse ports ********************************/
			for (uint8_t p=1 ; p<=MAX_NUM_OF_PORTS ; p++) {
				MessageParams[p-1] = REVERSED;
			}
			MessageParams[MAX_NUM_OF_PORTS] = NORMAL;	/* Make sure the inport is not reversed */
			SendMessageToModule(i, CODE_PORT_DIRECTION, MAX_NUM_OF_PORTS+1);
			osDelay(50);

			/* Step 3b - Ask the module to explore adjacent Neighbors *******************/
			SendMessageToModule(i, CODE_EXPLORE_ADJ, 0);
			osDelay(300);

			/* Step 3c - Assign IDs to new modules **************************************/
			for (j=1 ; j<=MAX_NUM_OF_PORTS ; j++)
			{
				temp16 = Neighbors2[j-1][0];		/* Neighbor ID */
				temp1 = (uint8_t)(temp16>>8);
				if (temp16 != 0 && temp1 == 0)		/* UnIDed module */
				{
					/* New ID */
					MessageParams[1] = ++currentID;
					N = currentID;			        /* Update number of modules in the Array */
					/* Modify Neighbors table */
					Neighbors2[j-1][0] = ( (uint16_t) currentID << 8 ) + (uint8_t)(Neighbors2[j-1][0]);
					/* Ask the module to ID its yet unIDed Neighbors */
					MessageParams[0] = 1;		    /* change neighbor ID */
					MessageParams[2] = j;		    /* neighbor port */
					SendMessageToModule(i, CODE_MODULE_ID, 3);
					osDelay(50);
				}
			}

			/* Step 3d - Update master topology Array ***********************************/
			for (j=1 ; j<=MAX_NUM_OF_PORTS ; j++)
			{
				if (Neighbors2[j-1][0])
				{
					temp16 = Neighbors2[j-1][0];
					temp1 = (uint8_t)(temp16>>8);			/* Neighbor ID */
					temp2 = (uint8_t)(Neighbors2[j-1][0]);	/* Neighbor port */
					if (temp1 != 1)			                /* Exclude the master */
					{
						/* Update module i section */
						if (Array[i-1][j] == 0) {
							Array[i-1][j] = ( temp1 << 3 ) | temp2;		/* Neighbor ID | Neighbor port */
						}
						/* Update module i Neighbors */
						if (Array[temp1-1][temp2] == 0) {
							Array[temp1-1][0]	= Neighbors2[j-1][1];	/* Neighbor PN */
							Array[temp1-1][temp2] = ( i << 3 ) | j;		/* Module i ID | Module i port */
						}
					}
				}
			}

			/* Reset Neighbors2 Array */
			memset(Neighbors2, 0, sizeof(Neighbors2) );

			/* Step 3e - Ask all discovered modules to update their topology Array ******/
			for (j=2 ; j<=currentID ; j++)
			{
//				memcpy(MessageParams, Array, (size_t) (currentID*(MAX_NUM_OF_PORTS+1)*2) );
//				SendMessageToModule(j, CODE_TOPOLOGY, (size_t) (currentID*(MAX_NUM_OF_PORTS+1)*2));
				SendLargeMessageToModule(j, CODE_TOPOLOGY, (uint8_t *) Array, (currentID*(MAX_NUM_OF_PORTS+1)*2));
				osDelay(100);
			}
		}

		/* Update lastID */
		lastID = currentID;
	}

	/* Step 4: ******************************************************************/
	/* Make sure all connected modules have been discovered *********************/
	/* **************************************************************************/

	pcPort = ExtraPcPort;
	ExploreNeighbors(pcPort);
	osDelay(50);

	/* Check for any unIDed Neighbors */
	for (i=1 ; i<=NUM_OF_PORTS ; i++)
	{
		temp16 = Neighbors[i-1][0];		    /* Neighbor ID */
		temp1 = (uint8_t)(temp16>>8);
		if (temp16 != 0 && temp1 == 0) {	/* UnIDed module */
			result = BOS_ERR_UnIDedModule;
		}
	}
	/* Ask other modules for any unIDed Neighbors */
	for (i=2 ; i<=currentID ; i++)
	{
		SendMessageToModule(i, CODE_EXPLORE_ADJ, 0);
		osDelay(300);
		/* Check for any unIDed Neighbors */
		for (j=1 ; j<=MAX_NUM_OF_PORTS ; j++)
		{
			temp16 = Neighbors2[j-1][0];		/* Neighbor ID */
			temp1 = (uint8_t)(temp16>>8);
			if (temp16 != 0 && temp1 == 0) {	/* UnIDed module */
				result = BOS_ERR_UnIDedModule;
			}
		}
	}

	/* Step 5: ******************************************************************/
	/* If no unIDed modules found, generate and distribute port directions ******/
	/* **************************************************************************/

	if (result == BOS_OK)
	{
		/* Step 5a - Virtually reset the state of master ports to Normal ************/
		for (port=1 ; port<=NUM_OF_PORTS ; port++) {
			ArrayPortsDir[0] &= (~(0x8000>>(port-1)));		   /* Set bit to zero */
		}
		/* Step 5b - Update other modules ports starting from the last one **********/
		for (i=currentID ; i>=2 ; i--)
		{
			for (p=1 ; p<=MAX_NUM_OF_PORTS ; p++)
			{
				if (!Array[i-1][p])	{
					/* If empty port leave normal */
					MessageParams[p-1] = NORMAL;
					ArrayPortsDir[i-1] &= (~(0x8000>>(p-1)));	 /* Set bit to zero */
				} else {
					/* If not empty, check neighbor */
					temp16 = Array[i-1][p];
					temp1 = (uint8_t)(temp16>>3);				 /* Neighbor ID */
					temp2 = (uint8_t)(temp16 & 0x0007);			 /* Neighbor port */
					/* Check neighbor port direction */
					if ( !(ArrayPortsDir[temp1-1] & (0x8000>>(temp2-1))) ) {
						/* Neighbor port is normal */
						MessageParams[p-1] = REVERSED;
						ArrayPortsDir[i-1] |= (0x8000>>(p-1));	  /* Set bit to one */
					} else {
						/* Neighbor port is reversed */
						MessageParams[p-1] = NORMAL;
						ArrayPortsDir[i-1] &= (~(0x8000>>(p-1))); /* Set bit to zero */
					}
				}
			}

			/* Step 5c - Check if an inport is reversed *********************************/
			/* Find out the inport to this module from master */
			FindRoute(1, i);
			temp1 = Route[NumberOfHops(i)-1];				/* previous module = Route[Number of hops - 1] */
			temp2 = FindRoute(i, temp1);
			/* Is the inport reversed? */
			if ( (temp1 == i) || (MessageParams[temp2-1] == REVERSED) )
				MessageParams[MAX_NUM_OF_PORTS] = REVERSED;	/* Make sure the inport is reversed */

			/* Step 5d - Update module ports directions *********************************/
			SendMessageToModule(i, CODE_PORT_DIRECTION, MAX_NUM_OF_PORTS+1);
			osDelay(10);
		}

		/* Step 5e - Update master ports > all normal *******************************/
		for (port=1 ; port<=NUM_OF_PORTS ; port++) {
			if (port != pcPort)	SwapUartPins(GetUart(port), NORMAL);
		}
	}

	/* Step 6: ******************************************************************/
	/* Test new port directions by pinging all modules **************************/
	/* **************************************************************************/

	if (result == BOS_OK)
	{
		for (i=2 ; i<=N ; i++)
		{
			SendMessageToModule(i, CODE_PING, 0);
			osDelay(10*NumberOfHops(i));
		}
	}

	/* Step 7: ******************************************************************/
	/* Save all (topology and port directions) in RO/EEPROM *********************/
	/* **************************************************************************/

	if (result == BOS_OK)
	{
		/* Save data in the master */
		SaveTopologyToRO();
		SaveEEportsDir();
		osDelay(100);
		/* Ask other modules to save their data too */
		for (i=2 ; i<=N ; i++)
		{
			SendMessageToModule(i, CODE_EXP_EEPROM, 0);
			osDelay(10*NumberOfHops(i));
		}

	}

	return result;
}
#endif

/***************************************************************************/
#ifndef __N
/* Explore adjacent Neighbors */
BOS_Status ExploreNeighbors(uint8_t ignore){
	BOS_Status result =BOS_OK;
	
	/* Send Hi messages to adjacent Neighbors */
	for(uint8_t port =1; port <= NUM_OF_PORTS; port++){
		if(port != ignore){
			/* This module info */
			MessageParams[0] =(uint8_t )(myPN >> 8);
			MessageParams[1] =(uint8_t )myPN;
			MessageParams[2] =port;
			/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
			SendMessageFromPort(port,0,0,CODE_HI,3);
			/* Minimum delay between two consecutive SendMessage commands (with response) */
			osDelay(5);
		}
	}
	
	return result;
}
#endif

/***************************************************************************/
/* Find Array broadcast routes starting from a given module (Takes about 50 usec) */
BOS_Status FindBroadcastRoutes(uint8_t src){
	BOS_Status result =BOS_OK;
	uint8_t p =0, m =0, level =0, untaged =0;
	uint8_t modules[N];			// Todo: Optimize to make bit-wise
	
	/* 1. Initialize modules list and broadcast routes */
	for(m =0; m < N; m++){
		modules[m] =0;
		bcastRoutes[m] =0;
	}
	/* Tag the source */
	modules[src - 1] =++level;
	
	/* 2. Source module should send to all Neighbors */
	/* Move one level */
	++level;
	
	for(p =1; p <= 6; p++){
		if(Array[src - 1][p]){
			bcastRoutes[src - 1] |=(0x01 << (p - 1));
			modules[(Array[src - 1][p] >> 3) - 1] =level;												// Tag this module as already broadcasted-to
		}
	}
	
	/* 3. Starting from source Neighbors,
	 * check all other modules we haven't broadcasted-to yet, one by one */
	do{
		/* Reset the untaged counter */
		untaged =0;
		/* Move one level */
		++level;
		
		/* Scan all modules in the list */
		for(m =0; m < N; m++){
			/* This module is already broadcasted-to from the previous level */
			if(modules[m] == (level - 1)){
				/* Check all Neighbors if they're not already broadcasted-to */
				for(p =1; p <= 6; p++){
					/* Found an untaged module */
					if(Array[m][p] && (modules[(Array[m][p] >> 3) - 1] == 0)){
						bcastRoutes[m] |=(0x01 << (p - 1));
						/* Tag this module as already broadcasted-to */
						modules[(Array[m][p] >> 3) - 1] =level;
						++untaged;
					}
				}
			}
		}
	} while(untaged);
	
	return result;
}

/***************************************************************************/
/* Find the shortest Route to a module using Dijkstra's algorithm
 
 Algorithm (from Wikipedia):

 1- Assign to every node a tentative distance value: set it to zero for our initial node
 and to infinity for all other nodes.

 2- Set the initial node as current. Mark all other nodes unvisited. Create a set of all
 the unvisited nodes called the unvisited set.

 3- For the current node, consider all of its unvisited Neighbors and calculate their tentative
 distances. Compare the newly calculated tentative distance to the current assigned value and
 assign the smaller one. For example, if the current node A is marked with a distance of 6,
 and the edge connecting it with a neighbor B has length 2, then the distance to B (through A)
 will be 6 + 2 = 8. If B was previously marked with a distance greater than 8 then change it to 8.
 Otherwise, keep the current value.

 4- When we are done considering all of the Neighbors of the current node, mark the current
 node as visited and remove it from the unvisited set. A visited node will never be checked again.

 5- If the destination node has been marked visited (when planning a Route between two specific
 nodes) or if the smallest tentative distance among the nodes in the unvisited set is infinity
 (when planning a complete traversal; occurs when there is no connection between the initial
 node and remaining unvisited nodes), then stop. The algorithm has finished.

 6- Otherwise, select the unvisited node that is marked with the smallest tentative distance,
 set it as the new "current node", and go back to step 3.

 */
uint8_t FindRoute(uint8_t sourceID,uint8_t desID){
#ifdef __N
	uint8_t Q[__N] = {0}; /* All nodes initially in Q (unvisited nodes) */
#else
	uint8_t Q[50] ={0}; /* All nodes initially in Q (unvisited nodes) */
#endif
	
	uint8_t alt =0;
	uint8_t u =0;
	uint8_t v =0;
	uint8_t j =0;
	
	memset(Route,0,sizeof(Route));
	RouteDist[sourceID - 1] =0; /* Distance from source to source */
	RoutePrev[sourceID - 1] =0; /* Previous node in optimal path initialization undefined */
	
	/* Check adjacent Neighbors first! */
	for(int col =1; col <= 6; col++){
		if(Array[sourceID - 1][col] && ((Array[sourceID - 1][col] >> 3) == desID)){
			RouteDist[desID - 1] =1;
			Route[0] =desID;
			return col;
		}
	}
	
	/* Initialization */
	for(int i =1; i <= N; i++){
		/* Where i has not yet been removed from Q (unvisited nodes) */
		if(i != sourceID){
			/* Unknown distance function from source to i */
			RouteDist[i - 1] =0xFF;
			/* Previous node in optimal path from source */
			RoutePrev[i - 1] =0;
		}
	}
	
	/* Algorithm */
	while(!QnotEmpty(Q)){
		/* Source node in first case */
		u =minArr(RouteDist,Q) + 1;
		if(u == desID){
			goto finishedRoute;
		}
		else
			Q[u - 1] =1; /* Remove u from Q */

		/* For each neighbor v where v is still in Q */
		/* Check all module ports */
		for(uint8_t n =1; n <= 6; n++){
			/* There's a neighbor v at this port n */
			if(Array[u - 1][n]){
				v =(Array[u - 1][n] >> 3);
				/* v is still in Q */
				if(!Q[v - 1]){
					/* Add one hop */
					alt =RouteDist[u - 1] + 1;
					/* A shorter path to v has been found */
					if(alt < RouteDist[v - 1]){
						RouteDist[v - 1] =alt;
						RoutePrev[v - 1] =u;
					}
				}
			}
		}
	}
	
	finishedRoute:

	/* Build the virtual Route */
	/* Construct the shortest path with a stack Route */
	while(RoutePrev[u - 1]){
		/* Push the vertex onto the stack */
		Route[j++] =u;
		/* Traverse from target to source */
		u =RoutePrev[u - 1];
	}
	
	/* Check which port leads to the correct module */
	for(int col =1; col <= 6; col++){
		if(Array[sourceID - 1][col] && ((Array[sourceID - 1][col] >> 3) == Route[RouteDist[desID - 1] - 1])){
			return col;
		}
	}
	
	return 0;
}

/***************************************************************************/
/* Used by FoundRoute: Find the index of the minimum module in dist that is still unvisited */
uint8_t minArr(uint8_t *arr,uint8_t *Q){
	uint8_t smallest =0xFF;
	uint8_t index =0;
	
	/* Consider first element as smallest */
	/* Not visited yet */
	if(!Q[0])
		smallest =arr[0];
	
	for(int i =0; i < N; i++){
		if((arr[i] < smallest) && !Q[i]){
			smallest =arr[i];
			index =i;
		}
	}
	
	return index;
}

/***************************************************************************/
/* Used by FoundRoute: Check if Q is empty (all modules have been visited) */
uint8_t QnotEmpty(uint8_t *Q){
	char temp =1;
	
	for(int i =0; i < N; i++){
		temp &=Q[i];
	}
	
	return temp;
}

/***************************************************************************/
/* Display Array topology in human-readable format through module port */
void DisplayTopology(uint8_t port){
	/* Print table header */
	sprintf(pcUserMessage,"\n\r(Module:Port)\t\t");
	writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
	HAL_MAX_DELAY);
	for(uint8_t i =1; i <= NUM_OF_PORTS; i++){
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
		strncpy(pcUserMessage,ModulePNstring[(Array[row][0])],5);
		writePxMutex(port,pcUserMessage,5,cmd50ms,HAL_MAX_DELAY);
		writePxMutex(port,"\t",1,cmd50ms,HAL_MAX_DELAY);
		/* Connections */
		for(uint8_t col =1; col <= NUM_OF_PORTS; col++){
			if(!Array[row][col])
				sprintf(pcUserMessage,"%d\t",0);
			else
				sprintf(pcUserMessage,"%d:%d\t",(Array[row][col] >> 3),(Array[row][col] & 0x07));
			writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,
			HAL_MAX_DELAY);
		}
		writePxMutex(port,"\n\r",2,cmd50ms,HAL_MAX_DELAY);
	}
	
	writePxMutex(port,"\n",1,cmd50ms,HAL_MAX_DELAY);
}

/***************************************************************************/
/* Display ports directions in human-readable format through module port */
void DisplayPortsDir(uint8_t port){
	sprintf(pcUserMessage,"\n\rThese ports are reversed:");
	writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,HAL_MAX_DELAY);
	
	for(uint8_t i =1; i <= N; i++){
		for(uint8_t p =1; p <= MAX_NUM_OF_PORTS; p++){
			if((ArrayPortsDir[i - 1] & (0x8000 >> (p - 1)))) /* Port is reversed */
			{
				sprintf(pcUserMessage,"\n\rModule %d : P%d",i,p);
				writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,HAL_MAX_DELAY);
			}
		}
	}
	
	sprintf(pcUserMessage,"\n\n\rAll other ports are normal\n\r");
	writePxMutex(port,pcUserMessage,strlen(pcUserMessage),cmd50ms,HAL_MAX_DELAY);
}

/***************************************************************************/
/* Display a description of current module status (Firmware, Ports, P2P DMAs) */
void DisplayModuleStatus(uint8_t port){
	int8_t *pcOutputString;
	uint16_t temp =0;
	
	/* Obtain the address of the output buffer. */
	pcOutputString =FreeRTOS_CLIGetOutputBuffer();
	
	strcpy((char* )pcOutputString,"");
	
	sprintf(pcUserMessage,"\n\r*** Module %d Status ***\n",myID);
	strcat((char* )pcOutputString,pcUserMessage);
	sprintf(pcUserMessage,"\n\rConnected via port: P%d\n\r",pcPort);
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
	for(uint8_t i =1; i <= NUM_OF_PORTS; i++){
		sprintf(pcUserMessage,"P%d: ",i);
		strcat((char* )pcOutputString,pcUserMessage);
		switch(PortStatus[i]){
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
		if(UARTDMAHandler[i - 1]->Instance == 0){
			sprintf(pcUserMessage,"\n\rStreaming DMA %d is free",i);
			strcat((char* )pcOutputString,pcUserMessage);
		}
		else{
			sprintf(pcUserMessage,"\n\rStreaming DMA %d is streaming from P%d to P%d",i,GetPort(UARTDMAHandler[i - 1]->Parent),GetPort(dmaStreamDst[i - 1]));
			strcat((char* )pcOutputString,pcUserMessage);
		}
	}
	strcat((char* )pcOutputString,"\n\r");
	
	/* Ports direction */
	strcat((char* )pcOutputString,"\n\rThese ports are reversed: ");
	temp =strlen((char* )pcOutputString);
	for(uint8_t p =1; p <= NUM_OF_PORTS; p++){
		if((ArrayPortsDir[myID - 1] & (0x8000 >> (p - 1)))) /* Port is reversed */
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

/***************************************************************************/
/* Extract module ID from it's alias, ID string or keyword */
int16_t GetID(char *string){
	uint8_t id =0, i =0;
	
	/* Check keywords */
	if(!strcmp(string,"me"))
		return myID;
	else if(!strcmp(string,"all"))
		return BOS_BROADCAST;
	/* Check IDs */
	else if(string[0] == '#'){
		id =atol(string + 1);
		if(id > 0 && id <= N)
			return id;
		else if(id == myID)
			return myID;
		else
			return BOS_ERR_WrongID;
	} /* Check alias */
	else{
		/* Check module alias */
		for(i =0; i < N; i++){
			if(!strcmp(string,ModuleAlias[i]) && (*string != 0))
				return (i);
		}
		
		/* Check group alias */
		for(i =0; i < MAX_NUM_OF_GROUPS; i++){
			if(!strcmp(string,GroupAlias[i]))
				return (BOS_MULTICAST | (i << 8));
		}
		
		return BOS_ERR_WrongName;
	}
}

/***************************************************************************/
/* Name a module with an alias */
BOS_Status NameModule(uint8_t module,char *alias){
	BOS_Status result =BOS_OK;
	int i =0;
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;
	
	/* 1. Check module alias with keywords */
	for(i =0; i < NUM_OF_KEYWORDS; i++){
		if(!strcmp(alias,BOSkeywords[i]))
			return BOS_ERR_Keyword;
	}
	
	/* 2. Check module alias with other module aliases */
	for(i =1; i < N; i++){
		if(!strcmp(alias,ModuleAlias[i]))
			return BOS_ERR_ExistingAlias;
	}
	
	/* 3. Check module alias with group aliases */
	for(i =0; i < MAX_NUM_OF_GROUPS; i++){
		if(!strcmp(alias,GroupAlias[i]))
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
	strcpy(ModuleAlias[module],alias);
	
	/* 6. Share new module alias with other modules */

	/* 7. Save new alias to emulated EEPROM */
	result =SaveEEalias();
	
	return result;
}

/***************************************************************************/
/* Add a module to this group */
BOS_Status AddModuleToGroup(uint8_t module,char *group){
	BOS_Status result =BOS_OK;
	int i =0, j =0;
	static const CLI_Definition_List_Item_t *pxCommand = NULL;
	const int8_t *pcRegisteredCommandString;
	size_t xCommandStringLength;
	
	/* Check alias with other group aliases */

	for(i =0; i < MAX_NUM_OF_GROUPS; i++){
		/* This group already exists */
		if(!strcmp(group,GroupAlias[i])){
			/* 1. Add this module to the group */
			GroupModules[module - 1] |=(0x0001 << i);
			
			/* 2. Save group to emulated EEPROM -- Should call this manually */
			//result = SaveEEgroup();			
			return result;
		}
	}
	
	/* This is a new group - Verify alias and create the group */

	/* 1. Check group alias with keywords */
	for(j =0; j < NUM_OF_KEYWORDS; j++){
		if(!strcmp(group,BOSkeywords[j]))
			return BOS_ERR_Keyword;
	}
	
	/* 2. Check group alias with module aliases */
	for(j =1; j < N; j++){
		if(!strcmp(group,ModuleAlias[j]))
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
	for(i =0; i < MAX_NUM_OF_GROUPS; i++){
		if(!GroupAlias[i][0]){
			strcpy(GroupAlias[i],group);
			break;
		}
	}
	
	/* 5. Add this module to the new group */
	GroupModules[module - 1] |=(0x0001 << i);
	
	/* 6. Share new group with other modules */

	/* 7. Save new group to emulated EEPROM - Should call this manually */
	//result = SaveEEgroup();			
	return result;
}

/***************************************************************************/
/* @breif: Write a value to a remote module.
 * @Note: in the destination call AddBOSvar(VariableFormat_t format,uint32_t address) to assign an index to a new BOS variable.
 * @param1: module: Remote module ID.
 * @param2: localVarAddress: Local memory address (RAM).
 * @param3: remoteVarAddress: Remote memory address (RAM). Write either BOS variables from 1 to MAX_BOS_VARS or a virtual RAM address.
 * @param4: format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL).
 * @param5: timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
 * @retval: BOS_Status.
 */
BOS_Status WriteToRemote(uint8_t module,uint32_t localVarAddress,uint32_t remoteVarAddress,VariableFormat_t format,uint32_t timeout/*,uint8_t force*/){

//	uint8_t response;
	uint16_t code;

	/* Check whether response is enabled or disabled */
//	response =OptionByte.Response;
//	if(timeout)
//		OptionByte.Response = BOS_RESPONSE_MSG;
//	else
//		OptionByte.Response = BOS_RESPONSE_NONE;

//	/* Check if a force write is needed */
//	if(force)
//		code = CODE_WRITE_REMOTE_FORCE;
//	else
		code = CODE_WRITE_REMOTE;

	/* Writing to a BOS var */
	if(remoteVarAddress < FLASH_BASE){
		MessageParams[0] =remoteVarAddress;			// Send BOS variable index
		MessageParams[1] =format;						// Send local format
		/* Send variable value based on local format */
		switch(format){
			case FMT_BOOL:
			case FMT_UINT8:
				MessageParams[2] =*(__IO uint8_t* )localVarAddress;
				SendMessageToModule(module,CODE_WRITE_REMOTE,3);
				break;
			case FMT_INT8:
				MessageParams[2] =*(__IO int8_t* )localVarAddress;
				SendMessageToModule(module,CODE_WRITE_REMOTE,3);
				break;
			case FMT_UINT16:
				MessageParams[2] =(uint8_t )((*(__IO uint16_t* )localVarAddress) >> 0);
				MessageParams[3] =(uint8_t )((*(__IO uint16_t* )localVarAddress) >> 8);
				SendMessageToModule(module,CODE_WRITE_REMOTE,4);
				break;
			case FMT_INT16:
				MessageParams[2] =(uint8_t )((*(__IO int16_t* )localVarAddress) >> 0);
				MessageParams[3] =(uint8_t )((*(__IO int16_t* )localVarAddress) >> 8);
				SendMessageToModule(module,CODE_WRITE_REMOTE,4);
				break;
			case FMT_UINT32:
				MessageParams[2] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 0);
				MessageParams[3] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 8);
				MessageParams[4] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 16);
				MessageParams[5] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 24);
				SendMessageToModule(module,CODE_WRITE_REMOTE,6);
				break;
			case FMT_INT32:
				MessageParams[2] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 0);
				MessageParams[3] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 8);
				MessageParams[4] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 16);
				MessageParams[5] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 24);
				SendMessageToModule(module,CODE_WRITE_REMOTE,6);
				break;
			case FMT_FLOAT:
				MessageParams[2] =*(__IO uint8_t* )(localVarAddress + 0);
				MessageParams[3] =*(__IO uint8_t* )(localVarAddress + 1);
				MessageParams[4] =*(__IO uint8_t* )(localVarAddress + 2);
				MessageParams[5] =*(__IO uint8_t* )(localVarAddress + 3);
				MessageParams[6] =*(__IO uint8_t* )(localVarAddress + 4);
				MessageParams[7] =*(__IO uint8_t* )(localVarAddress + 5);
				MessageParams[8] =*(__IO uint8_t* )(localVarAddress + 6);
				MessageParams[9] =*(__IO uint8_t* )(localVarAddress + 7); // You cannot bitwise floats
				SendMessageToModule(module,CODE_WRITE_REMOTE,10);
				break;
			default:
				break;
		}
	}
	/* Writing to a memory address */
	else{
		MessageParams[0] =0;
		MessageParams[1] =format;							// Local format
		MessageParams[2] =(uint8_t )(remoteVarAddress >> 24);
		MessageParams[3] =(uint8_t )(remoteVarAddress >> 16); // Remote address
		MessageParams[4] =(uint8_t )(remoteVarAddress >> 8);
		MessageParams[5] =(uint8_t )remoteVarAddress;
		/* Send variable value based on local format */
		switch(format){
			case FMT_BOOL:
			case FMT_UINT8:
				MessageParams[6] =*(__IO uint8_t* )localVarAddress;
				SendMessageToModule(module,code,7);
				break;
			case FMT_INT8:
				MessageParams[6] =*(__IO int8_t* )localVarAddress;
				SendMessageToModule(module,code,7);
				break;
			case FMT_UINT16:
				MessageParams[6] =(uint8_t )((*(__IO uint16_t* )localVarAddress) >> 0);
				MessageParams[7] =(uint8_t )((*(__IO uint16_t* )localVarAddress) >> 8);
				SendMessageToModule(module,code,8);
				break;
			case FMT_INT16:
				MessageParams[6] =(uint8_t )((*(__IO int16_t* )localVarAddress) >> 0);
				MessageParams[7] =(uint8_t )((*(__IO int16_t* )localVarAddress) >> 8);
				SendMessageToModule(module,code,8);
				break;
			case FMT_UINT32:
				MessageParams[6] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 0);
				MessageParams[7] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 8);
				MessageParams[8] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 16);
				MessageParams[9] =(uint8_t )((*(__IO uint32_t* )localVarAddress) >> 24);
				SendMessageToModule(module,code,10);
				break;
			case FMT_INT32:
				MessageParams[6] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 0);
				MessageParams[7] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 8);
				MessageParams[8] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 16);
				MessageParams[9] =(uint8_t )((*(__IO int32_t* )localVarAddress) >> 24);
				SendMessageToModule(module,code,10);
				break;
			case FMT_FLOAT:
				MessageParams[6] =*(__IO uint8_t* )(localVarAddress + 0);
				MessageParams[7] =*(__IO uint8_t* )(localVarAddress + 1);
				MessageParams[8] =*(__IO uint8_t* )(localVarAddress + 2);
				MessageParams[9] =*(__IO uint8_t* )(localVarAddress + 3);
				MessageParams[10] =*(__IO uint8_t* )(localVarAddress + 4);
				MessageParams[11] =*(__IO uint8_t* )(localVarAddress + 5);
				MessageParams[12] =*(__IO uint8_t* )(localVarAddress + 6);
				MessageParams[13] =*(__IO uint8_t* )(localVarAddress + 7); // You cannot bitwise floats
				SendMessageToModule(module,code,14);
				break;
			default:
				break;
		}
	}
	
//	/* Restore response settings to default */
//	OptionByte.Response =response;

	/* If confirmation is requested, wait for it until timeout */
	if(timeout){
		uint32_t t0 =HAL_GetTick();
		while((ResponseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){};
		return ResponseStatus;
	}
	
	return BOS_OK;
}

///***************************************************************************/
///* --- Write a value to a remote module and force full-page erase when writing to Flash.
// module: Remote module ID.
// localAddress: Local memory address (RAM or Flash).
// remoteAddress: Remote memory address (RAM or Flash). Use the 1 to MAX_BOS_VARS to write BOS variables.
// format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
// timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
// */
//BOS_Status WriteRemoteForce(uint8_t module,uint32_t localAddress,uint32_t remoteAddress,VariableFormat_t format,uint32_t timeout){
//	return WriteToRemote(module,localAddress,remoteAddress,format,timeout,1);
//}

/***************************************************************************/
/* @breif: Read a variable from a remote module.
 * @param1: module: Remote module ID.
 * @param2: remoteVarAddress: Remote value memory address (RAM). Use the 1 to MAX_BOS_VARS to read BOS variables with unknown addresses.
 * @param3: remoteFormat (output): Pointer to format (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL).
 * @param4: timeout: Read timeout in msec.
 * @retval: pointer to the remote value. Cast this pointer to match the appropriate format.
 */
uint32_t* ReadRemoteVar(uint8_t module,uint32_t remoteVarAddress,VariableFormat_t *remoteFormat,uint32_t timeout){
	/* Reset local buffer */
	RemoteBuffer = REMOTE_BOS_VAR;
	
	/* Send the Message */
	MessageParams[0] =remoteVarAddress + REMOTE_BOS_VAR; // Send BOS variable index
	SendMessageToModule(module,CODE_READ_REMOTE,1);
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	while((ResponseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){
	};
	
	/* Return the read value address */
//	if(ResponseStatus == BOS_OK){
	/* Return the remote var format */
	*remoteFormat =RemoteVarFormat;

	return ((uint32_t* )&RemoteBuffer);
//	}
//	else
//		return NULL;
}

/***************************************************************************/
/* @breif: Read a memory address from a remote module.
 * @param1: module: Remote module ID.
 * @param1: remoteVarAddress: Remote variable memory address (RAM).
 * @param1: requestedFormat (input): Requested format of remote memory location (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
 * @param1: timeout: Read timeout in msec.
 * @retval: pointer to the remote value. Cast this pointer to match the appropriate format.
 */
uint32_t* ReadRemoteMemory(uint8_t module,uint32_t remoteVarAddress,VariableFormat_t requestedFormat,uint32_t timeout){
	/* Reset local buffer */
	RemoteBuffer = REMOTE_MEMORY_ADD;
	
	/* Send the Message */
	MessageParams[0] = REMOTE_MEMORY_ADD;
	MessageParams[1] =requestedFormat;						// Requested format
	MessageParams[2] =(uint8_t )(remoteVarAddress >> 24);
	MessageParams[3] =(uint8_t )(remoteVarAddress >> 16); // Remote address
	MessageParams[4] =(uint8_t )(remoteVarAddress >> 8);
	MessageParams[5] =(uint8_t )remoteVarAddress;
	RequestFormat =requestedFormat;	// Set a flag that we requested a memory location
	SendMessageToModule(module,CODE_READ_REMOTE,6);
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	while((ResponseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){
	};
	
	/* Return the read value address */
//	if(ResponseStatus == BOS_OK)
		return ((uint32_t* )&RemoteBuffer);
//	else
//		return NULL;
}

/***************************************************************************/
/* @breif: Read a parameter from a remote module.
 This API returns a pointer to the remote parameter. Cast this pointer to match the appropriate format.
 If the returned parameter is NULL, then remote parameter does not exist or remote module is not responsive.
 * @param1: module: Remote module ID.
 * @param2: paramString: Remote parameter string address (RAM). Write either BOS variables from 1 to MAX_BOS_VARS or a virtual RAM address.
 * @param3: remoteFormat (output): Pointer to format (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL).
 * @param4: timeout: Read timeout in msec.
 * @retval: pointer to the remote value. Cast this pointer to match the appropriate format.
 */
uint32_t* ReadRemoteParam(uint8_t module,char *paramString,VariableFormat_t *remoteFormat,uint32_t timeout){
	/* Reset local buffer */
	RemoteBuffer = REMOTE_MODULE_PARAM;
	
	/* Send the Message */
	MessageParams[0] = REMOTE_MODULE_PARAM;
	memcpy(&MessageParams[1],paramString,strlen(paramString)); // copy BOS parameter index to location
	SendMessageToModule(module,CODE_READ_REMOTE,strlen(paramString) + 1);
	
	/* Wait until read is complete */
	uint32_t t0 =HAL_GetTick();
	while((ResponseStatus != BOS_OK) && ((HAL_GetTick() - t0) < timeout)){
	};
	
	/* Return the read value address */
	if(ResponseStatus == BOS_OK){
		/* Return the remote var format */
		*remoteFormat =RemoteVarFormat;
		
		return ((uint32_t* )&RemoteBuffer);
	}
	else
		return NULL;
}

/***************************************************************************/
/* @breif:  Write a value to a remote module.
 * @Note:   in the destination call AddBOSvar(VariableFormat_t format,uint32_t address) to assign an index to a new BOS variable.
 * @param1: dstModuleID: Remote module ID.
 * @param2: localVarAddress: Local memory address (RAM).
 * @param3: remoteVarAddress: Remote memory address (RAM). Write either BOS variables from 1 to MAX_BOS_VARS or a virtual RAM address.
 * @param4: format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
 * @param5: timeout: Write confirmation timeout in msec. Use 0 to disable confirmation.
 * @retval: BOS_Status.
 */
BOS_Status WriteRemote(uint8_t dstModuleID,uint32_t localVarAddress,uint32_t remoteVarAddress,VariableFormat_t format,uint32_t timeout){
	return WriteToRemote(dstModuleID,localVarAddress,remoteVarAddress,format,timeout/*,0*/);
}

/***************************************************************************/
/* @breif: Assign an index to a new BOS variable.
 * @Note: BOS variables must be global or static to ensure we don't reference a stack address.
 * @param1: format: Local format sent to remote module (FMT_UINT8, FMT_INT8, FMT_UINT16, FMT_INT16, FMT_UINT32, FMT_INT32, FMT_FLOAT, FMT_BOOL)
 * @param2: address: Local memory address (RAM).
 * @retval: a new index to BOS variable.
 */
uint8_t AddBOSvar(VariableFormat_t format,uint32_t address){
	for(uint8_t v =0; v < MAX_BOS_VARS; v++){
		if((bosVarRegister[v] & 0x000F) == 0)		// Index not assigned yet
		{
			bosVarRegister[v] =format + ((address - SRAM_BASE) << 16);
			return (v + 1);
		}
	}
	
	/* Memory full */
	return 0;
}

/***************************************************************************/
/* Make a data string with format weekday / month / date / year */
char* GetDateString(void){
	static const char formatDateStr[] ="%s %02d/%02d/%04d";
	char *buffer =malloc(30 * sizeof(int8_t));
	memset(buffer,0x00,30 * sizeof(int8_t));
	sprintf(buffer,formatDateStr,WeekdayString[BOS.Date.Weekday - 1],BOS.Date.Month,BOS.Date.Day,BOS.Date.Year);
	return buffer;
}

/***************************************************************************/
/* Make a time string with format hour / minute / second */
char* GetTimeString(void){
	static const char formatTimeStr[] ="%02d:%02d:%02d";
	char *buffer =malloc(10 * sizeof(int8_t));
	memset(buffer,0x00,10 * sizeof(int8_t));
	sprintf(buffer,formatTimeStr,BOS.Time.Hours,BOS.Time.Minutes,BOS.Time.Seconds);
	return buffer;
}

/***************************************************************************/
/* Bridge two Array/communication ports together */
BOS_Status Bridge(uint8_t port1,uint8_t port2){
	/* Link the ports together with an infinite DMA stream */
	return StartScastDMAStream(port1,myID,port2,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,true);
}

/***************************************************************************/
/* Un-bridge two Array/communication ports */
BOS_Status Unbridge(uint8_t port1,uint8_t port2){
	/* Remove the stream from EEPROM */
	SaveEEstreams(0,0,0,0,0,0,0,0,0);
	
	/* Stop the DMA streams and enable messaging back on these ports */
	if(UARTDMAHandler[port1 - 1]->Instance != 0 && UARTDMAHandler[port2 - 1]->Instance != 0){
		SwitchStreamDMAToMsg(port1);
		SwitchStreamDMAToMsg(port2);
		return BOS_OK;
	}
	else if(UARTDMAHandler[port1 - 1]->Instance != 0){
		SwitchStreamDMAToMsg(port1);
		return BOS_OK;
	}
	else if(UARTDMAHandler[port2 - 1]->Instance != 0){
		SwitchStreamDMAToMsg(port2);
		return BOS_OK;
	}
	else{
		return BOS_ERR_WrongValue;
	}
}

/***************************************************************************/
/* Print formatted text to one of the module ports */
BOS_Status printfp(uint8_t port,char *str){
	if(writePxMutex(port,str,strlen(str),1,1) == HAL_OK)
		return BOS_OK;
	else
		return BOS_ERROR;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
