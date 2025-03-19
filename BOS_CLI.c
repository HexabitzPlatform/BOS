/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_CLI.c
 Description   : Source code for Bitz Operating Command Line Interface(CLI).

 */

#include "BOS.h"

/* Local Variables *********************************************************/
static char *pcWelcomeMessage ="\n\r\n\r====================================================	\
     \n\r====================================================	\
     \n\r||            Welcome to BitzOS CLI!              ||	\
	 \n\r||       (C) COPYRIGHT HEXABITZ 2017-2025.        ||	\
     \n\r||                                                ||	\
	 \n\r||      Please check the project website at       ||	\
	 \n\r||             http://hexabitz.com/               ||	\
     \n\r||                                                ||	\
     \n\r||   Type help for a list of available commands.  ||	\
     \n\r====================================================	\
     \n\r====================================================	\
     \n\n\r";
static char *pcNewLine ="\r\n";
static char *pcEndOfCommandOutputString ="\r\n[Press ENTER to execute the previous command again]\r\n>";
char pcWelcomePortMessage[40] ={0};
uint16_t timedoutMsg = 0;
//uint8_t numOfRecordedSnippets =0;
snippet_t snippets[MAX_SNIPPETS];		/* Buffer to hold CLI Snippets */

/* Exported variables ******************************************************/
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];

/* Global function prototypes **********************************************/
bool ParseSnippetCommand(char *snippetBuffer,int8_t *cliBuffer);

/* Private function prototypes *********************************************/
BOS_Status AddSnippet(uint8_t code,char *string);
BOS_Status ParseSnippetCondition(char *string);
bool CheckSnippetCondition(uint8_t index);
void CLI_CommandParser(uint8_t port,bool enableOutput,int8_t *cInputString,int8_t *pcOutputString);

/* BOS exported functions **************************************************/
extern void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
extern uint8_t IsModuleParameter(char *name);
extern uint8_t IsMathOperator(char *string);
extern uint8_t SaveSnippetsToRO(void);
//extern char Processor_type(uint8_t module_name);

/***************************************************************************/
/*****************************  Private Functions **************************/
/***************************************************************************/

void prvCLITask(void *pvParameters){
	char cRxedChar ='\0';
	int8_t cInputIndex =0;
	int8_t *pcOutputString = NULL;
	static int8_t cInputString[cmdMAX_INPUT_SIZE];
	static int8_t cLastInputString[cmdMAX_INPUT_SIZE];
	uint16_t chr =0;

	(void )pvParameters;

	/* Wait indefinitely until a '\r' is received on one of the ports */
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

	/* Obtain the address of the output buffer */
	pcOutputString =FreeRTOS_CLIGetOutputBuffer();

	/* Restore baud rate to the default for all ports except the PC communication port */
	if(BOS.clibaudrate != DEF_ARRAY_BAUDRATE){
		for(uint8_t port =1; port <= NumOfPorts; port++){
			if(port != PcPort)
				UpdateBaudrate(port,DEF_ARRAY_BAUDRATE);
		}
	}

	/* Send the welcome message */
	sprintf(pcWelcomePortMessage,"Connected to module %d (%s), port P%d.\n\n\r>",myID,modulePNstring[myPN],PcPort);
	writePxITMutex(PcPort,pcWelcomeMessage,strlen(pcWelcomeMessage),10);
	writePxITMutex(PcPort,pcWelcomePortMessage,strlen(pcWelcomePortMessage),10);

	for(;;){

		/* Check if a new character has been received.
		 * Reading only one byte at a time using CLI Flags. */
		if(Read_In_CLI_Task_Flag == 1){
			cRxedChar =CLI_Data;
			CLI_Data =0;

			/* Clear the flag to allow new input */
			Read_In_CLI_Task_Flag =0;

			/* Echo the received character back to the terminal */
			writePxITMutex(PcPort,&cRxedChar,1,10);

			switch(cRxedChar){
				/* If the Enter key is pressed, process the command */
				case '\r': /* Enter key */
					writePxITMutex(PcPort,pcNewLine,strlen(pcNewLine),10);

					/* Repeat last command if input is empty */
					if(cInputIndex == 0){
						strcpy((char* )cInputString,(char* )cLastInputString);
					}

					/* Parse the user input and execute the command */
					CLI_CommandParser(PcPort,true,cInputString,pcOutputString);

					/* Store last command for potential reuse */
					strcpy((char* )cLastInputString,(char* )cInputString);

					cInputIndex =0;
					memset(cInputString,0x00,cmdMAX_INPUT_SIZE);
					break;

					/* Ignore newline characters (in case input comes with \r\n) */
				case '\n':
					break;

					/* Handle backspace (user wants to delete a character)
					 * 127: ASCII code for Delete key */
				case '\b':
				case 127:
					if(cInputIndex > 0){
						cInputIndex--;
						cInputString[cInputIndex] ='\0';
					}
					break;

					/* Accept only printable characters (ASCII values between 32 and 126) */
				default:
					if(isprint((unsigned char )cRxedChar)){
						if(cInputIndex < cmdMAX_INPUT_SIZE){
							cInputString[cInputIndex] =cRxedChar;
							cInputIndex++;
						}
					}
					break;
			}
		}
		taskYIELD();
	}
}

/***************************************************************************/
/* Hexabitz CLI command parser */
void CLI_CommandParser(uint8_t port,bool enableOutput,int8_t *cInputString,int8_t *pcOutputString){
	portBASE_TYPE xReturned;
	char idString[MaxLengthOfAlias] ={0};
	char *loc =0;
	static uint8_t recordSnippet;
	static uint8_t group;
	int16_t id =0;
	
	/* Pass the received command to the command interpreter repeatedly until it returns pdFALSE */
	do{
		/* Convert input string to lower case */
		StringToLowerCase((char* )cInputString);
		
		/* Check for a conditional statement (if) */
		if(!recordSnippet && !strncmp((char* )cInputString,"if ",3)){
			/* Add the condition to Command Snippets (after removing "if " */
			if(AddSnippet(SNIPPET_CONDITION,(char* )(cInputString + 3)) != BOS_OK){
				sprintf((char* )pcOutputString,"\nCannot store more Command Snippets. Please delete existing ones and try again.\n\r");
				recordSnippet =0;
			}
			else{
				/* Start recording Commands after the condition */
				recordSnippet = SNIPPET_COMMANDS;
				pcOutputString[0] ='\r';
			}
			xReturned = pdFALSE;
		}

		/* Check for the end of a conditional command (end if) */
		else if(recordSnippet && !strncmp((char* )cInputString,"end if",6)){
			/* Stop recording Commands for the conditional Command Snippet */
			recordSnippet =0;
			/* Activate the Snippet */
			AddSnippet(SNIPPET_ACTIVATE,"");
			/* If snippet saved successfuly */
			sprintf((char* )pcOutputString,"\nConditional statement accepted and added to Command Snippets.\n\r");
			xReturned = pdFALSE;
		}

		/* Should I record any Command Snippets? */
		else if(recordSnippet == SNIPPET_COMMANDS){
			/* Add this Command to Command Snippets */
			if(AddSnippet(SNIPPET_COMMANDS,(char* )cInputString) != BOS_OK)
				sprintf((char* )pcOutputString,"\nCannot store more Command Snippets. Please delete existing ones and try again.\n\r");
			else
				pcOutputString[0] ='\r';
			xReturned = pdFALSE;
		}

		/* Parse a normal Command */
		else{

			/* Check if command contains a dot and it's not "BOS." or a decimal number */
			loc =strchr((char* )cInputString,'.');

			if(loc != NULL && strncmp((char* )loc - 3,"bos",3) && !isdigit(*(loc + 1))){

				/* Extract module ID/alias or group alias */
				strncpy(idString,(char* )cInputString,(size_t )(loc - (char* )cInputString));
				id =GetID(idString);

				if(id == myID){
					/* Extract and process the command */
					xReturned =FreeRTOS_CLIProcessCommand((const signed char* )(loc + 1),pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
				}

				else if(id == BOS_ERR_WrongName){
					sprintf((char* )pcOutputString,"Wrong module name! Please try again.\n\r");
					xReturned = pdFALSE;
				}

				else if(id == BOS_ERR_WrongID){
					sprintf((char* )pcOutputString,"Wrong module ID! Please try again.\n\r");
					xReturned = pdFALSE;
				}

				/* Handle broadcast */
				else if(id == BOS_BROADCAST){
					memset(broadcastResponse,0x00,sizeof(broadcastResponse));
					strncpy((char* )messageParams,loc + 1,(size_t )(strlen((char* )cInputString) - strlen((char* )idString) - 1));
					BroadcastMessage(myID,BOS_BROADCAST,CODE_CLI_COMMAND,strlen((char* )cInputString) - strlen((char* )idString));		// Send terminating zero
					/* Execute locally */
					xReturned =FreeRTOS_CLIProcessCommand((const signed char* )(loc + 1),pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
					strcat((char* )pcOutputString,"Command broadcasted to all\n\r");
				}

				/* Handle multicast */
				else if((uint8_t )id == BOS_MULTICAST){
					group =id >> 8;
					memset(broadcastResponse,0x00,sizeof(broadcastResponse));
					strncpy((char* )messageParams,loc + 1,(size_t )(strlen((char* )cInputString) - strlen((char* )idString) - 1));
					BroadcastMessage(myID,group,CODE_CLI_COMMAND,strlen((char* )cInputString) - strlen((char* )idString));		// Send terminating zero
					/* Do I need to execute locally? */
					if(InGroup(myID,group))
						xReturned =FreeRTOS_CLIProcessCommand((const signed char* )(loc + 1),pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
					sprintf((char* )pcOutputString,"%sMulticast Command forwarded to group %s\n\r",pcOutputString,idString);
				}

				/* Handle forwarding commands */
				/* Special commands that convert into custom a Message */
				else{
					/* remote update */
					if(!strncmp((char* )loc + 1,"update",6)){
						OptionByte.Response = BOS_RESPONSE_NONE;
						SendMessageToModule(id,CODE_UPDATE,0);
						osDelay(100);
						/* Execute locally */
						remoteBootloaderUpdate(myID,id,PcPort,0);
					}
					else{
						/* Forward the command */
						strncpy((char* )messageParams,loc + 1,(size_t )(strlen((char* )cInputString) - strlen((char* )idString) - 1));
						SendMessageToModule(id,CODE_CLI_COMMAND,strlen((char* )cInputString) - strlen((char* )idString) - 1);
						sprintf((char* )pcOutputString,"Command forwarded to Module %d\n\r",id);

						if((strlen((char* )pcOutputString) > 0) && enableOutput)
							writePxMutex(port,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms,1);

						memset(pcOutputString,0x00,strlen((char* )pcOutputString));
					}

					/* Wait for response if needed */
					if(OptionByte.Response == BOS_RESPONSE_ALL){
						ulTaskNotifyTake(pdTRUE,1000);		//cmd500ms
						/* If timeout */
						if(responseStatus != BOS_OK){
							++timedoutMsg;
							sprintf((char* )pcOutputString,"%sModule %d is not reachable.\n\r",(char* )pcOutputString,id);
						}
					}
					xReturned = pdFALSE;
				}

			}
			else{
				/* Process the command locally */
				xReturned =FreeRTOS_CLIProcessCommand(cInputString,pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
			}
		}
		
		/* Write the generated string to the UART. */
		if(strlen((char* )pcOutputString) > 0 && enableOutput)
			writePxMutex(port,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms,HAL_MAX_DELAY);

		memset(pcOutputString,0x00,strlen((char* )pcOutputString));
		
	} while(xReturned != pdFALSE);
	
	memset(idString,0x00,MaxLengthOfAlias);
	
	/* Start to transmit a line separator, just to make the output easier to read. */
	if(!recordSnippet && enableOutput)
		writePxMutex(port,pcEndOfCommandOutputString,strlen(pcEndOfCommandOutputString),cmd50ms,HAL_MAX_DELAY);
	
}

/***************************************************************************/
/* Convert a string to lower case */
void StringToLowerCase(char *string){
	for(int i =0; string[i]; i++){
		string[i] =tolower(string[i]);
	}
}

/***************************************************************************/
/* Add a set of Commands to Command Snippets and activate */
BOS_Status AddSnippet(uint8_t code,char *string){

	char *temp = NULL;
	int currentLength =0;

	/* Reference to the last recorded snippet */
	snippet_t *currentSnippet =&snippets[numOfRecordedSnippets - 1];

	/* Ensure there is at least one snippet recorded */
	if(numOfRecordedSnippets == 0){
		return BOS_ERROR;
	}

	/* Check for codes */
	switch(code){
		case SNIPPET_ACTIVATE:
			/* Activate the last recorded snippet */
			currentSnippet->state = true;
			/* Save snippet state to read-only memory */
			SaveSnippetsToRO();
			break;
			
			/* Parse the condition string for the snippet */
		case SNIPPET_CONDITION:
			return ParseSnippetCondition(string);
			
			/* Handle adding commands to the snippet */
		case SNIPPET_COMMANDS:
			/* Check if a command buffer already exists */
			if(currentSnippet->cmd != NULL){
				/* Reallocate memory to accommodate the new command */
				currentLength =strlen(currentSnippet->cmd);

				/* Use a temporary pointer to avoid memory leaks in case of allocation failure */
				/* Add two more bytes for the ENTER key (0x13) and end of string (0x00) */
				char *temp =(char* )realloc(currentSnippet->cmd,currentLength + strlen(string) + 2);

				if(temp == NULL){
					return BOS_ERR_SNIP_MEM_FULL;  /* Memory allocation failed */
				}

				currentSnippet->cmd =temp;

				/* Append the new command */
				*(currentSnippet->cmd + currentLength) =0x13;  /* ENTER key separator (0x13) */
				strcpy(currentSnippet->cmd + currentLength + 1,string);
			}
			else{
				/* Allocate a new buffer for the command */
				currentSnippet->cmd =(char* )malloc(strlen(string) + 1);

				if(currentSnippet->cmd == NULL){
					memset(currentSnippet,0,sizeof(snippet_t));  /* Reset snippet structure */
					return BOS_ERR_SNIP_MEM_FULL;  /* Memory allocation failed */
				}

				/* Copy the command into the allocated buffer */
				strcpy(currentSnippet->cmd,string);
			}
			break;
			
		default:
			return BOS_ERROR;
	}
	
	return BOS_OK;
}

/***************************************************************************/
/* Parse Snippet conditions into the internal buffer */
BOS_Status ParseSnippetCondition(char *string){

	BOS_Status status =BOS_OK;
	uint8_t port =0;
	static int8_t cInputString[cmdMAX_INPUT_SIZE];

	// A. Verify first there's still memory left to store Snippets	
	if(numOfRecordedSnippets == MAX_SNIPPETS){
		return BOS_ERR_SNIP_MEM_FULL;
	}
	// Initialize the next empty location
	else{
		snippets[numOfRecordedSnippets].cond.conditionType =0;
		snippets[numOfRecordedSnippets].cond.mathOperator =0;
		memset(snippets[numOfRecordedSnippets].cond.buffer1,0,4);
	}
	
	// B. Parse Snippets based on their condition type 
	
	// #1: Button event: condition starts with "bx." 
	if(string[0] == 'b' && string[2] == '.'){
		if(string[1] >= '0' && string[1] <= (NumOfPorts + '0'))		// Valid port number
		{
			port =string[1] - '0';
			snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_BUTTON_EVENT;
			snippets[numOfRecordedSnippets].cond.mathOperator =0;			// No math operations
			snippets[numOfRecordedSnippets].cond.buffer1[0] =port;		// Store button port number
			
			/* Store button event and event parameter if needed */
			if(!strncmp((char* )&string[3],"clicked",7)){
				snippets[numOfRecordedSnippets].cond.buffer1[1] =CLICKED;
				if((button[port].events & BUTTON_EVENT_CLICKED) != BUTTON_EVENT_CLICKED)		// Enable the event
//					SetButtonEvents(port,1,0,0,0,0,0,0,0,BUTTON_EVENT_MODE_OR);
					SetButtonEvents(port,CLICKED,BUTTON_EVENT_MODE_OR);
				status =BOS_OK;
			}
			else if(!strncmp((char* )&string[3],"double clicked",14)){
				snippets[numOfRecordedSnippets].cond.buffer1[1] =DBL_CLICKED;
				if((button[port].events & BUTTON_EVENT_DBL_CLICKED) != BUTTON_EVENT_DBL_CLICKED)
//					SetButtonEvents(port,0,1,0,0,0,0,0,0,BUTTON_EVENT_MODE_OR);
					SetButtonEvents(port,CLICKED,BUTTON_EVENT_MODE_OR);
				status =BOS_OK;
			}
			
			++numOfRecordedSnippets;		// Record a successful Snippet			
		}
	}
	// Module-related conditions (local only for now)
	else{
		strcpy((char* )cInputString,string);
		
		// This is probably a three part condition, extract them out
		char *firstPart, *secondPart, *thirdPart;
		uint8_t modPar1 =0, modPar2 =0;
		firstPart =strtok((char* )cInputString," ");
		secondPart =strtok( NULL," ");
		thirdPart =strtok( NULL," ");
		
		// Check if first part is module parameter or event
		if(firstPart == NULL){
			return BOS_ERR_WrongParam;
		}
		else{
			modPar1 =IsModuleParameter(firstPart);
			// Found a module parameter and no more strings
			if(modPar1 && secondPart == NULL && thirdPart == NULL){
				// #2: Module event
				snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_MODULE_EVENT;
				snippets[numOfRecordedSnippets].cond.buffer1[1] =modPar1;		// Leaving first buffer byte for remote module ID
				
				++numOfRecordedSnippets;		// Record a successful Snippet	
				return BOS_OK;
			}
			else if(secondPart != NULL && thirdPart != NULL){
				modPar2 =IsModuleParameter(thirdPart);
				if(modPar2) 		// Found a module parameter
				{
					// #4: Module parameter and parameter
					snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_MODULE_PARAM_PARAM;
					snippets[numOfRecordedSnippets].cond.buffer1[1] =modPar1;		// Leaving first buffer byte for remote module ID
					snippets[numOfRecordedSnippets].cond.buffer2[1] =modPar2;		// Leaving first buffer byte for remote module ID
				}
				else{
					// #3: Module parameter and constant	
					snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_MODULE_PARAM_CONST;
					snippets[numOfRecordedSnippets].cond.buffer1[1] =modPar1;		// Leaving first buffer byte for remote module ID
					// Extract the constant
					float constant =atof(thirdPart);
					memcpy(snippets[numOfRecordedSnippets].cond.buffer2,&constant,sizeof(float));		// This buffer can be misaligned and cause hardfault on F0
				}
				// Extract the math operator
				snippets[numOfRecordedSnippets].cond.mathOperator =IsMathOperator(secondPart);
				if(!snippets[numOfRecordedSnippets].cond.mathOperator)
					return BOS_ERR_WrongParam;
				
				++numOfRecordedSnippets;		// Record a successful Snippet
				return BOS_OK;
			}
			else{
				return BOS_ERR_WrongParam;
			}
		}
	}
	
	/* Note: after exiting this function, numOfRecordedSnippets refers to the next empty Snippet.
	 * Subtract by one to reference the last Snippet. */

	return status;
}

/***************************************************************************/
/* Check if Snippet conditional is true or false */
bool CheckSnippetCondition(uint8_t index){
	uint8_t temp8;
	float flt1, flt2;
	
	/* Check conditions based on Snippet tupe */

	switch(snippets[index].cond.conditionType){
		case SNIP_COND_BUTTON_EVENT:
			temp8 =snippets[index].cond.buffer1[0]; 	// Button port
			/* Check if button state matches Snippet button event */
			if(snippets[index].cond.buffer1[1] == button[temp8].state)
				return true;
			else
				return false;
			
		case SNIP_COND_MODULE_EVENT:
			break;
			
		case SNIP_COND_MODULE_PARAM_CONST:
			// Get the constant and module parameter values. 
			flt1 =*(float* )modParam[snippets[index].cond.buffer1[1] - 1].paramPtr;
			memcpy((uint8_t* )&flt2,&snippets[index].cond.buffer2,sizeof(float));		// This buffer can be misaligned and cause hardfault on F0
			// Compare them mathematically
			switch(snippets[index].cond.mathOperator){
				case MATH_EQUAL:
					if(flt1 == flt2)
						return true;
					break;
				case MATH_GREATER:
					if(flt1 > flt2)
						return true;
					break;
				case MATH_SMALLER:
					if(flt1 < flt2 && flt1 != 0.0f)
						return true;
					break;
				case MATH_GREATER_EQUAL:
					if(flt1 >= flt2)
						return true;
					break;
				case MATH_SMALLER_EQUAL:
					if(flt1 <= flt2 && flt1 != 0.0f)
						return true;
					break;
				case MATH_NOT_EQUAL:
					if(flt1 != flt2 && flt1 != 0.0f)
						return true;
					break;
				default:
					break;
			}
			break;
			
		case SNIP_COND_MODULE_PARAM_PARAM:
			break;
			
		default:
			break;
	}
	
	return false;
}

/***************************************************************************/
/* Execute activated Command Snippets */
BOS_Status ExecuteSnippet(void){
	BOS_Status result =BOS_OK;
	uint16_t s =0;
	int8_t *pcOutputString;
	static int8_t cInputString[cmdMAX_INPUT_SIZE];
	
	/* Must get this address even if output is not used otherwise memory will corrupt */
	/* Obtain the address of the output buffer.  Note there is no mutual
	 exclusion on this buffer as it is assumed only one command console
	 interface will be used at any one time. */
	pcOutputString =FreeRTOS_CLIGetOutputBuffer();
	
	/* Go through activated Snippets */
	for(s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].state)								// Check for activated Snippets
		{
			if(CheckSnippetCondition(s))				// Process Snippet condition
			{
				OptionByte.Response = BOS_RESPONSE_MSG;		// Disable CLI response
				// Loop over all recorded Snippet commands
				while(ParseSnippetCommand(snippets[s].cmd,(int8_t* )&cInputString) != false){
					/* Pass the received command to the command interpreter.  The
					 command interpreter is called repeatedly until it returns
					 pdFALSE as it might generate more than one string. */
					CLI_CommandParser(PcPort,false,cInputString,pcOutputString);
					
					/* Clear output buffer since we do not need it. Input buffer is cleared in  CLI_CommandParser */
					memset(pcOutputString,0x00,strlen((char* )pcOutputString));
				}
			}
		}
	}
	
	return result;
}

/***************************************************************************/
/****************************** Global Functions ***************************/
/***************************************************************************/
/* Parse Snippet commands into the internal buffer */
bool ParseSnippetCommand(char *snippetBuffer,int8_t *cliBuffer){
	static char *ptrStart, *ptrEnd;

	if(snippets[numOfRecordedSnippets - 1].cmd == NULL)
		return false;

	// Initialize the start pointer to snippet buffer address
	if(!ptrStart)
		ptrStart =snippetBuffer;

	// Did we already reach end of Snippet buffer?
	if(*ptrStart == 0x00){
		ptrStart =0;		// Initialize the start pointer for next Snippet
		cliBuffer = NULL;
		return false;
	}

	// Search the buffer for first occurance of 0x13 (ENTER key)
	ptrEnd =strchr(ptrStart,0x13);
	if(ptrEnd != NULL){
		strncpy((char* )cliBuffer,ptrStart,ptrEnd - ptrStart);
		ptrStart =ptrEnd + 1;
	}
	else{
		strcpy((char* )cliBuffer,ptrStart);
		ptrStart +=strlen((const char* )cliBuffer);
	}

	return true;
}
/***************************************************************************/
