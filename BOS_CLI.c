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
uint16_t TimedoutMsg = 0;
Snippet_t Snippets[MAX_SNIPPETS];		/* Buffer to hold CLI Snippets */

/* Exported variables ******************************************************/
extern uint8_t UARTRxBuf[NUM_OF_PORTS][MSG_RX_BUF_SIZE];

/* Global function prototypes **********************************************/
bool ParseSnippetCommand(char *snippetBuffer,int8_t *cliBuffer);
Module_Status GetModuleParameter(uint8_t paramIndex, float *value);

/* Private function prototypes *********************************************/
BOS_Status AddSnippet(uint8_t code,char *string);
BOS_Status ParseSnippetCondition(char *string);
bool CheckSnippetCondition(uint8_t index);
void CLI_CommandParser(uint8_t port,bool enableOutput,int8_t *cInputString,int8_t *pcOutputString);

/* BOS exported functions **************************************************/
extern void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
extern BOS_Status SetButtonEvents(uint8_t port, ButtonState_e buttonState, uint8_t mode);
extern uint8_t IsModuleParameter(char *name);
extern uint8_t IsMathOperator(char *string);
extern uint8_t SaveSnippetsToRO(void);

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
	if(BOS.cliBaudrate != DEF_ARRAY_BAUDRATE){
		for(uint8_t port =1; port <= NUM_OF_PORTS; port++){
			if(port != pcPort)
				UpdateBaudrate(port,DEF_ARRAY_BAUDRATE);
		}
	}

	/* Send the welcome message */
	sprintf(pcWelcomePortMessage,"Connected to module %d (%s), port P%d.\n\n\r>",myID,ModulePNstring[myPN],pcPort);
	writePxITMutex(pcPort,pcWelcomeMessage,strlen(pcWelcomeMessage),10);
	writePxITMutex(pcPort,pcWelcomePortMessage,strlen(pcWelcomePortMessage),10);

	for(;;){

		/* Check if a new character has been received.
		 * Reading only one byte at a time using CLI Flags. */
		if(cliDataInputFlag == 1){
			cRxedChar =cliData;
			cliData =0;

			/* Clear the flag to allow new input */
			cliDataInputFlag =0;

			/* Echo the received character back to the terminal */
			writePxITMutex(pcPort,&cRxedChar,1,10);

			switch(cRxedChar){
				/* If the Enter key is pressed, process the command */
				case '\r': /* Enter key */
					writePxITMutex(pcPort,pcNewLine,strlen(pcNewLine),10);

					/* Repeat last command if input is empty */
					if(cInputIndex == 0){
						strcpy((char* )cInputString,(char* )cLastInputString);
					}

					/* Parse the user input and execute the command */
					CLI_CommandParser(pcPort,true,cInputString,pcOutputString);

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
	char idString[MAX_LENGTH_OF_ALIAS] ={0};
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
					memset(BroadcastResponse,0x00,sizeof(BroadcastResponse));
					strncpy((char* )MessageParams,loc + 1,(size_t )(strlen((char* )cInputString) - strlen((char* )idString) - 1));
					BroadcastMessage(myID,BOS_BROADCAST,CODE_CLI_COMMAND,strlen((char* )cInputString) - strlen((char* )idString));		// Send terminating zero
					/* Execute locally */
					xReturned =FreeRTOS_CLIProcessCommand((const signed char* )(loc + 1),pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
					strcat((char* )pcOutputString,"Command broadcasted to all\n\r");
				}

				/* Handle multicast */
				else if((uint8_t )id == BOS_MULTICAST){
					group =id >> 8;
					memset(BroadcastResponse,0x00,sizeof(BroadcastResponse));
					strncpy((char* )MessageParams,loc + 1,(size_t )(strlen((char* )cInputString) - strlen((char* )idString) - 1));
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
						RemoteBootloaderUpdate(myID,id,pcPort,0);
					}
					else{
						/* Forward the command */
						strncpy((char* )MessageParams,loc + 1,(size_t )(strlen((char* )cInputString) - strlen((char* )idString) - 1));
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
						if(ResponseStatus != BOS_OK){
							++TimedoutMsg;
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
	
	memset(idString,0x00,MAX_LENGTH_OF_ALIAS);
	
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
	Snippet_t *currentSnippet =&Snippets[NumOfRecordedSnippets - 1];

	/* Check for codes */
	switch(code){
		case SNIPPET_ACTIVATE:
			/* Activate the last recorded snippet */
			currentSnippet->State = true;
			/* Save snippet state to read-only memory */
			SaveSnippetsToRO();
			break;
			
			/* Parse the condition string for the snippet */
		case SNIPPET_CONDITION:
			return ParseSnippetCondition(string);
			
			/* Handle adding commands to the snippet */
		case SNIPPET_COMMANDS:
			/* Check if a command buffer already exists */
			if(currentSnippet->CMD != NULL){
				/* Reallocate memory to accommodate the new command */
				currentLength =strlen(currentSnippet->CMD);

				/* Use a temporary pointer to avoid memory leaks in case of allocation failure */
				/* Add two more bytes for the ENTER key (0x13) and end of string (0x00) */
				char *temp =(char* )realloc(currentSnippet->CMD,currentLength + strlen(string) + 2);

				if(temp == NULL){
					return BOS_ERR_SNIP_MEM_FULL;  /* Memory allocation failed */
				}

				currentSnippet->CMD =temp;

				/* Append the new command */
				*(currentSnippet->CMD + currentLength) =0x13;  /* ENTER key separator (0x13) */
				strcpy(currentSnippet->CMD + currentLength + 1,string);
			}
			else{
				/* Allocate a new buffer for the command */
				currentSnippet->CMD =(char* )malloc(strlen(string) + 1);

				if(currentSnippet->CMD == NULL){
					memset(currentSnippet,0,sizeof(Snippet_t));  /* Reset snippet structure */
					return BOS_ERR_SNIP_MEM_FULL;  /* Memory allocation failed */
				}

				/* Copy the command into the allocated buffer */
				strcpy(currentSnippet->CMD,string);
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
	char *firstPart = NULL;
	char *secondPart = NULL;
	char *thirdPart = NULL;
	uint8_t port =0;
	uint8_t modPar1 =0;
	uint8_t modPar2 =0;
	static int8_t cInputString[cmdMAX_INPUT_SIZE];

	/* Ensure there is available memory for storing Snippets */
	if(NumOfRecordedSnippets >= MAX_SNIPPETS){
		return BOS_ERR_SNIP_MEM_FULL;
	}

	/* Initialize snippet structure */
	Snippet_t *currentSnippet =&Snippets[NumOfRecordedSnippets];
	memset(&currentSnippet->Condition,0,sizeof(SnippetConditions_t));
	
	/******************* CONDITION TYPE #1: BUTTON EVENT *******************/
	/* Check if the condition starts with "bx." (Button event) */
	if(string[0] == 'b' && string[2] == '.'){
		if(string[1] >= '0' && (string[1] - '0') <= NUM_OF_PORTS){
			/* Extract the button port */
			port =string[1] - '0';
			currentSnippet->Condition.ConditionType = SNIP_COND_BUTTON_EVENT;
			currentSnippet->Condition.Buffer1[0] =port; /* Store button port number */

			/* Store button event type */
			if(!strncmp(&string[3],"clicked",7)){
				currentSnippet->Condition.Buffer1[1] =CLICKED;
				if(!(Button[port].Event & BUTTON_EVENT_CLICKED)){
					SetButtonEvents(port,CLICKED,BUTTON_EVENT_MODE_OR);
				}
			}
			else if(!strncmp(&string[3],"double clicked",14)){
				currentSnippet->Condition.Buffer1[1] =DBL_CLICKED;
				if(!(Button[port].Event & BUTTON_EVENT_DBL_CLICKED)){
					SetButtonEvents(port,DBL_CLICKED,BUTTON_EVENT_MODE_OR);
				}
			}

			else if(!strncmp(&string[3],"released",8)){
				currentSnippet->Condition.Buffer1[1] =RELEASED;
				if(!(Button[port].Event & BUTTON_EVENT_RELEASED)){
					SetButtonEvents(port,RELEASED,BUTTON_EVENT_MODE_OR);
				}
			}
			else{
				return BOS_ERR_WrongParam;
			}
			
			/* Record snippet */

			NumOfRecordedSnippets++;

			return BOS_OK;
		}
	}
	/************** CONDITION TYPE #2 & #3 & #4: Module-related ************/

	/* Copy string into a local buffer for tokenization */
	strncpy((char* )cInputString,string,cmdMAX_INPUT_SIZE - 1);
	cInputString[cmdMAX_INPUT_SIZE - 1] ='\0'; /* Ensure null termination */

	/* Tokenize the condition into three parts */
	firstPart =strtok((char* )cInputString," ");
	secondPart =strtok( NULL," ");
	thirdPart =strtok( NULL," ");

	/* Check if the first part is a valid module parameter or event */
	if(firstPart == NULL){
		return BOS_ERR_WrongParam;
	}

	modPar1 =IsModuleParameter(firstPart);

	/******************* CONDITION TYPE #2: MODULE EVENT *******************/
	if(modPar1 && secondPart == NULL && thirdPart == NULL){
		currentSnippet->Condition.ConditionType = SNIP_COND_MODULE_EVENT;
		currentSnippet->Condition.Buffer1[1] =modPar1;
		NumOfRecordedSnippets++; /* Record snippet */
		return BOS_OK;
	}

	/********** CONDITION TYPE #3 & #4: MODULE PARAMETER CHECK  ************/
	if(secondPart != NULL && thirdPart != NULL){
		modPar2 =IsModuleParameter(thirdPart);
		
		if(modPar2){
			/* CONDITION TYPE #4: Module parameter compared to another parameter */
			currentSnippet->Condition.ConditionType = SNIP_COND_MODULE_PARAM_PARAM;
			currentSnippet->Condition.Buffer1[1] =modPar1; /* Leaving first buffer byte for remote module ID */
			currentSnippet->Condition.Buffer2[1] =modPar2; /* Leaving first buffer byte for remote module ID */
		}
		else{
			/* CONDITION TYPE #3: Module parameter compared to a constant */
			currentSnippet->Condition.ConditionType = SNIP_COND_MODULE_PARAM_CONST;
			currentSnippet->Condition.Buffer1[1] =modPar1; /* Leaving first buffer byte for remote module ID */

			/* Extract the constant */
			float constant =atof(thirdPart);
			memcpy(currentSnippet->Condition.Buffer2,&constant,sizeof(float));

		}

		/* Validate and store the math operator */
		currentSnippet->Condition.MathOperator =IsMathOperator(secondPart);
		if(!currentSnippet->Condition.MathOperator)
			return BOS_ERR_WrongParam;

		NumOfRecordedSnippets++; /* Record snippet */
		return BOS_OK;
	}
	return BOS_ERR_WrongParam;

}

/***************************************************************************/
/* Check if Snippet conditional is true or false */
bool CheckSnippetCondition(uint8_t index){
	uint8_t temp8 =0;
	float flt1 =0.0f;
	float flt2 =0.0f;
	
	/* Check conditions based on Snippet type */
	switch(Snippets[index].Condition.ConditionType){

		/* Button Event */
		case SNIP_COND_BUTTON_EVENT:
			temp8 =Snippets[index].Condition.Buffer1[0]; /* Get button port */
			/* Check if button state matches Snippet button event */
			if(Snippets[index].Condition.Buffer1[1] == Button[temp8].State)
				return true;
			else
				return false;
			
		/* Module Event */
		case SNIP_COND_MODULE_EVENT:
			// TODO: Implement event checking logic
			break;
			
		/* Module Parameter Compared to Constant */
		case SNIP_COND_MODULE_PARAM_CONST:
			/* Get the module parameter value */
			GetModuleParameter(Snippets[index].Condition.Buffer1[1] , &flt1);
			/* This buffer can be misaligned and cause hardfault */
			memcpy((uint8_t* )&flt2,&Snippets[index].Condition.Buffer2,sizeof(float));

			/* Perform mathematical comparison */
			switch(Snippets[index].Condition.MathOperator){
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
            // TODO: Implement parameter-to-parameter comparison
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
	uint16_t snippetIndex =0;
	int8_t *pcOutputString =NULL;
	static int8_t cInputString[cmdMAX_INPUT_SIZE];
	
	/* Get the output buffer address.
	 * Note: No mutual exclusion is applied because it is assumed that
	 * only one command console interface will be active at a time. */
	pcOutputString =FreeRTOS_CLIGetOutputBuffer();
	
	/* Loop through all recorded Snippets */
	for(snippetIndex =0; snippetIndex < NumOfRecordedSnippets; snippetIndex++){
		/* Process only active Snippets */
		if(Snippets[snippetIndex].State){
			/* Process Snippet condition */
			if(CheckSnippetCondition(snippetIndex)){
				/* Disable CLI response to prevent unnecessary output */
				OptionByte.Response = BOS_RESPONSE_MSG;

				/* Loop over all recorded commands within the snippet */
				while(ParseSnippetCommand(Snippets[snippetIndex].CMD,(int8_t* )&cInputString) != false){
					/* Pass the parsed command to the CLI command parser */
					CLI_CommandParser(pcPort,false,cInputString,pcOutputString);
					
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
	static char *ptrStart = NULL;
	static char *ptrEnd = NULL;

	/* Return false if the snippet command buffer is NULL */
	if(Snippets[NumOfRecordedSnippets - 1].CMD == NULL)
		return false;

	/* Initialize ptrStart if it's the first call */
	if(ptrStart == NULL)
		ptrStart =snippetBuffer;

	/* Check if we reached the end of the snippet buffer */
	if(*ptrStart == '\0'){
		ptrStart = NULL; /* Reset pointer for the next snippet */
		return false;
	}

	/* Search for the first occurrence of the ENTER key (0x13) */
	ptrEnd =strchr(ptrStart,0x13);

	if(ptrEnd != NULL){
		/* Copy the command from ptrStart to cliBuffer, ensuring safe copy */
		strncpy((char* )cliBuffer,ptrStart,ptrEnd - ptrStart);
		cliBuffer[ptrEnd - ptrStart] ='\0'; /* Null-terminate the string */

		/* Move ptrStart to the next command */
		ptrStart =ptrEnd + 1;
		;
	}
	else{
		/* If no ENTER key is found, copy the remaining string */
		strncpy((char* )cliBuffer,ptrStart,cmdMAX_INPUT_SIZE - 1);
		cliBuffer[cmdMAX_INPUT_SIZE - 1] ='\0'; /* Ensure null termination */

		/* Move ptrStart to the end of the buffer */
		ptrStart +=strlen((const char* )cliBuffer);
	}

	return true;
}
/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
