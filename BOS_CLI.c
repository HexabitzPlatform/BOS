/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/
	
/*
		MODIFIED by Hexabitz for BitzOS (BOS) V0.2.2 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
*/

#include "BOS.h"

/* Internal Variables --------------------------------------------------------*/
snippet_t snippets[MAX_SNIPPETS];		// Buffer to hold CLI Snippets
uint8_t numOfRecordedSnippets = 0;


static char * pcWelcomeMessage = 	\
"\n\r\n\r====================================================	\
     \n\r====================================================	\
     \n\r||            Welcome to BitzOS CLI!              ||	\
		 \n\r||       (C) COPYRIGHT HEXABITZ 2017-2020.        ||	\
     \n\r||                                                ||	\
		 \n\r||      Please check the project website at       ||	\
		 \n\r||             http://hexabitz.com/               ||	\
     \n\r||                                                ||	\
     \n\r||   Type help for a list of available commands.  ||	\
     \n\r====================================================	\
     \n\r====================================================	\
     \n\n\r";  
static char * pcNewLine = "\r\n";
static char * pcEndOfCommandOutputString = "\r\n[Press ENTER to execute the previous command again]\r\n>";
char pcWelcomePortMessage[40] = {0};

/* Exported Variables --------------------------------------------------------*/
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
extern uint16_t timedoutMsg;
extern uint8_t UARTRxBufIndex[NumOfPorts];

/* Internal functions ---------------------------------------------------------*/

BOS_Status AddSnippet(uint8_t code, char *string);
BOS_Status ParseSnippetCondition(char *string);
bool ParseSnippetCommand(char *snippetBuffer, int8_t *cliBuffer);
bool CheckSnippetCondition(uint8_t index);
void CLI_CommandParser(uint8_t port, bool enableOutput, int8_t *cInputString, int8_t *pcOutputString);

/* BOS exported internal functions */
extern void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport);
extern uint8_t IsModuleParameter(char* name);
extern uint8_t IsMathOperator(char* string);
extern uint8_t SaveToRO(void);

/*-----------------------------------------------------------*/

void prvCLITask( void *pvParameters )
{
char cRxedChar; int8_t cInputIndex = 0, *pcOutputString; 
static int8_t cInputString[ cmdMAX_INPUT_SIZE ], cLastInputString[ cmdMAX_INPUT_SIZE ];
uint16_t chr = 0;

	( void ) pvParameters;
	
	/* Wait indefinitly until a '\r' is received on one of the ports */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	cRxedChar = '\0';

	/* Note: DMA is not being used on transmit functions because it caused output errors. Maybe due to high baudrate. */
	
	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console
	interface will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/* By default, the UART interrupt priority will have been set to the lowest
	possible.  It must be kept at or below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
	but	can be raised above its default priority using a FreeRTOS_ioctl() call
	with the ioctlSET_INTERRUPT_PRIORITY command. */
	//xReturned = FreeRTOS_ioctl( xConsoleUART, ioctlSET_INTERRUPT_PRIORITY, ( void * ) ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY - 1 ) );
	//configASSERT( xReturned );

	/* Set baudrate back to default for all other ports */
	if (BOS.clibaudrate != DEF_ARRAY_BAUDRATE)
	{
		for (uint8_t port=1 ; port<=NumOfPorts ; port++) 
		{	
			if (port != PcPort)
				UpdateBaudrate(port, DEF_ARRAY_BAUDRATE);
		}
	}
	
	/* Send the welcome message. */
	sprintf(pcWelcomePortMessage, "Connected to module %d (%s), port P%d.\n\n\r>", myID, modulePNstring[myPN], PcPort);
	writePxITMutex(PcPort, pcWelcomeMessage, strlen(pcWelcomeMessage), 10);
	writePxITMutex(PcPort, pcWelcomePortMessage, strlen(pcWelcomePortMessage), 10);

	
	for( ;; )
	{
		/* Only interested in reading one character at a time from the circular buffer. Start looping from last character. */
		for (chr=UARTRxBufIndex[PcPort-1] ; chr<MSG_RX_BUF_SIZE ; chr++)
		{
			if (UARTRxBuf[PcPort-1][chr]) {
				cRxedChar = UARTRxBuf[PcPort-1][chr];
				UARTRxBuf[PcPort-1][chr] = 0;
				UARTRxBufIndex[PcPort-1] = chr;
				break;
			}
			if (chr == MSG_RX_BUF_SIZE-1)	{
				chr = UARTRxBufIndex[PcPort-1] = 0;
			}			
			taskYIELD();
		}
			
		/* Echo the character back. */
		writePxITMutex(PcPort, &cRxedChar, 1, 10);
		
		if( cRxedChar == '\r' )
		{
			/* The input command string is complete. Ensure the previous
			UART transmission has finished before sending any more data.
			This task will be held in the Blocked state while the Tx completes,
			if it has not already done so, so no CPU time will be wasted by
			polling. */
			writePxITMutex(PcPort, pcNewLine, strlen(pcNewLine), 10);
			
			
			/* See if the command is empty, indicating that the last command is
			to be executed again. */
			if( cInputIndex == 0 )
			{
				strcpy( ( char * ) cInputString, ( char * ) cLastInputString );
			}

			/* Pass the received command to the command interpreter.  The
			command interpreter is called repeatedly until it returns
			pdFALSE as it might generate more than one string. */
			CLI_CommandParser(PcPort, true, cInputString, pcOutputString);
			
			/* All the strings generated by the input command have been sent.
			Clear the input	string ready to receive the next command.  Remember
			the command that was just processed first in case it is to be
			processed again. */
			strcpy( ( char * ) cLastInputString, ( char * ) cInputString );
			cInputIndex = 0;
			memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );

		}
		else
		{
			if( cRxedChar == '\n' )
			{
				/* Ignore the character. */
			}
			else if( cRxedChar == '\b' )
			{
				/* Backspace was pressed.  Erase the last character in the
				string - if any. */
				if( cInputIndex > 0 )
				{
					cInputIndex--;
					cInputString[ cInputIndex ] = '\0';
				}
			}
			else
			{
				/* A character was entered.  Add it to the string
				entered so far.  When a \r is entered the complete
				string will be passed to the command interpreter. */
				if( ( cRxedChar >= ' ' ) && ( cRxedChar <= '~' ) )
				{
					if( cInputIndex < cmdMAX_INPUT_SIZE )
					{
						cInputString[ cInputIndex ] = cRxedChar;
						cInputIndex++;
					}
				}
			}
		}   
    
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

/* Hexabitz CLI command parser
*/
void CLI_CommandParser(uint8_t port, bool enableOutput, int8_t *cInputString, int8_t *pcOutputString)
{
	static uint8_t recordSnippet, group; portBASE_TYPE xReturned; 
	char* loc = 0; int16_t id = 0; char idString[MaxLengthOfAlias] = {0};
	
	/* Pass the received command to the command interpreter.  The
	command interpreter is called repeatedly until it returns
	pdFALSE as it might generate more than one string. */
	do
	{
		/* Once again, just check to ensure the UART has completed
		sending whatever it was sending last.  This task will be held
		in the Blocked state while the Tx completes, if it has not
		already done so, so no CPU time	is wasted polling. */
		
		/* Convert input string to lower case */
		StringToLowerCase(( char * )cInputString);
						
		/* Check for a conditional statement (if) */
		if (!recordSnippet && !strncmp((char *)cInputString, "if ", 3)) 
		{
			/* Add the condition to Command Snippets (after removing "if " */
			if (AddSnippet(SNIPPET_CONDITION, ( char * ) (cInputString+3)) != BOS_OK) {
				sprintf( ( char * ) pcOutputString, "\nCannot store more Command Snippets. Please delete existing ones and try again.\n\r");
				recordSnippet = 0;
			} else {
				/* Start recording Commands after the condition */
				recordSnippet = SNIPPET_COMMANDS;
				pcOutputString[0] = '\r';
			}
			xReturned = pdFALSE;
		} 
		/* Check for the end of a conditional command (end if) */
		else if (recordSnippet && !strncmp((char *)cInputString, "end if", 6))
		{
			/* Stop recording Commands for the conditional Command Snippet */
			recordSnippet = 0;
			/* Activate the Snippet */
			AddSnippet(SNIPPET_ACTIVATE, "");				
			/* If snippet saved successfuly */
			sprintf( ( char * ) pcOutputString, "\nConditional statement accepted and added to Command Snippets.\n\r");
			xReturned = pdFALSE;
		}
		/* Should I record any Command Snippets? */
		else if (recordSnippet == SNIPPET_COMMANDS)
		{
			/* Add this Command to Command Snippets */
			if (AddSnippet(SNIPPET_COMMANDS, ( char * ) cInputString) != BOS_OK)
				sprintf( ( char * ) pcOutputString, "\nCannot store more Command Snippets. Please delete existing ones and try again.\n\r");
			else
				pcOutputString[0] = '\r';
			xReturned = pdFALSE;
		}
		/* Parse a normal Command */
		else 
		{
			/* Check if command contains a dot and it's not "BOS." or a decimal number */
			loc = strchr( ( char * ) cInputString, '.');
			if ( loc != NULL && strncmp((char *)loc-3, "bos", 3) && !isdigit(*(loc+1)) ) 
			{					
				/* Extract module ID/alias or group alias */
				strncpy(idString, ( char * ) cInputString, (size_t) (loc - (char*)cInputString));
				id = GetID(idString);
				if (id == myID) {
					/* Extract and process the command */
					xReturned = FreeRTOS_CLIProcessCommand( (const signed char*)(loc+1), pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );		
				}	else if (id == BOS_ERR_WrongName) {		
					sprintf( ( char * ) pcOutputString, "Wrong module name! Please try again.\n\r");
					xReturned = pdFALSE;
				}	else if (id == BOS_ERR_WrongID) {
					sprintf( ( char * ) pcOutputString, "Wrong module ID! Please try again.\n\r");
					xReturned = pdFALSE;						
				}	else if (id == BOS_BROADCAST) {
					/* Check if command is broadcastable */									

					/* Broadcast the command */								
					memset( broadcastResponse, 0x00, sizeof(broadcastResponse) );
					strncpy( ( char * ) messageParams, loc+1, (size_t)(strlen( (char*) cInputString)-strlen( (char*) idString)-1));
							BroadcastMessage(myID, BOS_BROADCAST, CODE_CLI_COMMAND, strlen( (char*) cInputString)-strlen( (char*) idString));		// Send terminating zero
					/* Execute locally */
					xReturned = FreeRTOS_CLIProcessCommand( (const signed char*)(loc+1), pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );	
					strcat( ( char * ) pcOutputString, "Command broadcasted to all\n\r");
					/* Todo: check module response if needed */
					//sprintf( ( char * ) pcOutputString, "Module %d is not reachable.\n\r", m);	
				}	else if ((uint8_t)id == BOS_MULTICAST) {	
					group = id >> 8;
							/* Todo: Check if command is broadcastable */									

					/* Multicast the command */								
					memset( broadcastResponse, 0x00, sizeof(broadcastResponse) );
					strncpy( ( char * ) messageParams, loc+1, (size_t)(strlen( (char*) cInputString)-strlen( (char*) idString)-1));
							BroadcastMessage(myID, group, CODE_CLI_COMMAND, strlen( (char*) cInputString)-strlen( (char*) idString));		// Send terminating zero
					/* Do I need to execute locally? */
					if (InGroup(myID, group))
						xReturned = FreeRTOS_CLIProcessCommand( (const signed char*)(loc+1), pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );	
					sprintf( ( char * ) pcOutputString, "%sMulticast Command forwarded to group %s\n\r", pcOutputString, idString);
				}	
				else 
				{
					/* Special commands that convert into custom a Message */
					if (!strncmp((char *)loc+1, "update", 6)) {			// remote update
						BOS.response = BOS_RESPONSE_NONE;				
								SendMessageToModule(id, CODE_UPDATE, 0);
						osDelay(100);
						/* Execute locally */
						remoteBootloaderUpdate(myID, id, PcPort, 0);
					} 
					else 
					{						
						/* Forward the command */
						strncpy( ( char * ) messageParams, loc+1, (size_t)(strlen((char*) cInputString)-strlen((char*) idString)-1));
								SendMessageToModule(id, CODE_CLI_COMMAND, strlen((char*) cInputString)-strlen((char*) idString)-1);
						sprintf( ( char * ) pcOutputString, "Command forwarded to Module %d\n\r", id);

						if ((strlen((char*)pcOutputString) > 0) && enableOutput)
							writePxMutex(port, (char*)pcOutputString, strlen((char*)pcOutputString), cmd50ms, 1);		
						memset( pcOutputString, 0x00, strlen((char*)pcOutputString) );
					}
					
					/* Wait for response if needed */
					if (BOS.response == BOS_RESPONSE_ALL)
					{
						ulTaskNotifyTake(pdTRUE, 1000);		//cmd500ms
						/* If timeout */
								if (responseStatus != BOS_OK) {
									++timedoutMsg;
									sprintf( ( char * ) pcOutputString, "%sModule %d is not reachable.\n\r", ( char * ) pcOutputString, id);
					}	
							}	
					xReturned = pdFALSE;
				}						
			} 
			else 
			{
				/* Process the command locally */
				xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );		
			}
		}
		
		/* Write the generated string to the UART. */
		if (strlen((char*)pcOutputString) > 0 && enableOutput)
			writePxMutex(port, (char*)pcOutputString, strlen((char*)pcOutputString), cmd50ms, HAL_MAX_DELAY);		
		memset( pcOutputString, 0x00, strlen((char*)pcOutputString) );

	} while( xReturned != pdFALSE );

	memset( idString, 0x00, MaxLengthOfAlias );
	
	/* Start to transmit a line separator, just to make the output easier to read. */
	if(!recordSnippet && enableOutput)
		writePxMutex(port, pcEndOfCommandOutputString, strlen(pcEndOfCommandOutputString), cmd50ms, HAL_MAX_DELAY);		

}

/*-----------------------------------------------------------*/

/* Convert a string to lower case
*/
void StringToLowerCase(char *string)
{
	for(int i = 0; string[i]; i++){
		string[i] = tolower(string[i]);
	}
}

/*-----------------------------------------------------------*/

/* Add a set of Commands to Command Snippets and activate
*/
BOS_Status AddSnippet(uint8_t code, char *string)
{
	/* Check for codes */
	switch (code)
	{
		case SNIPPET_ACTIVATE :
			snippets[numOfRecordedSnippets-1].state = true;
			SaveToRO();
			break;
		
		case SNIPPET_CONDITION :
			return ParseSnippetCondition(string);
		
		case SNIPPET_COMMANDS :
			// Did we allocate a buffer already?
			if (snippets[numOfRecordedSnippets-1].cmd != NULL) 
			{
				// re-allocate with new size
				int currentLenght = strlen(snippets[numOfRecordedSnippets-1].cmd);
				// Add two more bytes for the ENTER key (0x13) and end of string (0x00)
				snippets[numOfRecordedSnippets-1].cmd = (char *) realloc(snippets[numOfRecordedSnippets-1].cmd, currentLenght+strlen(string)+2);
				// Copy the command
				strcpy(snippets[numOfRecordedSnippets-1].cmd + currentLenght + 1, string);
				*(snippets[numOfRecordedSnippets-1].cmd + currentLenght) = 0x13;		// ENTER key between commands
			} 
			// Allocate a new buffer
			else
			{
				// Allocate memory buffer
				snippets[numOfRecordedSnippets-1].cmd = (char *) malloc(strlen(string)+1);
				// Copy the command
				strcpy(snippets[numOfRecordedSnippets-1].cmd, string);
			}
			// Return error if allocation fails
			if (snippets[numOfRecordedSnippets-1].cmd == NULL) {
				memset(&snippets[numOfRecordedSnippets-1], 0, sizeof(snippet_t) );
				return BOS_ERR_SNIP_MEM_FULL;
			}
			
			break;
		
		default:
			break;
	}	
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* Parse Snippet conditions into the internal buffer
*/
BOS_Status ParseSnippetCondition(char *string)
{
	static int8_t cInputString[ cmdMAX_INPUT_SIZE ];
	BOS_Status status = BOS_OK;
	uint8_t port = 0;
	
	// A. Verify first there's still memory left to store Snippets	
	if (numOfRecordedSnippets == MAX_SNIPPETS)
	{	
		return BOS_ERR_SNIP_MEM_FULL;
	}
	// Initialize the next empty location
	else
	{
		snippets[numOfRecordedSnippets].cond.conditionType = 0;
		snippets[numOfRecordedSnippets].cond.mathOperator = 0;			
		memset(snippets[numOfRecordedSnippets].cond.buffer1, 0, 4);			
	}
	
	// B. Parse Snippets based on their condition type 
	
	// #1: Button event: condition starts with "bx." 
	if(string[0] == 'b' && string[2] == '.')
	{
		if(string[1] >= '0' && string[1] <= (NumOfPorts+'0'))		// Valid port number
		{
			port = string[1]-'0';
			snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_BUTTON_EVENT;
			snippets[numOfRecordedSnippets].cond.mathOperator = 0;			// No math operations
			snippets[numOfRecordedSnippets].cond.buffer1[0] = port;		// Store button port number	
			
			/* Store button event and event parameter if needed */
			if (!strncmp((char *)&string[3], "clicked", 7))
			{
				snippets[numOfRecordedSnippets].cond.buffer1[1] = CLICKED;	
				if ((button[port].events & BUTTON_EVENT_CLICKED) != BUTTON_EVENT_CLICKED)		// Enable the event
					SetButtonEvents(port, 1, 0, 0, 0, 0, 0, 0, 0, BUTTON_EVENT_MODE_OR);
				status = BOS_OK;
			}
			else if (!strncmp((char *)&string[3], "double clicked", 14))
			{
				snippets[numOfRecordedSnippets].cond.buffer1[1] = DBL_CLICKED;			
				if ((button[port].events & BUTTON_EVENT_DBL_CLICKED) != BUTTON_EVENT_DBL_CLICKED)
					SetButtonEvents(port, 0, 1, 0, 0, 0, 0, 0, 0, BUTTON_EVENT_MODE_OR);
				status = BOS_OK;					
			}
			else if (!strncmp((char *)&string[3], "pressed for ", 12))
			{
				if (!button[port].pressedX1Sec) {	
					snippets[numOfRecordedSnippets].cond.buffer1[1] = PRESSED_FOR_X1_SEC;	
					snippets[numOfRecordedSnippets].cond.buffer1[2] = atoi((char *)&string[15]);
					SetButtonEvents(port, 0, 0, snippets[numOfRecordedSnippets].cond.buffer1[2], 0, 0, 0, 0, 0, BUTTON_EVENT_MODE_OR);
					status = BOS_OK;
				} else if (!button[port].pressedX2Sec) {	
					snippets[numOfRecordedSnippets].cond.buffer1[1] = PRESSED_FOR_X2_SEC;	
					snippets[numOfRecordedSnippets].cond.buffer1[2] = atoi((char *)&string[15]);
					SetButtonEvents(port, 0, 0, 0, snippets[numOfRecordedSnippets].cond.buffer1[2], 0, 0, 0, 0, BUTTON_EVENT_MODE_OR);
					status = BOS_OK;		
				} else if (!button[port].pressedX3Sec) {	
					snippets[numOfRecordedSnippets].cond.buffer1[1] = PRESSED_FOR_X3_SEC;	
					snippets[numOfRecordedSnippets].cond.buffer1[2] = atoi((char *)&string[15]);
					SetButtonEvents(port, 0, 0, 0, 0, snippets[numOfRecordedSnippets].cond.buffer1[2], 0, 0, 0, BUTTON_EVENT_MODE_OR);
					status = BOS_OK;	
				} else {
					status = BOS_ERR_BUTTON_PRESS_EVENT_FULL;
				}							
			}
			else if (!strncmp((char *)&string[3], "released for ", 13))
			{
				if (!button[port].releasedY1Sec) {	
					snippets[numOfRecordedSnippets].cond.buffer1[1] = RELEASED_FOR_Y1_SEC;	
					snippets[numOfRecordedSnippets].cond.buffer1[2] = atoi((char *)&string[16]);
					SetButtonEvents(port, 0, 0, 0, 0, 0, snippets[numOfRecordedSnippets].cond.buffer1[2], 0, 0, BUTTON_EVENT_MODE_OR);
					status = BOS_OK;
				} else if (!button[port].releasedY2Sec) {	
					snippets[numOfRecordedSnippets].cond.buffer1[1] = RELEASED_FOR_Y2_SEC;	
					snippets[numOfRecordedSnippets].cond.buffer1[2] = atoi((char *)&string[16]);
					SetButtonEvents(port, 0, 0, 0, 0, 0, 0, snippets[numOfRecordedSnippets].cond.buffer1[2], 0, BUTTON_EVENT_MODE_OR);
					status = BOS_OK;		
				} else if (!button[port].releasedY3Sec) {	
					snippets[numOfRecordedSnippets].cond.buffer1[1] = RELEASED_FOR_Y3_SEC;	
					snippets[numOfRecordedSnippets].cond.buffer1[2] = atoi((char *)&string[16]);
					SetButtonEvents(port, 0, 0, 0, 0, 0, 0, 0, snippets[numOfRecordedSnippets].cond.buffer1[2], BUTTON_EVENT_MODE_OR);					
					status = BOS_OK;	
				} else {
					status = BOS_ERR_BUTTON_RELEASE_EVENT_FULL;
				}									
			}				
			
			++numOfRecordedSnippets;		// Record a successful Snippet			
		}
	}
	// Module-related conditions (local only for now)
	else
	{
		strcpy( (char *)cInputString, string);

		// This is probably a three part condition, extract them out
		char *firstPart, *secondPart, *thirdPart; uint8_t modPar1 = 0, modPar2 = 0;
		firstPart = strtok ( (char *)cInputString, " ");
		secondPart = strtok ( NULL, " ");
		thirdPart = strtok ( NULL, " ");
		
		// Check if first part is module parameter or event
		if (firstPart == NULL) 
		{
			return BOS_ERR_WrongParam;
		} 
		else 
		{
			modPar1 = IsModuleParameter(firstPart);
			// Found a module parameter and no more strings
			if (modPar1 && secondPart == NULL && thirdPart == NULL) 
			{
			// #2: Module event	
				snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_MODULE_EVENT;			
				snippets[numOfRecordedSnippets].cond.buffer1[1] = modPar1;		// Leaving first buffer byte for remote module ID
				
				++numOfRecordedSnippets;		// Record a successful Snippet	
				return BOS_OK;
			} 
			else if (secondPart != NULL && thirdPart != NULL) 
			{
				modPar2 = IsModuleParameter(thirdPart);
				if (modPar2) 		// Found a module parameter
				{
					// #4: Module parameter and parameter
					snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_MODULE_PARAM_PARAM;
					snippets[numOfRecordedSnippets].cond.buffer1[1] = modPar1;		// Leaving first buffer byte for remote module ID
					snippets[numOfRecordedSnippets].cond.buffer2[1] = modPar2;		// Leaving first buffer byte for remote module ID				
				} 
				else 
				{
					// #3: Module parameter and constant	
					snippets[numOfRecordedSnippets].cond.conditionType = SNIP_COND_MODULE_PARAM_CONST;
					snippets[numOfRecordedSnippets].cond.buffer1[1] = modPar1;		// Leaving first buffer byte for remote module ID
					// Extract the constant
					float constant = atof(thirdPart);
					memcpy(snippets[numOfRecordedSnippets].cond.buffer2, &constant, sizeof(float));		// This buffer can be misaligned and cause hardfault on F0
				}				
				// Extract the math operator
				snippets[numOfRecordedSnippets].cond.mathOperator = IsMathOperator(secondPart);
				if (!snippets[numOfRecordedSnippets].cond.mathOperator)
					return BOS_ERR_WrongParam;
				
				++numOfRecordedSnippets;		// Record a successful Snippet
				return BOS_OK;				
			} 
			else 
			{
				return BOS_ERR_WrongParam;
			}				
		}
	}
	
	// Note: after exiting this function, numOfRecordedSnippets refers to the next empty Snippet. Substract by one to reference the last Snippet.
	
	return status;
}

/*-----------------------------------------------------------*/

/* Parse Snippet commands into the internal buffer
*/
bool ParseSnippetCommand(char *snippetBuffer, int8_t *cliBuffer)
{
	static char *ptrStart, *ptrEnd;
	
	if (snippets[numOfRecordedSnippets-1].cmd == NULL)	return false;
	
	// Initialize the start pointer to snippet buffer address
	if (!ptrStart)	ptrStart = snippetBuffer;	
	
	// Did we already reach end of Snippet buffer?
	if (*ptrStart == 0x00) {
		ptrStart = 0;		// Initialize the start pointer for next Snippet
		cliBuffer = NULL;
		return false;
	}
	
	// Search the buffer for first occurance of 0x13 (ENTER key)
	ptrEnd = strchr(ptrStart,0x13);
	if (ptrEnd != NULL)
	{
		strncpy((char *)cliBuffer, ptrStart, ptrEnd-ptrStart);
		ptrStart = ptrEnd+1;
	}
	else
	{
		strcpy((char *)cliBuffer, ptrStart);
		ptrStart += strlen((const char *) cliBuffer);
	}

	return true;
}

/*-----------------------------------------------------------*/

/* Check if Snippet conditional is true or false
*/
bool CheckSnippetCondition(uint8_t index)
{
	uint8_t temp8;
	float flt1, flt2;
	
	/* Check conditions based on Snippet tupe */	

	switch (snippets[index].cond.conditionType)
  {
  	case SNIP_COND_BUTTON_EVENT :				
  		temp8 = snippets[index].cond.buffer1[0]; 	// Button port
			/* Check if button state matches Snippet button event */
			if (snippets[index].cond.buffer1[1] == button[temp8].state)
				return true;
			else 
				return false;			
			
		case SNIP_COND_MODULE_EVENT :	
			break;
			
						
		case SNIP_COND_MODULE_PARAM_CONST :	
			// Get the constant and module parameter values. 
			flt1 = *(float *)modParam[snippets[index].cond.buffer1[1]-1].paramPtr;
			memcpy( (uint8_t *)&flt2, &snippets[index].cond.buffer2, sizeof(float));		// This buffer can be misaligned and cause hardfault on F0
			// Compare them mathematically
			switch (snippets[index].cond.mathOperator)
      {
      	case MATH_EQUAL:					if (flt1 == flt2)	return true;	break;
      	case MATH_GREATER:				if (flt1 > flt2)	return true;	break;
		    case MATH_SMALLER:				if (flt1 < flt2 && flt1 != 0.0f)	return true;	break;
			  case MATH_GREATER_EQUAL:	if (flt1 >= flt2)	return true;	break;
			  case MATH_SMALLER_EQUAL:	if (flt1 <= flt2 && flt1 != 0.0f)	return true;	break;
				case MATH_NOT_EQUAL:			if (flt1 != flt2 && flt1 != 0.0f)	return true;	break;
      	default:
      		break;
      }
			break;
			
		case SNIP_COND_MODULE_PARAM_PARAM :	
			break;
					
  	default:
  		break;
  }
	
	return false;
}

/*-----------------------------------------------------------*/

/* Execute activated Command Snippets
*/
BOS_Status ExecuteSnippet(void)
{
	BOS_Status result = BOS_OK;
	uint16_t s = 0;
	int8_t *pcOutputString;
	static int8_t cInputString[ cmdMAX_INPUT_SIZE ];
	
	/* Must get this address even if output is not used otherwise memory will corrupt */
	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console
	interface will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();	
	
	/* Go through activated Snippets */
	for(s=0 ; s<numOfRecordedSnippets ; s++)
  {
		if (snippets[s].state)								// Check for activated Snippets
		{	
			if (CheckSnippetCondition(s))				// Process Snippet condition 				
			{				
				BOS.response = BOS_RESPONSE_MSG;		// Disable CLI response
				// Loop over all recorded Snippet commands
				while (ParseSnippetCommand(snippets[s].cmd, (int8_t *) &cInputString) != false)
				{			
					/* Pass the received command to the command interpreter.  The
					command interpreter is called repeatedly until it returns
					pdFALSE as it might generate more than one string. */
					CLI_CommandParser(PcPort, false, cInputString, pcOutputString);
					
					/* Clear output buffer since we do not need it. Input buffer is cleared in  CLI_CommandParser */
					memset( pcOutputString, 0x00, strlen((char*) pcOutputString) );
				}
			}
		}
  }
	
	return result;
}

/*-----------------------------------------------------------*/
