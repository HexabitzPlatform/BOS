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
		MODIFIED by Hexabitz for BitzOS (BOS) V0.1.0 - Copyright (C) 2017 Hexabitz
    All rights reserved
*/

#include "BOS.h"






/* Internal Variables --------------------------------------------------------*/

uint8_t snippets[SNIPPETS_BUF_SIZE] = {0};			// Buffer to hold Command Snippets
uint16_t currentSnipSize = 0;

static char * pcWelcomeMessage = 	\
"\n\r\n\r====================================================	\
     \n\r====================================================	\
     \n\r||            Welcome to BitzOS CLI!              ||	\
		 \n\r||         (C) COPYRIGHT HEXABITZ 2016.           ||	\
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

/* Internal functions ---------------------------------------------------------*/

BOS_Status AddSnippet(uint8_t type, char *string);
BOS_Status ProcessSnippet(uint16_t location);
BOS_Status ActivateButtonSnippet(uint16_t location);


/*-----------------------------------------------------------*/

void prvUARTCommandConsoleTask( void *pvParameters )
{
char cRxedChar; int8_t cInputIndex = 0, *pcOutputString; uint8_t port;
static int8_t cInputString[ cmdMAX_INPUT_SIZE ], cLastInputString[ cmdMAX_INPUT_SIZE ];
char* loc = 0; uint8_t id = 0; char idString[MaxLengthOfAlias] = {0};
portBASE_TYPE xReturned; uint8_t recordSnippet = 0;

	( void ) pvParameters;
	
	/* Wait indefinitly until a '\r' is received on one of the ports */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	port = PcPort;
	cRxedChar = '\0';
	
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

	/* Send the welcome message. */
	sprintf(pcWelcomePortMessage, "Connected to module %d, port P%d.\n\n\r>", myID, port);
	writePxITMutex(port, pcWelcomeMessage, strlen(pcWelcomeMessage), cmd50ms);
	writePxITMutex(port, pcWelcomePortMessage, strlen(pcWelcomePortMessage), cmd50ms);
	
	
	for( ;; )
	{
		/* Only interested in reading one character at a time. */
		readPxMutex(port, &cRxedChar, sizeof( cRxedChar ), cmd50ms, HAL_MAX_DELAY);
			
		/* Echo the character back. */
		writePxITMutex(port, &cRxedChar, 1, cmd50ms);
		
		if( cRxedChar == '\r' )
		{
			/* The input command string is complete. Ensure the previous
			UART transmission has finished before sending any more data.
			This task will be held in the Blocked state while the Tx completes,
			if it has not already done so, so no CPU time will be wasted by
			polling. */
			writePxITMutex(port, pcNewLine, strlen(pcNewLine), cmd50ms);
			
			
			/* See if the command is empty, indicating that the last command is
			to be executed again. */
			if( cInputIndex == 0 )
			{
				strcpy( ( char * ) cInputString, ( char * ) cLastInputString );
			}

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
					AddSnippet(SNIPPET_CONDITION, ( char * ) (cInputString+3));
					/* Start recording Commands after the condition */
					recordSnippet = SNIPPET_CONDITION_CMDS;
					pcOutputString[0] = '\r';
				} 
				/* Check for the end of a conditional command (end if) */
				else if (recordSnippet && !strncmp((char *)cInputString, "end if", 6))
				{
					/* Stop recording Commands for the conditional Command Snippet */
					recordSnippet = 0;
					/* Check memory */
					
					
					/* Activate the Snippet via the LSB */
					AddSnippet((SNIPPET_CONDITION|0x01), "");				
					/* If snippet saved successfuly */
					sprintf( ( char * ) pcOutputString, "\nConditional statement accepted and added to Command Snippets.\n\r");
				}
				/* Should I record any Command Snippets? */
				else if (recordSnippet == SNIPPET_CONDITION_CMDS)
				{
					/* Add this Command to Command Snippets */
					AddSnippet(SNIPPET_CONDITION_CMDS, ( char * ) cInputString);
					pcOutputString[0] = '\r';
				}
				/* Parse a normal Command */
				else 
				{
					/* Check if command contains a dot and it's not "BOS." */
					loc = strchr( ( char * ) cInputString, '.');
					if (loc != NULL && strncmp((char *)loc-3, "bos", 3)) 
					{					
						/* Extract module ID */
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
							BroadcastMessage(0, myID, CODE_CLI_command, strlen( (char*) cInputString)-strlen( (char*) idString)-1);
							/* Execute locally */
							xReturned = FreeRTOS_CLIProcessCommand( (const signed char*)(loc+1), pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );						
							/* Todo: check module response if needed */
							//sprintf( ( char * ) pcOutputString, "Module %d is not reachable.\n\r", m);													
						}	
						else 
						{
							/* Forward the command */
							strncpy( ( char * ) messageParams, loc+1, (size_t)(strlen( (char*) cInputString)-strlen( (char*) idString)-1));
							SendMessageToModule(id, CODE_CLI_command, strlen( (char*) cInputString)-strlen( (char*) idString)-1);
							xReturned = pdFALSE;
							/* Wait for response if needed */
							if (BOS.response == BOS_RESPONSE_ALL)
							{
								ulTaskNotifyTake(pdTRUE, 1000);		//cmd500ms
								/* If timeout */
								if (responseStatus != BOS_OK)
									sprintf( ( char * ) pcOutputString, "Module %d is not reachable.\n\r", id);
							}
						}						
					} 
					else 
					{
						/* Process the command locally */
						xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );		
					}
				}
				
				/* Write the generated string to the UART. */
				writePxMutex(port, (char*) pcOutputString, strlen((char*) pcOutputString), cmd50ms, HAL_MAX_DELAY);		
				memset( pcOutputString, 0x00, strlen((char*) pcOutputString) );
		
			} while( xReturned != pdFALSE );
					

			/* All the strings generated by the input command have been sent.
			Clear the input	string ready to receive the next command.  Remember
			the command that was just processed first in case it is to be
			processed again. */
			strcpy( ( char * ) cLastInputString, ( char * ) cInputString );
			cInputIndex = 0;
			memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
			memset( idString, 0x00, MaxLengthOfAlias );
			
			/* Start to transmit a line separator, just to make the output easier to read. */
			if(!recordSnippet)
				writePxITMutex(port, pcEndOfCommandOutputString, strlen(pcEndOfCommandOutputString), cmd50ms);
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
BOS_Status AddSnippet(uint8_t type, char *string)
{
	BOS_Status result = BOS_OK;
	int16_t i = 0;
	
	if (currentSnipSize >= SNIPPETS_BUF_SIZE)
		return BOS_ERR_SNIP_MEM_FULL;	
	
	/* Check for activation code */
	if (type&0x01)
	{
		/* Search for last matching code */
		for(i=currentSnipSize ; i>=0 ; i--)
    {
			if ( (snippets[i]|0x01) == type) {
				snippets[i] = type;																					// Snippet has been activated (via LSB)
				break;
			}
    }
		/* Add end of Snippet delimiter */
		snippets[currentSnipSize++] = SNIPPET_END;
		/* Activate button events if this is a conditional button Snippet */
		ActivateButtonSnippet(i+1);
	}
	/* Normal code */
	else
	{
		switch (type)
		{
			case SNIPPET_CONDITION :
				snippets[currentSnipSize++] = SNIPPET_CONDITION;						// Add condition delimiter
				strcpy((char *)&snippets[currentSnipSize++], string);				// Copy the condition
				currentSnipSize += strlen(string);
				break;
			
			case SNIPPET_CONDITION_CMDS :
				snippets[currentSnipSize++] = SNIPPET_CONDITION_CMDS;				// Add condition Command delimiter
				strcpy((char *)&snippets[currentSnipSize++], string);				// Copy the Command
				currentSnipSize += strlen(string);
				break;
			
			default:
				break;
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* Process logical conditions in a Snippet
*/
BOS_Status ProcessSnippet(uint16_t location)
{
	BOS_Status result = BOS_ERROR;
	uint8_t port = 0;
	
	/* 1. Parse condition text */	
	// Todo: This is done every time. It should be performed on Snippet activation only
	
	/* 2. Check the state of logical condition */
	
	/* Test if condition starts with "bx." */
	if(snippets[location] == 'b' && snippets[location+2] == '.')
	{
		if(snippets[location+1] >= '0' && snippets[location+1] <= (NumOfPorts+'0'))		// Valid port number
		{
			port = snippets[location+1]-'0';
			/* Check if the event occured */
			if (!strncmp((char *)&snippets[location+3], "clicked", 7))
			{
				if (button[port].state == CLICKED)	return BOS_OK;
			}
			else if (!strncmp((char *)&snippets[location+3], "double clicked", 14))
			{
				if (button[port].state == DBL_CLICKED)	return BOS_OK;				
			}
			else if (!strncmp((char *)&snippets[location+3], "pressed for ", 12))
			{
				if (button[port].state == PRESSED_FOR_X1_SEC && button[port].pressedX1Sec == atoi((char *)&snippets[location+15]))	
					return BOS_OK;
				else if (button[port].state == PRESSED_FOR_X2_SEC && button[port].pressedX2Sec == atoi((char *)&snippets[location+15]))	
					return BOS_OK;		
				else if (button[port].state == PRESSED_FOR_X3_SEC && button[port].pressedX3Sec == atoi((char *)&snippets[location+15]))	
					return BOS_OK;					
			}
			else if (!strncmp((char *)&snippets[location+3], "released for ", 13))
			{
				if (button[port].state == RELEASED_FOR_Y1_SEC && button[port].releasedY1Sec == atoi((char *)&snippets[location+16]))	
					return BOS_OK;
				else if (button[port].state == RELEASED_FOR_Y2_SEC && button[port].releasedY2Sec == atoi((char *)&snippets[location+16]))	
					return BOS_OK;		
				else if (button[port].state == RELEASED_FOR_Y3_SEC && button[port].releasedY3Sec == atoi((char *)&snippets[location+16]))	
					return BOS_OK;					
			}
		}
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* Execute activated Command Snippets
*/
BOS_Status ExecuteSnippet(void)
{
	BOS_Status result = BOS_OK;
	portBASE_TYPE xReturned; uint16_t s = 0;
	int8_t *cInputString, *pcOutputString;
	
	/* Must get this address even if output is not used otherwise memory will corrupt */
	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console
	interface will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();	
	
	/* Go through activated Snippets */
	for(s=0 ; s<=currentSnipSize ; s++)
  {
		if (snippets[s]&0x01)									// Check the LSB (activated codes)
		{	
			switch (snippets[s]&0xFE)						// Go through actual codes
      {
      	case SNIPPET_CONDITION :
					if (ProcessSnippet(s+1) == BOS_OK)				// Process Snippet Condition at this location
					{
						/* Condition is TRUE, execute Snippet Commands */
						for(int c=s+1 ; c<=currentSnipSize ; c++)
						{
							/* Extract the command */
							if (snippets[c] == SNIPPET_CONDITION_CMDS)						
							{	
								cInputString = (signed char *) &snippets[c+1];
								/* Pass the received command to the command interpreter.  The
								command interpreter is called repeatedly until it returns
								pdFALSE as it might generate more than one string. */
								do
								{
									xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );	
								} while( xReturned != pdFALSE );
								/* Clear output buffer since we do not need it */
								memset( pcOutputString, 0x00, strlen((char*) pcOutputString) );
							}
							/* You reached the end of this Snippet so stop here */
							else if (snippets[c] == SNIPPET_END)
								break;
						}
						/* Disable the Snippet Move this to delete Snippets section */
						//snippets[s] &= 0xFE;
					}
																				
      		break;

      	default :													// This is not a code so exit
      		break;
      }
			
		}
  }
	
	return result;
}

/*-----------------------------------------------------------*/

/* Activate button events for a conditional button Snippet
*/
BOS_Status ActivateButtonSnippet(uint16_t location)
{
	BOS_Status result = BOS_OK;
	uint8_t port = snippets[location+1]-'0';

	/* Make sure it's a conditional button Snippet */
	if(snippets[location] == 'b' && snippets[location+2] == '.')
	{
		if(snippets[location+1] >= '0' && snippets[location+1] <= (NumOfPorts+'0'))		// Valid port number
		{	
			/* Check if appropriate events are enabled */
			if (!strncmp((char *)&snippets[location+3], "clicked", 7))
			{
				if ((button[port].events & BUTTON_EVENT_CLICKED) != BUTTON_EVENT_CLICKED)
					button[port].events |= BUTTON_EVENT_CLICKED;
			}
			else if (!strncmp((char *)&snippets[location+3], "double clicked", 14))
			{
				if ((button[port].events & BUTTON_EVENT_DBL_CLICKED) != BUTTON_EVENT_DBL_CLICKED)
					button[port].events |= BUTTON_EVENT_DBL_CLICKED;	
			}
			else if (!strncmp((char *)&snippets[location+3], "pressed for ", 12))
			{
				if ((button[port].events & BUTTON_EVENT_PRESSED_FOR_X1_SEC) != BUTTON_EVENT_PRESSED_FOR_X1_SEC) {
					button[port].events |= BUTTON_EVENT_PRESSED_FOR_X1_SEC;
					button[port].pressedX1Sec = atoi((char *)&snippets[location+15]);
				} else if ((button[port].events & BUTTON_EVENT_PRESSED_FOR_X2_SEC) != BUTTON_EVENT_PRESSED_FOR_X2_SEC) {
					button[port].events |= BUTTON_EVENT_PRESSED_FOR_X2_SEC;
					button[port].pressedX2Sec = atoi((char *)&snippets[location+15]);
				} else if ((button[port].events & BUTTON_EVENT_PRESSED_FOR_X3_SEC) != BUTTON_EVENT_PRESSED_FOR_X3_SEC) {
					button[port].events |= BUTTON_EVENT_PRESSED_FOR_X3_SEC;
					button[port].pressedX3Sec = atoi((char *)&snippets[location+15]);
				} else {
					return BOS_ERR_BUTTON_PRESS_EVENT_FULL;
				}		
			}
			else if (!strncmp((char *)&snippets[location+3], "released for ", 13))
			{
				/* First make sure this event is enabled */
				if ((button[port].events & BUTTON_EVENT_RELEASED_FOR_Y1_SEC) != BUTTON_EVENT_RELEASED_FOR_Y1_SEC) {
					button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y1_SEC;
					button[port].releasedY1Sec = atoi((char *)&snippets[location+16]);
				} else if ((button[port].events & BUTTON_EVENT_RELEASED_FOR_Y2_SEC) != BUTTON_EVENT_RELEASED_FOR_Y2_SEC) {
					button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y2_SEC;
					button[port].releasedY2Sec = atoi((char *)&snippets[location+16]);
				} else if ((button[port].events & BUTTON_EVENT_RELEASED_FOR_Y3_SEC) != BUTTON_EVENT_RELEASED_FOR_Y3_SEC) {
					button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y3_SEC;
					button[port].releasedY3Sec = atoi((char *)&snippets[location+16]);
				} else {
					return BOS_ERR_BUTTON_RELEASE_EVENT_FULL;
				}				
			}
		}
	}
	
	return result;	
}

/*-----------------------------------------------------------*/
