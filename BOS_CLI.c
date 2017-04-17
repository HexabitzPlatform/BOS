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
		MODIFIED by Hexabitz for BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved
*/

#include "BOS.h"


/*-----------------------------------------------------------*/


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

/*-----------------------------------------------------------*/

void prvUARTCommandConsoleTask( void *pvParameters )
{
char cRxedChar; int8_t cInputIndex = 0, *pcOutputString; uint8_t port, m = 1;
static int8_t cInputString[ cmdMAX_INPUT_SIZE ], cLastInputString[ cmdMAX_INPUT_SIZE ];
char* loc = 0; uint8_t id = 0; char idString[MaxLengthOfAlias] = {0};
portBASE_TYPE xReturned;

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
			/* The input command string is complete.  Ensure the previous
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
				
				/* Check if command contains a dot */
				loc = strchr( ( char * ) cInputString, '.');
				if (loc != NULL) {					
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
								
						if (!(broadcastResponse[m-1]) ) {	
							/* Execute locally */
							if (m == myID) {
								xReturned = FreeRTOS_CLIProcessCommand( (const signed char*)(loc+1), pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
								if (xReturned == pdFALSE)	{
									broadcastResponse[myID-1] = 1;
									/* Activate next message */
									xReturned = pdTRUE; ++m;
								}			
							/* Broadcast the command */								
							} else {
								strncpy( ( char * ) messageParams, loc+1, (size_t)(strlen( (char*) cInputString)-strlen( (char*) idString)-1));
								SendMessageToModule(m, CODE_CLI_command, strlen( (char*) cInputString)-strlen( (char*) idString)-1);
								/* Wait for response for a maximum of 1000msec */
								ulTaskNotifyTake(pdTRUE, 1000);		
								/* If timeout */
								if (responseStatus != BOS_OK)	
									sprintf( ( char * ) pcOutputString, "Module %d is not reachable.\n\r", m);
								else
									broadcastResponse[m-1] = 1;
								/* Activate next message */
								xReturned = pdTRUE; ++m;
								osDelay(50);
							}
							/* Is broadcast finished? */
							if (m > N) {
								xReturned = pdFALSE;
								m = 1;
								memset( broadcastResponse, 0x00, sizeof(broadcastResponse) );			
							}
						}									
					}	else {
						/* Forward the command */
						strncpy( ( char * ) messageParams, loc+1, (size_t)(strlen( (char*) cInputString)-strlen( (char*) idString)-1));
						SendMessageToModule(id, CODE_CLI_command, strlen( (char*) cInputString)-strlen( (char*) idString)-1);
						xReturned = pdFALSE;
						/* Wait for response for a maximum of 1000msec */
						ulTaskNotifyTake(pdTRUE, 5000);		//cmd500ms
						/* If timeout */
						if (responseStatus != BOS_OK)
							sprintf( ( char * ) pcOutputString, "Module %d is not reachable.\n\r", id);
					}						
				} else {
					/* Process the command locally */
					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );		
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



