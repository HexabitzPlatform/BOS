/*
    BitzOS (BOS) V0.2.2 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved

    File Name     : BOS_messaging.c
    Description   : Source code for Bitz messaging APIs.

*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private and global variables ----------------------------------------------*/
/* Exporterd variables */
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE];		// Buffer for messages received and ready to be parsed 
extern char message[MAX_MESSAGE_SIZE];										// Buffer to construct a message to be sent
extern uint8_t dstGroupID;
extern uint16_t bcastRoutes[MaxNumOfModules];				/* P1 is LSB */
extern uint8_t crcBuffer[MAX_MESSAGE_SIZE];
extern bool AddBcastPayload;

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


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

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
	uint8_t length = 0, shift = 0; static uint16_t totalNumberOfParams = 0; static uint16_t ptrShift = 0;
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
		message[5] = (BOS.response) | (BOS.trace<<2) | (extendCode<<1) | (extendOptions);
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
			memcpy((char*)&message[7+shift], (&messageParams[0]+ptrShift), numberOfParams);
			/* Calculate message length */
			length = numberOfParams + shift + 4;
		} else {
			/* Long message: Set Options byte 8th bit */
			message[5] |= 0x80;		
			totalNumberOfParams = numberOfParams;
			numberOfParams = MAX_PARAMS_PER_MESSAGE;
			/* Break into multiple messages */
			while (totalNumberOfParams != 0)
			{		
				if ( (totalNumberOfParams/numberOfParams) >= 1) 
				{	
					/* Call this function recursively */
					SendMessageFromPort(port, src, dst, code, numberOfParams);
					osDelay(10);
					/* Update remaining number of parameters */
					totalNumberOfParams -= numberOfParams;
					ptrShift += numberOfParams;
				} 
				else 
				{
					message[5] &= 0x7F;		/* Last message. Reset long message flag */
					numberOfParams = totalNumberOfParams;
					memcpy((char*)&message[7+shift], (&messageParams[0]+ptrShift), numberOfParams);
					ptrShift = 0; totalNumberOfParams = 0;
					/* Calculate message length */
					length = numberOfParams + shift + 4;
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
	memcpy(crcBuffer, &message[0], length + 3);
	
	/* crc calculation function added for test */
	message[length+3] = CalculateCRC8((uint32_t *)&crcBuffer, (length + 3));
	/* end of crc calculation function addition */
	
//	message[length+3] = HAL_CRC_Calculate(&hcrc, (uint32_t *)&crcBuffer, (length + 3)/4);
//	if ((length + 3)%4 != 0) 							// Non-word-aligned packet
//		message[length+3] = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&crcBuffer[((length + 3)/4)*4], 1);

	memset(crcBuffer, 0, sizeof(crcBuffer));
	//if(! message[length+3]){message[length+3]=1;}  /*Making sure CRC Value Is not Zero*/
	
	/* Transmit the message - single-cast */
	if (dst != BOS_BROADCAST && dst != BOS_MULTICAST) 
	{
		writePxDMAMutex(port, message, length+4, cmd50ms);
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

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
