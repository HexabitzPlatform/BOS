/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_messaging.c
 Description   : Source code for Bitz messaging APIs.

 */

/* Includes ****************************************************************/
#include "BOS_messaging.h"

/* Private and global variables ********************************************/
uint8_t crcBuffer[MAX_MESSAGE_SIZE] ={0};
uint8_t StreamCplt =1;
uint16_t dstP[NumOfPorts] ={0};
uint32_t ports =0;

volatile uint8_t RemoteResponseFlag;
volatile uint8_t numOfElement;
volatile uint32_t RemoteResponseBuffer[4];

#ifndef __N
uint8_t route[MaxNumOfModules];
#else
	 uint8_t route[__N];
#endif

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

/* Exported Functions ******************************************************/
extern BOS_Status SaveEEstreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src1,uint8_t dst1,uint8_t src2,uint8_t dst2,uint8_t src3,uint8_t dst3);

/* Private function prototypes *********************************************/
BOS_Status SetupDMAStreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src,uint8_t dst);
BOS_Status BroadcastReceivedMessage(uint8_t dstType,uint8_t IncomingPort);
BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
void StreamTimerCallback(TimerHandle_t xTimerStream);
void NotifyMessagingTask(uint8_t port);

/***************************************************************************/
/*****************************  Private Functions **************************/
/***************************************************************************/

/* Setup DMA streams upon request from another module */
BOS_Status SetupDMAStreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src,uint8_t dst){
	TimerHandle_t xTimerStream = NULL;
	
	/* Sanity check to prevent invalid streaming requests */
	/* Prevent streaming within the same module */
	if(src == dst){
		portStatus[src] =STREAM;
		return BOS_ERR_WrongParam;
	}
	/* Ensure valid source and destination */
	if(src == 0 || dst == 0){
		return BOS_ERR_WrongParam;
	}
	
	/* Start DMA streams */
	if(direction == FORWARD){
		/* Virtual port: Source -> Destination memory */
		if(dst == P_VIRTUAL){
			if(StartDMAstream(GetUart(src),GetUart(dst),count) == BOS_ERR_PORT_BUSY)
				return BOS_ERR_PORT_BUSY;
		}
		else{
			if(StartDMAstream(GetUart(src),GetUart(dst),1) == BOS_ERR_PORT_BUSY)
				return BOS_ERR_PORT_BUSY;
		}
		/* Create a timeout timer */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(timeout),pdFALSE,(void* )&src,StreamTimerCallback);
		dmaStreamTotal[src - 1] =count;
	}

	else if(direction == BACKWARD){
		if(StartDMAstream(GetUart(dst),GetUart(src),1) == BOS_ERR_PORT_BUSY)
			return BOS_ERR_PORT_BUSY;
		/* Create a timeout timer */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(timeout),pdFALSE,(void* )&dst,StreamTimerCallback);
		dmaStreamTotal[dst - 1] =count;
	}

	else if(direction == BIDIRECTIONAL){
		if(StartDMAstream(GetUart(src),GetUart(dst),1) == BOS_ERR_PORT_BUSY)
			return BOS_ERR_PORT_BUSY;

		/* Create a timeout timer */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(timeout),pdFALSE,(void* )&src,StreamTimerCallback);
		dmaStreamTotal[src - 1] =count;
		if(StartDMAstream(GetUart(dst),GetUart(src),1) == BOS_ERR_PORT_BUSY)
			return BOS_ERR_PORT_BUSY;

		/* Create a timeout timer */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(timeout),pdFALSE,(void* )&dst,StreamTimerCallback);
		dmaStreamTotal[dst - 1] =count;
	}
	else
		return BOS_ERR_WrongParam;
	
	/* Store port mapping for active DMA streams */
	if(direction == FORWARD)
		dstP[src - 1] =(direction << 8) + dst;
	else if(direction == BACKWARD)
		dstP[dst - 1] =(direction << 8) + src;
	else if(direction == BIDIRECTIONAL){
		dstP[src - 1] =(direction << 8) + dst;
		dstP[dst - 1] =(direction << 8) + src;
	}

	ports =(direction << 16) + (dst << 8) + src;

	/* Start timeout timers if they were created successfully */
	if(xTimerStream != NULL)
		xTimerStart(xTimerStream,portMAX_DELAY);
	
	return BOS_OK;
}

/***************************************************************************/
/* DMA stream timer callback */
void StreamTimerCallback(TimerHandle_t xTimerStream){
	uint8_t srcP =0;
	uint8_t dstP =0;
	uint8_t direction =0;

	/* Extract direction, destination, and source from 'ports' */
	direction =ports >> 16;
	dstP =ports >> 8;
	srcP =(uint8_t )ports;

    /* Process timeout based on the streaming direction */
    switch (direction) {
        case FORWARD:
            SwitchStreamDMAToMsg(srcP);
            break;

        case BACKWARD:
            SwitchStreamDMAToMsg(dstP);
            StreamCplt = 1;  /* Mark stream as complete */
            break;

        case BIDIRECTIONAL:
            SwitchStreamDMAToMsg(srcP);
            SwitchStreamDMAToMsg(dstP);
            StreamCplt = 1;  /* Mark stream as complete */
            break;

        default:
            /* Handle invalid direction */
            break;
    }
}

/***************************************************************************/
/* Forward a received message to its destination */
BOS_Status ForwardReceivedMessage(uint8_t incomingPort){
	BOS_Status result =BOS_OK;
	uint8_t port =0;
	uint8_t dst =0;
	
	/* Single-cast. Do not add broadcast ID */
	AddBcastPayload = false;
	
	dst =cMessage[incomingPort - 1][0];
	
	/* Find best output port for destination module */
#ifdef __N
	port = Output_Port_Array[dst - 1];
#else
	port =FindRoute(myID,dst);
#endif
	
	/* Forward the message.
	 * Set src and code to 0 to inform the API to copy the exact message received on
	 * incomingPort which is passed thru numberOfParams and to use port as output port */
	SendMessageFromPort(port,0,dst,0,incomingPort);
	
	return result;
}

/***************************************************************************/
/* Broadcast a received message to all connected modules */
BOS_Status BroadcastReceivedMessage(uint8_t dstGroup,uint8_t incomingPort){
	BOS_Status result =BOS_OK;
	
	/* Broadcast ID and groups are already in the payload. Don't add new ones */
	AddBcastPayload = false;
	dstGroupID =dstGroup;
	
	/* Forward the message with a broadcast flag.
	 * Set src and code to 0 to inform the API to copy the exact message received on
	 * incomingPort which is passed thru numberOfParams.
	 * Src will be updated with original source inside the function. */

	if(dstGroup == BOS_BROADCAST)
		SendMessageFromPort(0,0,BOS_BROADCAST,0,incomingPort);
	else
		SendMessageFromPort(0,0,BOS_MULTICAST,0,incomingPort);
	
	return result;
}

/***************************************************************************/
/* Activate Messaging Tasks */
void NotifyMessagingTask(uint8_t port){
	switch(port){
#ifdef _P1
		case P1:
			xTaskNotifyGive(P1MsgTaskHandle);
			break;
#endif
#ifdef _P2
		case P2:
			xTaskNotifyGive(P2MsgTaskHandle);
			break;
#endif
#ifdef _P3
		case P3:
			xTaskNotifyGive(P3MsgTaskHandle);
			break;
#endif
#ifdef _P4
		case P4:
			xTaskNotifyGive(P4MsgTaskHandle);
			break;
#endif
#ifdef _P5
		case P5:
			xTaskNotifyGive(P5MsgTaskHandle);
			break;
#endif
#ifdef _P6
		case P6:
			xTaskNotifyGive(P6MsgTaskHandle);
			break;
#endif
		default:
			break;
	}
}

/***************************************************************************/
/* Broadcast a message to all connected modules */
BOS_Status BroadcastMessage(uint8_t src,uint8_t dstGroup,uint16_t code,uint16_t numberOfParams){
	/* Set a flag to populate broadcast ID and groups */
	AddBcastPayload = true;
	dstGroupID =dstGroup;
	
	/* Send the message out with a broadcast flag */
	if(dstGroup == BOS_BROADCAST)
		SendMessageFromPort(0,src,BOS_BROADCAST,code,numberOfParams);
	else
		SendMessageFromPort(0,src,BOS_MULTICAST,code,numberOfParams);
	
	/* Reset messageParams buffer */
	memset(messageParams,0,numberOfParams);
	
	return BOS_OK;
}

/***************************************************************************/
/* Read message codes data from a remote sensor "input" module */
BOS_Status ReadDataFromSensorModule(uint8_t disModuleID,uint16_t Code,uint32_t *pDataReceived,uint16_t timeout){
	BOS_Status result =BOS_OK;
	uint8_t dataIndex =0;
	uint32_t tickstart =HAL_GetTick();

	/* Sending a message to the module */
	messageParams[0] =myID; /* source module ID */
	SendMessageToModule(disModuleID,Code,1);

	/* timeout loop */
	while(0 == RemoteResponseFlag){
		if((HAL_GetTick() - tickstart) > timeout)
			return BOS_ERR_TIMEOUT;
	}

	if(RemoteResponseFlag){
		RemoteResponseFlag =0;
		tickstart =HAL_GetTick();
		for(dataIndex =0; dataIndex < numOfElement; dataIndex++)
			pDataReceived[dataIndex] =RemoteResponseBuffer[dataIndex];

		/* NULL DATA */
		if((0 == RemoteResponseBuffer[0]) && (0 == RemoteResponseBuffer[1]) && (0 == RemoteResponseBuffer[2]) && (0 == RemoteResponseBuffer[3]))
			return BOS_ERROR;

		for(size_t i =0; i < sizeof(RemoteResponseBuffer) / sizeof(RemoteResponseBuffer[0]); i++){
			RemoteResponseBuffer[i] =0x00;
		}

		return BOS_OK;
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Send a message to a group of modules. If current module is part of the group it will be exempted */
BOS_Status SendMessageToGroup(char *group,uint16_t code,uint16_t numberOfParams){
	BOS_Status result =BOS_OK;
	uint8_t i =0;
	
	/* Search for group alias*/

	for(i =0; i < MaxNumOfGroups; i++){
		/* This group exists */
		if(!strcmp(group,groupAlias[i])){
			/* Multicast the message to this group */
			result =BroadcastMessage(myID,i,code,numberOfParams);
			
			return result;
		}
	}
	
	/* This group does not exist */
	return BOS_ERR_WrongGroup;
}

/***************************************************************************/
/* Send a message to another module */
BOS_Status SendMessageToModule(uint8_t dst,uint16_t code,uint16_t numberOfParams){
	BOS_Status result =BOS_OK;
	uint8_t port =0;
	
	/* Single-cast message */
	if(dst != BOS_BROADCAST){

		/* Find best output port for destination module */
#ifdef __N
	    port = Output_Port_Array[dst - 1];
#else
		port =FindRoute(myID,dst);
#endif
		
		/* Transmit the message from this port */
		SendMessageFromPort(port,myID,dst,code,numberOfParams);
		
		/* Reset messageParams buffer */
		memset(messageParams,0,numberOfParams);
	}
	/* Broadcast message */
	else{
		BroadcastMessage(myID,BOS_BROADCAST,code,numberOfParams);
	}
	
	return result;
}

/***************************************************************************/
/* Send large data (over 46 Bytes) to module */
BOS_Status SendLargeMessageToModule(uint8_t dst,uint16_t code,uint8_t *pParameters,uint16_t numberOfParams){
	uint16_t totalNumberOfParams =numberOfParams;
	uint16_t ptrShift =0, chunkSize =0;
	uint8_t port =0;
	bool LongMessageFlag = false;

	/* Find best output port for destination module */
#ifdef __N
		port = Output_Port_Array[dst - 1];
#else
	    port =FindRoute(myID,dst);
#endif

	while(totalNumberOfParams > 0){
		chunkSize =(totalNumberOfParams > MAX_PARAMS_PER_MESSAGE) ?MAX_PARAMS_PER_MESSAGE: totalNumberOfParams;

		/* Copy the relevant chunk of data into messageParams */
		memcpy(messageParams,pParameters + ptrShift,chunkSize);

		/* Update total number of remaining parameters */
		totalNumberOfParams -=chunkSize;
		ptrShift +=chunkSize;

		if(totalNumberOfParams > 0){
			/* Set long message flag */
			OptionByte.LongMessage = true;
		}
		else{
			/* Last message, clear long message flag */
			OptionByte.LongMessage = false;
		}

		/* Send the message */
		SendMessageToModule(dst,code,chunkSize);

	}

	return BOS_OK;
}

/***************************************************************************/
/* --- Send a message from a specific port 
 Note: The messageParams buffer does not get erased here to enable reuse for other transmissions.
 Make sure you manually erase the buffer when you're done with it. 

 #		port			src				dst							case
 ====================================================================================
 1		0				!0				!0							Broadcast or multi-cast message.
 2		0				 0				!0							Broadcast or multi-cast message forwarded from another port (which is passed to the API thru numberOfParams).
 3		0				!0				 0							Not allowed.
 4		0				 0				 0							Not allowed.
 5	   !0				!0				!0							Single-cast message.
 6     !0        		 0				!0							Either single-cast message with myID as source module OR (if code == 0)
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	    single-cast message forwarded from another port (which is passed to the API thru numberOfParams).
 7     !0        	    !0				 0							Not allowed.
 8     !0                0				 0							Message sent to adjacent neighbor (e.g., if ID is unknown) with myID as source module.
 */
BOS_Status SendMessageFromPort(uint8_t port,uint8_t src,uint8_t dst,uint16_t code,uint16_t numberOfParams){
	BOS_Status result =BOS_OK;
	UBaseType_t TaskPriority;
	bool extendCode = false;
	bool extendOptions = false;
	uint8_t shift =0;
	uint8_t length =0;
	uint8_t groupMembers =0;
	static uint8_t LongMessageFlag =0;
	static uint16_t totalNumberOfParams =0;
	static uint16_t ptrShift =0, pp =0;

	/* Sanity check for invalid broadcast/multi-cast cases */
	if((port == 0 && dst == 0) ||																												// cases 3 & 4
	(port == 0 && dst != BOS_BROADCAST && dst != BOS_MULTICAST) || 				// cases 1 & 2
	(port != 0 && src != 0 && dst == 0)){																						// case 7
		return BOS_ERR_WrongParam;
	}
	
	/* Increase the priority of current running task */
	TaskPriority =uxTaskPriorityGet( NULL);
	vTaskPrioritySet( NULL,osPriorityHigh - osPriorityIdle);
	
	/***************************************************************************/
	/* Start constructing the message buffer ***********************************/
	/***************************************************************************/

	/* Add the HZ Delimiter to mark the start of the message */
	message[0] ='H';
	message[1] ='Z';
	
	/* If copying from an incoming message (e.g., broadcast or received message) */
	if((port == 0 && src == 0 && (dst == BOS_BROADCAST || dst == BOS_MULTICAST)) || code == 0)					// case 2 and part of case 6
	{
		/* Retrieve message length from received buffer */
		length =messageLength[numberOfParams - 1];
		
		/* Copy message buffer from the received message */
		memcpy(&message[3],&cMessage[numberOfParams - 1][0],(size_t )length);
	}
	/* Construct message from scratch - case 5 */
	else{
		/* Sending to adjacent neighbors - case 2, case 8 and part of case 6 */
		if(src == 0)
			src =myID;
		
		/* ToDo: Implement extended options */
		if(OptionByte.ExtendedOptions == true)
			++shift;

		/* Extended code flag? */
		if(code > 0xFF)
			OptionByte.ExtendedMessageCode = true;
		
		/* Construct the message */

		/* Header */
		message[2] =length;
		message[3] =dst;
		message[4] =src;

		/* Options */
		/* Set the options bits */
//	    OptionByte.Trace = UserOptionByte.Trace;
//	    OptionByte.Acknowledgment = UserOptionByte.Acknowledgment;
//	    OptionByte.Reserved = 0;
//	    OptionByte.Response = UserOptionByte.Response;
//	    OptionByte.LongMessage = LongMessageFlag;

		/* Assign the byte value */
		message[5] =*(uint8_t* )&OptionByte;

		/* Code - LSB first */
		message[6 + shift] =(uint8_t )code;

		if(OptionByte.ExtendedMessageCode){
			++shift;
			message[6 + shift] =(uint8_t )(code >> 8);
		}
		
		/* Parameters */
		if(numberOfParams <= MAX_PARAMS_PER_MESSAGE){
			memcpy((char* )&message[7 + shift],(&messageParams[0] + ptrShift),numberOfParams);
			/* Calculate message length */
			length =numberOfParams + shift + 4;
		}
//		else{
//			/* Long message: Set Options byte 8th bit */
//			LongMessageFlag = true;
//			message[5] |=0x80;
//			totalNumberOfParams =numberOfParams;
//			numberOfParams = MAX_PARAMS_PER_MESSAGE;
//			/* Break into multiple messages */
//			while(totalNumberOfParams != 0){
//				if((totalNumberOfParams / numberOfParams) >= 1){
//					/* Call this function recursively */
//					SendMessageFromPort(port,src,dst,code,numberOfParams);
////					osDelay(10);
//					/* Update remaining number of parameters */
//					totalNumberOfParams -=numberOfParams;
//					ptrShift +=numberOfParams;
//				}
//				else{
//					LongMessageFlag = false;
//					message[5] &=0x7F; /* Last message. Reset long message flag */
//					numberOfParams =totalNumberOfParams;
//					memcpy((char* )&message[7 + shift],(&messageParams[0] + ptrShift),numberOfParams);
//					ptrShift =0;
//					totalNumberOfParams =0;
//					/* Calculate message length */
//					length =numberOfParams + shift + 4;
//				}
//			}
//		}
		
		/* Check if brodcast payload (bcast ID and groups) should be appended to message payload */
		/* TODO - handle the edge case of brodcast/multi-cast long message.
		 * bcastID should go into each message but the groups only in the last one */
		if(AddBcastPayload == true){

			/* If it's a multi-cast, add group members */
			if(dstGroupID < BOS_BROADCAST){
				/* Extract and add group member IDs to the Message */
				for(uint16_t i =1; i <= N; i++)						// N modules
				    {
					if(InGroup(i,dstGroupID)){
						++groupMembers;							// Add this member
						if((numberOfParams + groupMembers + 1) < MAX_PARAMS_PER_MESSAGE)
							message[7 + shift + numberOfParams + groupMembers - 1] =i;
						else
							return BOS_ERR_MSG_DOES_NOT_FIT;
					}
				}
				/* Add number of members */
				message[7 + shift + numberOfParams + groupMembers] =groupMembers;
			}
			
			/* Add unique broadcast ID */
			if((dstGroupID == BOS_BROADCAST) && ((numberOfParams + 1) < MAX_PARAMS_PER_MESSAGE))
				message[7 + shift + numberOfParams] =++bcastID;
			else if(dstGroupID == BOS_BROADCAST)
				return BOS_ERR_MSG_DOES_NOT_FIT;
			else if((dstGroupID < BOS_BROADCAST) && ((numberOfParams + groupMembers + 2) < MAX_PARAMS_PER_MESSAGE))		// Multicast
				message[7 + shift + numberOfParams + groupMembers + 1] =++bcastID;
			else if(dstGroupID < BOS_BROADCAST)																																		// Multicast
				return BOS_ERR_MSG_DOES_NOT_FIT;
			
			/* Calculate new message length */
			if(dstGroupID == BOS_BROADCAST)
				length +=1;		// + bcastID
			else
				length +=groupMembers + 2;		// + bcastID + number of group member + group members IDs 
		}
	}
	
	/* Copy message length */
	message[2] =length;
	
	/* Compute and append CRC */
	memcpy(crcBuffer,&message[0],length + 3);
	message[length + 3] =CalculateCRC8(crcBuffer,(length + 3));
	memset(crcBuffer,0,sizeof(crcBuffer));
	
	/* Send Single-cast Message */
	if(dst != BOS_BROADCAST && dst != BOS_MULTICAST){
//		writePxITMutex(port,message,length + 4,cmd50ms);
		Send_BOS_Message(port,message,length + 4,cmd50ms,dst);

//		if(code == MSG_Acknowledgment_Accepted || code == MSG_rejected){
////			Send_BOS_Message(port,message,length + 4,cmd50ms,dst);
//			writePxITMutex(port,message,length + 4,cmd50ms);
//		}
//		else{
//			for(uint8_t Number_of_attempt =0; Number_of_attempt < 3; Number_of_attempt++){
////				Send_BOS_Message(port,message,length + 4,cmd50ms,dst);
//				writePxITMutex(port,message,length + 4,cmd50ms);
//				if(ACK_FLAG == true)
//					break;
//				if(rejected_FLAG == true)
//					Send_BOS_Message(port,message,length + 4,cmd50ms,dst);
//			}
//		}
//		ACK_FLAG =false;
//		rejected_FLAG =false;
	}

	else{
		/* Broadcast or multicast handling */

		/* Update original source ID */
		if(code == 0 && src == 0){
			src =message[4];
		}
		
		/* Get broadcast routes */
		FindBroadcastRoutes(src);
		
		/* Send to all my broadcast ports */
		for(uint8_t p =1; p <= NumOfPorts; p++){
			if((bcastRoutes[myID - 1] >> (p - 1)) & 0x01){
				/* Transmit the message from this port */
				Send_BOS_Message(p,message,length + 4,cmd50ms,dst);
//				writePxITMutex(p,message,length + 4,cmd50ms);
				if(rejected_FLAG == true){
					Send_BOS_Message(port,message,length + 4,cmd50ms,dst);
//					writePxITMutex(port,message,length + 4,cmd50ms);
				}
			}
//			rejected_FLAG=false;
			Delay_us(10);
		}
	}
	
	/* Restore the original task priority */
	vTaskPrioritySet( NULL,TaskPriority);
	
	/* Reset responseStatus in case response is expected
	 * TODO should be tailored for each port */
	responseStatus =BOS_ERR_NoResponse;
	
	return result;
}

/***************************************************************************/
/* Start a single-cast DMA stream across the array.
 * Transfer ends after (count) bytes are transferred or timeout (ms),
 * whichever comes first. If stored = true, the stream is stored in emulated eeprom */
BOS_Status StartScastDMAStream(uint8_t srcP,uint8_t srcM,uint8_t dstP,uint8_t dstM,uint8_t direction,uint32_t count,uint32_t timeout,bool stored){
	BOS_Status result =BOS_OK;
	uint8_t port =0, temp1 =0, temp2 =0;
	
	/* Is the source a different module? */
	if(srcM != myID){
		/* Forward this task to the source module */
		messageParams[0] =(uint8_t )(count >> 24); /* Count */
		messageParams[1] =(uint8_t )(count >> 16);
		messageParams[2] =(uint8_t )(count >> 8);
		messageParams[3] =(uint8_t )count;
		messageParams[4] =(uint8_t )(timeout >> 24); /* Timeout */
		messageParams[5] =(uint8_t )(timeout >> 16);
		messageParams[6] =(uint8_t )(timeout >> 8);
		messageParams[7] =(uint8_t )timeout;
		messageParams[8] =direction; /* Stream direction */
		messageParams[9] =srcP;      /* Source port */
		messageParams[10] =dstM;     /* destination module */
		messageParams[11] =dstP;     /* destination port */
		messageParams[12] =stored;   /* EEPROM storage */

		/* Send the message to the source module */
		SendMessageToModule(srcM,CODE_DMA_SCAST_STREAM,13);
		
		return result;
	}
	
	/* Inform all participating modules along the route */
	for(uint8_t i =0; i < sizeof(route); i++){
		FindRoute(srcM,dstM);
		/* Message other modules */
		if(route[i]){
			/* Find out the inport and outport to this module from previous one */
			if(route[i + 1]){
				temp1 =FindRoute(route[i],route[i + 1]);
			}
			else{
				temp1 =FindRoute(route[i],srcM);
			}
			FindRoute(srcM,dstM);
			if(route[i] == dstM){
				temp2 =dstP;
			}
			else{
				temp2 =FindRoute(route[i],route[i - 1]);
			}
			/* Message parameters*/
			messageParams[0] =(uint8_t )(count >> 24); /* Count */
			messageParams[1] =(uint8_t )(count >> 16);
			messageParams[2] =(uint8_t )(count >> 8);
			messageParams[3] =(uint8_t )count;
			messageParams[4] =(uint8_t )(timeout >> 24); /* Timeout */
			messageParams[5] =(uint8_t )(timeout >> 16);
			messageParams[6] =(uint8_t )(timeout >> 8);
			messageParams[7] =(uint8_t )timeout;
			messageParams[8] =direction; /* Stream direction */
			messageParams[9] =temp1;     /* Source port */
			messageParams[10] =temp2;    /* destination port */
			messageParams[11] =stored;   /* EEPROM storage */
			FindRoute(srcM,dstM);

			/* Send message to the current route module */
			SendMessageToModule(route[i],CODE_DMA_CHANNEL,12);
			osDelay(10);
		}
	}

	/* Check if the source port is virtual (indicating memory transfer) */
	if(srcP != P_VIRTUAL){
		if(srcM == dstM)
			port =dstP;
		else
			port =FindRoute(srcM,dstM);

		/* Setup my own DMA stream */
		SetupDMAStreams(direction,count,timeout,srcP,port);

		/* If storage is enabled, save the stream configuration */
		if(stored){
			SaveEEstreams(direction,count,timeout,srcP,port,0,0,0,0);
		}
	}
	
	return result;
}

/***************************************************************************/
/*
 * @brief: Transferring stream data from port in the source module to port in the destination module and vice versa.
 * @param1: source port number.
 * @param2: source module id.
 * @param3: destination port number.
 * @param4: destination module id.
 * @param5: flow data direction(FORWARD,BACKWARD,BIDIRECTIONAL)
 * @param6: size of stream data.
 * @param7: flow data timeout.
 * @param8: storing the physical data flow path in EEPROM memory(true,false)
 * @retval: BOS_Status.
 */
BOS_Status StreamPortToPort(uint8_t srcP,uint8_t srcM,uint8_t dstP,uint8_t dstM,uint8_t direction,uint32_t size,uint32_t timeout,bool stored){
	BOS_Status result =BOS_OK;

	/* If the stream completes either by reaching the total size limit or by timing out, reconfigure the stream path */
	if(StreamCplt == 1){
		if(BOS_OK != StartScastDMAStream(srcP,srcM,dstP,dstM,direction,size,timeout,stored))
			return result =BOS_ERROR;

		StreamCplt =0;
	}

	return result;
}

/***************************************************************************/
/*
 * @brief: Transferring stream data from port in the source module to RAM memory in the destination module.
 * @param1: source port number.
 * @param2: destination module id.
 * @param3: size of stream data.
 * @param4: flow data timeout.
 * @param5: storing the physical data flow path in EEPROM memory(true,false)
 * @retval: BOS_Status.
 */
BOS_Status StreamPortToMemory(uint8_t srcP,uint8_t dstM,uint32_t size,uint32_t timeout,bool stored){
	BOS_Status result =BOS_OK;
	uint8_t port =0;

	/* If the stream completes either by reaching the total size limit or by timing out, reconfigure the stream path */
	if(StreamCplt == 1){
		if(BOS_OK != StartScastDMAStream(srcP,myID,P_VIRTUAL,dstM,FORWARD,size,timeout,stored))
			return result =BOS_ERROR;

		StreamCplt =0;
	}

	return result;
}

/***************************************************************************/
/*
 * @brief: Transferring stream data from RAM memory in the source module to port in the destination module.
 * @param1: destination port number.
 * @param2: destination module id.
 * @param3: buffer to be sent.
 * @param4: size of stream data.
 * @param5: flow data timeout.
 * @param6: storing the physical data flow path in EEPROM memory(true,false)
 * @retval: BOS_Status.
 */
BOS_Status StreamMemoryToPort(uint8_t dstP,uint8_t dstM,uint8_t *pBuffer,uint32_t size,uint32_t timeout,bool stored){
	BOS_Status result =BOS_OK;
	uint8_t port =0;

	/* If the stream completes either by reaching the total size limit or by timing out, reconfigure the stream path */
	if(StreamCplt == 1){
		if(BOS_OK != StartScastDMAStream(P_VIRTUAL,myID,dstP,dstM,FORWARD,size,timeout,stored))
			return result =BOS_ERROR;

		StreamCplt =0;
	}
	if(myID == dstM)
		port =dstP;
	else
		port =FindRoute(myID,dstM);
	/* Timeout before sending data to ensure the UART DMA destination is set */
	HAL_Delay(10);
	HAL_UART_Transmit_IT(GetUart(port),pBuffer,size);

	return result;
}

/***************************************************************************/
/*
 * @brief: Transferring stream data from RAM memory in the source module to RAM memory in the destination module.
 * @param1: destination module id.
 * @param2: buffer to be sent.
 * @param3: size of stream data.
 * @param4: flow data timeout.
 * @param5: storing the physical data flow path in EEPROM memory(true,false)
 * @retval: BOS_Status.
 */
BOS_Status StreamMemoryToMemory(uint8_t dstM,uint8_t *pBuffer,uint32_t size,uint32_t timeout,bool stored){
	BOS_Status result =BOS_OK;
	uint8_t port =0;

	/* If the stream completes either by reaching the total size limit or by timing out, reconfigure the stream path */
	if(StreamCplt == 1){
		if(BOS_OK != StartScastDMAStream(P_VIRTUAL,myID,P_VIRTUAL,dstM,FORWARD,size,timeout,stored))
			return result =BOS_ERROR;

		StreamCplt =0;
	}
	port =FindRoute(myID,dstM);
	/* Timeout before sending data to ensure the UART DMA destination is set */
	HAL_Delay(10);
	HAL_UART_Transmit_IT(GetUart(port),pBuffer,size);

	return result;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
