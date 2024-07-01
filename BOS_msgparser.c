/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_msgparser.c
 Description   : Source code for Bitz messaging parser.
 
 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

//New BackEndTask Variables:
uint16_t Accepted_Messages = 0, Rejected_Messages = 0, Message_counter=0;
uint8_t Calculate_CRC_Buffer[MSG_MAX_SIZE];
/* Private and global variables ----------------------------------------------*/
/* Used in the run time stats calculations */


uint16_t stackWaterMark;
uint16_t rejectedMsg =0, acceptedMsg =0, timedoutMsg =0, ADCPort =0, ADCSide =0;
float InternalVoltageReferance =0, InternalTemperature =0, ADCPercentage =0, ADCValue =0;
uint32_t totalnumberofrecevedmesg =0;
int packetStart =0, packetEnd =0, packetLength =0, parseStart =0;

/* Receiving the Defalt_Value for the H1DR5 module */
receive_defalt_value defalt_data;

/* Remote Buffer of Messages */
RemoteDataBuffer_t RemoteDataBuffer;

/* Exported Variables */
extern uint8_t cMessage[NumOfPorts][MAX_MESSAGE_SIZE]; // Buffer for messages received and ready to be parsed 
extern char message[MAX_MESSAGE_SIZE]; // Buffer to construct a message to be sent
extern uint8_t crcBuffer[MAX_MESSAGE_SIZE];
extern uint8_t UARTRxBufIndex[NumOfPorts];
extern uint8_t messageLength[NumOfPorts];
extern uint8_t messageParams[MAX_PARAMS_PER_MESSAGE];
volatile uint32_t MBmessageParams[9] ={0};
extern char cRxedChar;
extern uint8_t longMessage;
extern uint16_t longMessageLastPtr;
static uint8_t longMessageScratchpad[(MaxNumOfPorts + 1) * MaxNumOfModules];
extern BOS_Status responseStatus;
extern uint8_t bcastID; // Counter for unique broadcast ID
extern uint8_t PcPort;
extern uint8_t BOS_initialized;
extern uint64_t remoteBuffer;
extern varFormat_t remoteVarFormat;
#ifndef __N
extern uint16_t array[MaxNumOfModules][MaxNumOfPorts + 1]; /* Array topology */
extern uint16_t arrayPortsDir[MaxNumOfModules];
extern uint8_t broadcastResponse[MaxNumOfModules];
extern uint16_t groupModules[MaxNumOfModules];
#else
extern uint16_t arrayPortsDir[__N ];
#endif

/* Routing and Topology */
extern uint16_t neighbors2[NumOfPorts][2];

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

/* Private function prototypes -----------------------------------------------*/
extern uint8_t SaveToRO(void);
#ifndef __N
extern uint8_t ClearROtopology(void);
#endif
extern uint8_t LoadROtopology(void);
extern BOS_Status SaveEEportsDir(void);
extern BOS_Status ClearEEportsDir(void);
extern BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
extern BOS_Status BroadcastReceivedMessage(uint8_t dstType,uint8_t IncomingPort);
extern BOS_Status SetupDMAStreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src,uint8_t dst);
extern BOS_Status User_MessagingParser(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);
extern void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
/* Module exported internal functions */
extern Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Module exported internal functions */
extern uint8_t IsModuleParameter(char *name);

/* BOS exported internal functions */
extern void CheckAttachedButtons(void);
extern void ResetAttachedButtonStates(uint8_t *deferReset);
extern BOS_Status ExecuteSnippet(void);
extern void NotifyMessagingTask(uint8_t port);
/* -----------------------------------------------------------------------
 |												 Private Functions	 		|
 -----------------------------------------------------------------------
 */
/* BackEndTask function */
void BackEndTask(void *argument){

	uint8_t calculated_crc,port_number,length,port_index;

			uint8_t temp_length[NumOfPorts] = {0};
			uint8_t temp_index[NumOfPorts] = {0};

	for(;;)
	{
       for(port_DMA=0;port_DMA<NumOfPorts;)
       {port_index=port_DMA;
		index_input[port_DMA]=MSG_RX_BUF_SIZE-(*index_dma[port_DMA]);

		if(index_input[port_DMA] !=index_process[port_DMA])
		{ port_number =port_DMA+1;
			if(UARTRxBuf[port_number-1][index_process[port_DMA]] == 0x0D && portStatus[port_number] == FREE)
			{
				for(int i=0;i<=NumOfPorts;i++) // Free previous CLI port
				{
					if(portStatus[i] == CLI)
					{
						portStatus[i] = FREE;
					}
				}
				portStatus[port_number] =CLI; // Continue the CLI session on this port
				PcPort = port_number;

				CLI_Data = UARTRxBuf[port_number-1][index_process[port_DMA]];

				xTaskNotifyGive(xCommandConsoleTaskHandle);

				if(Activate_CLI_For_First_Time_Flag == 1) Read_In_CLI_Task_Flag = 1;
				Activate_CLI_For_First_Time_Flag = 1;

			}
			else if(portStatus[port_number] == CLI)
			{
				CLI_Data = UARTRxBuf[port_number-1][index_process[port_DMA]];
				Read_In_CLI_Task_Flag = 1;
			}

			else if(UARTRxBuf[port_number-1][index_process[port_DMA]]  == 'H' && portStatus[port_number] == FREE)
			{
				portStatus[port_number] =H_Status; // H  Character was received, waiting for Z character.
			}

			else if(UARTRxBuf[port_number-1][index_process[port_DMA]]  == 'Z' && portStatus[port_number] == H_Status)
			{
				portStatus[port_number] =Z_Status; // Z  Character was received, waiting for length byte.
			}

			else if(UARTRxBuf[port_number-1][index_process[port_DMA]]  != 'Z' && portStatus[port_number] == H_Status)
			{
				portStatus[port_number] =FREE; // Z  Character was not received, so there is no message to receive.
			}

			else if(portStatus[port_number] == Z_Status)
			{
				portStatus[port_number] =MSG; // Receive length byte.
				MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][2] =UARTRxBuf[port_number-1][index_process[port_DMA]] ;
				temp_index[port_index] = 3;
				temp_length[port_index] =UARTRxBuf[port_number-1][index_process[port_DMA]]  + 1;
			}

			else if(portStatus[port_number] == MSG)
			{
				if(temp_length[port_index] > 1)
				{
					MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][temp_index[port_index]] =UARTRxBuf[port_number-1][index_process[port_DMA]] ;
					temp_index[port_index]++;
					temp_length[port_index]--;
				}
				else
				{
					MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][temp_index[port_index]] =UARTRxBuf[port_number-1][index_process[port_DMA]] ;
					temp_index[port_index]++;
					temp_length[port_index]--;
					MSG_Buffer_Index_End[port_index]++;
					if(MSG_Buffer_Index_End[port_index] == MSG_COUNT) MSG_Buffer_Index_End[port_index] = 0;


					Process_Message_Buffer[Process_Message_Buffer_Index_End] = port_number;
					Process_Message_Buffer_Index_End++;
					if(Process_Message_Buffer_Index_End == MSG_COUNT) Process_Message_Buffer_Index_End = 0;
					portStatus[port_number] =FREE; // End of receiving message.
				}
			}
			index_process[port_DMA]++;
			if(index_process[port_DMA]==MSG_RX_BUF_SIZE)
				{index_process[port_DMA]=0;}
		}
		else if(index_input[port_DMA] ==index_process[port_DMA])
		   {
			port_DMA++;
			}

		if(Process_Message_Buffer_Index_End != Process_Message_Buffer_Index_Start)
		{
			port_number = Process_Message_Buffer[Process_Message_Buffer_Index_Start];
			port_index = port_number - 1;
			MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][0] = 'H';
			MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][1] = 'Z';

			length = MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][2];

			Calculate_CRC_Buffer[0] = MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][0];
			Calculate_CRC_Buffer[1] = MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][1];
			Calculate_CRC_Buffer[2] = MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][2];
			for(int i=0;i<length;i++)
			{
				Calculate_CRC_Buffer[i+3] = MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][i + 3];
			}

			calculated_crc = CalculateCRC8(Calculate_CRC_Buffer,length + 3);

			Message_counter++;
			if(calculated_crc == MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][length + 3])
			{
				Accepted_Messages++;
				messageLength[port_index] =length;
				memcpy(&cMessage[port_index][0],&MSG_Buffer[port_index][MSG_Buffer_Index_Start[port_index]][3],length);
				if(cMessage[port_index][0] == myID || cMessage[port_index][0] == BOS_BROADCAST || cMessage[port_index][0] == BOS_MULTICAST)
					/* Notify messaging tasks */
					NotifyMessagingTask(port_number);

				else{
					/* Forward message */
					ForwardReceivedMessage(port_number);
				    }
			}
			else
			{
				Rejected_Messages++;
				//TODO: Implement something here when the message is rejected.
			}

			MSG_Buffer_Index_Start[port_index]++;
			if(MSG_Buffer_Index_Start[port_index] == MSG_COUNT) MSG_Buffer_Index_Start[port_index] = 0;

			Process_Message_Buffer_Index_Start++;
			if(Process_Message_Buffer_Index_Start == MSG_COUNT) Process_Message_Buffer_Index_Start = 0;
		}

		taskYIELD();
	}
}
}

/*-----------------------------------------------------------*/

/* ---------------------------- PxMessagingTask function ---------------------------------  */

void PxMessagingTask(void *argument){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef status =HAL_OK;
	uint8_t port, src, dst, temp, i, p, shift, numOfParams;
	uint16_t code;
	uint32_t count, timeout, temp32;
	bool extendCode = false, extendOptions = false;
	static int8_t cCLIString[cmdMAX_INPUT_SIZE];
	portBASE_TYPE xReturned;
	int8_t *pcOutputString;
	static uint8_t bcastLastID;
	
	port =(int8_t )(unsigned )argument;
	
	/* Infinite loop */
	for(;;){
		
		/* Wait forever until a message is received on one of the ports */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		
		if(messageLength[port - 1]){
			/* Long message? Read Options Byte MSB */
			if(cMessage[port - 1][2] >> 7){
				longMessage =1;
			}
			else{
				longMessage =0;
			}
			
			/* Read message source and destination */
			dst =cMessage[port - 1][0];
			src =cMessage[port - 1][1];
			
			/* Reset array index shift */
			shift =0;
			
			/* Read message options */
			if(cMessage[port - 1][2] & 0x01){ // 1st bit (LSB) Extended options - TODO handle extended options case
				extendOptions = true;
				(void )extendOptions; // remove warning
				++shift;
			}
			extendCode =(cMessage[port - 1][2] >> 1) & 0x01; 					// 2nd bit Extended code
			BOSMessaging.trace =(traceOptions_t )((cMessage[port - 1][2] >> 2) & 0x03);  // 3rd-4th bits Trace
			BOSMessaging.received_Acknowledgment =((cMessage[port - 1][2] >> 4) & 0x01);						    // 5th bit Reserved
			BOSMessaging.response =(cMessage[port - 1][2]) & 0x60; 					    // 6th-7th bits Response mode
			// 8th bit (MSB) Long message
			
			/* Read message code - LSB first */
			if(extendCode == true){
				code =(((uint16_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][3 + shift]);
				++shift;
			}
			else{
				code =cMessage[port - 1][3 + shift];
			}

			/*ACK Massage */
			if(true == BOSMessaging.received_Acknowledgment){
				BOSMessaging.Acknowledgment =false;
				SendMessageToModule(src,MSG_Acknowledgment_Accepted,0);
			}

			/* Is it a transit message? Check for the case when module is being IDed */
			if((dst && (dst < BOS_MULTICAST) && (dst != myID) && (myID != 1)) || (dst && (dst < BOS_MULTICAST) && (dst != myID) && (myID == 1) && (code != CODE_MODULE_ID))){
				/* Forward the message to its destination */
				ForwardReceivedMessage(port);
				if(BOSMessaging.trace)
					indMode =IND_SHORT_BLINK;
				
				/* Special messages that require local action */
				if(code == CODE_UPDATE){ // Remote bootloader update
					Delay_ms(100);
					remoteBootloaderUpdate(src,dst,port,0);
				}
				else if(code == CODE_UPDATE_VIA_PORT){ // Remote 'via port' bootloader update
					Delay_ms(100);
					remoteBootloaderUpdate(src,dst,port,cMessage[port - 1][shift]);
				}
			}
			/* Either broadcast or multicast local message */
			else{
				/* Is it a broadcast message with unique ID? */
				if(dst == BOS_BROADCAST && cMessage[port - 1][messageLength[port - 1] - 1] != bcastLastID){
					bcastID =bcastLastID =cMessage[port - 1][messageLength[port - 1] - 1]; // Store bcastID
					BroadcastReceivedMessage(BOS_BROADCAST,port);
					cMessage[port - 1][messageLength[port - 1] - 1] =0; // Reset bcastID location
					result =BOS_OK;
				}
				/* Reflection of last broadcast message! */
				else if(dst == BOS_BROADCAST && cMessage[port - 1][messageLength[port - 1] - 1] == bcastLastID){
					result =BOS_ERR_MSG_Reflection;
				}
				
				/* Is it a multicast message with unique ID? */
				if(dst == BOS_MULTICAST && cMessage[port - 1][messageLength[port - 1] - 1] != bcastLastID){
					bcastID =bcastLastID =cMessage[port - 1][messageLength[port - 1] - 1]; // Store bcastID
					BroadcastReceivedMessage(BOS_MULTICAST,port);
					cMessage[port - 1][messageLength[port - 1] - 1] =0; // Reset bcastID location
					temp =cMessage[port - 1][messageLength[port - 1] - 2]; // Number of members in this multicast group - TODO breaks when message is 14 length and padded
					/* Am I part of this multicast group? */
					result =BOS_ERR_WrongID;
					for(i =0; i < temp; i++){
						if(myID == cMessage[port - 1][messageLength[port - 1] - 2 - temp + i]){
							result =BOS_OK;
							break;
						}
					}
				}
				/* Reflection of last multi-cast message! */
				else if(dst == BOS_MULTICAST && cMessage[port - 1][messageLength[port - 1] - 1] == bcastLastID){
					result =BOS_ERR_MSG_Reflection;
				}
				
				/* Set shift index to the start of message payload (parameters) */
				shift +=4;
				
				/* Message payload size */
				numOfParams =messageLength[port - 1] - shift;
				
				/* Process BOS Messages payload */
				if(result == BOS_OK){
					switch(code){
						case CODE_UNKNOWN_MESSAGE:
							break;
							
						case CODE_PING:
							indMode =IND_PING;
							osDelay(10);
							if(BOSMessaging.response == BOS_RESPONSE_ALL || BOSMessaging.response == BOS_RESPONSE_MSG)
								SendMessageToModule(src,CODE_PING_RESPONSE,0);
							break;
							
						case CODE_PING_RESPONSE:
							if(!moduleAlias[myID][0])
								sprintf((char* )pcUserMessage,"Hi from module %d\r\n",src);
							else
								sprintf((char* )pcUserMessage,"Hi from module %d (%s)\r\n",src,moduleAlias[src]);
							writePxMutex(PcPort,pcUserMessage,strlen(pcUserMessage),cmd50ms,HAL_MAX_DELAY);
							responseStatus =BOS_OK;
							break;
							
						case CODE_IND_ON:
							IND_ON();
							break;
							
						case CODE_IND_OFF:
							IND_OFF();
							break;
							
						case CODE_IND_TOGGLE:
							IND_toggle();
							break;
							
						case CODE_HI:
							/* Record your neighbor info */
							neighbors[port - 1][0] =((uint16_t )src << 8) + cMessage[port - 1][2 + shift]; /* Neighbor ID + Neighbor own port */
							neighbors[port - 1][1] =((uint16_t )cMessage[port - 1][shift] << 8) + cMessage[port - 1][1 + shift]; /* Neighbor PN */
							/* Send your own info */
							messageParams[1] =(uint8_t )myPN;
							messageParams[0] =(uint8_t )(myPN >> 8);
							messageParams[2] =port;
							osDelay(2);
							/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
							SendMessageFromPort(port,0,0,CODE_HI_RESPONSE,3);
							break;
							
						case CODE_HI_RESPONSE:
							/* Record your neighbor info */
							neighbors[port - 1][0] =((uint16_t )src << 8) + cMessage[port - 1][2 + shift]; /* Neighbor ID + Neighbor own port */
							neighbors[port - 1][1] =((uint16_t )cMessage[port - 1][shift] << 8) + cMessage[port - 1][1 + shift]; /* Neighbor PN */
							responseStatus =BOS_OK;
							break;
							/* Receiving the Defalt_Value for the H1DR5 module */
						case CODE_H1DR5_receive_Defalt_Value:
							defalt_data.Local_mac_addr[0]= cMessage[port - 1][0 + shift];
							defalt_data.Local_mac_addr[1]= cMessage[port - 1][1 + shift];
							defalt_data.Local_mac_addr[2]= cMessage[port - 1][2 + shift];
							defalt_data.Local_mac_addr[3]= cMessage[port - 1][3 + shift];
							defalt_data.Local_mac_addr[4]= cMessage[port - 1][4 + shift];
							defalt_data.Local_mac_addr[5]= cMessage[port - 1][5 + shift];

							defalt_data.Remote_mac_addr[0]= cMessage[port - 1][6 + shift];
							defalt_data.Remote_mac_addr[1]= cMessage[port - 1][7 + shift];
							defalt_data.Remote_mac_addr[2]= cMessage[port - 1][8 + shift];
							defalt_data.Remote_mac_addr[3]= cMessage[port - 1][9 + shift];
							defalt_data.Remote_mac_addr[4]= cMessage[port - 1][10 + shift];
							defalt_data.Remote_mac_addr[5]= cMessage[port - 1][11 + shift];

							defalt_data.Local_IP[0]= cMessage[port - 1][12 + shift];
							defalt_data.Local_IP[1]= cMessage[port - 1][13 + shift];
							defalt_data.Local_IP[2]= cMessage[port - 1][14 + shift];
							defalt_data.Local_IP[3]= cMessage[port - 1][15 + shift];

							defalt_data.Remote_IP[0]= cMessage[port - 1][16 + shift];
							defalt_data.Remote_IP[1]= cMessage[port - 1][17 + shift];
							defalt_data.Remote_IP[2]= cMessage[port - 1][18 + shift];
							defalt_data.Remote_IP[3]= cMessage[port - 1][19 + shift];

							defalt_data.ip_mask[0]= cMessage[port - 1][20 + shift];
							defalt_data.ip_mask[1]= cMessage[port - 1][21 + shift];
							defalt_data.ip_mask[2]= cMessage[port - 1][22 + shift];
							defalt_data.ip_mask[3]= cMessage[port - 1][23 + shift];

							defalt_data.ip_dest[0]= cMessage[port - 1][24 + shift];
							defalt_data.ip_dest[1]= cMessage[port - 1][25 + shift];
							defalt_data.ip_dest[2]= cMessage[port - 1][26 + shift];
							defalt_data.ip_dest[3]= cMessage[port - 1][27 + shift];

							defalt_data.Local_PORT= cMessage[port - 1][28 + shift];
							defalt_data.Remote_PORT= cMessage[port - 1][29 + shift];

							break;

#ifndef __N
						case CODE_EXPLORE_ADJ:
							ExploreNeighbors(port);
							indMode =IND_TOPOLOGY;
							osDelay(10);
							temp =0;
							/* Exploration response message */
							for(uint8_t p =1; p <= NumOfPorts; p++){
								if(neighbors[p - 1][0]){
									messageParams[temp] =p;
									memcpy(messageParams + temp + 1,neighbors[p - 1],(size_t )(4));
									temp +=5;
								}
							}
							SendMessageToModule(src,CODE_EXPLORE_ADJ_RESPONSE,temp);
							break;
							
						case CODE_EXPLORE_ADJ_RESPONSE:
							/* Extract the other module neighbors */
							temp =numOfParams / 5;
							for(uint8_t k =0; k < temp; k++){
								memcpy(&neighbors2[(cMessage[port - 1][shift + k * 5]) - 1][0],&cMessage[port - 1][1 + shift + k * 5],(size_t )(4));
							}
							responseStatus =BOS_OK;
							break;
#endif						
						case CODE_PORT_DIRECTION:
							/* Reverse/un-reverse ports according to command parameters */
							for(uint8_t p =1; p <= NumOfPorts; p++){
								if(p != port)
									SwapUartPins(GetUart(p),cMessage[port - 1][shift + p - 1]);
							}
							/* Check the input port direction */
							SwapUartPins(GetUart(port),cMessage[port - 1][shift + MaxNumOfPorts]);
							break;
							
						case CODE_MODULE_ID:
							if(cMessage[port - 1][shift] == 0) /* Change my own ID */
								myID =cMessage[port - 1][1 + shift];
							else if(cMessage[port - 1][shift] == 1){ /* Change my neighbor's ID */
								messageParams[0] =0; /* change own ID */
								messageParams[1] =cMessage[port - 1][1 + shift]; /* The new ID */
								SendMessageFromPort(cMessage[port - 1][2 + shift],0,0,CODE_MODULE_ID,3);
							}
							break;
							
						case CODE_TOPOLOGY:
							if(longMessage){
								/* array is 2-byte oriented thus memcpy can copy only even number of bytes TODO test maybe broken */
								/* Use a 1-byte oriented scratchpad */
								memcpy(&longMessageScratchpad[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
								longMessageLastPtr +=numOfParams;
							}
							else{
								memcpy(&longMessageScratchpad[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
								longMessageLastPtr +=numOfParams;
								N =(longMessageLastPtr / (MaxNumOfPorts + 1)) / 2;
								/* Copy the scratchpad to array */
								memcpy(&array,&longMessageScratchpad,longMessageLastPtr);
								longMessageLastPtr =0;
//indMode = IND_TOPOLOGY;
							}
							break;
							
						case CODE_READ_PORT_DIR:
							ReadPortsDirMSG(src);
								break;

						case CODE_READ_PORT_DIR_RESPONSE:
							/* Read module ports directions */
							for(p =0; p < numOfParams; p++){
								arrayPortsDir[src - 1] |=(0x8000 >> ((cMessage[port - 1][shift + p]) - 1));
							}
							responseStatus =BOS_OK;
							break;
							
						case CODE_BAUDRATE:
							/* Change baudrate of specified ports */
							temp =temp32 =0;
							temp32 =((uint32_t )cMessage[port - 1][shift] << 24) + ((uint32_t )cMessage[port - 1][1 + shift] << 16) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + cMessage[port - 1][3 + shift];
							if(cMessage[port - 1][4 + shift] == 0xFF) // All ports
							{
								for(p =1; p <= NumOfPorts; p++){
									UpdateBaudrate(p,temp32);
								}
							}
							else{
								for(p =0; p < numOfParams; p++){
									temp =cMessage[port - 1][4 + shift + p];
									if(temp > 0 && temp <= NumOfPorts){
										UpdateBaudrate(temp,temp32);
									}
								}
							}
							break;
							
						case CODE_EXP_EEPROM:
							SaveToRO();
							SaveEEportsDir();
							indMode =IND_PING;
							break;
							
						case CODE_DEF_ARRAY:
							/* Clear the topology */
							ClearEEportsDir();
#ifndef __N
							ClearROtopology();
#endif
							osDelay(100);
							indMode =IND_TOPOLOGY;
							break;
							
						case CODE_CLI_COMMAND:
							/* Obtain the address of the output buffer */
							pcOutputString =FreeRTOS_CLIGetOutputBuffer();
							/* Copy the command */
							if(dst == BOS_BROADCAST)
								memcpy(cCLIString,&cMessage[port - 1][shift],(size_t )(numOfParams - 1)); // remove bcastID
							else if(dst == BOS_MULTICAST)
								memcpy(cCLIString,&cMessage[port - 1][shift],(size_t )(numOfParams - temp - 2)); // remove bcastID + groupm members + group count
							else
								memcpy(cCLIString,&cMessage[port - 1][shift],(size_t )numOfParams);
							do{
								/* Pass the inport to CLI command parsers temporarily through PcPort */
								temp =PcPort;
								PcPort =port;
								/* Process the command locally */
								xReturned =FreeRTOS_CLIProcessCommand(cCLIString,pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
								/* Restore back PcPort */
								PcPort =temp;
								/* Respond to the CLI command */
								if(BOSMessaging.response == BOS_RESPONSE_ALL){
									/* Copy the generated string to messageParams */
									memcpy(messageParams,pcOutputString,strlen((char* )pcOutputString));
									/* Send command response */
									SendMessageToModule(src,CODE_CLI_RESPONSE,strlen((char* )pcOutputString));
									osDelay(10);
								}
							} while(xReturned != pdFALSE);
							/* Reset the buffer */
							memset(cCLIString,0x00,cmdMAX_INPUT_SIZE);
							break;
							
						case CODE_CLI_RESPONSE:
							/* Obtain the address of the output buffer and clear the buffer. */
							pcOutputString =FreeRTOS_CLIGetOutputBuffer();
							memset(pcOutputString,0x00,strlen((char* )pcOutputString));
							/* Copy the response */
							if(longMessage){
								memcpy(&pcOutputString[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
								longMessageLastPtr +=numOfParams;
							}
							else{
								memcpy(&pcOutputString[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
								longMessageLastPtr =0;
								responseStatus =BOS_OK;
								/* Wake up the CliTask again */
								xTaskNotify((xCommandConsoleTaskHandle),0,eNoAction); // Notify the task without modifying its notification value
							}
							break;
							
						case CODE_UPDATE:
							/* Trigger ST factory bootloader update */
							#ifndef STM32G0B1xx
							/* Address for RAM signature (STM32F09x) - Last 4 words of SRAM */
							*((unsigned long* )0x20007FF0) =0xDEADBEEF;
							#else
							/* Address for RAM signature (STM32G0Bx) - Last 4 words of SRAM */
							*((unsigned long* )0x20023FF0) =0xDEADBEEF;
							#endif							
							indMode =IND_PING;
							osDelay(10);
							NVIC_SystemReset();
							break;
							
						case CODE_UPDATE_VIA_PORT:
							/* I'm the last module before target. First, ask the target to jump to factory bootloader */
							SendMessageFromPort(cMessage[port - 1][shift],0,0,CODE_UPDATE,0);
							osDelay(100);
							/* Then, setup myself for remote 'via port' update */
							remoteBootloaderUpdate(src,myID,port,cMessage[port - 1][shift]);
							break;
							
						case CODE_DMA_CHANNEL:
							/* Read EEPROM storage flag */
							temp =cMessage[port - 1][11 + shift];
							if(numOfParams == 15)
								temp =cMessage[port - 1][13 + shift];
							if(numOfParams == 17)
								temp =cMessage[port - 1][15 + shift];
							count =((uint32_t )cMessage[port - 1][shift] << 24) + ((uint32_t )cMessage[port - 1][1 + shift] << 16) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + cMessage[port - 1][3 + shift];
							timeout =((uint32_t )cMessage[port - 1][4 + shift] << 24) + ((uint32_t )cMessage[port - 1][5 + shift] << 16) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + cMessage[port - 1][7 + shift];
							
							/* Activate the stream */
							if(temp == false){
								count =((uint32_t )cMessage[port - 1][shift] << 24) + ((uint32_t )cMessage[port - 1][1 + shift] << 16) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + cMessage[port - 1][3 + shift];
								timeout =((uint32_t )cMessage[port - 1][4 + shift] << 24) + ((uint32_t )cMessage[port - 1][5 + shift] << 16) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + cMessage[port - 1][7 + shift];
								if(cMessage[port - 1][9 + shift] && cMessage[port - 1][10 + shift])
									SetupDMAStreams(cMessage[port - 1][8 + shift],count,timeout,cMessage[port - 1][9 + shift],cMessage[port - 1][10 + shift]);
								if(cMessage[port - 1][11 + shift] && cMessage[port - 1][12 + shift])
									SetupDMAStreams(cMessage[port - 1][8 + shift],count,timeout,cMessage[port - 1][11 + shift],cMessage[port - 1][12 + shift]);
								if(cMessage[port - 1][13 + shift] && cMessage[port - 1][14 + shift])
									SetupDMAStreams(cMessage[port - 1][8 + shift],count,timeout,cMessage[port - 1][13 + shift],cMessage[port - 1][14 + shift]);
							}
							/* Save stream paramters in EEPROM */
							else{
								EE_WriteVariable(_EE_DMA_STREAM_BASE,cMessage[port - 1][8 + shift]); /* Direction */
								EE_WriteVariable(_EE_DMA_STREAM_BASE + 1,((uint16_t )cMessage[port - 1][shift] << 8) + cMessage[port - 1][1 + shift]); /* Count high half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE + 2,((uint16_t )cMessage[port - 1][2 + shift] << 8) + cMessage[port - 1][3 + shift]); /* Count low half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE + 3,((uint16_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift]); /* Timeout high half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE + 4,((uint16_t )cMessage[port - 1][6 + shift] << 8) + cMessage[port - 1][7 + shift]); /* Timeout low half-word */
								EE_WriteVariable(_EE_DMA_STREAM_BASE + 5,((uint16_t )cMessage[port - 1][9 + shift] << 8) + cMessage[port - 1][10 + shift]); /* src1 | dst1 */
								if(numOfParams == 19)
									EE_WriteVariable(_EE_DMA_STREAM_BASE + 6,((uint16_t )cMessage[port - 1][11 + shift] << 8) + cMessage[port - 1][12 + shift]); /* src2 | dst2 */
								if(numOfParams == 21)
									EE_WriteVariable(_EE_DMA_STREAM_BASE + 7,((uint16_t )cMessage[port - 1][13 + shift] << 8) + cMessage[port - 1][14 + shift]); /* src3 | dst3 */
								/* Reset MCU */
								NVIC_SystemReset();
							}
							break;
							
						case CODE_DMA_SCAST_STREAM:
							count =((uint32_t )cMessage[port - 1][shift] << 24) + ((uint32_t )cMessage[port - 1][1 + shift] << 16) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + cMessage[port - 1][3 + shift];
							timeout =((uint32_t )cMessage[port - 1][4 + shift] << 24) + ((uint32_t )cMessage[port - 1][5 + shift] << 16) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + cMessage[port - 1][7 + shift];
							StartScastDMAStream(cMessage[port - 1][9 + shift],myID,cMessage[port - 1][11 + shift],cMessage[port - 1][10 + shift],cMessage[port - 1][8 + shift],count,timeout,cMessage[port - 1][12 + shift]);
							break;
							
						case CODE_READ_REMOTE:
							if(cMessage[port - 1][shift] == REMOTE_MEMORY_ADD) // request for a memory address
							{
// Get requested address
								temp32 =((uint32_t )cMessage[port - 1][2 + shift] << 24) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
// Get variable according to requested format
								switch(cMessage[port - 1][1 + shift]) // requested format
								{
									case FMT_BOOL:
									case FMT_UINT8:
										messageParams[0] =*(__IO uint8_t* )temp32;
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,1);
										break;
									case FMT_INT8:
										messageParams[0] =*(__IO int8_t* )temp32;
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,1);
										break;
									case FMT_UINT16:
										messageParams[0] =(uint8_t )((*(__IO uint16_t* )temp32) >> 0);
										messageParams[1] =(uint8_t )((*(__IO uint16_t* )temp32) >> 8);
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,2);
										break;
									case FMT_INT16:
										messageParams[0] =(uint8_t )((*(__IO int16_t* )temp32) >> 0);
										messageParams[1] =(uint8_t )((*(__IO int16_t* )temp32) >> 8);
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,2);
										break;
									case FMT_UINT32:
										messageParams[0] =(uint8_t )((*(__IO uint32_t* )temp32) >> 0);
										messageParams[1] =(uint8_t )((*(__IO uint32_t* )temp32) >> 8);
										messageParams[2] =(uint8_t )((*(__IO uint32_t* )temp32) >> 16);
										messageParams[3] =(uint8_t )((*(__IO uint32_t* )temp32) >> 24);
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,4);
										break;
									case FMT_INT32:
										messageParams[0] =(uint8_t )((*(__IO int32_t* )temp32) >> 0);
										messageParams[1] =(uint8_t )((*(__IO int32_t* )temp32) >> 8);
										messageParams[2] =(uint8_t )((*(__IO int32_t* )temp32) >> 16);
										messageParams[3] =(uint8_t )((*(__IO int32_t* )temp32) >> 24);
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,4);
										break;
									case FMT_FLOAT:
										messageParams[0] =*(__IO uint8_t* )(temp32 + 0);
										messageParams[1] =*(__IO uint8_t* )(temp32 + 1);
										messageParams[2] =*(__IO uint8_t* )(temp32 + 2);
										messageParams[3] =*(__IO uint8_t* )(temp32 + 3);
										SendMessageToModule(src,
										CODE_READ_REMOTE_RESPONSE,8);
										break; // You cannot bitwise floats
									default:
										break;
								}
							}
							else if(cMessage[port - 1][shift] == REMOTE_MODULE_PARAM) // request for a Module param
							{
								cMessage[port - 1][messageLength[port - 1] - 1] =0; // adding string termination
								temp =IsModuleParameter((char* )&cMessage[port - 1][1 + shift]); // extrating module parameter
								if(temp == 0){ // Parameter does not exist
									SendMessageToModule(src,
									CODE_READ_REMOTE_RESPONSE,1);
								}
								else{
// Parameter exists. Get its pointer
									temp32 =(uint32_t )modParam[temp - 1].paramPtr;
									messageParams[0] =modParam[temp - 1].paramFormat;
// Send parameter according to its format
									switch(messageParams[0]) // requested format
									{
										case FMT_BOOL:
										case FMT_UINT8:
											messageParams[1] =*(__IO uint8_t* )temp32;
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,2);
											break;
										case FMT_INT8:
											messageParams[1] =*(__IO int8_t* )temp32;
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,2);
											break;
										case FMT_UINT16:
											messageParams[1] =(uint8_t )((*(__IO uint16_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO uint16_t* )temp32) >> 8);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,3);
											break;
										case FMT_INT16:
											messageParams[1] =(uint8_t )((*(__IO int16_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO int16_t* )temp32) >> 8);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,3);
											break;
										case FMT_UINT32:
											messageParams[1] =(uint8_t )((*(__IO uint32_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO uint32_t* )temp32) >> 8);
											messageParams[3] =(uint8_t )((*(__IO uint32_t* )temp32) >> 16);
											messageParams[4] =(uint8_t )((*(__IO uint32_t* )temp32) >> 24);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,5);
											break;
										case FMT_INT32:
											messageParams[1] =(uint8_t )((*(__IO int32_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO int32_t* )temp32) >> 8);
											messageParams[3] =(uint8_t )((*(__IO int32_t* )temp32) >> 16);
											messageParams[4] =(uint8_t )((*(__IO int32_t* )temp32) >> 24);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,5);
											break;
										case FMT_FLOAT:
											messageParams[1] =*(__IO uint8_t* )(temp32 + 0);
											messageParams[2] =*(__IO uint8_t* )(temp32 + 1);
											messageParams[3] =*(__IO uint8_t* )(temp32 + 2);
											messageParams[4] =*(__IO uint8_t* )(temp32 + 3); // You cannot bitwise floats
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,9);
											break;
										default:
											break;
									}
								}
							}
							else if(cMessage[port - 1][shift] >= REMOTE_BOS_VAR) // request for a BOS var
							{
								messageParams[0] =BOS_var_reg[cMessage[port - 1][shift] - REMOTE_BOS_VAR - 1] & 0x000F; // send variable format (lower 4 bits)
								if(messageParams[0] == 0){ // Variable does not exist
									SendMessageToModule(src,
									CODE_READ_REMOTE_RESPONSE,1);
								}
								else{
// Variable exists. Get its memory address
									temp32 =(BOS_var_reg[cMessage[port - 1][shift] - REMOTE_BOS_VAR - 1] >> 16) + SRAM_BASE;
// Send variable according to its format
									switch(messageParams[0]) // requested format
									{
										case FMT_BOOL:
										case FMT_UINT8:
											messageParams[1] =*(__IO uint8_t* )temp32;
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,2);
											break;
										case FMT_INT8:
											messageParams[1] =*(__IO int8_t* )temp32;
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,2);
											break;
										case FMT_UINT16:
											messageParams[1] =(uint8_t )((*(__IO uint16_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO uint16_t* )temp32) >> 8);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,3);
											break;
										case FMT_INT16:
											messageParams[1] =(uint8_t )((*(__IO int16_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO int16_t* )temp32) >> 8);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,3);
											break;
										case FMT_UINT32:
											messageParams[1] =(uint8_t )((*(__IO uint32_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO uint32_t* )temp32) >> 8);
											messageParams[3] =(uint8_t )((*(__IO uint32_t* )temp32) >> 16);
											messageParams[4] =(uint8_t )((*(__IO uint32_t* )temp32) >> 24);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,5);
											break;
										case FMT_INT32:
											messageParams[1] =(uint8_t )((*(__IO int32_t* )temp32) >> 0);
											messageParams[2] =(uint8_t )((*(__IO int32_t* )temp32) >> 8);
											messageParams[3] =(uint8_t )((*(__IO int32_t* )temp32) >> 16);
											messageParams[4] =(uint8_t )((*(__IO int32_t* )temp32) >> 24);
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,5);
											break;
										case FMT_FLOAT:
											messageParams[1] =*(__IO uint8_t* )(temp32 + 0);
											messageParams[2] =*(__IO uint8_t* )(temp32 + 1);
											messageParams[3] =*(__IO uint8_t* )(temp32 + 2);
											messageParams[4] =*(__IO uint8_t* )(temp32 + 3); // You cannot bitwise floats
											SendMessageToModule(src,
											CODE_READ_REMOTE_RESPONSE,9);
											break;
										default:
											break;
									}
								}
							}
							
							break;
							
						case CODE_READ_REMOTE_RESPONSE:
							if(remoteBuffer == REMOTE_BOS_VAR || remoteBuffer == REMOTE_MODULE_PARAM) // We requested a BOS variable or module param
							{
// Read variable according to its format
								remoteVarFormat =(varFormat_t )cMessage[port - 1][shift];
								switch(cMessage[port - 1][shift]) // Remote format
								{// Note that cMessage[port-1][5+shift] can be unaligned. That's why we cannot use simple memory access
									case 0: // This variable does not exist
										responseStatus =BOS_ERR_REMOTE_READ_NO_VAR;
										break;
									case FMT_BOOL:
									case FMT_UINT8:
										remoteBuffer =cMessage[port - 1][1 + shift];
										break;
									case FMT_INT8:
										remoteBuffer =(int8_t )cMessage[port - 1][1 + shift];
										break;
									case FMT_UINT16:
										remoteBuffer =((uint16_t )cMessage[port - 1][1 + shift] << 0) + ((uint16_t )cMessage[port - 1][2 + shift] << 8);
										break;
									case FMT_INT16:
										remoteBuffer =((int16_t )cMessage[port - 1][1 + shift] << 0) + ((int16_t )cMessage[port - 1][2 + shift] << 8);
										break;
									case FMT_UINT32:
										remoteBuffer =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
										break;
									case FMT_INT32:
										remoteBuffer =((int32_t )cMessage[port - 1][1 + shift] << 0) + ((int32_t )cMessage[port - 1][2 + shift] << 8) + ((int32_t )cMessage[port - 1][3 + shift] << 16) + ((int32_t )cMessage[port - 1][4 + shift] << 24);
										break;
									case FMT_FLOAT:
										remoteBuffer =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
										break;
									default:
										break;
								}
							}
							else if(remoteBuffer == REMOTE_MEMORY_ADD) // We requested a memory location
							{
// Read variable according to requested format
								switch(remoteBuffer) // Requested format
								{// Note that cMessage[port-1][shift] can be unaligned. That's why we cannot use simple memory access
									case FMT_BOOL:
									case FMT_UINT8:
										remoteBuffer =cMessage[port - 1][shift];
										break;
									case FMT_INT8:
										remoteBuffer =(int8_t )cMessage[port - 1][shift];
										break;
									case FMT_UINT16:
										remoteBuffer =((uint16_t )cMessage[port - 1][shift] << 0) + ((uint16_t )cMessage[port - 1][1 + shift] << 8);
										break;
									case FMT_INT16:
										remoteBuffer =((int16_t )cMessage[port - 1][shift] << 0) + ((int16_t )cMessage[port - 1][1 + shift] << 8);
										break;
									case FMT_UINT32:
										remoteBuffer =((uint32_t )cMessage[port - 1][shift] << 0) + ((uint32_t )cMessage[port - 1][1 + shift] << 8) + ((uint32_t )cMessage[port - 1][2 + shift] << 16) + ((uint32_t )cMessage[port - 1][3 + shift] << 24);
										break;
									case FMT_INT32:
										remoteBuffer =((int32_t )cMessage[port - 1][shift] << 0) + ((int32_t )cMessage[port - 1][1 + shift] << 8) + ((int32_t )cMessage[port - 1][2 + shift] << 16) + ((int32_t )cMessage[port - 1][3 + shift] << 24);
										break;
									case FMT_FLOAT:
										remoteBuffer =((uint32_t )cMessage[port - 1][shift] << 0) + ((uint32_t )cMessage[port - 1][1 + shift] << 8) + ((uint32_t )cMessage[port - 1][2 + shift] << 16) + ((uint32_t )cMessage[port - 1][3 + shift] << 24);
										break;
									default:
										break;
								}
							}
							else{
							}
// Remote read status
							if(responseStatus != BOS_ERR_REMOTE_READ_NO_VAR)
								responseStatus =BOS_OK;
							break;
							
						case CODE_WRITE_REMOTE:
						case CODE_WRITE_REMOTE_FORCE:

							responseStatus =BOS_OK; // Initialize response
							if(cMessage[port - 1][shift]) // request for a BOS var
							{
// Check variable index is within the limit of MAX_BOS_VARS
								if(cMessage[port - 1][shift] <= MAX_BOS_VARS){
									temp32 =(BOS_var_reg[cMessage[port - 1][shift] - 1] >> 16) + SRAM_BASE+0x10000; // Get var memory addres
// Modify the variable or create a new one if it does not exist
									switch(cMessage[port - 1][1 + shift]) // requested format
									{
										case FMT_BOOL:
										case FMT_UINT8:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(uint8_t)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) // Write remote value
												*(__IO uint8_t* )temp32 =cMessage[port - 1][2 + shift];
											break;
											
										case FMT_INT8:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(int8_t)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) // Write remote value
												*(__IO int8_t* )temp32 =(int8_t )cMessage[port - 1][2 + shift];
											break;
											
										case FMT_UINT16:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(uint16_t)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) // Write remote value
												*(__IO uint16_t* )temp32 =((uint16_t )cMessage[port - 1][2 + shift] << 0) + ((uint16_t )cMessage[port - 1][3 + shift] << 8);
											break;
											
										case FMT_INT16:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(int16_t)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) // Write remote value
												*(__IO int16_t* )temp32 =((int16_t )cMessage[port - 1][2 + shift] << 0) + ((int16_t )cMessage[port - 1][3 + shift] << 8);
											break;
											
										case FMT_UINT32:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(uint32_t)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) // Write remote value
												*(__IO uint32_t* )temp32 =((uint32_t )cMessage[port - 1][2 + shift] << 0) + ((uint32_t )cMessage[port - 1][3 + shift] << 8) + ((uint32_t )cMessage[port - 1][4 + shift] << 16) + ((uint32_t )cMessage[port - 1][5 + shift] << 24);
											break;
											
										case FMT_INT32:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(int32_t)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL) // Write remote value
												*(__IO int32_t* )temp32 =((int32_t )cMessage[port - 1][2 + shift] << 0) + ((int32_t )cMessage[port - 1][3 + shift] << 8) + ((int32_t )cMessage[port - 1][4 + shift] << 16) + ((int32_t )cMessage[port - 1][5 + shift] << 24);
											break;
											
										case FMT_FLOAT:
											if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) == 0){ // Variable does not exist
												temp32 =(uint32_t )malloc(sizeof(float)); // Create a new one
												if(temp32 != 0){
													BOS_var_reg[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
												}
												else{ // Cannot alocate memory
													responseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
												}
											}
											if(responseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL){ // Write remote value
												remoteBuffer =((uint32_t )cMessage[port - 1][2 + shift] << 0) + ((uint32_t )cMessage[port - 1][3 + shift] << 8) + ((uint32_t )cMessage[port - 1][4 + shift] << 16) + ((uint32_t )cMessage[port - 1][5 + shift] << 24);
												*(float* )temp32 =*(float* )&remoteBuffer;
											}
											break;
											
										default:
											break;
									}
									
									/* Update local format if needed - Todo give local warning later */
									if((BOS_var_reg[cMessage[port - 1][shift] - 1] & 0x000F) != cMessage[port - 1][1 + shift]){
										BOS_var_reg[cMessage[port - 1][shift] - 1] &=(0xFFF0 + cMessage[port - 1][1 + shift]);
										responseStatus =BOS_ERR_LOCAL_FORMAT_UPDATED;
									}
								}
								else{
									responseStatus =BOS_ERR_REMOTE_WRITE_INDEX; // BOS var index out of range
								}
							}
							else // request for a memory address
							{
								// Get the requested address
								temp32 =((uint32_t )cMessage[port - 1][2 + shift] << 24) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
								// Write data to Flash or SRAM based on requested format
								if(temp32 >= SRAM_BASE && temp32 < (SRAM_BASE + SRAM_SIZE)) // SRAM
								{
									switch(cMessage[port - 1][1 + shift]) // Requested format
									{
										case FMT_BOOL:
										case FMT_UINT8:
											*(__IO uint8_t* )temp32 =cMessage[port - 1][6 + shift];
											break;
										case FMT_INT8:
											*(__IO int8_t* )temp32 =(int8_t )cMessage[port - 1][6 + shift];
											break;
										case FMT_UINT16:
											*(__IO uint16_t* )temp32 =((uint16_t )cMessage[port - 1][6 + shift] << 0) + ((uint16_t )cMessage[port - 1][7 + shift] << 8);
											break;
										case FMT_INT16:
											*(__IO int16_t* )temp32 =((int16_t )cMessage[port - 1][6 + shift] << 0) + ((int16_t )cMessage[port - 1][7 + shift] << 8);
											break;
										case FMT_UINT32:
											*(__IO uint32_t* )temp32 =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);
											break;
										case FMT_INT32:
											*(__IO int32_t* )temp32 =((int32_t )cMessage[port - 1][6 + shift] << 0) + ((int32_t )cMessage[port - 1][7 + shift] << 8) + ((int32_t )cMessage[port - 1][8 + shift] << 16) + ((int32_t )cMessage[port - 1][9 + shift] << 24);
											break;
										case FMT_FLOAT:
											remoteBuffer =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);
											*(float* )temp32 =*(float* )&remoteBuffer;
											break;
										default:
											break;
									}
								}
								else if(temp32 >= FLASH_BASE && temp32 < (FLASH_BASE + FLASH_SIZE)) // Flash
								{
									HAL_FLASH_Unlock();
									/* Erase page if force write is requested */
									if(code == CODE_WRITE_REMOTE_FORCE){
										EraseSector(temp32);
									}
									/* Write new value */
									if(responseStatus == BOS_OK){
										switch(cMessage[port - 1][1 + shift]) // Requested format
										{
											case FMT_BOOL:
											case FMT_UINT8:
											case FMT_INT8:
												if(*(__IO uint16_t* )temp32 != 0xFFFF){
													responseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
													break;
												}
												else{
													remoteBuffer =cMessage[port - 1][6 + shift];
													// TOCHECKLATER
													// available values in G0 MCU:
													//TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
													//TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
													#ifndef STM32G0B1xx
													status =HAL_FLASH_Program(
													FLASH_TYPEPROGRAM_HALFWORD,temp32,remoteBuffer);
													#endif										
													break;
												}
											case FMT_UINT16:
											case FMT_INT16:
												if(*(__IO uint16_t* )temp32 != 0xFFFF){
													responseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
													break;
												}
												else{
													remoteBuffer =((uint16_t )cMessage[port - 1][6 + shift] << 0) + ((uint16_t )cMessage[port - 1][7 + shift] << 8);
													// TOCHECKLATER
													// available values in G0 MCU:
													//TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
													//TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
													#ifndef STM32G0B1xx
													status =HAL_FLASH_Program(
													FLASH_TYPEPROGRAM_HALFWORD,temp32,remoteBuffer);		
													#endif
													break;
												}
											case FMT_UINT32:
											case FMT_INT32:
												if(*(__IO uint32_t* )temp32 != 0xFFFFFFFF){
													responseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
													break;
												}
												else{
													remoteBuffer =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);													// TOCHECKLATER
													// TOCHECKLATER
													// available values in G0 MCU:
													//TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
													//TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
													#ifndef STM32G0B1xx
													status =HAL_FLASH_Program(
												    FLASH_TYPEPROGRAM_WORD,temp32,remoteBuffer);
													#endif
													break;
												}
											case FMT_FLOAT:
												if(*(__IO uint32_t* )temp32 != 0xFFFFFFFF){
													responseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
													break;
												}
												else{
													remoteBuffer =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);
													status =HAL_FLASH_Program(
													FLASH_TYPEPROGRAM_DOUBLEWORD,temp32,remoteBuffer);
													break;
												}
											default:
												break;
										}
									}
									HAL_FLASH_Lock();
									if(status != HAL_OK)
										responseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
								}
								else
									responseStatus =BOS_ERR_REMOTE_WRITE_ADDRESS;
							}
							
							/* Send confirmation back */
							if(BOSMessaging.response == BOS_RESPONSE_ALL || BOSMessaging.response == BOS_RESPONSE_MSG){
								messageParams[0] =responseStatus;
								SendMessageToModule(src,CODE_WRITE_REMOTE_RESPONSE,1);
							}
							break;
							
						case CODE_WRITE_REMOTE_RESPONSE:
							responseStatus =(BOS_Status )cMessage[port - 1][shift];
							break;
							
						case CODE_PORT_FORWARD:
							writePxMutex(cMessage[port - 1][shift],(char* )&cMessage[port - 1][shift + 1],numOfParams - 1,10,10);
							break;
							
						case CODE_READ_REMOTE_ModBus_RESPONSE:
							switch(cMessage[port - 1][0 + shift]){
								case 0:
									MBmessageParams[0] =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
									MBmessageParams[1] =((uint32_t )cMessage[port - 1][5 + shift] << 0) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + ((uint32_t )cMessage[port - 1][7 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 24);
									MBmessageParams[2] =((uint32_t )cMessage[port - 1][9 + shift] << 0) + ((uint32_t )cMessage[port - 1][10 + shift] << 8) + ((uint32_t )cMessage[port - 1][11 + shift] << 16) + ((uint32_t )cMessage[port - 1][12 + shift] << 24);
									break;
									
								case 1:
									MBmessageParams[3] =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
									MBmessageParams[4] =((uint32_t )cMessage[port - 1][5 + shift] << 0) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + ((uint32_t )cMessage[port - 1][7 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 24);
									MBmessageParams[5] =((uint32_t )cMessage[port - 1][9 + shift] << 0) + ((uint32_t )cMessage[port - 1][10 + shift] << 8) + ((uint32_t )cMessage[port - 1][11 + shift] << 16) + ((uint32_t )cMessage[port - 1][12 + shift] << 24);
									break;
									
								case 2:
									MBmessageParams[6] =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);

							}
						case CODE_READ_ADC_VALUE:
							ADCPort =cMessage[port - 1][shift];
							ADCSide =cMessage[port - 1][shift + 1];
							if(0 == ADCSide){
								ADCSelectChannel(ADCPort,"top");
								ReadADCChannel(ADCPort,"top",&ADCValue);
							}
							else if(1 == ADCSide){
								ADCSelectChannel(ADCPort,"bottom");
								ReadADCChannel(ADCPort,"bottom",&ADCValue);
							}
							
						case CODE_READ_TEMPERATURE:
						case CODE_READ_VREF:
							ReadTempAndVref(&InternalTemperature,&InternalVoltageReferance);

						case CODE_READ_ADC_PERCENTAGE:
							ADCPort =cMessage[port - 1][shift];
							GetReadPrecentage(ADCPort,&ADCPercentage);
							MBmessageParams[7] =((uint32_t )cMessage[port - 1][5 + shift] << 0) + ((uint32_t )cMessage[port - 1][6 + shift] << 8) + ((uint32_t )cMessage[port - 1][7 + shift] << 16) + ((uint32_t )cMessage[port - 1][8 + shift] << 24);
							MBmessageParams[8] =((uint32_t )cMessage[port - 1][9 + shift] << 0) + ((uint32_t )cMessage[port - 1][10 + shift] << 8) + ((uint32_t )cMessage[port - 1][11 + shift] << 16) + ((uint32_t )cMessage[port - 1][12 + shift] << 24);
							break;
						case MSG_Acknowledgment_Accepted:
							ACK_FLAG =1;
							break;

						case MSG_rejected:
							rejected_FLAG =1;
							break;
						case CODE_READ_RESPONSE:
												switch (cMessage[port - 1][shift]) {
										     	case 0:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													responseStatus = BOS_ERR_REMOTE_READ_NO_VAR;
											        }  else result =BOS_ERROR ;
													break;
												case FMT_BOOL:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.Databool= cMessage[port - 1][2 + shift];
											        }  else result =BOS_ERROR ;
													break;
												case FMT_UINT8:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.DataU8[0]= cMessage[port - 1][2 + shift];
													RemoteDataBuffer.DataU8[1]= cMessage[port - 1][3 + shift];
													RemoteDataBuffer.DataU8[2]= cMessage[port - 1][4 + shift];
											        }  else result =BOS_ERROR ;
													break;
												case FMT_INT8:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.Data8 = (int8_t) cMessage[port - 1][2 + shift];
											        }  else result =BOS_ERROR ;
													break;
												case FMT_UINT16:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.DataU16[0] = ((uint16_t) cMessage[port - 1][2+ shift] << 0)+ ((uint16_t) cMessage[port - 1][3 + shift]<< 8);
													RemoteDataBuffer.DataU16[1] = ((uint16_t) cMessage[port - 1][4+ shift] << 0)+ ((uint16_t) cMessage[port - 1][5 + shift]<< 8);
													RemoteDataBuffer.DataU16[2] = ((uint16_t) cMessage[port - 1][6+ shift] << 0)+ ((uint16_t) cMessage[port - 1][7 + shift]<< 8);

											        }  else result =BOS_ERROR ;
													break;
												case FMT_INT16:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.Data16 = ((int16_t) cMessage[port - 1][2+ shift] << 0)+ ((int16_t) cMessage[port - 1][3 + shift]<< 8);
											        }  else result =BOS_ERROR ;
													break;
												case FMT_UINT32:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.DataU32 = ((uint32_t) cMessage[port - 1][2	+ shift] << 0)	+ ((uint32_t) cMessage[port - 1][3 + shift]	<< 8)+ ((uint32_t) cMessage[port - 1][4 + shift]<< 16)+ ((uint32_t) cMessage[port - 1][5 + shift]<< 24);
											        }  else result =BOS_ERROR ;
													break;
												case FMT_INT32:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
											        {	result =BOS_OK ;
													RemoteDataBuffer.Data32 = ((int32_t) cMessage[port - 1][2+ shift] << 0)+ ((int32_t) cMessage[port - 1][3 + shift]<< 8)+ ((int32_t) cMessage[port - 1][4 + shift]<< 16)+ ((int32_t) cMessage[port - 1][5 + shift]<< 24);
											        }  else result =BOS_ERROR ;
													break;
												case FMT_FLOAT:
											        if( BOS_OK == cMessage[port - 1][1+ shift])
									   		        {	result =BOS_OK ;
													uint32_t temp = ((uint32_t) cMessage[port - 1][2+ shift] << 0)| ((uint32_t) cMessage[port - 1][3 + shift]<< 8)| ((uint32_t) cMessage[port - 1][4 + shift]<< 16)| ((uint32_t) cMessage[port - 1][5 + shift]<< 24);
													RemoteDataBuffer.DataFloat[0] = *((float*)&temp);
													temp=0;
													temp = ((uint32_t) cMessage[port - 1][6+ shift] << 0)| ((uint32_t) cMessage[port - 1][7 + shift]<< 8)| ((uint32_t) cMessage[port - 1][8 + shift]<< 16)| ((uint32_t) cMessage[port - 1][9 + shift]<< 24);
													RemoteDataBuffer.DataFloat[1] = *((float*)&temp);
													temp=0;
												    temp = ((uint32_t) cMessage[port - 1][10+ shift] << 0)| ((uint32_t) cMessage[port - 1][11 + shift]<< 8)| ((uint32_t) cMessage[port - 1][12 + shift]<< 16)| ((uint32_t) cMessage[port - 1][13 + shift]<< 24);
												    RemoteDataBuffer.DataFloat[2] = *((float*)&temp);
													temp=0;
													temp = ((uint32_t) cMessage[port - 1][14+ shift] << 0)| ((uint32_t) cMessage[port - 1][15 + shift]<< 8)| ((uint32_t) cMessage[port - 1][16 + shift]<< 16)| ((uint32_t) cMessage[port - 1][17 + shift]<< 24);
													RemoteDataBuffer.DataFloat[3] = *((float*)&temp);
													temp=0;
											        }  else result =BOS_ERROR ;
													break;
												default:
													break;
												}
													break;

						default:
							/* First check user-defined messages */
							result =(BOS_Status )User_MessagingParser(code,port,src,dst,shift);
							/* If not found, then check module messages */
							if(result == BOS_ERR_UnknownMessage){
								result =(BOS_Status )Module_MessagingTask(code,port,src,dst,shift);
							}
							break;
					}
				}
			}
		}
		
		/* Is it unknown message? */
		if(result == BOS_ERR_UnknownMessage){
			SendMessageToModule(src,CODE_UNKNOWN_MESSAGE,0);
			result =BOS_OK;
		}
		
		/* Reset message buffer */
		memset(cMessage[port - 1],0,(size_t )messageLength[port - 1]);
		messageLength[port - 1] =0;
		if(portStatus[port] != STREAM && portStatus[port] != CLI && portStatus[port] != PORTBUTTON){
			/* Free the port */
			portStatus[port] =FREE;
		}

		taskYIELD();
	}
	
}

/*-----------------------------------------------------------*/

/* --- User message parser. 
 This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak BOS_Status User_MessagingParser(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift)

{
	BOS_Status result =BOS_ERR_UnknownMessage;
	
	return result;
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/

