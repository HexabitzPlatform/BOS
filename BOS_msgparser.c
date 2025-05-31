/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_msgparser.c
 Description   : Source code for Bitz messaging parser.
 
 */


/* Includes ****************************************************************/
#include "BOS.h"

/***************************************************************************/
/* Private and global variables ********************************************/
/***************************************************************************/

/* BackEndTask global variables ********************************************/
uint8_t crcCalculateBuffer[MSG_MAX_SIZE];
uint8_t cliFirstTimeActivation =0;
uint8_t MessageIndexStart[NUM_OF_PORTS] ={0};
uint8_t MessageIndexEnd[NUM_OF_PORTS] ={0};
uint8_t MessageBuffer[NUM_OF_PORTS][MSG_COUNT][MSG_MAX_SIZE] ={0};
uint8_t ProcessMessageBuffer[MSG_COUNT] ={0};
uint8_t ProcessMessageIndexStart =0;
uint8_t ProcessMessageIndexEnd =0;
volatile uint8_t bcastLastID = 0;
uint8_t IndexProcess[NUM_OF_PORTS] ={0};
uint8_t IndexInput[NUM_OF_PORTS] ={0};
uint8_t cliData =0;
uint8_t dmaPortIndex =0;

uint16_t AcceptedMessages =0;
uint16_t RejectedMessages =0;
uint16_t MessageCounter =0;

/* PxMsgTaskHandle global variables ****************************************/
uint8_t PortSelect =0;
uint8_t PinSelect =0;
uint16_t adcPort =0;
uint16_t adcSide =0;
float adcValue =0;
float adcPercentage =0;
float InternalTemperature =0;
float InternalVoltageReferance =0;

receive_defalt_value EthernetDefaultSetting;  /* Receiving the Default setting of the H1DR5 module */
RemoteDataBuffer_t RemoteDataBuffer;          /* Remote Buffer of Messages */

/***************************************************************************/
/* Exported variables ******************************************************/
/***************************************************************************/
extern uint8_t ExtraPcPort;
extern volatile uint8_t RemoteResponseFlag;
extern volatile uint8_t NumOfElement;
extern volatile uint32_t RemoteResponseBuffer[4];
extern VariableFormat_t RemoteVarFormat;

/* Exported Messaging tasks handles ****************************************/
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

/***************************************************************************/
/* Exported Functions ******************************************************/
/***************************************************************************/
extern uint8_t SaveTopologyToRO(void);
#ifndef __N
extern uint8_t ClearROtopology(void);
#endif
extern BOS_Status SaveEEportsDir(void);
extern BOS_Status ClearEEportsDir(void);
extern BOS_Status ForwardReceivedMessage(uint8_t IncomingPort);
extern BOS_Status BroadcastReceivedMessage(uint8_t dstType,uint8_t IncomingPort);
extern BOS_Status SetupDMAStreams(uint8_t direction,uint32_t count,uint32_t timeout,uint8_t src,uint8_t dst);
extern void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* Module exported internal functions */
extern Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);
extern uint8_t IsModuleParameter(char *name);

/* BOS exported internal functions */
extern void CheckAttachedButtons(void);
extern void ResetAttachedButtonStates(uint8_t *deferReset);
extern BOS_Status ExecuteSnippet(void);
extern void NotifyMessagingTask(uint8_t port);


/***************************************************************************/
/* Private function prototypes *********************************************/
/***************************************************************************/
BOS_Status User_MessagingParser(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);


/***************************************************************************/
/*****************************  Private Functions **************************/
/***************************************************************************/

/***************************************************************************/
/* BackEndTask function ****************************************************/
/***************************************************************************/
void BackEndTask(void *argument) {

	BOS_Status result =BOS_OK;

	uint8_t calculated_crc, port_number, length, port_index , dst;
	uint8_t temp_length[NUM_OF_PORTS] = { 0 };
	uint8_t temp_index[NUM_OF_PORTS] = { 0 };
	uint8_t NumofModulesinGroup = 0;

	for (;;) {

        /* Wait for notification from USART interrupt handler */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		/* Parsing all module ports */
		for (dmaPortIndex = 0; dmaPortIndex < NUM_OF_PORTS;) {
			/* Computes how many new bytes have been received on each port: */
			port_index = dmaPortIndex;
			IndexInput[dmaPortIndex] = MSG_RX_BUF_SIZE - (*dmaIndex[dmaPortIndex]);

			/***************************************************************************************/
			/* 1- Check if there's new data to process *********************************************/
			/***************************************************************************************/
			if (IndexInput[dmaPortIndex] != IndexProcess[dmaPortIndex]) {
				port_number = dmaPortIndex + 1;

				/* CLI Handling: If the received byte is 0x0D and the port is free,
				 * it assigns the port to the CLI.*/
				if (UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]] == 0x0D && PortStatus[port_number] == FREE) {
					for (int i = 0; i <= NUM_OF_PORTS; i++) { // Free previous CLI port
						if (PortStatus[i] == CLI)
							PortStatus[i] = FREE;

					}
					/* Continue the CLI session on this port */
					PortStatus[port_number] = CLI;
					pcPort = port_number;
					ExtraPcPort = port_number;

					cliData = UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]];

					xTaskNotifyGive(xCommandConsoleTaskHandle);

					if (cliFirstTimeActivation == 1)
						cliDataInputFlag = 1;

					cliFirstTimeActivation = 1;

				}
				/* Continue processing CLI data if the port is already in CLI mode */
				else if (PortStatus[port_number] == CLI) {
					cliData = UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]];
					cliDataInputFlag = 1;
				}

				/* Hexabitz Protocol Handling (H and Z Characters): */
				else if (UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]] == 'H' && PortStatus[port_number] == FREE) {
					PortStatus[port_number] = H_Status; // H  Character was received, waiting for Z character.
				}

				else if (UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]] == 'Z' && PortStatus[port_number] == H_Status) {
					PortStatus[port_number] = Z_Status; // Z  Character was received, waiting for length byte.
				}

				else if (UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]] != 'Z' && PortStatus[port_number] == H_Status) {
					PortStatus[port_number] = FREE; // Z  Character was not received, so there is no message to receive.
				}

				/* If a length byte is received after 'Z',
				 * it prepares to receive the message content. */
				else if (PortStatus[port_number] == Z_Status) {
					PortStatus[port_number] = MSG; // Receive length byte.
					MessageBuffer[port_index][MessageIndexEnd[port_index]][2] = UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]];
					temp_index[port_index] = 3;
					temp_length[port_index] = UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]] + 1;
				}

				/* Message Reception Handling: */
				else if (PortStatus[port_number] == MSG) {
					/* The message is received and stored in the MessageBuffer.*/
					if (temp_length[port_index] > 1) {
						MessageBuffer[port_index][MessageIndexEnd[port_index]][temp_index[port_index]] = UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]];
						temp_index[port_index]++;
						temp_length[port_index]--;
					} else {
						/* If there is only one byte left to receive (when the message is fully received)
						 * it updates indices and processes the message. */
						MessageBuffer[port_index][MessageIndexEnd[port_index]][temp_index[port_index]] = UARTRxBuf[port_number - 1][IndexProcess[dmaPortIndex]];
						temp_index[port_index]++;
						temp_length[port_index]--;
						MessageIndexEnd[port_index]++;
						if (MessageIndexEnd[port_index] == MSG_COUNT)
							MessageIndexEnd[port_index] = 0;

						ProcessMessageBuffer[ProcessMessageIndexEnd] = port_number;
						ProcessMessageIndexEnd++;
						if (ProcessMessageIndexEnd == MSG_COUNT)
							ProcessMessageIndexEnd = 0;

						/* The PortStatus is set to FREE(End of receiving message)
						 * indicating that the port is ready to receive a new message. */
						PortStatus[port_number] = FREE;
					}
				}

				/* After processing each byte, update the processing index */
				IndexProcess[dmaPortIndex]++;
				if (IndexProcess[dmaPortIndex] == MSG_RX_BUF_SIZE)
					IndexProcess[dmaPortIndex] = 0;

			}

			/***************************************************************************************/
			/* 2- In case there is no bytes to process *********************************************/
			/***************************************************************************************/
			/* Increase the DMA port index to parse all Module ports */
			else if (IndexInput[dmaPortIndex] == IndexProcess[dmaPortIndex]) {
				dmaPortIndex++;
			}

			/***************************************************************************************/
			/* 3- Message Processing ***************************************************************/
			/***************************************************************************************/
			if (ProcessMessageIndexEnd != ProcessMessageIndexStart) {
				port_number = ProcessMessageBuffer[ProcessMessageIndexStart];
				port_index = port_number - 1;
				MessageBuffer[port_index][MessageIndexStart[port_index]][0] = 'H';
				MessageBuffer[port_index][MessageIndexStart[port_index]][1] = 'Z';

				length = MessageBuffer[port_index][MessageIndexStart[port_index]][2];
				dst = MessageBuffer[port_index][MessageIndexStart[port_index]][3];

				/* Forward Message in these cases: wrong ID , dst ~= 0 (explore) ,not MULTICAST , not BROADCAST */
				if ((dst != myID) && (dst != 0) && (dst != BOS_BROADCAST) && (dst != BOS_MULTICAST)) {
					MessageLength[port_index] = length;
					memcpy(&cMessage[port_index][0], &MessageBuffer[port_index][MessageIndexStart[port_index]][3],length);

					/* in case trace feature is enabled: */
					OptionByte.Trace = ((cMessage[port_number - 1][2] >> 2) & 0x01);
					if(OptionByte.Trace)
						IndicatorMode =IND_SHORT_BLINK;

					ForwardReceivedMessage(port_number);

				} else {
					/* Notify Messaging Tasks if Message is for Current Module:
					 * Prepare CRC Buffer and Calculate CRC. */
					crcCalculateBuffer[0] = MessageBuffer[port_index][MessageIndexStart[port_index]][0];
					crcCalculateBuffer[1] = MessageBuffer[port_index][MessageIndexStart[port_index]][1];
					crcCalculateBuffer[2] = MessageBuffer[port_index][MessageIndexStart[port_index]][2];
					for (int i = 0; i < length; i++) {
						crcCalculateBuffer[i + 3] = MessageBuffer[port_index][MessageIndexStart[port_index]][i + 3];
					}

					calculated_crc = CalculateCRC8(crcCalculateBuffer, length + 3);

					MessageCounter++;
					if (calculated_crc == MessageBuffer[port_index][MessageIndexStart[port_index]][length + 3]) {
						AcceptedMessages++;
						MessageLength[port_index] = length;
						memcpy(&cMessage[port_index][0], &MessageBuffer[port_index][MessageIndexStart[port_index]][3],length);

						result =BOS_OK;

						/* Is it a broadcast or a multi-cast message with unique ID? */
						if(dst == BOS_BROADCAST || dst == BOS_MULTICAST) {
						if(dst == BOS_BROADCAST && cMessage[port_number - 1][MessageLength[port_number - 1] - 1] != bcastLastID){
							bcastID =bcastLastID =cMessage[port_number - 1][MessageLength[port_number - 1] - 1]; /* Store bcastID */
							BroadcastReceivedMessage(BOS_BROADCAST,port_number);
							cMessage[port_number - 1][MessageLength[port_number - 1] - 1] =0; /* Reset bcastID location */
						}
						/* Reflection of last broadcast message! */
						else if(dst == BOS_BROADCAST && cMessage[port_number - 1][MessageLength[port_number - 1] - 1] == bcastLastID){
							result =BOS_ERR_MSG_Reflection;
						}

						if(dst == BOS_MULTICAST && cMessage[port_number - 1][MessageLength[port_number - 1] - 1] != bcastLastID){
							bcastID =bcastLastID =cMessage[port_number - 1][MessageLength[port_number - 1] - 1]; /* Store bcastID */
							BroadcastReceivedMessage(BOS_MULTICAST,port_number);
							cMessage[port_number - 1][MessageLength[port_number - 1] - 1] =0; /* Reset bcastID location */
							/* Number of members in this multicast group
							 * TODO: breaks when message is 14 length and padded */
							NumofModulesinGroup =cMessage[port_number - 1][MessageLength[port_number - 1] - 2];
							/* Am I part of this multicast group? */
							result =BOS_ERR_WrongID;
								for (uint8_t i = 0; i < NumofModulesinGroup; i++) {
									if (myID == cMessage[port_number - 1][MessageLength[port_number - 1] - 2 - NumofModulesinGroup + i]) {
										result = BOS_OK;
										break;
									}
								}
							}
						/* Reflection of last multi-cast message! */
						else if(dst == BOS_MULTICAST && cMessage[port_number - 1][MessageLength[port_number - 1] - 1] == bcastLastID){
							result =BOS_ERR_MSG_Reflection;
						}
					}

						/* Notify messaging tasks */
						if (result == BOS_OK)
							NotifyMessagingTask(port_number);

					} else {
						RejectedMessages++;
						//TODO: Implement something here when the message is rejected.
					}

				}

				MessageIndexStart[port_index]++;
				if (MessageIndexStart[port_index] == MSG_COUNT)
					MessageIndexStart[port_index] = 0;

				ProcessMessageIndexStart++;
				if (ProcessMessageIndexStart == MSG_COUNT)
					ProcessMessageIndexStart = 0;
			}
		}
	}
}

/***************************************************************************/
/* PxMessagingTask function ************************************************/
/***************************************************************************/
void PxMessagingTask(void *argument){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef status =HAL_OK;
	portBASE_TYPE xReturned;

	bool extendCode = false, extendOptions = false;
	int8_t *pcOutputString;
	static int8_t cCLIString[cmdMAX_INPUT_SIZE];
	uint8_t port, src, dst, temp, i, p, shift, numOfParams;
	static uint8_t longMessageScratchpad[(MAX_NUM_OF_PORTS + 1) * MAX_NUM_OF_MODULES];
	uint16_t longMessageLastPtr =0;
	uint16_t code;
	uint32_t count, timeout, temp32;
	uint16_t messageCode ;
	
	port =(int8_t )(unsigned )argument;
	
	/* Infinite loop */
	for(;;){
		result =BOS_OK;
		
		/* Wait forever until a message is received on one of the ports */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		
		if(MessageLength[port - 1]){
			
			/* Read message source and destination */
			dst =cMessage[port - 1][0];
			src =cMessage[port - 1][1];
			
			/* Reset Array index shift */
			shift =0;
			
			/* Assign the value of option byte to OptionByte structure */
			*(uint8_t* )&OptionByte =(cMessage[port - 1][2]);

			/* Read message options */
			/* TODO handle extended options case */
			if(OptionByte.ExtendedOptions){ // 1st bit (LSB) Extended options
				extendOptions = true;
				(void )extendOptions; /* remove warning */
				++shift;
			}

			/* Read message code - LSB first */
			if(OptionByte.ExtendedMessageCode){
				code =(((uint16_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][3 + shift]);
				++shift;
			}
			else
				code =cMessage[port - 1][3 + shift];

			/*ACK Massage */
			if(OptionByte.Acknowledgment){
				OptionByte.Acknowledgment =false;
				SendMessageToModule(src,MSG_ACKNOWLEDGMENT_ACCEPTED,0);
			}

			/* Set shift index to the start of message payload (parameters) */
			shift +=4;

			/* Message payload size */
			numOfParams =MessageLength[port - 1] - shift;

			/* Process BOS Messages payload */
			if(result == BOS_OK){
				switch(code){
					case CODE_UNKNOWN_MESSAGE:
						break;

					case CODE_PING:
						IndicatorMode =IND_PING;
						if(OptionByte.Response == BOS_RESPONSE_ALL || OptionByte.Response == BOS_RESPONSE_MSG)
							SendMessageToModule(src,CODE_PING_RESPONSE,0);
						break;

					case CODE_PING_RESPONSE:
						if(pcPort == 0){
							if(!ModuleAlias[myID][0])
								sprintf((char* )pcUserMessage,"Hi from module %d\r\n",src);
							else
								sprintf((char* )pcUserMessage,"Hi from module %d (%s)\r\n",src,ModuleAlias[src]);
							writePxMutex(pcPort,pcUserMessage,strlen(pcUserMessage),cmd50ms,HAL_MAX_DELAY);
						}
						ResponseStatus =BOS_OK;
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
						Neighbors[port - 1][0] =((uint16_t )src << 8) + cMessage[port - 1][2 + shift]; /* Neighbor ID + Neighbor own port */
						Neighbors[port - 1][1] =((uint16_t )cMessage[port - 1][shift] << 8) + cMessage[port - 1][1 + shift]; /* Neighbor PN */

						IndicatorMode =IND_TOPOLOGY;

						/* Send your own info */
						MessageParams[0] =(uint8_t )(myPN >> 8);
						MessageParams[1] =(uint8_t )myPN;
						MessageParams[2] =port;
						osDelay(2);
						/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
						SendMessageFromPort(port,0,0,CODE_HI_RESPONSE,3);
						break;

					case CODE_HI_RESPONSE:
						/* Record your neighbor info */
						Neighbors[port - 1][0] =((uint16_t )src << 8) + cMessage[port - 1][2 + shift]; /* Neighbor ID + Neighbor own port */
						Neighbors[port - 1][1] =((uint16_t )cMessage[port - 1][shift] << 8) + cMessage[port - 1][1 + shift]; /* Neighbor PN */
						ResponseStatus =BOS_OK;
						break;

					case CODE_H1DR5_DEFAULTVALUES:
						EthernetDefaultSetting.Local_mac_addr[0] =cMessage[port - 1][0 + shift];
						EthernetDefaultSetting.Local_mac_addr[1] =cMessage[port - 1][1 + shift];
						EthernetDefaultSetting.Local_mac_addr[2] =cMessage[port - 1][2 + shift];
						EthernetDefaultSetting.Local_mac_addr[3] =cMessage[port - 1][3 + shift];
						EthernetDefaultSetting.Local_mac_addr[4] =cMessage[port - 1][4 + shift];
						EthernetDefaultSetting.Local_mac_addr[5] =cMessage[port - 1][5 + shift];

						EthernetDefaultSetting.Remote_mac_addr[0] =cMessage[port - 1][6 + shift];
						EthernetDefaultSetting.Remote_mac_addr[1] =cMessage[port - 1][7 + shift];
						EthernetDefaultSetting.Remote_mac_addr[2] =cMessage[port - 1][8 + shift];
						EthernetDefaultSetting.Remote_mac_addr[3] =cMessage[port - 1][9 + shift];
						EthernetDefaultSetting.Remote_mac_addr[4] =cMessage[port - 1][10 + shift];
						EthernetDefaultSetting.Remote_mac_addr[5] =cMessage[port - 1][11 + shift];

						EthernetDefaultSetting.Local_IP[0] =cMessage[port - 1][12 + shift];
						EthernetDefaultSetting.Local_IP[1] =cMessage[port - 1][13 + shift];
						EthernetDefaultSetting.Local_IP[2] =cMessage[port - 1][14 + shift];
						EthernetDefaultSetting.Local_IP[3] =cMessage[port - 1][15 + shift];

						EthernetDefaultSetting.Remote_IP[0] =cMessage[port - 1][16 + shift];
						EthernetDefaultSetting.Remote_IP[1] =cMessage[port - 1][17 + shift];
						EthernetDefaultSetting.Remote_IP[2] =cMessage[port - 1][18 + shift];
						EthernetDefaultSetting.Remote_IP[3] =cMessage[port - 1][19 + shift];

						EthernetDefaultSetting.ip_mask[0] =cMessage[port - 1][20 + shift];
						EthernetDefaultSetting.ip_mask[1] =cMessage[port - 1][21 + shift];
						EthernetDefaultSetting.ip_mask[2] =cMessage[port - 1][22 + shift];
						EthernetDefaultSetting.ip_mask[3] =cMessage[port - 1][23 + shift];

						EthernetDefaultSetting.ip_dest[0] =cMessage[port - 1][24 + shift];
						EthernetDefaultSetting.ip_dest[1] =cMessage[port - 1][25 + shift];
						EthernetDefaultSetting.ip_dest[2] =cMessage[port - 1][26 + shift];
						EthernetDefaultSetting.ip_dest[3] =cMessage[port - 1][27 + shift];

						EthernetDefaultSetting.Local_PORT =cMessage[port - 1][28 + shift];
						EthernetDefaultSetting.Remote_PORT =cMessage[port - 1][29 + shift];
						break;

#ifndef __N
					case CODE_EXPLORE_ADJ:
						ExploreNeighbors(port);
						IndicatorMode =IND_TOPOLOGY;
						osDelay(50);
						temp =0;
						/* Exploration response message */
						for(uint8_t p =1; p <= NUM_OF_PORTS; p++){
							if(Neighbors[p - 1][0]){
								MessageParams[temp] =p;
								memcpy(MessageParams + temp + 1,Neighbors[p - 1],(size_t )(4));
								temp +=5;
							}
						}
						SendMessageToModule(src,CODE_EXPLORE_ADJ_RESPONSE,temp);
						break;

					case CODE_EXPLORE_ADJ_RESPONSE:
						/* Extract the other module Neighbors */
						temp =numOfParams / 5;
						for(uint8_t k =0; k < temp; k++){
							memcpy(&Neighbors2[(cMessage[port - 1][shift + k * 5]) - 1][0],&cMessage[port - 1][1 + shift + k * 5],(size_t )(4));
						}
						ResponseStatus =BOS_OK;
						break;
#endif

					case CODE_PORT_DIRECTION:
						/* Reverse/un-reverse ports according to command parameters */
						for(uint8_t p =1; p <= NUM_OF_PORTS; p++){
							if(p != port)
								SwapUartPins(GetUart(p),cMessage[port - 1][shift + p - 1]);
						}
						/* Check the input port direction */
						SwapUartPins(GetUart(port),cMessage[port - 1][shift + MAX_NUM_OF_PORTS]);
						break;

					case CODE_MODULE_ID:
						if(cMessage[port - 1][shift] == 0) /* Change my own ID */
							myID =cMessage[port - 1][1 + shift];
						else if(cMessage[port - 1][shift] == 1){ /* Change my neighbor's ID */
							MessageParams[0] =0; /* change own ID */
							MessageParams[1] =cMessage[port - 1][1 + shift]; /* The new ID */
							SendMessageFromPort(cMessage[port - 1][2 + shift],0,0,CODE_MODULE_ID,3);
						}
						break;

					case CODE_TOPOLOGY:
						if(OptionByte.LongMessage){
							/* Array is 2-byte oriented thus memcpy can copy only even number of bytes
							 * TODO test maybe broken */
							/* Use a 1-byte oriented scratchpad */
							memcpy(&longMessageScratchpad[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
							longMessageLastPtr +=numOfParams;
						}
						else{
							memcpy(&longMessageScratchpad[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
							longMessageLastPtr +=numOfParams;
							N =(longMessageLastPtr / (MAX_NUM_OF_PORTS + 1)) / 2;
							/* Copy the scratchpad to Array */
							memcpy(&Array,&longMessageScratchpad,longMessageLastPtr);
							longMessageLastPtr =0;
							IndicatorMode =IND_TOPOLOGY;
						}
						break;

					case CODE_READ_PORT_DIR:
						ReadPortsDirMSG(src);
						break;

					case CODE_READ_PORT_DIR_RESPONSE:
						/* Read module ports directions */
						for(p =0; p < numOfParams; p++){
							ArrayPortsDir[src - 1] |=(0x8000 >> ((cMessage[port - 1][shift + p]) - 1));
						}
						ResponseStatus =BOS_OK;
						break;

					case CODE_BAUDRATE:
						/* Change baudrate of specified ports */
						temp =temp32 =0;
						temp32 =((uint32_t )cMessage[port - 1][shift] << 24) + ((uint32_t )cMessage[port - 1][1 + shift] << 16) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + cMessage[port - 1][3 + shift];
						if(cMessage[port - 1][4 + shift] == 0xFF) // All ports
						{
							for(p =1; p <= NUM_OF_PORTS; p++){
								UpdateBaudrate(p,temp32);
							}
						}
						else{
							for(p =0; p < numOfParams; p++){
								temp =cMessage[port - 1][4 + shift + p];
								if(temp > 0 && temp <= NUM_OF_PORTS){
									UpdateBaudrate(temp,temp32);
								}
							}
						}
						break;

					case CODE_EXP_EEPROM:
						SaveTopologyToRO();
						SaveEEportsDir();
						IndicatorMode =IND_PING;
						break;

					case CODE_DEF_ARRAY:
						/* Clear the topology */
						ClearEEportsDir();
#ifndef __N
						ClearROtopology();
#endif
						osDelay(100);
						IndicatorMode =IND_TOPOLOGY;
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
							/* Pass the inport to CLI command parsers temporarily through pcPort */
							temp =pcPort;
							pcPort =port;
							/* Process the command locally */
							xReturned =FreeRTOS_CLIProcessCommand(cCLIString,pcOutputString,configCOMMAND_INT_MAX_OUTPUT_SIZE);
							/* Restore back pcPort */
							pcPort =temp;
							/* Respond to the CLI command */
							if(OptionByte.Response == BOS_RESPONSE_ALL){
								/* Copy the generated string to MessageParams */
								memcpy(MessageParams,pcOutputString,strlen((char* )pcOutputString));
								/* Send command response */
								SendLargeMessageToModule(src,CODE_CLI_RESPONSE,(uint8_t* )pcOutputString,strlen((char* )pcOutputString));
//									SendMessageToModule(src,CODE_CLI_RESPONSE,strlen((char* )pcOutputString));
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
						if(OptionByte.LongMessage){
							memcpy(&pcOutputString[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
							longMessageLastPtr +=numOfParams;
						}
						else{
							memcpy(&pcOutputString[0] + longMessageLastPtr,&cMessage[port - 1][shift],(size_t )numOfParams);
							longMessageLastPtr =0;
							ResponseStatus =BOS_OK;
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
						IndicatorMode =IND_PING;
						osDelay(10);
						NVIC_SystemReset();
						break;

					case CODE_UPDATE_VIA_PORT:
						/* I'm the last module before target.
						 * First, ask the target to jump to factory bootloader */
						SendMessageFromPort(cMessage[port - 1][shift],0,0,CODE_UPDATE,0);
						osDelay(100);
						/* Then, setup myself for remote 'via port' update */
						RemoteBootloaderUpdate(src,myID,port,cMessage[port - 1][shift]);
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
							/* Get requested address */
							temp32 =((uint32_t )cMessage[port - 1][2 + shift] << 24) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
							/* Get variable according to requested format */
							switch(cMessage[port - 1][1 + shift]) /* requested format */
							{
								case FMT_BOOL:
								case FMT_UINT8:
									MessageParams[0] =*(__IO uint8_t* )temp32;
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,1);
									break;

								case FMT_INT8:
									MessageParams[0] =*(__IO int8_t* )temp32;
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,1);
									break;

								case FMT_UINT16:
									MessageParams[0] =(uint8_t )((*(__IO uint16_t* )temp32) >> 0);
									MessageParams[1] =(uint8_t )((*(__IO uint16_t* )temp32) >> 8);
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,2);
									break;

								case FMT_INT16:
									MessageParams[0] =(uint8_t )((*(__IO int16_t* )temp32) >> 0);
									MessageParams[1] =(uint8_t )((*(__IO int16_t* )temp32) >> 8);
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,2);
									break;

								case FMT_UINT32:
									MessageParams[0] =(uint8_t )((*(__IO uint32_t* )temp32) >> 0);
									MessageParams[1] =(uint8_t )((*(__IO uint32_t* )temp32) >> 8);
									MessageParams[2] =(uint8_t )((*(__IO uint32_t* )temp32) >> 16);
									MessageParams[3] =(uint8_t )((*(__IO uint32_t* )temp32) >> 24);
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,4);
									break;

								case FMT_INT32:
									MessageParams[0] =(uint8_t )((*(__IO int32_t* )temp32) >> 0);
									MessageParams[1] =(uint8_t )((*(__IO int32_t* )temp32) >> 8);
									MessageParams[2] =(uint8_t )((*(__IO int32_t* )temp32) >> 16);
									MessageParams[3] =(uint8_t )((*(__IO int32_t* )temp32) >> 24);
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,4);
									break;

								case FMT_FLOAT:
									MessageParams[0] =*(__IO uint8_t* )(temp32 + 0);
									MessageParams[1] =*(__IO uint8_t* )(temp32 + 1);
									MessageParams[2] =*(__IO uint8_t* )(temp32 + 2);
									MessageParams[3] =*(__IO uint8_t* )(temp32 + 3);
									SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,8);
									break;
								default:
									break;
							}
						}
						else if(cMessage[port - 1][shift] == REMOTE_MODULE_PARAM) /* request for a Module param */
						{
							/* adding string termination */
							cMessage[port - 1][MessageLength[port - 1] - 1] =0;
							/* Extracting module parameter */
							temp =IsModuleParameter((char* )&cMessage[port - 1][1 + shift]);
							if(temp == 0){
								/* Parameter does not exist */
								SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,1);
							}
							else{
								/* Parameter exists. Get its pointer */
								temp32 =(uint32_t )ModuleParam[temp - 1].ParamPtr;
								MessageParams[0] =ModuleParam[temp - 1].ParamFormat;
								/* Send parameter according to its format */
								switch(MessageParams[0]) /* requested format */
								{
									case FMT_BOOL:
									case FMT_UINT8:
										MessageParams[1] =*(__IO uint8_t* )temp32;
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,2);
										break;

									case FMT_INT8:
										MessageParams[1] =*(__IO int8_t* )temp32;
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,2);
										break;

									case FMT_UINT16:
										MessageParams[1] =(uint8_t )((*(__IO uint16_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO uint16_t* )temp32) >> 8);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,3);
										break;

									case FMT_INT16:
										MessageParams[1] =(uint8_t )((*(__IO int16_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO int16_t* )temp32) >> 8);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,3);
										break;

									case FMT_UINT32:
										MessageParams[1] =(uint8_t )((*(__IO uint32_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO uint32_t* )temp32) >> 8);
										MessageParams[3] =(uint8_t )((*(__IO uint32_t* )temp32) >> 16);
										MessageParams[4] =(uint8_t )((*(__IO uint32_t* )temp32) >> 24);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,5);
										break;

									case FMT_INT32:
										MessageParams[1] =(uint8_t )((*(__IO int32_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO int32_t* )temp32) >> 8);
										MessageParams[3] =(uint8_t )((*(__IO int32_t* )temp32) >> 16);
										MessageParams[4] =(uint8_t )((*(__IO int32_t* )temp32) >> 24);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,5);
										break;

									case FMT_FLOAT:
										MessageParams[1] =*(__IO uint8_t* )(temp32 + 0);
										MessageParams[2] =*(__IO uint8_t* )(temp32 + 1);
										MessageParams[3] =*(__IO uint8_t* )(temp32 + 2);
										MessageParams[4] =*(__IO uint8_t* )(temp32 + 3);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,9);
										break;

									default:
										break;
								}
							}
						}
						else if(cMessage[port - 1][shift] >= REMOTE_BOS_VAR) /* request for a BOS var */
						{
							MessageParams[0] =bosVarRegister[cMessage[port - 1][shift] - REMOTE_BOS_VAR - 1] & 0x000F; /* send variable format (lower 4 bits) */
							/* Variable does not exist */
							if(MessageParams[0] == 0){
								SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,1);
							}
							else{
                                /* Variable exists. Get its memory address */
                                temp32 =(bosVarRegister[cMessage[port - 1][shift] - REMOTE_BOS_VAR - 1] >> 16) + SRAM_BASE+ 0x10000;
                                /* Send variable according to its format */
								switch(MessageParams[0]) // requested format
								{
									case FMT_BOOL:
									case FMT_UINT8:
										MessageParams[1] =*(__IO uint8_t* )temp32;
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,2);
										break;

									case FMT_INT8:
										MessageParams[1] =*(__IO int8_t* )temp32;
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,2);
										break;

									case FMT_UINT16:
										MessageParams[1] =(uint8_t )((*(__IO uint16_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO uint16_t* )temp32) >> 8);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,3);
										break;

									case FMT_INT16:
										MessageParams[1] =(uint8_t )((*(__IO int16_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO int16_t* )temp32) >> 8);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,3);
										break;

									case FMT_UINT32:
										MessageParams[1] =(uint8_t )((*(__IO uint32_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO uint32_t* )temp32) >> 8);
										MessageParams[3] =(uint8_t )((*(__IO uint32_t* )temp32) >> 16);
										MessageParams[4] =(uint8_t )((*(__IO uint32_t* )temp32) >> 24);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,5);
										break;

									case FMT_INT32:
										MessageParams[1] =(uint8_t )((*(__IO int32_t* )temp32) >> 0);
										MessageParams[2] =(uint8_t )((*(__IO int32_t* )temp32) >> 8);
										MessageParams[3] =(uint8_t )((*(__IO int32_t* )temp32) >> 16);
										MessageParams[4] =(uint8_t )((*(__IO int32_t* )temp32) >> 24);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,5);
										break;

									case FMT_FLOAT:
										MessageParams[1] =*(__IO uint8_t* )(temp32 + 0);
										MessageParams[2] =*(__IO uint8_t* )(temp32 + 1);
										MessageParams[3] =*(__IO uint8_t* )(temp32 + 2);
										MessageParams[4] =*(__IO uint8_t* )(temp32 + 3);
										SendMessageToModule(src,CODE_READ_REMOTE_RESPONSE,9);
										break;

									default:
										break;
								}
							}
						}

						break;

					case CODE_READ_REMOTE_RESPONSE:
						/* We requested a BOS variable or module param */
						if(RemoteBuffer == REMOTE_BOS_VAR || RemoteBuffer == REMOTE_MODULE_PARAM){
							/* Read variable according to its format */
							RemoteVarFormat =(VariableFormat_t )cMessage[port - 1][shift];
							switch(cMessage[port - 1][shift]) /* Remote format */
							{
								case 0: /* This variable does not exist */
									ResponseStatus =BOS_ERR_REMOTE_READ_NO_VAR;
									break;

								case FMT_BOOL:
								case FMT_UINT8:
									RemoteBuffer =cMessage[port - 1][1 + shift];
									break;

								case FMT_INT8:
									RemoteBuffer =(int8_t )cMessage[port - 1][1 + shift];
									break;

								case FMT_UINT16:
									RemoteBuffer =((uint16_t )cMessage[port - 1][1 + shift] << 0) + ((uint16_t )cMessage[port - 1][2 + shift] << 8);
									break;

								case FMT_INT16:
									RemoteBuffer =((int16_t )cMessage[port - 1][1 + shift] << 0) + ((int16_t )cMessage[port - 1][2 + shift] << 8);
									break;

								case FMT_UINT32:
									RemoteBuffer =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
									break;

								case FMT_INT32:
									RemoteBuffer =((int32_t )cMessage[port - 1][1 + shift] << 0) + ((int32_t )cMessage[port - 1][2 + shift] << 8) + ((int32_t )cMessage[port - 1][3 + shift] << 16) + ((int32_t )cMessage[port - 1][4 + shift] << 24);
									break;

								case FMT_FLOAT:
									RemoteBuffer =((uint32_t )cMessage[port - 1][1 + shift] << 0) + ((uint32_t )cMessage[port - 1][2 + shift] << 8) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 24);
									break;

								default:
									break;
							}

						}
						else if(RemoteBuffer == REMOTE_MEMORY_ADD) /* We requested a memory location */
						{
							/* Read variable according to requested format */
							switch(RequestFormat) /* Requested format */
							{
								case FMT_BOOL:
								case FMT_UINT8:
									RemoteBuffer =cMessage[port - 1][shift];
									break;

								case FMT_INT8:
									RemoteBuffer =(int8_t )cMessage[port - 1][shift];
									break;

								case FMT_UINT16:
									RemoteBuffer =((uint16_t )cMessage[port - 1][shift] << 0) + ((uint16_t )cMessage[port - 1][1 + shift] << 8);
									break;

								case FMT_INT16:
									RemoteBuffer =((int16_t )cMessage[port - 1][shift] << 0) + ((int16_t )cMessage[port - 1][1 + shift] << 8);
									break;

								case FMT_UINT32:
									RemoteBuffer =((uint32_t )cMessage[port - 1][shift] << 0) + ((uint32_t )cMessage[port - 1][1 + shift] << 8) + ((uint32_t )cMessage[port - 1][2 + shift] << 16) + ((uint32_t )cMessage[port - 1][3 + shift] << 24);
									break;

								case FMT_INT32:
									RemoteBuffer =((int32_t )cMessage[port - 1][shift] << 0) + ((int32_t )cMessage[port - 1][1 + shift] << 8) + ((int32_t )cMessage[port - 1][2 + shift] << 16) + ((int32_t )cMessage[port - 1][3 + shift] << 24);
									break;

								case FMT_FLOAT:
									RemoteBuffer =((uint32_t )cMessage[port - 1][shift] << 0) + ((uint32_t )cMessage[port - 1][1 + shift] << 8) + ((uint32_t )cMessage[port - 1][2 + shift] << 16) + ((uint32_t )cMessage[port - 1][3 + shift] << 24);
									break;

								default:
									break;
							}
						}
						else{
						}
						/* Remote read status */
						if(ResponseStatus != BOS_ERR_REMOTE_READ_NO_VAR)
							ResponseStatus =BOS_OK;
						break;

					case CODE_WRITE_REMOTE:
//					case CODE_WRITE_REMOTE_FORCE:

						ResponseStatus =BOS_OK; /* Initialize response */
						if(cMessage[port - 1][shift]) /* request for a BOS var */
						{
							/* Check variable index is within the limit of MAX_BOS_VARS */
							if(cMessage[port - 1][shift] <= MAX_BOS_VARS){
								/* Get var memory address */
                                temp32 =(bosVarRegister[cMessage[port - 1][shift] - 1] >> 16) + SRAM_BASE+0x10000; // Get var memory addres
                                /* Modify the variable or create a new one if it does not exist */
								switch(cMessage[port - 1][1 + shift]) // requested format
								{
									case FMT_BOOL:
									case FMT_UINT8:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											/* Create a new one */
											temp32 =(uint32_t )malloc(sizeof(uint8_t));
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												/* Cannot allocate memory */
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										/* Write remote value */
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)
											*(__IO uint8_t* )temp32 =cMessage[port - 1][2 + shift];
										break;

									case FMT_INT8:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											/* Create a new one */
											temp32 =(uint32_t )malloc(sizeof(int8_t));
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												/* Cannot allocate memory */
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										/* Write remote value */
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)
											*(__IO int8_t* )temp32 =(int8_t )cMessage[port - 1][2 + shift];
										break;

									case FMT_UINT16:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											temp32 =(uint32_t )malloc(sizeof(uint16_t));
											/* Create a new one */
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												/* Cannot allocate memory */
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										/* Write remote value */
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)
											*(__IO uint16_t* )temp32 =((uint16_t )cMessage[port - 1][2 + shift] << 0) + ((uint16_t )cMessage[port - 1][3 + shift] << 8);
										break;

									case FMT_INT16:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											/* Create a new one */
											temp32 =(uint32_t )malloc(sizeof(int16_t));
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												/* Cannot allocate memory */
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										/* Write remote value */
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)
											*(__IO int16_t* )temp32 =((int16_t )cMessage[port - 1][2 + shift] << 0) + ((int16_t )cMessage[port - 1][3 + shift] << 8);
										break;

									case FMT_UINT32:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											temp32 =(uint32_t )malloc(sizeof(uint32_t));
											/* Create a new one */
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												/* Cannot allocate memory */
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)
											*(__IO uint32_t* )temp32 =((uint32_t )cMessage[port - 1][2 + shift] << 0) + ((uint32_t )cMessage[port - 1][3 + shift] << 8) + ((uint32_t )cMessage[port - 1][4 + shift] << 16) + ((uint32_t )cMessage[port - 1][5 + shift] << 24);
										break;

									case FMT_INT32:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											temp32 =(uint32_t )malloc(sizeof(int32_t));
											/* Create a new one */
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL)
											*(__IO int32_t* )temp32 =((int32_t )cMessage[port - 1][2 + shift] << 0) + ((int32_t )cMessage[port - 1][3 + shift] << 8) + ((int32_t )cMessage[port - 1][4 + shift] << 16) + ((int32_t )cMessage[port - 1][5 + shift] << 24);
										break;

									case FMT_FLOAT:
										/* Variable does not exist */
										if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) == 0){
											temp32 =(uint32_t )malloc(sizeof(float));
											/* Create a new one */
											if(temp32 != 0){
												bosVarRegister[cMessage[port - 1][shift] - 1] =((temp32 - SRAM_BASE) << 16) + cMessage[port - 1][1 + shift];
											}
											else{
												ResponseStatus =BOS_ERR_REMOTE_WRITE_MEM_FULL;
											}
										}
										if(ResponseStatus != BOS_ERR_REMOTE_WRITE_MEM_FULL){
											RemoteBuffer =((uint32_t )cMessage[port - 1][2 + shift] << 0) + ((uint32_t )cMessage[port - 1][3 + shift] << 8) + ((uint32_t )cMessage[port - 1][4 + shift] << 16) + ((uint32_t )cMessage[port - 1][5 + shift] << 24);
											*(float* )temp32 =*(float* )&RemoteBuffer;
										}
										break;

									default:
										break;
								}

								/* Update local format if needed
								 * Todo give local warning later */
								if((bosVarRegister[cMessage[port - 1][shift] - 1] & 0x000F) != cMessage[port - 1][1 + shift]){
									bosVarRegister[cMessage[port - 1][shift] - 1] &=(0xFFF0 + cMessage[port - 1][1 + shift]);
									ResponseStatus =BOS_ERR_LOCAL_FORMAT_UPDATED;
								}
							}
							else{
								/* BOS var index out of range */
								ResponseStatus =BOS_ERR_REMOTE_WRITE_INDEX;
							}
						}
						else
						{
							/* request for a memory address */
							/* Get the requested address */
							temp32 =((uint32_t )cMessage[port - 1][2 + shift] << 24) + ((uint32_t )cMessage[port - 1][3 + shift] << 16) + ((uint32_t )cMessage[port - 1][4 + shift] << 8) + cMessage[port - 1][5 + shift];
							/* Write data to Flash or SRAM based on requested format */
							if(temp32 >= SRAM_BASE && temp32 < (SRAM_BASE + SRAM_SIZE_MAX)) // SRAM
							{
								switch(cMessage[port - 1][1 + shift]) /* Requested format */
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
										RemoteBuffer =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);
										*(float* )temp32 =*(float* )&RemoteBuffer;
										break;

									default:
										break;
								}
							}
//							else if(temp32 >= FLASH_BASE && temp32 < (FLASH_BASE + FLASH_SIZE)) // Flash
//							{
//								HAL_FLASH_Unlock();
////								/* Erase page if force write is requested */
////								if(code == CODE_WRITE_REMOTE_FORCE){
////									EraseSector(PAGE(temp32));
////								}
//								/* Write new value */
//								if(ResponseStatus == BOS_OK){
//									switch(cMessage[port - 1][1 + shift]) // Requested format
//									{
//										case FMT_BOOL:
//										case FMT_UINT8:
//										case FMT_INT8:
//											if(*(__IO uint16_t* )temp32 != 0xFFFF){
//												ResponseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
//												break;
//											}
//											else{
//												RemoteBuffer =cMessage[port - 1][6 + shift];
//												// TOCHECKLATER
//												// available values in G0 MCU:
//												//TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
//												//TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
//												#ifndef STM32G0B1xx
//												status =HAL_FLASH_Program(
//												FLASH_TYPEPROGRAM_HALFWORD,temp32,RemoteBuffer);
//												#endif
//												break;
//											}
//										case FMT_UINT16:
//										case FMT_INT16:
//											if(*(__IO uint16_t* )temp32 != 0xFFFF){
//												ResponseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
//												break;
//											}
//											else{
//												RemoteBuffer =((uint16_t )cMessage[port - 1][6 + shift] << 0) + ((uint16_t )cMessage[port - 1][7 + shift] << 8);
//												// TOCHECKLATER
//												// available values in G0 MCU:
//												//TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
//												//TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
//												#ifndef STM32G0B1xx
//												status =HAL_FLASH_Program(
//												FLASH_TYPEPROGRAM_HALFWORD,temp32,RemoteBuffer);
//												#endif
//												break;
//											}
//										case FMT_UINT32:
//										case FMT_INT32:
//											if(*(__IO uint32_t* )temp32 != 0xFFFFFFFF){
//												ResponseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
//												break;
//											}
//											else{
//												RemoteBuffer =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);													// TOCHECKLATER
//												// TOCHECKLATER
//												// available values in G0 MCU:
//												//TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
//												//TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
//												#ifndef STM32G0B1xx
//												status =HAL_FLASH_Program(
//												FLASH_TYPEPROGRAM_WORD,temp32,RemoteBuffer);
//												#endif
//												break;
//											}
//										case FMT_FLOAT:
//											if(*(__IO uint32_t* )temp32 != 0xFFFFFFFF){
//												ResponseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
//												break;
//											}
//											else{
//												RemoteBuffer =((uint32_t )cMessage[port - 1][6 + shift] << 0) + ((uint32_t )cMessage[port - 1][7 + shift] << 8) + ((uint32_t )cMessage[port - 1][8 + shift] << 16) + ((uint32_t )cMessage[port - 1][9 + shift] << 24);
//												status =HAL_FLASH_Program(
//												FLASH_TYPEPROGRAM_DOUBLEWORD,temp32,RemoteBuffer);
//												break;
//											}
//										default:
//											break;
//									}
//								}
//								HAL_FLASH_Lock();
//								if(status != HAL_OK)
//									ResponseStatus =BOS_ERR_REMOTE_WRITE_FLASH;
//							}
							else
								ResponseStatus =BOS_ERR_REMOTE_WRITE_ADDRESS;
						}

						/* Send confirmation back */
						if(OptionByte.Response == BOS_RESPONSE_ALL || OptionByte.Response == BOS_RESPONSE_MSG){
							MessageParams[0] =ResponseStatus;
							SendMessageToModule(src,CODE_WRITE_REMOTE_RESPONSE,1);
						}
						break;

					case CODE_WRITE_REMOTE_RESPONSE:
						ResponseStatus =(BOS_Status )cMessage[port - 1][shift];
						break;

					case CODE_PORT_FORWARD:
						writePxMutex(cMessage[port - 1][shift],(char* )&cMessage[port - 1][shift + 1],numOfParams - 1,10,10);
						break;

					case CODE_READ_ADC_VALUE:
						adcPort =cMessage[port - 1][shift];
						adcSide =cMessage[port - 1][shift + 1];
						if(0 == adcSide){
							ADCSelectPort(adcPort);
							ReadADCChannel(adcPort,"top",&adcValue);
						}
						else if(1 == adcSide){
							ADCSelectPort(adcPort);
							ReadADCChannel(adcPort,"bottom",&adcValue);
						}

					case CODE_READ_TEMPERATURE:
					case CODE_READ_VREF:
						ReadTempAndVref(&InternalTemperature,&InternalVoltageReferance);

//					case CODE_READ_ADC_PERCENTAGE:
//						adcPort = cMessage[port - 1][shift];
//						GetReadPercentage(adcPort, &adcPercentage);
//						MBmessageParams[7] = ((uint32_t) cMessage[port - 1][5 + shift] << 0)
//								+ ((uint32_t) cMessage[port - 1][6 + shift] << 8)
//								+ ((uint32_t) cMessage[port - 1][7 + shift] << 16)
//								+ ((uint32_t) cMessage[port - 1][8 + shift] << 24);
//						MBmessageParams[8] = ((uint32_t) cMessage[port - 1][9 + shift] << 0)
//								+ ((uint32_t) cMessage[port - 1][10 + shift] << 8)
//								+ ((uint32_t) cMessage[port - 1][11 + shift] << 16)
//								+ ((uint32_t) cMessage[port - 1][12 + shift] << 24);
//						break;

					case MSG_ACKNOWLEDGMENT_ACCEPTED:
						ACKMessageFlag =1;
						break;

					case MSG_REJECTED:
						RejectedMessageFlag =1;
						break;

					case CODE_READ_RESPONSE:
						RemoteResponseFlag =1;
					    /* Message code stored in bytes [3 + shift, 4 + shift], dedicated for Arduino and Raspberry Pi */
						messageCode  = ((uint16_t)cMessage[port - 1][3 + shift] << 0) | ((uint16_t)cMessage[port - 1][4 + shift] << 8);

						switch(cMessage[port - 1][shift]){
							case 0:
								if(BOS_OK == cMessage[port - 1][1 + shift]){
									result =BOS_OK;
									ResponseStatus =BOS_ERR_REMOTE_READ_NO_VAR;
								}
								else
									result =BOS_ERROR;
								break;

					case FMT_BOOL:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = (uint32_t) cMessage[port - 1][5 + shift];
						} else {
							result = BOS_ERROR;
						}
						break;

					case FMT_UINT8:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = (uint32_t) cMessage[port - 1][5 + shift];
							((uint32_t*) RemoteResponseBuffer)[1] = (uint32_t) cMessage[port - 1][6 + shift];
							((uint32_t*) RemoteResponseBuffer)[2] = (uint32_t) cMessage[port - 1][7 + shift];
						} else {
							result = BOS_ERROR;
						}
						break;

					case FMT_INT8:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = (uint32_t) cMessage[port - 1][5 + shift];
						} else {
							result = BOS_ERROR;
						}
						break;

					case FMT_UINT16:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = ((uint32_t) cMessage[port - 1][5 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][6 + shift] << 8);
							((uint32_t*) RemoteResponseBuffer)[1] = ((uint32_t) cMessage[port - 1][7 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][8 + shift] << 8);
							((uint32_t*) RemoteResponseBuffer)[2] = ((uint32_t) cMessage[port - 1][9 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][10 + shift] << 8);
						} else {
							result = BOS_ERROR;
						}
						break;

					case FMT_INT16:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = ((uint32_t) cMessage[port - 1][5 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][6 + shift] << 8);
						} else {
							result = BOS_ERROR;
						}
						break;

					case FMT_UINT32:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = ((uint32_t) cMessage[port - 1][5 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][6 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][7 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][8 + shift] << 24);
						} else {
							result = BOS_ERROR;
						}
						break;

					case FMT_INT32:
						if (BOS_OK == cMessage[port - 1][1 + shift]) {
							result = BOS_OK;
							NumOfElement = cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							((uint32_t*) RemoteResponseBuffer)[0] = ((uint32_t) cMessage[port - 1][5 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][6 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][7 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][8 + shift] << 24);
							((uint32_t*) RemoteResponseBuffer)[1] = ((uint32_t) cMessage[port - 1][9 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][10 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][11 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][12 + shift] << 24);
							((uint32_t*) RemoteResponseBuffer)[2] = ((uint32_t) cMessage[port - 1][13 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][14 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][15 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][16 + shift] << 24);
						} else {
							result = BOS_ERROR;
						}
						break;

							case FMT_FLOAT:
								if(BOS_OK == cMessage[port - 1][1 + shift]){
									result =BOS_OK;
									NumOfElement =cMessage[port - 1][2 + shift];
							/* Skip command code bytes [3 + shift, 4 + shift] */
							RemoteResponseBuffer[0] = ((uint32_t) cMessage[port - 1][5 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][6 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][7 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][8 + shift] << 24);
							RemoteResponseBuffer[1] = ((uint32_t) cMessage[port - 1][9 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][10 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][11 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][12 + shift] << 24);
							RemoteResponseBuffer[2] = ((uint32_t) cMessage[port - 1][13 + shift] << 0)
									| ((uint32_t) cMessage[port - 1][14 + shift] << 8)
									| ((uint32_t) cMessage[port - 1][15 + shift] << 16)
									| ((uint32_t) cMessage[port - 1][16 + shift] << 24);		}
								else
									result =BOS_ERROR;
								break;

							default:
								break;
						}
						break;

						/* Power Mode: Stop mode enable */
					case ENABLE_STOP_MODE_UARTX:

						PortSelect =(cMessage[port - 1][shift]);

						if(result != EnableStopModebyUARTx(PortSelect))
							result =BOS_ERROR;
						break;

						/* Power Mode: Standby mode enable */
					case ENABLE_STANDBY_MODE_WAKE_UP_PINX:

						PinSelect =(cMessage[port - 1][shift]);

						if(result != EnableStandbyModebyWakeupPinx(PinSelect))
							result =BOS_ERROR;
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
		
		/* Is it unknown message? */
		if(result == BOS_ERR_UnknownMessage){
			SendMessageToModule(src,CODE_UNKNOWN_MESSAGE,0);
			result =BOS_OK;
		}
		
		/* Reset message buffer */
		memset(cMessage[port - 1],0,(size_t )MessageLength[port - 1]);
		MessageLength[port - 1] =0;
		if(PortStatus[port] != STREAM && PortStatus[port] != CLI && PortStatus[port] != PORTBUTTON){
			/* Free the port */
			PortStatus[port] =FREE;
		}

		taskYIELD();
	}
}

/***************************************************************************/
/* User message parser:
 This function is declared as __weak to be overwritten by other implementations in user file.
 */
__weak BOS_Status User_MessagingParser(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift)

{
	BOS_Status result =BOS_ERR_UnknownMessage;
	
	return result;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
