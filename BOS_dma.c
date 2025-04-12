/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_dma.c
 Description   : Source code for BOS communication/backend DMAs.

 Required MCU resources:

 >> At least n UART RX DMA channels where n is number of module ports (up to number of available UARTs).
 >> At least one UART TX DMA channel.
 >> Any extra channels can be assigned to front-end.

 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "BOS_DMA.h"

/* Private variables *******************************************************/
uint8_t UARTRxBuf[NUM_OF_PORTS][MSG_RX_BUF_SIZE] ={0};
UART_HandleTypeDef *dmaStreamDst[NUM_OF_PORTS] ={0};
uint32_t dmaStreamCount[NUM_OF_PORTS] ={0};
uint32_t dmaStreamTotal[NUM_OF_PORTS] ={0};
bool MsgDMAStopped[NUM_OF_PORTS] ={0};

/* Exported variables ******************************************************/
extern uint16_t dmaDstPort[NUM_OF_PORTS];
extern uint8_t StreamCplt;

/***************************************************************************/
/* Setup and start a streaming DMA (port-to-port) */
BOS_Status StartDMAstream(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num){
	uint8_t srcPort =GetPort(huartSrc);
	
	/* switch the DMA channel to streaming if it's available */
	if(PortStatus[srcPort] == FREE || PortStatus[srcPort] == MSG || PortStatus[srcPort] == CLI){
		SwitchMsgDMAToStream(srcPort);
	}
	else if(PortStatus[srcPort] == STREAM){
		return BOS_ERR_PORT_BUSY;
	}
	else
		return BOS_ERR_PORT_BUSY;

	/* Setup the streaming destination */
	dmaStreamDst[srcPort - 1] =huartDst;
	
	/* Lock the source port by marking it as STREAM
	 * This prevents other tasks from using it while streaming is active */
	PortStatus[srcPort] =STREAM;
	
	/* Initialize the DMA stream counter */
	dmaStreamCount[srcPort - 1] =0;
	
	/* Setup and start the actual DMA stream */
	DMA_STREAM_Setup(huartSrc,huartDst,num);
	
	return BOS_OK;
}

/***************************************************************************/
/* DMA interrupt service routine */
void DMA_IRQHandler(uint8_t port){

	if(PortStatus[port] != STREAM){
		HAL_DMA_IRQHandler(UARTDMAHandler[port - 1]);
	}
	else{
		HAL_DMA_IRQHandler(UARTDMAHandler[port - 1]);
		if(dmaStreamTotal[port - 1])
			++dmaStreamCount[port - 1];
		if(dmaStreamCount[port - 1] >= dmaStreamTotal[port - 1] || ((uint8_t )dmaDstPort[port - 1] == P_VIRTUAL)){

			uint8_t direction =dmaDstPort[port - 1] >> 8;
			uint8_t dst =(uint8_t )dmaDstPort[port - 1];
			if((direction == FORWARD) || (direction == BACKWARD)){
				SwitchStreamDMAToMsg(port);
				StreamCplt =1;
			}

			else{
				SwitchStreamDMAToMsg(port);
				SwitchStreamDMAToMsg(dst);
				StreamCplt =1;
			}
		}
	}
}

/***************************************************************************/
/* Reset UART ORE (overrun) flag in case other modules were already transmitting on startup */
void ResetUartORE(void){
#if defined(_USART1)
	__HAL_UART_CLEAR_OREFLAG(&huart1);
#endif
#if defined(_USART2)
	__HAL_UART_CLEAR_OREFLAG(&huart2);
#endif
#if defined(_USART3)
	__HAL_UART_CLEAR_OREFLAG(&huart3);
#endif
#if defined(_USART4) || defined(_UART4)
	__HAL_UART_CLEAR_OREFLAG(&huart4);
#endif
#if defined(_USART5) || defined(_UART5)
	__HAL_UART_CLEAR_OREFLAG(&huart5);
#endif
#if defined(_USART6)
	__HAL_UART_CLEAR_OREFLAG(&huart6);
#endif
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
