/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_dma.c
 Description   : Source code for BOS communication/backend DMAs.

 Required MCU resources :

 >> At least n UART RX DMA channels where n is number of module ports (up to number of available UARTs).
 >> At least one UART TX DMA channel.
 >> Any extra channels can be assigned to front-end.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "BOS_DMA.h"

/*Rx_Data[NumOfPorts]: This array is used to receive data from all ports
 *
 * Access each port Byte:
 * Rx_Data[GetPort(huart) - 1];
 */
uint8_t Rx_Data[NumOfPorts] = {0};

/* Exported variables ---------------------------------------------------------*/

uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE] ={0};
uint8_t crcBuffer[MAX_MESSAGE_SIZE] ={0};
uint8_t UARTRxBufIndex[NumOfPorts] ={0};
UART_HandleTypeDef *dmaStreamDst[NumOfPorts] ={0};
uint32_t dmaStreamCount[NumOfPorts] ={0};
uint32_t dmaStreamTotal[NumOfPorts] ={0};
bool MsgDMAStopped[NumOfPorts] ={0};

/* Private variables ---------------------------------------------------------*/

/* External functions --------------------------------------------------------*/
extern void DMA_STREAM_Setup(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num);


/*-----------------------------------------------------------*/

/* Setup and start a streaming DMA (port-to-port) 
 */
BOS_Status StartDMAstream(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num){
	uint8_t srcPort =GetPort(huartSrc);
	
	// 1. Check if single- or multi-cast 
	// 1.a. If single-cast, switch the DMA channel to streaming if it's available 
	if(portStatus[srcPort] == FREE || portStatus[srcPort] == MSG || portStatus[srcPort] == CLI)		// This port is not streaming so it's single-cast
	{
		SwitchMsgDMAToStream(srcPort);
	}
	// 1.b. If multi-cast, do some stuff - TODO
	else if(portStatus[srcPort] == STREAM){
		return BOS_ERR_PORT_BUSY;		// Multi-casting not implemented right now
	}
	else
		return BOS_ERR_PORT_BUSY;
	
	// 2. Setup streaming destination
	dmaStreamDst[srcPort - 1] =huartDst;
	
	// 3. Lock the ports 
	portStatus[srcPort] =STREAM;
	
	// 4. Initialize stream counter 
	dmaStreamCount[srcPort - 1] =0;
	
	// 5. Setup and start the DMA stream
	DMA_STREAM_Setup(huartSrc,huartDst,num);
	
	return BOS_OK;
}

/*-----------------------------------------------------------*/

/* DMA interrupt service routine 
 */
void DMA_IRQHandler(uint8_t port){
	if(portStatus[port] != STREAM){
		HAL_DMA_IRQHandler(&msgRxDMA[port - 1]);
	}
	else{
		HAL_DMA_IRQHandler(&streamDMA[port - 1]);
		if(dmaStreamTotal[port - 1])
			++dmaStreamCount[port - 1];
		if(dmaStreamCount[port - 1] >= dmaStreamTotal[port - 1]){
			StopStreamDMA(port);
		}
	}
}

/*-----------------------------------------------------------*/

/* Reset UART ORE (overrun) flag in case other modules were already transmitting on startup
 */
void ResetUartORE(void){
#if defined(_Usart1)
	__HAL_UART_CLEAR_OREFLAG(&huart1);
#endif
#if defined(_Usart2)
	__HAL_UART_CLEAR_OREFLAG(&huart2);
#endif
#if defined(_Usart3)
	__HAL_UART_CLEAR_OREFLAG(&huart3);
#endif
#if defined(_Usart4) || defined(_Uart4)
	__HAL_UART_CLEAR_OREFLAG(&huart4);
#endif
#if defined(_Usart5) || defined(_Uart5)
	__HAL_UART_CLEAR_OREFLAG(&huart5);
#endif
#if defined(_Usart6)
	__HAL_UART_CLEAR_OREFLAG(&huart6);
#endif
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
