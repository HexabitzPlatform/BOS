/*
 BitzOS (BOS) V0.3.1 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_dma.h
 Description   : Header file for for BOS communication/backend DMAs.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_DMA_H
#define BOS_DMA_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*Rx_Data[NumOfPorts]: This array is used to receive data from all ports
 *
 * Access each port Byte:
 * Rx_Data[GetPort(huart) - 1];
 */
extern uint8_t Rx_Data[NumOfPorts];

/* External definitions -------------------------------------------------------*/
#define MSG_DMA_PRIORITY 					DMA_PRIORITY_HIGH					// Messaging backend priority
#define STREAM_DMA_PRIORITY 				DMA_PRIORITY_MEDIUM
#define FRONTEND_DMA_PRIORITY 				DMA_PRIORITY_LOW

/* External variables ---------------------------------------------------------*/

extern UART_HandleTypeDef *dmaStreamDst[NumOfPorts];
extern uint32_t dmaStreamCount[NumOfPorts];
extern uint32_t dmaStreamTotal[NumOfPorts];
extern bool MsgDMAStopped[NumOfPorts];

/* External functions ---------------------------------------------------------*/

extern BOS_Status StartDMAstream(UART_HandleTypeDef *huartSrc,UART_HandleTypeDef *huartDst,uint16_t num);
extern void DMA_IRQHandler(uint8_t port);
extern void ResetUartORE(void);

#endif /* BOS_DMA_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
