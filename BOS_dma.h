/*
    BitzOS (BOS) V0.1.4 - Copyright (C) 2018 Hexabitz
    All rights reserved
		
    File Name     : BOS_dma.h
    Description   : Header file for for BOS communication/backend DMAs. Also provides APIs for front-end DMA use.
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_DMA_H
#define BOS_DMA_H


/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* External definitions -------------------------------------------------------*/
#define MSG_DMA_PRIORITY 					DMA_PRIORITY_HIGH					// Messaging backend priority
#define STREAM_DMA_PRIORITY 			DMA_PRIORITY_MEDIUM
#define FRONTEND_DMA_PRIORITY 		DMA_PRIORITY_LOW


/* External variables ---------------------------------------------------------*/
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
extern uint8_t UARTTxBuf[3][MSG_TX_BUF_SIZE]


/* External functions ---------------------------------------------------------*/






#endif /* BOS_DMA_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
