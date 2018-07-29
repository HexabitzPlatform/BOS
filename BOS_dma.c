/*
    BitzOS (BOS) V0.1.4 - Copyright (C) 2018 Hexabitz
    All rights reserved

    File Name     : BOS_dma.c
    Description   : Source code for BOS communication/backend DMAs. Also provides APIs for front-end DMA use.
		
		Required MCU resources : 
		
			>> At least n UART RX DMA channels where n is number of module ports (up to number of available UARTs).
			>> At least one UART TX DMA channel.
			>> Any extra channels can be assigned to front-end.

*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "BOS_DMA.h"


/* Exported variables ---------------------------------------------------------*/

uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE] = {0};
uint8_t UARTTxBuf[3][MSG_TX_BUF_SIZE] = {0};

/* Private variables ---------------------------------------------------------*/




/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
