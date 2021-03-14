/*
    BitzOS (BOS) V0.2.4 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved

    File Name     : BOS_inputs.c
    Description   : Source code for Bitz digital and analog inputs.
	
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS_inputs.h"

/* Private and global variables ----------------------------------------------*/
/* Buttons */
button_t button[NumOfPorts+1] = {0};
uint32_t pressCounter[NumOfPorts+1] = {0};
uint32_t releaseCounter[NumOfPorts+1] = {0};
uint8_t dblCounter[NumOfPorts+1] = {0};
bool needToDelayButtonStateReset = false, delayButtonStateReset = false;
ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef sConfig = {0};

/* Private buttons function prototypes -----------------------------------------------*/
BOS_Status CheckForTimedButtonPress(uint8_t port);
BOS_Status CheckForTimedButtonRelease(uint8_t port);
extern BOS_Status GetPortGPIOs(uint8_t port, uint32_t *TX_Port, uint16_t *TX_Pin, uint32_t *RX_Port, uint16_t *RX_Pin);
void buttonPressedCallback(uint8_t port);
void buttonReleasedCallback(uint8_t port);
void buttonClickedCallback(uint8_t port);
void buttonDblClickedCallback(uint8_t port);
void buttonPressedForXCallback(uint8_t port, uint8_t eventType);
void buttonReleasedForYCallback(uint8_t port, uint8_t eventType);

/* Private ADC function prototypes -----------------------------------------------*/

void MX_ADC_Init(void);
void Error_Handler(void);
uint8_t Get_channel(UART_HandleTypeDef *huart, char * side);
uint8_t Get_Rank(uint8_t Port, char* side);
void ReadTempAndVref(float *temp, float *Vref);
void ReadADCChannel(uint8_t Port , char * side,float *ADC_Value);
void ADCSelectChannel(uint8_t ADC_port, char* side);

/* Private ADC variables -----------------------------------------------*/

#define Vref_Cal ((uint16_t *)((uint32_t)0x1ffff7BA))
#define V25  1.41
#define Avg_Slope 4.3
uint8_t Channel=0;;
uint16_t ADCchannelvalue[4]={0};
uint16_t ADC_value_temp=0;
uint16_t ADC_value_Vref=0;
uint8_t ADC_flag=0,Rank_t=0;
/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- Button press callback. DO NOT MODIFY THIS CALLBACK. 
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonPressedCallback(uint8_t port)
{	
}

/*-----------------------------------------------------------*/	

void CheckAttachedButtons(void)
{
	uint32_t TX_Port, RX_Port; 
	uint16_t TX_Pin, RX_Pin;
	uint8_t connected = GPIO_PIN_RESET, state = 0;
	static uint8_t clicked;
	
	for(uint8_t i=1 ; i<=NumOfPorts ; i++)
	{
		if (button[i].type)			// Only check defined butons
		{
			/* 1. Reset button state */
			if (delayButtonStateReset == false)	button[i].state = NONE;		

			/* 2. Get button GPIOs */
			GetPortGPIOs(i, &TX_Port, &TX_Pin, &RX_Port, &RX_Pin);
			
			/* 3. Check if port pins are connected */
			HAL_GPIO_WritePin((GPIO_TypeDef *)TX_Port, TX_Pin, GPIO_PIN_RESET); Delay_us(10);
			if (HAL_GPIO_ReadPin((GPIO_TypeDef *)RX_Port, RX_Pin) == GPIO_PIN_RESET) 
			{
				HAL_GPIO_WritePin((GPIO_TypeDef *)TX_Port, TX_Pin, GPIO_PIN_SET); Delay_us(10);
				connected = HAL_GPIO_ReadPin((GPIO_TypeDef *)RX_Port, RX_Pin); 
			}		
			HAL_GPIO_WritePin((GPIO_TypeDef *)TX_Port, TX_Pin, GPIO_PIN_RESET);
			
			/* 4. Determine button state based on port reading and button type */
			switch (button[i].type)
      {
      	case MOMENTARY_NO:
					if (connected == GPIO_PIN_SET)	
						state = CLOSED;
					else if (connected == GPIO_PIN_RESET)
						state = OPEN;			
      		break;
				
      	case MOMENTARY_NC:
					if (connected == GPIO_PIN_SET)	
						state = CLOSED;
					else if (connected == GPIO_PIN_RESET) 
						state = OPEN;	
      		break;
				
      	case ONOFF_NO:
					if (connected == GPIO_PIN_SET)	
						state = ON;
					else if (connected == GPIO_PIN_RESET) 
						state = OFF;
      		break;
				
      	case ONOFF_NC:
					if (connected == GPIO_PIN_SET)	
						state = OFF;
					else if (connected == GPIO_PIN_RESET) 
						state = ON;
      		break;
				
      	default:
      		break;
      }
			
			/* 5. Debounce this state and update button struct if needed */		
			
			/* 5.A. Possible change of state 1: OPEN > CLOSED or OFF >> ON */
			if (state == CLOSED || state == ON)												
			{
				if (pressCounter[i] < 0xFFFF)	
					++pressCounter[i];																			// Advance the debounce counter
				else	
					pressCounter[i] = 0;																		// Reset debounce counter					
			}
			
			/* 5.B. Possible change of state 2: CLOSED > OPEN or ON >> OFF */
			if (state == OPEN || state == OFF)												
			{
				if (releaseCounter[i] < 0xFFFF)
					++releaseCounter[i];																		// Advance the debounce counter
				else	
					releaseCounter[i] = 0;																	// Reset debounce counter		
				
				if (clicked == 2 && dblCounter[i] <= BOS.buttons.maxInterClickTime)				// Advance the inter-click counter		
					++dblCounter[i];			
				else if (dblCounter[i] > BOS.buttons.maxInterClickTime)	{
					clicked = 0;
					dblCounter[i] = 0;																			// Reset the inter-click counter
				}					
			}
			
			/* Analyze state */
			
			/* 5.C. On press: Record a click if pressed less than 1 second */
			if (pressCounter[i] < BOS.buttons.debounce) 									
			{
				// This is noise. Ignore it
			} 
			else 
			{
				if (pressCounter[i] == BOS.buttons.debounce)
				{
					button[i].state = PRESSED;															// Record a PRESSED event. This event is always reset on next tick.
					++pressCounter[i];
				}
				
				if (releaseCounter[i] > BOS.buttons.debounce)							// Reset releaseCounter if needed - to avoid masking pressCounter on NO switches
					releaseCounter[i] = 0;					
				
				if (pressCounter[i] > BOS.buttons.singleClickTime && pressCounter[i] < 500)	
				{
					if (clicked == 0)
						clicked = 1;																					// Record a possible single click 
					else if (clicked == 2) {
						if (dblCounter[i] > BOS.buttons.minInterClickTime && dblCounter[i] < BOS.buttons.maxInterClickTime) {
							clicked = 3;																				// Record a possible double click 
							dblCounter[i] = 0;																	// Reset the inter-click counter
						}
					}						
				}								
				else if (pressCounter[i] >= 500 && pressCounter[i] < 0xFFFF)	
				{
					if (clicked)	clicked = 0;															// Cannot be a click
					// Process PRESSED_FOR_X_SEC events
					CheckForTimedButtonPress(i);
				}	
			}
			
			/* 5.D. On release: Record a click if pressed less than 1 second */
			if (releaseCounter[i] < BOS.buttons.debounce) 							
			{
				// This is noise. Ignore it
			} 	
			else 
			{
				if (releaseCounter[i] == BOS.buttons.debounce)
				{
					button[i].state = RELEASED;															// Record a RELEASED event. This event is always reset on next tick.
					++releaseCounter[i];
				}
				
				if (pressCounter[i] > BOS.buttons.debounce)								// Reset pressCounter if needed - to avoid masking releaseCounter on NC switches
					pressCounter[i] = 0;				
				
				if (releaseCounter[i] > BOS.buttons.singleClickTime && releaseCounter[i] < 500)	
				{
					if (clicked == 1)
					{
						button[i].state = CLICKED;														// Record a single button click event
						clicked = 2;																					// Prepare for a double click
					}
					else if (clicked == 3)
					{
						button[i].state = DBL_CLICKED;												// Record a double button click event
						clicked = 0;																					// Prepare for a single click					
					}
				}					
				else if (releaseCounter[i] >= 500 && releaseCounter[i] < 0xFFFF)	
				{
					// Process RELEASED_FOR_Y_SEC events
					CheckForTimedButtonRelease(i);
				}	
			}	
			
			/* 6. Run button callbacks if needed */
			switch (button[i].state)
      {
      	case PRESSED :
					buttonPressedCallback(i);
					button[i].state = NONE;
      		break;
				
      	case RELEASED :
					buttonReleasedCallback(i);
					button[i].state = NONE;
      		break;
				
      	case CLICKED :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_CLICKED)) 
					{
						delayButtonStateReset = true;
						buttonClickedCallback(i);
					}
      		break;
				
      	case DBL_CLICKED :				
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_DBL_CLICKED)) 
					{
						delayButtonStateReset = true;
						buttonDblClickedCallback(i);
					}
      		break;
					
      	case PRESSED_FOR_X1_SEC :		
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_PRESSED_FOR_X1_SEC)) 
					{				
						delayButtonStateReset = true;
						buttonPressedForXCallback(i, PRESSED_FOR_X1_SEC-8);
					}
					break;
				case PRESSED_FOR_X2_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_PRESSED_FOR_X2_SEC)) 
					{
						delayButtonStateReset = true;
						buttonPressedForXCallback(i, PRESSED_FOR_X2_SEC-8);
					}
					break;
				case PRESSED_FOR_X3_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_PRESSED_FOR_X3_SEC)) 
					{
						delayButtonStateReset = true;
						buttonPressedForXCallback(i, PRESSED_FOR_X3_SEC-8);
					}
					break;
				
      	case RELEASED_FOR_Y1_SEC :	
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_RELEASED_FOR_Y1_SEC)) 
					{
						delayButtonStateReset = true;
						buttonReleasedForYCallback(i, RELEASED_FOR_Y1_SEC-11);
					}
					break;		
					
				case RELEASED_FOR_Y2_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_RELEASED_FOR_Y2_SEC)) 
					{	
						delayButtonStateReset = true;
						buttonReleasedForYCallback(i, RELEASED_FOR_Y2_SEC-11);
					}
					break;			
					
				case RELEASED_FOR_Y3_SEC :
					if (!delayButtonStateReset && (button[i].events & BUTTON_EVENT_RELEASED_FOR_Y3_SEC)) 
					{	
						delayButtonStateReset = true;
						buttonReleasedForYCallback(i, RELEASED_FOR_Y3_SEC-11);
					}
					break;
				
      	default:
      		break;
      }	
		}					// Done checking this button
	}						// Done checking all buttons
	
}

/*-----------------------------------------------------------*/	

/* --- Check for timed press button events
*/
BOS_Status CheckForTimedButtonPress(uint8_t port)
{
	BOS_Status result = BOS_OK;
	uint32_t t1 = button[port].pressedX1Sec, t2 = button[port].pressedX2Sec, t3 = button[port].pressedX3Sec;
	
	/* Convert to ms */
	t1 *= 1000; t2 *= 1000; t3 *= 1000;
	
	if (pressCounter[port] == t1)	
	{	
		button[port].state = PRESSED_FOR_X1_SEC;
	}
	else if (pressCounter[port] == t2)	
	{	
		button[port].state = PRESSED_FOR_X2_SEC;
	}		
	else if (pressCounter[port] == t3)	
	{	
		button[port].state = PRESSED_FOR_X2_SEC;
	}	

	return result;	
}

/*-----------------------------------------------------------*/	

/* --- Check for timed release button events
*/
BOS_Status CheckForTimedButtonRelease(uint8_t port)
{
	BOS_Status result = BOS_OK;
	uint32_t t1 = button[port].releasedY1Sec, t2 = button[port].releasedY2Sec, t3 = button[port].releasedY3Sec;

	/* Convert to ms */
	t1 *= 1000; t2 *= 1000; t3 *= 1000;
	
	if (releaseCounter[port] == t1)	
	{	
		button[port].state = RELEASED_FOR_Y1_SEC;
	}
	else if (releaseCounter[port] == t2)	
	{	
		button[port].state = RELEASED_FOR_Y2_SEC;
	}		
	else if (releaseCounter[port] == t3)	
	{	
		button[port].state = RELEASED_FOR_Y2_SEC;
	}	

	return result;	
}

/*-----------------------------------------------------------*/	

/* --- Reset state of attached buttons to avoid recurring callbacks
*/
void ResetAttachedButtonStates(uint8_t *deferReset)
{
	if (!*deferReset)
	{
		for(uint8_t i=1 ; i<=NumOfPorts ; i++)
		{
			if(button[i].state != NONE)
				button[i].state = NONE;
		}
	}	
	//*deferReset = 0;
}

/*-----------------------------------------------------------*/

/* --- Define a new button attached to one of array ports
					buttonType: MOMENTARY_NO, MOMENTARY_NC, ONOFF_NO, ONOFF_NC
					port: array port (P1 - Px)
*/
BOS_Status AddPortButton(uint8_t buttonType, uint8_t port)
{
	BOS_Status result = BOS_OK;
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t TX_Port, RX_Port; 
	uint16_t TX_Pin, RX_Pin, temp16, res;
	uint8_t temp8 = 0;
	
	/* 1. Stop communication at this port (only if the scheduler is running) - TODO update*/
	if (BOS_initialized) {
		osSemaphoreRelease(PxRxSemaphoreHandle[port]);		/* Give back the semaphore if it was taken */
		osSemaphoreRelease(PxTxSemaphoreHandle[port]);
	}
	portStatus[port] = PORTBUTTON;	
	
	/* 2. Deinitialize UART (only if module is initialized) */
	if (BOS_initialized) {
		HAL_UART_DeInit(GetUart(port));
	}
	
	/* 3. Initialize GPIOs */
	GetPortGPIOs(port, &TX_Port, &TX_Pin, &RX_Port, &RX_Pin);		
	/* Ouput (TXD) */
	GPIO_InitStruct.Pin = TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init((GPIO_TypeDef *)TX_Port, &GPIO_InitStruct);
	/* Input (RXD) */
	GPIO_InitStruct.Pin = RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init((GPIO_TypeDef *)RX_Port, &GPIO_InitStruct);

	/* 4. Update button struct */
	button[port].type = buttonType;	
	
	/* 5. Add to EEPROM if not already there */
	res = EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1), &temp16);
	if(!res)																														// This variable exists
	{
		temp8 = (uint8_t)(temp16 >> 8);
		if ( ((temp8 >> 4) == port) && ((temp8 & 0x0F) == buttonType) )		// This is same port and same type, do not update
			return BOS_OK;
		else 																															// Update the variable
		{																														
			temp16 = ((uint16_t)port << 12) | ((uint16_t)buttonType << 8);
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), temp16);
			/* Reset times */
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, 0);
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, 0);
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, 0);
		}
	}
	else																																// Variable does not exist. Create a new one
	{
	  	temp16 = ((uint16_t)port << 12) | ((uint16_t)buttonType << 8);
	  	EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), temp16);		
	  	/* Reset times */
	  	EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, 0);
	  	EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, 0);
	  	EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, 0);
	}
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Undefine a button attached to one of array ports and restore the port to default state
					port: array port (P1 - Px)
*/
BOS_Status RemovePortButton(uint8_t port)
{
	BOS_Status result = BOS_OK;
	uint16_t res, temp16;
	
	/* 1. Remove from button struct */
	button[port].type = NONE;
	button[port].state = NONE;
	button[port].events = 0;
	button[port].pressedX1Sec = 0; button[port].pressedX2Sec = 0; button[port].pressedX3Sec = 0;
	button[port].releasedY1Sec = 0; button[port].releasedY2Sec = 0; button[port].releasedY3Sec = 0;
	
	/* 2. Remove from EEPROM if it's already there */
	res = EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1), &temp16);
	if(!res)																														// This variable exists, reset all to zeros
	{
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), 0);
		/* Reset times */
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, 0);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, 0);
		EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, 0);		
	}
	
	/* 3. Initialize UART at this port */
	UART_HandleTypeDef* huart = GetUart(port);
	
	if (huart->Instance == USART1) 
	{	
#ifdef _Usart1		
		MX_USART1_UART_Init();
#endif
	} 
	else if (huart->Instance == USART2) 
	{	
#ifdef _Usart2	
		MX_USART2_UART_Init();
#endif
	} 
	else if (huart->Instance == USART3) 
	{	
#ifdef _Usart3	
		MX_USART3_UART_Init();
#endif
	} 
	else if (huart->Instance == USART4) 
	{	
#ifdef _Usart4	
		MX_USART4_UART_Init();
#endif
	} 
	else if (huart->Instance == USART5) 
	{	
#ifdef _Usart5	
		MX_USART5_UART_Init();
#endif
	} 
	else if (huart->Instance == USART6) 
	{	
#ifdef _Usart6	
		MX_USART6_UART_Init();
#endif
	} 
	else if (huart->Instance == USART7) 
	{	
#ifdef _Usart7	
		MX_USART7_UART_Init();
#endif
	} 
	else if (huart->Instance == USART8) 
	{	
#ifdef _Usart8	
		MX_USART8_UART_Init();
#endif
	} 
	else
		result = BOS_ERROR;			
	
	/* 4. Start scanning this port */
	portStatus[port] = FREE;
	/* Read this port again */
	HAL_UART_Receive_IT(huart, (uint8_t *)&cRxedChar, 1);	
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Setup button events and callbacks
					port: array port (P1 - Px) where the button is attached 
					clicked: Single click event (1: Enable, 0: Disable)
					dbl_clicked: Double click event (1: Enable, 0: Disable)
					pressed_x1sec, pressed_x1sec, pressed_x1sec: Press time for events X1, X2 and X3 in seconds. Use 0 to disable the event. 
					released_x1sec, released_x1sec, released_x1sec: Release time for events Y1, Y2 and Y3 in seconds. Use 0 to disable the event. 
					mode: BUTTON_EVENT_MODE_CLEAR to clear events marked with 0, BUTTON_EVENT_MODE_OR to OR events marked with 1 with existing events.
*/
BOS_Status SetButtonEvents(uint8_t port, uint8_t clicked, uint8_t dbl_clicked, uint8_t pressed_x1sec, uint8_t pressed_x2sec, uint8_t pressed_x3sec,\
													uint8_t released_y1sec, uint8_t released_y2sec, uint8_t released_y3sec, uint8_t mode)
{
	BOS_Status result = BOS_OK;	
	uint16_t res, temp16; uint8_t temp8;
	
	if (button[port].type == NONE)
		return BOS_ERR_BUTTON_NOT_DEFINED;
	
	button[port].pressedX1Sec = pressed_x1sec; button[port].pressedX2Sec = pressed_x2sec; button[port].pressedX3Sec = pressed_x3sec;
	button[port].releasedY1Sec = released_y1sec; button[port].releasedY2Sec = released_y2sec; button[port].releasedY3Sec = released_y3sec;
	
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && clicked)) {				
		button[port].events |= BUTTON_EVENT_CLICKED;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !clicked) {
		button[port].events &= ~BUTTON_EVENT_CLICKED;		
	}
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && dbl_clicked)) {		
		button[port].events |= BUTTON_EVENT_DBL_CLICKED;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !dbl_clicked) {
		button[port].events &= ~BUTTON_EVENT_DBL_CLICKED;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && pressed_x1sec)) {			
		button[port].events |= BUTTON_EVENT_PRESSED_FOR_X1_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !pressed_x1sec) {
		button[port].events &= ~BUTTON_EVENT_PRESSED_FOR_X1_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && pressed_x2sec)) {		
		button[port].events |= BUTTON_EVENT_PRESSED_FOR_X2_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !pressed_x2sec) {
		button[port].events &= ~BUTTON_EVENT_PRESSED_FOR_X2_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && pressed_x3sec)) {		
		button[port].events |= BUTTON_EVENT_PRESSED_FOR_X3_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !pressed_x3sec) {
		button[port].events &= ~BUTTON_EVENT_PRESSED_FOR_X3_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && released_y1sec)) {		
		button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y1_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !released_y1sec) {
		button[port].events &= ~BUTTON_EVENT_RELEASED_FOR_Y1_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && released_y2sec)) {		
		button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y2_SEC;
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !released_y2sec) {
		button[port].events &= ~BUTTON_EVENT_RELEASED_FOR_Y2_SEC;		
	}		
	if (mode == BUTTON_EVENT_MODE_OR || (mode == BUTTON_EVENT_MODE_CLEAR && released_y3sec)) {		
		button[port].events |= BUTTON_EVENT_RELEASED_FOR_Y3_SEC;	
	} else if (mode == BUTTON_EVENT_MODE_CLEAR && !released_y3sec) {
		button[port].events &= ~BUTTON_EVENT_RELEASED_FOR_Y3_SEC;		
	}
	
	/* Add to EEPROM */
	res = EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1), &temp16);
	if(!res)																														// This variable exists
	{
		temp8 = (uint8_t)(temp16 >> 8);																		// Keep upper byte
		/* Store event flags */
		if ((uint8_t)(temp16) != button[port].events) {										// Update only if different
			temp16 = ((uint16_t)temp8 << 8) | (uint16_t)button[port].events;
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1), temp16);
		}
		
		/* Store times - only if different */
		EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1)+1, &temp16);
		if ( temp16 != (((uint16_t)pressed_x1sec << 8) | (uint16_t) released_y1sec) )
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+1, ((uint16_t)pressed_x1sec << 8) | (uint16_t) released_y1sec);
		
		EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1)+2, &temp16);
		if ( temp16 != (((uint16_t)pressed_x2sec << 8) | (uint16_t) released_y2sec) )
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+2, ((uint16_t)pressed_x2sec << 8) | (uint16_t) released_y2sec);
		
		EE_ReadVariable(_EE_BUTTON_BASE+4*(port-1)+3, &temp16);
		if ( temp16 != (((uint16_t)pressed_x3sec << 8) | (uint16_t) released_y3sec) )
			EE_WriteVariable(_EE_BUTTON_BASE+4*(port-1)+3, ((uint16_t)pressed_x3sec << 8) | (uint16_t) released_y3sec);
	}	// TODO - var does not exist after adding button!
	else																																// Variable does not exist. Return error
		return BOS_ERR_BUTTON_NOT_DEFINED;	
		
	
	return result;
}


/* ADC init function */

/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
 * to read multiple ADC channel in Port 2 and port 3 and for calculate
 * internal temperature and internal voltage reference which is equal in stm32f0 to around 1.2v.
 *
  */
void MX_ADC_Init(void)
{
	  hadc.Instance = ADC1;
	  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc.Init.LowPowerAutoWait = DISABLE;
	  hadc.Init.LowPowerAutoPowerOff = DISABLE;
	  hadc.Init.ContinuousConvMode = ENABLE;
	  hadc.Init.DiscontinuousConvMode = DISABLE;
	  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc.Init.DMAContinuousRequests = DISABLE;
	  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;

	  if (HAL_ADC_Init(&hadc) != HAL_OK)
	  {
		  Error_Handler();
	  }
	ADC_flag=1;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC GPIO Configuration
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC GPIO Configuration
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}




/** select port 2 & port 3 for the selected ADC regular channel to be converted. */

void ADCSelectChannel(uint8_t ADC_port, char* side){

	if(ADC_port==2 || ADC_port==3){

			HAL_UART_DeInit(GetUart(ADC_port));
			portStatus[ADC_port-1] = CUSTOM;

			Channel=Get_channel(GetUart(ADC_port),side);
			Rank_t=Get_Rank(ADC_port,side);
			if(ADC_flag==0) MX_ADC_Init();
	     }

}
void ReadADCChannel(uint8_t Port , char * side,float *ADC_Value){


	if(ADC_flag==1){

			/* --- Enable chosen channel to be read.*/

				Channel=Get_channel(GetUart(Port),side);
				Rank_t=Get_Rank(Port, side);

				sConfig.Channel = Channel;
				sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
				sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
				 if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
						Error_Handler();
											}
				HAL_ADC_Start(&hadc);
				HAL_ADC_PollForConversion(&hadc,100);
				ADCchannelvalue[Rank_t] = HAL_ADC_GetValue(&hadc);

				HAL_ADC_Stop(&hadc);

				/* --- Disable chosen channel.*/
				sConfig.Channel = Channel;
				sConfig.Rank = ADC_RANK_NONE;
				sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
				if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
						Error_Handler();
					}

			}
	*ADC_Value=(float )ADCchannelvalue[Rank_t];

	}

void ReadTempAndVref(float *temp, float *Vref){

	if(0==ADC_flag) MX_ADC_Init();

	/* --- Enable internal temperature channel.*/

	   sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	   sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	   {
	     Error_Handler();
	   }

		HAL_ADC_Start(&hadc);

		HAL_ADC_PollForConversion(&hadc,100);
		ADC_value_temp = HAL_ADC_GetValue(&hadc);
		*temp =((3.3*ADC_value_temp/4095 - V25)/Avg_Slope)+25;

		   HAL_ADC_Stop(&hadc);


/* --- Disable internal temperature channel.*/

	   sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	   sConfig.Rank = ADC_RANK_NONE;
	   sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	   {
	     Error_Handler();
	   }

	   /* Enable internal Voltage Reference channel */

	   sConfig.Channel = ADC_CHANNEL_VREFINT;
	   sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	   sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
	     Error_Handler();
	   }

		HAL_ADC_Start(&hadc);

		HAL_ADC_PollForConversion(&hadc,100);
		ADC_value_Vref = HAL_ADC_GetValue(&hadc);
		*Vref = 3.3 * (*Vref_Cal)/ADC_value_Vref;

		HAL_ADC_Stop(&hadc);


	 /* Disable internal Voltage Reference channel */

	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	  sConfig.Rank = ADC_RANK_NONE;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK){
			  	     Error_Handler();
			  	   }
	}



/* --- Get the ADC_channel Number for a given UART.
	*/
uint8_t Get_channel(UART_HandleTypeDef *huart, char * side){


		 if (huart->Instance == USART2 &&  side=="top" )
			return ADC_CHANNEL_2;
		else if (huart->Instance == USART2 &&  side=="bottom" )
			return ADC_CHANNEL_3;
		else if (huart->Instance == USART6 &&  side=="top" )
			return ADC_CHANNEL_4;
		else if (huart->Instance == USART6 &&  side=="bottom" )
			return ADC_CHANNEL_5;
}

void Error_Handler(void){

HAL_Delay(100);

}

uint8_t Get_Rank(uint8_t Port, char* side){

	if(Port==2 && side=="top") Rank_t=0;
	else if(Port==2 && side== "bottom") Rank_t=1;
	else if(Port==3 && side== "top") Rank_t =2;
	else if(Port==3 && side== "bottom") Rank_t =3;
	return Rank_t;
}




void Deinit_ADC_Channel(uint8_t port){

	HAL_UART_Init(GetUart(port));
	HAL_ADC_DeInit(&hadc);
	portStatus[port-1] = FREE;
	ADC_flag=0;
}

/*-----------------------------------------------------------*/

/* --- Button release callback. DO NOT MODIFY THIS CALLBACK.
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonReleasedCallback(uint8_t port)
{
}

/*-----------------------------------------------------------*/

/* --- Button single click callback. DO NOT MODIFY THIS CALLBACK.
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonClickedCallback(uint8_t port)
{
}

/*-----------------------------------------------------------*/

/* --- Button double click callback. DO NOT MODIFY THIS CALLBACK.
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonDblClickedCallback(uint8_t port)
{
}

/*-----------------------------------------------------------*/

/* --- Button pressed_for_x callbacks. DO NOT MODIFY THIS CALLBACK.
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonPressedForXCallback(uint8_t port, uint8_t eventType)
{
}

/*-----------------------------------------------------------*/

/* --- Button released_for_y callbacks. DO NOT MODIFY THIS CALLBACK.
		This function is declared as __weak to be overwritten by other implementations in user file.
*/
__weak void buttonReleasedForYCallback(uint8_t port, uint8_t eventType)
{
}

/* --- Port buttons state parser
*/
/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
