/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : H01R0.c
    Description   : Source code for module H01R0.
										RGB LED (Cree CLVBA-FKA-CC1F1L1BB7R3R3)
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4,5,6 for module ports.
			>> Timer 2 for RGB LED periodic timer.
			>> Timer 3 for RGB Red LED dutycycle.
			>> Timer 14 for RGB Green LED dutycycle.
			>> Timer 16 for RGB Blue LED dutycycle.
			>> GPIOB 0,1,2 for RGB LED.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/
#ifdef H01R0
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
uint8_t globalRed = 0, globalGreen = 0, globalBlue = 0;
#endif
#ifdef H01R1
TIM_HandleTypeDef htim3;
#endif
uint8_t RGB_LED_State = 0;
uint8_t RGB_LED_Intensity_Old = 0;

/* Private function prototypes -----------------------------------------------*/	
#ifdef H01R0
void TIM2_Init(void);
void TIM3_Init(void);
void TIM14_Init(void);
void TIM16_Init(void);
H01R0_Status RGB_LED_intensity(uint8_t intensityRed, uint8_t intensityGreen, uint8_t intensityBlue);
#endif
#ifdef H01R1
void TIM3_Init(void);
#endif

/* Create CLI commands --------------------------------------------------------*/

/* CLI command structure : on */
const CLI_Command_Definition_t onCommandDefinition =
{
	( const int8_t * ) "on", /* The command string to type. */
	( const int8_t * ) "(H01R0) on:\r\n Turn RGB LED on (white color) at a specific intensity (0-100%)\r\n\r\n",
	onCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : off */
const CLI_Command_Definition_t offCommandDefinition =
{
	( const int8_t * ) "off", /* The command string to type. */
	( const int8_t * ) "(H01R0) off:\r\n Turn RGB LED off\r\n\r\n",
	offCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : color */
const CLI_Command_Definition_t colorCommandDefinition =
{
	( const int8_t * ) "color", /* The command string to type. */
	( const int8_t * ) "(H01R0) color:\r\n Set RGB LED color (1st par.) at a specific intensity (0-100%) (2nd par.)\n\rRegistered colors are:\
\r\nBLACK, WHITE, RED, BLUE, GREEN, YELLOW, CYAN, and MAGENTA \r\n\r\n",
	colorCommand, /* The function to run. */
	2 /* Two parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : RGB */
const CLI_Command_Definition_t RGBCommandDefinition =
{
	( const int8_t * ) "RGB", /* The command string to type. */
	( const int8_t * ) "(H01R0) RGB:\r\n Set RGB LED red (1st par.), green (2nd par.), and blue (3rd par.) values (0-255) at a specific intensity (0-100%) (4th par.)\r\n\r\n",
	RGBCommand, /* The function to run. */
	4 /* Four parameters are expected. */
};
/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

#ifdef H01R0
/* TIM2 init function - LED periodic 1 msec timebase */
void TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM2_CLK_ENABLE();

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	/* Timer base configuration */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);
	
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}
/*-----------------------------------------------------------*/
/* TIM3 init function - Red LED dutycycle */
void TIM3_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM3_CLK_ENABLE();

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/* Timer base configuration */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);
	
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}
/* TIM14 init function - Green LED dutycycle */
void TIM14_Init(void)
{
	/* Peripheral clock enable */
	__TIM14_CLK_ENABLE();

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM14_IRQn);

	/* Timer base configuration */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);
}
/* TIM16 init function - Red LED dutycycle */
void TIM16_Init(void)
{
	/* Peripheral clock enable */
	__TIM16_CLK_ENABLE();

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(TIM16_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM16_IRQn);

	/* Timer base configuration */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);
}
/*-----------------------------------------------------------*/
/* This function handles TIM2 global interrupt (LED periodic base).
*/
void TIM2_IRQHandler(void)
{	
	HAL_TIM_IRQHandler(&htim2);
	
	if (globalRed) {
		HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	if (globalGreen) {
		HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim14);
	}
	if (globalBlue) {
		HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim16);
	}
}
/*-----------------------------------------------------------*/
/* This function handles TIM3 global interrupt (Red LED dutycycle).
*/
void TIM3_IRQHandler(void)
{	
	HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_RESET);
	
	HAL_TIM_IRQHandler(&htim3);
	
	HAL_TIM_Base_Stop_IT(&htim3);
}
/* This function handles TIM14 global interrupt (Green LED dutycycle).
*/
void TIM14_IRQHandler(void)
{	
	HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_RESET);
	
	HAL_TIM_IRQHandler(&htim14);
	
	HAL_TIM_Base_Stop_IT(&htim14);
}
/* This function handles TIM16 global interrupt (Blue LED dutycycle).
*/
void TIM16_IRQHandler(void)
{	
	HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_RESET);
	
	HAL_TIM_IRQHandler(&htim16);
	
	HAL_TIM_Base_Stop_IT(&htim16);
}
/*-----------------------------------------------------------*/
/* Calculate LED intensity dutycycle (1-100%) and restart the timers.
*/
H01R0_Status RGB_LED_intensity(uint8_t intensityRed, uint8_t intensityGreen, uint8_t intensityBlue)
{
	H01R0_Status result = H01R0_OK;
	
	if (globalRed && (intensityRed > 0) && (intensityRed < 100)) 
	{
		/* Periodic timer */
		HAL_TIM_Base_Start_IT(&htim2);
		/* Dutycycle (percentage of 1 msec) */
		htim3.Init.Period = intensityRed * 10;		
		HAL_TIM_Base_Init(&htim3);
		HAL_TIM_Base_Start_IT(&htim3);
	} 
	else if (globalRed && intensityRed != 100) {
		result = H01R0_ERR_WrongIntensity;
		return result;
	}

	if (globalGreen && (intensityGreen > 0) && (intensityGreen < 100)) 
	{
		/* Periodic timer */
		HAL_TIM_Base_Start_IT(&htim2);
		/* Dutycycle (percentage of 1 msec) */
		htim14.Init.Period = intensityGreen * 10;		
		HAL_TIM_Base_Init(&htim14);
		HAL_TIM_Base_Start_IT(&htim14);
	} 
	else if (globalGreen && intensityGreen != 100) {
		result = H01R0_ERR_WrongIntensity;
		return result;
	}

	if (globalBlue && (intensityBlue > 0) && (intensityBlue < 100)) 
	{
		/* Periodic timer */
		HAL_TIM_Base_Start_IT(&htim2);
		/* Dutycycle (percentage of 1 msec) */
		htim16.Init.Period = intensityBlue * 10;		
		HAL_TIM_Base_Init(&htim16);
		HAL_TIM_Base_Start_IT(&htim16);
	} 
	else if (globalBlue && intensityBlue != 100) {
		result = H01R0_ERR_WrongIntensity;
		return result;
	}
	
	return result;
}
#endif
/*-----------------------------------------------------------*/
#ifdef H01R1
/* TIM3 init function - Front-end RED LED PWM Timer 16-bit 
*/
void TIM3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
	
	/* Peripheral clock enable */
  __TIM3_CLK_ENABLE();
	
	/**TIM3 GPIO Configuration    
	PB0     ------> TIM3_CH3 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral interrupt init */
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/* Timer base configuration */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

	/* Timer PWM configuration */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

}
/*-----------------------------------------------------------*/
/* --- Load and start red LED PWM ---
			Inputs:
					period Signal period in usec.
					width Width of 'on' signal in usec.
*/
void startPWM_RED(uint16_t period, uint16_t width)
{
		htim3.Instance->CCR3 = width;
	
		htim3.Init.Period = period;
		HAL_TIM_Base_Init(&htim3);
	
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);		
}
/*-----------------------------------------------------------*/
/* This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{	
		HAL_TIM_IRQHandler(&htim3);
}
#endif	

/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/
#ifdef H01R0
/* --- H01R0 module initialization. 
*/
void H01R0_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Peripheral clock enable */
	__GPIOB_CLK_ENABLE();
	
	/* RGB LED GPIO */
	GPIO_InitStruct.Pin = _RGB_LED_RED | _RGB_LED_GREEN | _RGB_LED_BLUE;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(_RGB_LED_PORT, &GPIO_InitStruct);
	
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	
	/* RGB LED Periodic timer */
	TIM2_Init();
	/* Red LED dutycycle */
	TIM3_Init();
	/* Green LED dutycycle */
	TIM14_Init();
	/* Blue LED dutycycle */
	TIM16_Init();
}
#endif
#ifdef H01R1
/* --- H01R1 module initialization. 
*/
void H01R1_Init(void)
{
	/* LED PWM Timers */
	TIM3_Init();
	//TIM14_Init();
	//TIM16_Init();
}
#endif

/*-----------------------------------------------------------*/
#ifdef H01R0
/* --- Set RGB colors on LED (continuously) using PWM intensity modulation. 
*/
H01R0_Status RGB_LED_setRGB(uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity)
{
	H01R0_Status result = H01R0_OK;
	uint8_t intensityRed = 0, intensityGreen = 0, intensityBlue = 0;

	if (!intensity) {	
		RGB_LED_off();
	} else if (intensity > 100) {
		result = H01R0_ERR_WrongIntensity;
		return result;
	} else {
			
		/* Stop the timers */
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_Base_Stop_IT(&htim14);
		HAL_TIM_Base_Stop_IT(&htim16);
		
		/* RED LED */
		if (red == 0) {
			globalRed = 0;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_RESET);
		} else if (red == 255) {
			globalRed = 1; intensityRed = intensity;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_SET);
		} else if (red > 0 || red < 255) {
			globalRed = 1; intensityRed = ((float)red/255)*intensity;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_SET);
		} else {
			result = H01R0_ERR_WrongColor;
			return result;
		}		
			
		/* GREEN LED */
		if (green == 0) {
			globalGreen = 0;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_RESET);
		} else if (green == 255) {
			globalGreen = 1; intensityGreen = intensity;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_SET);
		} else if (green > 0 || green < 255) {
			globalGreen = 1; intensityGreen = ((float)green/255)*intensity;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_SET);
		} else {
			result = H01R0_ERR_WrongColor;
			return result;
		}	

		/* BLUE LED */
		if (blue == 0) {
			globalBlue = 0;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_RESET);
		} else if (blue == 255) {
			globalBlue = 1; intensityBlue = intensity;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_SET);
		} else if (blue > 0 || blue < 255) {
			globalBlue = 1; intensityBlue = ((float)blue/255)*intensity;
			HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_SET);
		} else {
			result = H01R0_ERR_WrongColor;
			return result;
		}	
		
		/* Adjust intensity */
		result = RGB_LED_intensity(intensityRed, intensityGreen, intensityBlue);
	}
	
	return result;
}
#endif
/*-----------------------------------------------------------*/

/* --- Set LED color from a predefined color list (continuously) 
				using PWM intensity modulation. 
*/
H01R0_Status RGB_LED_setColor(uint8_t color, uint8_t intensity)
{
	H01R0_Status result = H01R0_OK;

	if (!intensity) {	
		RGB_LED_off();	
	} else {
			
		switch (color)
		{
			case BLACK 		: result = RGB_LED_off();
				break;
			case WHITE 		: result = RGB_LED_on(intensity);
				break;
			case RED 			: result = RGB_LED_setRGB(255,0,0,intensity);
				break;
			case BLUE 		: result = RGB_LED_setRGB(0,0,255,intensity);
				break;
			case YELLOW 	: result = RGB_LED_setRGB(255,255,0,intensity);
				break;
			case CYAN 		: result = RGB_LED_setRGB(0,255,255,intensity);
				break;
			case MAGENTA 	: result = RGB_LED_setRGB(255,0,255,intensity);
				break;
			case GREEN 		: result = RGB_LED_setRGB(0,255,0,intensity);
				break;
			default				:	result = H01R0_ERR_WrongColor;
				break;
		}
	}
	return result;
}

/*-----------------------------------------------------------*/

/* --- Turn on RGB LED (white color) --- 
*/
H01R0_Status RGB_LED_on(uint8_t intensity)
{
	H01R0_Status result = H01R0_OK;
#ifdef H01R0
	
	if (!intensity) {	
		RGB_LED_off();	
	} else {
		
		/* Stop the timers */
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_Base_Stop_IT(&htim14);
		HAL_TIM_Base_Stop_IT(&htim16);
		globalRed = 1; globalGreen = 1; globalBlue = 1;
		
		HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_SET);
		HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_SET);
		
		/* Calculate dutycycle and restart the timers */
		result = RGB_LED_intensity(intensity, intensity, intensity);	
	}
	
	/* Update LED state */
	RGB_LED_State = 1;
	RGB_LED_Intensity_Old = intensity;	
	
#endif
	return result;
}

/*-----------------------------------------------------------*/

/* --- Turn off RGB LED --- 
*/
H01R0_Status RGB_LED_off(void)
{
	H01R0_Status result = H01R0_OK;
#ifdef H01R0
	globalRed = 0; globalGreen = 0; globalBlue = 0;
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Base_Stop_IT(&htim14);
	HAL_TIM_Base_Stop_IT(&htim16);
	HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_RED, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_GREEN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_RGB_LED_PORT, _RGB_LED_BLUE, GPIO_PIN_RESET);

	/* Update LED state */
	RGB_LED_State = 0;
	RGB_LED_Intensity_Old = 0;
#endif
	return result;
}

/*-----------------------------------------------------------*/

/* --- Toggle RGB LED --- 
*/
H01R0_Status RGB_LED_toggle(uint8_t intensity)
{
	H01R0_Status result = H01R0_OK;
	
	if (RGB_LED_State)
		result = RGB_LED_off();
	else
		result = RGB_LED_on(intensity);
	
	return result;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	
	int8_t *pcParameterString1; portBASE_TYPE xParameterStringLength1 = 0; 
	uint8_t intensity = 0;
	static const int8_t *pcOKMessage = ( int8_t * ) "RGB LED is on at intensity %d%%\r\n";
	static const int8_t *pcWrongIntensityMessage = ( int8_t * ) "Wrong intensity!\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,		/* The command string itself. */
									1,						/* Return the first parameter. */
									&xParameterStringLength1	/* Store the parameter string length. */
								);
	intensity = ( uint8_t ) atol( ( char * ) pcParameterString1 );
	
	result = RGB_LED_on(intensity);	
	
	/* Respond to the command */
	if (result == H01R0_OK)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOKMessage, intensity);
	else if (result == H01R0_ERR_WrongIntensity)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongIntensityMessage);
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE offCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage = ( int8_t * ) "RGB LED is off\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the command */
	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessage);
	RGB_LED_off();	
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE colorCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	uint8_t color = 0; uint8_t intensity = 0; 
	static int8_t *pcParameterString1, *pcParameterString2; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;
	
	static const int8_t *pcOKMessage = ( int8_t * ) "RGB LED color is %s at intensity %d%%\n\r";
	static const int8_t *pcWrongColorMessage = ( int8_t * ) "Unknown color!\n\r";
	static const int8_t *pcWrongIntensityMessage = ( int8_t * ) "Wrong intensity!\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,		/* The command string itself. */
									1,						/* Return the first parameter. */
									&xParameterStringLength1	/* Store the parameter string length. */
								);
	/* Read the color value. */
	if (!strncmp((const char *)pcParameterString1, "BLACK", xParameterStringLength1) || !strncmp((const char *)pcParameterString1, "black", xParameterStringLength1))
		color = BLACK;
	else if (!strncmp((const char *)pcParameterString1, "WHITE", xParameterStringLength1) || !strncmp(( const char *)pcParameterString1, "white", xParameterStringLength1))
		color = WHITE;
	else if (!strncmp((const char *)pcParameterString1, "RED", xParameterStringLength1) || !strncmp((const char *)pcParameterString1, "red", xParameterStringLength1))
		color = RED;
	else if (!strncmp((const char *) pcParameterString1, "BLUE", xParameterStringLength1) || !strncmp((const char *) pcParameterString1, "blue", xParameterStringLength1))
		color = BLUE;
	else if (!strncmp((const char *) pcParameterString1, "YELLOW", xParameterStringLength1) || !strncmp((const char *) pcParameterString1, "yellow", xParameterStringLength1))
		color = YELLOW;
	else if (!strncmp((const char *) pcParameterString1, "CYAN", xParameterStringLength1) || !strncmp((const char *) pcParameterString1, "cyan", xParameterStringLength1))
		color = CYAN;
	else if (!strncmp((const char *) pcParameterString1, "MAGENTA", xParameterStringLength1) || !strncmp((const char *) pcParameterString1, "magenta", xParameterStringLength1))
		color = MAGENTA;
	else if (!strncmp((const char *) pcParameterString1, "GREEN", xParameterStringLength1) || !strncmp((const char *) pcParameterString1, "green", xParameterStringLength1))
		color = GREEN;
	
	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	intensity = ( uint8_t ) atol( ( char * ) pcParameterString2 );
	
	result = RGB_LED_setColor(color, intensity);
	
	/* Respond to the command */
	if (result == H01R0_OK) 
	{
		/* Isolate first parameter string. Null character manually added */
		pcParameterString1[xParameterStringLength1] = '\0'; 
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOKMessage, pcParameterString1, intensity);
	}
	else if (result == H01R0_ERR_WrongColor)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongColorMessage);
	else if (result == H01R0_ERR_WrongIntensity)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongIntensityMessage);
	

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE RGBCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	uint8_t red = 0; uint8_t green = 0; uint8_t blue = 0; uint8_t intensity = 0; 
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;
	portBASE_TYPE xParameterStringLength3 = 0, xParameterStringLength4 = 0;
	
	static const int8_t *pcOKMessage = ( int8_t * ) "RGB LED is (%d, %d, %d) at intensity %d%%\n\r";
	static const int8_t *pcWrongColorMessage = ( int8_t * ) "Wrong color value!\n\r";
	static const int8_t *pcWrongIntensityMessage = ( int8_t * ) "Wrong intensity!\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	red = ( uint8_t ) atol( ( char * ) pcParameterString1 );
	
	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	green = ( uint8_t ) atol( ( char * ) pcParameterString2 );
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	blue = ( uint8_t ) atol( ( char * ) pcParameterString3 );
	
	/* Obtain the 4th parameter string. */
	pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
	intensity = ( uint8_t ) atol( ( char * ) pcParameterString4 );
	
	result = RGB_LED_setRGB(red, green, blue, intensity);
	
	/* Respond to the command */
	if (result == H01R0_OK) 
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOKMessage, red, green, blue, intensity);
	else if (result == H01R0_ERR_WrongColor)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongColorMessage);
	else if (result == H01R0_ERR_WrongIntensity)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongIntensityMessage);
	

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
