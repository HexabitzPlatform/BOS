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
uint8_t rgbLedMode = 0;
uint8_t rgbRed = 0, rgbGreen = 0, rgbBlue = 0, rgbColor = 0; 
uint32_t rgbPeriod = 0, rgbDC = 0; int16_t rgbCount = 0;

extern TaskHandle_t FrontEndTaskHandle;

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

portBASE_TYPE onCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE offCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE colorCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE RGBCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE toggleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE pulseColorCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE pulseRGBCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE sweepCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
portBASE_TYPE dimCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : on */
const CLI_Command_Definition_t onCommandDefinition =
{
	( const int8_t * ) "on", /* The command string to type. */
	( const int8_t * ) "(H01R0) on:\r\n Turn RGB LED on (white color) at a specific intensity (0-100%) (1st par.)\r\n\r\n",
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
/* CLI command structure : toggle */
const CLI_Command_Definition_t toggleCommandDefinition =
{
	( const int8_t * ) "toggle", /* The command string to type. */
	( const int8_t * ) "(H01R0) toggle:\r\n Toggle RGB LED (white color) at a specific intensity (0-100%) (1st par.)\r\n\r\n",
	toggleCommand, /* The function to run. */
	1 /* One parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : pulseColor */
const CLI_Command_Definition_t pulseColorCommandDefinition =
{
	( const int8_t * ) "pulseColor", /* The command string to type. */
	( const int8_t * ) "(H01R0) pulseColor:\r\n Send a pulse on RGB LED using a specific color (1st par.), pulse period (ms) (2nd par.), pulse duty cycle (ms) (3rd par.) \
and pulse repeat times (4th par.) (type 'inf' for periodic signal)\r\n\r\n",
	pulseColorCommand, /* The function to run. */
	4 /* Four parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : pulseRGB */
const CLI_Command_Definition_t pulseRGBCommandDefinition =
{
	( const int8_t * ) "pulseRGB", /* The command string to type. */
	( const int8_t * ) "(H01R0) pulseRGB:\r\n Send a pulse on RGB LED using RGB values (1st, 2nd and 3rd par.) (0-255), pulse period (ms) (4th par.), pulse duty cycle (ms) (5th par.) \
and pulse repeat times (6th par.) (type 'inf' for periodic signal)\r\n\r\n",
	pulseRGBCommand, /* The function to run. */
	6 /* Six parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : sweep */
const CLI_Command_Definition_t sweepCommandDefinition =
{
	( const int8_t * ) "sweep", /* The command string to type. */
		( const int8_t * ) "(H01R0) sweep:\r\n Perform color sweep on RGB LED using a specific sweep mode ('basic': sweep basic color only, 'fine': sweep all colors) (1st par.), sweep period (ms) (2nd par.) \
and sweep repeat times (3rd par.) (type 'inf' for periodic signal)\r\n\r\n",
	sweepCommand, /* The function to run. */
	3 /* Three parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : dim */
const CLI_Command_Definition_t dimCommandDefinition =
{
	( const int8_t * ) "dim", /* The command string to type. */
		( const int8_t * ) "(H01R0) dim:\r\n Dim a specific color (1st par.) on RGB LED using a specific dim mode ('up', 'upwait', 'down', 'downwait', 'updown', 'downup', 'updownwait', 'downupwait') (2nd par.), \
sweep period (ms) (3rd par.), wait time (ms) (4th par.) and dim repeat times (5th par.) (type 'inf' for periodic signal)\r\n\r\n",
	dimCommand, /* The function to run. */
	5 /* Five parameters are expected. */
};
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H01R0 message processing task. 
*/
H01R0_Status H01R0_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	H01R0_Status result = H01R0_OK;
	uint32_t period = 0; uint32_t dc = 0; int32_t repeat = 0;
	
	switch (code)
	{
		case CODE_H01R0_on :
			RGB_LED_on(cMessage[port-1][4]);
			break;
		
		case CODE_H01R0_off :
			RGB_LED_off();
			break;
		
		case CODE_H01R0_toggle :
			if (RGB_LED_State)
				RGB_LED_off();
			else
				RGB_LED_on(cMessage[port-1][4]);
			break;

		case CODE_H01R0_color :
			if (cMessage[port-1][4] == 0) {
			/* Color definition from color list */
				RGB_LED_setColor(cMessage[port-1][5], cMessage[port-1][6]);
			} else if (cMessage[port-1][4] == 1) {
			/* RGB color */
				RGB_LED_setRGB(cMessage[port-1][5], cMessage[port-1][6], cMessage[port-1][7], cMessage[port-1][8]);			
			}	
			break;
		
		case CODE_H01R0_pulse :
			if (cMessage[port-1][4] == 0) {
			/* Color definition from color list */
				period = ( (uint32_t) cMessage[port-1][6] << 24 ) + ( (uint32_t) cMessage[port-1][7] << 16 ) + ( (uint32_t) cMessage[port-1][8] << 8 ) + cMessage[port-1][9];
				dc = ( (uint32_t) cMessage[port-1][10] << 24 ) + ( (uint32_t) cMessage[port-1][11] << 16 ) + ( (uint32_t) cMessage[port-1][12] << 8 ) + cMessage[port-1][13];
				repeat = ( (uint32_t) cMessage[port-1][14] << 24 ) + ( (uint32_t) cMessage[port-1][15] << 16 ) + ( (uint32_t) cMessage[port-1][16] << 8 ) + cMessage[port-1][17];
				RGB_LED_pulseColor(cMessage[port-1][5], period, dc, repeat);
			} else if (cMessage[port-1][4] == 1) {
			/* RGB color */
				period = ( (uint32_t) cMessage[port-1][8] << 24 ) + ( (uint32_t) cMessage[port-1][9] << 16 ) + ( (uint32_t) cMessage[port-1][10] << 8 ) + cMessage[port-1][11];
				dc = ( (uint32_t) cMessage[port-1][12] << 24 ) + ( (uint32_t) cMessage[port-1][13] << 16 ) + ( (uint32_t) cMessage[port-1][14] << 8 ) + cMessage[port-1][15];
				repeat = ( (uint32_t) cMessage[port-1][16] << 24 ) + ( (uint32_t) cMessage[port-1][17] << 16 ) + ( (uint32_t) cMessage[port-1][18] << 8 ) + cMessage[port-1][19];
				RGB_LED_pulseRGB(cMessage[port-1][5], cMessage[port-1][6], cMessage[port-1][7], period, dc, repeat);			
			}				
			break;
			
		case CODE_H01R0_sweep :
			period = ( (uint32_t) cMessage[port-1][5] << 24 ) + ( (uint32_t) cMessage[port-1][6] << 16 ) + ( (uint32_t) cMessage[port-1][7] << 8 ) + cMessage[port-1][8];
			repeat = ( (uint32_t) cMessage[port-1][9] << 24 ) + ( (uint32_t) cMessage[port-1][10] << 16 ) + ( (uint32_t) cMessage[port-1][11] << 8 ) + cMessage[port-1][12];
			RGB_LED_sweep(cMessage[port-1][4], period, repeat);
			break;
		
		case CODE_H01R0_dim :
			period = ( (uint32_t) cMessage[port-1][6] << 24 ) + ( (uint32_t) cMessage[port-1][7] << 16 ) + ( (uint32_t) cMessage[port-1][8] << 8 ) + cMessage[port-1][9];
			dc = ( (uint32_t) cMessage[port-1][10] << 24 ) + ( (uint32_t) cMessage[port-1][11] << 16 ) + ( (uint32_t) cMessage[port-1][12] << 8 ) + cMessage[port-1][13];
			repeat = ( (uint32_t) cMessage[port-1][14] << 24 ) + ( (uint32_t) cMessage[port-1][15] << 16 ) + ( (uint32_t) cMessage[port-1][16] << 8 ) + cMessage[port-1][17];
			RGB_LED_dim(cMessage[port-1][4], cMessage[port-1][5], period, dc, repeat);
			break;
		
		default:
			result = H01R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

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
	uint8_t intensityRed = 0, intensityGreen = 0, intensityBlue = 0, temp = 0;

	if (!intensity) {	
		temp = rgbLedMode;			/* Backup rgbLedMode so that it's not reset */
		RGB_LED_off();
		rgbLedMode = temp;
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
	uint8_t temp = 0;
	
	if (!intensity) {	
		temp = rgbLedMode;			/* Backup rgbLedMode so that it's not reset */
		RGB_LED_off();	
		rgbLedMode = temp;
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
	rgbLedMode = 0;			/* If you do not want to reset the mode, you need to backup rgbLedMode before calling RGB_LED_off */
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

/* --- Activate the RGB LED pulse command with RGB values. Set repeat to -1 for periodic signals --- 
*/
H01R0_Status RGB_LED_pulseRGB(uint8_t red, uint8_t green, uint8_t blue, uint32_t period, uint32_t dc, int32_t repeat)
{
	H01R0_Status result = H01R0_OK;
	
	rgbRed = red; rgbGreen = green; rgbBlue = blue;
	rgbPeriod = period; rgbDC = dc; rgbCount = repeat;
	
	rgbLedMode = RGB_pulseRGB;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Activate the RGB LED pulse command with a specific color. Set repeat to -1 for periodic signals --- 
*/
H01R0_Status RGB_LED_pulseColor(uint8_t color, uint32_t period, uint32_t dc, int32_t repeat)
{
	H01R0_Status result = H01R0_OK;
	
	rgbColor = color;
	rgbPeriod = period; rgbDC = dc; rgbCount = repeat;
	
	rgbLedMode = RGB_pulseColor;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Pulse the RGB LED --- 
*/
void RGBpulse(uint8_t mode)
{
	uint8_t temp = 0;
	
	if (rgbCount > 0 || rgbCount == -1) 
	{
		if (mode == RGB_pulseRGB)
			RGB_LED_setRGB(rgbRed, rgbGreen, rgbBlue, 100);
		else if (mode == RGB_pulseColor)
			RGB_LED_setColor(rgbColor, 100);	
		osDelay(rgbDC);
		temp = rgbLedMode;			/* Backup rgbLedMode so that it's not reset */
		RGB_LED_off();
		rgbLedMode = temp;
		osDelay(rgbPeriod-rgbDC);
		
		if (rgbCount > 0)
			rgbCount--;
	} 
	else 
	{
		rgbLedMode = 0;
	}
}

/*-----------------------------------------------------------*/

/* --- Activate the RGB LED sweep mode. Set repeat to -1 for periodic signals. Minimum period for fine sweep is 6 x 256 = 1536 ms --- 
*/
H01R0_Status RGB_LED_sweep(uint8_t mode, uint32_t period, int32_t repeat)
{
	H01R0_Status result = H01R0_OK;
	
	rgbPeriod = period; rgbCount = repeat;
	rgbLedMode = mode;

	return result;
}

/*-----------------------------------------------------------*/

/* --- RGB LED basic color sweep --- 
*/
void RGBsweepBasic(void)
{
	static uint32_t temp;
		
	temp = rgbPeriod/6;
	
	if (rgbCount > 0 || rgbCount == -1) 
	{
		RGB_LED_setColor(RED, 100);
		osDelay(temp);
		RGB_LED_setColor(YELLOW, 100);
		osDelay(temp);
		RGB_LED_setColor(GREEN, 100);
		osDelay(temp);
		RGB_LED_setColor(CYAN, 100);
		osDelay(temp);
		RGB_LED_setColor(BLUE, 100);
		osDelay(temp);
		RGB_LED_setColor(MAGENTA, 100);
		osDelay(temp);
	
		if (rgbCount > 0)
			rgbCount--;
	} 
	else 
	{
		RGB_LED_off();
	}
}

/*-----------------------------------------------------------*/

/* --- RGB LED fine color sweep --- 
*/
void RGBsweepFine(void)
{
	static uint32_t temp;
		
	temp = rgbPeriod/6;
	
	if (rgbCount > 0 || rgbCount == -1) 
	{
		/* Red */
		for(uint8_t i=0 ; i<255 ; i++)
		{
			RGB_LED_setRGB(255, i, 0, 100);	
			osDelay(ceil(temp/255));
		}	
		/* Yellow */		
		for(uint8_t i=255 ; i>0 ; i--)
		{
			RGB_LED_setRGB(i, 255, 0, 100);
			osDelay(ceil(temp/255));
		}
		/* Green */
		for(uint8_t i=0 ; i<255 ; i++)
		{		
			RGB_LED_setRGB(0, 255, i, 100);
			osDelay(ceil(temp/255));
		}		
		/* Cyan */
		for(uint8_t i=255 ; i>0 ; i--)
		{		
			RGB_LED_setRGB(0, i, 255, 100);
			osDelay(ceil(temp/255));
		}
		/* Blue */
		for(uint8_t i=0 ; i<255 ; i++)
		{			
			RGB_LED_setRGB(i, 0, 255, 100);
			osDelay(ceil(temp/255));
		}		
		/* Magenta */
		for(uint8_t i=255 ; i>0 ; i--)
		{			
			RGB_LED_setRGB(255, 0, i, 100);
			osDelay(ceil(temp/255));
		}	
	
		if (rgbCount > 0)
			rgbCount--;
	} 
	else 
	{
		RGB_LED_off();
	}
}

/*-----------------------------------------------------------*/

/* --- Activate RGB LED dim mode using one of the basic colors. Set repeat to -1 for periodic signals --- 
*/
H01R0_Status RGB_LED_dim(uint8_t color, uint8_t mode, uint32_t period, uint32_t wait, int32_t repeat)
{
	H01R0_Status result = H01R0_OK;
	
	rgbColor = color;
	rgbPeriod = period; rgbDC = wait; rgbCount = repeat;
	rgbLedMode = mode;
	
	return result;
}

/*-----------------------------------------------------------*/

/* --- Dim the RGB LED --- 
*/
void RGBdim(uint8_t mode)
{
	uint8_t temp = 0;
	
	if (rgbCount > 0 || rgbCount == -1) 
	{
		temp = rgbLedMode;			/* Store mode so that it's not reset */
		if (mode == RGB_dimUp) {
			for(uint8_t i=0 ; i<=100 ; i++)
			{
				RGB_LED_setColor(rgbColor, i);	
				osDelay(rgbPeriod/100);
			}	
		} else if (mode == RGB_dimUpWait) {
			for(uint8_t i=0 ; i<=100 ; i++)
			{
				RGB_LED_setColor(rgbColor, i);	
				osDelay((rgbPeriod-rgbDC)/100);
			}				
			osDelay(rgbDC);		
		} else if (mode == RGB_dimDown) {
			for(uint8_t i=101 ; i>0 ; i--)
			{
				RGB_LED_setColor(rgbColor, i-1);	
				osDelay(rgbPeriod/100);
			}			
		} else if (mode == RGB_dimDownWait) {
			for(uint8_t i=101 ; i>0 ; i--)
			{
				RGB_LED_setColor(rgbColor, i-1);	
				osDelay((rgbPeriod-rgbDC)/100);
			}	
			osDelay(rgbDC);
		} else if (mode == RGB_dimUpDown) {
			for(uint8_t i=0 ; i<=100 ; i++)
			{
				RGB_LED_setColor(rgbColor, i);	
				osDelay(rgbPeriod/200);
			}				
			for(uint8_t i=101 ; i>0 ; i--)
			{
				RGB_LED_setColor(rgbColor, i-1);	
				osDelay(rgbPeriod/200);
			}				
		} else if (mode == RGB_dimDownUp) {
			for(uint8_t i=101 ; i>0 ; i--)
			{
				RGB_LED_setColor(rgbColor, i-1);	
				osDelay(rgbPeriod/200);
			}					
			for(uint8_t i=0 ; i<=100 ; i++)
			{
				RGB_LED_setColor(rgbColor, i);	
				osDelay(rgbPeriod/200);
			}				
		} else if (mode == RGB_dimUpDownWait) {
			for(uint8_t i=0 ; i<=100 ; i++)
			{
				RGB_LED_setColor(rgbColor, i);	
				osDelay((rgbPeriod-rgbDC)/200);
			}				
			for(uint8_t i=101 ; i>0 ; i--)
			{
				RGB_LED_setColor(rgbColor, i-1);	
				osDelay((rgbPeriod-rgbDC)/200);
			}		
			osDelay(rgbDC);
		} else if (mode == RGB_dimDownUpWait) {
			for(uint8_t i=101 ; i>0 ; i--)
			{
				RGB_LED_setColor(rgbColor, i-1);	
				osDelay((rgbPeriod-rgbDC)/200);
			}				
			for(uint8_t i=0 ; i<=100 ; i++)
			{
				RGB_LED_setColor(rgbColor, i);	
				osDelay((rgbPeriod-rgbDC)/200);
			}	
			osDelay(rgbDC);
			rgbLedMode = temp;
		}

		if (rgbCount > 0)
			rgbCount--;
	} 
	else 
	{
		rgbLedMode = 0;
	}
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
	uint8_t color = 0; uint8_t intensity = 0; char par[15] = {0};
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
		/* Isolate first parameter string */
		strncpy(par, ( char * ) pcParameterString1, xParameterStringLength1);
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOKMessage, par, intensity);
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

portBASE_TYPE toggleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	
	int8_t *pcParameterString1; portBASE_TYPE xParameterStringLength1 = 0; 
	uint8_t intensity = 0;
	static const int8_t *pcOK1Message = ( int8_t * ) "RGB LED is on at intensity %d%%\r\n";
	static const int8_t *pcOK0Message = ( int8_t * ) "RGB LED is off\r\n";
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
	
	result = RGB_LED_toggle(intensity);	
	
	/* Respond to the command */
	if ( (result == H01R0_OK) && RGB_LED_State)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOK1Message, intensity);
	else if ( (result == H01R0_OK) && !RGB_LED_State)
		sprintf( ( char * ) pcWriteBuffer, ( char * ) pcOK0Message, intensity);
	else if (result == H01R0_ERR_WrongIntensity)
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcWrongIntensityMessage);
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE pulseColorCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	uint8_t color = 0; uint32_t period = 0, dc = 0; int32_t repeat = 0; char par[15] = {0};
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;
	portBASE_TYPE xParameterStringLength3 = 0, xParameterStringLength4 = 0;
	
	static const int8_t *pcMessage = ( int8_t * ) "A %s pulse with period %d ms and duty cycle %d ms is generated %d times\n\r";
	static const int8_t *pcMessageInf = ( int8_t * ) "A %s pulse with period %d ms and duty cycle %d ms is generated periodically\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
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
	period = ( uint32_t ) atol( ( char * ) pcParameterString2 );
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	dc = ( uint32_t ) atol( ( char * ) pcParameterString3 );
	
	/* Obtain the 4th parameter string. */
	pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
	if (!strcmp( ( char * ) pcParameterString4, "inf") || !strcmp( ( char * ) pcParameterString4, "INF"))
		repeat = -1;
	else
		repeat = ( int32_t ) atol( ( char * ) pcParameterString4 );
	
	result = RGB_LED_pulseColor(color, period, dc, repeat);
	
	/* Respond to the command */
	if (result == H01R0_OK) 
	{
		/* Isolate first parameter string */
		strncpy(par, ( char * ) pcParameterString1, xParameterStringLength1);
		if (repeat == -1)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageInf, par, period, dc);
		else
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, par, period, dc, repeat);
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE pulseRGBCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	uint8_t red = 0; uint8_t green = 0; uint8_t blue = 0; uint32_t period = 0, dc = 0; int32_t repeat = 0;
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4; 
	static int8_t *pcParameterString5, *pcParameterString6;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0, xParameterStringLength3 = 0;
	portBASE_TYPE xParameterStringLength4 = 0, xParameterStringLength5 = 0, xParameterStringLength6 = 0;
	
	static const int8_t *pcMessage = ( int8_t * ) "A (%d, %d, %d) RGB pulse with period %d ms and duty cycle %d ms is generated %d times\n\r";
	static const int8_t *pcMessageInf = ( int8_t * ) "A (%d, %d, %d) RGB pulse with period %d ms and duty cycle %d ms is generated periodically\n\r";
	
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
	period = ( uint32_t ) atol( ( char * ) pcParameterString4 );
	
	/* Obtain the 5th parameter string. */
	pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);
	dc = ( uint32_t ) atol( ( char * ) pcParameterString5 );
	
	/* Obtain the 6th parameter string. */
	pcParameterString6 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 6, &xParameterStringLength6);
	if (!strcmp( ( char * ) pcParameterString6, "inf") || !strcmp( ( char * ) pcParameterString6, "INF"))
		repeat = -1;
	else
		repeat = ( int32_t ) atol( ( char * ) pcParameterString6 );
	
	result = RGB_LED_pulseRGB(red, green, blue, period, dc, repeat);
	
	/* Respond to the command */
	if (result == H01R0_OK) 
	{
		if (repeat == -1)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageInf, red, green, blue, period, dc);
		else
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, red, green, blue, period, dc, repeat);
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE sweepCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	uint8_t mode = 0; uint32_t period = 0; int32_t repeat = 0; char par[15] = {0};
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0, xParameterStringLength3 = 0;
	
	static const int8_t *pcMessage = ( int8_t * ) "The RGB LED performs a %s color sweep with period %d ms. The sweep is repeated %d times\n\r";
	static const int8_t *pcMessageInf = ( int8_t * ) "The RGB LED performs a %s color sweep with period %d ms. The sweep is repeated periodically\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	if (!strncmp( ( char * ) pcParameterString1, "basic", xParameterStringLength1) || !strncmp( ( char * ) pcParameterString1, "BASIC", xParameterStringLength1))
		mode = RGB_sweepBasic;
	else if (!strncmp( ( char * ) pcParameterString1, "fine", xParameterStringLength1) || !strncmp( ( char * ) pcParameterString1, "FINE", xParameterStringLength1))
		mode = RGB_sweepFine;
	
	/* Obtain the 2nd parameter string. */
	pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	period = ( uint32_t ) atol( ( char * ) pcParameterString2 );
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	if (!strcmp( ( char * ) pcParameterString3, "inf") || !strcmp( ( char * ) pcParameterString3, "INF"))
		repeat = -1;
	else
		repeat = ( int32_t ) atol( ( char * ) pcParameterString3 );
	
	result = RGB_LED_sweep(mode, period, repeat);
	
	/* Respond to the command */
	if (result == H01R0_OK) 
	{	
		/* Isolate first parameter string */
		strncpy(par, ( char * ) pcParameterString1, xParameterStringLength1); 
		if (repeat == -1)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageInf, par, period);
		else
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, par, period, repeat);
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE dimCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	H01R0_Status result = H01R0_OK;
	uint8_t color = 0, mode = 0; uint32_t period = 0, wait = 0; int32_t repeat = 0;
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3, *pcParameterString4 , *pcParameterString5; 
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0, xParameterStringLength3 = 0; 
	portBASE_TYPE xParameterStringLength4 = 0, xParameterStringLength5 = 0;
	char par1[15] = {0}, par2[15] = {0};
	
	static const int8_t *pcMessage = ( int8_t * ) "The RGB LED dims a %s color with period %d ms and wait time %d ms. The dim mode is %s and the dim cycle is repeated %d times\n\r";
	static const int8_t *pcMessageInf = ( int8_t * ) "The RGB LED dims a %s color with period %d ms and wait time %d ms. The dim mode is %s and the dim cycle is repeated periodically\n\r";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	/* Obtain the 1st parameter string. */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
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
	if (!strncmp( ( char * ) pcParameterString2, "up", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UP", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "Up", xParameterStringLength2))
		mode = RGB_dimUp;
	else if (!strncmp( ( char * ) pcParameterString2, "upwait", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UPWAIT", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UpWait", xParameterStringLength2))
		mode = RGB_dimUpWait;
	else if (!strncmp( ( char * ) pcParameterString2, "down", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DOWN", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "Down", xParameterStringLength2))
		mode = RGB_dimDown;
	else if (!strncmp( ( char * ) pcParameterString2, "downwait", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DOWNWAIT", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DownWait", xParameterStringLength2))
		mode = RGB_dimDownWait;
	else if (!strncmp( ( char * ) pcParameterString2, "updown", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UPDOWN", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UpDown", xParameterStringLength2))
		mode = RGB_dimUpDown;
	else if (!strncmp( ( char * ) pcParameterString2, "downup", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DOWNUP", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DownUp", xParameterStringLength2))
		mode = RGB_dimDownUp;
	else if (!strncmp( ( char * ) pcParameterString2, "updownwait", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UPDOWNWAIT", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "UpDownWait", xParameterStringLength2))
		mode = RGB_dimUpDownWait;
	else if (!strncmp( ( char * ) pcParameterString2, "downupwait", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DOWNUPWAIT", xParameterStringLength2) || !strncmp( ( char * ) pcParameterString2, "DownUpWait", xParameterStringLength2))
		mode = RGB_dimDownUpWait;
	
	/* Obtain the 3rd parameter string. */
	pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	period = ( uint32_t ) atol( ( char * ) pcParameterString3 );

	/* Obtain the 4th parameter string. */
	pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
	wait = ( uint32_t ) atol( ( char * ) pcParameterString4 );
	
	/* Obtain the 5th parameter string. */
	pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);
	if (!strcmp( ( char * ) pcParameterString5, "inf") || !strcmp( ( char * ) pcParameterString5, "INF"))
		repeat = -1;
	else
		repeat = ( int32_t ) atol( ( char * ) pcParameterString5 );
	
	result = RGB_LED_dim(color, mode, period, wait, repeat);
	
	/* Respond to the command */
	if (result == H01R0_OK) 
	{	
		/* Isolate first and second parameter strings. */
		strncpy(par1, ( char * ) pcParameterString1, xParameterStringLength1);
		strncpy(par2, ( char * ) pcParameterString2, xParameterStringLength2);
		if (repeat == -1)
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageInf, par1, period, wait, par2);
		else
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessage, par1, period, wait, par2, repeat);
	}
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
