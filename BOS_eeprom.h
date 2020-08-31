/**
  ******************************************************************************
  * @file    BOS_eeprom.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
/*
		MODIFIED by Hexabitz for BitzOS (BOS) V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOS_EEPROM_H
#define __BOS_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Exported constants --------------------------------------------------------*/


/* Pages A and B base and end addresses - Each page is extended into two pages 1 and 2 */
#define PAGEA1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGEA1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
#define PAGEA2_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGEA2_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

#define PAGEB1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE)))
#define PAGEB1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (3 * PAGE_SIZE - 1)))
#define PAGEB2_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + (3 * PAGE_SIZE)))
#define PAGEB2_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (4 * PAGE_SIZE - 1)))


/* Used Flash pages for EEPROM emulation - Each one is twice page size */
#define PAGEA                 ((uint16_t)0x0000)
#define PAGEB                 ((uint16_t)0x0002)

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/* EEPROM Variables' number (up to 1024 16-bit variables) */
#define NumOfEEPROMvar       1024				


/* EEPROM virtual addresses - Consider MaxNumOfModules is 25 */
// BOS Addressing Space 1 - 499
#define _EE_NBASE									1	
#define _EE_PORT_DIR_BASE					2					// Todo: Move to RO - 25 modules - 25 variables
#define _EE_ALIAS_BASE						28				// 25 modules/10 chars - 125 variables
#define _EE_GROUP_ALIAS_BASE			153				// 10 groups/10 chars - 50 variables
#define _EE_GROUP_MODULES_BASE		203				// 25 modules - 25 variables
#define _EE_DMA_STREAM_BASE				228				// 8 variables				
#define _EE_BUTTON_BASE						236				// 4 * MaxNumOfPorts (10) variables for buttons: port(4 bits), type (4 bits), events (8 bits)
																					// pressed_for_x_1 (8 bits), released_for_y_1 (8 bits), etc.
#define _EE_PARAMS_BASE						276				// Parameter base: BOS trace (MSB) | BOS response - 1 variable 
#define _EE_PARAMS_DEBOUNCE				277				// Parameter: Button debounce - 1 variable
#define _EE_PARAMS_SINGLE_CLICK		278				// Parameter: Button single click - 1 variable
#define _EE_PARAMS_DBL_CLICK			279				// Parameter: Button double click (inter-click min and max) - 1 variable
#define _EE_CLI_BAUD							280				// Parameter: CLI baudrate - LSB halfword, MSB halfword - 2 variables
#define _EE_PARAMS_RTC						282				// Parameter: RTC hourformat | RTC daylightsaving - 1 variable
#define _EE_PARAMS_DISABLE_CLI		283				// Parameter: Disable CLI - 1 variable

// Module Addressing Space 500 - 599
#define _EE_MODULE_BASE						500

// User Addressing Space 600 - 1024
#define _EE_EMPTY_VAR_BASE				600


#if MaxNumOfModules > 25						// Update
 #warning "Data for 25 modules only will be stored in EEPROM."
#endif


/* Exported variables --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t EE_Init(void);
uint16_t EE_Format(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
uint16_t Flash_WriteVariable(uint32_t Address, uint16_t Data);


#endif /* __BOS_EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
