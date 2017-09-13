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
		MODIFIED by Hexabitz for BitzOS (BOS) V0.1.1 - Copyright (C) 2017 Hexabitz
    All rights reserved
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOS_EEPROM_H
#define __BOS_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Exported constants --------------------------------------------------------*/
/* Define the size of the sectors to be used */
#define FLASH_SIZE						((uint32_t)0x20000)
#define PAGE_SIZE             ((uint32_t)0x0800)  /* Page size = 2KByte for STM32F07x 
																												and STM32F09x devices */
																												
/* Memory map:
				- Application: 0x08000000 - 0x0801D800 >> 118 KB
		 - Read-only (RO): 0x0801D800 - 0x0801E000 >> 2 KB, used to store topology information
		- Emulated EEPROM: 0x0801E000 - 0x08020000 >> 8 KB, fits 1024 16-bit variables in 2 main-duplicate pages (A and B)
*/
#define APP_START_ADDRESS  		((uint32_t)0x08000000) 
#define RO_START_ADDRESS  		((uint32_t)0x0801D800) 
#define EEPROM_START_ADDRESS  ((uint32_t)0x0801E000) 


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
#define NumOfEEPROMvar        1024				


/* EEPROM virtual addresses - Consider MaxNumOfModules is 25 */
#define _EE_NBase								1	
#define _EE_portDirBase					2					// Move to RO - 25 modules
#define _EE_aliasBase						28				// 25 modules
#define _EE_DMAStreamsBase			159				
#define _EE_ButtonBase					167				// 4 * MaxNumOfPorts (10) variables for buttons: port(4 bits), type (4 bits), events (8 bits)
																					// pressed_for_x_1 (8 bits), released_for_y_1 (8 bits), etc.
#define _EE_EmptyVarBase				207
#define _EE_ParamsBase					500				// Parameter base: BOS response
#define _EE_ParamsDebounce			501				// Parameter: Button debounce
#define _EE_ParamsSinClick			502				// Parameter: Button single click
#define _EE_ParamsDblClick			503				// Parameter: Button double click (inter-click min and max)
#define _EE_CLIBaud							504				// Parameter: CLI baudrate (two variables) - LSB halfword, MSB halfword


#if MaxNumOfModules > 25						// Update
 #warning "Only data for 25 modules will be stored in EEPROM."
#endif


/* Exported variables --------------------------------------------------------*/
extern uint16_t VirtAddVarTab[NumOfEEPROMvar+1];

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
