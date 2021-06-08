/**
 ******************************************************************************
 * @file    BOS_eeprom.c 
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    29-May-2012
 * @brief   This file provides all the EEPROM emulation firmware functions.
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
 MODIFIED by Hexabitz for BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
 All rights reserved
 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variable used to store variable value in read sequence */
uint16_t DataVar =0;

/* Variables used for Erase pages under interruption */
extern FLASH_ProcessTypeDef pFlash;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress,
    uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_FindValidPage(uint8_t Operation);

/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint16_t EE_Init(void){
	uint16_t PageStatusA =6, PageStatusB =6;
	uint16_t VarIdx =0;
	uint16_t EepromStatus =0, ReadStatus =0;
	int16_t x =-1;
	uint16_t FlashStatus =HAL_ERROR;
	
	HAL_FLASH_Unlock();
	
	/* Get PageA status */
	PageStatusA = (*(__IO uint16_t*) PAGEA1_BASE_ADDRESS);
	/* Get PageB status */
	PageStatusB = (*(__IO uint16_t*) PAGEB1_BASE_ADDRESS);
	
	/* Check for invalid header states and repair if necessary */
	switch(PageStatusA){
		case ERASED:
		if(PageStatusB == VALID_PAGE) /* PageA erased, PageB valid */
		{
			/* Erase PageA */
			FLASH_PageErase(PAGEA1_BASE_ADDRESS);
			/* If erase operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* Erase PageA */
				FLASH_PageErase(PAGEA2_BASE_ADDRESS);
				FlashStatus =FLASH_WaitForLastOperation(
				    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				}
				else {
					/* Operation is completed, disable the PER Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
				}
			}
		}
		else if(PageStatusB == RECEIVE_DATA) /* PageA erased, PageB receive */
		{
			/* Erase PageA */
			FLASH_PageErase(PAGEA1_BASE_ADDRESS);
			/* If erase operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* Erase PageA */
				FLASH_PageErase(PAGEA2_BASE_ADDRESS);
				FlashStatus =FLASH_WaitForLastOperation(
				    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				}
				else {
					/* Operation is completed, disable the PER Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
				}
			}
			/* Mark PageB as valid */
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PAGEB1_BASE_ADDRESS,
			VALID_PAGE);
			/* If program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* If the program operation is completed, disable the PG Bit */
				CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
			}
		}
		else /* First EEPROM access (PageA&B are erased) or invalid state -> format EEPROM */
		{
			/* Erase both PageA and PageB and set PageA as valid page */
			FlashStatus =EE_Format();
			/* If erase/program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
		}
			break;
		
		case RECEIVE_DATA:
		if(PageStatusB == VALID_PAGE) /* PageA receive, PageB valid */
		{
			/* Transfer data from PageB to PageA */
			for(VarIdx =1; VarIdx <= NumOfEEPROMvar; VarIdx++) {
				if( (*(__IO uint16_t*) (PAGEA1_BASE_ADDRESS + 6)) == VarIdx) {
					x =VarIdx;
				}
				if(VarIdx != x) {
					/* Read the last variables' updates */
					ReadStatus =EE_ReadVariable(VarIdx,&DataVar);
					/* In case variable corresponding to the virtual address was found */
					if(ReadStatus != 0x1) {
						/* Transfer the variable to the PageA */
						EepromStatus =EE_VerifyPageFullWriteVariable(VarIdx,
						    DataVar);
						/* If program operation was failed, a Flash error code is returned */
						/* Wait for last operation to be completed */
						FlashStatus =FLASH_WaitForLastOperation(
						    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
						if(FlashStatus != HAL_OK) {
							return EepromStatus;
						}
					}
				}
			}
			/* Mark PageA as valid */
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PAGEA1_BASE_ADDRESS,
			VALID_PAGE);
			/* If program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* If the program operation is completed, disable the PG Bit */
				CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
			}
			/* Erase PageB */
			FLASH_PageErase(PAGEB1_BASE_ADDRESS);
			/* If erase operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* Erase PageB */
				FLASH_PageErase(PAGEB2_BASE_ADDRESS);
				FlashStatus =FLASH_WaitForLastOperation(
				    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				}
				else {
					/* Operation is completed, disable the PER Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
				}
			}
		}
		else if(PageStatusB == ERASED) /* PageA receive, PageB erased */
		{
			/* Erase PageB */
			FLASH_PageErase(PAGEB1_BASE_ADDRESS);
			/* If erase operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* Erase PageB */
				FLASH_PageErase(PAGEB2_BASE_ADDRESS);
				FlashStatus =FLASH_WaitForLastOperation(
				    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				}
				else {
					/* Operation is completed, disable the PER Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
				}
			}
			/* Mark PageA as valid */
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PAGEA1_BASE_ADDRESS,
			VALID_PAGE);
			/* If program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* If the program operation is completed, disable the PG Bit */
				CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
			}
		}
		else /* Invalid state -> format eeprom */
		{
			/* Erase both PageA and PageB and set PageA as valid page */
			FlashStatus =EE_Format();
			/* If erase/program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
		}
			break;
		
		case VALID_PAGE:
		if(PageStatusB == VALID_PAGE) /* Invalid state -> format eeprom */
		{
			/* Erase both PageA and PageB and set PageA as valid page */
			FlashStatus =EE_Format();
			/* If erase/program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return FlashStatus;
			}
		}
		else if(PageStatusB == ERASED) /* PageA valid, PageB erased */
		{
			/* Erase PageB */
			FLASH_PageErase(PAGEB1_BASE_ADDRESS);
			/* If erase operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* Erase PageB */
				FLASH_PageErase(PAGEB2_BASE_ADDRESS);
				FlashStatus =FLASH_WaitForLastOperation(
				    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				}
				else {
					/* Operation is completed, disable the PER Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
				}
			}
		}
		else /* PageA valid, PageB receive */
		{
			/* Transfer data from PageA to PageB */
			for(VarIdx =1; VarIdx <= NumOfEEPROMvar; VarIdx++) {
				if( (*(__IO uint16_t*) (PAGEB1_BASE_ADDRESS + 6)) == VarIdx) {
					x =VarIdx;
				}
				if(VarIdx != x) {
					/* Read the last variables' updates */
					ReadStatus =EE_ReadVariable(VarIdx,&DataVar);
					/* In case variable corresponding to the virtual address was found */
					if(ReadStatus != 0x1) {
						/* Transfer the variable to the PageB */
						EepromStatus =EE_VerifyPageFullWriteVariable(VarIdx,
						    DataVar);
						/* If program operation was failed, a Flash error code is returned */
						/* Wait for last operation to be completed */
						FlashStatus =FLASH_WaitForLastOperation(
						    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
						if(FlashStatus != HAL_OK) {
							return EepromStatus;
						}
					}
				}
			}
			/* Mark PageB as valid */
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PAGEB1_BASE_ADDRESS,
			VALID_PAGE);
			/* If program operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* If the program operation is completed, disable the PG Bit */
				CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
			}
			/* Erase PageA */
			FLASH_PageErase(PAGEA1_BASE_ADDRESS);
			/* If erase operation was failed, a Flash error code is returned */
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* Erase PageA */
				FLASH_PageErase(PAGEA2_BASE_ADDRESS);
				FlashStatus =FLASH_WaitForLastOperation(
				    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				}
				else {
					/* Operation is completed, disable the PER Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
				}
			}
		}
			break;
		
		default: /* Any other state -> format eeprom */
		/* Erase both PageA and PageB and set PageA as valid page */
		FlashStatus =EE_Format();
		/* If erase/program operation was failed, a Flash error code is returned */
		/* Wait for last operation to be completed */
		FlashStatus =FLASH_WaitForLastOperation(
		    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK) {
			return FlashStatus;
		}
			break;
	}
	
	HAL_FLASH_Lock();
	
	return HAL_OK;
}

/**
 * @brief  Returns the last stored variable data, if found, which correspond to
 *   the passed virtual address
 * @param  VirtAddress: Variable virtual address
 * @param  Data: Global variable contains the read variable value
 * @retval Success or error status:
 *           - 0: if variable was found
 *           - 1: if the variable was not found
 *           - NO_VALID_PAGE: if no valid page was found.
 */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t *Data){
	uint16_t ValidPage = PAGEA;
	uint16_t AddressValue =0x5555, ReadStatus =1;
	uint32_t Address =0, PageStartAddress =0;
	
	/* Get active Page for read operation */
	ValidPage =EE_FindValidPage(READ_FROM_VALID_PAGE);
	
	/* Check if there is no valid page */
	if(ValidPage == NO_VALID_PAGE) {
		return NO_VALID_PAGE;
	}
	
	/* Get the valid Page start Address */
	PageStartAddress =(uint32_t) (EEPROM_START_ADDRESS
	    + (uint32_t) (ValidPage * PAGE_SIZE));
	
	/* Get the valid Page end Address - Each page is twice page size */
	Address =(uint32_t) ( (EEPROM_START_ADDRESS - 2)
	    + (uint32_t) ( (2 + ValidPage) * PAGE_SIZE));
	
	/* Check each active page address starting from end */
	while(Address > (PageStartAddress + 2)) {
		/* Get the current location content to be compared with virtual address */
		AddressValue = (*(__IO uint16_t*) Address);
		
		/* Compare the read address with the virtual address */
		if(AddressValue == VirtAddress) {
			/* Get content of Address-2 which is variable value */
			*Data = (*(__IO uint16_t*) (Address - 2));
			
			/* In case variable value is read, reset ReadStatus flag */
			ReadStatus =0;
			
			break;
		}
		else {
			/* Next address location */
			Address =Address - 4;
		}
	}
	
	/* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
	return ReadStatus;
}

/**
 * @brief  Writes/upadtes variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data){
	uint16_t Status =0;
	
	HAL_FLASH_Unlock();
	
	/* Write the variable virtual address and value in the EEPROM */
	Status =EE_VerifyPageFullWriteVariable(VirtAddress,Data);
	
	/* In case the EEPROM active page is full */
	if(Status == PAGE_FULL) {
		/* Perform Page transfer */
		Status =EE_PageTransfer(VirtAddress,Data);
	}
	
	HAL_FLASH_Lock();
	
	/* Return last operation status */
	return Status;
}

/**
 * @brief  Erases PAGEA and PAGEB and writes VALID_PAGE header to PAGEA
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
uint16_t EE_Format(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	
	HAL_FLASH_Unlock();
	
	/* Erase PageA */
	FLASH_PageErase(PAGEA1_BASE_ADDRESS);
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If erase operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* Erase PageA */
		FLASH_PageErase(PAGEA2_BASE_ADDRESS);
		FlashStatus =FLASH_WaitForLastOperation(
		    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		}
		else {
			/* Operation is completed, disable the PER Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
		}
	}
	
	/* Set PageA as valid page: Write VALID_PAGE at Page0 base address */
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PAGEA1_BASE_ADDRESS,
	VALID_PAGE);
	
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If program operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* If the program operation is completed, disable the PG Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
	}
	
	/* Erase PageB */
	FLASH_PageErase(PAGEB1_BASE_ADDRESS);
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If program operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* Erase PageB */
		FLASH_PageErase(PAGEB2_BASE_ADDRESS);
		FlashStatus =FLASH_WaitForLastOperation(
		    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		}
		else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
	}
	
	HAL_FLASH_Lock();
	
	/* Return Page1 erase operation status */
	return FlashStatus;
}

/**
 * @brief  Find valid Page for write or read operation
 * @param  Operation: operation to achieve on the valid page.
 *   This parameter can be one of the following values:
 *     @arg READ_FROM_VALID_PAGE: read operation from valid page
 *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
 * @retval Valid page number (PAGEA or PAGEB) or NO_VALID_PAGE in case
 *   of no valid page was found
 */
static uint16_t EE_FindValidPage(uint8_t Operation){
	uint16_t PageStatusA =6, PageStatusB =6;
	
	/* Get PageA actual status */
	PageStatusA = (*(__IO uint16_t*) PAGEA1_BASE_ADDRESS);
	
	/* Get PageB actual status */
	PageStatusB = (*(__IO uint16_t*) PAGEB1_BASE_ADDRESS);
	
	/* Write or read operation */
	switch(Operation){
		case WRITE_IN_VALID_PAGE: /* ---- Write operation ---- */
		if(PageStatusB == VALID_PAGE) {
			/* PageA receiving data */
			if(PageStatusA == RECEIVE_DATA) {
				return PAGEA; /* PageA valid */
			}
			else {
				return PAGEB; /* PageB valid */
			}
		}
		else if(PageStatusA == VALID_PAGE) {
			/* PageB receiving data */
			if(PageStatusB == RECEIVE_DATA) {
				return PAGEB; /* PageB valid */
			}
			else {
				return PAGEA; /* PageA valid */
			}
		}
		else {
			return NO_VALID_PAGE; /* No valid Page */
		}
		
		case READ_FROM_VALID_PAGE: /* ---- Read operation ---- */
		if(PageStatusA == VALID_PAGE) {
			return PAGEA; /* PageA valid */
		}
		else if(PageStatusB == VALID_PAGE) {
			return PAGEB; /* PageB valid */
		}
		else {
			return NO_VALID_PAGE; /* No valid Page */
		}
		
		default:
		return PAGEA; /* PageA valid */
	}
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress,
    uint16_t Data){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t ValidPage = PAGEA;
	uint32_t Address =0, PageEndAddress =0;
	
	HAL_FLASH_Unlock();
	
	/* Get valid Page for write operation */
	ValidPage =EE_FindValidPage(WRITE_IN_VALID_PAGE);
	
	/* Check if there is no valid page */
	if(ValidPage == NO_VALID_PAGE) {
		return NO_VALID_PAGE;
	}
	
	/* Get the valid Page start Address */
	Address =(uint32_t) (EEPROM_START_ADDRESS
	    + (uint32_t) (ValidPage * PAGE_SIZE));
	
	/* Get the valid Page end Address - Each page is twice pages size */
	PageEndAddress =(uint32_t) ( (EEPROM_START_ADDRESS - 2)
	    + (uint32_t) ( (2 + ValidPage) * PAGE_SIZE));
	
	/* Check each active page address starting from begining */
	while(Address < PageEndAddress) {
		/* Verify if Address and Address+2 contents are 0xFFFFFFFF */
		if( (*(__IO uint32_t*) Address) == 0xFFFFFFFF) {
			/* Set variable data */
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Data);
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			/* If program operation was failed, a Flash error code is returned */
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* If the program operation is completed, disable the PG Bit */
				CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
			}
			
			/* Set variable virtual address */
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address + 2,
			    VirtAddress);
			
			/* Wait for last operation to be completed */
			FlashStatus =FLASH_WaitForLastOperation(
			    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
			/* If program operation was failed, a Flash error code is returned */
			if(FlashStatus != HAL_OK) {
				return pFlash.ErrorCode;
			}
			else {
				/* If the program operation is completed, disable the PG Bit */
				CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
			}
			
			/* Return program operation status */
			return pFlash.ErrorCode;
		}
		else {
			/* Next address location */
			Address =Address + 4;
		}
	}
	
	HAL_FLASH_Lock();
	
	/* Return PAGE_FULL in case the valid page is full */
	return PAGE_FULL;
}

/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint32_t NewPageAddress =0, OldPageAddress =0;
	uint16_t ValidPage = PAGEA, VarIdx =0;
	uint16_t EepromStatus =0, ReadStatus =0;
	
	HAL_FLASH_Unlock();
	
	/* Get active Page for read operation */
	ValidPage =EE_FindValidPage(READ_FROM_VALID_PAGE);
	
	if(ValidPage == PAGEB) /* PageB valid */
	{
		/* New page address where variable will be moved to */
		NewPageAddress = PAGEA1_BASE_ADDRESS;
		
		/* Old page address where variable will be taken from */
		OldPageAddress = PAGEB1_BASE_ADDRESS;
	}
	else if(ValidPage == PAGEA) /* PageA valid */
	{
		/* New page address where variable will be moved to */
		NewPageAddress = PAGEB1_BASE_ADDRESS;
		
		/* Old page address where variable will be taken from */
		OldPageAddress = PAGEA1_BASE_ADDRESS;
	}
	else {
		return NO_VALID_PAGE; /* No valid Page */
	}
	
	/* Set the new Page status to RECEIVE_DATA status */
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,NewPageAddress,RECEIVE_DATA);
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If program operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* If the program operation is completed, disable the PG Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
	}
	
	/* Write the variable passed as parameter in the new active page */
	EepromStatus =EE_VerifyPageFullWriteVariable(VirtAddress,Data);
	/* If program operation was failed, a Flash error code is returned */
	if(EepromStatus != HAL_OK) {
		return EepromStatus;
	}
	
	/* Transfer process: transfer variables from old to the new active page */
	for(VarIdx =1; VarIdx <= NumOfEEPROMvar; VarIdx++) {
		if(VarIdx != VirtAddress) /* Check each variable except the one passed as parameter */
		{
			/* Read the other last variable updates */
			ReadStatus =EE_ReadVariable(VarIdx,&DataVar);
			/* In case variable corresponding to the virtual address was found */
			if(ReadStatus != 0x1) {
				/* Transfer the variable to the new active page */
				EepromStatus =EE_VerifyPageFullWriteVariable(VarIdx,DataVar);
				/* If program operation was failed, a Flash error code is returned */
				if(EepromStatus != HAL_OK) {
					return EepromStatus;
				}
			}
		}
	}
	
	/* Erase the old Page: Set old Page status to ERASED status */
	FLASH_PageErase(OldPageAddress);
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If erase operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* Erase the other half of the old Page: Set old Page status to ERASED status */
		FLASH_PageErase(OldPageAddress + PAGE_SIZE);
		FlashStatus =FLASH_WaitForLastOperation(
		    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		}
		else {
			/* Operation is completed, disable the PER Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
		}
	}
	
	/* Set new Page status to VALID_PAGE status */
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,NewPageAddress,VALID_PAGE);
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If program operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* If the program operation is completed, disable the PG Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
	}
	
	HAL_FLASH_Lock();
	
	/* Return last operation flash status */
	return FlashStatus;
}

/**
 * @brief  Writes/upadtes variable data in Flash.
 * @param  Address: Variable address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t Flash_WriteVariable(uint32_t Address, uint16_t Data){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	
	HAL_FLASH_Unlock();
	
	/* Set variable data */
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,Data);
	/* Wait for last operation to be completed */
	FlashStatus =FLASH_WaitForLastOperation(
	    (uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	/* If program operation was failed, a Flash error code is returned */
	if(FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	}
	else {
		/* If the program operation is completed, disable the PG Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
	}
	
	HAL_FLASH_Lock();
	
	/* Return last operation flash status */
	return FlashStatus;
}

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
