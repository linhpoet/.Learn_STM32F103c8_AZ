#include "stm32f10x.h"
#include <stdio.h>

int main()
{
	return(0);
}

void Flash_Unlock()
{
	/*2.3.2 This sequence consists of two write cycles, where two key values (KEY1
		and KEY2) are written to the FLASH_KEYR address (refer to Section 2.3.1 for key values)*/
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

void Flash_Erase(volatile uint32_t u32StartAddr)
{
	/*Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register*/
	/*BSY: Busy This indicates that a Flash operation is in progress. This is set on the beginning of a Flash
		operation and reset when the operation finishes or when an error occurs.*/
	/*Meaning: BSY = 1, have Flash operation; BSY = 0, no Flash memory operation*/
	/*trong define FLASH_SR_BSY = 0x01; so if bit 0 of Flash_CR register (BSY) = 1, the condition is correct */
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*doi cho cong viec phia trc da xong*/
	}
	
	/*Check flash_CR_LOCK*/
	/*FLASH_CR_LOCK = 0x80 = 0b10000000, cau lenh nay nghia la kiem tra bit thu 8 trong thanh ghi CR*/
	if (FLASH->CR & FLASH_CR_LOCK == FLASH_CR_LOCK)
	{
		/*perform unlock sequency*/
		Flash_Unlock();
	}
	/*Set the Per bit in the flash register*/
	/*FLASH_CR_PER = 1<<2*/
	FLASH->CR |= FLASH_CR_PER;
	
	/*write into flash_AR register to select a page to erase*/
	FLASH->AR = u32StartAddr;
	
	/*Set FLASH_CR_STRT*/
	FLASH->CR |= FLASH_CR_STRT;
	
	/*wait for the BSY bit to be reset*/
	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		
	}
	/*clear PER bit in the FLASH_CR register*/
	FLASH->CR &= ~FLASH_CR_PER;
	
	/*clear STRT bit in the FLAHS_CR register*/
	FLASH->CR &= ~FLASH_CR_STRT;
}

/*u32StartAddr: dia chi page can write; u8BuffetWrite: du lieu can write; u32Length:chieu dai can write*/
FlashStatus Flash_write(volatile uint32_t u32StartAddr, uint8_t* u8BuffetWrite, uint32_t u32Length)
{
	uint32_t u32Count = 0;
	/*check input paras*/
	if ((u8BuffetWrite == 0x00) || (u32Length == 0) || (u32Length % 2 != 0))
	{
		return FLASH_ERRORS;
	}
	/*Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register*/
	while ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*doi cho cong viec phia trc da xong*/
	}
	
	/*Check flash_CR_LOCK*/
	/*FLASH_CR_LOCK = 0x80 = 0b10000000, cau lenh nay nghia la kiem tra bit thu 8 trong thanh ghi CR*/
	if (FLASH->CR & FLASH_CR_LOCK == FLASH_CR_LOCK)
	{
		/*perform unlock sequency*/
		Flash_Unlock();
	}
	/*set FLASH_CR_PG */
	FLASH->CR |= FLASH_CR_PG;
	
	/*Perform half-word write at the desired address*/
	for (u32Count = 0;u32Count < (u32Length/2); u32Count++)
	{
		*(uint16_t*)(u32StartAddr + u32Count*2) = *(uint16_t*)(u8BuffetWrite + u32Count*2);
	}
	
	/*wait for the BSY bit to be reset*/
	while((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		
	}
	/*clear PG bit in FLASH_CR register*/
	FLASH->CR &= ~FLASH_CR_PG;
	
	return FLASH_NO_ERRORS;
}

FlashStatus Flash_Read(volatile uint32_t u32StartAddr




