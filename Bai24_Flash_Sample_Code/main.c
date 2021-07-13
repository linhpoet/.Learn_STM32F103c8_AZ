#include "stm32f10x.h"

#define LENGTH_BUFFER 8U

#define FALSE 0U
#define TRUE 1U

typedef enum
{
	FLASH_ERRORS = 0U,    /* There is a error */
	FLASH_NO_ERRORS,      /* There is no errors */
	FLASH_PENDING,        /* Working is pending  */
	FLASH_ERRORS_TIMEOUT  /* There is a error due to timeout */
	} FlashStatus;

/**********************************************************************************************************************
*                                              FLASH
***********************************************************************************************************************/
FlashStatus Flash_Erase(volatile uint32_t u32StartAddr, uint32_t u32TimeOut);
FlashStatus Flash_Write_Syn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length);
FlashStatus Flash_Write_ASyn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length);
FlashStatus Flash_Read(volatile uint32_t u32StartAddr, uint8_t* u8BufferRead, uint32_t u32Length);
void Flash_Unlock(void);

FlashStatus Debug(volatile uint32_t u32StartAddr, uint32_t u32Test);

uint8_t aDataWrite[LENGTH_BUFFER] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
_Bool bAsyn = FALSE;
FlashStatus eReturnCode = FLASH_NO_ERRORS;

int main()
{
	uint8_t aDataBuffer[LENGTH_BUFFER] = {0};
	
	Flash_Erase((uint32_t)0x08001000, 100);
	if (FLASH_NO_ERRORS != Flash_Write_Syn((uint32_t)0x08001000, aDataWrite, LENGTH_BUFFER))
	{
		/* Errors occured*/
		while(1);
	}
	if (FLASH_NO_ERRORS != Flash_Read((uint32_t)0x08001000, aDataBuffer, LENGTH_BUFFER))
	{
		/*Errors occured*/
		while(1);
	}
	Flash_Erase((uint32_t)0x08001000, 100);
	
	return 0;	
}

/**********************************************************************************************************************
*                                              FLASH
***********************************************************************************************************************/
void Flash_Unlock(void)
{
	/* This sequence consists of two write cycles, where two key values (KEY1 and KEY2) are written to the FLASH_KEYR address*/
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

FlashStatus Flash_Erase(volatile uint32_t u32StartAddr, uint32_t u32TimeOut)
{
	/* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
	while(((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY) && (u32TimeOut > 0U))
	{
		/*  Wating for Bsy bit */
		u32TimeOut --;
		if (u32TimeOut == 0)
		{
			return FLASH_ERRORS_TIMEOUT;
		}
	}
	/* Check unlock sequences */
	if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
	{
		Flash_Unlock();
	}
	/* Set the PER bit in the FLASH_CR register */
  FLASH->CR |= FLASH_CR_PER;
	/* Program the FLASH_AR register to select a page to erase */
  FLASH->AR = u32StartAddr;
	/* Set the STRT bit in the FLASH_CR register */
  FLASH->CR |= FLASH_CR_STRT;
	/* Wait for the BSY bit to be reset */
  while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*  Wating for Bsy bit */
	}
	/* Clear PER bit in the FLASH_CR register */
  FLASH->CR &= ~FLASH_CR_PER; 
	/* Clear STRT bit in the FLASH_CR register */
	FLASH->CR &= ~FLASH_CR_STRT;
	
	return FLASH_NO_ERRORS;
}

FlashStatus Flash_Write_Syn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length)
{
	uint32_t u32Count = 0U;
	
	/* Check input paras */
	if((u8BufferWrite == 0x00U) || (u32Length == 0U) || u32Length%2U != 0U)
	{
		return FLASH_ERRORS;
	}
   /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
	while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*  Wating for Bsy bit */
	}
	/* Check unlock sequences */
	if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
	{
		Flash_Unlock();
	}
	/* Write FLASH_CR_PG to 1 */
	FLASH->CR |= FLASH_CR_PG;
  /* Perform half-word write at the desired address*/
	for(u32Count = 0U; u32Count < (u32Length/2); u32Count ++ )
	{
		*(uint16_t*)(u32StartAddr + u32Count*2U) = *(uint16_t*)(u8BufferWrite + u32Count*2U);
	}
	/* Wait for the BSY bit to be reset */
  while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*  Wating for Bsy bit */
	}
	/* Clear PG bit in the FLASH_CR register */
	FLASH->CR &= ~FLASH_CR_PG;
	
	return FLASH_NO_ERRORS;
}
FlashStatus Flash_Write_ASyn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length)
{
	uint32_t u32Count = 0U;
	FlashStatus ReturnCode;
	if( FALSE == bAsyn)
	{
	    /* Check input paras */
	    if((u8BufferWrite == 0x00U) || (u32Length == 0U) || u32Length%2U != 0U)
	    {
	    	return FLASH_ERRORS;
	    }
       /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
	    while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	    {
	    	/*  Wating for Bsy bit */
	    }
	    /* Check unlock sequences */
	    if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
	    {
	    	Flash_Unlock();
	    }
	    /* Write FLASH_CR_PG to 1 */
	    FLASH->CR |= FLASH_CR_PG;
      /* Perform half-word write at the desired address*/
	    for(u32Count = 0U; u32Count < (u32Length/2); u32Count ++ )
	    {
	    	*(uint16_t*)(u32StartAddr + u32Count*2U) = *(uint16_t*)(u8BufferWrite + u32Count*2U);
	    }
			bAsyn = TRUE;
			ReturnCode = FLASH_PENDING;
	}
	else
	{
	    /* Wait for the BSY bit to be reset */
      if((FLASH->SR&FLASH_SR_BSY) != FLASH_SR_BSY)
	    {
	    	 /* Clear PG bit in the FLASH_CR register */
	       FLASH->CR &= ~FLASH_CR_PG;
				 bAsyn = FALSE;
				 ReturnCode = FLASH_NO_ERRORS;
	    }
	}
	
	return ReturnCode;
}
FlashStatus Flash_Read(volatile uint32_t u32StartAddr, uint8_t* u8BufferRead, uint32_t u32Length)
{
	
	/* Check input paras */
	if((u8BufferRead == 0x00U) || (u32Length == 0U))
	{
		return FLASH_ERRORS;
	}
	do
	{
	   if(( u32StartAddr%4U == 0U) && ((uint32_t)u8BufferRead%4U == 0U) && (u32Length >= 4U))
		 {
		   *(uint32_t*)(u8BufferRead) = *(uint32_t*)(u32StartAddr);
		   u8BufferRead = u8BufferRead + 4U;
		 	 u32StartAddr = u32StartAddr + 4U;
			 u32Length = u32Length - 4U;
		 }
		 else if(( u32StartAddr%2U == 0U) && ((uint32_t)u8BufferRead%2U == 0U) && (u32Length >= 2U))
		 {
		    *(uint16_t*)(u8BufferRead) = *(uint16_t*)(u32StartAddr);
		 	  u8BufferRead = u8BufferRead + 2U;
		 	  u32StartAddr = u32StartAddr + 2U;
			  u32Length = u32Length - 2U;
		 }
		 else
		 {
		    *(uint8_t*)(u8BufferRead) = *(uint8_t*)(u32StartAddr);
		 	  u8BufferRead = u8BufferRead + 1U;
		 	  u32StartAddr = u32StartAddr + 1U;
			  u32Length = u32Length - 1U;
		 }
	}
	while(u32Length > 0U);
	
	return FLASH_NO_ERRORS;
}
