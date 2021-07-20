#include "stm32f10x.h"

uint32_t CRC_Calculation(uint32_t* pInputCRC, uint32_t u32Length);
void DMA_ConfigChannel_1( uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTransfer);
uint32_t u32Source[8] = {1,2,3,4,5,6,7,9};
uint32_t u32Destination[8] = {};
	/*tinh chieu dai cua mang*/
uint32_t u32Length = sizeof(u32Source) / sizeof(uint32_t);
uint32_t u32CRCCal1;
uint32_t u32CRCCal2;
int main()
{
	/*truyen mang*/
	DMA_ConfigChannel_1((uint32_t *)&u32Source, (uint32_t *)u32Destination, u32Length);
	u32CRCCal1 = CRC_Calculation(u32Source, u32Length);
	u32CRCCal2 = CRC_Calculation(u32Destination, u32Length);

	while(1)
	{
	}
}

/*	3 gia tri truyen vao ham DMA la: 	dia chi bat dau
																			dia chi di chuyen toi(destination)
																			size cua du lieu*/

void DMA_ConfigChannel_1( uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTransfer)
{
	/*enable clock for DMA1*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*
	1.	Set the perifheral register address in the DMA_CCARx register
			The data will be moved from/to this address to/from the memory after the peripheral event
	*/
	DMA1_Channel1->CPAR = (uint32_t)pStartAddress;
	/*
	2.	Set the memory address in the DMA_CMARx register. 
			The data will be written to or read from this memory after the peripheral event.
	*/
	DMA1_Channel1->CMAR = (uint32_t) pDestination;
	/*
	3.	Configure the total number of data to be transferred in the DMA_CNDTRx register.
			After each peripheral event, this value will be decremented.
	*/
	DMA1_Channel1->CNDTR = u32NumberDataTransfer;
	/*
	4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	5.	Configure data transfer direction, circular mode, peripheral & memory incremented
			mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
			DMA_CCRx register
			
			Bit14 Mem2Mem = 1			; vi dang lam memory to memory
			Bits 13:12 PL = 10		; channel priority level : high
			Bits 11:10 Msize = 10	; memory size = 32 bit, so bit cua data bo nho
			Bits 9:8	Psize = 10	; peripheral size = 32 bit, so bit cua data ngoai vi
			Bit 	7		MINC		= 1	; Memory increment mode enabled
			Bit 	6		PINC 		= 1	; Peripheral increment mode enabled
			Bit 	5 	CIRC		= 1	;	bang 1 thi transfer lien tuc lap lai, neu =0 thi chi transfer 1 lan
			Bit 	4 	DIR			= 0	;	Read from peripheral
			Bit 	3 	TEIE		= 0	; TE interrupt enabled, cho ph?p ng?t khi c? l?i trong qu? tr?nh truy?n hay kh?ng.
			Bit		2		HTIE		= 0	;	HT interrupt disabled, cho ph?p ng?t khi truy?n xong data ? ch? d? half word.	
			Bit 	1 	TCIE		= 0 ; TC interrupt enabled, cho ph?p ng?t khi truy?n xong data ? ch? d? word.
			Bit 	0 	EN			=	1	;	channel is enable
	*/
	DMA1_Channel1->CCR |= 0b110101011100001;
	/*6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register*/
	DMA1_Channel1->CCR |= 0x01;
	
}

uint32_t CRC_Calculation(uint32_t* pInputCRC, uint32_t u32Length)
{
	uint32_t u32Count;
	/*enable clock for CRC*/
	RCC->AHBENR |= 0x40;
	/**/
	CRC->DR = 0;
	/*Resets the CRC calculation unit and sets the data register to 0xFFFF FFFF*/
	CRC->CR |= 0x01;
	for(u32Count = 0; u32Count < u32Length; u32Count++)
	{
		CRC->DR = *pInputCRC;
		/*tro toi dia chi phan tu tiep theo*/
		pInputCRC++;
	}
	return (CRC->DR);
}


