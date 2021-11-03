#include "stm32f10x.h"

void Delay_ms(uint32_t u32Delay);
void DMA_ConfigChannel_1( uint32_t *PeripheralAddr, uint32_t *MemoryAddr, uint32_t u32NumberDataTransfer);
uint16_t u16Source[8] = {1,2,3,4,5,6,7,8};
uint16_t u16Destination[8] = {9,10,11,12,13,14,15,16};
uint16_t u16Start = 1;
int main()
{
	DMA_ConfigChannel_1((uint32_t *)&u16Source, (uint32_t *)u16Destination, 8);
	while(1)
	{
		u16Start++;
		Delay_ms(1000);
	}
}


void DMA_ConfigChannel_1( uint32_t *PeripheralAddr, uint32_t *MemoryAddr, uint32_t u32NumberDataTransfer)
{
	/*enable clock for DMA1*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*
	1.	Set the perifheral register address in the DMA_CCARx register
			The data will be moved from/to this address to/from the memory after the peripheral event
	*/
	DMA1_Channel1->CPAR = (uint32_t)PeripheralAddr;
	/*
	2.	Set the memory address in the DMA_CMARx register. 
			The data will be written to or read from this memory after the peripheral event.
	*/
	DMA1_Channel1->CMAR = (uint32_t) MemoryAddr;
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
			Bits 11:10 Msize = 01	; memory size = 16 bit, so bit cua data bo nho
			Bits 9:8	Psize = 01	; peripheral size = 16 bit, so bit cua data ngoai vi
			Bit 	7		MINC		= 1	; Memory increment mode enabled
			Bit 	6		PINC 		= 1	; Peripheral increment mode enable
			Bit 	5 	CIRC		= 1	;	bang 1 thi transfer lien tuc lap lai, neu =0 thi chi transfer 1 lan
			Bit 	4 	DIR			= 0	;	Read from peripheral
			Bit 	3 	TEIE		= 0	; TE interrupt enabled, cho ph�p ng?t khi c� l?i trong qu� tr�nh truy?n hay kh�ng.
			Bit		2		HTIE		= 0	;	HT interrupt disabled, cho ph�p ng?t khi truy?n xong data ? ch? d? half word.	
			Bit 	1 	TCIE		= 0 ; TC interrupt enabled, cho ph�p ng?t khi truy?n xong data ? ch? d? word.
			Bit 	0 	EN			=	1	;	channel is enable
	*/
	DMA1_Channel1->CCR |= 0b110010111100001;
	/*6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register*/
	DMA1_Channel1->CCR |= 0x01;
	
}

void Delay_ms(uint32_t u32Delay)
{	
	while (u32Delay)
	{
	/*dem ve 0*/
	SysTick->LOAD = 72000 - 1;
	SysTick->VAL = 0;
	/*clear couterFlag; clock src selection is Processor clock(AHB) ; enable counter*/
	SysTick->CTRL = 0x05;
	
	//while (!(SysTick->CTRL & (1<<16)))
	while((SysTick->CTRL & (1<<16)) == 0)
	{
		/*wait until counterFlag = 1, 1ms*/
	}
	--u32Delay;
	}
}
