#include "stm32f10x.h"

int main()
{
	
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
			
			Bit14 Mem2Mem = 0			; vi dang lam ADC to memory
			Bits 13:12 PL = 10		; channel priority level : high
			Bits 11:10 Msize = 00	; memory size = 8 bit
			Bits 9:8	Psize = 01	; peripheral size = 16 bit
			Bit 	7		MINC		= 0	;
			Bit 	6		PINC 		= 1	;
			Bit 	5 	CIRC		= 0	;	chi transfer 1 lan, neu bang 1 thi transfer lien tuc lap lai
			Bit 	4 	DIR			= 1	;	Read from memory
			Bit 	3 	TEIE		= 1	; TE interrupt enabled
			Bit		2		HTIE		= 0	;	HT interrupt disabled
			Bit 	1 	TCIE		= 1 ; TC interrupt enabled
			Bit 	0 	EN			=	0	;	channel is enable
	*/
	DMA1_Channel1->CCR |= 0b010001001011010;
	/*6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register*/
	DMA1_Channel1->CCR |= 0x01;
	
}