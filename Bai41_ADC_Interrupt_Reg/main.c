#include "stm32f10x.h"

void GPIO_ADC_Config();
void ADC_Interrupt_Register_Config();
void ADC1_2_IRQHandler();
void DMA_ConfigChannel_1( uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTransfer);

#define NUMBER_OF_ADC_CHANNEL 8U
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001244C)
uint16_t u16Destination[NUMBER_OF_ADC_CHANNEL];
uint32_t u32AdcValueIRQ;
uint32_t u32AdcValueMain;
uint32_t u32Count;

int main()
{
	GPIO_ADC_Config();
	ADC_Interrupt_Register_Config();
	//DMA_ConfigChannel_1(ADC1_DR_ADDRESS, &u16Destination, NUMBER_OF_ADC_CHANNEL);
	while(1)
	{
				u32AdcValueMain =(uint32_t)ADC1->DR;

	}
}

void GPIO_ADC_Config()
{
	/*enable clock for gpioA*/
	RCC->APB2ENR |= 1<<2;
	/*port A0-A7: analog input mode*/
	GPIOA->CRL = 0;	
}

void ADC_Interrupt_Register_Config()
{
	/*enable clock for ADC1*/
	RCC->APB2ENR |= 1<<9;
	
	/*ADC independence mode*/
	ADC1->CR1 &= ~(0x1111 << 16);
	/*ADC scan mode enable*/
	ADC1->CR1 |= 1<<8;
	/*Enable continuous conversion mode*/
	ADC1->CR2 |= 1<<1;
	/*Bits 19:17 EXTSEL[2:0]: External event select for regular group: 111: SWSTART */
	ADC1->CR2 |= 7<<17;
	/*0: Right Alignment*/
	ADC1->CR2 &= ~(1<<11);
	/*SQR1 Bits 23:20 L[3:0]: 8 conversion*/
	ADC1->SQR1 |= (8-1)<<20;
	
	/*ADC sample time 41.5 cycles for 8 channel 0:7*/
	ADC1->SMPR2 |= 0b100100100100100100100100;
	
	/*1: Enable ADC and to start conversion*/
	ADC1->CR2 |= 0x01;
	/*enable DMA for ADC1*/
	ADC1->CR2 |= 1<<8;	
	/*enable adc1 reset calibration register*/
	ADC1->CR2 |= 1<<3;
	/*check the end of adc1 reset calibration register : while until Calibration completed*/
	while(ADC1->CR2 & 1<<3);
	/*start adc1 calibration*/
	ADC1->CR2 |= 1<<2;
	/*check the end of adc1 calibration*/
	while(ADC1->CR2 & 1<< 2)
	/*start adc1 software conversion*/
	ADC1->CR2 |= 5<<20;
	
	/*		ADC Interrupt Config		*/
	/*clear eoc flag*/
	ADC1->SR &= ~(1<<1);
	/*EOC interrupt enabled. An interrupt is generated when the EOC bit is set.*/
	ADC1->CR1 |= 1<<5;
	/*enable global interrupt*/
	NVIC->ISER[0] |= 1<<18;
	
		
}

void ADC1_2_IRQHandler()
{
	/*eoc interrupt = 1 && eoc flag == 1 ??*/
	if ((ADC1->CR1 & (1<<5)) && (ADC1->SR & (1<<1)))
	{
		u32Count++;
		u32AdcValueIRQ =(uint32_t)ADC1->DR;
	}
	/*clear eoc flag*/
	ADC1->SR &= ~(1<<1);

}

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
			Bits 11:10 Msize = 01	; memory size = 16 bit, so bit cua data bo nho
			Bits 9:8	Psize = 01	; peripheral size = 16 bit, so bit cua data ngoai vi
			Bit 	7		MINC		= 1	; Memory increment mode enabled
			Bit 	6		PINC 		= 0	; Peripheral increment mode disabled
			Bit 	5 	CIRC		= 1	;	bang 1 thi transfer lien tuc lap lai, neu =0 thi chi transfer 1 lan
			Bit 	4 	DIR			= 0	;	Read from peripheral
			Bit 	3 	TEIE		= 0	; TE interrupt enabled, cho phép ng?t khi có l?i trong quá trình truy?n hay không.
			Bit		2		HTIE		= 0	;	HT interrupt disabled, cho phép ng?t khi truy?n xong data ? ch? d? half word.	
			Bit 	1 	TCIE		= 0 ; TC interrupt enabled, cho phép ng?t khi truy?n xong data ? ch? d? word.
			Bit 	0 	EN			=	1	;	channel is enable
	*/
	DMA1_Channel1->CCR |= 0b010010110100001;
	/*6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register*/
	DMA1_Channel1->CCR |= 0x01;
	
}
