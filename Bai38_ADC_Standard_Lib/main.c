#include "stm32f10x.h"

#define NUMBER_OF_ADC_CHANNEL 8U
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001244C)

volatile uint16_t u16AdcValues[NUMBER_OF_ADC_CHANNEL];

void GPIO_Lib_ADC_Config();
void ADC_Lib_Config();
void DMA_ConfigChannel_1( uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTransfer);

int main()
{
	GPIO_Lib_ADC_Config();
	ADC_Lib_Config();
	DMA_ConfigChannel_1((uint32_t *)ADC1_DR_ADDRESS, (uint32_t *)u16AdcValues, NUMBER_OF_ADC_CHANNEL);
	while(1)
	{
		
	}
}

void GPIO_Lib_ADC_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC_Lib_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_InitTypeDef ADC_InitStructure;
	/*ADC configuration*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 8;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*config each channel*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5);
	
	/*enable adc 1*/
	ADC_Cmd(ADC1, ENABLE);
	/*enable DMA for ADC*/
	ADC_DMACmd(ADC1, ENABLE);
	/*enable adc1 reset calibration register*/
	ADC_ResetCalibration(ADC1);
	/*cheack the end of adc1 reset calibration register*/
	while(ADC_GetResetCalibrationStatus(ADC1));
	/*start adc1 calibration*/
	ADC_StartCalibration(ADC1);
	/*check the end of adc1 calibration*/
	while(ADC_GetCalibrationStatus(ADC1));
	/*start adc1 software conversion*/
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
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

