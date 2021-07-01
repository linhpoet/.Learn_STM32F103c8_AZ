#include "stm32f10x.h"

int main()
{
	
}

void GPIO_ADC_Config()
{
	/*enable clock for gpioA*/
	RCC->APB2ENR |= 1<<2;
	/*port A0 analog input mode*/
	GPIOA->CRL = 0;
	
}

void ADC_Register_Config()
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
	ADC1->CR2 |= 8<<17;
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
	
}