#include "stm32f10x.h"

int main()
{
	
}

void GPIO_ADC_Config()
{
	/*enable clock for gpioA*/
	RCC->APB2ENR |= 1<<2;
	/*port A0 analog input mode*/
	GPIOA->CRL |= 0b0000
	
}

void ADC_Register_Config()
{
	/*enable clock for ADC1*/
	RCC->APB2ENR |= 1<<9;
	/*0: Right Alignment*/
	ADC1->CR2 &= ~(1<<11);
	/*Continuous conversion mode*/
	ADC1->CR2 |= 1<<1;
	
	/*1: Enable ADC and to start conversion*/
	ADC1->CR2 |= 0x01;
	
	
}