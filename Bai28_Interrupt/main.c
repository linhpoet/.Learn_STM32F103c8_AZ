#include "stm32f10x.h"

void GPIO_Config();
void EXTI0();
void EXTI0_IRQHandler();

int main()
{
	GPIO_Config();
	EXTI0();
	GPIOC->ODR = 0	;
	while (1);
}

void GPIO_Config()
{
	/*enable GPIOA*/
	RCC->APB2ENR |= 1<<2;
	/*GPIOA PortA0 input pull up or down*/
	GPIOA->CRL = 0x08;
	/*GPIOA PortA0 input pull up*/
	GPIOA->BSRR |= 1<<0;
	/*GPIOC output pushpull 10Mhz*/
	RCC->APB2ENR |= 1<<4;
	/*gpioC config*/
	GPIOC->CRH = 0x11111111;
}

void GPIO_StandardLib_Config()
{
	GPIO_InitTypeDef GPIO_Initstructure;
	/*enable clock for GPIOA*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_Initstructure);
}

void EXTI0()
{
	/*AFIO enable*/
	RCC->APB2ENR |= 0x01;
	/*PA0 exti0*/
	AFIO->EXTICR[1] = 0x00;
	/*Fall trigger enable for line 0*/
	EXTI->FTSR |= 0x01;
	/*Rising trigger disabled for line 0*/
	EXTI->RTSR &= ~0x01;	
	/*disable Software event*/
	EXTI->SWIER &= ~0x01;
	EXTI->EMR &= ~0x01;
	/*Enable interrupt mask*/
	EXTI->IMR |= 0x01;
	/*clear pending */
	EXTI->PR &= ~0x01;
	NVIC->ISER[0] = 0x40;
}

void EXTI0_StandardLib()
{
	EXTI_InitTypeDef 	EXTI_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/*enable clock for afioen*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/*external interrupt configuration register 1 (AFIO_EXTICRL)*/
	/* 0001: PA[x] pin*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	/*clear interrupt exti0 flag*/
	EXTI_ClearITPendingBit(EXTI_Line0);
	/*exti line configuration*/
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/*NVIC configuration*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void)
{
	if ((EXTI->PR) && (EXTI->IMR))
	{
		/*clear EXTI0 flag*/
		EXTI->PR &= ~0x01;
		/*toggle gpioC*/
		GPIOC->ODR = ~(GPIOC->ODR);
	}		
}