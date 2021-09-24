#include "stm32f10x.h"


void GPIO_Config();
void Delay_Ms(uint32_t u32Delay);
void Interrupt_Config();
void EXTI0_IRQHandler(void);
void EXTI0_StandardLib();


int main()
{
	GPIO_Config();
	Interrupt_Config();
	GPIOC->ODR = 0xffff;
	while(1)
	{

	}
}

void EXTI0_IRQHandler(void)
{
	if ((EXTI->PR) && (EXTI->IMR))
	{
	/*	
		clear pending
		This bit is cleared by writing a ‘1’ into the bit.
	*/
		EXTI->PR |= 0x01;
		/*toggle gpioC*/
		GPIOC->ODR = ~(GPIOC->ODR);
	}		
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

void Interrupt_Config()
{
	/*PA0 input interrupt request*/
	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);
	/*enable clock for afioen I/O*/
	RCC->APB2ENR |= 0x01;
	/*chon PA0 lam dau vao cho exti0*/
	/*exticr[0] o day nhung trong thanh ghi thi tac dong vao exticr1*/
	AFIO->EXTICR[0] = 0x00;
	/*Fall trigger enable for line 0*/
	EXTI->FTSR |= 0x01;
	/*Rising trigger disabled for line 0*/
	EXTI->RTSR &= ~0x01;	
	/*disable Software event*/
	EXTI->SWIER &= ~0x01;
	EXTI->EMR &= ~0x01;
	/*Enable interrupt mask*/
	EXTI->IMR |= 0x01;
/*	clear pending
		This bit is cleared by writing a ‘1’ into the bit.
*/
	EXTI->PR |= 0x01;
	/*enable EXTI0 in cortex m3*/
	NVIC->ISER[0] = 0x40;
}

void GPIO_Config()
{	
	/*enable clock for GPIO_A,B,C,D,E*/
	RCC->APB2ENR |= 1<<2;
	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<4;
	RCC->APB2ENR |= 1<<5;
	RCC->APB2ENR |= 1<<6;

	/*GPIOC[15:8] output Pushpull*/
	GPIOC->CRH = 0x11111111;
}

void Delay_Ms(uint32_t u32Delay)
{
	while(u32Delay)
	{
		SysTick->LOAD = 72*1000-1;
		SysTick->VAL = 0;
		SysTick->CTRL = 5;
		while (!(SysTick->CTRL & (1<<16)))
		{
			
		}
		--u32Delay;
	}
}