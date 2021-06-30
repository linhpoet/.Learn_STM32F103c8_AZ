#include "stm32f10x.h"

void Timer2_Config(void);
void Delay_Timer2_ms(uint32_t ms);
void Delay_Timer2_StandardLib_ms(uint32_t ms);
void Timer2_StandardLib_Config();
void GPIO_Config();

uint32_t u32CntInterrupt;

int main()
{
	//Timer2_Config();
	Timer2_Config();
	GPIO_Config();
	while(1);
	
}

void TIM2_IRQHandler()
{
	if (((TIM2->DIER & 0x01) == 1) && ((TIM2->SR & 0x01) == 1))
	{
		/*clear update interrupt flag*/
	TIM2->SR &= ~(0x01);
		/*counter test*/
		u32CntInterrupt++;
		/*if pc13 == 1*/
		if (GPIOC->ODR & (1<<13))
			GPIOC->ODR &= ~(1<<13);
		else GPIOC->ODR |= 1<<13;
		
	}
}

void GPIO_Config()
{
	/*GPIOC output pushpull 10Mhz*/
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH = 0x11111111;
}

void Delay_Timer2_ms(uint32_t ms)
{
	while(ms)
	{
		TIM2->CNT = 0;
		while((TIM2->CNT) < 998);
		ms --;
	}
}
	
void Timer2_Config(void)
{
	/*enable clock for timer 2*/
	RCC->APB1ENR |= 0x01;
	/*select prescaller value- fTIM = 1/1.000.000*/
	TIM2->PSC = (72 -1);
	/*uesed as upcounter*/
	TIM2->CR1 &= ~(1<<4);
	/*count from 0*/
	TIM2->CNT = 0;
	/*select auto_reload value*/
	TIM2->ARR = 0xffff;
	/*Timer 2 Counter enable*/
	TIM2->CR1 |= 0x01;
	/* Generate an update event to reload the Prescaler and the Repetition counter values immediately */
	TIM2->EGR = 0x01;
	/*generate an update event by counter overflow- wait until cnt = 0 (overflow)*/
	//while (TIM2->CNT != 0);
}

void Delay_Timer2_StandardLib_ms(uint32_t ms)
{
	while (ms)
	{
		TIM_SetCounter(TIM2, 0);
		while ((TIM_GetCounter(TIM2)) < 1000);
		ms--;
	}
}

void Timer2_StandardLib_Config()
{
	TIM_TimeBaseInitTypeDef TimerInit;
	/*enable clock for timer 2*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TimerInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerInit.TIM_Period = 0xffff;
	TimerInit.TIM_Prescaler = 72 - 1;
	
	/*gan cac gia tri o struct TimerInit cho cac thanh ghi tac dong timer 2*/
	TIM_TimeBaseInit(TIM2, &TimerInit);
	/*enable timer 2*/
	TIM_Cmd(TIM2, ENABLE);
}


void Timer_Interrupt_Config(void)
{
	/*enable clock for timer 2*/
	RCC->APB1ENR |= 0x01;
	/*select prescaller value- fTIM = 10000*/
	TIM2->PSC = (7200 -1);
	/*uesed as upcounter*/
	TIM2->CR1 &= ~(1<<4);
	/*count from 0*/
	TIM2->CNT = 0;
	/*select auto_reload value*/
	TIM2->ARR = 10000 - 1;
	/*update timer interrupt enable*/
	TIM2->DIER = 0x01;
	/*clear update interrupt flag*/
	TIM2->SR &= ~(0x01);
	/*Timer 2 Counter enable*/
	TIM2->CR1 |= 0x01;
	/* Generate an update event to reload the Prescaler and the Repetition counter values immediately */
	TIM2->EGR = 0x01;
	/*generate an update event by counter overflow- wait until cnt = 0 (overflow)*/
	//while (TIM2->CNT != 0);
	/*enable timer 2 global interrupt*/
	NVIC->ISER[0] = 1<<28;	
}






