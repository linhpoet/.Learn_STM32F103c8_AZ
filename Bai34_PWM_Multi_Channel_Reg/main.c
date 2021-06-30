/*linh*/
#include "stm32f10x.h"

void GPIO_Config_Lib();
void PWM_Timer2_Multi_Channel_Register(int Duty_Ch1,int Duty_Ch2,int Duty_Ch3,int Duty_Ch4);

int main()
{
	GPIO_Config_Lib();
	PWM_Timer2_Multi_Channel_Register(25,50,75,100);
	while(1)
	{
		
	}
}

/****GPIOA0,1,2,3 mode alternate function_ push pull*****/
void GPIO_Config_Lib()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

void PWM_Timer2_Multi_Channel_Register(int Duty_Ch1,int Duty_Ch2,int Duty_Ch3,int Duty_Ch4)
{
	/*enable clock for timer 2*/
	RCC->APB1ENR = 0x01;
	/*fTIM = 72M/7200 = 10000, tg 1 tick = 1/10000*/
	TIM2->PSC = (7200-1);
	/**/
	TIM2->ARR = (10000-1);
	
	/***config multi channel for pwm***/
	/* 110:	pwm mode, CH1 & CH2 */
	TIM2->CCMR1 = 0x6060;
	/* 110:	pwm mode 1, CH3, CH4*/
	TIM2->CCMR2 = 0x6060;
	
	/*select do rong for each CH, duty: do rong xung cao trong 1 chu ky pwm*/
	/*	10000: so tick trong 1 chu ki timer*/
	TIM2->CCR1 = (Duty_Ch1*10000)/100;
	TIM2->CCR2 = (Duty_Ch2*10000)/100;
	TIM2->CCR3 = (Duty_Ch3*10000)/100;
	TIM2->CCR4 = (Duty_Ch4*10000)/100;
	
	/*
	-	CC1E, CC2E, CC3E, CC4E: capture/compare 1,2,3,4 output enable
	-	CC1P, CC2P, CC3P, CC4P:	oc1 active high
	*/
	TIM2->CCER = 0x1111;
	/*bit CEN- counter enable*/
	TIM2->CR1 = 0x01;
	/*generate an update event*/
	TIM2->EGR = 0x01;
}


