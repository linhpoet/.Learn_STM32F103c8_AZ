#include "stm32f10x.h"

void GPIO_Config_Lib();
void GPIO_Config();
void PWM_Timer2_Register(int duty);

int main()
{
	/*GPIOA*/
	GPIO_Config_Lib();
	PWM_Timer2_Register(1);
	while(1)
	{	
		
	}
}

void GPIO_Config_Lib()
{
	GPIO_InitTypeDef	GPIO_Initstructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_Initstructure);
}

void GPIO_Config()
{
	/*GPIOC output pushpull 10Mhz*/
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH = 0x11111111;
	/*enable clock for PORTA*/
	RCC->APB2ENR |= 1<<2;
	/*GPIOCA output pushpull 10Mhz*/
	GPIOA->CRH = 0x11111111;
}

void PWM_Timer2_Register(int duty)
{
	/*enable clock for timer 2*/
	RCC->APB1ENR |= 0x01;
	/*select auto reload value- so tick trong 1 chu ki PWM=> Tpwm = (ARR+1)*1/fTIM*/
	TIM2->ARR = 10000-1;
	/*select prescaller value- fTIM =72M/(PSC+1) = 10000*/
	TIM2->PSC = (7200-1);
	/*110-PWM mode 1*/
	TIM2->CCMR1 = 0x60;
	/*select duty cycle-gia tri %: thoi gian muc 1 trong 1 chu ky pwm*/
	TIM2->CCR1 = (duty*10000/100);
	/*
		-CC1E: Capture/compare 1 output enable
		-CC1P: 0: OC1 active high or 1: low
	*/
	TIM2->CCER = 0x01;
	/*bit 0 CEN: Counter enable and 0: upcounter*/
	TIM2->CR1 = 0x01;
	/*generate an update event to reload the Prescaller and the Repetition counter values immediately	*/
	TIM2->EGR = 0x01;
}