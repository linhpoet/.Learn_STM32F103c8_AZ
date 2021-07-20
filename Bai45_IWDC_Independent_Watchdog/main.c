#include "stm32f10x.h"

void GPIO_Lib_Config();
void GPIO_Register_Config();
void IWDG_Register_Config();
void Delay_ms(uint32_t u32Delay);

int main()
{
	GPIO_Lib_Config();
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	IWDG_Register_Config();
	Delay_ms(1000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	while(1)
	{
		/*update watchdog timeout de khong bi reset*/
		IWDG_Register_Config();
	}
}

void IWDG_Register_Config()
{
	/*Writing the key value 5555h to enable access to the IWDG_PR and IWDG_RLR registers*/
	IWDG->KR = 0x5555;
	/*Prescaller divider /32 */
  IWDG->PR = 3;
	/*timeout = 2s =(1/ (40000/32)) * (2499+1)*/
	IWDG->RLR = 2499;
	/*Whenever the key value 0xAAAA is written in the IWDG_KR register, the IWDG_RLR value
		is reloaded in the counter and the watchdog reset is prevented*/
	IWDG->KR = 0xaaaa;
	/*start watchdog*/
	IWDG->KR = 0xcccc;
}

void GPIO_Register_Config()
{
	/*GPIOC output pushpull 10Mhz*/
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH = 0x11111111;
}

void GPIO_Lib_Config()
{
	GPIO_InitTypeDef GPIO_InitStrcture;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStrcture.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStrcture.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStrcture.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStrcture);
}

void Delay_ms(uint32_t u32Delay)
{	
	while (u32Delay)
	{
	/*dem ve 0*/
	SysTick->LOAD = 72000 - 1;
	SysTick->VAL = 0;
	/*clear couterFlag; clock src selection is Processor clock(AHB) ; enable counter*/
	SysTick->CTRL = 0x05;
	
	//while (!(SysTick->CTRL & (1<<16)))
	while((SysTick->CTRL & (1<<16)) == 0)
	{
		/*wait until counterFlag = 1, 1ms*/
	}
	--u32Delay;
	}
}