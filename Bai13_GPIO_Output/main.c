#include "stm32f10x.h"

void Delay_Ms(uint32_t u32Delay);
void GPIO_Register_Config();
void GPIO_Lib_Config();
void GPIO_Config_Lib();

int main()
{
	GPIO_Lib_Config();
	
	while(1)
	{
		/*3 cau lenh nay giong nhau*/
		/*GPIOC->ODR = 0xffff*/
		/*tạo 1 biến con trỏ kiểu struct GPIO_TypeDef, và trỏ tới các địa chỉ trong struct*/
		/*((GPIO_TypeDef*) (((uint32_t)0x40000000) + 0x10000 + 0x1000))->ODR = 0xffff;*/
		/*GPIO_TypeDef *ptr = 0x40000000 + 0x10000 + 0x1000;
		ptr->ODR = 0xffff;*/
		
		GPIOC->ODR = 0xffff;
		Delay_Ms(1000);
		GPIOC->ODR = 0x00;
		Delay_Ms(1000);
	}

}

void GPIO_Register_Config()
{
	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<4;
	/*GPIOC[15:8] output Pushpull*/
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