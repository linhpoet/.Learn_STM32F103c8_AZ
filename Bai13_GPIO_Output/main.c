#include "stm32f10x.h"

void Delay_Ms(uint32_t u32Delay);
void GPIO_Register_Config();
void GPIO_Lib_Config();
void GPIO_Config_Lib();
#define dem1 0x80002312

int main()
{
	GPIO_Register_Config();
	int dem = 12;
	while(1)
	{
		/*3 cau lenh nay giong nhau*/
			/*GPIOC->ODR = 0xffff*/
			/*tạo 1 biến con trỏ kiểu struct GPIO_TypeDef, co dia chi tai GPIOx, và trỏ tới cac thanh phan trong struct*/
			/*((GPIO_TypeDef*) (((uint32_t)0x40000000) + 0x10000 + 0x1000)) ->ODR = 0xffff;*/			//kieu nhu dat struct gpio_typedef vao dia chi cua GPIOx
			/*GPIO_TypeDef *ptr = 0x40000000 + 0x10000 + 0x1000;
			ptr->ODR = 0xffff;*/
		GPIOC->ODR |= 1<<13;
		GPIOB->ODR |= 1<<9;
		Delay_Ms(100);
		GPIOC->ODR &= ~(1<<13);
		GPIOB->ODR &= ~(1<<9);

		Delay_Ms(100);
		dem++;
		//ADC_SMPR1_SMP17 = 0x18;
	}

}

void GPIO_Register_Config()
{	
	/*enable clock for GPIO_A,B,C,D,E*/
	RCC->APB2ENR |= 1<<2;
	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<4;
	RCC->APB2ENR |= 1<<5;
	RCC->APB2ENR |= 1<<6;

	/*GPIOC[15:8] output Pushpull*/
	GPIOC->CRH = 0x11111111;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
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