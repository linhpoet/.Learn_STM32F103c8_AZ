#include "stm32f10x.h"
#include "My_Lcd_8bit_lib.h"

void Delay_Ms(uint32_t u32Delay);

int main()
{
	RCC->APB2ENR |= 1<<2;
	RCC->APB2ENR |= 1<<3;
	/*PORTC[7:0] output push pull-0001*/
	GPIOB->CRL = 0x11111111;
	/*PORTB[7:0] output push pull-0001**/
	GPIOA->CRL = 0x11111111;
	
	lcd_command(Mode8bit2line);					//8bit, 2 dong
	lcd_command(DisplayOn);						//bat hien thi
	lcd_command(ClearDisplay);					//clear
	
	lcd_string(" Chuc 2 mung");
	while(1)
	{
		
	}
	
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
