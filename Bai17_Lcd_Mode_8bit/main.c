#include "Stm32_Lib_LCD.h"


void __delay_ms(uint32_t u32Delay);
void GPIO_Register_Config();
int main(void)
{
	GPIO_Register_Config();
	lcd_command(Mode8bit2line);																//8bit, 2 dong
	lcd_command(DisplayOn);																//bat hien thi
	lcd_command(ClearDisplay);
	lcd_string("lcd mode 8 bit");
	lcd_command(Goto2);
	int rate = 222;	
	lcd_Number(rate);

  while (1)
  {
  }
  
}

void __delay_ms(uint32_t u32Delay)
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

void GPIO_Register_Config()
{
	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<4;
	RCC->APB2ENR |= 1<<2;
	/*GPIOC[15:8] output Pushpull*/
	GPIOB->CRL = 0x11111111;
	GPIOA->CRL = 0x11111111;
}



