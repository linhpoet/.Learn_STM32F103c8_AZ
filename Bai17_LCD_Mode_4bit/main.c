#include "My_Lcd_4bit_lib.h"


void __delay_ms(uint32_t u32Delay);
void GPIO_Register_Config();
int main(void)
{
	GPIO_Register_Config();
	/*dua con tro ve dau man hinh*/
    lcd_command(0x02);                          //mode 4 bit
    lcd_command(Mode4bit2line);					//4bit, 2 dong
	lcd_command(DisplayOn);						//bat hien thi
	lcd_command(ClearDisplay);					//clear
	
	lcd_string("chuc mung");
    lcd_command(Goto2);
    int x =210+300;
    lcd_number(x);

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



