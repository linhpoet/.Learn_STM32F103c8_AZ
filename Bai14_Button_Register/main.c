#include "stm32f10x.h"

int main()
{
	/*enable clock for PORTC and PORTB*/
	RCC->APB2ENR |= 1<<4;
	RCC->APB2ENR |= 1<<3;
	/*PC13 output max speed 10MHz- test*/
	GPIOC->CRH |= 1<<20;
	GPIOC->ODR &= ~(1<<13);
	/*config pc15 output puspull*/
	GPIOC->CRH |= 1<<28;
	GPIOC->CRH &= ~(1<<30);
	GPIOC->CRH &= ~(1<<31);
	
	while (1)
	{
		/*reset value of GPIOx->CRH = 0x44444444, so GPIOB15 in floating input mode- 0100*/
		/*read input pb15*/
		if (GPIOB->IDR & 1<<15)
			GPIOC->ODR |= 1<<15;
			else GPIOC->ODR &= ~(1<<15);
	}		
}
