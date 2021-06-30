#include "stm32f10x.h"
#include <stdint.h>

void Delay_ms(uint32_t u32Delay);
void Start_Breakpoint();
void Stop_Breakpoint();
void SysTick_Handler();
float fTimeMeasurement;

int main()
{
	/*Test measure time of function */
	Start_Breakpoint();
	/*function bat ki*/
	Stop_Breakpoint();
	
	/*GPIOC output*/
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH = 0x11111111;
	while (1)
	{
		/*test Delay*/
		GPIOC->ODR |= 1<<13;
		Delay_ms(1000);
		GPIOC->ODR &= ~1<<13;
		Delay_ms(1000);			
	}
}


/*1 tick -> 1/72.000.000(s)*/
/*72000 tick->1ms=1/1000(s)*/
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


void Start_Breakpoint()
{
	SysTick->LOAD = 72000 - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x07;	
}

void Stop_Breakpoint()
{
	fTimeMeasurement = fTimeMeasurement + (71999 - (SysTick->VAL))*1/72000.0;
	/*disable counter*/
	SysTick->CTRL = 0x00;
}


void SysTick_Handler(void)
{
	fTimeMeasurement++;
}


