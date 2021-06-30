#include "stm32f10x.h"
#include <stdint.h>

void Delay_ms(uint32_t u32Delay);
void Start_Breakpoint();
void Stop_Breakpoint();
void SysTick_Handler();
float fTimeMeasurement = 0.0;
void RTC_Config();

int cnt;

int main()
{
	
	/*Test measure time of function */
	RTC_Config();
	Start_Breakpoint();
	/*delay 1s*/
	RTC->CRL &= ~0x01;
	while ((RTC->CRL & 0x01) == 0);
	Stop_Breakpoint();
	
	/*GPIOC output*/
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH = 0x11111111;
	while (1)
	{
		/*blink led f=1hz*/
		GPIOC->ODR &= ~1<<13;
		RTC->CRL &= ~0x01;
		while ((RTC->CRL & 0x01) == 0);
		GPIOC->ODR |= 1<<13;
		RTC->CRL &= ~0x01;
		while ((RTC->CRL & 0x01) == 0);
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
	/*calculate time*/
	fTimeMeasurement = fTimeMeasurement + (71999 - (SysTick->VAL))*1/71999.0;
	/*disable counter*/
	SysTick->CTRL &= ~0x01;
}


void SysTick_Handler(void)
{
	fTimeMeasurement =fTimeMeasurement + 1.0;
}


void RTC_Config()
{
	/*BKPEN = 1- enable backup interface*/
	RCC->APB1ENR |= 1<<27;
	/*PWREN = 1- enable power interface*/
	RCC->APB1ENR |= 1<<28;
	/*DPB = 1- access rtc and backup register enable*/
	PWR->CR |= 1<<8;
	
	/*LSEON = 1- external 32khz osc on (lse on)*/
	RCC->BDCR |=0x01;
	/*LSEDRY = 1: External 32 kHz oscillator ready, so wait until LSEDRY != 0*/
	while (((RCC->BDCR >> 1) & 0x01) != 1);
	/*	while ((RCC->BDCR & 1<<1) != 1); 2cau lenh tuong tu nhau */
	
	/*RTC clock source selection*/
	/*RTCSEL = 01- : LSE oscillator clock used as RTC clock*/
	RCC->BDCR |= 1<<8;
	RCC->BDCR &= ~(1<<9);
	
	/*RTC enable*/
	RCC->BDCR |= 1<<15;
	
	/*1. Poll RTOFF, wait until its value goes to ‘1’*/
	while (((RTC->CRL >> 5) & 0x01) != 1);
	/*2. Set the CNF bit to enter configuration mode*/
	RTC->CRL |= 1<<4;
	/*3. Write to one or more RTC registers*/
	/*fTR_CLK = fRTCCLK/(PRL[19:0]+1)*/
	RTC->PRLL = 0x7FFF;
	RTC->PRLH = 0x00;
	
	/*4. Clear the CNF bit to exit configuration mode*/
	RTC->CRL &= ~(1<<4);
	/*5. Poll RTOFF, wait until its value goes to ‘1’ to check the end of the write operation.*/
	while (((RTC->CRL >> 5) & 0x01) != 1);
}


