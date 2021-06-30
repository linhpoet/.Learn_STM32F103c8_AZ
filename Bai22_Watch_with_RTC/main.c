#include "stm32f10x.h"
#include <stdint.h>
#include "My_Lcd_8bit_lib.h"

void RTC_Config();
void GPIO_Config();
void Delay_Ms(uint32_t u32Delay);
int cnt;
uint8_t u8hour,u8minute,u8second;
uint8_t u8hour_alarm,u8minute_alarm;

int main()
{
	RTC_Config();
	/*LCD_config*/
	GPIO_Config();	
	lcd_command(Mode8bit2line);					//8bit, 2 dong
	lcd_command(DisplayOn);						//bat hien thi
	lcd_command(ClearDisplay);					//clear
	
	while(1)
	{
		lcd_command(ClearDisplay);
		lcd_number(u8hour);
		lcd_string(":");
		lcd_number(u8minute);
		lcd_string(":");
		lcd_number(u8second);
	}
}

void GPIO_Config()
{
	/*eable GPIOA,B,C*/
	RCC->APB2ENR |= 1<<2;
	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<4;
	/*output push pull 10Mhz, 0-7*/
	GPIOB->CRL = 0x11111111;
	GPIOA->CRL = 0x11111111;
	/**/
	GPIOC->CRH = 0x11111111;
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
	
	/*SECIE: Second interrupt enable*/
	 RTC->CRH |= 0x01;
	 /*Clear Interrupt flag-SECF*/
	 RTC->CRL &= ~0x01;
	
	/*4. Clear the CNF bit to exit configuration mode*/
	RTC->CRL &= ~(1<<4);
	/*5. Poll RTOFF, wait until its value goes to ‘1’ to check the end of the write operation.*/
	while (((RTC->CRL >> 5) & 0x01) != 1);
	
	/*enable interrupt in core, trong 10.1.2 Table 61. Vector table for connectivity line devices*/
	NVIC->ISER[0] |= 1<<3;
}

void RTC_IRQHandler()
{
	/*Check enable interrupt(SECIE) and interrupt flag*/
	if (((RTC->CRH	& 0x01) == 1) && ((RTC->CRL & 0x01) == 1))
	{
		/*clear interrupt flag*/
		RTC->CRL &= ~0x01;
		u8second++;
		if (u8second == 60)
		{
			u8second = 0;
			u8minute++;
			if (u8minute == 60)
			{
				u8minute = 0;
				u8hour++;
				if (u8hour == 24)
				{
					u8hour = 0;
				}
			}
		}
		/*Alarm*/
		if((u8hour_alarm == u8hour) && (u8minute_alarm == u8minute))
				GPIOC->ODR |= 1<<13;
		else GPIOC->ODR &= ~1<<13;
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