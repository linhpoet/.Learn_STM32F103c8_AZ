#include "stm32f10x.h"

void Gpio_Init(void);
void Spi_2_Init(void);
void SPI2_SentData(uint8_t u8data);
void Delay_ms(uint32_t u32Delay);

uint8_t u8Buff[256U] = {0U};


int main(void)
{
	for (int i=0;i < 256; i++)
	{
		u8Buff[i] = i;
	}
	uint8_t u8Data = 0x80;
	Gpio_Init();
	Spi_2_Init();
	
	uint16_t count = 0;
	
	int i = 0;
	while (1) 
	{
		/*b12 = 0*/
		GPIOB->ODR &= ~(1<<12);
		for(int i=0; i<256; i++)
		{
			/*sent data*/
			SPI2->DR = u8Buff[i];
			/*wait*/
			while (((SPI2 ->SR) & SPI_I2S_FLAG_BSY) != 0)
			{
		
			}
		}
		/*b12 = 1*/
		GPIOB->ODR |= 1<<12;	
	}
	
	
}
void Gpio_Init(void)
{
	GPIO_InitTypeDef gpioInit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_13;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &gpioInit);
	
	/* PB12 - CS */
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_12;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	/* PB13 - SCK */
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_13;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	
	/* PB14 - MISO */
	gpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioInit.GPIO_Pin = GPIO_Pin_14;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	
	/* PB15 - MOSI */
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_15;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
}

void Spi_2_Init(void)
{
	/* Enable Clock*/
	RCC->APB1ENR |= 0x00004000;
	
	/*  BR[2:0]: Baud rate control -> 100: fPCLK/32 */
	SPI2->CR1 |= 0x20U;

	/* Bit 2 MSTR: Master selection -> 1: Master configuration */
	SPI2->CR1 |= 0x04U;
	/* SSM: Software slave management -> 1: Software slave management enabled
	      SSI: Internal slave select
	*/
	SPI2->CR1 |= 0x300U;
	
	/* Bit1 CPOL: Clock polarity -> 0: CK to 0 when idle*/
	//SPI2->CR1 |= 0x2;
	
	/* Bit 0 CPHA: Clock phase -> 0: CK to 0 when idle*/
	//SPI2->CR1 |= 0x1;
	
	/* SPI enable */
	SPI2->CR1 |= 0x40U;
	
}

void SPI2_SentData(uint8_t u8data)
{
	SPI2->DR = u8data;
	while (((SPI2 ->SR) & SPI_I2S_FLAG_BSY) != 0)
	{
		
	}
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