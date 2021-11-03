#include "stm32f10x.h"

void Gpio_Init(void);
void Spi_2_Init(void);
void Spi_2_Lib_Init();
void Spi_1_Lib_Init();
void SPI2_SentData(uint8_t u8data);
void SPI1_SentData(uint8_t u8data);
void Delay_ms(uint32_t u32Delay);

uint8_t u8Buff[256] = {0U};
#define SPI2DR (uint32_t)0x4000380C

int main(void)
{
	for (int i=0;i < 256; i++)
	{
		u8Buff[i] = i;
	}

	Gpio_Init();
	Spi_1_Lib_Init();
	
	uint16_t count = 0;
	
	while (1) 
	{
		for (int i=0; i<256; i++)
		SPI1_SentData(u8Buff[i]);
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
	
	/*16bit data format*/
	//SPI2->CR1 |= 1<<11;
	
	/* SPI enable */
	SPI2->CR1 |= 0x40U;
	
}


void Spi_2_Lib_Init()
{
	/*
	GPIO_InitTypeDef gpioInit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_13;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &gpioInit);
	
	// PB12 - CS 
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_12;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	// PB13 - SCK 
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_13;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	
	// PB14 - MISO 
	gpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioInit.GPIO_Pin = GPIO_Pin_14;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	
	// PB15 - MOSI 
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_15;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInit);
	*/
	
	SPI_InitTypeDef SPI_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/*toc do SPI = 36MHz/32*/
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	/*bat dau truyen tai suon dau tien*/
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	/*khi k truyen thi o muc CK = LOW*/
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	/*SPI_CRCPR- chua da thuc tinh CRC - bai 48*/
	SPI_InitStruct.SPI_CRCPolynomial = SPI_CRC_Tx;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	/*truyen tu bit trong so cao den thap-trai sang phai*/
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	/*Bit 9 SSM: Software slave management enable*/
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2, &SPI_InitStruct);
	
	SPI_Cmd(SPI2, ENABLE);
	
}

void Spi_1_Lib_Init()
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	SPI_InitTypeDef SPI_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/*toc do SPI = 36MHz/32*/
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	/*bat dau truyen tai suon dau tien*/
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	/*khi k truyen thi o muc CK = LOW*/
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	/*SPI_CRCPR- chua da thuc tinh CRC - bai 48*/
	SPI_InitStruct.SPI_CRCPolynomial = SPI_CRC_Tx;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	/*truyen tu bit trong so cao den thap-trai sang phai*/
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	/*Bit 9 SSM: Software slave management enable*/
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1, &SPI_InitStruct);
	
	SPI_Cmd(SPI1, ENABLE);
	
}

void SPI2_SentData(uint8_t u8data)
{
		/*CS = 0*/
		GPIOB->ODR &= ~(1<<12);
		/*sent data*/	
		/*wait TXE flag = 1- Transmit buffer empty*/
		/*false if (SPI2->SR & SPI_SR_TXE) == 1 tuc la bit thu 2 cua SR = 1*/
		while(!(SPI2->SR & SPI_SR_TXE));
		SPI2->DR = u8data;
		/*wait*/
		/*wait Receive buffer not empty */
		while(!(SPI2->SR & SPI_SR_RXNE));
		/*in communication*/
		while(SPI2->SR & SPI_SR_BSY);
		/*CS = 1*/
		GPIOB->ODR |= 1<<12;
}
void SPI1_SentData(uint8_t u8data)
{
		/*CS = 0*/
		GPIOA->ODR &= ~(1<<4);
		/*sent data*/	
		/*wait Transmit buffer empty*/
		/*false if (SPI2->SR & SPI_SR_TXE) == 1 tuc la bit thu 2 cua SR = 1*/
		while(!(SPI1->SR & SPI_SR_TXE));
		SPI1->DR = u8data;
		/*wait*/
		/*wait RXNE = 1- Receive buffer not empty - data tranfer is complete */
		while(!(SPI1->SR & SPI_SR_RXNE));
		/*in communication*/
		while(SPI1->SR & SPI_SR_BSY);
		/*CS = 1*/
		GPIOA->ODR |= 1<<4;
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