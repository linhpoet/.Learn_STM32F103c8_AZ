#include "stm32f10x.h"

uint8_t Spi_TransmitAndReceive(uint8_t u8Data);
void Erase_Sector_ExternalFlash(uint32_t u32Address);
void Write_Page_ExternalFlash(uint32_t u32Address, uint8_t *pu8Data);
void Read_ExternalFlash(uint32_t u32Address, uint8_t *pu8Data, uint32_t len);
uint8_t ReadStatusRegisters1(void);
void Gpio_Init(void);
void Spi_2_Init(void);

uint8_t u8Buff[256U] = {0U};
uint8_t u8ReadBuff[256U] = {0U};

int main(void)
{
	uint32_t u32Count = 0U;
	
	for (u32Count = 0; u32Count < 256; u32Count++) 
	{
		u8Buff[u32Count] = u32Count;
	}
	
	Gpio_Init();
	Spi_2_Init();
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	/* Erase one sector of flash */
	Erase_Sector_ExternalFlash(0x00000000);
	/* Write to flash at address 0x00000000 with one page 256bytes*/
	Write_Page_ExternalFlash(0x00000000, u8Buff);
	/* Read to flash at address 0x00000000 with one page 256bytes*/
	Read_ExternalFlash(0x00, u8ReadBuff, 256U);
	/* Erase one sector of flash */
	Erase_Sector_ExternalFlash(0x00000000);
  /* Read to flash at address 0x00000000 with one page 256bytes*/
	Read_ExternalFlash(0x00, u8ReadBuff, 256U);
	while (1) 
	{
		

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
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
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

uint8_t Spi_TransmitAndReceive(uint8_t u8Data)
{
	/* Data received or to be transmitted. */
	SPI2->DR = u8Data;
	
	/* Waiting for 0: SPI (or I2S) not busy */
	while (((SPI2 ->SR) & SPI_I2S_FLAG_BSY) != 0)
	{
		
	}
	
	/* Data received or to be transmitted. */
	 return SPI2->DR;
}
uint8_t ReadStatusRegisters1(void)
{
	uint8_t u8ReadStatusRegisters1;
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	/* Read Status Register-1 (05h) and Read Status Register-2 (35h) */
	
	Spi_TransmitAndReceive(0x05);
	u8ReadStatusRegisters1 = Spi_TransmitAndReceive(0x00);

	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	return (u8ReadStatusRegisters1 & 0x1U);
}
void Read_ExternalFlash(uint32_t u32Address, uint8_t *pu8Data, uint32_t u32Length)
{
	uint32_t u32iter = 0U;
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	
	/* Read Data (03h) */
	Spi_TransmitAndReceive(0x03);
	Spi_TransmitAndReceive((u32Address >> 16));
	Spi_TransmitAndReceive((u32Address >> 8));
	Spi_TransmitAndReceive((u32Address));
	for (u32iter = 0; u32iter < u32Length; ++u32iter) 
	{
		pu8Data[u32iter] = Spi_TransmitAndReceive(0x00);
	}
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
void Write_Page_ExternalFlash(uint32_t u32Address, uint8_t *pu8Data)
{
	uint16_t u16Iter;
	
	/* Write Enable (06h) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	Spi_TransmitAndReceive(0x06);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	while(1 == ReadStatusRegisters1());
	
	/* Page Program (02h) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	Spi_TransmitAndReceive(0x02);
	Spi_TransmitAndReceive((u32Address >> 16));
	Spi_TransmitAndReceive((u32Address >> 8));
	Spi_TransmitAndReceive((u32Address));
	for (u16Iter = 0; u16Iter < 256; ++u16Iter) 
	{
		 Spi_TransmitAndReceive(pu8Data[u16Iter]);
	}
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	while(1 == ReadStatusRegisters1());
}
void Erase_Sector_ExternalFlash(uint32_t u32Address)
{
	/* Write Enable (06h) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	Spi_TransmitAndReceive(0x06);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	while(1 == ReadStatusRegisters1());
	
	/* Sector Erase (20h) */
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	Spi_TransmitAndReceive(0x20);
	Spi_TransmitAndReceive((u32Address >> 16));
	Spi_TransmitAndReceive((u32Address >> 8));
	Spi_TransmitAndReceive((u32Address));
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	while(1 == ReadStatusRegisters1());
}
