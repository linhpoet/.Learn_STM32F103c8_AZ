#include "stm32f10x.h"

#define SPI2_DR_ADDRESS    ((uint32_t)0x4000380C)

void Gpio_Init(void);
void Spi_2_Init(void);
void SPI2_SentData(uint8_t u8data);
void DMA_ConfigChannel_1( uint32_t *pPeripheralAddr, uint32_t *pMemoryAddr, uint32_t u32NumberDataTransfer); 
void Delay_ms(uint32_t u32Delay);

uint8_t u8Buff[256U] = {0U};
uint8_t u8Data = 0x80;

int main(void)
{
	for (int i=0;i < 256; i++)
	{
		u8Buff[i] = i;
	}

	Gpio_Init();
	Spi_2_Init();
	
	uint16_t count = 0;

	/*Transmit without DMA*/
	for(int i=0; i<256; i++)
	{
		SPI2_SentData(u8Buff[i]);
	}
	Delay_ms(10);
	/*Transmit with DMA*/
	DMA_ConfigChannel_1(&(SPI2->DR), &u8Buff, 256);
		
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

void DMA_ConfigChannel_1( uint32_t *pPeripheralAddr, uint32_t *pMemoryAddr, uint32_t u32NumberDataTransfer)
{
	/*CS = 0*/
	GPIOB->ODR &= ~(1<<12);
	/*enable clock for DMA1*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*
	1.	Set the perifheral register address in the DMA_CCARx register
			The data will be moved from/to this address to/from the memory after the peripheral event
	*/
	DMA1_Channel5->CPAR = (uint32_t)pPeripheralAddr;
	/*
	2.	Set the memory address in the DMA_CMARx register. 
			The data will be written to or read from this memory after the peripheral event.
	*/
	DMA1_Channel5->CMAR = (uint32_t) pMemoryAddr;
	/*
	3.	Configure the total number of data to be transferred in the DMA_CNDTRx register.
			After each peripheral event, this value will be decremented.
	*/
	DMA1_Channel5->CNDTR = u32NumberDataTransfer;
	/*
	4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	5.	Configure data transfer direction, circular mode, peripheral & memory incremented
			mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
			DMA_CCRx register
			
			Bit14 Mem2Mem = 0			; ngoai vi SPI2 gui yeu cau ve DMA
			Bits 13:12 PL = 10		; channel priority level : high
			Bits 11:10 Msize = 00	; 00-memory size = 8 bit, so bit cua data bo nho
			Bits 9:8	Psize = 00	; 00-peripheral size = 8 bit, so bit cua data ngoai vi
			Bit 	7		MINC		= 1	; Memory increment mode enabled
			Bit 	6		PINC 		= 0	; Peripheral increment mode disable
			Bit 	5 	CIRC		= 0	;	bang 1 thi transfer lien tuc lap lai, neu =0 thi chi transfer 1 lan
			Bit 	4 	DIR			= 1	;	Read from memory- lay tu memomry roi truyen sang ngoai vi SPI
			Bit 	3 	TEIE		= 0	; TE interrupt enabled, cho ph?p ng?t khi c? l?i trong qu? tr?nh truy?n hay kh?ng.
			Bit		2		HTIE		= 0	;	HT interrupt disabled, cho ph?p ng?t khi truy?n xong data ? ch? d? half word.	
			Bit 	1 	TCIE		= 0 ; TC interrupt enabled, cho ph?p ng?t khi truy?n xong data ? ch? d? word.
			Bit 	0 	EN			=	1	;	channel is enable
	*/
	DMA1_Channel5->CCR |= 0b010000010010001;
	/*6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register*/
	DMA1_Channel5->CCR |= 0x01;
	/*Transmit when SPI send a request - Tx buffer DMA enabled*/
	SPI2->CR2 = 0x02;
	//SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
	
	/*wait DMA transmit completely*/ 
	while((DMA1->ISR & (1<<17)) == 0);
	//while (DMA_GetFlagStatus(DMA1_FLAG_TC5) == RESET);
	
	/*CS = 1*/
	GPIOB->ODR |= 1<<12;	
}

void SPI2_SentData(uint8_t u8data)
{
		/*CS = 0*/
		GPIOB->ODR &= ~(1<<12);
		/*sent data*/	
		/*wait Transmit buffer empty*/
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