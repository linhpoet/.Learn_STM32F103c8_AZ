/*
Tham khao:http://laptrinharmst.blogspot.com/2018/04/bai-11-spi-voi-stm32f1.html
SPI1: master truyen du lieu sang SPI2:slave
sau khi SPI1 truyen thi SPI2->DR khac 0 => SPI2 RXNE flag = 1 => vao chuong trinh ngat
Trong chuong trinh ngat:	
		SPI_SLAVE_Buffer_Rx = SPI2->DR
		kiem tra gia tri truyen cuoi, khi 2 gia tri cuoi 2 buffer truyen nhan = nhau thi flag_end = 1 => set B9


*/

#include "stm32f10x.h"

#define BufferSize 20
// SPI1 : MASTER.
//SPI2	:	SLAVE

uint8_t SPI_MASTER_Buffer_Tx[BufferSize] = "le 0 quang linh";
uint8_t SPI_MASTER_Buffer_Rx[BufferSize];
																						
uint8_t SPI_SLAVE_Buffer_Tx[BufferSize];
uint8_t SPI_SLAVE_Buffer_Rx[BufferSize];
/**/
uint8_t M_Tx = 0, M_Rx = 0;
uint8_t S_Tx = 0, S_Rx = 0;
uint8_t Flag_end=0;
void SPI2_Slave_Config();
void SPI1_Master_Config();
void Delay_ms(uint32_t u32Delay);
void GPIO_Configuration();

/*vao ngat sau khi thuc hien send data => SPI_DR khac 0*/																						
void SPI2_IRQHandler()
{
	/*if RXNE == 1- RX buffer not empty*/
	/*Nghia la sau khi thuc hien send data- SPI2->DR khac 0 thi se vao ngat de luu gia tri ve SPI_SLAVE_Buffer_Rx*/
	if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
	{
		/*SPI_SLAVE_Buffer_Rx[S_Rx] = SPI2->DR*/
		SPI_SLAVE_Buffer_Rx[S_Rx] = SPI_I2S_ReceiveData(SPI2);
		
		if(SPI_SLAVE_Buffer_Rx[0] == SPI_MASTER_Buffer_Tx[0])
		{
			SPI_SLAVE_Buffer_Tx[S_Rx] = SPI_SLAVE_Buffer_Rx[S_Rx];
			S_Rx++;
			/*2 gia tri cuoi bang nhau thi nghia la truyen da xong*/
			if(SPI_SLAVE_Buffer_Rx[BufferSize-1] == SPI_MASTER_Buffer_Tx[BufferSize-1]) {Flag_end = 1;}
			
		}
		else {S_Rx=0; Flag_end = 0;}
	}
	/*clear RXNE flag*/
	SPI_I2S_ClearFlag(SPI2, SPI_I2S_IT_RXNE);
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
		/*wait Receive buffer not empty */
		while(!(SPI1->SR & SPI_SR_RXNE));
		/*in communication*/
		while(SPI1->SR & SPI_SR_BSY);
		/*CS = 1*/
		GPIOA->ODR |= 1<<4;
}

int main()
{
	uint8_t u8Buff[256] = {0U};

	for (int i=0;i < 256; i++)
	{
		u8Buff[i] = i;
	}
	GPIO_Configuration();
	SPI1_Master_Config();
	SPI2_Slave_Config();
	
  while (1)
  {
		{
			/*spi2_CR2 RXNEIE = 1 => enable RXNEIE interrupt*/
			/* generate an interrupt request when the RXNE flag is set (recieve buffer not empty).*/
			SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
			Delay_ms(10);
			GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction) (0));
			S_Rx = 0;
			M_Tx = 0;
			for(M_Tx=0;M_Tx<BufferSize;M_Tx++)
					{
						/*wait until TXE flag SPI1 is set - transmit buffer is empty*/
						while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}
							/*send data*/
						SPI_I2S_SendData(SPI1, SPI_MASTER_Buffer_Tx[M_Tx]);
							/*sau khi truyen thi SPI2_DR khac 0 nghia la RXNE flag = 1 =>vao chuong trinh ngat */
					}
					Delay_ms(1000);
		}
		/*sau khi truyen xong*/
		if(Flag_end==1)
		{
			Flag_end=0;
			/*disable SPI2 interrupt*/
			SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
			/*B9 is set*/
			GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction) (1));
			
		}
		Delay_ms(10000);
  }
}


void SPI2_Slave_Config()
{
	/*GPIO config*/
	GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//B14: MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	//B13:CLK		B15:MISO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	//B12:CS
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*SPI config*/	
	SPI_InitTypeDef SPI_InitStructure;
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;					
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;	
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		
		SPI_Init(SPI2, &SPI_InitStructure);			
		
		SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
		SPI_Cmd(SPI2, ENABLE);	
		
		NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure and enable SPI_SLAVE interrupt --------------------------------*/
				NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
}


void SPI1_Master_Config()
{
	/*GPIO Config*/
	GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA, ENABLE);
	//A5:CLK		A7:MOSI
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	//A6:MISO			
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	//A4:CS
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

void GPIO_Configuration(void)
{
	//RCC_PCLK2Config(RCC_HCLK_Div2); 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction) (0));
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

