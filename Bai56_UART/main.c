#include "stm32f10x.h"

#define LengthOfRxBuffer	5

void USART_Lib_Config(USART_TypeDef* USARTx);			//8 bit data transmit, no parity
void USART_Reg_Config(USART_TypeDef* USARTx);			//9 bit data transmit, odd parity
void GPIO_Config();
void USART2_IRQHandler(void);
void Delay_ms(uint32_t u32Delay);
void My_USART_SendData(USART_TypeDef* pUSARTx, uint16_t u8Data);
uint8_t My_USART_ReceiveData(USART_TypeDef* pUSARTx);
void DMA_ConfigChannel_1( uint32_t *pPeripheralAddr, uint32_t *pMemoryAddr, uint32_t u32NumberDataTransfer);

uint8_t u8Buffer_Transmit[5] = "abcde123";
uint16_t u16Buffer_Transmit[5] = {5,6,7,8,9};
uint8_t u8Buffer_Receive[10] = {};
uint8_t Rx=0;

int main()
{
	GPIO_Config();
	USART_Lib_Config(USART2);

	while(1)
	{
		if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
		{
			u8Buffer_Receive[Rx] = My_USART_ReceiveData(USART2);
			Rx++;
		}
	}
		
	
	/*transmit with DMA*/
	//DMA_ConfigChannel_1(&(USART2->DR), u8Buffer_Transmit, 5);
//	while(1)
//	{
//		/*	Transmit and Receive to/from Laptop without interrupt	*/
////		config and enable interrupt in USART_Config
//		
//		/*	Transmit and Receive to/from Laptop without interrupt	*/
//		if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
//		{
//			
//			u8Buffer_Receive[Rx] = My_USART_ReceiveData(USART2);
//			//send to Laptop
//			My_USART_SendData(USART2, u8Buffer_Receive[Rx]);
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
//			//neu gui \n thi ket thuc chuoi du lieu
//			if(u8Buffer_Receive[Rx] == '\n') 
//			{
//				Rx=0;
//				//can xoa cac phan tu con lai cua mang truoc do
//			}
//			else Rx++;
//		}
		
		
//		for(int i=0; i<5; i++)
//		{
//			//Transmit
//			My_USART_SendData(USART2, u8Buffer_Transmit[i]);
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
//		}
		
		//Delay_ms(10);
}


void GPIO_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStruct);	

}

void USART_Lib_Config(USART_TypeDef* USARTx)
{
	/*gpio - PA2:TX		PA3:RX*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	
	/*uart 2 config*/
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1_5;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USARTx, &USART_InitStruct);
	
//	/*Interrupt Config*/
//	NVIC_InitTypeDef NVIC_InitStrut;
//	NVIC_InitStrut.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStrut.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStrut.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStrut.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStrut);
	
	
	/*usart interrupt config*/
	/*xoa co ngat nhan cho lan dau su dung */
	//USART_ClearFlag(USARTx, USART_IT_RXNE);
	/*enable RxNE interrupt*/
	//USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	
	/*enable USARTx*/
	USART_Cmd(USARTx, ENABLE);
}

void USART_Reg_Config(USART_TypeDef* pUSARTx)
{
	/*gpio - PA2:TX		PA3:RX*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*usart2 Reg Config*/
		RCC->APB1ENR |= 1<<17;
		/*baudrate = 9600*/
		pUSARTx->BRR = 0xEA6;
		/*Bit 12 M = 0: 1 Start bit, 9 Data bits, n Stop bit*/
		pUSARTx->CR1 |= (1<<12);
		/*Bit 10 PCE: Parity control enable*/
		pUSARTx->CR1 |= (1<<10);
		/*Odd Parity*/
		pUSARTx->CR1 |= 1<<9;
		/*Bit 3 TE: Transmitter enable*/
		pUSARTx->CR1 |= 1<<3;
		/*Bit 2 RE: Receiver enable*/
		pUSARTx->CR1 |= 1<<2;
		/*Bits 13:12 = 00: 1.5 STOP bits*/
		pUSARTx->CR2 |= (3<<12);
		/*00 - hardware flow control disabled*/
		pUSARTx->CR3 &= ~(3<<8);
		
		/*DMA enable transmitter*/
		pUSARTx->CR3 |= 1<<7;

		
			/*Core Interrupt Config*/
	NVIC_InitTypeDef NVIC_InitStrut;
	NVIC_InitStrut.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStrut.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStrut.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStrut.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStrut);
	
			/*Core Interrupt Config by using Register*/
	uint8_t IRQn;
	if(pUSARTx == USART1)
		IRQn = 37;
	else if(pUSARTx == USART2)
		IRQn = 38;
	else if(pUSARTx == USART3)
		IRQn = 39;
	else if(pUSARTx == UART4)
		IRQn = 52;
	else if(pUSARTx == UART5)
		IRQn = 53;
	NVIC->ISER[1] |= 1<<(IRQn-32);
	
		
	
	/*enable RxNE interrupt*/
	//USARTx->CR1 |= 1<<5;
	/*Bit 13 UE: USART enable*/
	pUSARTx->CR1 |= 1<<13;	
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		u8Buffer_Receive[Rx] = My_USART_ReceiveData(USART2);
		
		My_USART_SendData(USART2, u8Buffer_Receive[Rx]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		if(u8Buffer_Receive[Rx] == '\n') Rx=0;
		else Rx++;
	}
	USART_ClearFlag(USART2, USART_IT_RXNE);
}

void My_USART_SendData(USART_TypeDef* pUSARTx, uint16_t Data)
{
	/*send data*/
	pUSARTx->DR = (Data & (uint16_t)0x01FF);
	/*wait*/
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}


uint8_t My_USART_ReceiveData(USART_TypeDef* pUSARTx)
{	
	return pUSARTx->DR;
}

/*	3 gia tri truyen vao ham DMA la: 	dia chi (PeripheralAddr or MemoryAddr)
																			dia chi (PeripheralAddr or MemoryAddr)
																			size cua du lieu*/
void DMA_ConfigChannel_1( uint32_t *pPeripheralAddr, uint32_t *pMemoryAddr, uint32_t u32NumberDataTransfer)
{
	/*enable clock for DMA1*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/*
	1.	Set the perifheral register address in the DMA_CCARx register
			The data will be moved from/to this address to/from the memory after the peripheral event
	*/
	DMA1_Channel7->CPAR = (uint32_t)pPeripheralAddr;
	/*
	2.	Set the memory address in the DMA_CMARx register. 
			The data will be written to or read from this memory after the peripheral event.
	*/
	DMA1_Channel7->CMAR = (uint32_t) pMemoryAddr;
	/*
	3.	Configure the total number of data to be transferred in the DMA_CNDTRx register.
			After each peripheral event, this value will be decremented.
	*/
	DMA1_Channel7->CNDTR = u32NumberDataTransfer;
	/*
	4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	5.	Configure data transfer direction, circular mode, peripheral & memory incremented
			mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
			DMA_CCRx register
			
			Bit14 Mem2Mem = 0			; ngoai vi USART gui yeu cau ve DMA
			Bits 13:12 PL = 10		; channel priority level : high
			Bits 11:10 Msize = 00	; 00-memory size = 8 bit, so bit cua data bo nho
			Bits 9:8	Psize = 00	; 00-peripheral size = 8 bit, so bit cua data ngoai vi
			Bit 	7		MINC		= 1	; Memory increment mode enabled
			Bit 	6		PINC 		= 0	; Peripheral increment mode disable
			Bit 	5 	CIRC		= 1	;	bang 1 thi transfer lien tuc lap lai, neu =0 thi chi transfer 1 lan
			Bit 	4 	DIR			= 1	;	Read from memory
			Bit 	3 	TEIE		= 0	; TE interrupt enabled, cho ph?p ng?t khi c? l?i trong qu? tr?nh truy?n hay kh?ng.
			Bit		2		HTIE		= 0	;	HT interrupt disabled, cho ph?p ng?t khi truy?n xong data ? ch? d? half word.	
			Bit 	1 	TCIE		= 0 ; TC interrupt enabled, cho ph?p ng?t khi truy?n xong data ? ch? d? word.
			Bit 	0 	EN			=	1	;	channel is enable
	*/
	DMA1_Channel7->CCR = 0b010000010110000;
	
	/*clear TC bit*/
	USART2->SR &= ~(1<<6);
	/*6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register*/
	DMA1_Channel7->CCR |= 0x01;
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

