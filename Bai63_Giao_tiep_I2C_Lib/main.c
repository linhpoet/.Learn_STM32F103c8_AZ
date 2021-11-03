#include "stm32f10x.h"

uint8_t u8TxBuffer[8] = {1,2,3,4,5,6,7,8};

void I2C_Reg_Config();
void I2C_Lib_Config();
void I2C2_MasterSentData(uint8_t SlaveAddr, uint32_t *pTxBuffer, uint8_t TxBufferLength);
void I2C2_MasterReceiveData(uint8_t SlaveAddr, uint32_t *pRxBuffer, uint8_t RxBufferLength);
void I2C2_SlaveSentData(uint8_t SlaveAddr, uint32_t *pTxBuffer, uint8_t TxBufferLength);
void I2C2_SlaveReceiveData(uint8_t SlaveAddr, uint32_t *pRxBuffer, uint8_t RxBufferLength);
void Delay_ms(uint32_t u32Delay);

int main()
{
	I2C_Lib_Config();
	I2C2_MasterSentData(0x7A, u8TxBuffer, 8);
	
	while(1)
	{
		GPIOC->ODR |= 1<<13;
		Delay_ms(100);
		GPIOC->ODR &= ~(1<<13);
		Delay_ms(100);
	}
}

/*figure 273 RM*/
void I2C2_MasterSentData(uint8_t SlaveAddr, uint32_t *pTxBuffer, uint8_t TxBufferLength)
{
	/*1. gui tin hieu START*/
	I2C2->CR1 |= 1<<8;
	/*2. wait SB=1 - Start condition generated.*/
	while(!(I2C2->SR1 & 0x01));
	
	/*send slave Address and R/W bit - 7bit cao chua addr va bit 0 chua R/W*/
	SlaveAddr = SlaveAddr << 1;
		/*RW = 0*/
	SlaveAddr &= ~0x01;
	I2C2->DR = (uint8_t) SlaveAddr;

	/*EV6-wait ADDR = 1*/
	while( !(I2C2->SR1 & 0x02));
	/*clear ADDR flag by reading SR1followed by reading SR2*/
	uint32_t dummyRead = I2C2->SR1;
	dummyRead = I2C2->SR2;
	
	/*send data*/
	while(TxBufferLength)
	{
		/*EV8-wait TxE = 1*/
		while(!(I2C2->SR1 & 1<<7));
		/*clear TxE flag by writing to the DR register*/
		I2C2->DR = *pTxBuffer;
		pTxBuffer--;
		TxBufferLength--;
	}
	/*EV8_2-wait TxE = 1*/
	while(!(I2C2->SR1 & 1<<7));
	/*EV8_2-wait BTF=1*/
	while(!(I2C2->SR1 & 1<<2));
	/*gui tin hieu Stop - automatically clear BTF*/
	I2C2->CR1 |= 1<<9;	
}

void I2C2_MasterReceiveData(uint8_t SlaveAddr, uint32_t *pRxBuffer, uint8_t RxBufferLength)
{
	/*1.gui tin hieu START*/
	I2C2->CR1 |= 1<<8;
	/*2.confirm that start generation is completed by checking SB flag in SR1*/
	/*Note: Until SB is cleared SCL will be stretched (pull to LOW)*/
	while(!(I2C2->SR1 & 0x01));
	
	/*3.Send address of Slave*/
	SlaveAddr = SlaveAddr << 1;
		/*RW = 1*/
	SlaveAddr |= 0x01;
	I2C2->DR = (uint8_t) SlaveAddr;
	
	/*4. wait until address phase is completed - ADDR = 1*/
	while( !(I2C2->SR1 & 0x02));
	
	/*Read only 1 byte from slave*/
	if(RxBufferLength == 1)
	{
		/*Disable Acking*/
		I2C2->CR1 &= ~(1<<10);
		/*clear ADDR flag by reading SR1followed by reading SR2*/
		uint32_t dummyRead = I2C2->SR1;
		dummyRead = I2C2->SR2;		
		/*wait until RXNE = 1*/
		while(!((I2C2->SR1) & (1<<6)));
		/*generate STOP condition*/
		I2C2->CR1 |= 1<<9;	
		/*read data into buffer*/
		*pRxBuffer = I2C2->DR;
		return;
	}
	if(RxBufferLength>1)
	{
		/*clear ADDR flag by reading SR1followed by reading SR2*/
		uint32_t dummyRead = I2C2->SR1;
		dummyRead = I2C2->SR2;		
		/*read data until RxBufferLength = 0*/
		for(int i =RxBufferLength; i>0; i--)
		{
			/*wait until RxNE = 1*/
			while(!((I2C2->SR1) & (1<<6)));
			
			if(i==2)
			{
				/*khi RxNE = 1 o tren dung thi data ke cuoi da duoc nhan thanh cong*/
				/*viec xoa ACK bit phai thuc hien truoc khi co RxNE = 1(bao da nhan thanh cong data cuoi) */
				/*nen xoa o buoc nay, i=2 chu k phai i=1*/
				/*clear ACK bit*/
				I2C2->CR1 &= ~(1<<10);
				/*generate STOP condition*/
				I2C2->CR1 |= 1<<9;
			}
			/*clear RxNE-read data from DR into buffer*/
			*pRxBuffer = I2C2->DR;
			/*increment buffer address*/
			pRxBuffer++;
		}
	}
	/*re-enable ACK*/
	I2C2->CR1 |= 1<<10;
	
}

void I2C2_SlaveSentData(uint8_t MasterAddr, uint32_t *pTxBuffer, uint8_t TxBufferLength)
{
	
}

void I2C2_SlaveReceiveData(uint8_t MasterAddr, uint32_t *pRxBuffer, uint8_t RxBufferLength)
{
	
}


void I2C_Lib_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*GPIO Config - SDA vs SCL*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*I2C2 Config*/
	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0;
	I2C_Init(I2C2, &I2C_InitStruct);
	
	I2C_Cmd(I2C2, ENABLE);
		
}

void I2C_Reg_Config()
{
	/* I2C Peripheral under reset state*/
	I2C2->CR1 = 1<<15;
	/**/
	
	
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