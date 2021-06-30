#include "lcd.h"

static void delay(uint32_t nCount);
static void LCD_InitGPIO(void);
/*--------------- Initialize LCD ------------------*/
void lcd_init(void)
{
	LCD_InitGPIO();
	delay(60);
	PIN_LOW(RW_PORT, RW_PIN);
	PIN_LOW(D4_PORT,D4_PIN);
	PIN_HIGH(D5_PORT,D5_PIN);
	PIN_LOW(D6_PORT,D6_PIN);
	PIN_LOW(D7_PORT,D7_PIN);
	PIN_LOW(RS_PORT,RS_PIN);
	
	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);
	
	lcd_write(0,0x28);
	lcd_write(0,0x0c);
	lcd_write(0,0x06);
	lcd_write(0,0x01);
}

/*--------------- Write To LCD ---------------*/
void lcd_write(uint8_t type,uint8_t data)
{
	delay(50);
	if(type)
	{
		PIN_HIGH(RS_PORT,RS_PIN);
	}else
	{
		PIN_LOW(RS_PORT,RS_PIN);
	}
	
	//Send High Nibble
	if(data&0x80)
	{
		PIN_HIGH(D7_PORT,D7_PIN);
	}else
	{
		PIN_LOW(D7_PORT,D7_PIN);
	}
	
	if(data&0x40)
	{
		PIN_HIGH(D6_PORT,D6_PIN);
	}else
	{
		PIN_LOW(D6_PORT,D6_PIN);
	}
	
	if(data&0x20)
	{
		PIN_HIGH(D5_PORT,D5_PIN);
	}else
	{
		PIN_LOW(D5_PORT,D5_PIN);
	}
	
	if(data&0x10)
	{
		PIN_HIGH(D4_PORT,D4_PIN);
	}else
	{
		PIN_LOW(D4_PORT,D4_PIN);
	}
	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);
	

	//Send Low Nibble
	if(data&0x08)
	{
		PIN_HIGH(D7_PORT,D7_PIN);
	}else
	{
		PIN_LOW(D7_PORT,D7_PIN);
	}
	
	if(data&0x04)
	{
		PIN_HIGH(D6_PORT,D6_PIN);
	}else
	{
		PIN_LOW(D6_PORT,D6_PIN);
	}
	
	if(data&0x02)
	{
		PIN_HIGH(D5_PORT,D5_PIN);
	}else
	{
		PIN_LOW(D5_PORT,D5_PIN);
	}
	
	if(data&0x01)
	{
		PIN_HIGH(D4_PORT,D4_PIN);
	}else
	{
		PIN_LOW(D4_PORT,D4_PIN);
	}
	PIN_HIGH(EN_PORT,EN_PIN);
	PIN_LOW(EN_PORT,EN_PIN);
}

void lcd_puts_String(uint8_t x, uint8_t y, char *string)
{
	//Set Cursor Position
	#ifdef LCD16xN	//For LCD16x2 or LCD16x4
	switch(x)
	{
		case 0: //Row 0
			lcd_write(0,0x80+0x00+y);
			break;
		case 1: //Row 1
			lcd_write(0,0x80+0x40+y);
			break;
		case 2: //Row 2
			lcd_write(0,0x80+0x10+y);
			break;
		case 3: //Row 3
			lcd_write(0,0x80+0x50+y);
			break;
	}
	#endif
	
	#ifdef LCD20xN	//For LCD20x4
	switch(x)
	{
		case 0: //Row 0
			lcd_write(0,0x80+0x00+y);
			break;
		case 1: //Row 1
			lcd_write(0,0x80+0x40+y);
			break;
		case 2: //Row 2
			lcd_write(0,0x80+0x14+y);
			break;
		case 3: //Row 3
			lcd_write(0,0x80+0x54+y);
			break;
	}
	#endif
	
	while(*string)
	{
		lcd_write(1,*string);
		string++;
	}
}
void lcd_puts_Number(uint8_t x, uint8_t y, uint32_t u32Number)
{
	uint32_t u32Count = 0U;
	uint32_t u3CounterOfNumber = 1U;
	volatile uint32_t u32TempNumber = 0;
	uint8_t u8TempNumber[10] = {0};
	
	//Set Cursor Position
	#ifdef LCD16xN	//For LCD16x2 or LCD16x4
	switch(x)
	{
		case 0: //Row 0
			lcd_write(0,0x80+0x00+y);
			break;
		case 1: //Row 1
			lcd_write(0,0x80+0x40+y);
			break;
		case 2: //Row 2
			lcd_write(0,0x80+0x10+y);
			break;
		case 3: //Row 3
			lcd_write(0,0x80+0x50+y);
			break;
	}
	#endif
	
	#ifdef LCD20xN	//For LCD20x4
	switch(x)
	{
		case 0: //Row 0
			lcd_write(0,0x80+0x00+y);
			break;
		case 1: //Row 1
			lcd_write(0,0x80+0x40+y);
			break;
		case 2: //Row 2
			lcd_write(0,0x80+0x14+y);
			break;
		case 3: //Row 3
			lcd_write(0,0x80+0x54+y);
			break;
	}
	#endif
	if(u32Number < 10U)
	{
		lcd_write(1,(u32Number + 48U));
	}
	else
	{
		 u32TempNumber = u32Number;
      while(u32Number >= 10)
	    {
				  u32TempNumber = u32Number%10U;
				  u8TempNumber[u32Count] = u32TempNumber;
				  u32TempNumber = u32TempNumber/10U;
          u32Number = u32Number/10U;
          u3CounterOfNumber ++;
				  u32Count ++;
      }
			u32TempNumber = u32Number%10U;
			u8TempNumber[u32Count] = u32TempNumber;
			while(u3CounterOfNumber --)
			{
				lcd_write(1,u8TempNumber[u3CounterOfNumber] + 48U);
				if (u3CounterOfNumber == 0)
				{
					 break;
				}
	    }
	}
  
}
void lcd_clear(void)
{
	lcd_write(0,0x01);
}
static void delay(uint32_t nCount){
	__IO uint32_t index = 0;

	/* default system clock is 168MHz */
	for(index = (500 * nCount); index != 0; index--);

}
static void LCD_InitGPIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
  	
  RCC_APB2PeriphClockCmd(RS_CLOCK | RW_CLOCK | EN_CLOCK | D7_CLOCK | D6_CLOCK | D5_CLOCK | D4_CLOCK, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = RS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RS_PORT, &GPIO_InitStructure);   
	
	GPIO_InitStructure.GPIO_Pin = RW_PIN;
	GPIO_Init(RW_PORT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = EN_PIN;
	GPIO_Init(EN_PORT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = D7_PIN;
	GPIO_Init(D7_PORT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = D6_PIN;
	GPIO_Init(D6_PORT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = D5_PIN;
	GPIO_Init(D5_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = D4_PIN;
	GPIO_Init(D4_PORT, &GPIO_InitStructure); 
}

