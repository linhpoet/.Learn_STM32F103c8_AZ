#include "Stm32_Lib_LCD.h""


void delay(uint32_t nCount){
	__IO uint32_t index = 0;

	/* default system clock is 168MHz */
	for(index = (500 * nCount); index != 0; index--);

}

void lcd_command(char cmnd)
{
    /*(gui lenh) gui 4 bit co trong so cao(b4-7) ma khong lam anh huong cac bit con lai trong Portdata*/
    PortData =(PortData & 0x0F) | (cmnd & 0xF0);            
    /*RW=0,*/
	CommandPort &= ~(1<<RW);
    /*RS=0, ghi lenh*/
	CommandPort &= ~(1<<RS);			

	/*generate falling edge*/
	CommandPort |= (1<<E);
	delay(1);
    CommandPort &= ~(1<<E);
	delay(200);
    
    /*(gui lenh) gui 4 bit co trong so thap*/
    PortData = (PortData & 0x0F) | (cmnd << 4);      
    
    /*generate falling edge*/
	CommandPort |= (1<<E);
	delay(1);
    CommandPort &= ~(1<<E);
	delay(3);

}
void lcd_char(char data)
{  
   /*(gui lenh) gui 4 bit co trong so cao(b4-7) ma khong lam anh huong cac bit con lai trong Portdata*/
   PortData =(PortData & 0x0F) | (data & 0xF0);
   /*RW=0*/
    CommandPort &= ~(1<<RW);		
    /*RS=1, ghi du lieu*/
	CommandPort |= (1<<RS);			

	/*generate falling edge */
	CommandPort |= (1<<E);
	delay(1);
    CommandPort &= ~(1<<E);
	delay(200);
    
    /*(gui lenh) gui 4 bit co trong so thap*/
    PortData = (PortData & 0x0F) | (data << 4);         
    
    /*generate falling edge*/
	CommandPort |= (1<<E);
	delay(1);
    CommandPort &= ~(1<<E);
	delay(3);
}
void lcd_string(char *str)
{
	for(int i=0; i<strlen(str); i++)
	{
		lcd_char(str[i]);
	}
}

void lcd_number(int num)
{   
    uint16_t u16Count = 0;
    uint32_t u32Num_temp = num;
    uint8_t u8Num_temp[10] = {0};

    if (u32Num_temp < 10)
    {
        u16Count = 1;
        u8Num_temp[0] = u32Num_temp;
    }
    else while (u32Num_temp > 0)
    {
        u8Num_temp[u16Count] = u32Num_temp % 10;
        u32Num_temp = (u32Num_temp - u8Num_temp[u16Count]) / 10;
        u16Count++;
    }
    for(int count=u16Count-1; count>=0; count--)
				lcd_char(u8Num_temp[count] + 48U);
}

