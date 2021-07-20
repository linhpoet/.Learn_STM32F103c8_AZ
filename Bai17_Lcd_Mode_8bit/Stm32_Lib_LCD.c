#include "Stm32_Lib_LCD.h"


void delay(uint32_t nCount){
	__IO uint32_t index = 0;

	/* default system clock is 168MHz */
	for(index = (500 * nCount); index != 0; index--);

}


void lcd_command(char cmnd)
{
	CommandPort &= ~(1<<RW);			//RW=0,
	CommandPort &= ~(1<<RS);			//RS=0, ghi lenh

	DataPort = cmnd;																			//gui lenh (gui gia tri lenh vao thanh ghi cua port noi voi cac chan data (D0->D7)

	//		generate falling edge
	CommandPort |= (1<<E);
	delay(1);
	CommandPort &= ~(1<<E);
	delay(20);

}
void lcd_char(char data)
{
	/*RW=0*/
  CommandPort &= ~(1<<RW);		
  /*RS=1, ghi du lieu*/
	CommandPort |= (1<<RS);
	DataPort = data;																				//gui du lieu

	/*generate falling edge */
	CommandPort |= (1<<E);
	delay(1);
    CommandPort &= ~(1<<E);
	delay(200);
}
void lcd_string(char *str)
{
	for(uint32_t i=0; i<strlen(str); i++)
	{
		lcd_char(str[i]);
	}
}
void lcd_Number(uint32_t Num)
{
    uint16_t u16Count = 0;
    uint32_t u32Num_temp = Num;
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