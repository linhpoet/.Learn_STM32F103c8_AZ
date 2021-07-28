#include "stm32f10x.h"
#include <string.h>
#include <stdint.h>

#define LCD_8bit

#define RS              0
#define RW              1
#define E               2
#define DataPort				GPIOA->ODR
#define CommandPort			GPIOB->ODR

#define Mode4bit2line   0x28
#define Mode8bit2line   0x38
#define DisplayOn       0x0c
#define ClearDisplay    0x01
#define Goto1           0x80
#define Goto2           0xc0

void lcd_command(char cmnd);
void lcd_char(char data);
void lcd_string(char *str);
void delay(uint32_t nCount);
void lcd_Number(uint32_t Num);
