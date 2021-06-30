#ifndef		__LCD_H
#define		__LCD_H

#include "stm32f10x.h"


/*------------- Define LCD Use -----------------*/

/*Note: Comment which not use */

#define LCD16xN //For lcd16x2 or lcd16x4
/* #define LCD20xN //For lcd20x4 */

/*------------- Define For Connection -----------------*/

#define RS_PORT		GPIOA
#define RS_CLOCK	RCC_APB2Periph_GPIOA
#define RS_PIN		GPIO_Pin_8

#define RW_PORT		GPIOA
#define RW_CLOCK	RCC_APB2Periph_GPIOA
#define RW_PIN		GPIO_Pin_9

#define EN_PORT		GPIOA
#define EN_CLOCK	RCC_APB2Periph_GPIOA
#define EN_PIN		GPIO_Pin_10

#define D7_PORT		GPIOA
#define D7_CLOCK	RCC_APB2Periph_GPIOA
#define D7_PIN		GPIO_Pin_7

#define D6_PORT		GPIOA
#define D6_CLOCK	RCC_APB2Periph_GPIOA
#define D6_PIN		GPIO_Pin_6

#define D5_PORT		GPIOA
#define D5_CLOCK	RCC_APB2Periph_GPIOA
#define D5_PIN		GPIO_Pin_5

#define D4_PORT		GPIOA
#define D4_CLOCK	RCC_APB2Periph_GPIOA
#define D4_PIN		GPIO_Pin_4


/*------------ Declaring Private Macro -----------------*/

#define PIN_LOW(PORT,PIN)	  GPIO_ResetBits(PORT,PIN);
#define PIN_HIGH(PORT,PIN)	GPIO_SetBits(PORT,PIN);

/*------------ Declaring Function Prototype -------------*/
void lcd_init(void);
void lcd_write(uint8_t type,uint8_t data);
void lcd_puts_String(uint8_t x, uint8_t y, char *string);
void lcd_puts_Number(uint8_t x, uint8_t y, uint32_t u32Number);
void lcd_clear(void);
#endif

