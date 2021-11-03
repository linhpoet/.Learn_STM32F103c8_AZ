
#include <stdint.h>
main()
{
	uint32_t *pClockControl = (uint32_t*)(0x40023800 + 0x30);					//ahp1 peripheral clock enable reg
	uint32_t *pModeReg_GPIOD = (uint32_t*)(0x40020C00);
	uint32_t *pOutputReg_GPIOD = (uint32_t*)(0x40020C00 + 0x14);
	
	/*1. enable clock for GPIOD peripheral*/
	uint32_t temp = *pClockControl; 			//read gia tri trong thanh ghi enale clock
	temp |=0x08;
	*pClockControl = temp;
	
	/*2.config output mode */
	*pModeReg_GPIOD &=0xfcffffff;
	*pModeReg_GPIOD |= 0x01000000;
	
	/*3. set bit 12 as high*/
	*pOutputReg_GPIOD |= 1<<12;
	*pOutputReg_GPIOD |= 1<<13;
	while(1);
}