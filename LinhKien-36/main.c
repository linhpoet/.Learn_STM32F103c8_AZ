#include "stm32f10x.h"
#include <stdio.h>
#include "lcd.h"
#include "filter.h"
#include <stdlib.h>
#include <math.h>

#define LENGTH_BUFFER 8U
#define NUMBER_OF_ADC_CHANNEL 8U
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001244C)
#define FALSE 0U
#define TRUE 1U

typedef enum
{
	FLASH_ERRORS = 0U,    /* There is a error */
	FLASH_NO_ERRORS,      /* There is no errors */
	FLASH_PENDING,        /* Working is pending  */
	FLASH_ERRORS_TIMEOUT  /* There is a error due to timeout */
	} FlashStatus;

/**********************************************************************************************************************
*                                              Union
***********************************************************************************************************************/
typedef struct
{
	/* Register STK_CTRL */
	union
	{
			uint32_t STK_CTRL;
			struct
			{
				uint32_t ENABLE:1;
				uint32_t TICKINT:1;
				uint32_t CLKSOURCE:1;
				uint32_t unused_0:13;
				uint32_t COUNTFLAG:1;
				uint32_t unused_1:15;
			}B;
	}STK_CTRL_Register;
	/* Register STK_LOAD */
	union
	{
			uint32_t STK_CTRL;
			struct
			{
				uint32_t RELOAD:24;
				uint32_t unused_0:8;
			}B;
	}STK_LOAD_Register;
	/* Register STK_VAL */
	union
	{
			uint32_t STK_CTRL;
			struct
			{
				uint32_t CURRENT:24;
				uint32_t unused_0:8;
			}B;
	}STK_VAL_Register;
}SystemTick_Registers;

#define SYSTEM_TICK  ((SystemTick_Registers *)0xE000E010UL)

/**********************************************************************************************************************
*                                              ADC
***********************************************************************************************************************/
void DMA_ConfigChannel_11(uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTranfer);
void ADC_Config(void);
/**********************************************************************************************************************
*                                              GPIO blink C13
***********************************************************************************************************************/
void GPIO_ConfigPinC13(void);
/**********************************************************************************************************************
*                                              UART
***********************************************************************************************************************/
void UART_Config(uint8_t u8Uart);
/**********************************************************************************************************************
*                                              IWDG
***********************************************************************************************************************/
void IWDG_Setting(void);
/**********************************************************************************************************************
*                                              SysTick
***********************************************************************************************************************/
void SysTick_Interrupt(void);
/**********************************************************************************************************************
*                                              GPIO
***********************************************************************************************************************/
void GPIO_Config(void);
void Delay_SysTick_Ms(uint32_t u32Delay);
/**********************************************************************************************************************
*                                              FLASH
***********************************************************************************************************************/
FlashStatus Flash_Erase(volatile uint32_t u32StartAddr, uint32_t u32TimeOut);
FlashStatus Flash_Write_Syn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length);
FlashStatus Flash_Write_ASyn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length);
FlashStatus Flash_Read(volatile uint32_t u32StartAddr, uint8_t* u8BufferRead, uint32_t u32Length);
void Flash_Unlock(void);
void Start_BreakPoint(void);
void Stop_BreakPoint(void);
void EXTI0_Register(void);
void ShowLed7sSeg(uint8_t no, uint32_t u32Number);
void Display_7seg(uint32_t u32Number);
FlashStatus Debug(volatile uint32_t u32StartAddr, uint32_t u32Test);
uint32_t CRC_Caculate(volatile uint32_t *pStartAddr, uint32_t u32Length);
void Timer2_StandardLibrary(void);
void Timer2_Register(void);
void Delay_Timer2_Ms(uint32_t u32Delay);
void Timer2_Register_Interrupt(void);
void PWM_StandardLibrary(void);
void PWM_Timer2_Register_One_Channel(void);
void PWM_Timer2_Register_Muti_Channels(uint32_t SpeedDongco1, uint32_t SpeedDongco2,uint32_t SpeedDongco3, uint32_t SpeedDongco4);
void Delay(uint32_t nCount);
void Delay_Timer2_StandardLibrary_Ms(uint32_t u32Delay);
void EXTI0_StandardLibrary(void);
void RTC_Register(void);
void RTC_Config();
void ADC_StandardLibrary(void);
void Filter(void);
uint32_t CountString(char *pString);
void ReverseString(char *pString);
inline uint8_t CountNumber(uint32_t *pNumber);
void ReverseNumber(uint32_t *pNumber);
uint8_t CountNumberHex(uint32_t *pNumber);
void ReverseNumberHex(uint32_t *pNumber);
void ADC_Config_1Channel(void);
void ReverseNumberBin(uint32_t *pNumber);
	
#define PI 3.141592653589793f
volatile float _random,_sin,random_LPF_HPF,random_kalman,signal_LPF,signal_HPF,signal_kalman;
char aString[] = "LinhKien-36.com";
uint8_t LedCode7Seg[10]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};
uint32_t PwmDmd[2]={1U, 2U};
uint8_t aDataWrite[LENGTH_BUFFER] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
volatile uint16_t u16AdcValues[NUMBER_OF_ADC_CHANNEL];
volatile float fTimeMeasurement = 0.0; /* ms*/
_Bool bAsyn = FALSE;
FlashStatus eReturnCode = FLASH_NO_ERRORS;
uint32_t aCRC_Data_tranfer[3U] = {0x100, 0x200, 0x400};
uint32_t aCRC_Data_re[3U]      = {0x100, 0x200, 0x400};
uint32_t u32CrcCaclu1 = 0U;
uint32_t u32CrcCaclu2 = 0U;
uint32_t u32TimerInterrupt = 0U;
uint8_t u8Hours = 0U;
uint8_t u8Minutes = 0U;
uint8_t u8Seconds = 0U;
uint8_t u8Week = 2U;
uint8_t u8Hours_Alarm = 0U;
uint8_t u8Minutes_Alarm = 0U;
uint8_t u8Seconds_Alarm = 20U;
uint8_t u8Week_Alarm = 2U;
uint32_t u32Count = 1234567;
uint32_t u32Test = 0U;
uint16_t u16ValueAdc1Channel;

int main()
{
	uint8_t aDataBuffer[LENGTH_BUFFER] = {0};
	
	Flash_Erase((uint32_t)0x08001000, 100);
	if (FLASH_NO_ERRORS != Flash_Write_Syn((uint32_t)0x08001000, aDataWrite, LENGTH_BUFFER))
	{
		/* Errors occured*/
		while(1);
	}
	if (FLASH_NO_ERRORS != Flash_Read((uint32_t)0x08001000, aDataBuffer, LENGTH_BUFFER))
	{
		/*Errors occured*/
		while(1);
	}
	Flash_Erase((uint32_t)0x08001000, 100);
	
	return 0;	
}


void ReverseNumberBin(uint32_t *pNumber)
{
	uint8_t u8Count = 32U;
  uint32_t u32TempNumber = 0U;
	 
	while(u8Count > 0U)
	{
		u32TempNumber |= (((*pNumber >> (u8Count - 1U)) & 0x1U) << (32U - u8Count));
		u8Count--;
	}
	*pNumber = u32TempNumber;
}
void ADC_Config_1Channel(void)
{
    /* -- Enable clock for ADC1 and GPIOA -- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    /* ==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs== */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    ADC_InitTypeDef ADC_InitStructure;
    /* ADC1 configuration */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    /*We will convert multiple channels */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    /*select continuous conversion mode */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
    /*select no external triggering */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    /*right 12-bit data alignment in ADC data register */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    /*8 channels conversion */
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    /* load structure values to control and status registers */
    ADC_Init(ADC1, &ADC_InitStructure);
    /*configure each channel */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
    /*  Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* enable DMA for ADC */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
		/* Start ADC1 Software Conversion */
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		
		/*
     if (u16ValueAdc1Channel > 2100U)
		 {
			 Delay_SysTick_Ms(600000);
			 if (u16ValueAdc1Channel > 2100U)
		   {
			    Delay_SysTick_Ms(600000);
				  if (u16ValueAdc1Channel > 2100U)
		      {
					  Delay_SysTick_Ms(600000);
					  if (u16ValueAdc1Channel > 2100U)
		        {
			         GPIO_ResetBits(GPIOC, GPIO_Pin_13);
					  }
				  }
		    }
		 } 
		 if (u16ValueAdc1Channel <= 2100U)
		 {
			 Delay_SysTick_Ms(600000);
			 if (u16ValueAdc1Channel <= 2100U)
		   {
			    Delay_SysTick_Ms(600000);
				  if (u16ValueAdc1Channel <= 2100U)
		      {
					  Delay_SysTick_Ms(600000);
					  if (u16ValueAdc1Channel <= 2100U)
		        {
			         GPIO_SetBits(GPIOC, GPIO_Pin_13);
					  }
				  }
		    }
		 } */
		
}
uint8_t CountNumberHex(uint32_t *pNumber)
{
	uint8_t u8Count = 1U;
	uint8_t u8NumberHexCount = 9;
	uint32_t u32TempNumber = 0;
	
	do
	{
		u32TempNumber = (*pNumber >> (32U - u8Count*4))&0xFU;
		u8Count ++;
		u8NumberHexCount --;
	}while(u32TempNumber == 0U);
	
	return u8NumberHexCount;
}
void ReverseNumberHex(uint32_t *pNumber)
{
	uint8_t u8LengthNumber = 0U;
	uint8_t u8Count = 0U;
	uint32_t u32TempNumber = 0U;
  
	u8LengthNumber = CountNumberHex(pNumber);
	
	while(u8LengthNumber > 0U)
	{
		u32TempNumber |= (((*pNumber >> ((u8LengthNumber - 1)*4))&0xFU) << (u8Count*4U));
		u8LengthNumber --;
		u8Count++;
	}
	
	*pNumber = u32TempNumber;
}
inline uint8_t CountNumber(uint32_t *pNumber)
{
	uint8_t u8Count = 0U;
  uint32_t u32TempNumber = 0;
	
	u32TempNumber = *pNumber;
	
  if(u32TempNumber < 10U)
	{
		u8Count = 1U; 
	}
	else
	{
		  while(u32TempNumber >= 10)
	    {
          u32TempNumber = u32TempNumber/10U;
				  u8Count ++;
      }
	}
	u8Count ++;
	
	return u8Count;
}
void ReverseNumber(uint32_t *pNumber)
{
	uint8_t u8LengthNumber = 0U;
	uint8_t u8Count = 0U;
	uint8_t u8TempNumber[10U] = {0};
	uint32_t u32TempNumber = 0U;
	uint32_t u32Pow = 1;
	
	u8LengthNumber = CountNumber( pNumber);
	while(u8LengthNumber > 0U)
	{
		u32Pow = 1U;
		for (u8Count = 1; u8Count < u8LengthNumber; u8Count++)
		{
			u32Pow = u32Pow*10U;
		}
		u8TempNumber[u8LengthNumber] = *pNumber%10;
		u32TempNumber += u8TempNumber[u8LengthNumber] * u32Pow;
		*pNumber = *pNumber/10U;
		u8LengthNumber --;
	}
	
	*pNumber = u32TempNumber;
}
uint32_t CountString(char *pString)
{
	uint32_t u32CountNumber = 0U;
	
	while(*pString != NULL)
	{
		pString++;
		u32CountNumber++;
	}
	/* Add Null */
	u32CountNumber++;
	
	return u32CountNumber;
}
void ReverseString(char *pString)
{
	uint32_t left = 0U;
  uint32_t right = 0U;
  uint32_t temp = 0U;
	
	right =  CountString(pString) - 2U;
	
  while(left < right)
  {
       temp = pString[left];
       pString[left] = pString[right];
       pString[right] = temp;
       left++;
       right--;
   }
}
/*  Low pass filter - High pass filter - Kalman filter*/
void Filter(void)
{
  static float x=0;
  
  _sin = sin(x)*1000;
  x +=2*PI/1000;
  _random = (float)rand()/1000000;
  
  random_LPF_HPF = _random + _sin;
  signal_LPF = LPF(random_LPF_HPF,1,1000);
  signal_HPF = HPF(random_LPF_HPF,10,1000);
  
  random_kalman = (float)rand()/1000000;
  signal_kalman = kalman_single(random_kalman, 500, 10);
  
}
void RTC_Register(void)
{
/* - Config clock LSE */	
	/* Enable the power and backup interface clocks by setting the PWREN and BKPEN bits in the RCC_APB1ENR register */
	RCC->APB1ENR |= 0x18000000U;
	/* set the DBP bit the Power Control Register (PWR_CR) to enable access to the Backup registers and RTC */
	PWR->CR |= 0x100U;
	
	/* LSEON: External low-speed oscillator enable-> 32.768k */
	RCC->BDCR |= 0x01;
	while(((RCC->BDCR >> 1U) & 0x01U) != 1U);
	
	/* RTCSEL[1:0]: RTC clock source selection */
	/* 01: LSE oscillator clock used as RTC clock */
	RCC->BDCR |= 0x0100U;
  /* RTCEN: RTC clock enable */
  RCC->BDCR |= 0x8000;
/** - Config RTC */
  /* Poll RTOFF, wait until its value goes to ‘1' */	
	while (((RTC->CRL >> 5U) & 0x01U) != 1U);
	/* Set the CNF bit to enter configuration mode */
	RTC->CRL |= 0x10U;
	/* Write to one or more RTC registers */
	/* fTR_CLK = fRTCCLK/(PRL[19:0]+1)*/
	RTC->PRLH = 0U;
	RTC->PRLL = 0x7FFFU;
	/* Bit 0 SECIE: Second interrupt enable */
	RTC->CRH = 0x01U;
	/* clear interrupt flag */
	RTC->CRL &= ~0x01U;
	/* Clear the CNF bit to exit configuration mode */
  RTC->CRL &= ~0x10U;	
	/* Poll RTOFF, wait until its value goes to ‘1' */	
	while (((RTC->CRL >> 5U) & 0x01U) != 1U);
	
	/* Enable interrupt in Core */
	NVIC->ISER[0] = 0x08;
}
void RTC_IRQHandler()
{
	/*Check interrupt flag and enable interrupt bit*/
	if(((RTC->CRL & 0x01) == 1U) && ((RTC->CRH & 0x01) == 1U))
	{
    RTC->CRL &= ~0x01U;
		u8Seconds ++;
		if(u8Seconds == 60U)
		{
			u8Seconds = 0U;
			u8Minutes++;
			if(u8Minutes == 60U)
			{
				u8Minutes = 0U;
				u8Hours ++;
				if(u8Hours == 24U)
				{
					u8Hours = 0U;
					u8Week++;
				 	if(u8Week == 9)
					{
						u8Week = 2U;
					}
				}
			}
		}
	}
}

void EXTI0_StandardLibrary(void)
{
	  EXTI_InitTypeDef 					EXTI_InitStructure; 
    NVIC_InitTypeDef 					NVIC_InitStructure;
	
		/* Enable clock for AFIOEN*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		/* External interrupt configuration register 1 (AFIO_EXTICR1) */
	  /* 0001: PB[x] pin */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
		/* Clear the the EXTI line interrupt pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		/*EXTI line Configuration */
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		/*NVIC Configuration*/
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		Delay_SysTick_Ms(500);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}
   
}
void PWM_Timer2_Register_Muti_Channels(uint32_t SpeedDongco1, uint32_t SpeedDongco2,uint32_t SpeedDongco3, uint32_t SpeedDongco4)
{
	/* Enable clock for timer2*/
	RCC->APB1ENR |= 0x01;
	/* IMx auto-reload register (TIMx_ARR) */
	TIM2->ARR = 10000 - 1;
	/* PSC[15:0]: Prescaler value */
	TIM2->PSC = 7200 - 1;
	/* 110: PWM mode 1*/
	TIM2->CCMR1 = 0x6060;
	/* 110: PWM mode 1*/
	TIM2->CCMR2 = 0x6060;
	
/** TIM2_CH1 -> PA0*/		
	/* TIMx capture/compare register 1 (TIMx_CCR1) */
	TIM2->CCR1 = (SpeedDongco1*10000)/100;

/** TIM2_CH2 -> PA1*/		
	/* TIMx capture/compare register 1 (TIMx_CCR2) */
	TIM2->CCR2 = (SpeedDongco2*10000)/100;

/** TIM2_CH3 -> PA2*/		
	/* TIMx capture/compare register 1 (TIMx_CCR3) */
	TIM2->CCR3 = (SpeedDongco3*10000)/100;

/** TIM2_CH4 -> PA3*/		
	/* TIMx capture/compare register 1 (TIMx_CCR4) */
	
	TIM2->CCR4 = (SpeedDongco4*10000)/100;
	/* 
	   - CC1E, CC2E, CC3E, CC4E: Capture/Compare 1, 2, 3, 4 output enable 
	   - CC1P, CC2P, CC3P, CC4P: 0: OC1 active high or 1 : low  
	*/
	TIM2->CCER = 0x1111;
	/* Bit 0 CEN: Counter enable and  0: Counter used as upcounter */
	TIM2->CR1 = 0x01;
	/* Generate an update event to reload the Prescaler and the Repetition counter values immediately */
	TIM2->EGR = 0x01;
}
void PWM_Timer2_Register_One_Channel(void)
{
	/* Enable clock for timer2*/
	RCC->APB1ENR |= 0x01;
	/* IMx auto-reload register (TIMx_ARR) */
	TIM2->ARR = 10 - 1;
	/* PSC[15:0]: Prescaler value */
	TIM2->PSC = 7200 - 1;
	/* 110: PWM mode 1*/
	TIM2->CCMR1 = 0x60;
	/* TIMx capture/compare register 1 (TIMx_CCR1) */
	TIM2->CCR1 = (50*10)/100;
	/* 
	   - CC1E: Capture/Compare 1 output enable 
	   - CC1P: 0: OC1 active high or 1 : low  
	*/
	TIM2->CCER = 0x01;
	/* Bit 0 CEN: Counter enable and  0: Counter used as upcounter */
	TIM2->CR1 = 0x01;
	/* Generate an update event to reload the Prescaler and the Repetition counter values immediately */
	TIM2->EGR = 0x01;
}
void PWM_StandardLibrary(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef timerInit;
	TIM_OCInitTypeDef pwmInit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	timerInit.TIM_CounterMode = TIM_CounterMode_Up;
	timerInit.TIM_Period = 10000 - 1;
	timerInit.TIM_Prescaler = 7200 - 1;
	
	TIM_TimeBaseInit(TIM2, &timerInit);
	
	TIM_Cmd(TIM2, ENABLE);
	
	/* Ch1 with duty 10 % */
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = (10*10000)/100;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM2, &pwmInit);
	
	/* Ch2 with duty 25 % */
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = (25*10000)/100;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC2Init(TIM2, &pwmInit);
	
	/* Ch3 with duty 40 % */
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = (40*10000)/100;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM2, &pwmInit);

	/* Ch1 with duty 80 % */
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = (80*10000)/100;
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM2, &pwmInit);
}
void Timer2_Register_Interrupt(void)
{
	/* Enable clock for timer2*/
	RCC->APB1ENR |= 0x01;
	/* IMx auto-reload register (TIMx_ARR) */
	TIM2->ARR = 10000 - 1;
	/* Counter  = 0*/
	TIM2->CNT = 0U;
	/* PSC[15:0]: Prescaler value */
	TIM2->PSC = 7200 - 1;
	/* Bit 0 UIE: Update interrupt enable */
	TIM2->DIER = 0x01;
	/* Clear interrupt flag */
	TIM2->SR &= ~(0x01);
	/* Bit 0 CEN: Counter enable and  0: Counter used as upcounter*/
	TIM2->CR1 = 0x01;
	/* Generate an update event to reload the Prescaler and the Repetition counter values immediately */
	TIM2->EGR = 0x01;
	/* 6 settable EXTI0 Line0 interrupt*/
	NVIC->ISER[0] = 0x10000000;
	
	NVIC->IP[28] = 0x4U << 4U;
}
void TIM2_IRQHandler(void)
{
	if (((TIM2->SR & 0x01U) != 0U) && (((TIM2->DIER)&0x01U) == 0x01U))
	{
		TIM2->SR &= ~(0x01U);
		u32TimerInterrupt++;
		if(u32TimerInterrupt%2U == 0)
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
		}
		else
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		}
	}
}
void Delay_Timer2_Ms(uint32_t u32Delay)
{
	while(u32Delay)
	{
		TIM2->CNT = 0U;
		while ((TIM2->CNT) < 1000);
		u32Delay --;
	}	
}
void Timer2_Register(void)
{
	/* Enable clock for timer2*/
	RCC->APB1ENR |= 0x01;
	/* IMx auto-reload register (TIMx_ARR) */
	TIM2->ARR = 200-1;
	/* PSC[15:0]: Prescaler value */
	TIM2->PSC = ((SystemCoreClock/2)/2000000)-1;
	/* Bit 0 CEN: Counter enable and  0: Counter used as upcounter*/
	TIM2->CR1 = 0x01;
	/* Generate an update event to reload the Prescaler and the Repetition counter values immediately */
	TIM2->EGR = 0x01;
}
void Delay_Timer2_StandardLibrary_Ms(uint32_t u32Delay)
{
	while(u32Delay)
	{
		TIM_SetCounter(TIM2, 0U);
		while ((TIM_GetCounter(TIM2)) < 1000U);
		u32Delay --;
	}	
}
void Timer2_StandardLibrary(void)
{
	TIM_TimeBaseInitTypeDef TimerInit;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TimerInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerInit.TIM_Period = 0xFFFF;
	TimerInit.TIM_Prescaler = 72 - 1;
	
	TIM_TimeBaseInit(TIM2, &TimerInit);
	
	TIM_Cmd(TIM2, ENABLE);
}
uint32_t CRC_Caculate(volatile uint32_t *pStartAddr, uint32_t u32Length)
{
	uint32_t u32Count = 0U;
	
	/* Enable clock for crc*/
	RCC->AHBENR |= 0x40; 
	/* Resets the CRC calculation unit and sets the data register to 0xFFFF FFFF */
	CRC->CR = 0x01;
	for(u32Count = 0U; u32Count < u32Length; u32Count++)
	{
	   /* Used as an input register when writing new data into the CRC calculator. Holds the previous CRC calculation result when it is read*/
	   CRC->DR = *pStartAddr;
		 pStartAddr ++;
	}
	return (CRC->DR);
}
FlashStatus Debug(volatile uint32_t u32StartAddr, uint32_t u32Test)
{
	 if(u32StartAddr == 0)
	 {
	     eReturnCode = FLASH_ERRORS;
 	 }
	 if((u32Test % 2) == 0)
	 {
		  eReturnCode = FLASH_ERRORS;
	 }
	 
	 return eReturnCode;
}

void ShowLed7sSeg(uint8_t no, uint32_t u32Number)
{
	uint32_t u32TempNumber = 0U;
	
	/* A0,A1,A2,A3,A4,A5,A6,A7 
	Number 1 :  b = c = 0, remain =  1: 0xF9  (0x11111001)
	*/
	u32TempNumber = LedCode7Seg[u32Number];
	/***************a*****************/	
	if((u32TempNumber&0x01) == 0x01)		   		 
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_0);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_0);
	}		 
	u32TempNumber=u32TempNumber>>1;
	/***************b*****************/	   
	if((u32TempNumber&0x01) == 0x01)		   		 
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_1);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_1);
	}		 
	u32TempNumber=u32TempNumber>>1;
	/***************c*****************/	 
  if((u32TempNumber&0x01) == 0x01)		   		
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_2);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_2);
	}		 
	u32TempNumber=u32TempNumber>>1;
	/***************d*****************/		
  if((u32TempNumber&0x01) == 0x01)		   		
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_3);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_3);
	}		 
	u32TempNumber=u32TempNumber>>1;		 
/***************e*****************/		
  if((u32TempNumber&0x01) == 0x01)		   		
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_4);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_4);
	}		 
	u32TempNumber=u32TempNumber>>1;		
/***************f*****************/		
  if((u32TempNumber&0x01) == 0x01)		   		
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_5);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_5);
	}		 
	u32TempNumber=u32TempNumber>>1;		
/***************g*****************/		
  if((u32TempNumber&0x01) == 0x01)		   		
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_6);   
	}
  else
	{
	   GPIO_ResetBits(GPIOA ,GPIO_Pin_6);
	}	
	if (no == 1)
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_7);
	}
	else if (no == 2)
	{
		GPIO_SetBits(GPIOA ,GPIO_Pin_8);
	}
	else if (no == 3)
	{
	  GPIO_SetBits(GPIOA ,GPIO_Pin_9);
	}
	else if (no == 4)
	{
	  GPIO_SetBits(GPIOA ,GPIO_Pin_10);
	}
	else
	{
		GPIO_ResetBits(GPIOA ,GPIO_Pin_7);
	  GPIO_ResetBits(GPIOA ,GPIO_Pin_8);
	  GPIO_ResetBits(GPIOA ,GPIO_Pin_9);
	  GPIO_ResetBits(GPIOA ,GPIO_Pin_10);
	}
	Delay_SysTick_Ms(7);
}
void Display_7seg(uint32_t u32Number)
{
	uint32_t u32Digit;

	GPIO_ResetBits(GPIOA ,GPIO_Pin_7);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_10);
	
	u32Digit = u32Number / 1000;
	ShowLed7sSeg(4,u32Digit);
	
	GPIO_ResetBits(GPIOA ,GPIO_Pin_7);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_10);
	
	u32Number = u32Number - u32Digit * 1000;
	u32Digit = u32Number / 100;
	ShowLed7sSeg(3,u32Digit);
	
	GPIO_ResetBits(GPIOA ,GPIO_Pin_7);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_10);
	
	u32Number = u32Number - u32Digit * 100;
	u32Digit = u32Number / 10;
	ShowLed7sSeg(2,u32Digit);
	
	GPIO_ResetBits(GPIOA ,GPIO_Pin_7);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_8);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA ,GPIO_Pin_10);
	
	u32Number = u32Number - u32Digit * 10;
	u32Digit = u32Number;
	ShowLed7sSeg(1,u32Digit);
	
}
void EXTI0_Register(void)
{
/** Enable pin */	
	/* Enable clock for AFIOEN*/
	RCC->APB2ENR |= 0x01;  
  /* External interrupt configuration register 1 (AFIO_EXTICR1) */
	/* 0001: PB[x] pin */
  AFIO->EXTICR[0] |= 0x01;
/** Setup to enable interrupt*/	
	/* Pending register (EXTI_PR) */
	EXTI->PR = 0x01;
  /* Rising trigger selection register (EXTI_RTSR) */	
  EXTI->RTSR = 0x00;        
  /* Falling trigger selection register (EXTI_FTSR) */	
  EXTI->FTSR |= 0x01;    
  /* Software interrupt event register (EXTI_SWIER) */	
	EXTI->SWIER |= 0x00;
	/* Event mask register (EXTI_EMR) */
	EXTI->EMR = 0x00;
	/* Interrupt mask register (EXTI_IMR) */
	EXTI->IMR |= 0x01;
/** Enable IRQ Channels */	
	/* 6 settable EXTI0 Line0 interrupt*/
	NVIC->ISER[0] = 0x40;
	
}
void Start_BreakPoint(void)
{
   SYSTEM_TICK->STK_LOAD_Register.STK_CTRL = 72U * 1000U - 1U;
	 SYSTEM_TICK->STK_VAL_Register.STK_CTRL = 0U;
	 SYSTEM_TICK->STK_CTRL_Register.B.CLKSOURCE = 1U;
	 SYSTEM_TICK->STK_CTRL_Register.B.TICKINT = 1U;
   SYSTEM_TICK->STK_CTRL_Register.B.ENABLE = 1U;

}

void Stop_BreakPoint(void)
{
  fTimeMeasurement = fTimeMeasurement + (71999U - (SysTick->VAL))/71999.0;
	SYSTEM_TICK->STK_CTRL_Register.B.ENABLE = 0U;
}

void DMA_ConfigChannel_11(uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTranfer)
{
	/* Enable clock for DMA1 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /*
	* Set the peripheral register address in the DMA_CPARx register. 
	* The data will be moved from/ to this address to/ from the memory after the peripheral event
	*/
	DMA1_Channel1->CPAR = (uint32_t)pStartAddress;
	/*
	* Set the memory address in the DMA_CMARx register. The data will be written to or
  * read from this memory after the peripheral event
	*/
	DMA1_Channel1->CMAR = (uint32_t)pDestination;
	/* 
   *Configure the total number of data to be transferred in the DMA_CNDTRx register.
  * After each peripheral event, this value will be decremented
	*/
	DMA1_Channel1->CNDTR = u32NumberDataTranfer;
	/*
	 * Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	*/
	/*
	Configure data transfer direction, circular mode, peripheral & memory incremented
  mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
  DMA_CCRx register.
	*/
	DMA1_Channel1->CCR |= 0x25A0;
	/* Activate the channel by setting the ENABLE bit in the DMA_CCRx register. */
	DMA1_Channel1->CCR |= 0x01;
	
}
void ADC_Config(void)
{
    /* -- Enable clock for ADC1 and GPIOA -- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    /* ==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs== */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    ADC_InitTypeDef ADC_InitStructure;
    /* ADC1 configuration */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    /*We will convert multiple channels */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    /*select continuous conversion mode */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
    /*select no external triggering */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    /*right 12-bit data alignment in ADC data register */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    /*8 channels conversion */
    ADC_InitStructure.ADC_NbrOfChannel = 8;
    /* load structure values to control and status registers */
    ADC_Init(ADC1, &ADC_InitStructure);
    /*configure each channel */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5);
    /*  Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* enable DMA for ADC */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
		/* Start ADC1 Software Conversion */
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void GPIO_ConfigPinC13(void)
{
	/* Or  0b10000 --> Anabling Preiph GPIOC */
	RCC->APB2ENR |= 0x10; 
	/* Reset the PORT C PIN 13 */
	GPIOC->CRH &= 0xFF0FFFFF;  
	/* 11: Output mode, max speed 50 MHz. */
	GPIOC->CRH |= 0x00300000; 
}
/**********************************************************************************************************************
*                                              UART2
***********************************************************************************************************************/
void UART_Config(uint8_t u8Uart)
{
	if (u8Uart == 2U)
	{
		/* Enable UART2 */
		RCC->APB1ENR |=0x20000;
		/* Setup the baude rate for 9600 bps */
		USART2->BRR = 0xEA6; 
		/* Enable Uart Transmit */
		USART2->CR1 |= 8;
		/* Enable Uart Recive */
		USART2->CR1 |= 4;
		/* Enable Uart */
		USART2->CR1 |= 0x2000;
	}
}
/**********************************************************************************************************************
*                                              IWDG
***********************************************************************************************************************/
/*
*    u32RestInXmsec: Timeout which will reset MCU in u32RestInXmsec ms
*/
void IWDG_Setting(void)
{
	/* Write access to the IWDG_PR and IWDG_RLR registers is protected. To modify them, first write the code 0x5555 in the IWDG_KR register.*/
	IWDG->KR = 0x5555U; 
	/* 000: divider /4 */
	IWDG->PR = 0x00;
	/* Watchdog counter reload value */
	IWDG->RLR = 0xFFF;
	/* Reload WDG */
	IWDG->KR  = 0xAAAA;
  /* 
	* When the independent watchdog is started by writing the value 0xCCCC in the Key register (IWDG_KR), the counter starts counting down from the reset value of 0xFFF. 
 	* When it reaches the end of count value (0x000) a reset signal is generated (IWDG reset).
	*/
	IWDG->KR = 0xCCCCU;
}
/**********************************************************************************************************************
*                                              GPIO
***********************************************************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
  	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);   
	/* Configure B0-Trigger in output pushpull mode */
	
	/* Configure B1-Echo in input mode */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
  	
}
void Delay(uint32_t nCount) 
{
  while(nCount--)
  {
  }
}
void SysTick_Handler(void)
{
	fTimeMeasurement = fTimeMeasurement + 1.0;
}
void SysTick_Interrupt(void)
{
	/* Dealy 1ms*/
		SYSTEM_TICK->STK_LOAD_Register.STK_CTRL = 72U * 1000U - 1U;
		SYSTEM_TICK->STK_VAL_Register.STK_CTRL = 0U;
		SYSTEM_TICK->STK_CTRL_Register.B.CLKSOURCE = 1U;
	  SYSTEM_TICK->STK_CTRL_Register.B.TICKINT = 1U;
  	SYSTEM_TICK->STK_CTRL_Register.B.ENABLE = 1U;
}

void Delay_SysTick_Ms(uint32_t u32Delay)
{
	
	while(u32Delay) 
	{
		/* Dealy 1ms*/
		SysTick->LOAD = 72U * 1000U - 1U;
		SysTick->VAL = 0U;
		SysTick->CTRL = 5U;
		while (!(SysTick->CTRL & (1U << 16U)))
		{
			/* Wating for Returns 1 if timer counted to 0 since last time this was read*/
		}
		--u32Delay;
	}
}

/**********************************************************************************************************************
*                                              FLASH
***********************************************************************************************************************/
void Flash_Unlock(void)
{
	/* This sequence consists of two write cycles, where two key values (KEY1 and KEY2) are written to the FLASH_KEYR address*/
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

FlashStatus Flash_Erase(volatile uint32_t u32StartAddr, uint32_t u32TimeOut)
{
	/* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
	while(((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY) && (u32TimeOut > 0U))
	{
		/*  Wating for Bsy bit */
		u32TimeOut --;
		if (u32TimeOut == 0)
		{
			return FLASH_ERRORS_TIMEOUT;
		}
	}
	/* Check unlock sequences */
	if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
	{
		Flash_Unlock();
	}
	/* Set the PER bit in the FLASH_CR register */
  FLASH->CR |= FLASH_CR_PER;
	/* Program the FLASH_AR register to select a page to erase */
  FLASH->AR = u32StartAddr;
	/* Set the STRT bit in the FLASH_CR register */
  FLASH->CR |= FLASH_CR_STRT;
	/* Wait for the BSY bit to be reset */
  while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*  Wating for Bsy bit */
	}
	/* Clear PER bit in the FLASH_CR register */
  FLASH->CR &= ~FLASH_CR_PER; 
	/* Clear STRT bit in the FLASH_CR register */
	FLASH->CR &= ~FLASH_CR_STRT;
	
	return FLASH_NO_ERRORS;
}

FlashStatus Flash_Write_Syn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length)
{
	uint32_t u32Count = 0U;
	
	/* Check input paras */
	if((u8BufferWrite == 0x00U) || (u32Length == 0U) || u32Length%2U != 0U)
	{
		return FLASH_ERRORS;
	}
   /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
	while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*  Wating for Bsy bit */
	}
	/* Check unlock sequences */
	if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
	{
		Flash_Unlock();
	}
	/* Write FLASH_CR_PG to 1 */
	FLASH->CR |= FLASH_CR_PG;
  /* Perform half-word write at the desired address*/
	for(u32Count = 0U; u32Count < (u32Length/2); u32Count ++ )
	{
		*(uint16_t*)(u32StartAddr + u32Count*2U) = *(uint16_t*)(u8BufferWrite + u32Count*2U);
	}
	/* Wait for the BSY bit to be reset */
  while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	{
		/*  Wating for Bsy bit */
	}
	/* Clear PG bit in the FLASH_CR register */
	FLASH->CR &= ~FLASH_CR_PG;
	
	return FLASH_NO_ERRORS;
}
FlashStatus Flash_Write_ASyn(volatile uint32_t u32StartAddr, uint8_t* u8BufferWrite, uint32_t u32Length)
{
	uint32_t u32Count = 0U;
	FlashStatus ReturnCode;
	if( FALSE == bAsyn)
	{
	    /* Check input paras */
	    if((u8BufferWrite == 0x00U) || (u32Length == 0U) || u32Length%2U != 0U)
	    {
	    	return FLASH_ERRORS;
	    }
       /* Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register */
	    while((FLASH->SR&FLASH_SR_BSY) == FLASH_SR_BSY)
	    {
	    	/*  Wating for Bsy bit */
	    }
	    /* Check unlock sequences */
	    if ((FLASH->CR&FLASH_CR_LOCK) == FLASH_CR_LOCK )
	    {
	    	Flash_Unlock();
	    }
	    /* Write FLASH_CR_PG to 1 */
	    FLASH->CR |= FLASH_CR_PG;
      /* Perform half-word write at the desired address*/
	    for(u32Count = 0U; u32Count < (u32Length/2); u32Count ++ )
	    {
	    	*(uint16_t*)(u32StartAddr + u32Count*2U) = *(uint16_t*)(u8BufferWrite + u32Count*2U);
	    }
			bAsyn = TRUE;
			ReturnCode = FLASH_PENDING;
	}
	else
	{
	    /* Wait for the BSY bit to be reset */
      if((FLASH->SR&FLASH_SR_BSY) != FLASH_SR_BSY)
	    {
	    	 /* Clear PG bit in the FLASH_CR register */
	       FLASH->CR &= ~FLASH_CR_PG;
				 bAsyn = FALSE;
				 ReturnCode = FLASH_NO_ERRORS;
	    }
	}
	
	return ReturnCode;
}
FlashStatus Flash_Read(volatile uint32_t u32StartAddr, uint8_t* u8BufferRead, uint32_t u32Length)
{
	
	/* Check input paras */
	if((u8BufferRead == 0x00U) || (u32Length == 0U))
	{
		return FLASH_ERRORS;
	}
	do
	{
	   if(( u32StartAddr%4U == 0U) && ((uint32_t)u8BufferRead%4U == 0U) && (u32Length >= 4U))
		 {
		   *(uint32_t*)(u8BufferRead) = *(uint32_t*)(u32StartAddr);
		   u8BufferRead = u8BufferRead + 4U;
		 	 u32StartAddr = u32StartAddr + 4U;
			 u32Length = u32Length - 4U;
		 }
		 else if(( u32StartAddr%2U == 0U) && ((uint32_t)u8BufferRead%2U == 0U) && (u32Length >= 2U))
		 {
		    *(uint16_t*)(u8BufferRead) = *(uint16_t*)(u32StartAddr);
		 	  u8BufferRead = u8BufferRead + 2U;
		 	  u32StartAddr = u32StartAddr + 2U;
			  u32Length = u32Length - 2U;
		 }
		 else
		 {
		    *(uint8_t*)(u8BufferRead) = *(uint8_t*)(u32StartAddr);
		 	  u8BufferRead = u8BufferRead + 1U;
		 	  u32StartAddr = u32StartAddr + 1U;
			  u32Length = u32Length - 1U;
		 }
	}
	while(u32Length > 0U);
	
	return FLASH_NO_ERRORS;
}

void RTC_Config()
{
	/*BKPEN = 1- enable backup interface*/
	RCC->APB1ENR |= 1<<27;
	/*PWREN = 1- enable power interface*/
	RCC->APB1ENR |= 1<<28;
	/*DPB = 1- access rtc and backup register enable*/
	PWR->CR |= 1<<8;
	
	/*LSEON = 1- external 32khz osc on (lse on)*/
	RCC->BDCR |=0x01;
	/*LSEDRY = 1: External 32 kHz oscillator ready, so wait until LSEDRY != 0*/
	while (((RCC->BDCR >> 1) & 0x01) != 1);
	/*	while ((RCC->BDCR & 1<<1) != 1); 2cau lenh tuong tu nhau */
	
	/*RTC clock source selection*/
	/*RTCSEL = 01- : LSE oscillator clock used as RTC clock*/
	RCC->BDCR |= 1<<8;
	RCC->BDCR &= ~(1<<9);
	
	/*RTC enable*/
	RCC->BDCR |= 1<<15;
	
	/*1. Poll RTOFF, wait until its value goes to ‘1’*/
	while (((RTC->CRL >> 5) & 0x01) != 1);
	/*2. Set the CNF bit to enter configuration mode*/
	RTC->CRL |= 1<<4;
	/*3. Write to one or more RTC registers*/
	/*fTR_CLK = fRTCCLK/(PRL[19:0]+1)*/
	RTC->PRLL = 0x7FFF;
	RTC->PRLH = 0x00;
	
	/*4. Clear the CNF bit to exit configuration mode*/
	RTC->CRL &= ~(1<<4);
	/*5. Poll RTOFF, wait until its value goes to ‘1’ to check the end of the write operation.*/
	while (((RTC->CRL >> 5) & 0x01) != 1);
}