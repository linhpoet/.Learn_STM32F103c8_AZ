#include "Stm32_Lib_LCD.h"

void Delay_ms(uint32_t u32Delay);
void Timer2_Encoder_Register_Config();
void Timer2_Encoder_Lib_Config();
void GPIO_Lib_Config();
void PWM_Timer1_Multi_Channel_Register(int Duty_Ch1,int Duty_Ch2,int Duty_Ch3,int Duty_Ch4);
void PWM_Timer1_Multi_Channel_Lib(int Duty_Ch1,int Duty_Ch2,int Duty_Ch3,int Duty_Ch4);
void TIM23_ENCODER_Configuration(void);

int main()
{
	/*port for lcd*/
	RCC->APB2ENR |= 1<<3;
	RCC->APB2ENR |= 1<<4;
	RCC->APB2ENR |= 1<<2;
	
	volatile uint16_t u16Frequency = 1;
	GPIO_Lib_Config();
	PWM_Timer1_Multi_Channel_Lib(25,50,75,100);
	Timer2_Encoder_Register_Config();
	//Timer2_Counter_Register_Config();
	
	/*LCD config*/

	/*GPIOC[15:8] output Pushpull*/
	GPIOB->CRL = 0x11111111;
	/*dua con tro ve dau man hinh*/
    lcd_command(0x02);                          //mode 4 bit
    lcd_command(Mode4bit2line);					//4bit, 2 dong
	lcd_command(DisplayOn);						//bat hien thi
	lcd_command(ClearDisplay);					//clear
    
	while(1)
	{
		TIM2->CNT = 0;
		Delay_ms(1000);
		u16Frequency = TIM2->CNT;
		//u32Frequency++;
		lcd_command(ClearDisplay);
		lcd_Number(u16Frequency);
	}
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

void TIM23_ENCODER_Configuration(void)
{
	TIM_ICInitTypeDef			TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	/*Enable clock for timers*/
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_TIM2, ENABLE);

/* Configure Timer for Encoder Interface 2*/		
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_Period=0xFFFF;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	/*Configure Encoder Interface 1 capture channel*/
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICFilter=15;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	/*Configure Encoder Interface 2 capture channel*/
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICFilter=15;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	/*Configure Encoder Interface 1*/
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	/*Configure Encoder Interface 2*/
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	/*Reset Counter 1*/
	TIM_SetCounter(TIM3, 0);
	/*Reset Counter 2*/
	TIM_SetCounter(TIM2, 0);
	/*Enable 1nd Timer */
	TIM_Cmd(TIM3,ENABLE);	
	/*Enable 2nd Timer */
	TIM_Cmd(TIM2,ENABLE);
	
	/*Clear Interrupt Update flag 1*/
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);	
	/*Clear Interrupt Update flag 2*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	
	/*Enable Update Interrupt of 2ng TIMER*/
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	/*Enable Update Interrupt of 2ng TIMER*/
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void Timer2_Encoder_Lib_Config()
{
	TIM_ICInitTypeDef			TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	/*Enable clock for timers*/
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_TIM2, ENABLE);

/* Configure Timer for Encoder Interface 2*/		
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;
	TIM_TimeBaseInitStructure.TIM_Period=0xFFFF;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	/*Configure Encoder Interface 2 capture channel*/
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICFilter=15;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	/*Configure Encoder Interface 2*/
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	/*Reset Counter 2*/
	TIM_SetCounter(TIM2, 0);	
	/*Enable 2nd Timer */
	TIM_Cmd(TIM2,ENABLE);
	TIM2->CNT = 0;
		
	/*Clear Interrupt Update flag 2*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	
	/*Enable Update Interrupt of 2ng TIMER*/
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
}

void Timer2_Encoder_Register_Config()
{
	
	RCC->APB1ENR |= 0x01;
	
	TIM_TimeBaseInitTypeDef TimerInit;
	
	TimerInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerInit.TIM_Period = 0xffff;
	TimerInit.TIM_Prescaler = 1-1;
	
	/*gan cac gia tri o struct TimerInit cho cac thanh ghi tac dong timer 2*/
	TIM_TimeBaseInit(TIM2, &TimerInit);
	/*enable timer 2*/
	TIM_Cmd(TIM2, ENABLE);
	
	TIM2->CCMR1 = 0x01;
	
	TIM2->CCMR1 |= 3<<4;
	
	TIM2->CCER &= ~(0x01);
	/*Bits 3:2 IC1PSC: Input capture 1 prescaler  00: no prescaler, 
	capture is done each time an edge is detected on the capture input*/
	TIM2->CCMR1 &= ~(1<<2);
	TIM2->CCMR1 &= ~(1<<3);
	/* Capture enabled.*/
	TIM2->CCER |= 0x01;

}

void GPIO_Lib_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/*timer 2 encoder port a0,a1*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*Timer 1 PWM port a9,a8*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

void PWM_Timer1_Multi_Channel_Lib(int Duty_Ch1,int Duty_Ch2,int Duty_Ch3,int Duty_Ch4)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_TimeBaseInitTypeDef TimeBaseInitStructure;
	TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimeBaseInitStructure.TIM_ClockDivision = 0;
	TimeBaseInitStructure.TIM_Period = 10000-1;
	TimeBaseInitStructure.TIM_Prescaler = 72-1;
	TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = (Duty_Ch1*10000)/100;;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	/*pwm channel 1*/
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	
	/*pwm channel 2*/
	TIM_OCInitStructure.TIM_Pulse = (Duty_Ch2*10000)/100;;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM2);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);	
	
	/*tuong tu: pwm channel 3,4*/
	/*enable pwm_tim*/
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

