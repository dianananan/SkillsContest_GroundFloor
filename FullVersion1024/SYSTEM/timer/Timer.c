#include "stm32f4xx.h"
#include "Timer.h"
#include "roadway_check.h"
#include "activity.h"


volatile uint32_t global_times = 0;
volatile uint32_t delay_ms_const = 0;
volatile u32 WaitTimer_const;
volatile u32 WaitTimer_ms;


void Timer_Init(uint16_t arr,uint16_t psc) //定时器10
{
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	
	TIM_InitStructure.TIM_Period = arr;
	TIM_InitStructure.TIM_Prescaler = psc;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM10,&TIM_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM10, ENABLE);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update) == SET)
	{
			global_times++;
	}
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update);
}

u32 gt_get(void)
{
	return global_times;
}

u32 gt_get_sub(u32 c)
{
	if(c > global_times)
		c -= global_times;
	else
		c = 0;
	return c;
}


//void Timer12_Init(uint16_t arr,uint16_t psc) //定时器12
//{
//	TIM_TimeBaseInitTypeDef TIM_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
//	
//	TIM_InitStructure.TIM_Period = arr;
//	TIM_InitStructure.TIM_Prescaler = psc;
//	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //时钟分频
//	TIM_InitStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM10,&TIM_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);
//	TIM_Cmd(TIM12, ENABLE);
//}

//void TIM8_BRK_TIM12_IRQHandler(void) 
//{	
//	if(TIM_GetITStatus(TIM12,TIM_IT_Update) == SET)
//	{
//		if(WaitTimer_ms>0)
//			WaitTimer_ms--;
//	}
//	TIM_ClearITPendingBit(TIM12,TIM_IT_Update);
//}

u8 SetWaitTimer(u32 TimerSum)   //设置等待时间
{
	if(WaitTimer_ms != 0)
		return 0;
	else 
		WaitTimer_ms=TimerSum;
	return 1;
}

void EndWaitTim(void)
{
	WaitTimer_ms=0;
	WaitTimer_const=0;
}





