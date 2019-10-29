#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f4xx.h"


extern volatile uint32_t global_times;
extern volatile uint32_t delay_ms_const;
extern volatile u32 WaitTimer_const;  //定时器计数常数
extern volatile u32 WaitTimer_ms;	//时间ms
extern volatile u32 ECCTimer_ms;  //ETC

void Timer_Init(uint16_t arr,uint16_t psc);
//void Timer12_Init(uint16_t arr,uint16_t psc);
u32 gt_get_sub(u32 c);
u32 gt_get(void);
u8 SetWaitTimer(u32 TimerSum); //设置等待时间
//u8 GetWaitState(void);
//void OpenWaitTim(u32 TimerSum);
void EndWaitTim(void);

#endif


