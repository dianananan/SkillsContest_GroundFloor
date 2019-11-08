#ifndef __INIT_H
#define __INIT_H

//放置寄存器操作
#include "stm32f4xx.h"
#include "delay.h"
#include "Timer.h"
#include "activity.h"
#include "data_base.h"
#include "PathPlanning.h"
#include "Task.h"
#include "tba.h"
#include "bh1750.h"
#include "infrared.h"
#include "rc522.h"
#include "syn7318.h"

#define TIMER_ONE	1
#define TIMER_TWO	2
#define TIMER_THREE	3
#define TIMER_FOUR	4
#define TIMER_FIVE	5
#define TIMER_SIX	6
#define TIMER_SEVEN 7

extern u8 Sdata[6];

void Send_USART_To_Fifo(u8 *array,u8 len);//发送串口信息
void RFID_Funition(void);	//RFID触发函数

void ExtendArray(u8 *array, u8 Len ,u8 adr,u8 cont);  //扩展数组内容
void bubble_sort(u8 *arr, u8 len,u8 mode);   //冒泡排序  mode=1从小到大  mode=2从大到小
//路况检测时间设置
u8 Set_Time_Content(u8 Timex,u16 arr,u16 psc);

void RepeatedlySend_ZG(u8 *array,u8 sum,u8 delay);
void RepeatedlySend_WI(u8 *array,u8 sum,u8 delay);
void RepeatedlySend_HW(u8 *array,u8 sum,u8 delay);
void PrintfDebug(u16 sum); //打印一个值
void TaskBoardTest(u8 mode); //打印关照强度的信息到debug面板  mode=0 =1dis
//void Send_A72Data_To_Fifo(u8 *p ,u8 len);	//发送串口信息
void DelayTimerMS(u16 time);//设置高延时函数
u8 CheckSum(u8 *array,u8 orig); //计算校验和
void RevisalProtocol(u8 *array,u8 dis,u8 content);	//协议数组，更改的位置，更改的内容

#endif

