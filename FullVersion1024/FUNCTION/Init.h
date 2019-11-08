#ifndef __INIT_H
#define __INIT_H

//���üĴ�������
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

void Send_USART_To_Fifo(u8 *array,u8 len);//���ʹ�����Ϣ
void RFID_Funition(void);	//RFID��������

void ExtendArray(u8 *array, u8 Len ,u8 adr,u8 cont);  //��չ��������
void bubble_sort(u8 *arr, u8 len,u8 mode);   //ð������  mode=1��С����  mode=2�Ӵ�С
//·�����ʱ������
u8 Set_Time_Content(u8 Timex,u16 arr,u16 psc);

void RepeatedlySend_ZG(u8 *array,u8 sum,u8 delay);
void RepeatedlySend_WI(u8 *array,u8 sum,u8 delay);
void RepeatedlySend_HW(u8 *array,u8 sum,u8 delay);
void PrintfDebug(u16 sum); //��ӡһ��ֵ
void TaskBoardTest(u8 mode); //��ӡ����ǿ�ȵ���Ϣ��debug���  mode=0 =1dis
//void Send_A72Data_To_Fifo(u8 *p ,u8 len);	//���ʹ�����Ϣ
void DelayTimerMS(u16 time);//���ø���ʱ����
u8 CheckSum(u8 *array,u8 orig); //����У���
void RevisalProtocol(u8 *array,u8 dis,u8 content);	//Э�����飬���ĵ�λ�ã����ĵ�����

#endif

