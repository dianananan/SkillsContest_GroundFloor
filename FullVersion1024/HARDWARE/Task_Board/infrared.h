#include "stm32f4xx.h"
#include "sys.h"

#ifndef __INFRARED_H
#define __INFRARED_H

#define RI_TXD PFout(11)  

void Infrared_Init(void);
void Infrared_Send(u8 *s,int n);

u8 HW_Send_Dispose(u8 );
u8 HW_Send_Choose(u8 choose_task);
void light_get(void);
void Acquire_rank(u8 num);//����
void light_Self_Tes(void); //·�Ƶĵ�λ���  //�������һ���ٽ��в���
void light_control(u8 Rank);  //���뵲λ

u8 getligthsum(void); //���ص�ǰ����
u8 HD_BCDSwitch(u16 object);    //BCDת��
u8 ASCLLSwitch(u16 object);  		//ASCLLת��
	
#endif




