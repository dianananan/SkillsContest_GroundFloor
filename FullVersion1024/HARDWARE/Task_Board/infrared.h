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
void Acquire_rank(u8 num);//跳档
void light_Self_Tes(void); //路灯的挡位检测  //将其调到一档再进行操作
void light_control(u8 Rank);  //输入挡位

u8 getligthsum(void); //返回当前几档
u8 HD_BCDSwitch(u16 object);    //BCD转码
u8 ASCLLSwitch(u16 object);  		//ASCLL转码
	
#endif




