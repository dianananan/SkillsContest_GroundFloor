#ifndef __ROADWAY_H
#define __ROADWAY_H

#include "stm32f4xx.h"

#define CARSWITCH  1
#if CARSWITCH   //老车
	#define LeftMP1 750
	#define RightMP1 950
	#define Left_45_MP 400
	#define Right_45_MP 400
	#define LSWERVEMP180 1800
	#define RSWERVEMP180 1800
#else  //新车1
	#define LeftMP1 45
	#define RightMP1 56
	#define Left_45_MP 30
	#define Right_45_MP 25
	#define LSWERVEMP180 109
	#define RSWERVEMP180 109
#endif // car==0


extern uint8_t L_Flag,R_Flag;
extern uint8_t G_Flag,B_Flag;
extern u8 Upright_Flag;
extern uint8_t wheel_Nav_Flag;
extern uint8_t Track_Flag;	 
extern int Car_Spend;
extern uint16_t temp_MP,MP,TraLen;  //行走长度
extern uint16_t temp_Nav;
extern uint8_t Stop_Flag;
extern uint32_t Wheel_flag;
extern u8 RodCnt;   //循迹路口数

extern uint8_t Line_Flag ;
extern uint16_t count;
extern int LSpeed,RSpeed;

extern u8 gd, gdg;
extern u8 intocorner;
extern u8 light_flagF,light_flagB;  //light led sum
extern u8 reflag;   //路口计数


void Track_Check(u16 tracklen, u8 roadSum, u8 state);
void wheel_Track_ANGLE(u8 angle,u16 max);
void wheel_Track_check(void);
void Go_and_Back_Check(void);

void Roadway_Check(void);
void Roadway_Flag_clean(void);
void Roadway_mp_syn(void);
void Roadway_nav_syn(void);
uint16_t Roadway_mp_Get(void);
u8 Countbits(u8 tstByte);

void Control(int L_Spend,int R_Spend);
void Track_Correct(u16 tracklen, u8 state);

extern uint8_t Roadway_GoBack_Check(void);
void roadway_check_TimInit(uint16_t arr,uint16_t psc);

#endif


