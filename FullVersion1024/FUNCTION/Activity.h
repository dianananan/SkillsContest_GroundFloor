#ifndef __ACTIVITY_H
#define __ACTIVITY_H

#include "stm32f4xx.h"
#include "roadway_check.h"
#include "data_base.h"

#define ROW           3				//二维数组行
#define COLUMN		  100    		//二维数组列
#define PATHROW   	  0				//行0
#define PATHLENGTH    1				//行走数组长度

#define LEFT45		0x01				//左转45度	
#define LEFT 		0x02				//左转
#define LEFT180		0x03				//左转180度	
#define RIGHT45		0x04				//右转45度	
#define RIGHT 		0x05  				//右转	
#define RIGHT180	0x06				//右转180度								
#define GO			0x07				//前进											
#define BACK	    0x08				//后退	
#define CARSTOP     0x09				//停车										
#define CNTONE   	0x0A				//循迹一个十字路口    			
#define CNTTWO		0x0B  				//循迹两个十字路口				
#define CNTTHREE    0x0C				//循迹三个十字路口
#define CNTFOUR		0x0D				//循迹四个路口
#define CNTFIVE 	0x0E				//循迹五个路口
#define TRACKBLACK	0x0F 				//循迹一个路口黑线准确停
#define TRACKLENTH	0x10				//循迹距离						
#define MIDHALF		0x11				//循迹中等长度的一半	
#define GOCARBOYLEN	0x12				//走一个车身距离
#define LEFTSMALL	0x13				//左转前循迹的一小段距离												
#define RIGHTSMALL	0x14				//右转前循迹的一小段距离
#define TRAMP		0x15				//自定义循迹走码盘值	
#define STOPCAR		0x16				//小车停止
#define MIDDLE_LONGISH 0x17				//走到中间（较长的那边）
#define MIDDLE_SHORTE  0x18				//走到中间（较短的边）
#define GO_TERRAIN	0x19 				//过地形标志物
#define TRACK_LINE_SHORT 0x1A			//走一条边（短边）
#define TRACK_LINE_LANG  0x1B			//(走一条边) （长边）
#define DEBUG	0x1C					//debug



#define NOW_BARRIERSZ 0xf1			//道闸出现
#define NOW_ETC	0xf2				//ETC出现
#define NOW_TRAFFIC	0xf3			//交通灯出现
#define NOW_TASK  0xff    			//设置一个出现任务点
//***************************************************************************
#define TURNSPEED 	((80-Wheel_Speed_Cut)+Wheel_Speed_Up)	//转弯速度
#define CARSPEED   	((50-Walk_Speed_Cut)+Walk_Speed_Up)	//跑的速度
//***************************************************************************
#define MAXHALFLEN 	1150				//到达最长线一半的MP值
#define MIDHALFLEN	1400				//到达中等长度一半的MP值

#define BACKLEN		40					//倒退的MP值

#define CARBODYLEN	  600			//走一个车身距离
#define MP_LINE_SHORT 1150			//走一条边d的码盘值（短边）
#define MP_LINE_LANG  1400			//(走一条边的码盘值) （长边）

#define ZERO	0					//零
#define ROADONE	1					//循迹一个路口
#define ROADTWO	2					//循迹两个路口
#define ROADTHREE 3					//循迹三个路口
#define ROADFOUR 4	                //循迹四个路口
#define ROADFIVE 5	                //循迹五个路口

#define ROADMODE 1         //循迹路口模式
#define BLACKMODE 2         //循迹黑线模式
#define LENMODE  3			//循迹距离模式
#define TRACKTERRAIN 4		//走地形模式	

#define MAXPATH 30		//最大任务点个数

#define NAV45 1//转弯角度
#define NAV90 2
#define NAV180 3

//extern u8 intocorner,RodCnt; //路口判断标志，路口计数

extern u8 Walk_Speed_Cut;		//行走减速
extern u8 Walk_Speed_Up;		//行走加速
extern u8 Wheel_Speed_Cut;		//转弯减速
extern u8 Wheel_Speed_Up;		//转弯加速

typedef struct
{
    u8 NowPot;		//当前点
    u8 PathCount; //长度
	u16 mpvaule;		//行走的码盘值
    u8 PathGather[MAXPATH];
} RunContorl;
extern RunContorl isRunControl;

void Go_Test( u8 sp , u16 len);
void Back_Test(u8 sp , u16 len ); 
void Left_Test( u8 sp, u8 NAV) ;
void Right_Test(u8 sp,u8 NAV);  // 以80的速度右转弯
void Track_Test( u8 sp, u16 len, u8 cntrod, u8 state);  //循迹行走
void TrackingLamp_Test(u8 sp);
void STOP(void);   //停止

void runControl(void);
u8 Car_Run(u8 order);//行走函数
u8 getRunMpValue(int type, float typenum, float carbody, float linewidthnum); //mp计算函数  1 0 1 0
void trackLength(int tramp);
u8 getNowPathVaule(void);
void InitRunPathContorl(u8 *gather, u8 len); //初始化
void setMPVaule(u16 vaule);
u16 getMPVaule(void);
void Stop_Test(void); 

void runtimeInit(void);
void startAction(void);
void endAction(void);
u8 getActionState(void);
void startRun(void);
void endRun(void);
u8 getRunState(void);
void startTask(void);
void endTask(void);
u8 getTaskState(void);

#endif



