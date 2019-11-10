#ifndef __PATHPLANNING_H
#define __PATHPLANNING_H	 

#include <stdio.h>
#include "stm32f4xx.h"
#include "Activity.h"
#include "PathPlanning.h"

#define MAXPATH 30
#define MAXLIMIT 10

#define  getxy(vaule) ((vaule)&0x3f) //取出x，y
#define	 getDirection(vaule) ((vaule)>>6&3) //取出方向
#define	 getVauleX(vaule) ((vaule)&0x07)
#define  getVauleY(vaule) ((vaule)>>3&0x07)
#define  setVaule(x,y,dir) ((x)|(y<<3)|(dir<<6))
#define  setVauleDis(xy,dir)((dir<<6)|(xy))


#define LEFTDIR 0   //西（左）
#define FRONTDIR 1     //北（上）
#define RIGHTDIR 2      //东（右）
#define BACKDIR 3     //南（下）


typedef union
{
    u8 x,y,dir;
    u8 data;
}xyd;


struct LineNode
{
    xyd target;             //目标坐标点和任务方向
    xyd sexy[MAXLIMIT];     //任务必经点
    u8 TaskDir;     //任务方向 主要设置 45度 为了减少歧义 这里只能设置转45度 方向由前面的 xyd 控制
    u8 LastLineOption;  //最后一步的操作  是否走mp 前进还是后退
    u8 isback;
	u8 SpecialDispose;		//特殊处理点
};
extern struct LineNode TaskControl;

struct passivityGather
{
	 xyd TerrainPot;				//地形位置
	 xyd ETCPot;						//etc出现位置
	 xyd TrafficPot;				//交通灯出现位置
	 xyd BarrierszPot;				//道闸出现位置
	 xyd RFIDCard;						//RFID卡出现位置
	//没有字节对齐
};
extern struct passivityGather passivity;

extern xyd LimitPot[MAXLIMIT];     //任务必经点
extern u8 map[7][7];
extern u8 Carx,Cary,Cardirection;
extern u8 car_x,car_y,cardirection;	
extern short int rotate;//小车左转45
extern u8 NowTaskPot;		//当前的任务点
extern u8 ForwardMPFlag;	//是否转弯前一小段距离

void setMap(u8 x,u8 y,u8 s);
void initStartCoord(u8 x,u8 y,u8 dir);
void addCarLine(void);
u8 taskDirectionDeal(void);
void lastLine(void);
void setLimitPot(u8 sxy ,u8 loc);
u8 JudgeIntersection(int i ); 
void walkIntersection(void);
void dfs(u8 runxy,int n);
u8 getStep(u8 *x,u8 *y,u8 type);
u8 initTask(u8 target, u8 dir, u8 LastOption, u16 mpvaule, u8 runback, u8 Special);
u8 getMPvaule(void);
u8 getNowTask(void);
void setTaskLimitPot(void);
char getNewDirection(u8 type, u8 *current);
//void setLimitPot(u8 sxy,u8 exy);
void getCarPosition(u8 *currentdirection,u8 *x,u8 *y,u8 type);
void setTaskLimitGather(u8 *,u8); //设置多个限制点
void xydInit(xyd *data,u8 x ,u8 y,u8 dir);




#endif	
	
