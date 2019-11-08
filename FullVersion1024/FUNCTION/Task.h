#ifndef __TASK_H
#define __TASK_H

#include "pathPlanning.h"


//void RunTaskDataControl(void);	
u8 Task_Chooce(u8 taskchoose);	//任务选择
u16 getCsbDis(void);			//超声波探测距离

typedef struct 
{
	u8 TaskBegPoint;	//任务现在点 
	u8 TaskEndPoint;	//任务结束点
	u8 option[6];		//任务选择
	u8 TaskContent[6];	//内容
	u16 TaskVaule[6];	//任务值
}TaskOption;

typedef struct
{
		u16 distance;		//距离
		u8 LightLevelNow;	//现在的光源挡位
		u8 LightLevelTask;	//任务要求挡位
		u8 carport;			//车库
		xyd TaskPot;		//由某个任务得到的坐标
		u8 Shape[3];		//图形信息
}DataBase;

struct Mailbox	//邮箱
{
	u8 Graph_Sum_Shape[3];		//图形[0]颜色[1]形状[2]个数
	u8 PlateNumber[6];			//车牌	/0是默认车牌  1是读取到的车牌
	u8 TrafficLightA;			//交通灯
	xyd CoordinatePoint;		//返回点
	u8 OneNum;					//单纯数字
	DataBase ConfigInfo;		//运行时获得的动态信息
};//接收邮箱

extern struct Mailbox MailboxRe;	//接收邮箱
extern TaskOption CarRunTask;		//任务数组

void setNowTask(u8 NowTask,u8 cmd,u16 Vaule);
u8 getTaskCmd(void);
u8 getNowTask(void);
void TaskMenu(void);
void RunTaskControl(void);
u8 getNowLight(void);
void setNowlight(u8 );
u8 getNowGarage(void);

/************************************************************************/
void InitDataBase(void);//初始化数据（设置初值防止作为默认值）
u8 Compute(u8 *array,u8 len,u8 object); //object在数组出现的个数
/***********************************************************************/

enum
{
	TaskHW = 1,
	TaskWifi = 2 ,
	TaskZigbee = 3,
	TaskCar = 4
};


#endif
