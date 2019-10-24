#ifndef __TASK_H
#define __TASK_H

#include "pathPlanning.h"

extern u8 CP_SHOW1[6];//s车牌显示
extern u8 CP_SHOW2[6];//立体显示显示车牌协议
extern u8 DZ_CPF[8];//ZigBee显示前 车牌
extern u8 DZ_CPB[8];//ZigBee显示后 车牌
void RunTaskDataControl(void);
u8 Task_Chooce(u8 taskchoose);
u16 getCsbDis(void);//超声波探测距离
typedef struct 
{
	union
	{
		u8 hw;
		u8 wifi;
		u8 zigbee;
		u8 carcode;
		u8 data;
	}task;
	u8 option;
	u16 TaskVaule;
}TaskOption;
typedef struct
{
		u16 distance;			//距离
		u8 LightTransmission;	//光档
		u8 carport;			//车库
		xyd TaskPot;	//由某个任务得到的坐标
		u8 Shape[3];	//图形信息
}DataBase;
extern DataBase dynamicInfo;	//运行时获得的动态信息

extern TaskOption isRunTask;
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
u8 Compute(u8 *array,u8 len,u8 object); //计数
/***********************************************************************/

enum
{
	TaskHW = 1,
	TaskWifi = 2 ,
	TaskZigbee = 3,
	TaskCar = 4
};


#endif
