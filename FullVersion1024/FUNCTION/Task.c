#include "task.h"
#include "stdio.h"
#include "string.h"
#include "activity.h"
#include "stm32f4xx.h"
#include "delay.h"
#include "roadway_check.h"
#include "canp_hostcom.h"
#include "PathPlanning.h"
#include "TaskWifiChannel.h"
#include "TaskZigbeeChannel.h"
#include "infrared.h"
#include "Init.h"
#include "tba.h"
#include "Init.h"
#include "ultrasonic.h"


extern u8 C_Tab[];
xyd TaskPot;
TaskOption CarRunTask;
DataBase dynamicInfo;
struct Mailbox MailboxRe; //收信邮箱

void RunTaskControl()//标志物数据收发控制
{
    if(getActionState() == 0 && (getNowPathVaule() == NOW_TASK) && getRunState() && (CarRunTask.TaskEndPoint != 0))
    {
        if(isrun.TaskCarryOut == 1)
        {
            setNowTask(TaskCar, DELAY, 500);
        }
		TaskMenu();
        if(((CarRunTask.TaskEndPoint) <= CarRunTask.TaskBegPoint) && (getTaskState() == 0) && (CarRunTask.TaskEndPoint != 0))	//如果任务完成则 +1	(getTaskState() == 0)     CarRunTask.TaskEndPoint!=0
		{
            ++isRunControl.NowPot;
//			Send_InfoData_To_Fifo((u8 *)"now pot is",sizeof("now pot is"));
//			PrintfDebug(isRunControl.NowPot);
			CarRunTask.TaskBegPoint = 0;
			CarRunTask.TaskEndPoint = 0;
        }
    }
//    else if(getActionState() == 0 && getRunState())
//    {
//        if(NOW_TRAFFIC == getNowPathVaule()) //交通灯
//        {
//            if(isrun.TaskCarryOut == 0)
//            {
//                STOP();
//                Send_InfoData_To_Fifo((u8 *)"Taskis\n", sizeof("Taskis\n"));

//                Wifi_Send_Dispose(CMD_LIGHT_READ);
//            }
//            if(getTaskState() == 0)		//如果任务完成则 +1
//            {
//                ++isRunControl.NowPot;
//            }
//            xydInit(&passivity.TrafficPot, 0, 0, 0); //解除识别交通灯
//        }
//        else if (NOW_ETC == getNowPathVaule())
//        {
//            if(isrun.TaskCarryOut == 0)
//            {
//                if(zig_send_etc_flag == 0)STOP();
//                Zig_Send_Dispose(CMD_ETC);
//            }
//            if(getTaskState() == 0)		//如果任务完成则 +1
//            {
//                ++isRunControl.NowPot;
//            }
//        }
//        else if (NOW_BARRIERSZ == getNowPathVaule())
//        {
//            if(isrun.TaskCarryOut == 0)
//            {
//                STOP();
//                Zig_Send_Dispose(CMD_ZG_DOOR);
//            }
//            if(getTaskState() == 0)		//如果任务完成则 +1
//            {
//                ++isRunControl.NowPot;
//            }
//        }
//    }
}

void TaskMenu()
{
    switch(getTaskCmd())
    {
		case TaskHW:
			HW_Send_Dispose(getNowTask());
			break;
		
		case TaskWifi:
			Wifi_Send_Dispose(getNowTask());
			break;
		
		case TaskZigbee:
			Zig_Send_Dispose(getNowTask());
			break;

		case TaskCar:
			Task_Chooce(getNowTask());
			break;
    }
}

u8 Task_Chooce(u8 taskchoose)
{
	u16 Temp=0;
    if(getTaskState() == 1)return 0;
	
    startTask();//确定为控制小车板子上的任务
    switch(taskchoose)
    {
		case DELAY://延时
			DelayTimerMS(CarRunTask.TaskVaule[CarRunTask.TaskBegPoint]);//延时1秒钟
			break;
		case LED_L_OPEN://打开左转向灯
			Set_tba_WheelLED(L_LED,1);
			break;
		case LED_L_CLOSE://关闭左转向灯
			Set_tba_WheelLED(L_LED,0);
			break;
		case LED_R_OPEN://打开右转向灯
			Set_tba_WheelLED(R_LED,1);
			break;
		case LED_R_CLOSE://打开右转向灯
			Set_tba_WheelLED(R_LED,0);
			break;
		case LED_OPEN://打开左右转向灯
			Set_tba_WheelLED(L_LED,1);
			Set_tba_WheelLED(R_LED,1);
			break;
		case LED_CLOSE://关闭左右转向灯
			Set_tba_WheelLED(L_LED,0);
			Set_tba_WheelLED(R_LED,0);
			break;
		case BEEP_OPEN://打开蜂鸣器 //600ms 一次
			Set_tba_Beep(1);
			DelayTimerMS(CarRunTask.TaskVaule[CarRunTask.TaskBegPoint]); //延时时间
			Set_tba_Beep(0);
			break;
		case BEEP_CLOSE://关闭蜂鸣器
			Set_tba_Beep(0);
			break;
		case CMD_GETPOINT: //超声波测距
			Temp=getCsbDis();
			if(Temp<3000 || Temp<100)//设置小车距离检测位
				MailboxRe.ConfigInfo.distance = Temp; //dynamicInfo.distance=Temp; 
			break;
		case UPRIGHT:  //矫正当前点
			delay_ms(50);
			for(Temp=0;Temp<2;Temp++)
			{
				ExtendArray(isRunControl.PathGather,isRunControl.PathCount ,isRunControl.NowPot,BACK); //  BACK   DEBUG
				++isRunControl.PathCount;
				ExtendArray(isRunControl.PathGather,isRunControl.PathCount ,isRunControl.NowPot,TRACKLENTH);  //TRACKLENTH
				++isRunControl.PathCount;
			}
			Send_WifiData_To_Fifo(isRunControl.PathGather,isRunControl.PathCount);delay_ms(50);
			break;
		default:
			return 0;
			break;
    }
    endTask();
    return 1;
}
#define CSBMAXTESTSUM 3   //最大的测试次数//用于超声波等最大20次

u16 getCsbDis()//超声波探测距离
{
	u16 CsbDistance=0;  //超声波距离
	u8 i=0;
	for(i=0;i<CSBMAXTESTSUM;i++)
	{
		Ultrasonic_Ranging();
		delay_ms(50);
		if(dis>3000 || dis<10)//太远或者太近//地图最远不会超过三米
			continue;
		CsbDistance+=dis;	
	}
	CsbDistance/=CSBMAXTESTSUM;
	return CsbDistance;
}

void setNowTask(u8 NowTask, u8 cmd, u16 Vaule)
{
    CarRunTask.option[CarRunTask.TaskEndPoint] = NowTask;
	CarRunTask.TaskContent[CarRunTask.TaskEndPoint] = cmd;
    CarRunTask.TaskVaule[CarRunTask.TaskEndPoint] = Vaule;
	++CarRunTask.TaskEndPoint;
}

u8 getTaskCmd(void)
{
    return CarRunTask.option[CarRunTask.TaskBegPoint];
}
u8 getNowTask(void)
{
    return CarRunTask.TaskContent[CarRunTask.TaskBegPoint];
}


void InitDataBase()  //默认值
{
    MailboxRe.ConfigInfo.distance = 220;			//超声波
	MailboxRe.ConfigInfo.LightLevelNow =1;		//光源当前位置
    MailboxRe.ConfigInfo.LightLevelTask = 3; 	//光源几档
    MailboxRe.ConfigInfo.carport = 3; 			//车库几层
	
	strcpy((char *)MailboxRe.PlateNumber, "ABCDEF");	//初始化车牌
	MailboxRe.Graph_Sum_Shape[0] =0x01;   		//初始化颜色 
	MailboxRe.Graph_Sum_Shape[1] =0X01; 		//初始化图形
	MailboxRe.Graph_Sum_Shape[2] =0x03; 		//初始化个数
	
	
	RFID_S50.RFID_Mode = SLEEP;	//设置不寻卡
	RFID_S50.RFID_Read_Ok=0;
	RFID_S50.RFID_Write_Ok=0;
	RFID_S50.Area =1;	
}


u8 Compute(u8 *array, u8 len, u8 object) //计数
{
    u8 i = 0, sum = 0;
    for(; i < len; i++)
    {
        if(array[i] == object)
            ++sum;
    }
    return sum;
}

u8 getNowLight(void)  //返回光源挡位
{
    u8 error[8] = {0xff, 0x03, 0x02, 0x02, 0x00, 0x00, 0x07, 0xf0}; //光源挡位错误信息
    if(MailboxRe.ConfigInfo.LightLevelTask <= 4 && MailboxRe.ConfigInfo.LightLevelTask != 0)
    {
        return MailboxRe.ConfigInfo.LightLevelTask ;
    }
    else //计算有错误，报错，并返回1
    {
        Send_WifiData_To_Fifo(error, 8);
        delay_ms(100);
        return 1;
    }
}


void setNowlight(u8 Vaule)
{
    dynamicInfo.LightLevelTask = Vaule;
}


u8 getNowGarage()//返回车库层数
{
    u8 error[8] = {0xff, 0x03, 0x02, 0x03, 0x00, 0x00, 0x08, 0xf0}; //车库信息错误
    if(MailboxRe.ConfigInfo.carport <= 4 && MailboxRe.ConfigInfo.carport != 0)
    {
        return MailboxRe.ConfigInfo.carport;		//对应数组的下标
    }
    else //计算有错误，报错，并返回1
    {
        Send_WifiData_To_Fifo(error, 8);
        delay_ms(100);
        return 1;
    }
}

