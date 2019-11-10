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
struct Mailbox MailboxRe; //��������

void RunTaskControl()//��־�������շ�����
{
    if(getActionState() == 0 && (getNowPathVaule() == NOW_TASK) && getRunState() && (CarRunTask.TaskEndPoint != 0))
    {
        if(isrun.TaskCarryOut == 1)
        {
            setNowTask(TaskCar, DELAY, 500);
        }
		TaskMenu();
        if(((CarRunTask.TaskEndPoint) <= CarRunTask.TaskBegPoint) && (getTaskState() == 0) && (CarRunTask.TaskEndPoint != 0))	//������������ +1	(getTaskState() == 0)     CarRunTask.TaskEndPoint!=0
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
//        if(NOW_TRAFFIC == getNowPathVaule()) //��ͨ��
//        {
//            if(isrun.TaskCarryOut == 0)
//            {
//                STOP();
//                Send_InfoData_To_Fifo((u8 *)"Taskis\n", sizeof("Taskis\n"));

//                Wifi_Send_Dispose(CMD_LIGHT_READ);
//            }
//            if(getTaskState() == 0)		//������������ +1
//            {
//                ++isRunControl.NowPot;
//            }
//            xydInit(&passivity.TrafficPot, 0, 0, 0); //���ʶ��ͨ��
//        }
//        else if (NOW_ETC == getNowPathVaule())
//        {
//            if(isrun.TaskCarryOut == 0)
//            {
//                if(zig_send_etc_flag == 0)STOP();
//                Zig_Send_Dispose(CMD_ETC);
//            }
//            if(getTaskState() == 0)		//������������ +1
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
//            if(getTaskState() == 0)		//������������ +1
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
	
    startTask();//ȷ��Ϊ����С�������ϵ�����
    switch(taskchoose)
    {
		case DELAY://��ʱ
			DelayTimerMS(CarRunTask.TaskVaule[CarRunTask.TaskBegPoint]);//��ʱ1����
			break;
		case LED_L_OPEN://����ת���
			Set_tba_WheelLED(L_LED,1);
			break;
		case LED_L_CLOSE://�ر���ת���
			Set_tba_WheelLED(L_LED,0);
			break;
		case LED_R_OPEN://����ת���
			Set_tba_WheelLED(R_LED,1);
			break;
		case LED_R_CLOSE://����ת���
			Set_tba_WheelLED(R_LED,0);
			break;
		case LED_OPEN://������ת���
			Set_tba_WheelLED(L_LED,1);
			Set_tba_WheelLED(R_LED,1);
			break;
		case LED_CLOSE://�ر�����ת���
			Set_tba_WheelLED(L_LED,0);
			Set_tba_WheelLED(R_LED,0);
			break;
		case BEEP_OPEN://�򿪷����� //600ms һ��
			Set_tba_Beep(1);
			DelayTimerMS(CarRunTask.TaskVaule[CarRunTask.TaskBegPoint]); //��ʱʱ��
			Set_tba_Beep(0);
			break;
		case BEEP_CLOSE://�رշ�����
			Set_tba_Beep(0);
			break;
		case CMD_GETPOINT: //���������
			Temp=getCsbDis();
			if(Temp<3000 || Temp<100)//����С��������λ
				MailboxRe.ConfigInfo.distance = Temp; //dynamicInfo.distance=Temp; 
			break;
		case UPRIGHT:  //������ǰ��
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
#define CSBMAXTESTSUM 3   //���Ĳ��Դ���//���ڳ����������20��

u16 getCsbDis()//������̽�����
{
	u16 CsbDistance=0;  //����������
	u8 i=0;
	for(i=0;i<CSBMAXTESTSUM;i++)
	{
		Ultrasonic_Ranging();
		delay_ms(50);
		if(dis>3000 || dis<10)//̫Զ����̫��//��ͼ��Զ���ᳬ������
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


void InitDataBase()  //Ĭ��ֵ
{
    MailboxRe.ConfigInfo.distance = 220;			//������
	MailboxRe.ConfigInfo.LightLevelNow =1;		//��Դ��ǰλ��
    MailboxRe.ConfigInfo.LightLevelTask = 3; 	//��Դ����
    MailboxRe.ConfigInfo.carport = 3; 			//���⼸��
	
	strcpy((char *)MailboxRe.PlateNumber, "ABCDEF");	//��ʼ������
	MailboxRe.Graph_Sum_Shape[0] =0x01;   		//��ʼ����ɫ 
	MailboxRe.Graph_Sum_Shape[1] =0X01; 		//��ʼ��ͼ��
	MailboxRe.Graph_Sum_Shape[2] =0x03; 		//��ʼ������
	
	
	RFID_S50.RFID_Mode = SLEEP;	//���ò�Ѱ��
	RFID_S50.RFID_Read_Ok=0;
	RFID_S50.RFID_Write_Ok=0;
	RFID_S50.Area =1;	
}


u8 Compute(u8 *array, u8 len, u8 object) //����
{
    u8 i = 0, sum = 0;
    for(; i < len; i++)
    {
        if(array[i] == object)
            ++sum;
    }
    return sum;
}

u8 getNowLight(void)  //���ع�Դ��λ
{
    u8 error[8] = {0xff, 0x03, 0x02, 0x02, 0x00, 0x00, 0x07, 0xf0}; //��Դ��λ������Ϣ
    if(MailboxRe.ConfigInfo.LightLevelTask <= 4 && MailboxRe.ConfigInfo.LightLevelTask != 0)
    {
        return MailboxRe.ConfigInfo.LightLevelTask ;
    }
    else //�����д��󣬱���������1
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


u8 getNowGarage()//���س������
{
    u8 error[8] = {0xff, 0x03, 0x02, 0x03, 0x00, 0x00, 0x08, 0xf0}; //������Ϣ����
    if(MailboxRe.ConfigInfo.carport <= 4 && MailboxRe.ConfigInfo.carport != 0)
    {
        return MailboxRe.ConfigInfo.carport;		//��Ӧ������±�
    }
    else //�����д��󣬱���������1
    {
        Send_WifiData_To_Fifo(error, 8);
        delay_ms(100);
        return 1;
    }
}

